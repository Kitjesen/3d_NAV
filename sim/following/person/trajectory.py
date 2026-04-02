"""Trajectory generators for the simulated person.

All trajectories implement the PersonTrajectory protocol.
Each produces a PersonState per step with position, velocity, and heading.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Tuple

import numpy as np

from sim.following.interfaces import PersonState


def _lerp_angle(a: float, b: float, t: float) -> float:
    """Lerp between two angles handling wraparound."""
    diff = (b - a + math.pi) % (2 * math.pi) - math.pi
    return a + diff * t


# ---------------------------------------------------------------------------
# Waypoint walker utility (shared by all trajectory types)
# ---------------------------------------------------------------------------

class _WaypointWalker:
    """Walks through a sequence of (x, y) waypoints at constant speed."""

    def __init__(
        self,
        waypoints: List[Tuple[float, float]],
        speed: float = 0.8,
        loop: bool = False,
        z: float = 0.0,
    ):
        self._waypoints = [(float(x), float(y)) for x, y in waypoints]
        self._speed = speed
        self._loop = loop
        self._z = z
        self._idx = 0
        self._pos = np.array([*self._waypoints[0], z], dtype=np.float64)
        self._heading = 0.0
        self._done = False
        self._time = 0.0

    def reset(self) -> PersonState:
        self._idx = 0
        self._pos = np.array([*self._waypoints[0], self._z], dtype=np.float64)
        self._heading = 0.0
        self._done = False
        self._time = 0.0
        if len(self._waypoints) > 1:
            dx = self._waypoints[1][0] - self._waypoints[0][0]
            dy = self._waypoints[1][1] - self._waypoints[0][1]
            self._heading = math.atan2(dy, dx)
        return self._state(np.zeros(3))

    def step(self, dt: float) -> PersonState:
        if self._done:
            return self._state(np.zeros(3))

        self._time += dt
        target = self._waypoints[self._idx + 1] if self._idx + 1 < len(self._waypoints) else None

        if target is None:
            self._done = True
            return self._state(np.zeros(3))

        dx = target[0] - self._pos[0]
        dy = target[1] - self._pos[1]
        dist = math.hypot(dx, dy)

        if dist < 0.05:
            self._idx += 1
            if self._idx + 1 >= len(self._waypoints):
                if self._loop:
                    self._idx = 0
                else:
                    self._done = True
                    return self._state(np.zeros(3))
            return self.step(0.0)  # re-target next waypoint immediately

        # Move toward target
        target_heading = math.atan2(dy, dx)
        self._heading = _lerp_angle(self._heading, target_heading, min(1.0, 5.0 * dt))

        move_dist = min(self._speed * dt, dist)
        vx = (dx / dist) * self._speed
        vy = (dy / dist) * self._speed
        self._pos[0] += (dx / dist) * move_dist
        self._pos[1] += (dy / dist) * move_dist

        return self._state(np.array([vx, vy, 0.0]))

    @property
    def is_complete(self) -> bool:
        return self._done

    def _state(self, vel: np.ndarray) -> PersonState:
        return PersonState(
            position=self._pos.copy(),
            velocity=vel,
            heading=self._heading,
            visible=True,
            timestamp=self._time,
        )


# ---------------------------------------------------------------------------
# Concrete trajectory types
# ---------------------------------------------------------------------------

class StraightWalk:
    """Walk in a straight line for a given distance."""

    def __init__(self, start: Tuple[float, float], angle: float = 0.0,
                 distance: float = 10.0, speed: float = 0.8):
        end_x = start[0] + distance * math.cos(angle)
        end_y = start[1] + distance * math.sin(angle)
        self._walker = _WaypointWalker([start, (end_x, end_y)], speed=speed)

    def reset(self) -> PersonState:
        return self._walker.reset()

    def step(self, dt: float) -> PersonState:
        return self._walker.step(dt)

    @property
    def is_complete(self) -> bool:
        return self._walker.is_complete


class LTurnWalk:
    """Walk straight, make a 90-degree turn, walk straight."""

    def __init__(self, start: Tuple[float, float], leg1: float = 5.0,
                 leg2: float = 5.0, speed: float = 0.8, turn_right: bool = True):
        mid = (start[0] + leg1, start[1])
        if turn_right:
            end = (mid[0], mid[1] - leg2)
        else:
            end = (mid[0], mid[1] + leg2)
        self._walker = _WaypointWalker([start, mid, end], speed=speed)

    def reset(self) -> PersonState:
        return self._walker.reset()

    def step(self, dt: float) -> PersonState:
        return self._walker.step(dt)

    @property
    def is_complete(self) -> bool:
        return self._walker.is_complete


class UTurnWalk:
    """Walk forward, 180-degree turn, walk back."""

    def __init__(self, start: Tuple[float, float], distance: float = 5.0,
                 speed: float = 0.8):
        end_fwd = (start[0] + distance, start[1])
        self._walker = _WaypointWalker(
            [start, end_fwd, start], speed=speed
        )

    def reset(self) -> PersonState:
        return self._walker.reset()

    def step(self, dt: float) -> PersonState:
        return self._walker.step(dt)

    @property
    def is_complete(self) -> bool:
        return self._walker.is_complete


class StopAndGoWalk:
    """Alternating walk/stop cycles."""

    def __init__(self, start: Tuple[float, float], angle: float = 0.0,
                 speed: float = 0.8, walk_time: float = 3.0,
                 pause_time: float = 2.0, cycles: int = 3):
        self._speed = speed
        self._walk_time = walk_time
        self._pause_time = pause_time
        self._cycles = cycles
        self._angle = angle
        self._start = start
        # Build waypoints with implicit pauses
        total_dist = speed * walk_time * cycles
        end_x = start[0] + total_dist * math.cos(angle)
        end_y = start[1] + total_dist * math.sin(angle)
        self._walker = _WaypointWalker([start, (end_x, end_y)], speed=speed)
        self._time = 0.0
        self._cycle_time = walk_time + pause_time
        self._done = False

    def reset(self) -> PersonState:
        self._time = 0.0
        self._done = False
        return self._walker.reset()

    def step(self, dt: float) -> PersonState:
        self._time += dt
        cycle_pos = self._time % self._cycle_time
        if cycle_pos < self._walk_time and not self._walker.is_complete:
            return self._walker.step(dt)
        else:
            # Paused: return current position with zero velocity
            return PersonState(
                position=self._walker._pos.copy(),
                velocity=np.zeros(3),
                heading=self._walker._heading,
                timestamp=self._time,
            )

    @property
    def is_complete(self) -> bool:
        return self._time > self._cycle_time * self._cycles


class CircleWalk:
    """Walk in a circle."""

    def __init__(self, center: Tuple[float, float], radius: float = 3.0,
                 speed: float = 0.6, clockwise: bool = True):
        self._center = np.array([center[0], center[1], 0.0])
        self._radius = radius
        self._speed = speed
        self._direction = -1.0 if clockwise else 1.0
        self._omega = speed / radius * self._direction
        self._theta = 0.0
        self._time = 0.0
        self._done = False

    def reset(self) -> PersonState:
        self._theta = 0.0
        self._time = 0.0
        self._done = False
        return self._state()

    def step(self, dt: float) -> PersonState:
        self._time += dt
        self._theta += self._omega * dt
        if abs(self._theta) >= 2 * math.pi:
            self._done = True
        return self._state()

    @property
    def is_complete(self) -> bool:
        return self._done

    def _state(self) -> PersonState:
        x = self._center[0] + self._radius * math.cos(self._theta)
        y = self._center[1] + self._radius * math.sin(self._theta)
        vx = -self._radius * self._omega * math.sin(self._theta)
        vy = self._radius * self._omega * math.cos(self._theta)
        heading = math.atan2(vy, vx)
        return PersonState(
            position=np.array([x, y, 0.0]),
            velocity=np.array([vx, vy, 0.0]),
            heading=heading,
            timestamp=self._time,
        )


class Figure8Walk:
    """Walk a figure-8 (Lissajous curve)."""

    def __init__(self, center: Tuple[float, float], radius: float = 3.0,
                 speed: float = 0.6):
        self._center = np.array([center[0], center[1], 0.0])
        self._radius = radius
        self._speed = speed
        self._omega = speed / radius * 0.5
        self._theta = 0.0
        self._time = 0.0
        self._done = False

    def reset(self) -> PersonState:
        self._theta = 0.0
        self._time = 0.0
        self._done = False
        return self._state()

    def step(self, dt: float) -> PersonState:
        self._time += dt
        self._theta += self._omega * dt
        if self._theta >= 2 * math.pi:
            self._done = True
        return self._state()

    @property
    def is_complete(self) -> bool:
        return self._done

    def _state(self) -> PersonState:
        # Lissajous: x=sin(t), y=sin(2t)
        x = self._center[0] + self._radius * math.sin(self._theta)
        y = self._center[1] + self._radius * math.sin(2 * self._theta) * 0.5
        vx = self._radius * self._omega * math.cos(self._theta)
        vy = self._radius * self._omega * math.cos(2 * self._theta)
        heading = math.atan2(vy, vx)
        return PersonState(
            position=np.array([x, y, 0.0]),
            velocity=np.array([vx, vy, 0.0]),
            heading=heading,
            timestamp=self._time,
        )


class WaypointWalk:
    """General-purpose waypoint-following walk."""

    def __init__(self, waypoints: List[Tuple[float, float]],
                 speed: float = 0.8, loop: bool = False):
        self._walker = _WaypointWalker(waypoints, speed=speed, loop=loop)

    def reset(self) -> PersonState:
        return self._walker.reset()

    def step(self, dt: float) -> PersonState:
        return self._walker.step(dt)

    @property
    def is_complete(self) -> bool:
        return self._walker.is_complete


# ---------------------------------------------------------------------------
# Factory
# ---------------------------------------------------------------------------

TRAJECTORIES = {
    "straight": StraightWalk,
    "l_turn": LTurnWalk,
    "u_turn": UTurnWalk,
    "stop_and_go": StopAndGoWalk,
    "circle": CircleWalk,
    "figure8": Figure8Walk,
    "waypoint": WaypointWalk,
}
