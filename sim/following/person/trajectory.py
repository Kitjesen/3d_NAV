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


# ---------------------------------------------------------------------------
# Room-aware navigation (walks through doors, never through walls)
# ---------------------------------------------------------------------------

# go2_room_nova.xml layout: 20m x 14m house, 9 rooms
# Each room has a center and doors connect to the hallway
ROOM_GRAPH = {
    "hallway": {
        "center": (10.0, 5.0),
        "bounds": (6.0, 0.0, 14.0, 10.0),  # x_min, y_min, x_max, y_max
    },
    "living": {
        "center": (3.0, 2.5),
        "bounds": (0.0, 0.0, 6.0, 5.0),
    },
    "dining": {
        "center": (3.0, 7.5),
        "bounds": (0.0, 5.0, 6.0, 10.0),
    },
    "kitchen": {
        "center": (17.0, 2.5),
        "bounds": (14.0, 0.0, 20.0, 5.0),
    },
    "study": {
        "center": (17.0, 7.5),
        "bounds": (14.0, 5.0, 20.0, 10.0),
    },
    "master_br": {
        "center": (3.5, 12.0),
        "bounds": (0.0, 10.0, 7.0, 14.0),
    },
    "bathroom": {
        "center": (8.5, 12.0),
        "bounds": (7.0, 10.0, 10.0, 14.0),
    },
    "guest_br": {
        "center": (16.0, 12.0),
        "bounds": (12.0, 10.0, 20.0, 14.0),
    },
}

# Door positions: (x, y) center of the doorway gap
# Each door connects two rooms
DOORS = {
    ("hallway", "living"):     (6.0, 3.0),    # x=6, y=2.4~3.6
    ("hallway", "dining"):     (6.0, 8.0),    # x=6, y=7.4~8.6
    ("hallway", "kitchen"):    (14.0, 3.0),   # x=14, y=2.4~3.6
    ("hallway", "study"):      (14.0, 8.0),   # x=14, y=7.4~8.6
    ("hallway", "master_br"):  (3.0, 10.0),   # y=10, x=2.4~3.6
    ("hallway", "bathroom"):   (8.5, 10.0),   # y=10, x=7.9~9.1
    ("hallway", "guest_br"):   (12.0, 10.0),  # y=10, x=11.4~12.6
    ("living", "dining"):      (3.0, 5.0),    # y=5, x=2.4~3.6
    ("kitchen", "study"):      (17.0, 5.0),   # y=5, x=16.4~17.6
}


def _find_door(room_a: str, room_b: str) -> Tuple[float, float]:
    """Find the door connecting two rooms."""
    key = (room_a, room_b)
    if key in DOORS:
        return DOORS[key]
    key_rev = (room_b, room_a)
    if key_rev in DOORS:
        return DOORS[key_rev]
    raise ValueError(f"No door between {room_a} and {room_b}")


def _room_path_waypoints(room_sequence: List[str]) -> List[Tuple[float, float]]:
    """Convert a room sequence into waypoints that go through doors.

    Example: ["hallway", "master_br"]
      → [(10,5), (3.0,9.5), (3.0,10.5), (3.5,12)]
         center   approach    through     center
    """
    if not room_sequence:
        return []

    waypoints = []
    # Start at first room center
    first_center = ROOM_GRAPH[room_sequence[0]]["center"]
    waypoints.append(first_center)

    for i in range(len(room_sequence) - 1):
        src = room_sequence[i]
        dst = room_sequence[i + 1]
        door = _find_door(src, dst)

        # Approach door from source side (0.5m before door)
        dst_center = ROOM_GRAPH[dst]["center"]
        dx = dst_center[0] - door[0]
        dy = dst_center[1] - door[1]
        dist = math.hypot(dx, dy)
        if dist > 0.01:
            nx, ny = dx / dist, dy / dist
        else:
            nx, ny = 0.0, 1.0
        approach = (door[0] - nx * 0.5, door[1] - ny * 0.5)
        through = (door[0] + nx * 0.5, door[1] + ny * 0.5)

        waypoints.append(approach)
        waypoints.append(through)

        # End at destination center
        waypoints.append(dst_center)

    return waypoints


class RoomAwareWalk:
    """Person walks through rooms via doors — never through walls.

    Usage::

        walk = RoomAwareWalk(["hallway", "master_br"], speed=0.3)
        walk = RoomAwareWalk(["hallway", "living", "dining", "hallway", "master_br"])
    """

    def __init__(
        self,
        room_sequence: List[str],
        speed: float = 0.3,
        pause_at_door: float = 0.0,
        loop: bool = False,
    ):
        waypoints = _room_path_waypoints(room_sequence)
        self._walker = _WaypointWalker(waypoints, speed=speed, loop=loop)
        self._room_sequence = room_sequence
        self._pause_at_door = pause_at_door
        # Track which room the person is currently in
        self.current_room = room_sequence[0] if room_sequence else "hallway"

    def reset(self) -> PersonState:
        self.current_room = self._room_sequence[0] if self._room_sequence else "hallway"
        return self._walker.reset()

    def step(self, dt: float) -> PersonState:
        state = self._walker.step(dt)
        # Update current room based on position
        self.current_room = self._get_room(state.position)
        return state

    @property
    def is_complete(self) -> bool:
        return self._walker.is_complete

    @staticmethod
    def _get_room(pos: np.ndarray) -> str:
        """Determine which room a position is in."""
        x, y = float(pos[0]), float(pos[1])
        for name, info in ROOM_GRAPH.items():
            b = info["bounds"]
            if b[0] <= x <= b[2] and b[1] <= y <= b[3]:
                return name
        return "hallway"


class RoomTourWalk:
    """Person visits multiple rooms in sequence, returning to hallway between each.

    Usage::

        tour = RoomTourWalk(["living", "dining", "master_br"], speed=0.3)
        # Generates: hallway → living → hallway → dining → hallway → master_br
    """

    def __init__(
        self,
        rooms_to_visit: List[str],
        speed: float = 0.3,
        start_room: str = "hallway",
    ):
        # Build full sequence: start → room1 → start → room2 → ...
        full_seq = [start_room]
        for room in rooms_to_visit:
            if full_seq[-1] != "hallway" and room != "hallway":
                full_seq.append("hallway")
            full_seq.append(room)
        self._inner = RoomAwareWalk(full_seq, speed=speed)

    def reset(self) -> PersonState:
        return self._inner.reset()

    def step(self, dt: float) -> PersonState:
        return self._inner.step(dt)

    @property
    def is_complete(self) -> bool:
        return self._inner.is_complete

    @property
    def current_room(self) -> str:
        return self._inner.current_room
