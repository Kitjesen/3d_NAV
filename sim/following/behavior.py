"""Person-following behavior state machine.

Orchestrates the high-level decision loop for following a person:
  FOLLOW  → person visible, normal tracking
  SEARCH  → person lost briefly, look toward last seen direction
  EXPLORE → person lost for a while, expand search
  RECOVER → repeated failures, LLM decides next action
  WAIT    → person stationary, maintain distance

Each state maps to existing LingTu subsystems:
  FOLLOW  → VisualServoModule + PersonTracker + PurePursuit
  SEARCH  → ActionExecutor.LOOK_AROUND + APPROACH_TARGET
  EXPLORE → FrontierScorer + exploration_strategy
  RECOVER → LERa (Look → Explain → Replan)

The behavior FSM is the "brain" layer that sits on top of the
perception pipeline and following controller. It decides WHAT to do;
the controller decides HOW to do it.
"""
from __future__ import annotations

import logging
import math
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

import numpy as np

from sim.following.interfaces import (
    FollowCommand,
    PerceivedTarget,
    PersonState,
)

logger = logging.getLogger(__name__)


class BehaviorState(str, Enum):
    FOLLOW = "follow"       # person visible, tracking
    SEARCH = "search"       # lost briefly, look toward last direction
    EXPLORE = "explore"     # lost for a while, expand search
    RECOVER = "recover"     # repeated failures, need LLM help
    WAIT = "wait"           # person stopped, hold distance


@dataclass
class BehaviorConfig:
    """Tuning parameters for the behavior state machine."""
    # Timeouts for state transitions
    search_timeout_s: float = 5.0       # FOLLOW → SEARCH after this long without detection
    explore_timeout_s: float = 15.0     # SEARCH → EXPLORE after this long
    recover_timeout_s: float = 30.0     # EXPLORE → RECOVER after this long
    # Thresholds
    person_stopped_speed: float = 0.1   # person velocity below this → WAIT
    person_stopped_time_s: float = 1.0  # person must be stopped for this long to trigger WAIT
    # Search behavior
    search_turn_speed: float = 0.8      # rad/s for LOOK_AROUND
    search_approach_speed: float = 0.3  # m/s toward last seen position
    # Explore
    explore_radius: float = 5.0         # meters to expand search


@dataclass
class LastSeenInfo:
    """Memory of the last time the person was detected."""
    position: np.ndarray = field(default_factory=lambda: np.zeros(3))
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))
    heading: float = 0.0
    timestamp: float = 0.0
    # Computed: which direction to search
    disappear_direction: float = 0.0    # yaw angle toward last movement direction


class FollowingBehavior:
    """High-level behavior state machine for person following.

    Usage in the simulation loop::

        behavior = FollowingBehavior(config)

        # Each step:
        cmd = behavior.update(
            robot_pos, robot_yaw,
            perceived_target,     # from perception pipeline (or None)
            person_gt,            # ground truth for metrics (optional)
            dt,
        )
        # cmd is a FollowCommand to send to the locomotion controller
    """

    def __init__(
        self,
        config: BehaviorConfig | None = None,
        controller=None,
        search_advisor=None,
    ):
        self._cfg = config or BehaviorConfig()
        self._controller = controller
        self._search_advisor = search_advisor  # SearchAdvisor (optional)
        self._state = BehaviorState.FOLLOW
        self._state_enter_time = 0.0
        self._last_detection_time = 0.0
        self._last_seen = LastSeenInfo()
        self._person_stopped_since: float | None = None
        self._search_phase = 0  # 0=turn toward, 1=approach, 2=look_around
        self._search_turn_progress = 0.0
        self._time = 0.0
        self._total_failures = 0
        self._last_rgb: np.ndarray | None = None  # for VLM search

    @property
    def state(self) -> BehaviorState:
        return self._state

    @property
    def last_seen(self) -> LastSeenInfo:
        return self._last_seen

    def update(
        self,
        robot_pos: np.ndarray,
        robot_yaw: float,
        target: PerceivedTarget | None,
        person_gt: PersonState | None,
        dt: float,
    ) -> FollowCommand:
        """Run one step of the behavior state machine."""
        self._time += dt
        detected = target is not None and target.confidence > 0.1

        # Update last seen info when detected
        if detected:
            self._last_detection_time = self._time
            self._last_seen.position = target.position_world.copy()
            self._last_seen.velocity = target.velocity_world.copy()
            self._last_seen.timestamp = self._time
            speed = np.linalg.norm(target.velocity_world[:2])
            if speed > 0.05:
                self._last_seen.disappear_direction = math.atan2(
                    target.velocity_world[1], target.velocity_world[0]
                )
            self._total_failures = 0

        # Time since last detection
        lost_duration = self._time - self._last_detection_time

        # ── State transitions ──
        if detected:
            # Check if person stopped
            speed = np.linalg.norm(target.velocity_world[:2])
            if speed < self._cfg.person_stopped_speed:
                if self._person_stopped_since is None:
                    self._person_stopped_since = self._time
                elif self._time - self._person_stopped_since > self._cfg.person_stopped_time_s:
                    self._transition(BehaviorState.WAIT)
            else:
                self._person_stopped_since = None
                self._transition(BehaviorState.FOLLOW)
        else:
            # Not detected — escalate through search states
            if lost_duration < self._cfg.search_timeout_s:
                if self._state == BehaviorState.FOLLOW:
                    pass  # brief dropout, keep following last command
            elif lost_duration < self._cfg.explore_timeout_s:
                if self._state != BehaviorState.SEARCH:
                    self._transition(BehaviorState.SEARCH)
            elif lost_duration < self._cfg.recover_timeout_s:
                if self._state != BehaviorState.EXPLORE:
                    self._transition(BehaviorState.EXPLORE)
            else:
                if self._state != BehaviorState.RECOVER:
                    self._transition(BehaviorState.RECOVER)

        # ── Execute current state ──
        if self._state == BehaviorState.FOLLOW:
            return self._do_follow(robot_pos, robot_yaw, target, dt)
        elif self._state == BehaviorState.WAIT:
            return self._do_wait(robot_pos, robot_yaw, target, dt)
        elif self._state == BehaviorState.SEARCH:
            return self._do_search(robot_pos, robot_yaw, dt)
        elif self._state == BehaviorState.EXPLORE:
            return self._do_explore(robot_pos, robot_yaw, dt)
        elif self._state == BehaviorState.RECOVER:
            return self._do_recover(robot_pos, robot_yaw, dt)
        return FollowCommand()

    # ── State behaviors ──

    def _do_follow(self, robot_pos, robot_yaw, target, dt) -> FollowCommand:
        """Normal following — delegate to controller."""
        if target is None:
            # Brief dropout: continue toward last seen position
            fake_target = PerceivedTarget(
                position_world=self._last_seen.position.copy(),
                velocity_world=self._last_seen.velocity.copy(),
                confidence=0.5,
                timestamp=self._last_seen.timestamp,
            )
            if self._controller:
                return self._controller.compute(robot_pos, robot_yaw, fake_target, dt)
            return FollowCommand()
        if self._controller:
            return self._controller.compute(robot_pos, robot_yaw, target, dt)
        return FollowCommand()

    def _do_wait(self, robot_pos, robot_yaw, target, dt) -> FollowCommand:
        """Person stopped — hold position at target distance."""
        if target is not None and self._controller:
            cmd = self._controller.compute(robot_pos, robot_yaw, target, dt)
            # Dampen movement when person is stationary
            return FollowCommand(
                vx=cmd.vx * 0.3,
                vy=cmd.vy * 0.3,
                dyaw=cmd.dyaw * 0.5,
            )
        return FollowCommand()

    def _do_search(self, robot_pos, robot_yaw, dt) -> FollowCommand:
        """Person lost briefly — use SearchAdvisor cascade or geometric fallback.

        With SearchAdvisor (3-tier):
          Fast:   YOLO doors/corridors → immediate direction
          Medium: SmolVLM scene description → update direction (async)
          Slow:   Kimi API reasoning → update direction (async)

        Without SearchAdvisor (geometric fallback):
          0. Turn toward person's last movement direction
          1. Walk toward last seen position
          2. Slow 360 scan (look around)
        """
        time_in_state = self._time - self._state_enter_time

        # ── SearchAdvisor path (when available) ──
        if self._search_advisor is not None:
            advice = self._search_advisor.get_search_direction(
                rgb_image=self._last_rgb,
                last_position=self._last_seen.position,
                last_velocity=self._last_seen.velocity,
                robot_position=robot_pos,
                robot_yaw=robot_yaw,
            )
            if advice.confidence > 0.1:
                # Use advisor's recommended direction
                if advice.position is not None:
                    dx = advice.position[0] - robot_pos[0]
                    dy = advice.position[1] - robot_pos[1]
                    dist = math.hypot(dx, dy)
                    angle = math.atan2(dy, dx)
                    angle_err = self._angle_diff(angle, robot_yaw)
                    vx = min(self._cfg.search_approach_speed, dist * 0.4)
                    dyaw = np.clip(1.5 * angle_err, -1.0, 1.0)
                    logger.debug(
                        "SEARCH advisor [%s] conf=%.2f: %s → vx=%.2f dyaw=%.2f",
                        advice.source, advice.confidence,
                        advice.description[:50], vx, dyaw,
                    )
                    return FollowCommand(vx=float(vx), dyaw=float(dyaw))
                else:
                    angle_err = self._angle_diff(advice.direction, robot_yaw)
                    dyaw = np.clip(1.5 * angle_err, -self._cfg.search_turn_speed, self._cfg.search_turn_speed)
                    vx = 0.2 if abs(angle_err) < 0.3 else 0.0
                    return FollowCommand(vx=float(vx), dyaw=float(dyaw))

        # ── Geometric fallback (no advisor) ──
        if self._search_phase == 0:
            angle_err = self._angle_diff(self._last_seen.disappear_direction, robot_yaw)
            if abs(angle_err) < 0.2 or time_in_state > 3.0:
                self._search_phase = 1
                return FollowCommand()
            dyaw = np.clip(1.5 * angle_err, -self._cfg.search_turn_speed, self._cfg.search_turn_speed)
            return FollowCommand(dyaw=float(dyaw))

        elif self._search_phase == 1:
            dx = self._last_seen.position[0] - robot_pos[0]
            dy = self._last_seen.position[1] - robot_pos[1]
            dist = math.hypot(dx, dy)
            if dist < 1.0 or time_in_state > 7.0:
                self._search_phase = 2
                self._search_turn_progress = 0.0
                return FollowCommand()
            angle = math.atan2(dy, dx)
            angle_err = self._angle_diff(angle, robot_yaw)
            vx = min(self._cfg.search_approach_speed, dist * 0.5)
            dyaw = np.clip(1.2 * angle_err, -1.0, 1.0)
            return FollowCommand(vx=float(vx), dyaw=float(dyaw))

        else:
            self._search_turn_progress += self._cfg.search_turn_speed * 0.5 * dt
            if self._search_turn_progress > 2 * math.pi:
                self._search_phase = 0
            return FollowCommand(dyaw=float(self._cfg.search_turn_speed * 0.5))

    def _do_explore(self, robot_pos, robot_yaw, dt) -> FollowCommand:
        """Person lost for a while — expand search in predicted direction.

        Walk in the direction the person was heading, then spiral outward.
        In real system: would call FrontierScorer for information-gain exploration.
        """
        time_in_state = self._time - self._state_enter_time

        # Walk in predicted direction with slow yaw sweep
        target_x = self._last_seen.position[0] + self._last_seen.velocity[0] * time_in_state
        target_y = self._last_seen.position[1] + self._last_seen.velocity[1] * time_in_state

        dx = target_x - robot_pos[0]
        dy = target_y - robot_pos[1]
        dist = math.hypot(dx, dy)
        angle = math.atan2(dy, dx)
        angle_err = self._angle_diff(angle, robot_yaw)

        # Add a slow sweep to look around while moving
        sweep = 0.3 * math.sin(time_in_state * 0.5)

        vx = min(0.4, dist * 0.3)
        dyaw = np.clip(1.0 * angle_err + sweep, -1.0, 1.0)
        return FollowCommand(vx=float(vx), dyaw=float(dyaw))

    def _do_recover(self, robot_pos, robot_yaw, dt) -> FollowCommand:
        """Repeated failures — stop and wait for re-lock.

        In real system: would call LERa → LLM decides retry/expand/abort.
        In simulation: just stop.
        """
        self._total_failures += 1
        logger.info(
            "RECOVER: person lost for %.1fs, failures=%d. Stopping.",
            self._time - self._last_detection_time,
            self._total_failures,
        )
        return FollowCommand()

    # ── Helpers ──

    def _transition(self, new_state: BehaviorState) -> None:
        if new_state != self._state:
            logger.debug(
                "Behavior: %s → %s (t=%.1f, lost=%.1fs)",
                self._state.value, new_state.value,
                self._time, self._time - self._last_detection_time,
            )
            self._state = new_state
            self._state_enter_time = self._time
            self._search_phase = 0

    @staticmethod
    def _angle_diff(target: float, current: float) -> float:
        """Signed angle difference, wraps to [-pi, pi]."""
        diff = (target - current + math.pi) % (2 * math.pi) - math.pi
        return diff

    def reset(self) -> None:
        self._state = BehaviorState.FOLLOW
        self._state_enter_time = 0.0
        self._last_detection_time = 0.0
        self._last_seen = LastSeenInfo()
        self._person_stopped_since = None
        self._search_phase = 0
        self._time = 0.0
        self._total_failures = 0
