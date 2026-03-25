#!/usr/bin/env python3
"""
MissionArcModule -- Mission lifecycle FSM as a core Module (no rclpy).

Converts the ROS2 MissionArc node into a pure-Python Module that:
  1. Receives goal_pose, planner_status, adapter_status, odometry, stop signals
  2. Manages mission FSM: IDLE -> PLANNING -> EXECUTING -> COMPLETE/FAILED
  3. Supports patrol mode (multi-waypoint sequential navigation)
  4. Handles replan on failure, recovery/planning/mission timeouts
  5. Publishes mission_status + goal_pose_out

All algorithm logic preserved from mission_arc.py:
  - MissionState enum with all transitions
  - Patrol mode with loop support
  - Replan budget (max_replan_count)
  - Recovery timeout -> replan escalation
  - Planning / mission timeout detection
  - E-stop awareness
  - NaN/Inf goal rejection

State flow:
  IDLE --goal_pose--> PLANNING --success--> EXECUTING --goal_reached--> COMPLETE
                         |                     |                          |
                       failed               stuck/warn              (patrol mode)
                         |                     |                     next waypoint
                         v                     v                       |
                       FAILED              RECOVERING                  v
                                               |                   PLANNING
                                            cleared/timeout
                                               |
                                            REPLANNING

  Any state --cancel--> IDLE (clears all state)
"""

from __future__ import annotations

import logging
import math
import time
from enum import Enum
from typing import Any, Dict, List, Optional

import sys as _sys
import os as _os

# Ensure src/ is on path for core imports
_src_dir = _os.path.abspath(
    _os.path.join(_os.path.dirname(__file__), "..", "..", "..")
)
if _src_dir not in _sys.path:
    _sys.path.insert(0, _src_dir)

from core import Module, In, Out
from core.msgs.nav import Odometry
from core.msgs.geometry import Pose, PoseStamped, Vector3

logger = logging.getLogger(__name__)


class MissionState(Enum):
    IDLE = "IDLE"
    PLANNING = "PLANNING"
    EXECUTING = "EXECUTING"
    RECOVERING = "RECOVERING"
    REPLANNING = "REPLANNING"
    COMPLETE = "COMPLETE"
    FAILED = "FAILED"


# States that accept a new goal
_ACCEPT_GOAL_STATES = {MissionState.IDLE, MissionState.COMPLETE, MissionState.FAILED}


class MissionArcModule(Module, layer=5):
    """Mission lifecycle FSM -- core Module version.

    Integrates planner_status + adapter_status + odometry + stop signals
    into a unified mission-level state machine, publishing consistent
    mission_status for App / gRPC / logging consumption.

    Ports:
        In:
            goal_pose       (PoseStamped) -- single goal trigger
            patrol_goals    (dict)        -- patrol mode: {waypoints, loop, route_name}
            cancel          (dict)        -- cancel current mission (any value)
            planner_status  (str)         -- global planner status
            adapter_status  (dict)        -- waypoint tracking events
            odometry        (Odometry)    -- robot position
            stop_signal     (int)         -- safety signal (0=clear, 2=stop)

        Out:
            mission_status  (dict)        -- mission state + progress
            goal_pose_out   (PoseStamped) -- goal forwarded to planner
    """

    # -- Input Ports --
    goal_pose: In[PoseStamped]
    patrol_goals: In[dict]
    cancel: In[dict]
    planner_status: In[str]
    adapter_status: In[dict]
    odometry: In[Odometry]
    stop_signal: In[int]

    # -- Output Ports --
    mission_status: Out[dict]
    goal_pose_out: Out[PoseStamped]

    def __init__(
        self,
        max_replan_count: int = 3,
        recovery_timeout_sec: float = 8.0,
        planning_timeout_sec: float = 30.0,
        mission_timeout_sec: float = 600.0,
        **config: Any,
    ) -> None:
        super().__init__(**config)

        # Parameters
        self.max_replan = max_replan_count
        self.recovery_timeout = recovery_timeout_sec
        self.planning_timeout = planning_timeout_sec
        self.mission_timeout = mission_timeout_sec

        # State
        self._state = MissionState.IDLE
        self._goal: Optional[PoseStamped] = None
        self._goal_xy = (0.0, 0.0)
        self._robot_xy = (0.0, 0.0)
        self._mission_start = 0.0
        self._state_enter_time = 0.0
        self._replan_count = 0
        self._wp_completed = 0
        self._wp_total = 0
        self._estop_active = False
        self._last_planner_status = "IDLE"
        self._failure_reason = ""

        # Patrol mode state
        self._patrol_waypoints: List[dict] = []
        self._patrol_index = 0
        self._patrol_loop = False
        self._patrol_name = ""
        self._patrol_laps = 0

    def setup(self) -> None:
        """Register input subscriptions."""
        self.goal_pose.subscribe(self._on_goal)
        self.patrol_goals.subscribe(self._on_patrol_goals)
        self.cancel.subscribe(self._on_cancel)
        self.planner_status.subscribe(self._on_planner_status)
        self.adapter_status.subscribe(self._on_adapter_status)
        self.odometry.subscribe(self._on_odom)
        self.stop_signal.subscribe(self._on_stop)

    # ================================================================
    # State transitions
    # ================================================================

    def _set_state(self, new_state: MissionState, reason: str = "") -> None:
        old = self._state
        if old == new_state:
            return
        self._state = new_state
        self._state_enter_time = time.monotonic()
        logger.info("Mission: %s -> %s  %s", old.value, new_state.value, reason)

        if new_state == MissionState.FAILED:
            self._failure_reason = reason

        # Patrol mode: COMPLETE one waypoint then advance to next
        if new_state == MissionState.COMPLETE and self._patrol_waypoints:
            self._publish_mission_status()  # publish current wp complete
            if self._advance_patrol():
                return  # advanced to next wp, don't stay in COMPLETE

        if new_state in (MissionState.COMPLETE, MissionState.FAILED):
            self._publish_mission_status()  # immediate terminal state publish

    # ================================================================
    # Input handlers
    # ================================================================

    def _on_goal(self, msg: PoseStamped) -> None:
        """New single goal -> start mission (clears patrol mode)."""
        gx = msg.x
        gy = msg.y
        if not (math.isfinite(gx) and math.isfinite(gy)):
            logger.warning("Rejecting NaN/Inf goal: (%.2f, %.2f)", gx, gy)
            return

        if self._state not in _ACCEPT_GOAL_STATES and self._state != MissionState.EXECUTING:
            if self._state in (MissionState.PLANNING, MissionState.RECOVERING,
                               MissionState.REPLANNING):
                logger.warning(
                    "Goal received during %s, overriding", self._state.value
                )

        # Single-goal mode: clear patrol state
        self._patrol_waypoints = []
        self._patrol_index = 0
        self._patrol_loop = False
        self._patrol_name = ""
        self._patrol_laps = 0

        self._goal = msg
        self._goal_xy = (gx, gy)
        self._replan_count = 0
        self._wp_completed = 0
        self._wp_total = 0
        self._failure_reason = ""
        self._mission_start = time.monotonic()
        self._set_state(MissionState.PLANNING, "new goal received")

    def _on_patrol_goals(self, data: dict) -> None:
        """Patrol route -> start multi-waypoint mission."""
        if not data:
            return

        waypoints = data.get("waypoints", [])
        if not waypoints:
            logger.warning("Patrol: empty waypoints, ignoring")
            return

        # Validate waypoints
        valid: List[dict] = []
        for wp in waypoints:
            x = wp.get("x", 0.0)
            y = wp.get("y", 0.0)
            if math.isfinite(x) and math.isfinite(y):
                valid.append({
                    "x": x, "y": y,
                    "z": wp.get("z", 0.0),
                    "name": wp.get("name", "WP%d" % len(valid)),
                })
        if not valid:
            logger.warning("Patrol: no valid waypoints")
            return

        self._patrol_waypoints = valid
        self._patrol_loop = data.get("loop", False)
        self._patrol_name = data.get("route_name", "")
        self._patrol_index = 0
        self._patrol_laps = 0
        self._replan_count = 0
        self._failure_reason = ""
        self._mission_start = time.monotonic()

        logger.info(
            "Patrol started: \"%s\" %d waypoints, loop=%s",
            self._patrol_name, len(valid), self._patrol_loop,
        )

        self._send_patrol_goal()

    def _on_cancel(self, msg: Any) -> None:
        """Cancel current mission -> back to IDLE."""
        if self._state == MissionState.IDLE:
            return
        old_state = self._state
        self._patrol_waypoints = []
        self._patrol_index = 0
        self._patrol_loop = False
        self._patrol_name = ""
        self._patrol_laps = 0
        self._set_state(MissionState.IDLE, "cancelled from %s" % old_state.value)
        logger.info("Mission cancelled by user")

    def _send_patrol_goal(self) -> None:
        """Send current patrol waypoint to planner."""
        if self._patrol_index >= len(self._patrol_waypoints):
            return

        wp = self._patrol_waypoints[self._patrol_index]
        goal = PoseStamped(
            pose=Pose(position=Vector3(float(wp["x"]), float(wp["y"]),
                                       float(wp.get("z", 0.0)))),
            ts=time.time(),
            frame_id="map",
        )

        self._goal = goal
        self._goal_xy = (wp["x"], wp["y"])
        self._wp_completed = 0
        self._wp_total = 0
        self._replan_count = 0

        self.goal_pose_out.publish(goal)
        self._set_state(
            MissionState.PLANNING,
            "patrol waypoint %d/%d \"%s\" (%.1f, %.1f)" % (
                self._patrol_index + 1, len(self._patrol_waypoints),
                wp.get("name", ""), wp["x"], wp["y"],
            ),
        )

    def _advance_patrol(self) -> bool:
        """Current waypoint complete -> advance to next or finish."""
        if not self._patrol_waypoints:
            return False  # not in patrol mode

        self._patrol_index += 1

        if self._patrol_index >= len(self._patrol_waypoints):
            self._patrol_laps += 1
            if self._patrol_loop:
                self._patrol_index = 0
                logger.info("Patrol lap %d complete, restarting", self._patrol_laps)
                self._send_patrol_goal()
                return True
            else:
                logger.info(
                    "Patrol complete: %d waypoints", len(self._patrol_waypoints)
                )
                return False  # all done
        else:
            wp = self._patrol_waypoints[self._patrol_index]
            logger.info(
                "Advancing to waypoint %d/%d \"%s\"",
                self._patrol_index + 1,
                len(self._patrol_waypoints),
                wp.get("name", ""),
            )
            self._send_patrol_goal()
            return True

    def _on_planner_status(self, status: str) -> None:
        """Handle planner status updates."""
        status = status.strip()
        self._last_planner_status = status

        if self._state == MissionState.PLANNING:
            if status == "SUCCESS":
                self._set_state(MissionState.EXECUTING, "path planned")
            elif status == "FAILED":
                self._set_state(MissionState.FAILED, "planner returned FAILED")

        elif self._state == MissionState.REPLANNING:
            if status == "SUCCESS":
                self._set_state(
                    MissionState.EXECUTING,
                    "replan #%d succeeded" % self._replan_count,
                )
            elif status == "FAILED":
                self._set_state(
                    MissionState.FAILED,
                    "replan #%d failed" % self._replan_count,
                )

        elif self._state == MissionState.EXECUTING:
            if status == "WARN_STUCK":
                self._set_state(MissionState.RECOVERING, "pathFollower WARN_STUCK")
            elif status == "STUCK":
                self._set_state(MissionState.RECOVERING, "pathFollower STUCK")
            elif status == "GOAL_REACHED":
                self._set_state(MissionState.COMPLETE, "planner_status GOAL_REACHED")
            elif status == "FAILED":
                self._set_state(MissionState.FAILED, "planner FAILED during execution")

    def _on_adapter_status(self, data: dict) -> None:
        """Handle adapter status events."""
        if not isinstance(data, dict):
            return

        event = data.get("event", "")

        if event == "path_received":
            self._wp_total = data.get("total", 0)
            self._wp_completed = 0

        elif event == "waypoint_reached":
            self._wp_completed = data.get("index", 0) + 1
            self._wp_total = data.get("total", self._wp_total)
            # If recovering and waypoint advances -> recovered
            if self._state == MissionState.RECOVERING:
                self._set_state(MissionState.EXECUTING, "recovered (waypoint advancing)")

        elif event == "goal_reached":
            self._set_state(MissionState.COMPLETE, "adapter goal_reached")

        elif event == "replanning":
            self._replan_count += 1

        elif event == "stuck_final":
            if self._state in (MissionState.EXECUTING, MissionState.RECOVERING):
                self._try_replan("adapter stuck_final")

        elif event == "failed":
            if self._state in (MissionState.EXECUTING, MissionState.PLANNING):
                self._set_state(
                    MissionState.FAILED,
                    data.get("reason", "adapter failed"),
                )

    def _on_odom(self, msg: Odometry) -> None:
        """Update robot position."""
        x, y = msg.x, msg.y
        if math.isfinite(x) and math.isfinite(y):
            self._robot_xy = (x, y)

    def _on_stop(self, value: int) -> None:
        """Handle safety stop signal."""
        self._estop_active = (value >= 2)

    # ================================================================
    # Core logic
    # ================================================================

    def _try_replan(self, reason: str) -> None:
        """Attempt replan; if budget exceeded, FAIL.

        mission_arc does not re-send goal_pose. Replan execution is done
        by pct_path_adapter (sends replanning event and re-requests path).
        We only record state and wait for planner_status SUCCESS/FAILED.
        """
        if self._replan_count >= self.max_replan:
            self._set_state(
                MissionState.FAILED,
                "exceeded max replan (%d): %s" % (self.max_replan, reason),
            )
            return

        self._replan_count += 1
        self._set_state(
            MissionState.REPLANNING,
            "replan #%d: %s" % (self._replan_count, reason),
        )

    def tick(self) -> None:
        """Periodic check for timeouts + publish status. Call at ~2Hz."""
        now = time.monotonic()
        dt_state = now - self._state_enter_time

        # PLANNING timeout
        if self._state == MissionState.PLANNING:
            if dt_state > self.planning_timeout:
                self._set_state(
                    MissionState.FAILED,
                    "planning timeout (%.1fs)" % self.planning_timeout,
                )

        # RECOVERING timeout -> replan
        elif self._state == MissionState.RECOVERING:
            if dt_state > self.recovery_timeout:
                self._try_replan("recovery timeout")

        # REPLANNING timeout (same as planning)
        elif self._state == MissionState.REPLANNING:
            if dt_state > self.planning_timeout:
                self._set_state(
                    MissionState.FAILED,
                    "replan timeout (%.1fs)" % self.planning_timeout,
                )

        # Global mission timeout
        if self._state in (
            MissionState.PLANNING, MissionState.EXECUTING,
            MissionState.RECOVERING, MissionState.REPLANNING,
        ):
            if now - self._mission_start > self.mission_timeout:
                self._set_state(
                    MissionState.FAILED,
                    "mission timeout (%.1fs)" % self.mission_timeout,
                )

        self._publish_mission_status()

    # ================================================================
    # Status publishing
    # ================================================================

    def _distance_to_goal(self) -> float:
        dx = self._goal_xy[0] - self._robot_xy[0]
        dy = self._goal_xy[1] - self._robot_xy[1]
        return math.sqrt(dx * dx + dy * dy)

    def _publish_mission_status(self) -> None:
        now = time.monotonic()
        elapsed = now - self._mission_start if self._mission_start > 0 else 0.0

        status: Dict[str, Any] = {
            "state": self._state.value,
            "elapsed_sec": round(elapsed, 1),
            "replan_count": self._replan_count,
            "estop": self._estop_active,
        }

        if self._goal is not None:
            status["goal"] = {
                "x": round(self._goal_xy[0], 2),
                "y": round(self._goal_xy[1], 2),
                "frame": self._goal.frame_id,
            }
            status["distance_to_goal"] = round(self._distance_to_goal(), 2)

        if self._wp_total > 0:
            status["progress"] = {
                "waypoints_completed": self._wp_completed,
                "waypoints_total": self._wp_total,
                "percent": round(100.0 * self._wp_completed / self._wp_total, 1),
            }

        # Patrol mode extra info
        if self._patrol_waypoints:
            patrol_total = len(self._patrol_waypoints)
            patrol_pct = round(100.0 * self._patrol_index / patrol_total, 1)
            current_wp = (
                self._patrol_waypoints[self._patrol_index]
                if self._patrol_index < patrol_total else None
            )
            status["patrol"] = {
                "route_name": self._patrol_name,
                "current_index": self._patrol_index,
                "total_waypoints": patrol_total,
                "percent": patrol_pct,
                "loop": self._patrol_loop,
                "laps_completed": self._patrol_laps,
                "current_waypoint_name": (
                    current_wp.get("name", "") if current_wp else None
                ),
            }

        if self._state == MissionState.FAILED:
            status["failure_reason"] = self._failure_reason

        self.mission_status.publish(status)

    # ================================================================
    # Properties (read-only inspection)
    # ================================================================

    @property
    def state(self) -> MissionState:
        return self._state

    @property
    def state_value(self) -> str:
        return self._state.value

    @property
    def replan_count(self) -> int:
        return self._replan_count

    @property
    def failure_reason(self) -> str:
        return self._failure_reason

    @property
    def estop_active(self) -> bool:
        return self._estop_active
