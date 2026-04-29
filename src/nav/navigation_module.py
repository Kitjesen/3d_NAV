"""NavigationModule — unified path planning + tracking + mission FSM.

Internal services (not Modules — no ports, just algorithm logic):
  - GlobalPlannerService  (nav/global_planner_service.py) — plan() + downsample
  - WaypointTracker       (nav/waypoint_tracker.py)       — arrival + stuck detection

Ports:
  In:  goal_pose, odometry, costmap, instruction, stop_signal, patrol_goals, cancel
  Out: waypoint, global_path, planner_status, mission_status, adapter_status

Usage::

    bp.add(NavigationModule, planner="astar", tomogram="building.pickle")
    bp.add(NavigationModule, planner="pct")   # S100P with ele_planner.so
"""

from __future__ import annotations

import json
import logging
import math
import time
from typing import Any, Dict, List, Optional

import numpy as np

from core.module import Module, skill
from core.msgs.geometry import Pose, PoseStamped, Quaternion, Twist, Vector3
from core.msgs.nav import Odometry
from core.registry import register
from core.stream import In, Out
from nav.global_planner_service import GlobalPlannerService
from nav.waypoint_tracker import (
    EV_PATH_COMPLETE,
    EV_STUCK,
    EV_STUCK_WARN,
    EV_WAYPOINT_REACHED,
    WaypointTracker,
)

logger = logging.getLogger(__name__)


class MissionState:
    IDLE       = "IDLE"
    PLANNING   = "PLANNING"
    EXECUTING  = "EXECUTING"
    SUCCESS    = "SUCCESS"
    FAILED     = "FAILED"
    STUCK      = "STUCK"
    CANCELLED  = "CANCELLED"
    PATROLLING = "PATROLLING"

    # Legal state transitions. Any state → IDLE is always allowed (stop/cancel/reset).
    TRANSITIONS: dict[str, frozenset[str]] = {
        "IDLE":       frozenset(["PLANNING", "PATROLLING"]),
        "PLANNING":   frozenset(["EXECUTING", "FAILED", "IDLE", "CANCELLED"]),
        "EXECUTING":  frozenset(["SUCCESS", "FAILED", "STUCK", "IDLE", "PLANNING", "CANCELLED"]),
        "PATROLLING": frozenset(["EXECUTING", "SUCCESS", "FAILED", "STUCK", "IDLE", "PLANNING", "CANCELLED"]),
        "SUCCESS":    frozenset(["IDLE", "PLANNING", "PATROLLING"]),
        "FAILED":     frozenset(["IDLE", "PLANNING", "PATROLLING"]),
        "STUCK":      frozenset(["IDLE", "PLANNING", "CANCELLED"]),
        "CANCELLED":  frozenset(["IDLE", "PLANNING", "PATROLLING"]),
    }


@register("navigation", "default", description="Unified navigation module")
class NavigationModule(Module, layer=5):
    """Unified navigation: plan → track → FSM in one Module.

    Architecture:
      NavigationModule owns the mission FSM and patrol logic.
      GlobalPlannerService owns planning algorithm + downsample.
      WaypointTracker owns arrival detection + stuck detection.
    """

    # -- Inputs --
    goal_pose:    In[PoseStamped]
    odometry:     In[Odometry]
    costmap:      In[dict]
    instruction:  In[str]
    stop_signal:  In[int]
    patrol_goals: In[list]
    cancel:       In[str]
    teleop_active: In[bool]
    localization_status: In[dict]
    traversability: In[dict]  # W2-8: terrain class from TerrainModule
    # P4: TF jump events from SlamBridgeModule. PGO loop closures and BBS3D
    # relocalisations can move map→odom by metres in a single tick. Cached
    # global path + waypoint then point at the wrong place. We force a replan
    # on the next planning tick to catch up.
    map_frame_jump_event: In[dict]

    # -- Outputs --
    waypoint:       Out[PoseStamped]
    global_path:    Out[list]
    planner_status: Out[str]
    mission_status: Out[dict]
    adapter_status: Out[dict]
    recovery_cmd_vel: Out[Twist]  # direct cmd_vel for backup/rotate recovery

    def __init__(
        self,
        planner: str = "astar",
        tomogram: str = "",
        obstacle_thr: float = 49.9,
        waypoint_threshold: float = 1.5,
        stuck_timeout: float = 10.0,
        stuck_dist_thre: float = 0.15,
        max_replan_count: int = 3,
        downsample_dist: float = 2.0,
        enable_ros2_bridge: bool = False,
        allow_direct_goal_fallback: bool = False,
        goal_update_epsilon: float = 0.25,
        **kw,
    ):
        super().__init__(**kw)
        self._enable_ros2_bridge = enable_ros2_bridge
        self._allow_direct_goal_fallback = allow_direct_goal_fallback
        self._goal_update_epsilon = goal_update_epsilon

        self._planner_svc = GlobalPlannerService(
            planner_name=planner,
            tomogram=tomogram,
            obstacle_thr=obstacle_thr,
            downsample_dist=downsample_dist,
        )
        self._tracker = WaypointTracker(
            threshold=waypoint_threshold,
            stuck_timeout=stuck_timeout,
            stuck_dist=stuck_dist_thre,
        )

        # Mission FSM state
        self._state = MissionState.IDLE
        self._robot_pos = np.zeros(3)
        self._goal: np.ndarray | None = None
        self._replan_count = 0
        self._max_replan = max_replan_count
        self._failure_reason = ""
        self._mission_start_time = 0.0
        self._mission_timeout = kw.get("mission_timeout", 300.0)

        # Localization-aware pause/resume
        self._loc_state: str = "UNINIT"
        self._loc_confidence: float = 0.0
        self._degen_level: str = "NONE"
        self._paused_for_localization: bool = False
        self._pre_pause_state: str | None = None
        self._planning_timeout = kw.get("planning_timeout", 30.0)
        self._speed_scale: float = 1.0  # degeneracy-based speed multiplier

        # Teleop pause/resume
        self._paused_for_teleop: bool = False
        self._pre_teleop_goal: np.ndarray | None = None
        self._pre_teleop_state: str | None = None

        # Cooldown for costmap-triggered replanning (3s minimum between replans)
        self._last_costmap_replan_time = 0.0

        # Patrol FSM state
        self._patrol_goals: list[np.ndarray] = []
        self._patrol_index = 0
        self._patrol_loop = False

        # Optional ROS2 bridge (set in setup)
        self._ros2_node = None
        self._ros2_wp_pub = None

        # W2-8: latest terrain class from TerrainModule — defaults to "unknown"
        # which triggers the conservative backup+rotate recovery strategy.
        self._latest_traversability_class: str = "unknown"

    def setup(self) -> None:
        self._planner_svc.setup()

        self.goal_pose.subscribe(self._on_goal)
        self.odometry.subscribe(self._on_odom)
        self.costmap.subscribe(self._on_costmap)
        self.instruction.subscribe(self._on_instruction)
        self.stop_signal.subscribe(self._on_stop)
        self.patrol_goals.subscribe(self._on_patrol_goals)
        self.cancel.subscribe(self._on_cancel)
        self.teleop_active.subscribe(self._on_teleop_active)
        self.localization_status.subscribe(self._on_localization_status)
        self.traversability.subscribe(self._on_traversability)
        self.map_frame_jump_event.subscribe(self._on_map_frame_jump)

        if self._enable_ros2_bridge:
            try:
                from geometry_msgs.msg import PointStamped
                from rclpy.node import Node
                from rclpy.qos import QoSProfile, ReliabilityPolicy

                from core.ros2_context import ensure_rclpy, get_shared_executor

                ensure_rclpy()
                self._ros2_node = Node("nav_waypoint_bridge")
                qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
                self._ros2_wp_pub = self._ros2_node.create_publisher(
                    PointStamped, "/nav/way_point", qos
                )
                get_shared_executor().add_node(self._ros2_node)
                logger.info("NavigationModule: ROS2 waypoint bridge enabled -> /nav/way_point")
            except (ImportError, Exception) as e:
                logger.debug("NavigationModule: ROS2 not available: %s", e)
        else:
            logger.info("NavigationModule: ROS2 bridge disabled (Python autonomy stack active)")

        self._set_state(MissionState.IDLE)

    # ROS2 spin handled by shared executor (core.ros2_context)

    # ── Mission FSM ───────────────────────────────────────────────────────

    def _set_state(self, state: str) -> None:
        allowed = MissionState.TRANSITIONS.get(self._state, frozenset())
        if state != self._state and state not in allowed:
            logger.warning(
                "NavigationModule: illegal state transition %s → %s (allowed: %s)",
                self._state, state, sorted(allowed),
            )
        self._state = state
        self.planner_status.publish(state)
        self.mission_status.publish({
            "state": state,
            "replan_count": self._replan_count,
            "wp_index": self._tracker.wp_index,
            "wp_total": self._tracker.path_length,
            "speed_scale": self._speed_scale,
            "degeneracy": self._degen_level,
            "ts": time.time(),
        })

    # ── Input handlers ────────────────────────────────────────────────────

    def _on_goal(self, goal: PoseStamped) -> None:
        new_goal = np.array([
            goal.pose.position.x,
            goal.pose.position.y,
            goal.pose.position.z,
        ])
        if self._should_ignore_goal_update(new_goal):
            return
        self._goal = new_goal
        self._replan_count = 0
        self._plan()

    def _should_ignore_goal_update(self, new_goal: np.ndarray) -> bool:
        if self._goal is None:
            return False
        if self._state not in (MissionState.PLANNING, MissionState.EXECUTING, MissionState.PATROLLING):
            return False
        dist = float(np.linalg.norm(new_goal[:2] - self._goal[:2]))
        if dist > self._goal_update_epsilon:
            return False
        self.adapter_status.publish({
            "event": "goal_update_ignored",
            "distance": dist,
            "goal": [float(new_goal[0]), float(new_goal[1]), float(new_goal[2])],
            "ts": time.time(),
        })
        return True

    def _on_instruction(self, text: str) -> None:
        logger.info("NavigationModule received instruction: %s", text[:50])

    def _on_stop(self, level: int) -> None:
        if level >= 1:
            self._tracker.clear()
            self._set_state(MissionState.IDLE)

    def _on_teleop_active(self, active: bool) -> None:
        """Pause navigation when teleop engages, resume when released."""
        if active and not self._paused_for_teleop:
            # Save current mission state so we can resume
            if self._state in (MissionState.EXECUTING, MissionState.PATROLLING):
                self._pre_teleop_goal = self._goal.copy() if self._goal is not None else None
                self._pre_teleop_state = self._state
                self._tracker.clear()
                self._set_state(MissionState.IDLE)
                logger.info("NavigationModule: paused for teleop (saved goal)")
            self._paused_for_teleop = True

        elif not active and self._paused_for_teleop:
            self._paused_for_teleop = False
            # Resume previous mission if we had one
            if (self._pre_teleop_goal is not None
                    and self._state == MissionState.IDLE):
                logger.info("NavigationModule: teleop released, resuming navigation")
                self._goal = self._pre_teleop_goal
                self._pre_teleop_goal = None
                self._pre_teleop_state = None
                self._plan()
            else:
                logger.info("NavigationModule: teleop released, no mission to resume")
                self._pre_teleop_goal = None
                self._pre_teleop_state = None

    def _on_costmap(self, data: dict) -> None:
        grid = data.get("grid")
        if grid is None:
            return
        self._planner_svc.update_map(
            grid,
            resolution=data.get("resolution", 0.2),
            origin=data.get("origin"),
        )
        if (self._state in (MissionState.EXECUTING, MissionState.PATROLLING,
                            MissionState.FAILED)
                and self._goal is not None
                and time.time() - self._last_costmap_replan_time > 3.0):
            self._last_costmap_replan_time = time.time()
            self._plan()

    def _on_cancel(self, msg: str) -> None:
        if self._state in (MissionState.IDLE, MissionState.CANCELLED):
            return
        self._tracker.clear()
        self._patrol_goals.clear()
        self._patrol_index = 0
        self._failure_reason = f"cancelled: {msg}" if msg else "cancelled"
        self._set_state(MissionState.CANCELLED)
        logger.info("Mission cancelled: %s", msg)

    def _on_map_frame_jump(self, event: dict) -> None:
        """SlamBridge detected a map↔odom TF discontinuity (P4).

        The cached global path / waypoint were planned in the *old* map frame.
        Force an immediate replan so we don't drive the robot toward a
        coordinate that no longer matches reality. Costmap is in odom frame
        so it remains valid; ESDF / OccupancyGrid handle their own clear via
        their own subscription to this event.
        """
        if not isinstance(event, dict):
            return
        dt_m = event.get("dt_m", 0.0)
        dyaw = event.get("dyaw_deg", 0.0)
        # Only act if we have an active mission — idle robot doesn't care.
        if self._state in (MissionState.EXECUTING, MissionState.PATROLLING) \
                and self._goal is not None:
            logger.warning(
                "TF jump (Δt=%.2fm Δyaw=%.1f°) → forced replan",
                dt_m, dyaw)
            self._tracker.clear()  # invalidate current waypoint tracking
            self._plan()

    def _on_localization_status(self, msg: dict) -> None:
        prev = self._loc_state
        self._loc_state = msg.get("state", "UNINIT")
        self._loc_confidence = msg.get("confidence", 0.0)
        self._degen_level = msg.get("degeneracy", "NONE")

        if self._loc_state == "LOST" and prev != "LOST":
            if self._state in (MissionState.EXECUTING, MissionState.PATROLLING):
                self._pre_pause_state = self._state
                self._paused_for_localization = True
                self._tracker.pause()
                self._set_state(MissionState.IDLE)
                logger.warning("Navigation PAUSED: localization lost")

        elif self._loc_state == "TRACKING" and self._paused_for_localization:
            self._paused_for_localization = False
            if self._pre_pause_state and self._goal is not None:
                self._set_state(self._pre_pause_state)
                self._pre_pause_state = None
                logger.info("Navigation RESUMED: localization recovered")

        # Degeneracy-aware speed scaling
        self._apply_degeneracy_speed_limit()

    def _apply_degeneracy_speed_limit(self) -> None:
        """Scale navigation speed based on SLAM health.

        FALLBACK_GNSS_ONLY → 0.3x (cautious — SLAM has been DEGRADED for >10s
                                   with healthy GNSS; we are essentially flying
                                   on absolute fixes plus dead reckoning).
        DEGEN SEVERE       → 0.4x
        DEGEN MILD         → 0.7x
        otherwise          → 1.0x

        CRITICAL is handled by the DEGRADED→LOST path above (mission pauses).
        """
        prev_scale = self._speed_scale
        reason = ""
        if self._loc_state == "FALLBACK_GNSS_ONLY":
            self._speed_scale = 0.3
            reason = "FALLBACK_GNSS_ONLY"
        elif self._degen_level == "SEVERE":
            self._speed_scale = 0.4
            reason = "degeneracy=SEVERE"
        elif self._degen_level == "MILD":
            self._speed_scale = 0.7
            reason = "degeneracy=MILD"
        else:
            self._speed_scale = 1.0

        if self._speed_scale != prev_scale and self._state == MissionState.EXECUTING:
            if self._speed_scale < 1.0:
                logger.info("Navigation speed scaled to %.0f%% (%s)",
                            self._speed_scale * 100, reason)
            else:
                logger.info("Navigation speed restored to 100%%")

    def _on_patrol_goals(self, goals: list) -> None:
        if not goals:
            return
        self._patrol_goals.clear()
        self._patrol_loop = False
        for g in goals:
            if isinstance(g, dict):
                self._patrol_goals.append(
                    np.array([g["x"], g["y"], g.get("z", 0.0)])
                )
                if g.get("loop"):
                    self._patrol_loop = True
            elif isinstance(g, (list, tuple)) and len(g) >= 2:
                self._patrol_goals.append(
                    np.array([g[0], g[1], g[2] if len(g) > 2 else 0.0])
                )
        if self._patrol_goals:
            self._patrol_index = 0
            self._goal = self._patrol_goals[0]
            self._replan_count = 0
            self._set_state(MissionState.PATROLLING)
            logger.info(
                "Patrol started: %d goals, loop=%s",
                len(self._patrol_goals), self._patrol_loop,
            )
            self._plan()

    def _on_odom(self, odom: Odometry) -> None:
        self._robot_pos = np.array([
            odom.pose.position.x,
            odom.pose.position.y,
            odom.pose.position.z,
        ])

        if self._paused_for_localization:
            return

        if self._state not in (MissionState.EXECUTING, MissionState.PATROLLING):
            return

        status = self._tracker.update(self._robot_pos)

        if status.event == EV_PATH_COMPLETE:
            if self._state == MissionState.PATROLLING and self._patrol_goals:
                if self._advance_patrol():
                    return
            self._set_state(MissionState.SUCCESS)

        elif status.event == EV_WAYPOINT_REACHED:
            self.adapter_status.publish({
                "event": "waypoint_reached",
                "index": status.wp_index,
                "total": status.wp_total,
            })
            self._publish_waypoint()

        elif status.event == EV_STUCK_WARN:
            self.adapter_status.publish({
                "event": "WARN_STUCK",
                "elapsed": round(time.time() - self._mission_start_time, 1),
            })

        elif status.event == EV_STUCK:
            if self._replan_count < self._max_replan:
                self._replan_count += 1
                # Try backup motion before replanning (back up briefly, then rotate)
                self._execute_recovery_motion()
                logger.warning(
                    "Stuck detected, recovery motion + replan (%d/%d)",
                    self._replan_count, self._max_replan,
                )
                self._plan()
            else:
                self._failure_reason = "stuck after max replans"
                self._set_state(MissionState.STUCK)

        # Mission timeout
        if (self._mission_start_time > 0
                and time.time() - self._mission_start_time > self._mission_timeout):
            self._failure_reason = f"mission timeout ({self._mission_timeout}s)"
            self._set_state(MissionState.FAILED)
            logger.warning("Mission timeout after %.0fs", self._mission_timeout)

    # ── Recovery motion ───────────────────────────────────────────────────

    def _on_traversability(self, msg: dict) -> None:
        """W2-8: update latest terrain class from TerrainModule."""
        if not isinstance(msg, dict):
            return
        klass = msg.get("traversability_class") or msg.get("class")
        if klass:
            self._latest_traversability_class = str(klass)

    # Recovery strategies keyed by terrain class. Each entry specifies the
    # backup, rotate, and optional forward-nudge phases — setting a phase's
    # duration to 0 skips it. The default strategy (1.5s backup + 1.5s rotate)
    # kicks in for "unknown" or any class not listed below.
    _RECOVERY_STRATEGIES: dict[str, dict[str, Any]] = {
        "cliff": {
            "strategy": "rotate_only",
            "backup_speed": 0.0, "backup_duration": 0.0,
            "rotate_speed": 0.3, "rotate_duration": 2.5,
            "forward_speed": 0.0, "forward_duration": 0.0,
        },
        "unsafe_forward": {
            "strategy": "rotate_only",
            "backup_speed": 0.0, "backup_duration": 0.0,
            "rotate_speed": 0.3, "rotate_duration": 2.5,
            "forward_speed": 0.0, "forward_duration": 0.0,
        },
        "narrow": {
            "strategy": "short_backup_rotate",
            "backup_speed": -0.15, "backup_duration": 0.8,
            "rotate_speed": 0.5,   "rotate_duration": 0.8,
            "forward_speed": 0.0,  "forward_duration": 0.0,
        },
        "corridor": {
            "strategy": "short_backup_rotate",
            "backup_speed": -0.15, "backup_duration": 0.8,
            "rotate_speed": 0.5,   "rotate_duration": 0.8,
            "forward_speed": 0.0,  "forward_duration": 0.0,
        },
        "stuck_in_soft": {
            "strategy": "long_backup_nudge",
            "backup_speed": -0.15, "backup_duration": 2.0,
            "rotate_speed": 0.0,   "rotate_duration": 0.0,
            "forward_speed": 0.15, "forward_duration": 0.5,
        },
        "grip_loss": {
            "strategy": "long_backup_nudge",
            "backup_speed": -0.15, "backup_duration": 2.0,
            "rotate_speed": 0.0,   "rotate_duration": 0.0,
            "forward_speed": 0.15, "forward_duration": 0.5,
        },
    }

    _DEFAULT_RECOVERY_STRATEGY: dict[str, Any] = {
        "strategy": "default_backup_rotate",
        "backup_speed": -0.2, "backup_duration": 1.5,
        "rotate_speed": 0.5,  "rotate_duration": 1.5,
        "forward_speed": 0.0, "forward_duration": 0.0,
    }

    def _execute_recovery_motion(self) -> None:
        """W2-8: context-aware stuck recovery.

        Picks a recovery strategy based on the latest terrain class reported
        by TerrainModule. Emits adapter_status {"event":"recovery_started",
        "strategy":..., "reason":...} so operators can see which branch ran.
        """
        klass = getattr(self, "_latest_traversability_class", "unknown")
        strat = self._RECOVERY_STRATEGIES.get(klass, self._DEFAULT_RECOVERY_STRATEGY)

        self.adapter_status.publish({
            "event":    "recovery_started",
            "strategy": strat["strategy"],
            "reason":   klass,
            "ts":       time.time(),
        })
        logger.info(
            "Recovery: strategy=%s, reason=%s", strat["strategy"], klass,
        )

        step_hz = 10

        def _drive(linear_x: float, angular_z: float, duration: float) -> bool:
            """Publish a fixed cmd_vel for `duration` seconds at step_hz."""
            if duration <= 0.0:
                return True
            steps = int(duration * step_hz)
            for _ in range(steps):
                if self._state != MissionState.EXECUTING:
                    return False
                self.recovery_cmd_vel.publish(Twist(
                    linear=Vector3(linear_x, 0.0, 0.0),
                    angular=Vector3(0.0, 0.0, angular_z),
                ))
                time.sleep(1.0 / step_hz)
            return True

        # Backup phase
        if not _drive(strat["backup_speed"], 0.0, strat["backup_duration"]):
            return

        # Rotate phase — alternate direction per replan to avoid dead-lock
        if strat["rotate_duration"] > 0.0 and strat["rotate_speed"] != 0.0:
            direction = 1.0 if self._replan_count % 2 == 1 else -1.0
            if not _drive(0.0, strat["rotate_speed"] * direction, strat["rotate_duration"]):
                return

        # Forward nudge phase (slip/grip terrain)
        if not _drive(strat["forward_speed"], 0.0, strat["forward_duration"]):
            return

        # Stop
        self.recovery_cmd_vel.publish(Twist.zero())

    # ── Planning ──────────────────────────────────────────────────────────

    def _plan(self) -> None:
        if self._goal is None:
            self._set_state(MissionState.FAILED)
            return

        self._set_state(MissionState.PLANNING)
        self._mission_start_time = time.time()
        self._last_costmap_replan_time = time.time()

        path = None
        if not self._planner_svc.is_ready:
            if self._allow_direct_goal_fallback:
                path = self._direct_goal_path(reason="planner backend not ready")
            else:
                self._failure_reason = "planner backend not ready"
                self._set_state(MissionState.FAILED)
                return
        else:
            try:
                path, _ = self._planner_svc.plan(self._robot_pos, self._goal)
            except Exception as exc:
                if self._should_use_direct_goal_fallback(exc):
                    path = self._direct_goal_path(reason=str(exc))
                else:
                    logger.error("Planning failed: %s", exc)
                    self._failure_reason = str(exc)
                    self._set_state(MissionState.FAILED)
                    return

        self._failure_reason = ""
        self._tracker.reset(path, self._robot_pos)
        # Advance past any waypoints the robot is already at (e.g. the start)
        self._tracker.update(self._robot_pos)
        self.global_path.publish(path)
        self._set_state(MissionState.EXECUTING)
        self._publish_waypoint()

    def _should_use_direct_goal_fallback(self, exc: Exception) -> bool:
        if not self._allow_direct_goal_fallback:
            return False
        return not self._planner_svc.has_map

    def _direct_goal_path(self, reason: str = "") -> list[np.ndarray]:
        logger.warning(
            "NavigationModule: using direct-goal fallback waypoint (reason=%s)",
            reason or "planner unavailable",
        )
        self.adapter_status.publish({
            "event": "direct_goal_fallback",
            "reason": reason or "planner unavailable",
            "goal": [float(self._goal[0]), float(self._goal[1]), float(self._goal[2])],
            "ts": time.time(),
        })
        return [self._goal.copy()]

    def _advance_patrol(self) -> bool:
        """Advance to next patrol goal. Returns True if more goals remain."""
        self._patrol_index += 1
        if self._patrol_index >= len(self._patrol_goals):
            if self._patrol_loop:
                self._patrol_index = 0
            else:
                return False
        self._goal = self._patrol_goals[self._patrol_index]
        self._replan_count = 0
        logger.info(
            "Patrol advancing to goal %d/%d",
            self._patrol_index + 1, len(self._patrol_goals),
        )
        self._plan()
        return True

    def _publish_waypoint(self) -> None:
        wp = self._tracker.current_waypoint
        if wp is None:
            return
        pose = PoseStamped(
            pose=Pose(
                position=Vector3(
                    float(wp[0]), float(wp[1]),
                    float(wp[2]) if len(wp) > 2 else 0.0,
                ),
                orientation=Quaternion(0, 0, 0, 1),
            ),
            frame_id="map",
            ts=time.time(),
        )
        self.waypoint.publish(pose)
        if self._ros2_wp_pub is not None:
            try:
                from geometry_msgs.msg import PointStamped
                pt = PointStamped()
                pt.header.frame_id = "map"
                pt.point.x = float(wp[0])
                pt.point.y = float(wp[1])
                pt.point.z = float(wp[2]) if len(wp) > 2 else 0.0
                self._ros2_wp_pub.publish(pt)
            except Exception:
                pass

    # ── Skills (auto-discovered by MCPServerModule) ───────────────────────

    @skill
    def navigate_to(self, x: float, y: float, yaw: float = 0.0) -> str:
        """Navigate to map coordinates.

        Args:
            x: X coordinate in meters (map frame)
            y: Y coordinate in meters (map frame)
            yaw: Heading in radians (default 0)
        """
        q_w = math.cos(yaw / 2)
        q_z = math.sin(yaw / 2)
        self._on_goal(PoseStamped(
            pose=Pose(
                position=Vector3(x, y, 0.0),
                orientation=Quaternion(0.0, 0.0, q_z, q_w),
            ),
            frame_id="map", ts=0.0,
        ))
        return json.dumps({"status": "navigating", "goal": [x, y], "yaw": yaw})

    @skill
    def stop_navigation(self) -> str:
        """Immediately stop all robot motion and cancel current mission."""
        self._on_stop(2)
        return json.dumps({"status": "stopped"})

    @skill
    def cancel_mission(self, reason: str = "user_cancel") -> str:
        """Cancel current navigation mission gracefully."""
        self._on_cancel(reason)
        return json.dumps({"status": "cancelled", "reason": reason})

    @skill
    def get_navigation_status(self) -> str:
        """Get current navigation state, position, and mission progress."""
        pos = {"x": round(float(self._robot_pos[0]), 3),
               "y": round(float(self._robot_pos[1]), 3),
               "yaw": 0.0}
        goal = ({"x": round(float(self._goal[0]), 3),
                 "y": round(float(self._goal[1]), 3)}
                if self._goal is not None else None)
        return json.dumps({
            "state": self._state,
            "position": pos,
            "goal": goal,
            "path_length": self._tracker.path_length,
            "waypoint_index": self._tracker.wp_index,
            "replan_count": self._replan_count,
            "failure_reason": self._failure_reason,
        })

    @skill
    def start_patrol(self, waypoints_json: str) -> str:
        """Start patrol mode — navigate a list of waypoints sequentially.

        Args:
            waypoints_json: JSON array of [x, y] pairs,
                            e.g. "[[1.0, 2.0], [3.0, 4.0]]"
        """
        try:
            goals = json.loads(waypoints_json)
            self._on_patrol_goals(goals)
            return json.dumps({"status": "patrolling", "goals": len(goals)})
        except Exception as e:
            return json.dumps({"error": str(e)})

    # ── Lifecycle ──────────────────────────────────────────────────────────

    def stop(self) -> None:
        if self._ros2_node is not None:
            self._ros2_node.destroy_node()
            self._ros2_node = None

    def health(self) -> dict[str, Any]:
        info = super().port_summary()
        info["navigation"] = {
            "planner": self._planner_svc._planner_name,
            "state": self._state,
            "wp_index": self._tracker.wp_index,
            "wp_total": self._tracker.path_length,
            "replan_count": self._replan_count,
            "failure_reason": self._failure_reason,
            "patrol_index": self._patrol_index if self._patrol_goals else -1,
            "patrol_total": len(self._patrol_goals),
        }
        return info
