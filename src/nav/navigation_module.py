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
from core.stream import In, Out
from core.msgs.geometry import PoseStamped, Pose, Vector3, Quaternion
from core.msgs.nav import Odometry
from core.registry import register
from nav.global_planner_service import GlobalPlannerService
from nav.waypoint_tracker import (
    WaypointTracker,
    EV_WAYPOINT_REACHED,
    EV_PATH_COMPLETE,
    EV_STUCK_WARN,
    EV_STUCK,
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
    localization_status: In[dict]

    # -- Outputs --
    waypoint:       Out[PoseStamped]
    global_path:    Out[list]
    planner_status: Out[str]
    mission_status: Out[dict]
    adapter_status: Out[dict]

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
        allow_direct_goal_fallback: bool = True,
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
        self._goal: Optional[np.ndarray] = None
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
        self._pre_pause_state: Optional[str] = None
        self._planning_timeout = kw.get("planning_timeout", 30.0)
        self._speed_scale: float = 1.0  # degeneracy-based speed multiplier

        # Cooldown for costmap-triggered replanning (3s minimum between replans)
        self._last_costmap_replan_time = 0.0

        # Patrol FSM state
        self._patrol_goals: List[np.ndarray] = []
        self._patrol_index = 0
        self._patrol_loop = False

        # Optional ROS2 bridge (set in setup)
        self._ros2_node = None
        self._ros2_wp_pub = None

    def setup(self) -> None:
        self._planner_svc.setup()

        self.goal_pose.subscribe(self._on_goal)
        self.odometry.subscribe(self._on_odom)
        self.costmap.subscribe(self._on_costmap)
        self.instruction.subscribe(self._on_instruction)
        self.stop_signal.subscribe(self._on_stop)
        self.patrol_goals.subscribe(self._on_patrol_goals)
        self.cancel.subscribe(self._on_cancel)
        self.localization_status.subscribe(self._on_localization_status)

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
        """Scale navigation speed based on SLAM degeneracy level.

        NONE     → 1.0x (full speed)
        MILD     → 0.7x (slight reduction)
        SEVERE   → 0.4x (cautious)
        CRITICAL → pause (handled by DEGRADED→LOST path above)
        """
        prev_scale = self._speed_scale
        if self._degen_level == "SEVERE":
            self._speed_scale = 0.4
        elif self._degen_level == "MILD":
            self._speed_scale = 0.7
        else:
            self._speed_scale = 1.0

        if self._speed_scale != prev_scale and self._state == MissionState.EXECUTING:
            if self._speed_scale < 1.0:
                logger.info("Navigation speed scaled to %.0f%% (degeneracy: %s)",
                            self._speed_scale * 100, self._degen_level)
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
                logger.warning(
                    "Stuck detected, replanning (%d/%d)",
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
        self.global_path.publish(path)
        self._set_state(MissionState.EXECUTING)
        self._publish_waypoint()

    def _should_use_direct_goal_fallback(self, exc: Exception) -> bool:
        if not self._allow_direct_goal_fallback:
            return False
        return not self._planner_svc.has_map

    def _direct_goal_path(self, reason: str = "") -> List[np.ndarray]:
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

    def health(self) -> Dict[str, Any]:
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
