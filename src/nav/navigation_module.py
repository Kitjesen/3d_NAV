"""NavigationModule — unified path planning + tracking + mission FSM.

Replaces GlobalPlannerModule, PathAdapterModule, MissionArcModule with one Module.

Features:
  - Global path planning (pluggable: astar, pct via Registry)
  - Waypoint tracking with arrival detection
  - Stuck detection + auto-replan with max retry
  - Patrol mode (multi-point waypoint list)
  - Cancel support
  - Planning/mission timeout

Ports:
  In:  goal_pose, odometry, instruction, stop_signal, patrol_goals, cancel
  Out: waypoint, global_path, planner_status, mission_status, adapter_status

Usage::

    bp.add(NavigationModule, planner="astar", tomogram="building.pickle")
    bp.add(NavigationModule, planner="pct")  # S100P with ele_planner.so
"""

from __future__ import annotations

import json
import logging
import math
import os
import time
from typing import Any, Dict, List, Optional

import numpy as np

from core.module import Module, skill
from core.stream import In, Out
from core.msgs.geometry import PoseStamped, Pose, Vector3, Quaternion, Twist
from core.msgs.nav import Odometry, Path
from core.registry import register

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Mission states
# ---------------------------------------------------------------------------

class MissionState:
    IDLE = "IDLE"
    PLANNING = "PLANNING"
    EXECUTING = "EXECUTING"
    SUCCESS = "SUCCESS"
    FAILED = "FAILED"
    STUCK = "STUCK"
    CANCELLED = "CANCELLED"
    PATROLLING = "PATROLLING"


# ---------------------------------------------------------------------------
# NavigationModule
# ---------------------------------------------------------------------------

@register("navigation", "default", description="Unified navigation module")
class NavigationModule(Module, layer=5):
    """Unified navigation: plan → track → FSM in one Module.

    Combines GlobalPlanner + PathAdapter + MissionArc functionality.
    Each stage has a pluggable strategy.
    """

    # -- Inputs --
    goal_pose: In[PoseStamped]
    odometry: In[Odometry]
    costmap: In[dict]
    instruction: In[str]
    stop_signal: In[int]
    patrol_goals: In[list]
    cancel: In[str]

    # -- Outputs --
    waypoint: Out[PoseStamped]
    global_path: Out[list]
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
        **kw,
    ):
        super().__init__(**kw)
        self._planner_name = planner
        self._tomogram = tomogram
        self._obstacle_thr = obstacle_thr
        self._wp_threshold = waypoint_threshold
        self._stuck_timeout = stuck_timeout
        self._stuck_dist = stuck_dist_thre
        self._max_replan = max_replan_count
        self._downsample_dist = downsample_dist

        # Internal state
        self._planner_backend = None
        self._state = MissionState.IDLE
        self._robot_pos = np.zeros(3)
        self._path: List[np.ndarray] = []
        self._wp_index = 0
        self._replan_count = 0
        self._last_progress_time = time.time()
        self._last_progress_pos = np.zeros(3)
        self._goal: Optional[np.ndarray] = None
        self._failure_reason = ""

        # Patrol state
        self._patrol_goals: List[np.ndarray] = []
        self._patrol_index = 0
        self._patrol_loop = False

        # Timeouts
        self._planning_timeout = kw.get("planning_timeout", 30.0)
        self._mission_timeout = kw.get("mission_timeout", 300.0)
        self._mission_start_time = 0.0
        self._planning_start_time = 0.0

    def setup(self):
        self._planner_backend = self._create_planner()
        self.goal_pose.subscribe(self._on_goal)
        self.odometry.subscribe(self._on_odom)
        self.costmap.subscribe(self._on_costmap)
        self.instruction.subscribe(self._on_instruction)
        self.stop_signal.subscribe(self._on_stop)
        self.patrol_goals.subscribe(self._on_patrol_goals)
        self.cancel.subscribe(self._on_cancel)

        # Optional ROS2 bridge for C++ localPlanner
        self._ros2_node = None
        self._ros2_wp_pub = None
        try:
            import rclpy
            from rclpy.node import Node
            from geometry_msgs.msg import PointStamped
            from rclpy.qos import QoSProfile, ReliabilityPolicy
            if not rclpy.ok():
                rclpy.init(args=[])
            self._ros2_node = rclpy.create_node('nav_waypoint_bridge')
            qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
            self._ros2_wp_pub = self._ros2_node.create_publisher(PointStamped, '/nav/way_point', qos)
            logger.info("NavigationModule: ROS2 waypoint bridge enabled -> /nav/way_point")
        except (ImportError, Exception) as e:
            logger.debug("NavigationModule: ROS2 not available, waypoint bridge disabled: %s", e)

        self._set_state(MissionState.IDLE)

    def _create_planner(self):
        """Factory: select planning algorithm via registry (zero direct import)."""
        from core.registry import get
        name = self._planner_name.lower()
        try:
            # Ensure backends are registered (lazy import triggers @register)
            import global_planning.pct_adapters.src.global_planner_module  # noqa: F401
        except ImportError:
            pass
        try:
            BackendCls = get("planner_backend", name)
        except KeyError:
            from core.registry import list_plugins
            available = list_plugins("planner_backend")
            raise ValueError(f"Unknown planner: '{name}'. Available: {available}")

        # Resolve tomogram path: explicit > active map > empty
        tomogram_path = self._tomogram
        if not tomogram_path or not os.path.exists(tomogram_path):
            # Try the active map directory
            active_tomo = os.path.join(
                os.environ.get("NAV_MAP_DIR", os.path.expanduser("~/data/nova/maps")),
                "active", "tomogram.pickle")
            if os.path.exists(active_tomo):
                tomogram_path = active_tomo
                logger.info("NavigationModule: using active map tomogram: %s", tomogram_path)

        return BackendCls(tomogram_path, self._obstacle_thr)

    # -- State machine -------------------------------------------------------

    def _set_state(self, state: str):
        self._state = state
        self.planner_status.publish(state)
        self.mission_status.publish({
            "state": state,
            "replan_count": self._replan_count,
            "wp_index": self._wp_index,
            "wp_total": len(self._path),
            "ts": time.time(),
        })

    # -- Input handlers ------------------------------------------------------

    def _on_goal(self, goal: PoseStamped):
        """New navigation goal → plan path."""
        self._goal = np.array([goal.pose.position.x, goal.pose.position.y,
                               goal.pose.position.z])
        self._replan_count = 0
        self._plan()

    def _on_instruction(self, text: str):
        """Natural language instruction — not handled here, just log."""
        logger.info("NavigationModule received instruction: %s", text[:50])

    def _on_stop(self, level: int):
        """Stop signal → abort mission."""
        if level >= 1:
            self._path.clear()
            self._wp_index = 0
            self._set_state(MissionState.IDLE)

    def _on_costmap(self, data: dict):
        """Live costmap update from terrain analysis or mapper.

        Updates planner grid and triggers replan if currently executing.

        Args:
            data: {"grid": np.ndarray, "resolution": float, "origin": [x, y]}
        """
        if self._planner_backend and hasattr(self._planner_backend, 'update_map'):
            grid = data.get("grid")
            if grid is not None:
                self._planner_backend.update_map(
                    grid,
                    resolution=data.get("resolution", 0.2),
                    origin=data.get("origin"),
                )
                # Replan if active goal + enough time since last plan (3s cooldown)
                # Also replan on FAILED — initial plan may have failed before map was ready
                if (self._state in (MissionState.EXECUTING, MissionState.PATROLLING,
                                    MissionState.FAILED)
                        and self._goal is not None
                        and time.time() - self._mission_start_time > 3.0):
                    self._mission_start_time = time.time()  # reset cooldown
                    self._plan()

    def _on_cancel(self, msg: str):
        """Cancel current mission."""
        if self._state in (MissionState.IDLE, MissionState.CANCELLED):
            return
        self._path.clear()
        self._wp_index = 0
        self._patrol_goals.clear()
        self._patrol_index = 0
        self._failure_reason = f"cancelled: {msg}" if msg else "cancelled"
        self._set_state(MissionState.CANCELLED)
        logger.info("Mission cancelled: %s", msg)

    def _on_patrol_goals(self, goals: list):
        """Patrol mode — navigate to a list of goals sequentially.

        Args:
            goals: list of [x, y] or [x, y, z] coordinates, or
                   list of dicts with {"x": ..., "y": ..., "loop": bool}
        """
        if not goals:
            return
        self._patrol_goals.clear()
        self._patrol_loop = False

        for g in goals:
            if isinstance(g, dict):
                self._patrol_goals.append(np.array([g["x"], g["y"], g.get("z", 0.0)]))
                if g.get("loop"):
                    self._patrol_loop = True
            elif isinstance(g, (list, tuple)) and len(g) >= 2:
                self._patrol_goals.append(np.array([g[0], g[1], g[2] if len(g) > 2 else 0.0]))

        if self._patrol_goals:
            self._patrol_index = 0
            self._goal = self._patrol_goals[0]
            self._replan_count = 0
            self._set_state(MissionState.PATROLLING)
            logger.info("Patrol started: %d goals, loop=%s", len(self._patrol_goals), self._patrol_loop)
            self._plan()

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
        logger.info("Patrol advancing to goal %d/%d", self._patrol_index + 1, len(self._patrol_goals))
        self._plan()
        return True

    def _on_odom(self, odom: Odometry):
        """Odometry update → advance waypoint tracking + stuck detection."""
        self._robot_pos = np.array([
            odom.pose.position.x,
            odom.pose.position.y,
            odom.pose.position.z,
        ])

        if self._state not in (MissionState.EXECUTING, MissionState.PATROLLING) or not self._path:
            return

        # Waypoint arrival check
        if self._wp_index < len(self._path):
            wp = self._path[self._wp_index]
            dist = np.linalg.norm(self._robot_pos[:2] - wp[:2])

            if dist < self._wp_threshold:
                self._wp_index += 1
                self._last_progress_time = time.time()
                self._last_progress_pos = self._robot_pos.copy()

                self.adapter_status.publish({
                    "event": "waypoint_reached",
                    "index": self._wp_index,
                    "total": len(self._path),
                })

                if self._wp_index >= len(self._path):
                    # Path completed — check if patrol has more goals
                    if self._state == MissionState.PATROLLING and self._patrol_goals:
                        if self._advance_patrol():
                            return  # next patrol goal started
                    self._set_state(MissionState.SUCCESS)
                    return

                # Publish next waypoint
                self._publish_waypoint()

        # Stuck detection
        now = time.time()
        moved = np.linalg.norm(self._robot_pos[:2] - self._last_progress_pos[:2])
        elapsed = now - self._last_progress_time

        if elapsed > self._stuck_timeout * 0.5 and moved < self._stuck_dist:
            self.adapter_status.publish({"event": "WARN_STUCK",
                                         "elapsed": round(elapsed, 1)})
        if elapsed > self._stuck_timeout and moved < self._stuck_dist:
            if self._replan_count < self._max_replan:
                self._replan_count += 1
                logger.warning("Stuck detected, replanning (%d/%d)",
                               self._replan_count, self._max_replan)
                self._plan()
            else:
                self._failure_reason = "stuck after max replans"
                self._set_state(MissionState.STUCK)

        # Mission timeout
        if self._mission_start_time > 0:
            mission_elapsed = now - self._mission_start_time
            if mission_elapsed > self._mission_timeout:
                self._failure_reason = f"mission timeout ({self._mission_timeout}s)"
                self._set_state(MissionState.FAILED)
                logger.warning("Mission timeout after %.0fs", mission_elapsed)

    # -- Planning ------------------------------------------------------------

    def _plan(self):
        """Run global planner and start tracking."""
        if self._goal is None or self._planner_backend is None:
            self._set_state(MissionState.FAILED)
            return

        self._set_state(MissionState.PLANNING)
        self._mission_start_time = time.time()
        t0 = time.time()

        try:
            raw_path = self._planner_backend.plan(self._robot_pos, self._goal)
        except Exception:
            logger.exception("Planning failed")
            self._set_state(MissionState.FAILED)
            return

        if not raw_path:
            self._set_state(MissionState.FAILED)
            return

        # Downsample
        self._path = self._downsample(raw_path)
        self._wp_index = 0
        self._last_progress_time = time.time()
        self._last_progress_pos = self._robot_pos.copy()

        plan_ms = (time.time() - t0) * 1000
        logger.info("Planned %d waypoints in %.1fms (planner=%s)",
                     len(self._path), plan_ms, self._planner_name)

        self.global_path.publish(self._path)
        self._set_state(MissionState.EXECUTING)
        self._publish_waypoint()

    def _downsample(self, path: list) -> List[np.ndarray]:
        """Downsample path by minimum 3D distance."""
        if not path:
            return []
        result = [np.array(path[0][:3])]
        for p in path[1:]:
            pt = np.array(p[:3]) if len(p) >= 3 else np.array([p[0], p[1], 0.0])
            if np.linalg.norm(pt - result[-1]) >= self._downsample_dist:
                result.append(pt)
        # Always include goal
        goal = np.array(path[-1][:3]) if len(path[-1]) >= 3 else np.array([path[-1][0], path[-1][1], 0.0])
        if np.linalg.norm(goal - result[-1]) > 0.1:
            result.append(goal)
        return result

    def _publish_waypoint(self):
        """Publish current waypoint as PoseStamped."""
        if self._wp_index >= len(self._path):
            return
        wp = self._path[self._wp_index]
        pose = PoseStamped(
            pose=Pose(position=Vector3(float(wp[0]), float(wp[1]),
                                       float(wp[2]) if len(wp) > 2 else 0.0),
                      orientation=Quaternion(0, 0, 0, 1)),
            frame_id="map", ts=time.time(),
        )
        self.waypoint.publish(pose)
        if self._ros2_wp_pub is not None:
            try:
                from geometry_msgs.msg import PointStamped
                pt = PointStamped()
                pt.header.frame_id = 'map'
                pt.point.x = float(wp[0])
                pt.point.y = float(wp[1])
                pt.point.z = float(wp[2]) if len(wp) > 2 else 0.0
                self._ros2_wp_pub.publish(pt)
            except Exception:
                pass

    # -- AI-callable skills (auto-discovered by MCPServerModule) ---------------

    @skill
    def navigate_to(self, x: float, y: float, yaw: float = 0.0) -> str:
        """Navigate to map coordinates.

        Args:
            x: X coordinate in meters (map frame)
            y: Y coordinate in meters (map frame)
            yaw: Heading in radians (default 0)

        Returns:
            Status string
        """
        q_w = math.cos(yaw / 2)
        q_z = math.sin(yaw / 2)
        pose = PoseStamped(
            pose=Pose(
                position=Vector3(x, y, 0.0),
                orientation=Quaternion(0.0, 0.0, q_z, q_w),
            ),
            frame_id="map",
            ts=0.0,
        )
        self._on_goal(pose)
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
        pos = None
        if self._robot_pos is not None:
            pos = {"x": round(float(self._robot_pos[0]), 3),
                   "y": round(float(self._robot_pos[1]), 3),
                   "yaw": 0.0}
        goal = None
        if self._goal is not None:
            goal = {"x": round(float(self._goal[0]), 3),
                    "y": round(float(self._goal[1]), 3)}
        return json.dumps({
            "state": self._state,
            "position": pos,
            "goal": goal,
            "path_length": len(self._path),
            "waypoint_index": self._wp_index,
            "replan_count": self._replan_count,
            "failure_reason": self._failure_reason,
        })

    @skill
    def start_patrol(self, waypoints_json: str) -> str:
        """Start patrol mode — navigate a list of waypoints sequentially.

        Args:
            waypoints_json: JSON array of [x, y] pairs, e.g. "[[1.0, 2.0], [3.0, 4.0]]"

        Returns:
            Status string
        """
        try:
            goals = json.loads(waypoints_json)
            self._on_patrol_goals(goals)
            return json.dumps({"status": "patrolling", "goals": len(goals)})
        except Exception as e:
            return json.dumps({"error": str(e)})

    def stop(self):
        if self._ros2_node is not None:
            self._ros2_node.destroy_node()
            self._ros2_node = None

    # -- Health --------------------------------------------------------------

    def health(self) -> Dict[str, Any]:
        info = super().port_summary()
        info["navigation"] = {
            "planner": self._planner_name,
            "state": self._state,
            "wp_index": self._wp_index,
            "wp_total": len(self._path),
            "replan_count": self._replan_count,
            "failure_reason": self._failure_reason,
            "patrol_index": self._patrol_index if self._patrol_goals else -1,
            "patrol_total": len(self._patrol_goals),
        }
        return info
