# DEPRECATED: Use NavigationModule instead (from nav.navigation_module)
#!/usr/bin/env python3
"""
PathAdapterModule -- PCT Path Adapter as a core Module (no rclpy).

Converts the ROS2 PCTPathAdapter node into a pure-Python Module that:
  1. Receives global_path (Path) + odometry (Odometry) via In ports
  2. Tracks waypoint progression with closed-loop arrival checking
  3. Publishes next waypoint (PoseStamped) via Out port
  4. Publishes adapter status events (dict) via Out port
  5. Detects stuck conditions (WARN_STUCK / STUCK)

All algorithm logic preserved from pct_path_adapter.py:
  - Path downsampling by 3D distance
  - Waypoint arrival threshold check
  - max_index_jump protection
  - max_first_waypoint_dist validation
  - Stuck detection with progressive warn/stuck states
  - goal_reached dedup
"""

from __future__ import annotations

import logging
import math
import time
from typing import Any, Dict, List, Optional

import sys as _sys
import os as _os

_src_dir = _os.path.abspath(
    _os.path.join(_os.path.dirname(__file__), "..", "..", "..")
)
if _src_dir not in _sys.path:
    _sys.path.insert(0, _src_dir)

from core import Module, In, Out
from core.msgs.nav import Odometry, Path
from core.msgs.geometry import Pose, PoseStamped, Vector3

logger = logging.getLogger(__name__)


class PathAdapterModule(Module, layer=5):
    """PCT Path to Waypoint Adapter -- core Module version.

    Bridges the Global Planner and Local Planner via closed-loop waypoint
    tracking.  Instead of blindly publishing all path points, it monitors
    robot progress and only advances when the robot reaches the current
    waypoint (within threshold).

    Ports:
        In:
            global_path  (Path)      -- dense global path from planner
            odometry     (Odometry)  -- robot pose for arrival checking

        Out:
            waypoint       (PoseStamped) -- current target for local planner
            adapter_status (dict)        -- status events (JSON-serializable)
    """

    # -- Ports (auto-scanned by Module.__init__) --
    global_path: In[Path]
    odometry: In[Odometry]

    waypoint: Out[PoseStamped]
    adapter_status: Out[dict]

    def __init__(
        self,
        waypoint_distance: float = 0.5,
        arrival_threshold: float = 0.5,
        lookahead_dist: float = 1.0,
        max_index_jump: int = 3,
        max_first_waypoint_dist: float = 10.0,
        stuck_timeout: float = 10.0,
        stuck_dist_thre: float = 0.15,
        **config: Any,
    ) -> None:
        super().__init__(**config)

        # Parameters
        self.waypoint_distance = waypoint_distance
        self.arrival_threshold = arrival_threshold
        self.lookahead_dist = lookahead_dist
        self.max_index_jump = max_index_jump
        self.max_first_waypoint_dist = max_first_waypoint_dist
        self.stuck_timeout = stuck_timeout
        self.stuck_dist_thre = stuck_dist_thre

        # State
        self._current_path: List[PoseStamped] = []
        self._current_waypoint_idx: int = 0
        self._robot_pos: Optional[Vector3] = None
        self._path_received: bool = False
        self._goal_reached_reported: bool = False

        # Stuck detection state
        self._last_progress_time: float = 0.0
        self._last_progress_pos: Optional[Vector3] = None
        self._warn_stuck_sent: bool = False
        self._stuck_sent: bool = False

        # Recovery tracking
        self._recovery_frames: int = 0
        self._RECOVERY_CONFIRM_FRAMES: int = 3
        self._RECOVERY_SPEED_THRE: float = 0.05

    def setup(self) -> None:
        """Register input subscriptions."""
        self.global_path.subscribe(self._on_path)
        self.odometry.subscribe(self._on_odom)

    # ================================================================
    # Input handlers
    # ================================================================

    def _on_odom(self, msg: Odometry) -> None:
        """Update robot position from odometry."""
        x, y = msg.x, msg.y
        if not (math.isfinite(x) and math.isfinite(y)):
            return
        self._robot_pos = Vector3(x, y, msg.z)

    def _on_path(self, msg: Path) -> None:
        """Receive and process new global path."""
        if len(msg) == 0:
            return

        logger.info("Received new Global Path with %d points", len(msg))

        self._current_path = self._downsample_path(msg)
        self._current_waypoint_idx = 0
        self._path_received = True
        self._goal_reached_reported = False
        self._warn_stuck_sent = False
        self._stuck_sent = False
        self._last_progress_time = time.monotonic()
        self._last_progress_pos = (
            Vector3(self._robot_pos.x, self._robot_pos.y, self._robot_pos.z)
            if self._robot_pos else None
        )

        # Validate first waypoint distance
        if self._robot_pos is not None and self._current_path:
            first_wp = self._current_path[0]
            dist = self._get_distance_2d(
                self._robot_pos, Vector3(first_wp.x, first_wp.y, first_wp.z)
            )
            if dist > self.max_first_waypoint_dist:
                logger.warning(
                    "First waypoint too far (%.1fm > %.1fm), rejecting path",
                    dist,
                    self.max_first_waypoint_dist,
                )
                self._current_path = []
                self._path_received = False
                self._publish_status(
                    "failed", 0, 0, reason="first_waypoint_too_far"
                )
                return

        total = len(self._current_path)
        logger.info("Path processed into %d waypoints", total)
        self._publish_status("path_received", 0, total)

    # ================================================================
    # Core algorithm
    # ================================================================

    def tick(self) -> None:
        """Main control loop -- call at ~10Hz.

        Checks if robot reached current waypoint, advances index,
        detects stuck conditions, publishes next waypoint.
        """
        if (
            not self._path_received
            or not self._current_path
            or self._robot_pos is None
        ):
            return

        try:
            self._control_loop_inner()
        except Exception as e:
            logger.error("PathAdapterModule tick error: %s", e)

    def _control_loop_inner(self) -> None:
        """Actual control logic, exceptions caught by tick()."""
        target_pose = self._current_path[self._current_waypoint_idx]
        target_pos = Vector3(target_pose.x, target_pose.y, target_pose.z)

        dist_to_target = self._get_distance_2d(self._robot_pos, target_pos)
        total = len(self._current_path)

        # Check arrival
        if dist_to_target < self.arrival_threshold:
            if self._current_waypoint_idx < total - 1:
                old_idx = self._current_waypoint_idx
                new_idx = old_idx + 1

                # max_index_jump protection
                if new_idx - old_idx > self.max_index_jump:
                    logger.warning(
                        "Index jump %d > max %d, clamping",
                        new_idx - old_idx,
                        self.max_index_jump,
                    )
                    new_idx = old_idx + self.max_index_jump

                logger.info(
                    "Reached Waypoint %d. Proceeding to next.", old_idx
                )
                self._publish_status("waypoint_reached", old_idx, total)
                self._current_waypoint_idx = new_idx
                self._reset_stuck_timer()
                return  # Give one cycle to update
            else:
                if not self._goal_reached_reported:
                    logger.info("Goal Reached!")
                    self._publish_status(
                        "goal_reached", self._current_waypoint_idx, total
                    )
                    self._goal_reached_reported = True

        # Stuck detection
        self._check_stuck()

        # Publish current target waypoint
        wp = PoseStamped(
            pose=target_pose.pose,
            ts=time.time(),
            frame_id=target_pose.frame_id,
        )
        self.waypoint.publish(wp)

    # ================================================================
    # Stuck detection
    # ================================================================

    def _check_stuck(self) -> None:
        """Progressive stuck detection: half-timeout -> WARN_STUCK, full -> STUCK."""
        if self._robot_pos is None or self._goal_reached_reported:
            return

        now = time.monotonic()
        elapsed = now - self._last_progress_time

        # Check if robot has moved significantly
        if self._last_progress_pos is not None:
            dist_moved = self._get_distance_2d(
                self._robot_pos, self._last_progress_pos
            )
            if dist_moved > self.stuck_dist_thre:
                # Robot is moving, reset
                self._reset_stuck_timer()
                return

        # Half timeout -> WARN_STUCK
        half_timeout = self.stuck_timeout / 2.0
        if elapsed > half_timeout and not self._warn_stuck_sent:
            self._warn_stuck_sent = True
            total = len(self._current_path)
            logger.warning(
                "WARN_STUCK: no progress for %.1fs (half of %.1fs)",
                elapsed,
                self.stuck_timeout,
            )
            self._publish_status(
                "warn_stuck", self._current_waypoint_idx, total
            )

        # Full timeout -> STUCK
        if elapsed > self.stuck_timeout and not self._stuck_sent:
            self._stuck_sent = True
            total = len(self._current_path)
            logger.error("STUCK: no progress for %.1fs", elapsed)
            self._publish_status(
                "stuck", self._current_waypoint_idx, total
            )

    def _reset_stuck_timer(self) -> None:
        """Reset stuck detection state on progress."""
        self._last_progress_time = time.monotonic()
        self._last_progress_pos = (
            Vector3(self._robot_pos.x, self._robot_pos.y, self._robot_pos.z)
            if self._robot_pos
            else None
        )
        self._warn_stuck_sent = False
        self._stuck_sent = False
        self._recovery_frames = 0

    # ================================================================
    # Path processing
    # ================================================================

    def _downsample_path(self, path: Path) -> List[PoseStamped]:
        """Downsample dense path into sparser waypoints by 3D distance."""
        if len(path) == 0:
            return []

        poses = list(path.poses)
        downsampled = [poses[0]]
        last = poses[0]

        for pose in poses[1:]:
            dx = pose.x - last.x
            dy = pose.y - last.y
            dz = pose.z - last.z
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)
            if dist >= self.waypoint_distance:
                downsampled.append(pose)
                last = pose

        # Always include the final goal
        if downsampled[-1] is not poses[-1]:
            downsampled.append(poses[-1])

        return downsampled

    # ================================================================
    # Utilities
    # ================================================================

    @staticmethod
    def _get_distance_2d(p1: Vector3, p2: Vector3) -> float:
        """2D Euclidean distance."""
        return math.hypot(p1.x - p2.x, p1.y - p2.y)

    def _publish_status(
        self, event: str, index: int, total: int, **extra: Any
    ) -> None:
        """Publish adapter status event."""
        payload: Dict[str, Any] = {
            "event": event,
            "index": index,
            "total": total,
        }
        payload.update(extra)
        self.adapter_status.publish(payload)

    # ================================================================
    # Properties (read-only inspection)
    # ================================================================

    @property
    def current_waypoint_idx(self) -> int:
        return self._current_waypoint_idx

    @property
    def path_length(self) -> int:
        return len(self._current_path)

    @property
    def goal_reached(self) -> bool:
        return self._goal_reached_reported

    @property
    def robot_position(self) -> Optional[Vector3]:
        return self._robot_pos

