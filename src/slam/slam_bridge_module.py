"""SlamBridgeModule — bridges DDS SLAM outputs into the Python Module pipeline.

Subscribes to DDS topics (published by C++ SLAM nodes):
  /nav/map_cloud       — PointCloud2 from SLAM
  /nav/odometry        — Odometry from SLAM

Uses lightweight cyclonedds (no rclpy dependency). Falls back to rclpy if needed.

Usage::

    bp.add(SlamBridgeModule)
    bp.add(SlamBridgeModule, cloud_topic="/my/cloud")
"""

from __future__ import annotations

import logging
import math
import threading
import time as _time
from typing import Any, Dict, Optional

import numpy as np

from core.module import Module, skill
from core.msgs.geometry import Pose, Quaternion, Vector3
from core.msgs.gnss import GnssFixType, GnssOdom
from core.msgs.nav import Odometry
from core.msgs.sensor import PointCloud2
from core.registry import register
from core.stream import In, Out

logger = logging.getLogger(__name__)

# Localization health states
LOC_UNINIT = "UNINIT"        # No data received yet
LOC_TRACKING = "TRACKING"    # Receiving data, quality OK
LOC_DEGRADED = "DEGRADED"    # Receiving data, quality poor (cloud timeout or degeneracy)
# DEGRADED for too long with healthy GNSS available → flag fallback so downstream
# consumers (NavigationModule) can run cautiously while we still refine the
# state. The actual GNSS+IMU dead-reckoning takeover lands in S2.5 once the
# fault-injection harness exists; this state is the wiring half.
LOC_FALLBACK_GNSS_ONLY = "FALLBACK_GNSS_ONLY"
LOC_LOST = "LOST"            # No odometry for > timeout
LOC_DIVERGED = "DIVERGED"    # Odometry values physically impossible → auto-relocalize

# Degeneracy severity levels (from SLAM quality metrics)
DEGEN_NONE = "NONE"              # Normal operation
DEGEN_MILD = "MILD"              # Feature count low but usable
DEGEN_SEVERE = "SEVERE"          # Severe feature loss, high covariance
DEGEN_CRITICAL = "CRITICAL"      # Complete degeneracy (corridor/void)


@register("slam_bridge", "default", description="DDS SLAM → Python Module bridge")
class SlamBridgeModule(Module, layer=1):
    """Bridges SLAM point cloud and odometry into Module ports via DDS.

    Tries cyclonedds first (lightweight), falls back to rclpy.
    On systems without either, starts silently in stub mode.
    """

    map_cloud:             Out[PointCloud2]
    saved_map:             Out[PointCloud2]  # refined static map from localizer (map frame)
    odometry:              Out[Odometry]
    localization_quality:  Out[float]
    alive:                 Out[bool]
    localization_status:   Out[dict]
    gnss_fusion_health:    Out[dict]
    map_odom_tf:           Out[dict]  # {tx,ty,tz,qx,qy,qz,qw,valid} — localizer-emitted map→odom

    # Visual odometry input for selective DOF fusion during degeneracy
    visual_odom: In[Odometry]

    # GNSS ENU odometry for global drift anchoring (auto-wired from GnssModule)
    gnss_odom: In[GnssOdom]

    def __init__(
        self,
        cloud_topic: str = "/nav/map_cloud",
        odom_topic: str = "/nav/odometry",
        quality_topic: str = "/localization_quality",
        registered_cloud_topic: str = "/nav/registered_cloud",
        saved_map_topic: str = "/nav/saved_map_cloud",
        **kw,
    ):
        super().__init__(**kw)
        self._cloud_topic = cloud_topic
        self._odom_topic = odom_topic
        self._quality_topic = quality_topic
        self._registered_cloud_topic = registered_cloud_topic
        self._saved_map_topic = saved_map_topic
        self._reader = None
        self._rclpy_node = None  # fallback
        self._odom_recv_ts: list = []  # raw receive timestamps for true Hz

        # Localization health watchdog
        self._last_odom_time: float = 0.0
        self._last_cloud_time: float = 0.0
        self._loc_state: str = LOC_UNINIT
        self._odom_timeout: float = kw.get("odom_timeout", 2.0)
        self._cloud_timeout: float = kw.get("cloud_timeout", 5.0)
        self._watchdog_hz: float = kw.get("watchdog_hz", 2.0)
        self._shutdown_event = threading.Event()
        self._watchdog_thread: threading.Thread | None = None

        # Auto-recovery
        self._reconnect_timeout: float = kw.get("reconnect_timeout", 10.0)
        self._max_reconnects: int = kw.get("max_reconnects", 10)
        self._reconnect_count: int = 0

        # Drift guard — detect odometry divergence and auto-relocalize
        self._drift_max_speed: float = kw.get("drift_max_speed", 5.0)       # m/s, quad max ~3
        self._drift_max_jump: float = kw.get("drift_max_jump", 2.0)         # m between frames
        self._drift_max_pos: float = kw.get("drift_max_pos", 500.0)         # m from origin
        self._drift_confirm_frames: int = kw.get("drift_confirm_frames", 5) # consecutive bad
        self._drift_bad_count: int = 0
        self._drift_last_good_pos: np.ndarray | None = None                 # for relocalize
        self._drift_last_good_yaw: float = 0.0
        self._drift_relocalize_cooldown: float = kw.get("drift_relocalize_cooldown", 30.0)
        self._drift_last_relocalize: float = 0.0
        self._prev_odom_pos: np.ndarray | None = None

        # SLAM degeneracy detection
        self._degeneracy_topic: str = kw.get("degeneracy_topic", "/slam/degeneracy")
        self._degeneracy_detail_topic: str = kw.get(
            "degeneracy_detail_topic", "/slam/degeneracy_detail")
        self._degen_level: str = DEGEN_NONE
        self._icp_fitness: float = 0.0
        self._effective_ratio: float = 1.0  # effective features / total features
        self._eigenvalue_ratio: float = 0.0  # condition number (max_eig / min_eig)
        self._condition_number: float = 0.0
        self._degenerate_dof_count: int = 0
        self._eigenvalues: np.ndarray | None = None  # 6-DOF eigenvalues
        self._dof_mask: np.ndarray | None = None  # 1.0=constrained, 0.0=degenerate
        self._max_pos_cov: float = 0.0        # max position covariance from Odometry
        # IEKF internals exposed via /slam/degeneracy_detail (extended layout v2):
        #   pos_cov_trace = trace of position covariance (m²); growing trace is the
        #     earliest signal of IEKF divergence — leads pose blow-up by 30-60s.
        #   ieskf_iter_num = iterations the last update actually consumed.
        #   ieskf_converged = whether the loop exited via stop_func (vs hitting max_iter).
        self._pos_cov_trace: float = 0.0
        self._ieskf_iter_num: int = 0
        self._ieskf_converged: bool = True
        self._last_severe_warn: float = 0.0   # throttle SEVERE degeneracy warning
        self._last_degen_time: float = 0.0
        # Degeneracy thresholds (tunable)
        self._fitness_warn: float = kw.get("fitness_warn", 0.15)
        self._fitness_critical: float = kw.get("fitness_critical", 0.3)
        self._feature_ratio_warn: float = kw.get("feature_ratio_warn", 0.3)
        self._feature_ratio_critical: float = kw.get("feature_ratio_critical", 0.1)
        self._eigen_ratio_warn: float = kw.get("eigen_ratio_warn", 100.0)  # condition number

        # Selective DOF fusion — visual odometry during degeneracy
        # Based on Selective KF (arXiv 2412.17235): only fuse degenerate DOFs
        self._visual_odom_fusion: bool = kw.get("visual_odom_fusion", True)
        self._visual_alpha: float = kw.get("visual_alpha", 0.6)  # visual blend weight during CRITICAL
        self._last_slam_odom: Odometry | None = None
        self._last_visual_odom: Odometry | None = None
        self._visual_anchor_T: np.ndarray | None = None  # SLAM pose when visual odom activated
        self._visual_fused_count: int = 0

        # GNSS fusion — gently anchor long-term SLAM drift with RTK position.
        # Only XY is fused (Z / orientation stay with SLAM since GNSS altitude is
        # noisy and position alone gives no heading). Alignment is locked once,
        # then GNSS pulls SLAM toward global truth with weight alpha per frame.
        self._gnss_fusion: bool = kw.get("gnss_fusion", True)
        self._gnss_max_age_s: float = kw.get("gnss_max_age_s", 2.0)
        self._gnss_max_std_m: float = kw.get("gnss_max_std_m", 1.0)
        self._gnss_alpha_healthy: float = kw.get("gnss_alpha_healthy", 0.05)
        self._gnss_alpha_degraded: float = kw.get("gnss_alpha_degraded", 0.5)
        self._gnss_rtk_float_scale: float = kw.get("gnss_rtk_float_scale", 0.3)
        # Antenna position in body frame (metres). GNSS reports the antenna's
        # ENU position, so we compensate by rotating this offset into the map
        # frame (via SLAM orientation) and subtracting — giving the body
        # origin position that can be fused with SLAM odometry.
        _ant = kw.get("gnss_antenna_offset", (0.0, 0.0, 0.0))
        self._gnss_antenna_offset: np.ndarray = np.asarray(_ant, dtype=float)
        if self._gnss_antenna_offset.shape != (3,):
            raise ValueError(
                f"gnss_antenna_offset must be length-3, got {self._gnss_antenna_offset.shape}"
            )
        self._last_gnss_odom: GnssOdom | None = None
        self._last_gnss_rx_ts: float = 0.0
        self._gnss_map_offset: np.ndarray | None = None  # shape (2,) XY, map = enu + offset
        self._gnss_fused_count: int = 0

        # Residual guard — protects against locking on a bad first sample
        # (RTK multipath, init glitch). If aligned residual stays above the
        # warn threshold for too long, clear the offset to force re-lock on
        # the next healthy GNSS + SLAM sample.
        self._gnss_residual_warn_m: float = kw.get("gnss_residual_warn_m", 5.0)
        self._gnss_residual_warn_duration_s: float = kw.get(
            "gnss_residual_warn_duration_s", 10.0)
        self._gnss_residual_warn_ratio: float = kw.get(
            "gnss_residual_warn_ratio", 0.7)
        # Ring buffer of (ts, residual_m) for sliding-window ratio check
        self._gnss_residual_history: list[tuple[float, float]] = []
        self._gnss_last_residual_m: float = 0.0
        self._gnss_relock_count: int = 0
        self._gnss_last_relock_ts: float = 0.0

        # FALLBACK_GNSS_ONLY transition tracking. Wall-clock time we first
        # entered DEGRADED in the current degraded streak; cleared on recovery.
        # Once we have been DEGRADED for `_fallback_after_degraded_s` AND GNSS
        # is healthy (RTK_FIXED/FLOAT, fresh) we promote to FALLBACK so
        # downstream Navigation can shed speed.
        self._degraded_since: float = 0.0
        self._fallback_after_degraded_s: float = float(
            kw.get("fallback_after_degraded_s", 10.0))
        self._gnss_max_age_for_fallback_s: float = float(
            kw.get("gnss_max_age_for_fallback_s", 2.0))

    def setup(self) -> None:
        # Visual odometry input for selective fusion
        if self._visual_odom_fusion:
            self.visual_odom.subscribe(self._on_visual_odom)

        # GNSS input for global position anchoring
        if self._gnss_fusion:
            self.gnss_odom.subscribe(self._on_gnss_odom)

        # Try cyclonedds first (lightweight, no ROS2 env needed)
        if self._try_cyclonedds():
            return
        # Fallback to rclpy
        if self._try_rclpy():
            return
        logger.info("SlamBridgeModule: no DDS backend available, stub mode")

    def _try_cyclonedds(self) -> bool:
        try:
            from core.dds import _HAS_CYCLONEDDS, ROS2TopicReader
            # Honour the LINGTU_DISABLE_DDS kill switch — ROS2TopicReader
            # still imports (stub-safe) but its start() returns False;
            # without this check we'd silently construct a dead reader
            # instead of falling through to the rclpy path.
            if not _HAS_CYCLONEDDS:
                return False
            self._reader = ROS2TopicReader()
            self._reader.on_odometry(self._odom_topic, self._on_dds_odom)
            self._reader.on_pointcloud2(self._cloud_topic, self._on_dds_cloud)
            self._reader.on_pointcloud2(self._saved_map_topic, self._on_dds_saved_map)
            self._reader.on_float32(self._quality_topic, self._on_dds_quality)
            # Subscribe to /tf so we can relay map→odom transform (from localizer ICP).
            # Downstream GatewayModule applies this to map_cloud points before SSE
            # so frontend sees both saved_map (already map frame) and map_cloud
            # (Fast-LIO2 odom frame) overlaid in the same reference frame.
            self._reader.on_tf("/tf", self._on_dds_tf)
            # Note: only subscribe to cloud_topic — set cloud_topic="/nav/registered_cloud"
            # for localizer mode (avoids duplicate accumulation when both topics fire)
            logger.info("SlamBridgeModule: using cyclonedds (lightweight)")
            return True
        except ImportError:
            return False

    def _try_rclpy(self) -> bool:
        try:
            from nav_msgs.msg import Odometry as ROS2Odom
            from rclpy.node import Node
            from rclpy.qos import QoSProfile, ReliabilityPolicy
            from sensor_msgs.msg import PointCloud2
            from std_msgs.msg import Float32

            from core.ros2_context import ensure_rclpy, get_shared_executor

            ensure_rclpy()
            qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)
            self._rclpy_node = Node("slam_bridge")
            get_shared_executor().add_node(self._rclpy_node)
            self._rclpy_node.create_subscription(PointCloud2, self._cloud_topic, self._on_rclpy_cloud, qos)
            self._rclpy_node.create_subscription(PointCloud2, self._saved_map_topic, self._on_rclpy_saved_map, qos)
            # Note: only subscribe to cloud_topic — set cloud_topic="/nav/registered_cloud"
            # for localizer mode (avoids duplicate accumulation when both topics fire)
            self._rclpy_node.create_subscription(ROS2Odom, self._odom_topic, self._on_rclpy_odom, qos)
            # Also subscribe to /tf so map→odom reaches downstream even when
            # cyclonedds is disabled (LINGTU_DISABLE_DDS=1). Without this,
            # NavigationModule plans from odom-frame (0,0) instead of the
            # localizer-aligned map-frame position — PCT finds wrong start.
            try:
                from tf2_msgs.msg import TFMessage
                self._rclpy_node.create_subscription(
                    TFMessage, "/tf", self._on_rclpy_tf, qos)
            except Exception as _e:
                logger.warning("SlamBridge: /tf rclpy sub failed: %s", _e)
            # Subscribe to degeneracy metrics if available
            self._subscribe_degeneracy_rclpy(self._rclpy_node, qos)
            logger.info("SlamBridgeModule: using rclpy (fallback)")
            return True
        except (ImportError, Exception) as e:
            logger.debug("SlamBridgeModule: rclpy unavailable: %s", e)
            return False

    def _subscribe_degeneracy_rclpy(self, node, qos) -> None:
        """Subscribe to SLAM degeneracy metrics via rclpy (best-effort).

        Topics:
          /localization_quality     — Float32: ICP fitness score (lower=better)
          /slam/degeneracy          — Float32: effective_ratio (1.0=healthy)
          /slam/degeneracy_detail   — Float32MultiArray: 11-float detailed metrics
            [0]  effective_ratio
            [1]  condition_number
            [2]  min_eigenvalue
            [3]  max_eigenvalue
            [4]  degenerate_dof_count
            [5..10] eigenvalues (6 DOFs, ascending)
        """
        try:
            from rclpy.qos import QoSProfile, ReliabilityPolicy
            from std_msgs.msg import Float32, Float32MultiArray
            sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=1)
            node.create_subscription(
                Float32, self._quality_topic, self._on_rclpy_quality, sensor_qos)
            node.create_subscription(
                Float32, self._degeneracy_topic, self._on_rclpy_degeneracy, sensor_qos)
            node.create_subscription(
                Float32MultiArray, self._degeneracy_detail_topic,
                self._on_rclpy_degeneracy_detail, sensor_qos)
        except Exception as e:
            logger.debug("Degeneracy subscription unavailable: %s", e)

    def start(self) -> None:
        super().start()
        if self._reader:
            self._reader.spin_background()
            self.alive.publish(True)
        elif self._rclpy_node:
            self.alive.publish(True)
        else:
            self.alive.publish(False)
        # Start localization health watchdog
        self._shutdown_event.clear()
        self._watchdog_thread = threading.Thread(
            target=self._watchdog_loop, daemon=True,
            name="slam-bridge-watchdog")
        self._watchdog_thread.start()

    def stop(self) -> None:
        self._shutdown_event.set()
        if self._watchdog_thread and self._watchdog_thread.is_alive():
            self._watchdog_thread.join(timeout=2.0)
        self._watchdog_thread = None
        if self._reader:
            self._reader.stop()
        if self._rclpy_node:
            self._rclpy_node.destroy_node()
            self._rclpy_node = None
        super().stop()

    # ── cyclonedds callbacks (parsed CDR) ────────────────────────────────

    def _on_dds_quality(self, msg) -> None:
        """DDS_Float32 → localization quality (lower = better)."""
        try:
            val = float(msg.data)
            self.localization_quality.publish(val)
            self._icp_fitness = val
            self._last_degen_time = _time.time()
            self._update_degeneracy_level()
        except Exception as e:
            logger.debug("SlamBridge dds quality error: %s", e)

    def _check_drift(self, odom: Odometry) -> bool:
        """Return True if odometry looks diverged (should NOT be published).

        Checks:
          1. |velocity| > max_speed (quad can't exceed ~3 m/s)
          2. |position jump| > max_jump between consecutive frames
          3. |position| > max_pos from origin (indoor environment bound)

        After drift_confirm_frames consecutive bad readings, triggers
        auto-relocalize to last known good position.
        """
        pos = np.array([odom.x, odom.y, odom.z])
        speed = abs(getattr(odom, 'vx', 0.0))
        bad = False

        # Check 1: speed
        if speed > self._drift_max_speed:
            bad = True

        # Check 2: position jump
        if self._prev_odom_pos is not None:
            jump = float(np.linalg.norm(pos[:2] - self._prev_odom_pos[:2]))
            if jump > self._drift_max_jump:
                bad = True

        # Check 3: absolute position
        if np.linalg.norm(pos[:2]) > self._drift_max_pos:
            bad = True

        if bad:
            self._drift_bad_count += 1
        else:
            # Good frame — save as fallback position for relocalize
            self._drift_bad_count = 0
            self._drift_last_good_pos = pos.copy()
            self._drift_last_good_yaw = getattr(odom, 'yaw', 0.0)
            self._prev_odom_pos = pos.copy()
            return False

        self._prev_odom_pos = pos.copy()

        if self._drift_bad_count >= self._drift_confirm_frames:
            now = _time.time()
            if self._loc_state != LOC_DIVERGED:
                logger.error(
                    "Localization DIVERGED: pos=(%.1f,%.1f,%.1f) speed=%.1f "
                    "bad_frames=%d — suppressing odometry",
                    pos[0], pos[1], pos[2], speed, self._drift_bad_count)
                self._loc_state = LOC_DIVERGED

            # Auto-relocalize (with cooldown)
            if now - self._drift_last_relocalize > self._drift_relocalize_cooldown:
                self._drift_last_relocalize = now
                threading.Thread(
                    target=self._auto_relocalize,
                    daemon=True, name="drift-relocalize",
                ).start()
            return True  # suppress this frame

        return False

    def _auto_relocalize(self) -> None:
        """Trigger relocalize via ROS2 service using last known good position.

        If no good position was ever recorded (SLAM diverged from startup),
        falls back to origin (0, 0, 0).
        """
        import os
        import subprocess
        pos = self._drift_last_good_pos
        yaw = self._drift_last_good_yaw
        if pos is None:
            logger.warning("Drift guard: no good position recorded, relocalize to origin")
            pos = np.zeros(3)
            yaw = 0.0

        map_dir = os.environ.get("NAV_MAP_DIR", os.path.expanduser("~/data/inovxio/data/maps"))
        pcd_path = os.path.join(map_dir, "active", "map.pcd")
        if not os.path.isfile(pcd_path):
            # No active map to relocalize against (mapping mode or never saved).
            # Fallback: restart the SLAM systemd service to clear the IEKF
            # state — Fast-LIO2 starts fresh at the robot's current pose.
            # This costs ~3s of blank odometry but recovers from divergence.
            logger.error(
                "Drift guard: no active map PCD (%s) — falling back to "
                "systemctl restart slam to clear IEKF state",
                pcd_path)
            try:
                subprocess.run(
                    ["sudo", "systemctl", "restart", "slam"],
                    capture_output=True, text=True, timeout=15,
                    check=False,
                )
                logger.warning("Drift guard: slam service restarted")
                self._drift_bad_count = 0
                self._drift_last_good_pos = None  # reset so first new frame becomes anchor
            except Exception as e:
                logger.error("Drift guard: slam restart failed: %s", e)
            return

        logger.warning(
            "Drift guard: auto-relocalize to (%.2f, %.2f, yaw=%.2f) using %s",
            pos[0], pos[1], yaw, pcd_path)
        try:
            ros_env = (
                "source /opt/ros/humble/setup.bash && "
                "source ~/data/SLAM/navigation/install/setup.bash 2>/dev/null; "
                "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && "
            )
            subprocess.run(
                ["bash", "-c",
                 ros_env +
                 f"ros2 service call /nav/relocalize interface/srv/Relocalize "
                 f"\"{{pcd_path: '{pcd_path}', x: {pos[0]}, y: {pos[1]}, z: 0.0, "
                 f"yaw: {yaw}, pitch: 0.0, roll: 0.0}}\""],
                capture_output=True, text=True, timeout=30)
            logger.info("Drift guard: relocalize call completed")
            self._drift_bad_count = 0
        except Exception as e:
            logger.warning("Drift guard: relocalize failed: %s", e)

    def _on_dds_odom(self, msg) -> None:
        """DDS_Odometry → Module Odometry (with selective visual fusion + drift guard)."""
        try:
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            t = msg.twist.twist
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            from core.msgs.geometry import Twist
            slam_odom = Odometry(
                pose=Pose(
                    position=Vector3(x=p.x, y=p.y, z=p.z),
                    orientation=Quaternion(x=q.x, y=q.y, z=q.z, w=q.w),
                ),
                twist=Twist(
                    linear=Vector3(x=t.linear.x, y=t.linear.y, z=t.linear.z),
                    angular=Vector3(x=t.angular.x, y=t.angular.y, z=t.angular.z),
                ),
                ts=stamp,
            )
            fused = self._fuse_odometry(slam_odom)

            # Drift guard: suppress diverged odometry, trigger auto-relocalize
            if self._check_drift(fused):
                return  # don't publish garbage to downstream modules

            # Apply map→odom TF so downstream (NavigationModule, Gateway)
            # sees map-frame positions. Without this, planner uses odom
            # frame (robot boot origin) and PCT lookups hit wrong cells.
            fused = self._apply_map_odom_to_odometry(fused)
            self.odometry.publish(fused)
            now = _time.time()
            self._last_odom_time = now
            # Track raw receive rate (independent of downstream callback chain)
            self._odom_recv_ts.append(now)
            if len(self._odom_recv_ts) > 30:
                self._odom_recv_ts.pop(0)
        except Exception as e:
            logger.debug("SlamBridge dds odom error: %s", e)

    def _on_dds_cloud(self, msg) -> None:
        """DDS_PointCloud2 → Module PointCloud2."""
        try:
            n = msg.width * msg.height
            if n == 0:
                return
            step = msg.point_step
            data = np.array(msg.data, dtype=np.uint8)
            raw = data.reshape(n, step)
            xyz = np.zeros((n, 3), dtype=np.float32)
            xyz[:, 0] = np.frombuffer(raw[:, 0:4].tobytes(), dtype=np.float32)
            xyz[:, 1] = np.frombuffer(raw[:, 4:8].tobytes(), dtype=np.float32)
            xyz[:, 2] = np.frombuffer(raw[:, 8:12].tobytes(), dtype=np.float32)
            valid = np.isfinite(xyz).all(axis=1)
            xyz = xyz[valid]
            if xyz.shape[0] > 0:
                # Transform odom-frame points to map frame so downstream
                # (OccupancyGridModule, Gateway) sees coords consistent with
                # the odometry we publish. Without this, map_cloud lives in
                # Fast-LIO2 odom origin but robot_xy is map-frame → costmap
                # rendered at rotated/translated position.
                xyz_map = self._apply_map_odom_to_points(xyz)
                self.map_cloud.publish(PointCloud2.from_numpy(xyz_map, frame_id="map"))
                self._last_cloud_time = _time.time()
        except Exception as e:
            logger.debug("SlamBridge dds cloud error: %s", e)

    def _on_dds_saved_map(self, msg) -> None:
        """DDS saved-map PointCloud2 → Module saved_map Out."""
        try:
            n = msg.width * msg.height
            if n == 0:
                return
            step = msg.point_step
            raw = np.array(msg.data, dtype=np.uint8).reshape(n, step)
            xyz = np.zeros((n, 3), dtype=np.float32)
            xyz[:, 0] = np.frombuffer(raw[:, 0:4].tobytes(), dtype=np.float32)
            xyz[:, 1] = np.frombuffer(raw[:, 4:8].tobytes(), dtype=np.float32)
            xyz[:, 2] = np.frombuffer(raw[:, 8:12].tobytes(), dtype=np.float32)
            valid = np.isfinite(xyz).all(axis=1)
            xyz = xyz[valid]
            if xyz.shape[0] > 0:
                self.saved_map.publish(PointCloud2.from_numpy(xyz, frame_id="map"))
        except Exception as e:
            logger.debug("SlamBridge dds saved_map error: %s", e)

    def _apply_map_odom_to_points(self, xyz: np.ndarray) -> np.ndarray:
        """Transform an (N, 3) point cloud from odom frame to map frame using
        the cached map→odom TF. Returns the same array if TF not yet cached.
        Vectorized — avoids a per-point loop for tens of thousands of points.
        """
        T = getattr(self, "_T_map_odom", None)
        if T is None or xyz.shape[0] == 0:
            return xyz
        R = T[:3, :3].astype(np.float32)
        t = T[:3, 3].astype(np.float32)
        return (xyz @ R.T) + t

    def _apply_map_odom_to_odometry(self, odom: Odometry) -> Odometry:
        """Transform odom from Fast-LIO2 odom frame into map frame using the
        localizer-emitted map→odom TF. No-op when TF is not yet cached.
        Composes both translation and rotation — forgetting the rotation
        makes map.yaw == odom.yaw which is wrong whenever the BBS3D align
        produced a non-zero yaw (very common, e.g. -104° here).
        """
        T = getattr(self, "_T_map_odom", None)
        if T is None:
            return odom
        import numpy as _np
        # Position: map_p = T_map_odom * odom_p
        p = odom.pose.position
        ox, oy, oz = float(p.x), float(p.y), float(p.z)
        odom.pose.position.x = float(T[0, 0] * ox + T[0, 1] * oy + T[0, 2] * oz + T[0, 3])
        odom.pose.position.y = float(T[1, 0] * ox + T[1, 1] * oy + T[1, 2] * oz + T[1, 3])
        odom.pose.position.z = float(T[2, 0] * ox + T[2, 1] * oy + T[2, 2] * oz + T[2, 3])
        # Rotation: map_R_body = map_R_odom * odom_R_body
        # Convert odom quat → 3x3, multiply, convert back to quat.
        q = odom.pose.orientation
        qx, qy, qz, qw = float(q.x), float(q.y), float(q.z), float(q.w)
        xx, yy, zz = qx * qx, qy * qy, qz * qz
        xy, xz, yz = qx * qy, qx * qz, qy * qz
        wx, wy, wzz = qw * qx, qw * qy, qw * qz
        R_body = _np.array([
            [1 - 2 * (yy + zz),     2 * (xy - wzz),    2 * (xz + wy)],
            [    2 * (xy + wzz), 1 - 2 * (xx + zz),    2 * (yz - wx)],
            [    2 * (xz - wy),     2 * (yz + wx), 1 - 2 * (xx + yy)],
        ], dtype=_np.float64)
        R_map = T[:3, :3] @ R_body
        # 3x3 → quat (Shepperd-style, stable for any rotation)
        tr = R_map[0, 0] + R_map[1, 1] + R_map[2, 2]
        if tr > 0:
            S = 2.0 * _np.sqrt(tr + 1.0)
            nw = 0.25 * S
            nx = (R_map[2, 1] - R_map[1, 2]) / S
            ny = (R_map[0, 2] - R_map[2, 0]) / S
            nz = (R_map[1, 0] - R_map[0, 1]) / S
        elif (R_map[0, 0] > R_map[1, 1]) and (R_map[0, 0] > R_map[2, 2]):
            S = 2.0 * _np.sqrt(1.0 + R_map[0, 0] - R_map[1, 1] - R_map[2, 2])
            nw = (R_map[2, 1] - R_map[1, 2]) / S
            nx = 0.25 * S
            ny = (R_map[0, 1] + R_map[1, 0]) / S
            nz = (R_map[0, 2] + R_map[2, 0]) / S
        elif R_map[1, 1] > R_map[2, 2]:
            S = 2.0 * _np.sqrt(1.0 + R_map[1, 1] - R_map[0, 0] - R_map[2, 2])
            nw = (R_map[0, 2] - R_map[2, 0]) / S
            nx = (R_map[0, 1] + R_map[1, 0]) / S
            ny = 0.25 * S
            nz = (R_map[1, 2] + R_map[2, 1]) / S
        else:
            S = 2.0 * _np.sqrt(1.0 + R_map[2, 2] - R_map[0, 0] - R_map[1, 1])
            nw = (R_map[1, 0] - R_map[0, 1]) / S
            nx = (R_map[0, 2] + R_map[2, 0]) / S
            ny = (R_map[1, 2] + R_map[2, 1]) / S
            nz = 0.25 * S
        odom.pose.orientation.x = float(nx)
        odom.pose.orientation.y = float(ny)
        odom.pose.orientation.z = float(nz)
        odom.pose.orientation.w = float(nw)
        return odom

    def _cache_map_odom_tf(self, tx, ty, tz, qx, qy, qz, qw):
        """Cache map→odom as 4x4 for downstream variance-free transforms."""
        import numpy as _np
        xx, yy, zz = qx * qx, qy * qy, qz * qz
        xy, xz, yz = qx * qy, qx * qz, qy * qz
        wx, wy, wz = qw * qx, qw * qy, qw * qz
        R = _np.array([
            [1 - 2 * (yy + zz),     2 * (xy - wz),     2 * (xz + wy)],
            [    2 * (xy + wz), 1 - 2 * (xx + zz),     2 * (yz - wx)],
            [    2 * (xz - wy),     2 * (yz + wx), 1 - 2 * (xx + yy)],
        ], dtype=_np.float64)
        T = _np.eye(4, dtype=_np.float64)
        T[:3, :3] = R
        T[:3, 3] = [tx, ty, tz]
        self._T_map_odom = T

    def _on_rclpy_tf(self, msg) -> None:
        try:
            for t in msg.transforms:
                parent = t.header.frame_id or ""
                child = t.child_frame_id or ""
                if parent == "map" and child == "odom":
                    tr = t.transform.translation
                    q = t.transform.rotation
                    self._cache_map_odom_tf(
                        float(tr.x), float(tr.y), float(tr.z),
                        float(q.x), float(q.y), float(q.z), float(q.w))
                    self.map_odom_tf.publish({
                        "tx": float(tr.x), "ty": float(tr.y), "tz": float(tr.z),
                        "qx": float(q.x), "qy": float(q.y),
                        "qz": float(q.z), "qw": float(q.w),
                        "valid": True,
                    })
                    return
        except Exception as e:
            logger.debug("SlamBridge rclpy tf error: %s", e)

    def _on_dds_tf(self, msg) -> None:
        """DDS /tf TFMessage → extract map→odom, relay as dict to map_odom_tf Out.

        /tf is fan-in from every broadcaster, so we walk transforms and pick
        the specific (parent=map, child=odom) pair the localizer publishes.
        Downstream GatewayModule applies this to odom-frame point clouds so
        the browser sees saved_map + map_cloud overlaid correctly.
        """
        try:
            transforms = getattr(msg, "transforms", None) or []
            for t in transforms:
                parent = getattr(t.header, "frame_id", "") or ""
                child = getattr(t, "child_frame_id", "") or ""
                if parent == "map" and child == "odom":
                    trans = t.transform.translation
                    rot = t.transform.rotation
                    tf_msg = {
                        "tx": float(trans.x), "ty": float(trans.y), "tz": float(trans.z),
                        "qx": float(rot.x),   "qy": float(rot.y),
                        "qz": float(rot.z),   "qw": float(rot.w),
                        "valid": True,
                    }
                    self._cache_map_odom_tf(
                        tf_msg["tx"], tf_msg["ty"], tf_msg["tz"],
                        tf_msg["qx"], tf_msg["qy"], tf_msg["qz"], tf_msg["qw"])
                    self.map_odom_tf.publish(tf_msg)
                    return
        except Exception as e:
            logger.debug("SlamBridge dds tf error: %s", e)

    # ── rclpy callbacks (fallback) ───────────────────────────────────────

    def _on_rclpy_odom(self, msg) -> None:
        try:
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            t = msg.twist.twist
            from core.msgs.geometry import Twist
            slam_odom = Odometry(
                pose=Pose(
                    position=Vector3(x=float(p.x), y=float(p.y), z=float(p.z)),
                    orientation=Quaternion(x=float(q.x), y=float(q.y), z=float(q.z), w=float(q.w)),
                ),
                twist=Twist(
                    linear=Vector3(x=float(t.linear.x), y=float(t.linear.y), z=float(t.linear.z)),
                    angular=Vector3(x=float(t.angular.x), y=float(t.angular.y), z=float(t.angular.z)),
                ),
            )
            # Track max position covariance from IESKF P matrix (filled by lio_node.cpp)
            cov = msg.pose.covariance  # 36-element row-major 6x6
            if len(cov) >= 15:
                self._max_pos_cov = max(float(cov[0]), float(cov[7]), float(cov[14]))

            fused = self._fuse_odometry(slam_odom)
            # Apply map→odom TF so downstream (NavigationModule, Gateway)
            # sees map-frame positions. Without this, planner uses odom
            # frame (robot boot origin) and PCT lookups hit wrong cells.
            fused = self._apply_map_odom_to_odometry(fused)
            self.odometry.publish(fused)
            self._last_odom_time = _time.time()
        except Exception as e:
            logger.warning("SlamBridge rclpy odom error: %s", e)

    def _on_rclpy_cloud(self, msg) -> None:
        try:
            n = msg.width * msg.height
            if n == 0:
                return
            step = msg.point_step
            if step == 16:
                pts = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, 4)
                xyz = pts[:, :3].copy()
            else:
                raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(n, step)
                xyz = np.zeros((n, 3), dtype=np.float32)
                xyz[:, 0] = np.frombuffer(raw[:, 0:4].tobytes(), dtype=np.float32)
                xyz[:, 1] = np.frombuffer(raw[:, 4:8].tobytes(), dtype=np.float32)
                xyz[:, 2] = np.frombuffer(raw[:, 8:12].tobytes(), dtype=np.float32)
            valid = np.isfinite(xyz).all(axis=1)
            xyz = xyz[valid]
            if xyz.shape[0] > 0:
                # Transform odom-frame points to map frame so downstream
                # (OccupancyGridModule, Gateway) sees coords consistent with
                # the odometry we publish. Without this, map_cloud lives in
                # Fast-LIO2 odom origin but robot_xy is map-frame → costmap
                # rendered at rotated/translated position.
                xyz_map = self._apply_map_odom_to_points(xyz)
                self.map_cloud.publish(PointCloud2.from_numpy(xyz_map, frame_id="map"))
                self._last_cloud_time = _time.time()
        except Exception as e:
            logger.debug("SlamBridge rclpy cloud error: %s", e)

    def _on_rclpy_saved_map(self, msg) -> None:
        try:
            n = msg.width * msg.height
            if n == 0:
                return
            step = msg.point_step
            if step == 16:
                pts = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, 4)
                xyz = pts[:, :3].copy()
            else:
                raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(n, step)
                xyz = np.zeros((n, 3), dtype=np.float32)
                xyz[:, 0] = np.frombuffer(raw[:, 0:4].tobytes(), dtype=np.float32)
                xyz[:, 1] = np.frombuffer(raw[:, 4:8].tobytes(), dtype=np.float32)
                xyz[:, 2] = np.frombuffer(raw[:, 8:12].tobytes(), dtype=np.float32)
            valid = np.isfinite(xyz).all(axis=1)
            xyz = xyz[valid]
            if xyz.shape[0] > 0:
                self.saved_map.publish(PointCloud2.from_numpy(xyz, frame_id="map"))
        except Exception as e:
            logger.debug("SlamBridge rclpy saved_map error: %s", e)

    # ── Degeneracy metric callbacks ─────────────────────────────────────────

    def _on_rclpy_quality(self, msg) -> None:
        """ICP fitness score from localizer (lower = better)."""
        val = float(msg.data)
        self._icp_fitness = val
        self.localization_quality.publish(val)
        self._last_degen_time = _time.time()
        self._update_degeneracy_level()

    def _on_rclpy_degeneracy(self, msg) -> None:
        """Effective feature ratio from SLAM (1.0 = all features valid)."""
        self._effective_ratio = float(msg.data)
        self._last_degen_time = _time.time()
        self._update_degeneracy_level()

    def _on_rclpy_degeneracy_detail(self, msg) -> None:
        """Detailed degeneracy metrics from Hessian eigenvalue analysis.

        Float32MultiArray, original 11-float layout extended to 14:
          [0]  condition_number
          [1]  min_eigenvalue
          [2]  max_eigenvalue
          [3]  effective_ratio
          [4]  degenerate_dof_count
          [5..10] dof_mask (6 DOFs)
          [11] pos_cov_trace (m²)        — IEKF position covariance trace (NEW)
          [12] iter_num                   — IEKF iterations actually used (NEW)
          [13] converged (1.0 / 0.0)      — whether stop_func fired (NEW)
        Older publishers send only 11 floats; the extra fields are read with len guard.
        """
        d = msg.data
        if len(d) < 11:
            return
        self._condition_number = float(d[0])
        self._eigenvalue_ratio = float(d[0])  # condition_number = max_eig / min_eig
        self._effective_ratio = float(d[3])
        self._degenerate_dof_count = int(d[4])
        self._dof_mask = np.array([float(d[5 + i]) for i in range(6)])
        if len(d) >= 14:
            self._pos_cov_trace = float(d[11])
            self._ieskf_iter_num = int(d[12])
            self._ieskf_converged = bool(d[13] >= 0.5)
        # Reconstruct eigenvalues from mask (approximate; mask gives 0/1 per DOF)
        max_eig = float(d[2])
        min_eig = float(d[1])
        self._eigenvalues = np.array([min_eig if self._dof_mask[i] < 0.5 else max_eig
                                      for i in range(6)])

        # SEVERE early warning — throttled to once per 30s
        now = _time.time()
        if self._condition_number > 1e6 and (now - self._last_severe_warn) > 30.0:
            self._last_severe_warn = now
            logger.warning(
                "SEVERE SLAM degeneracy: cond=%.2e, effective_ratio=%.3f, degen_dofs=%d — "
                "covariance growth likely; consider stopping or relocating robot",
                self._condition_number, self._effective_ratio, self._degenerate_dof_count)

        self._last_degen_time = _time.time()
        self._update_degeneracy_level()

    def _update_degeneracy_level(self) -> None:
        """Classify degeneracy severity from SLAM quality metrics.

        Uses multiple signals:
          - ICP fitness score (localizer quality)
          - effective_ratio (Hessian-based, from C++ eigenvalue analysis)
          - condition_number (max_eig / min_eig, from degeneracy_detail)
          - degenerate_dof_count (how many of 6 DOFs are degenerate)
        """
        prev = self._degen_level

        # Hessian-based criteria (from degeneracy_detail)
        hessian_critical = (self._degenerate_dof_count >= 3
                            or self._condition_number > 10000)
        hessian_severe = (self._degenerate_dof_count >= 1
                          or self._condition_number > self._eigen_ratio_warn)

        # Critical: ICP fitness very bad OR almost no features OR Hessian critical
        if (self._icp_fitness > self._fitness_critical
                or self._effective_ratio < self._feature_ratio_critical
                or hessian_critical):
            level = DEGEN_CRITICAL
        # Severe: ICP poor OR low features OR Hessian severe
        elif (self._icp_fitness > self._fitness_warn
              or self._effective_ratio < self._feature_ratio_warn
              or hessian_severe):
            level = DEGEN_SEVERE
        # Mild: borderline but still usable
        elif (self._icp_fitness > self._fitness_warn * 0.7
              or self._effective_ratio < self._feature_ratio_warn * 1.5):
            level = DEGEN_MILD
        else:
            level = DEGEN_NONE

        self._degen_level = level
        if level != prev and level != DEGEN_NONE:
            logger.warning(
                "SLAM degeneracy: %s -> %s (fitness=%.3f, feat_ratio=%.2f, "
                "cond=%.0f, degen_dofs=%d)",
                prev, level, self._icp_fitness, self._effective_ratio,
                self._condition_number, self._degenerate_dof_count)
        elif level == DEGEN_NONE and prev != DEGEN_NONE:
            logger.info("SLAM degeneracy cleared: %s -> NONE", prev)

    # ── Visual odometry selective DOF fusion ────────────────────────────

    def _on_visual_odom(self, odom: Odometry) -> None:
        """Receive visual odometry from DepthVisualOdomModule.

        Based on Selective KF (arXiv 2412.17235):
        Only fuse visual odometry for degenerate DOF directions.
        When SLAM is healthy, visual odom is ignored.
        """
        self._last_visual_odom = odom

    def _fuse_odometry(self, slam_odom: Odometry) -> Odometry:
        """Two-stage fusion: visual (DOF-selective) then GNSS (global anchor)."""
        after_visual = self._fuse_visual(slam_odom)
        return self._fuse_gnss_position(after_visual)

    def _fuse_visual(self, slam_odom: Odometry) -> Odometry:
        """Selectively blend visual odometry into degenerate DOF directions.

        When degeneracy is NONE/MILD: pass SLAM odom unchanged.
        When SEVERE: blend visual at alpha=0.3 for degenerate directions.
        When CRITICAL: blend visual at alpha=0.6 for degenerate directions.

        Degenerate direction detection (simplified):
        - Corridor → X (along-corridor) is degenerate
        - Open field → X,Y (horizontal plane) are degenerate
        - We use ICP fitness + effective_ratio as proxy:
          * High fitness + low ratio → full translational degeneracy
          * High fitness + moderate ratio → partial (along-corridor)
        """
        if not self._visual_odom_fusion:
            return slam_odom
        if self._degen_level not in (DEGEN_SEVERE, DEGEN_CRITICAL):
            self._last_slam_odom = slam_odom
            return slam_odom
        if self._last_visual_odom is None:
            return slam_odom

        # Determine blend alpha based on severity
        if self._degen_level == DEGEN_CRITICAL:
            alpha = self._visual_alpha  # 0.6
        else:
            alpha = self._visual_alpha * 0.5  # 0.3 for SEVERE

        # Anchor: record SLAM pose when fusion first activates
        slam_pos = np.array([
            slam_odom.pose.position.x,
            slam_odom.pose.position.y,
            slam_odom.pose.position.z,
        ])
        vis_pos = np.array([
            self._last_visual_odom.pose.position.x,
            self._last_visual_odom.pose.position.y,
            self._last_visual_odom.pose.position.z,
        ])

        # Detect degenerate translational axes.
        # If Hessian DOF mask is available (from degeneracy_detail), use it
        # directly: first 3 DOFs are translational (tx, ty, tz).
        # Otherwise fall back to heuristic based on fitness/ratio.
        if self._dof_mask is not None and len(self._dof_mask) >= 3:
            # W2-9: strict preference for Hessian eigenvalue-based mask when
            # available. DOF mask: 1.0=constrained, 0.0=degenerate.
            # Invert: degenerate axes get alpha, constrained get 0.
            logger.debug(
                "slam_bridge: fusing via eigenvalue DOF mask (trans=%s)",
                self._dof_mask[:3].tolist(),
            )
            trans_mask = self._dof_mask[:3]  # tx, ty, tz
            degen_mask = np.where(trans_mask < 0.5, alpha, 0.0)
            # Always reduce Z fusion weight (vertical usually stable from IMU)
            degen_mask[2] *= 0.3
        elif self._effective_ratio < self._feature_ratio_critical:
            # Full translational degeneracy — blend all XYZ (heuristic fallback)
            logger.debug(
                "slam_bridge: heuristic full-XYZ path (effective_ratio=%.3f)",
                self._effective_ratio,
            )
            degen_mask = np.array([alpha, alpha, alpha * 0.3])
        elif self._icp_fitness > self._fitness_critical:
            # Corridor-like: blend XY, keep Z from SLAM (heuristic fallback)
            logger.debug(
                "slam_bridge: heuristic corridor path (icp_fitness=%.3f)",
                self._icp_fitness,
            )
            degen_mask = np.array([alpha, alpha, 0.0])
        else:
            # Partial: blend X (along-axis of highest uncertainty) — heuristic
            logger.debug("slam_bridge: heuristic partial-X path")
            degen_mask = np.array([alpha, alpha * 0.3, 0.0])

        # Selective blend: fused = slam * (1 - mask) + visual * mask
        fused_pos = slam_pos * (1.0 - degen_mask) + vis_pos * degen_mask

        self._visual_fused_count += 1
        if self._visual_fused_count % 50 == 1:
            logger.info(
                "Visual-SLAM fusion active: alpha=%.2f, degen=%s, "
                "slam_xy=(%.2f,%.2f), vis_xy=(%.2f,%.2f), fused_xy=(%.2f,%.2f)",
                alpha, self._degen_level,
                slam_pos[0], slam_pos[1],
                vis_pos[0], vis_pos[1],
                fused_pos[0], fused_pos[1],
            )

        # Build fused odometry (keep SLAM orientation + twist)
        fused_odom = Odometry(
            pose=Pose(
                position=Vector3(
                    x=float(fused_pos[0]),
                    y=float(fused_pos[1]),
                    z=float(fused_pos[2]),
                ),
                orientation=slam_odom.pose.orientation,
            ),
            twist=slam_odom.twist,
            ts=slam_odom.ts,
        )
        self._last_slam_odom = fused_odom
        return fused_odom

    # ── GNSS global position anchoring ──────────────────────────────────

    def _on_gnss_odom(self, odom: GnssOdom) -> None:
        """Store latest GNSS ENU odometry; used by _fuse_gnss_position."""
        self._last_gnss_odom = odom
        self._last_gnss_rx_ts = _time.time()

    def _fuse_gnss_position(self, odom: Odometry) -> Odometry:
        """Blend RTK GNSS ENU position into SLAM XY to anchor long-term drift.

        Strategy
        --------
        1. Gate on fix quality (RTK only), freshness, horizontal std.
        2. First qualifying sample while SLAM is healthy locks the
           ``map ↔ enu`` translation offset (one-shot alignment).
        3. Subsequent samples: weighted blend of SLAM XY with aligned GNSS XY.
           Weight rises when SLAM is degraded so GNSS carries more load.
        4. Z and orientation untouched — GNSS altitude is too noisy and a
           single position doesn't define heading.
        """
        if not self._gnss_fusion or self._last_gnss_odom is None:
            return odom

        g = self._last_gnss_odom

        # Freshness: drop if the last received sample is stale
        age = _time.time() - self._last_gnss_rx_ts
        if age > self._gnss_max_age_s:
            return odom

        # Quality: only RTK (fixed or float) carries position-level authority.
        # SINGLE/DGPS are too coarse (> 1 m) to help a LiDAR SLAM anchor.
        if g.fix_type not in (GnssFixType.RTK_FIXED, GnssFixType.RTK_FLOAT):
            return odom

        # Noise gate
        h_var = max(g.cov_e, 0.0) + max(g.cov_n, 0.0)
        h_std = (h_var * 0.5) ** 0.5
        if h_std > self._gnss_max_std_m:
            return odom

        slam_xy = np.array([odom.pose.position.x, odom.pose.position.y])

        # Lever-arm compensation: GNSS reports the antenna, SLAM reports the
        # body. Rotate the antenna offset from body to map frame using current
        # SLAM orientation, then subtract its XY from the GNSS ENU position
        # so both operands refer to the body origin.
        if self._gnss_antenna_offset.any():
            R_body2map = odom.pose.orientation.to_rotation_matrix()
            a_map = R_body2map @ self._gnss_antenna_offset
            gnss_xy = np.array([g.east - a_map[0], g.north - a_map[1]])
        else:
            gnss_xy = np.array([g.east, g.north])

        # One-shot alignment: lock the map↔ENU translation while SLAM healthy
        if self._gnss_map_offset is None:
            if self._loc_state != LOC_TRACKING:
                return odom
            if self._degen_level in (DEGEN_SEVERE, DEGEN_CRITICAL):
                return odom
            self._gnss_map_offset = slam_xy - gnss_xy
            logger.info(
                "GNSS-SLAM alignment locked: map_offset=(%.3f, %.3f) m "
                "(fix=%s, std=%.2fm)",
                self._gnss_map_offset[0], self._gnss_map_offset[1],
                g.fix_type.name, h_std,
            )
            return odom

        # GNSS in SLAM map frame
        gnss_in_map = gnss_xy + self._gnss_map_offset

        # Residual guard: track misalignment, force re-lock on sustained drift
        residual = float(np.linalg.norm(slam_xy - gnss_in_map))
        self._gnss_last_residual_m = residual
        if self._check_residual_and_maybe_relock(residual):
            # Just cleared the offset — skip blending this frame, next valid
            # GNSS sample will re-lock on a fresh pair.
            self._publish_gnss_health()
            return odom

        # Weight: degraded SLAM → trust GNSS more
        if self._degen_level in (DEGEN_SEVERE, DEGEN_CRITICAL):
            alpha = self._gnss_alpha_degraded
        else:
            alpha = self._gnss_alpha_healthy
        if g.fix_type == GnssFixType.RTK_FLOAT:
            alpha *= self._gnss_rtk_float_scale

        fused_xy = slam_xy * (1.0 - alpha) + gnss_in_map * alpha

        self._gnss_fused_count += 1
        if self._gnss_fused_count % 50 == 1:
            logger.info(
                "GNSS-SLAM fusion active: alpha=%.2f, fix=%s, std=%.2fm, "
                "residual=%.2fm, slam=(%.2f,%.2f) gnss=(%.2f,%.2f) "
                "fused=(%.2f,%.2f)",
                alpha, g.fix_type.name, h_std, residual,
                slam_xy[0], slam_xy[1],
                gnss_in_map[0], gnss_in_map[1],
                fused_xy[0], fused_xy[1],
            )
        self._publish_gnss_health()

        return Odometry(
            pose=Pose(
                position=Vector3(
                    x=float(fused_xy[0]),
                    y=float(fused_xy[1]),
                    z=odom.pose.position.z,
                ),
                orientation=odom.pose.orientation,
            ),
            twist=odom.twist,
            ts=odom.ts,
        )

    def _check_residual_and_maybe_relock(self, residual: float) -> bool:
        """Sliding-window check: if too many recent samples exceed threshold,
        clear the alignment offset so the next good sample re-locks it.

        Returns True iff the offset was just cleared.
        """
        now = _time.time()
        window = self._gnss_residual_warn_duration_s
        self._gnss_residual_history.append((now, residual))
        cutoff = now - window
        # Drop entries older than the window
        while self._gnss_residual_history and self._gnss_residual_history[0][0] < cutoff:
            self._gnss_residual_history.pop(0)

        # Need a filled window + enough samples (≥ 5) before firing
        if not self._gnss_residual_history:
            return False
        if self._gnss_residual_history[0][0] > now - window * 0.5:
            return False
        if len(self._gnss_residual_history) < 5:
            return False

        warn_count = sum(
            1 for _ts, r in self._gnss_residual_history
            if r > self._gnss_residual_warn_m
        )
        ratio = warn_count / len(self._gnss_residual_history)
        if ratio < self._gnss_residual_warn_ratio:
            return False

        # Fire: clear offset, bump counter, drop history
        logger.warning(
            "GNSS-SLAM residual sustained > %.1fm (%.0f%% of last %.1fs, "
            "n=%d); relocking alignment. Previous offset=%s",
            self._gnss_residual_warn_m, ratio * 100.0, window,
            len(self._gnss_residual_history),
            None if self._gnss_map_offset is None
            else f"({self._gnss_map_offset[0]:.2f}, {self._gnss_map_offset[1]:.2f})",
        )
        self._gnss_map_offset = None
        self._gnss_residual_history.clear()
        self._gnss_relock_count += 1
        self._gnss_last_relock_ts = now
        return True

    def _publish_gnss_health(self) -> None:
        """Publish a compact diagnostic snapshot for Dashboard / watchdog."""
        self.gnss_fusion_health.publish({
            "enabled": self._gnss_fusion,
            "alignment_locked": self._gnss_map_offset is not None,
            "map_offset": (
                None if self._gnss_map_offset is None
                else [float(self._gnss_map_offset[0]),
                      float(self._gnss_map_offset[1])]
            ),
            "last_residual_m": float(self._gnss_last_residual_m),
            "fused_count": int(self._gnss_fused_count),
            "relock_count": int(self._gnss_relock_count),
            "last_relock_ts": float(self._gnss_last_relock_ts),
            "last_fix_type": (
                self._last_gnss_odom.fix_type.name
                if self._last_gnss_odom is not None else "NONE"
            ),
            "last_gnss_age_s": (
                float(_time.time() - self._last_gnss_rx_ts)
                if self._last_gnss_rx_ts > 0 else float("inf")
            ),
        })

    def _is_gnss_healthy(self, now: float) -> bool:
        """True when GNSS is fresh and at RTK_FIXED/FLOAT precision.

        Used to gate the LOC_DEGRADED → LOC_FALLBACK_GNSS_ONLY transition:
        promoting to fallback only makes sense if we have a globally-anchored
        position to fall back to.
        """
        if self._last_gnss_odom is None or self._last_gnss_rx_ts <= 0:
            return False
        age = now - self._last_gnss_rx_ts
        if age > self._gnss_max_age_for_fallback_s:
            return False
        return self._last_gnss_odom.fix_type in (
            GnssFixType.RTK_FIXED, GnssFixType.RTK_FLOAT)

    # ── Localization health watchdog ──────────────────────────────��──────

    def _watchdog_loop(self) -> None:
        """Periodic localization health check."""
        interval = 1.0 / self._watchdog_hz
        while not self._shutdown_event.is_set():
            now = _time.time()
            odom_age = (now - self._last_odom_time) if self._last_odom_time > 0 else float("inf")
            cloud_age = (now - self._last_cloud_time) if self._last_cloud_time > 0 else float("inf")

            if self._loc_state == LOC_DIVERGED:
                # Drift guard set this — keep it until drift clears
                new_state = LOC_DIVERGED
                confidence = 0.0
            elif self._last_odom_time == 0.0:
                new_state = LOC_UNINIT
                confidence = 0.0
            elif odom_age > self._odom_timeout:
                new_state = LOC_LOST
                confidence = 0.0
            elif cloud_age > self._cloud_timeout:
                new_state = LOC_DEGRADED
                confidence = 0.3
            elif self._degen_level == DEGEN_CRITICAL:
                new_state = LOC_DEGRADED
                confidence = 0.1
            else:
                new_state = LOC_TRACKING
                confidence = max(0.0, 1.0 - odom_age / self._odom_timeout)

            # Promote sustained DEGRADED to FALLBACK_GNSS_ONLY when GNSS is
            # healthy. Today this only changes the published state so
            # NavigationModule can run cautiously; the GNSS+IMU dead-reckoning
            # takeover lands in S2.5 once fault-injection tooling exists.
            if new_state == LOC_DEGRADED:
                if self._degraded_since == 0.0:
                    self._degraded_since = now
                if (now - self._degraded_since) > self._fallback_after_degraded_s \
                        and self._is_gnss_healthy(now):
                    new_state = LOC_FALLBACK_GNSS_ONLY
                    confidence = max(confidence, 0.4)
            else:
                self._degraded_since = 0.0

            # Reduce confidence based on degeneracy
            # Boost confidence when visual fusion is active
            visual_active = (self._last_visual_odom is not None
                             and self._degen_level in (DEGEN_SEVERE, DEGEN_CRITICAL))
            if self._degen_level == DEGEN_SEVERE:
                confidence = min(confidence, 0.6 if visual_active else 0.4)
            elif self._degen_level == DEGEN_MILD:
                confidence = min(confidence, 0.7)
            elif self._degen_level == DEGEN_CRITICAL:
                if visual_active:
                    # Visual fusion compensates for CRITICAL — upgrade to DEGRADED not LOST
                    confidence = max(confidence, 0.3)

            if new_state != self._loc_state:
                if new_state == LOC_LOST:
                    logger.warning("Localization LOST: no odometry for %.1fs", odom_age)
                elif new_state == LOC_DEGRADED and self._degen_level != DEGEN_NONE:
                    logger.warning("Localization DEGRADED: SLAM degeneracy %s",
                                   self._degen_level)
                elif new_state == LOC_FALLBACK_GNSS_ONLY:
                    logger.warning(
                        "Localization FALLBACK_GNSS_ONLY: DEGRADED for %.1fs with "
                        "healthy GNSS — Navigation should run cautiously",
                        now - self._degraded_since)
                elif new_state == LOC_TRACKING and self._loc_state in (
                        LOC_LOST, LOC_DEGRADED, LOC_DIVERGED, LOC_FALLBACK_GNSS_ONLY):
                    logger.info("Localization recovered -> TRACKING from %s",
                                self._loc_state)
                    self._reconnect_count = 0
                    self._drift_bad_count = 0
                self._loc_state = new_state

            # Auto-recovery: reconnect DDS/rclpy if LOST for too long
            if (new_state == LOC_LOST
                    and odom_age > self._reconnect_timeout
                    and self._reconnect_count < self._max_reconnects):
                self._attempt_reconnect()

            self.localization_status.publish({
                "state": self._loc_state,
                "odom_age_ms": round(odom_age * 1000, 1),
                "cloud_age_ms": round(cloud_age * 1000, 1),
                "confidence": round(confidence, 2),
                "degeneracy": self._degen_level,
                "icp_fitness": round(self._icp_fitness, 4),
                "effective_ratio": round(self._effective_ratio, 3),
                "condition_number": round(self._condition_number, 1),
                "degenerate_dof_count": self._degenerate_dof_count,
                "cov_warning": self._max_pos_cov > 100.0,
                "max_pos_cov": round(self._max_pos_cov, 3),
                "pos_cov_trace": round(self._pos_cov_trace, 4),
                "ieskf_iter_num": self._ieskf_iter_num,
                "ieskf_converged": self._ieskf_converged,
            })

            self._shutdown_event.wait(timeout=interval)

    def _attempt_reconnect(self) -> None:
        """Try to reconnect DDS/rclpy backend after data loss."""
        self._reconnect_count += 1
        backoff = min(2.0 ** self._reconnect_count, 30.0)
        logger.warning(
            "SlamBridge: attempting reconnect (%d/%d, backoff %.0fs)",
            self._reconnect_count, self._max_reconnects, backoff)

        # Destroy old connections
        if self._reader:
            try:
                self._reader.stop()
            except Exception as e:
                logger.debug("SlamBridge: reader.stop() cleanup error: %s", e)
            self._reader = None
        if self._rclpy_node:
            try:
                self._rclpy_node.destroy_node()
            except Exception as e:
                logger.debug("SlamBridge: rclpy node cleanup error: %s", e)
            self._rclpy_node = None

        self._shutdown_event.wait(timeout=backoff)
        if self._shutdown_event.is_set():
            return

        # Try to reconnect
        if self._try_cyclonedds():
            self._reader.spin_background()
            logger.info("SlamBridge: cyclonedds reconnected")
            self.alive.publish(True)
        elif self._try_rclpy():
            logger.info("SlamBridge: rclpy reconnected")
            self.alive.publish(True)
        else:
            logger.warning("SlamBridge: reconnect failed")
            self.alive.publish(False)

    def health(self) -> dict[str, Any]:
        info = super().port_summary()
        now = _time.time()
        info["localization"] = {
            "state": self._loc_state,
            "odom_age_ms": round((now - self._last_odom_time) * 1000, 1) if self._last_odom_time else -1,
            "cloud_age_ms": round((now - self._last_cloud_time) * 1000, 1) if self._last_cloud_time else -1,
            "degeneracy": self._degen_level,
            "icp_fitness": round(self._icp_fitness, 4),
            "effective_ratio": round(self._effective_ratio, 3),
            "condition_number": round(self._condition_number, 1),
            "degenerate_dof_count": self._degenerate_dof_count,
            "dof_mask": self._dof_mask.tolist() if self._dof_mask is not None else None,
            "visual_fusion_active": self._degen_level in (DEGEN_SEVERE, DEGEN_CRITICAL) and self._last_visual_odom is not None,
            "visual_fused_count": self._visual_fused_count,
        }
        info["gnss_fusion"] = self._gnss_health_snapshot()
        return info

    # -- AI-callable skills ------------------------------------------------

    def _gnss_health_snapshot(self) -> dict[str, Any]:
        """Shared payload for @skill and health() — matches Out port schema."""
        return {
            "enabled": self._gnss_fusion,
            "alignment_locked": self._gnss_map_offset is not None,
            "map_offset": (
                None if self._gnss_map_offset is None
                else [float(self._gnss_map_offset[0]),
                      float(self._gnss_map_offset[1])]
            ),
            "antenna_offset_body": self._gnss_antenna_offset.tolist(),
            "last_residual_m": float(self._gnss_last_residual_m),
            "fused_count": int(self._gnss_fused_count),
            "relock_count": int(self._gnss_relock_count),
            "last_relock_ts": float(self._gnss_last_relock_ts),
            "last_fix_type": (
                self._last_gnss_odom.fix_type.name
                if self._last_gnss_odom is not None else "NONE"
            ),
            "last_gnss_age_s": (
                float(_time.time() - self._last_gnss_rx_ts)
                if self._last_gnss_rx_ts > 0 else float("inf")
            ),
        }

    @skill
    def get_gnss_fusion_status(self) -> str:
        """Return GNSS-SLAM fusion health: alignment, residual, fix quality,
        relock count, and last GNSS sample age. Use this to diagnose outdoor
        drift or suspected GNSS signal loss."""
        import json
        return json.dumps(self._gnss_health_snapshot(), default=str)

    @skill
    def relock_gnss_alignment(self) -> str:
        """Force re-locking of the GNSS↔SLAM alignment offset. The next
        valid RTK fix received while SLAM is healthy will establish a new
        offset. Use this after a known map switch or after diagnosing that
        the current alignment drifted."""
        import json
        prev = (
            None if self._gnss_map_offset is None
            else [float(self._gnss_map_offset[0]),
                  float(self._gnss_map_offset[1])]
        )
        self._gnss_map_offset = None
        self._gnss_residual_history.clear()
        self._gnss_relock_count += 1
        self._gnss_last_relock_ts = _time.time()
        logger.warning(
            "GNSS-SLAM alignment manually relocked (prev=%s)", prev)
        return json.dumps({
            "status": "relocked",
            "previous_offset": prev,
            "relock_count": self._gnss_relock_count,
        })

    @skill
    def set_gnss_fusion(self, enabled: bool) -> str:
        """Enable or disable the GNSS-SLAM fusion runtime switch. When
        disabled, SLAM odometry is published without any GNSS correction
        (equivalent to pure LiDAR-inertial SLAM). Persisted only in this
        process — does not write robot_config.yaml."""
        import json
        self._gnss_fusion = bool(enabled)
        if not self._gnss_fusion:
            # Dropping the alignment is kinder than leaving a stale offset
            self._gnss_map_offset = None
            self._gnss_residual_history.clear()
        logger.warning("GNSS fusion runtime switch: %s",
                        "ON" if self._gnss_fusion else "OFF")
        return json.dumps({
            "status": "ok",
            "enabled": self._gnss_fusion,
        })
