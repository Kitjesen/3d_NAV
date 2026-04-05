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
import numpy as np
from typing import Any, Dict, Optional

from core.module import Module
from core.stream import Out
from core.msgs.nav import Odometry
from core.msgs.sensor import PointCloud2
from core.msgs.geometry import Pose, Vector3, Quaternion
from core.registry import register

logger = logging.getLogger(__name__)

# Localization health states
LOC_UNINIT = "UNINIT"        # No data received yet
LOC_TRACKING = "TRACKING"    # Receiving data, quality OK
LOC_DEGRADED = "DEGRADED"    # Receiving data, quality poor (cloud timeout or degeneracy)
LOC_LOST = "LOST"            # No odometry for > timeout

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

    map_cloud: Out[PointCloud2]
    odometry:  Out[Odometry]
    alive:     Out[bool]
    localization_status: Out[dict]

    def __init__(
        self,
        cloud_topic: str = "/nav/map_cloud",
        odom_topic: str = "/nav/odometry",
        **kw,
    ):
        super().__init__(**kw)
        self._cloud_topic = cloud_topic
        self._odom_topic = odom_topic
        self._reader = None
        self._rclpy_node = None  # fallback

        # Localization health watchdog
        self._last_odom_time: float = 0.0
        self._last_cloud_time: float = 0.0
        self._loc_state: str = LOC_UNINIT
        self._odom_timeout: float = kw.get("odom_timeout", 2.0)
        self._cloud_timeout: float = kw.get("cloud_timeout", 5.0)
        self._watchdog_hz: float = kw.get("watchdog_hz", 2.0)
        self._shutdown_event = threading.Event()
        self._watchdog_thread: Optional[threading.Thread] = None

        # SLAM degeneracy detection
        self._degeneracy_topic: str = kw.get("degeneracy_topic", "/slam/degeneracy")
        self._quality_topic: str = kw.get("quality_topic", "/localization_quality")
        self._degen_level: str = DEGEN_NONE
        self._icp_fitness: float = 0.0
        self._effective_ratio: float = 1.0  # effective features / total features
        self._eigenvalue_ratio: float = 0.0  # min/max eigenvalue of Hessian
        self._last_degen_time: float = 0.0
        # Degeneracy thresholds (tunable)
        self._fitness_warn: float = kw.get("fitness_warn", 0.15)
        self._fitness_critical: float = kw.get("fitness_critical", 0.3)
        self._feature_ratio_warn: float = kw.get("feature_ratio_warn", 0.3)
        self._feature_ratio_critical: float = kw.get("feature_ratio_critical", 0.1)
        self._eigen_ratio_warn: float = kw.get("eigen_ratio_warn", 100.0)  # condition number

    def setup(self) -> None:
        # Try cyclonedds first (lightweight, no ROS2 env needed)
        if self._try_cyclonedds():
            return
        # Fallback to rclpy
        if self._try_rclpy():
            return
        logger.info("SlamBridgeModule: no DDS backend available, stub mode")

    def _try_cyclonedds(self) -> bool:
        try:
            from core.dds import ROS2TopicReader
            self._reader = ROS2TopicReader()
            self._reader.on_odometry(self._odom_topic, self._on_dds_odom)
            self._reader.on_pointcloud2(self._cloud_topic, self._on_dds_cloud)
            logger.info("SlamBridgeModule: using cyclonedds (lightweight)")
            return True
        except ImportError:
            return False

    def _try_rclpy(self) -> bool:
        try:
            from rclpy.node import Node
            from rclpy.qos import QoSProfile, ReliabilityPolicy
            from sensor_msgs.msg import PointCloud2
            from nav_msgs.msg import Odometry as ROS2Odom
            from core.ros2_context import ensure_rclpy, get_shared_executor

            ensure_rclpy()
            qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)
            self._rclpy_node = Node("slam_bridge")
            get_shared_executor().add_node(self._rclpy_node)
            self._rclpy_node.create_subscription(PointCloud2, self._cloud_topic, self._on_rclpy_cloud, qos)
            self._rclpy_node.create_subscription(ROS2Odom, self._odom_topic, self._on_rclpy_odom, qos)
            # Subscribe to degeneracy metrics if available
            self._subscribe_degeneracy_rclpy(self._rclpy_node, qos)
            logger.info("SlamBridgeModule: using rclpy (fallback)")
            return True
        except (ImportError, Exception) as e:
            logger.debug("SlamBridgeModule: rclpy unavailable: %s", e)
            return False

    def _subscribe_degeneracy_rclpy(self, node, qos) -> None:
        """Subscribe to SLAM degeneracy metrics via rclpy (best-effort)."""
        try:
            from rclpy.qos import QoSProfile, ReliabilityPolicy
            from std_msgs.msg import Float32
            sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=1)
            node.create_subscription(
                Float32, self._quality_topic, self._on_rclpy_quality, sensor_qos)
            node.create_subscription(
                Float32, self._degeneracy_topic, self._on_rclpy_degeneracy, sensor_qos)
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

    def _on_dds_odom(self, msg) -> None:
        """DDS_Odometry → Module Odometry."""
        try:
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            t = msg.twist.twist
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.odometry.publish(Odometry(
                pose=Pose(
                    position=Vector3(x=p.x, y=p.y, z=p.z),
                    orientation=Quaternion(x=q.x, y=q.y, z=q.z, w=q.w),
                ),
                vx=t.linear.x, vy=t.linear.y, vz=t.linear.z,
                wx=t.angular.x, wy=t.angular.y, wz=t.angular.z,
                ts=stamp,
            ))
            self._last_odom_time = _time.time()
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
                self.map_cloud.publish(PointCloud2.from_numpy(xyz, frame_id="map"))
                self._last_cloud_time = _time.time()
        except Exception as e:
            logger.debug("SlamBridge dds cloud error: %s", e)

    # ── rclpy callbacks (fallback) ───────────────────────────────────────

    def _on_rclpy_odom(self, msg) -> None:
        try:
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            t = msg.twist.twist
            self.odometry.publish(Odometry(
                pose=Pose(
                    position=Vector3(x=float(p.x), y=float(p.y), z=float(p.z)),
                    orientation=Quaternion(x=float(q.x), y=float(q.y), z=float(q.z), w=float(q.w)),
                ),
                vx=float(t.linear.x), vy=float(t.linear.y), vz=float(t.linear.z),
                wx=float(t.angular.x), wy=float(t.angular.y), wz=float(t.angular.z),
            ))
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
                self.map_cloud.publish(PointCloud2.from_numpy(xyz, frame_id="map"))
                self._last_cloud_time = _time.time()
        except Exception as e:
            logger.debug("SlamBridge rclpy cloud error: %s", e)

    # ── Degeneracy metric callbacks ─────────────────────────────────────────

    def _on_rclpy_quality(self, msg) -> None:
        """ICP fitness score from localizer (lower = better)."""
        self._icp_fitness = float(msg.data)
        self._last_degen_time = _time.time()
        self._update_degeneracy_level()

    def _on_rclpy_degeneracy(self, msg) -> None:
        """Effective feature ratio from SLAM (1.0 = all features valid)."""
        self._effective_ratio = float(msg.data)
        self._last_degen_time = _time.time()
        self._update_degeneracy_level()

    def _update_degeneracy_level(self) -> None:
        """Classify degeneracy severity from SLAM quality metrics."""
        prev = self._degen_level

        # Critical: ICP fitness very bad OR almost no features
        if (self._icp_fitness > self._fitness_critical
                or self._effective_ratio < self._feature_ratio_critical):
            level = DEGEN_CRITICAL
        # Severe: ICP poor OR low features
        elif (self._icp_fitness > self._fitness_warn
              or self._effective_ratio < self._feature_ratio_warn):
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
                "SLAM degeneracy: %s -> %s (fitness=%.3f, feat_ratio=%.2f)",
                prev, level, self._icp_fitness, self._effective_ratio)
        elif level == DEGEN_NONE and prev != DEGEN_NONE:
            logger.info("SLAM degeneracy cleared: %s -> NONE", prev)

    # ── Localization health watchdog ──────────────────────────────��──────

    def _watchdog_loop(self) -> None:
        """Periodic localization health check."""
        interval = 1.0 / self._watchdog_hz
        while not self._shutdown_event.is_set():
            now = _time.time()
            odom_age = (now - self._last_odom_time) if self._last_odom_time > 0 else float("inf")
            cloud_age = (now - self._last_cloud_time) if self._last_cloud_time > 0 else float("inf")

            if self._last_odom_time == 0.0:
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

            # Reduce confidence based on degeneracy
            if self._degen_level == DEGEN_SEVERE:
                confidence = min(confidence, 0.4)
            elif self._degen_level == DEGEN_MILD:
                confidence = min(confidence, 0.7)

            if new_state != self._loc_state:
                if new_state == LOC_LOST:
                    logger.warning("Localization LOST: no odometry for %.1fs", odom_age)
                elif new_state == LOC_DEGRADED and self._degen_level != DEGEN_NONE:
                    logger.warning("Localization DEGRADED: SLAM degeneracy %s",
                                   self._degen_level)
                elif new_state == LOC_TRACKING and self._loc_state in (LOC_LOST, LOC_DEGRADED):
                    logger.info("Localization recovered -> TRACKING")
                self._loc_state = new_state

            self.localization_status.publish({
                "state": self._loc_state,
                "odom_age_ms": round(odom_age * 1000, 1),
                "cloud_age_ms": round(cloud_age * 1000, 1),
                "confidence": round(confidence, 2),
                "degeneracy": self._degen_level,
                "icp_fitness": round(self._icp_fitness, 4),
                "effective_ratio": round(self._effective_ratio, 3),
            })

            self._shutdown_event.wait(timeout=interval)

    def health(self) -> Dict[str, Any]:
        info = super().port_summary()
        now = _time.time()
        info["localization"] = {
            "state": self._loc_state,
            "odom_age_ms": round((now - self._last_odom_time) * 1000, 1) if self._last_odom_time else -1,
            "cloud_age_ms": round((now - self._last_cloud_time) * 1000, 1) if self._last_cloud_time else -1,
            "degeneracy": self._degen_level,
            "icp_fitness": round(self._icp_fitness, 4),
            "effective_ratio": round(self._effective_ratio, 3),
        }
        return info
