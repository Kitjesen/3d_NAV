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
import numpy as np

from core.module import Module
from core.stream import Out
from core.msgs.nav import Odometry
from core.msgs.sensor import PointCloud
from core.msgs.geometry import Pose, Vector3, Quaternion
from core.registry import register

logger = logging.getLogger(__name__)


@register("slam_bridge", "default", description="DDS SLAM → Python Module bridge")
class SlamBridgeModule(Module, layer=1):
    """Bridges SLAM point cloud and odometry into Module ports via DDS.

    Tries cyclonedds first (lightweight), falls back to rclpy.
    On systems without either, starts silently in stub mode.
    """

    map_cloud: Out[PointCloud]
    odometry:  Out[Odometry]
    alive:     Out[bool]

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
            logger.info("SlamBridgeModule: using rclpy (fallback)")
            return True
        except (ImportError, Exception) as e:
            logger.debug("SlamBridgeModule: rclpy unavailable: %s", e)
            return False

    def start(self) -> None:
        super().start()
        if self._reader:
            self._reader.spin_background()
            self.alive.publish(True)
        elif self._rclpy_node:
            self.alive.publish(True)
        else:
            self.alive.publish(False)

    def stop(self) -> None:
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
            self.odometry.publish(Odometry(
                pose=Pose(
                    position=Vector3(x=p.x, y=p.y, z=p.z),
                    orientation=Quaternion(x=q.x, y=q.y, z=q.z, w=q.w),
                ),
                vx=t.linear.x, vy=t.linear.y, vz=t.linear.z,
                wx=t.angular.x, wy=t.angular.y, wz=t.angular.z,
            ))
        except Exception as e:
            logger.debug("SlamBridge dds odom error: %s", e)

    def _on_dds_cloud(self, msg) -> None:
        """DDS_PointCloud2 → Module PointCloud."""
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
                self.map_cloud.publish(PointCloud.from_numpy(xyz, frame_id="map"))
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
                self.map_cloud.publish(PointCloud.from_numpy(xyz, frame_id="map"))
        except Exception as e:
            logger.debug("SlamBridge rclpy cloud error: %s", e)
