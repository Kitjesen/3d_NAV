"""SlamBridgeModule — bridges ROS2 SLAM outputs into the Python Module pipeline.

Subscribes to ROS2:
  /nav/map_cloud       — sensor_msgs/PointCloud2 (XYZI) from SLAM
  /nav/odometry        — nav_msgs/Odometry from SLAM (higher quality than driver)

Publishes to Module ports:
  map_cloud: Out[PointCloud]  — consumed by OccupancyGridModule, ElevationMapModule, TerrainModule
  odometry: Out[Odometry]     — SLAM-corrected odometry (overrides driver dead-reckoning)

Graceful degradation: if rclpy is not available (Windows dev, stub mode),
the module starts silently with no ROS2 subscriptions.  Ports stay empty.

Usage::

    bp.add(SlamBridgeModule)                              # defaults
    bp.add(SlamBridgeModule, cloud_topic="/my/cloud")     # custom topic
"""

from __future__ import annotations

import logging
import struct
import threading
import time
from typing import Optional

import numpy as np

from core.module import Module
from core.stream import Out
from core.msgs.nav import Odometry
from core.msgs.sensor import PointCloud
from core.msgs.geometry import Pose, Vector3, Quaternion
from core.registry import register

logger = logging.getLogger(__name__)

_XYZI_POINT_STEP = 16  # 4 × float32


@register("slam_bridge", "default", description="ROS2 SLAM → Python Module bridge")
class SlamBridgeModule(Module, layer=1):
    """Bridges ROS2 SLAM point cloud and odometry into Module ports.

    On systems without rclpy, starts silently — no crash, no output.
    """

    # -- Outputs --
    map_cloud: Out[PointCloud]
    odometry:  Out[Odometry]   # SLAM-corrected, overrides driver dead-reckoning
    alive:     Out[bool]

    def __init__(
        self,
        node_name: str = "slam_bridge",
        cloud_topic: str = "/nav/map_cloud",
        odom_topic: str = "/nav/odometry",
        spin_rate: float = 50.0,
        qos_depth: int = 10,
        **kw,
    ):
        super().__init__(**kw)
        self._node_name = node_name
        self._cloud_topic = cloud_topic
        self._odom_topic = odom_topic
        self._spin_rate = spin_rate
        self._qos_depth = qos_depth

        self._node = None
        self._spin_thread: Optional[threading.Thread] = None
        self._running = False

    def setup(self) -> None:
        try:
            import rclpy
            from rclpy.node import Node
            from rclpy.qos import QoSProfile, ReliabilityPolicy
            from sensor_msgs.msg import PointCloud2
            from nav_msgs.msg import Odometry as ROS2Odom

            if not rclpy.ok():
                rclpy.init()

            qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                depth=self._qos_depth,
            )

            self._node = Node(self._node_name)
            self._node.create_subscription(
                PointCloud2, self._cloud_topic, self._on_ros2_cloud, qos)
            self._node.create_subscription(
                ROS2Odom, self._odom_topic, self._on_ros2_odom, qos)

            logger.info(
                "SlamBridgeModule: node '%s' — cloud=%s odom=%s",
                self._node_name, self._cloud_topic, self._odom_topic,
            )
        except ImportError:
            logger.info("SlamBridgeModule: rclpy not available, running in stub mode")
        except Exception as e:
            logger.warning("SlamBridgeModule: setup failed: %s", e)

    def start(self) -> None:
        super().start()
        if self._node is None:
            self.alive.publish(False)
            return

        self._running = True
        self._spin_thread = threading.Thread(
            target=self._spin_loop, name="slam_bridge_spin", daemon=True)
        self._spin_thread.start()
        self.alive.publish(True)

    def stop(self) -> None:
        self._running = False
        if self._spin_thread:
            self._spin_thread.join(timeout=3.0)
        if self._node:
            self._node.destroy_node()
            self._node = None
        super().stop()

    # ── ROS2 callbacks ────────────────────────────────────────────────────────

    def _on_ros2_cloud(self, msg) -> None:
        """Convert sensor_msgs/PointCloud2 (XYZI) → PointCloud and publish."""
        try:
            n_points = msg.width * msg.height
            if n_points == 0:
                return

            step = msg.point_step
            data = np.frombuffer(msg.data, dtype=np.uint8)

            if step == _XYZI_POINT_STEP:
                # Fast path: XYZI packed as 4×float32
                pts = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, 4)
                xyz = pts[:, :3].copy()
            else:
                # Generic: extract x,y,z offsets from fields
                offsets = {}
                for f in msg.fields:
                    offsets[f.name] = f.offset
                if not all(k in offsets for k in ("x", "y", "z")):
                    return
                xyz = np.zeros((n_points, 3), dtype=np.float32)
                for i in range(n_points):
                    base = i * step
                    xyz[i, 0] = struct.unpack_from("<f", data, base + offsets["x"])[0]
                    xyz[i, 1] = struct.unpack_from("<f", data, base + offsets["y"])[0]
                    xyz[i, 2] = struct.unpack_from("<f", data, base + offsets["z"])[0]

            # Filter NaN/inf
            valid = np.isfinite(xyz).all(axis=1)
            xyz = xyz[valid]

            if xyz.shape[0] > 0:
                self.map_cloud.publish(PointCloud.from_numpy(xyz, frame_id="map"))

        except Exception as e:
            logger.debug("SlamBridge: cloud parse error: %s", e)

    def _on_ros2_odom(self, msg) -> None:
        """Convert nav_msgs/Odometry → Odometry and publish."""
        try:
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            t = msg.twist.twist
            odom = Odometry(
                pose=Pose(
                    position=Vector3(x=p.x, y=p.y, z=p.z),
                    orientation=Quaternion(x=q.x, y=q.y, z=q.z, w=q.w),
                ),
            )
            self.odometry.publish(odom)
        except Exception as e:
            logger.debug("SlamBridge: odom parse error: %s", e)

    # ── Spin loop ─────────────────────────────────────────────────────────────

    def _spin_loop(self) -> None:
        import rclpy
        period = 1.0 / self._spin_rate
        while self._running and rclpy.ok():
            rclpy.spin_once(self._node, timeout_sec=period)
