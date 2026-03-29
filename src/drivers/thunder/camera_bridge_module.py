"""CameraBridgeModule — bridges ROS2 camera topics into Python Module pipeline.

Subscribes to ROS2:
  /camera/color/image_raw  — sensor_msgs/Image (BGR8)
  /camera/depth/image_raw  — sensor_msgs/Image (16UC1 mm or 32FC1 m)
  /camera/camera_info      — sensor_msgs/CameraInfo

Publishes to Module ports:
  color_image: Out[Image]           — PerceptionModule, VisualServoModule
  depth_image: Out[Image]           — PerceptionModule, VisualServoModule
  camera_info: Out[CameraIntrinsics]— PerceptionModule, VisualServoModule

Graceful no-op when rclpy unavailable (Windows dev, stub mode).

Usage::

    bp.add(CameraBridgeModule)
    bp.add(CameraBridgeModule, color_topic="/cam/rgb", depth_topic="/cam/depth")
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Optional

import numpy as np

from core.module import Module
from core.stream import Out
from core.msgs.sensor import Image, ImageFormat, CameraIntrinsics
from core.registry import register

logger = logging.getLogger(__name__)


@register("camera_bridge", "default", description="ROS2 camera → Python Module bridge")
class CameraBridgeModule(Module, layer=1):
    """Bridges ROS2 camera topics (RGB + depth + intrinsics) into Module ports.

    On systems without rclpy, starts silently — no crash, no output.
    """

    # -- Outputs --
    color_image: Out[Image]
    depth_image: Out[Image]
    camera_info: Out[CameraIntrinsics]
    alive:       Out[bool]

    def __init__(
        self,
        node_name: str = "camera_bridge",
        color_topic: str = "/camera/color/image_raw",
        depth_topic: str = "/camera/depth/image_raw",
        info_topic: str = "/camera/camera_info",
        spin_rate: float = 30.0,
        qos_depth: int = 5,
        **kw,
    ):
        super().__init__(**kw)
        self._node_name = node_name
        self._color_topic = color_topic
        self._depth_topic = depth_topic
        self._info_topic = info_topic
        self._spin_rate = spin_rate
        self._qos_depth = qos_depth

        self._node = None
        self._executor = None
        self._spin_thread: Optional[threading.Thread] = None
        self._running = False

    def setup(self) -> None:
        try:
            import rclpy
            from rclpy.node import Node
            from rclpy.qos import QoSProfile, ReliabilityPolicy
            from sensor_msgs.msg import Image as ROS2Image, CameraInfo

            if not rclpy.ok():
                rclpy.init()

            qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                depth=self._qos_depth,
            )

            self._node = Node(self._node_name)
            self._node.create_subscription(
                ROS2Image, self._color_topic, self._on_ros2_color, qos)
            self._node.create_subscription(
                ROS2Image, self._depth_topic, self._on_ros2_depth, qos)
            self._node.create_subscription(
                CameraInfo, self._info_topic, self._on_ros2_info, qos)

            logger.info(
                "CameraBridgeModule: node '%s' — color=%s depth=%s info=%s",
                self._node_name, self._color_topic, self._depth_topic, self._info_topic,
            )
        except ImportError:
            logger.info("CameraBridgeModule: rclpy not available, running in stub mode")
        except Exception as e:
            logger.warning("CameraBridgeModule: setup failed: %s", e)

    def start(self) -> None:
        super().start()
        if self._node is None:
            self.alive.publish(False)
            return

        self._running = True
        self._spin_thread = threading.Thread(
            target=self._spin_loop, name="camera_bridge_spin", daemon=True)
        self._spin_thread.start()
        self.alive.publish(True)

    def stop(self) -> None:
        self._running = False
        if self._spin_thread:
            self._spin_thread.join(timeout=3.0)
        if self._executor:
            self._executor.shutdown()
            self._executor = None
        if self._node:
            self._node.destroy_node()
            self._node = None
        super().stop()

    # ── ROS2 callbacks ────────────────────────────────────────────────────────

    def _on_ros2_color(self, msg) -> None:
        try:
            encoding = getattr(msg, "encoding", "bgr8").lower()
            h, w = msg.height, msg.width
            raw = bytes(msg.data)

            if encoding in ("bgr8", "rgb8"):
                arr = np.frombuffer(raw, dtype=np.uint8).reshape(h, w, 3)
                fmt = ImageFormat.BGR if encoding == "bgr8" else ImageFormat.RGB
            elif encoding in ("mono8", "8uc1"):
                arr = np.frombuffer(raw, dtype=np.uint8).reshape(h, w)
                fmt = ImageFormat.GRAY
            else:
                arr = np.frombuffer(raw, dtype=np.uint8)
                fmt = ImageFormat.GRAY

            self.color_image.publish(Image(
                data=arr.copy(), format=fmt,
                ts=time.time(), frame_id=msg.header.frame_id or "camera",
            ))
        except Exception as e:
            logger.debug("CameraBridge: color parse error: %s", e)

    def _on_ros2_depth(self, msg) -> None:
        try:
            encoding = getattr(msg, "encoding", "16uc1").lower()
            h, w = msg.height, msg.width
            raw = bytes(msg.data)

            if encoding in ("16uc1",):
                arr = np.frombuffer(raw, dtype=np.uint16).reshape(h, w)
                fmt = ImageFormat.DEPTH_U16
            elif encoding in ("32fc1",):
                arr = np.frombuffer(raw, dtype=np.float32).reshape(h, w)
                fmt = ImageFormat.DEPTH_F32
            else:
                arr = np.frombuffer(raw, dtype=np.uint16).reshape(h, w)
                fmt = ImageFormat.DEPTH_U16

            self.depth_image.publish(Image(
                data=arr.copy(), format=fmt,
                ts=time.time(), frame_id=msg.header.frame_id or "camera",
            ))
        except Exception as e:
            logger.debug("CameraBridge: depth parse error: %s", e)

    def _on_ros2_info(self, msg) -> None:
        try:
            k = msg.k  # 3x3 intrinsic matrix, row-major
            self.camera_info.publish(CameraIntrinsics(
                fx=float(k[0]), fy=float(k[4]),
                cx=float(k[2]), cy=float(k[5]),
                width=int(msg.width), height=int(msg.height),
            ))
        except Exception as e:
            logger.debug("CameraBridge: info parse error: %s", e)

    # ── Spin loop ─────────────────────────────────────────────────────────────

    def _spin_loop(self) -> None:
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        period = 1.0 / self._spin_rate
        while self._running and rclpy.ok():
            self._executor.spin_once(timeout_sec=period)
