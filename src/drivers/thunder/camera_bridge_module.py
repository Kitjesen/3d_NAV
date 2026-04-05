"""CameraBridgeModule — bridges ROS2 camera topics into Python Module pipeline.

Subscribes to ROS2:
  /camera/color/image_raw  — sensor_msgs/Image (BGR8)
  /camera/depth/image_raw  — sensor_msgs/Image (16UC1 mm or 32FC1 m)
  /camera/camera_info      — sensor_msgs/CameraInfo

Publishes to Module ports:
  color_image: Out[Image]           — already undistorted if distortion present
  depth_image: Out[Image]           — already undistorted (nearest-neighbor remap)
  camera_info: Out[CameraIntrinsics]— with zero distortion (post-undistort)

Undistortion is applied at source so all downstream consumers
(Perception, VisualServo, Reconstruction, DepthVisualOdom) operate
under the correct pinhole camera model without each needing to undistort.

Graceful no-op when rclpy unavailable (Windows dev, stub mode).

Usage::

    bp.add(CameraBridgeModule)
    bp.add(CameraBridgeModule, color_topic="/cam/rgb", depth_topic="/cam/depth")
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Optional, Tuple

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
        self._running = False

        # Undistortion maps — computed once on first camera_info with nonzero distortion
        self._undistort_maps: Optional[Tuple[np.ndarray, np.ndarray]] = None
        self._undistorted_intrinsics: Optional[CameraIntrinsics] = None
        self._K: Optional[np.ndarray] = None  # 3x3 camera matrix (undistorted)

    def setup(self) -> None:
        try:
            from rclpy.node import Node
            from rclpy.qos import QoSProfile, ReliabilityPolicy
            from sensor_msgs.msg import Image as ROS2Image, CameraInfo
            from core.ros2_context import ensure_rclpy, get_shared_executor

            ensure_rclpy()

            qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                depth=self._qos_depth,
            )

            self._node = Node(self._node_name)
            get_shared_executor().add_node(self._node)
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
        self.alive.publish(True)

    def stop(self) -> None:
        self._running = False
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

            # Apply undistortion at source (linear interpolation for color)
            if self._undistort_maps is not None:
                import cv2
                arr = cv2.remap(arr, *self._undistort_maps, cv2.INTER_LINEAR)

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

            # Apply undistortion at source (nearest-neighbor for depth to avoid blending)
            if self._undistort_maps is not None:
                import cv2
                arr = cv2.remap(arr, *self._undistort_maps, cv2.INTER_NEAREST)

            self.depth_image.publish(Image(
                data=arr.copy(), format=fmt,
                ts=time.time(), frame_id=msg.header.frame_id or "camera",
            ))
        except Exception as e:
            logger.debug("CameraBridge: depth parse error: %s", e)

    def _on_ros2_info(self, msg) -> None:
        try:
            k = msg.k  # 3x3 intrinsic matrix, row-major
            fx, fy = float(k[0]), float(k[4])
            cx, cy = float(k[2]), float(k[5])
            w, h = int(msg.width), int(msg.height)

            # Parse distortion coefficients (plumb_bob: k1, k2, p1, p2, k3)
            d = getattr(msg, "d", None) or []
            dk1 = float(d[0]) if len(d) > 0 else 0.0
            dk2 = float(d[1]) if len(d) > 1 else 0.0
            dp1 = float(d[2]) if len(d) > 2 else 0.0
            dp2 = float(d[3]) if len(d) > 3 else 0.0
            dk3 = float(d[4]) if len(d) > 4 else 0.0

            # Build undistortion maps once (idempotent)
            if self._undistort_maps is None and any(
                abs(v) > 1e-12 for v in (dk1, dk2, dp1, dp2, dk3)
            ):
                self._init_undistort_maps(fx, fy, cx, cy, w, h,
                                          dk1, dk2, dp1, dp2, dk3)

            # Publish intrinsics — with zero distortion since images are pre-undistorted
            if self._undistorted_intrinsics is not None:
                self.camera_info.publish(self._undistorted_intrinsics)
            else:
                self.camera_info.publish(CameraIntrinsics(
                    fx=fx, fy=fy, cx=cx, cy=cy,
                    width=w, height=h,
                    dist_k1=dk1, dist_k2=dk2,
                    dist_p1=dp1, dist_p2=dp2, dist_k3=dk3,
                ))
        except Exception as e:
            logger.debug("CameraBridge: info parse error: %s", e)

    def _init_undistort_maps(
        self, fx, fy, cx, cy, w, h, k1, k2, p1, p2, k3,
    ) -> None:
        """Compute undistortion remap tables. Called once."""
        try:
            import cv2
            K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
            D = np.array([k1, k2, p1, p2, k3], dtype=np.float64)
            self._undistort_maps = cv2.initUndistortRectifyMap(
                K, D, None, K, (w, h), cv2.CV_16SC2,
            )
            self._K = K
            # After undistortion, distortion coefficients are zero
            self._undistorted_intrinsics = CameraIntrinsics(
                fx=fx, fy=fy, cx=cx, cy=cy,
                width=w, height=h,
                dist_k1=0.0, dist_k2=0.0,
                dist_p1=0.0, dist_p2=0.0, dist_k3=0.0,
            )
            logger.info(
                "CameraBridge: undistortion maps computed (k1=%.4f, k2=%.4f) — "
                "all published images are now rectified",
                k1, k2,
            )
        except ImportError:
            logger.warning("CameraBridge: cv2 not available, skipping undistortion")

    # ── Spin loop ─────────────────────────────────────────────────────────────

    # Spin handled by shared executor (core.ros2_context)
