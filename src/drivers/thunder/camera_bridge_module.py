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
from typing import Any, Dict, Optional, Tuple

import numpy as np

from core.module import Module
from core.msgs.sensor import CameraIntrinsics, Image, ImageFormat
from core.registry import register
from core.stream import Out

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
        self._rclpy_available = False

        # Undistortion maps — computed once on first camera_info with nonzero distortion
        self._undistort_maps: tuple[np.ndarray, np.ndarray] | None = None
        self._undistorted_intrinsics: CameraIntrinsics | None = None
        self._K: np.ndarray | None = None  # 3x3 camera matrix (undistorted)

        # Auto-recovery: monitor data freshness, reconnect if stale
        self._last_color_ts: float = 0.0
        self._last_depth_ts: float = 0.0
        self._stale_timeout: float = kw.get("stale_timeout", 5.0)
        self._max_reconnects: int = kw.get("max_reconnects", 10)
        self._reconnect_count: int = 0
        self._watchdog_thread: threading.Thread | None = None
        self._shutdown_event = threading.Event()

    def setup(self) -> None:
        self._create_ros2_node()

    def _create_ros2_node(self) -> bool:
        """Create ROS2 node and subscriptions. Returns True on success."""
        try:
            from rclpy.node import Node
            from rclpy.qos import QoSProfile, ReliabilityPolicy
            from sensor_msgs.msg import CameraInfo
            from sensor_msgs.msg import Image as ROS2Image

            from core.ros2_context import ensure_rclpy, get_shared_executor

            ensure_rclpy()
            self._rclpy_available = True

            qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                depth=self._qos_depth,
            )

            # Destroy old node if exists (reconnection path)
            if self._node is not None:
                try:
                    self._node.destroy_node()
                except Exception:
                    pass

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
            return True
        except ImportError:
            logger.info("CameraBridgeModule: rclpy not available, running in stub mode")
            return False
        except Exception as e:
            logger.warning("CameraBridgeModule: setup failed: %s", e)
            return False

    def start(self) -> None:
        super().start()
        if self._node is None:
            self.alive.publish(False)
            return
        self._running = True
        self.alive.publish(True)
        # Start stale-data watchdog for auto-recovery
        self._shutdown_event.clear()
        self._watchdog_thread = threading.Thread(
            target=self._watchdog_loop, daemon=True,
            name="camera-bridge-watchdog")
        self._watchdog_thread.start()

    def stop(self) -> None:
        self._running = False
        self._shutdown_event.set()
        if self._watchdog_thread and self._watchdog_thread.is_alive():
            self._watchdog_thread.join(timeout=2.0)
        self._watchdog_thread = None
        if self._node:
            try:
                self._node.destroy_node()
            except Exception:
                pass
            self._node = None
        super().stop()

    def health(self) -> dict[str, Any]:
        info = super().port_summary()
        info["frame_count"] = getattr(self, "_frame_count", 0)
        now = time.time()
        if self._last_color_ts > 0:
            dt = now - self._last_color_ts
            info["fps"] = round(1.0 / dt, 1) if dt > 0 else 0.0
        else:
            info["fps"] = 0.0
        return info

    # ── Auto-recovery watchdog ────────────────────────────────────────────────
    #
    # Three escalation levels:
    #   L1  Reconnect rclpy subscription  (QoS / topic mismatch)
    #   L2  systemctl restart camera      (camera node crash)
    #   L3  USB reset + restart camera    (hardware disconnect)

    def _watchdog_loop(self) -> None:
        """Monitor data freshness. Escalate recovery on repeated failures."""
        startup_grace = 30.0  # seconds after start before checking "never received"
        start_time = time.time()

        while not self._shutdown_event.is_set():
            self._shutdown_event.wait(timeout=3.0)
            if not self._running or not self._rclpy_available:
                continue

            now = time.time()

            # Determine if camera is stale
            if self._last_color_ts > 0:
                color_age = now - self._last_color_ts
                is_stale = color_age > self._stale_timeout
            else:
                # Never received a frame — wait for startup grace period
                is_stale = (now - start_time) > startup_grace

            if not is_stale:
                if self._reconnect_count > 0:
                    logger.info("CameraBridge: data resumed after %d recovery attempt(s)",
                                self._reconnect_count)
                    self._reconnect_count = 0
                continue

            if self._reconnect_count >= self._max_reconnects:
                if self._reconnect_count == self._max_reconnects:
                    logger.error(
                        "CameraBridge: max recoveries (%d) reached, giving up",
                        self._max_reconnects)
                    self.alive.publish(False)
                    self._reconnect_count += 1
                continue

            self._reconnect_count += 1
            level = self._recovery_level(self._reconnect_count)
            backoff = min(5.0 * self._reconnect_count, 30.0)

            if level == 1:
                logger.warning(
                    "CameraBridge: L1 recovery — reconnect rclpy (%d/%d)",
                    self._reconnect_count, self._max_reconnects)
                if self._create_ros2_node():
                    self.alive.publish(True)
                else:
                    self.alive.publish(False)
            elif level == 2:
                logger.warning(
                    "CameraBridge: L2 recovery — restart camera.service (%d/%d)",
                    self._reconnect_count, self._max_reconnects)
                self._restart_camera_service()
                self._shutdown_event.wait(timeout=8.0)
                if self._shutdown_event.is_set():
                    break
                self._create_ros2_node()
            else:
                logger.warning(
                    "CameraBridge: L3 recovery — USB reset + restart (%d/%d)",
                    self._reconnect_count, self._max_reconnects)
                self._usb_reset_camera()
                self._shutdown_event.wait(timeout=5.0)
                self._restart_camera_service()
                self._shutdown_event.wait(timeout=8.0)
                if self._shutdown_event.is_set():
                    break
                self._create_ros2_node()

            self._shutdown_event.wait(timeout=backoff)

    @staticmethod
    def _recovery_level(attempt: int) -> int:
        """L1 for first 2 attempts, L2 for next 3, L3 after that."""
        if attempt <= 2:
            return 1
        if attempt <= 5:
            return 2
        return 3

    @staticmethod
    def _restart_camera_service() -> None:
        """Restart the camera systemd service."""
        import subprocess
        try:
            subprocess.run(
                ["sudo", "systemctl", "restart", "camera"],
                capture_output=True, timeout=10,
            )
            logger.info("CameraBridge: camera.service restarted")
        except Exception as e:
            logger.warning("CameraBridge: failed to restart camera.service: %s", e)

    @staticmethod
    def _usb_reset_camera() -> None:
        """Reset Orbbec USB device without physical unplug."""
        import subprocess
        try:
            # Find Orbbec device in sysfs (vendor 2bc5)
            result = subprocess.run(
                ["bash", "-c",
                 "grep -rl '2bc5' /sys/bus/usb/devices/*/idVendor 2>/dev/null "
                 "| head -1 | xargs dirname"],
                capture_output=True, text=True, timeout=5,
            )
            dev_path = result.stdout.strip()
            if dev_path:
                auth_path = f"{dev_path}/authorized"
                # Power cycle: deauthorize → wait → reauthorize
                subprocess.run(
                    ["bash", "-c", f"echo 0 | sudo tee {auth_path} && "
                     f"sleep 2 && echo 1 | sudo tee {auth_path}"],
                    capture_output=True, timeout=10,
                )
                logger.info("CameraBridge: USB reset via %s", auth_path)
            else:
                logger.warning("CameraBridge: Orbbec USB device not found in sysfs")
        except Exception as e:
            logger.warning("CameraBridge: USB reset failed: %s", e)

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

            now = time.time()
            self._last_color_ts = now
            self.color_image.publish(Image(
                data=arr.copy(), format=fmt,
                ts=now, frame_id=msg.header.frame_id or "camera",
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

            now = time.time()
            self._last_depth_ts = now
            self.depth_image.publish(Image(
                data=arr.copy(), format=fmt,
                ts=now, frame_id=msg.header.frame_id or "camera",
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
