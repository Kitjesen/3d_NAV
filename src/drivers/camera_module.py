"""CameraModule — camera sensor as a pluggable Module.

Backends:
  "usb"       — USB camera via OpenCV (cv2.VideoCapture)
  "realsense" — Intel RealSense D435i (RGB-D)
  "sim"       — Simulated images (testing)

Usage::

    bp.add(CameraModule, backend="usb", device_id=0)
    # Out: image, depth
"""

from __future__ import annotations

import logging
import time
from typing import Any, Dict, Optional

import numpy as np

from core.module import Module
from core.stream import Out
from core.msgs.sensor import Image, CameraIntrinsics
from core.registry import register

logger = logging.getLogger(__name__)


@register("camera", "usb", description="USB camera via OpenCV")
@register("camera", "realsense", description="Intel RealSense D435i RGB-D")
@register("camera", "sim", description="Simulated camera images")
class CameraModule(Module, layer=1):
    """Camera sensor driver — publishes RGB images and optional depth."""

    # -- Outputs --
    image: Out[Image]
    depth: Out[Image]
    intrinsics: Out[CameraIntrinsics]
    alive: Out[bool]

    def __init__(self, backend: str = "usb", device_id: int = 0,
                 width: int = 640, height: int = 480, fps: float = 30.0, **kw):
        super().__init__(**kw)
        self._backend = backend
        self._device_id = device_id
        self._width = width
        self._height = height
        self._fps = fps
        self._cap = None

    def setup(self):
        if self._backend == "usb":
            self._setup_usb()
        elif self._backend == "realsense":
            self._setup_realsense()
        elif self._backend == "sim":
            logger.info("CameraModule [sim]: simulated images")
        else:
            logger.warning("CameraModule: unknown backend '%s'", self._backend)

    def _setup_usb(self):
        """Setup USB camera via OpenCV."""
        try:
            import cv2
            self._cap = cv2.VideoCapture(self._device_id)
            if self._cap.isOpened():
                self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._width)
                self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)
                logger.info("CameraModule [usb]: opened device %d (%dx%d)",
                            self._device_id, self._width, self._height)
            else:
                logger.warning("CameraModule [usb]: failed to open device %d", self._device_id)
                self._cap = None
        except ImportError:
            logger.warning("CameraModule [usb]: cv2 not available")

    def _setup_realsense(self):
        """Setup Intel RealSense (lazy import)."""
        try:
            import pyrealsense2 as rs
            logger.info("CameraModule [realsense]: pyrealsense2 available")
        except ImportError:
            logger.warning("CameraModule [realsense]: pyrealsense2 not available")

    def start(self):
        super().start()
        self.alive.publish(True)

    def stop(self):
        if self._cap is not None:
            self._cap.release()
            self._cap = None
        self.alive.publish(False)
        super().stop()

    def capture(self) -> Optional[np.ndarray]:
        """Capture one frame (usb backend). Returns BGR ndarray or None."""
        if self._cap is not None and self._cap.isOpened():
            ret, frame = self._cap.read()
            if ret:
                self.image.publish(Image(data=frame, ts=time.time()))
                return frame
        return None

    def publish_sim_image(self, width: int = 640, height: int = 480):
        """Publish a simulated image (for testing)."""
        bgr = np.random.randint(0, 255, (height, width, 3), dtype=np.uint8)
        self.image.publish(Image(data=bgr, ts=time.time()))

    def health(self) -> Dict[str, Any]:
        info = super().port_summary()
        info["camera"] = {
            "backend": self._backend,
            "device_id": self._device_id,
            "resolution": f"{self._width}x{self._height}",
        }
        return info
