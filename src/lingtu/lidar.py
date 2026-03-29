"""LiDAR — import and use, hardware details hidden."""

from __future__ import annotations

import logging
import threading
import time
from typing import Callable, List, Optional

import numpy as np

logger = logging.getLogger(__name__)


class LiDAR:
    """Livox MID-360 LiDAR — plug and play.

    Usage::

        lidar = LiDAR()
        lidar.start()
        cloud = lidar.get_cloud()    # numpy (N,3)
        lidar.on_cloud(my_callback)  # register callback
        lidar.stop()
    """

    def __init__(self, ip: str = "192.168.1.115", port: int = 56300):
        self._ip = ip
        self._port = port
        self._native = None  # NativeModule for livox_driver
        self._callbacks: List[Callable] = []
        self._latest_cloud: Optional[np.ndarray] = None
        self._running = False
        self._started = False

    def start(self) -> "LiDAR":
        """Start LiDAR driver. Returns self for chaining."""
        if self._started:
            return self
        try:
            from core.config import get_config
            from core.native_factories import livox_driver
            cfg = get_config()
            self._native = livox_driver(cfg)
            self._native.setup()
            self._native.start()
            self._started = True
            logger.info("LiDAR started (ip=%s)", self._ip)
        except Exception as e:
            logger.error("LiDAR start failed: %s", e)
        return self

    def stop(self) -> None:
        if self._native:
            self._native.stop()
            self._native = None
        self._started = False

    def on_cloud(self, callback: Callable) -> None:
        """Register a point cloud callback: fn(numpy_Nx3)."""
        self._callbacks.append(callback)

    def get_cloud(self) -> Optional[np.ndarray]:
        """Get latest point cloud (N,3). None if no data yet."""
        return self._latest_cloud

    @property
    def is_running(self) -> bool:
        return self._started

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *args):
        self.stop()

    def __repr__(self):
        return "LiDAR(ip=%s, running=%s)" % (self._ip, self._started)
