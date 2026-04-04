"""Lidar — plug-and-play Livox MID-360 interface.

Usage::

    lidar = Lidar()
    lidar.connect("192.168.1.115")

    # callback style
    lidar.on_cloud(lambda pts: print(pts.shape))   # pts: numpy (N, 4) x,y,z,intensity

    # polling style
    import time
    time.sleep(0.5)
    cloud = lidar.get_cloud()     # numpy (N, 4) or None if no data yet

    lidar.disconnect()

    # context manager
    with Lidar("192.168.1.115") as lidar:
        cloud = lidar.get_cloud()
"""

from __future__ import annotations

import logging
import threading
import time
from copy import deepcopy
from typing import Callable, List, Optional

import numpy as np

from ._dds import HAS_LIVOX_IDL, LivoxCustomMsg

logger = logging.getLogger(__name__)


class Lidar:
    """Livox MID-360 LiDAR — connect, get clouds, disconnect.

    The IP given to ``connect()`` (or the constructor) overrides
    whatever is in robot_config.yaml for this instance only.
    """

    def __init__(self, ip: Optional[str] = None):
        self._ip = ip
        self._native = None
        self._dds = None
        self._callbacks: List[Callable[[np.ndarray], None]] = []
        self._latest_cloud: Optional[np.ndarray] = None
        self._lock = threading.Lock()
        self._connected = False

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def connect(self, ip: Optional[str] = None) -> "Lidar":
        """Start the LiDAR driver and begin streaming point clouds.

        Args:
            ip: LiDAR IP address, e.g. "192.168.1.115".
                Overrides robot_config.yaml for this session.
                Falls back to constructor IP, then config file.

        Returns:
            self — for chaining: ``Lidar().connect("192.168.1.115").on_cloud(fn)``
        """
        if self._connected:
            return self

        if ip:
            self._ip = ip

        cfg = self._build_config()
        self._start_native_driver(cfg)
        self._start_dds_bridge()
        self._connected = True
        logger.info("Lidar connected (ip=%s)", cfg.lidar.lidar_ip)
        return self

    def disconnect(self) -> None:
        """Stop the LiDAR driver and DDS bridge."""
        if self._dds:
            self._dds.stop()
            self._dds = None
        if self._native:
            self._native.stop()
            self._native = None
        self._connected = False
        logger.info("Lidar disconnected")

    def on_cloud(self, callback: Callable[[np.ndarray], None]) -> "Lidar":
        """Register a callback fired on every new point cloud frame.

        Args:
            callback: fn(pts) where pts is numpy (N, 4): x, y, z, intensity.

        Returns:
            self — for chaining.
        """
        self._callbacks.append(callback)
        return self

    def get_cloud(self) -> Optional[np.ndarray]:
        """Return the latest point cloud as numpy (N, 4): x, y, z, intensity.

        Returns None if no data has arrived yet.
        """
        with self._lock:
            return self._latest_cloud

    def wait_for_cloud(self, timeout: float = 5.0) -> Optional[np.ndarray]:
        """Block until the first cloud arrives (or timeout).

        Returns:
            numpy (N, 4) or None on timeout.
        """
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            cloud = self.get_cloud()
            if cloud is not None:
                return cloud
            time.sleep(0.02)
        return None

    @property
    def is_connected(self) -> bool:
        return self._connected

    # ------------------------------------------------------------------
    # Context manager
    # ------------------------------------------------------------------

    def __enter__(self) -> "Lidar":
        if not self._connected:
            self.connect()
        return self

    def __exit__(self, *_) -> None:
        self.disconnect()

    def __repr__(self) -> str:
        return "Lidar(ip=%s, connected=%s)" % (self._ip, self._connected)

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _build_config(self):
        """Return a RobotConfig, overriding lidar_ip if self._ip is set."""
        from core.config import get_config
        cfg = get_config()
        if self._ip and self._ip != cfg.lidar.lidar_ip:
            cfg = deepcopy(cfg)
            cfg.lidar.lidar_ip = self._ip
        return cfg

    def _start_native_driver(self, cfg) -> None:
        """Launch livox_ros_driver2 as a NativeModule subprocess."""
        try:
            from slam.native_factories import livox_driver
            self._native = livox_driver(cfg)
            self._native.setup()
            self._native.start()
        except Exception as e:
            logger.error("Lidar: failed to start native driver: %s", e)

    def _start_dds_bridge(self) -> None:
        """Subscribe to /lidar/scan via cyclonedds to receive point clouds."""
        if not HAS_LIVOX_IDL:
            logger.warning(
                "Lidar: cyclonedds not available — on_cloud/get_cloud will not work.\n"
                "  Install with: pip install cyclonedds"
            )
            return
        try:
            from core.dds import DDSReader
            self._dds = DDSReader()
            self._dds.subscribe("/lidar/scan", LivoxCustomMsg, self._on_dds_scan)
            self._dds.spin_background()
            logger.info("Lidar: DDS bridge active on /lidar/scan")
        except Exception as e:
            logger.error("Lidar: DDS bridge failed: %s", e)

    def _on_dds_scan(self, msg) -> None:
        """Convert LivoxCustomMsg → numpy (N, 4) and fire callbacks."""
        if not msg or not msg.points:
            return
        try:
            pts = msg.points
            arr = np.empty((len(pts), 4), dtype=np.float32)
            for i, p in enumerate(pts):
                arr[i, 0] = p.x
                arr[i, 1] = p.y
                arr[i, 2] = p.z
                arr[i, 3] = float(p.reflectivity)

            with self._lock:
                self._latest_cloud = arr

            for cb in self._callbacks:
                try:
                    cb(arr)
                except Exception as e:
                    logger.warning("Lidar callback error: %s", e)
        except Exception as e:
            logger.debug("Lidar _on_dds_scan error: %s", e)
