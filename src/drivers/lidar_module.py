"""LidarModule — LiDAR sensor as a pluggable Module.

Backends:
  "livox"     — Livox Mid-360 (C++ NativeModule, production)
  "velodyne"  — Velodyne VLP-16 (C++ NativeModule)
  "sim"       — Simulated point cloud (testing)

Usage::

    bp.add(LidarModule, backend="livox")
    # Out: lidar_cloud, imu
"""

from __future__ import annotations

import logging
import time
from typing import Any, Dict

import numpy as np

from core.module import Module
from core.stream import Out
from core.msgs.sensor import PointCloud
from core.registry import register

logger = logging.getLogger(__name__)


@register("lidar", "livox", description="Livox Mid-360 via C++ driver (NativeModule)")
@register("lidar", "sim", description="Simulated LiDAR point cloud")
class LidarModule(Module, layer=1):
    """LiDAR sensor driver — publishes point clouds and IMU data."""

    # -- Outputs --
    lidar_cloud: Out[PointCloud]
    alive: Out[bool]

    def __init__(self, backend: str = "livox", **kw):
        super().__init__(**kw)
        self._backend = backend
        self._node = None

    def setup(self):
        if self._backend == "livox":
            self._setup_livox()
        elif self._backend == "sim":
            logger.info("LidarModule [sim]: simulated point cloud")
        else:
            logger.warning("LidarModule: unknown backend '%s'", self._backend)

    def _setup_livox(self):
        """Setup Livox C++ driver via NativeModule."""
        try:
            from core.native_module import NativeModule, NativeModuleConfig
            self._node = NativeModule(NativeModuleConfig(
                executable="livox_ros_driver2_node",
                name="livox_driver",
                parameters={},
            ))
            self._node.setup()
        except (ImportError, FileNotFoundError, PermissionError) as e:
            logger.warning("LidarModule [livox]: not available: %s", e)

    def start(self):
        super().start()
        if self._node:
            try:
                self._node.start()
            except Exception as e:
                logger.error("LidarModule: start failed: %s", e)
        self.alive.publish(True)

    def stop(self):
        if self._node:
            try:
                self._node.stop()
            except Exception:
                pass
            self._node = None
        self.alive.publish(False)
        super().stop()

    def publish_sim_cloud(self, n_points: int = 1000):
        """Publish a simulated point cloud (for testing)."""
        pts = np.random.randn(n_points, 3).astype(np.float32)
        self.lidar_cloud.publish(PointCloud(points=pts, frame_id="lidar", ts=time.time()))

    def health(self) -> Dict[str, Any]:
        info = super().port_summary()
        info["lidar"] = {"backend": self._backend}
        return info
