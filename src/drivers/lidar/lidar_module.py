"""LidarModule — Livox MID-360 driver as a Module-First component.

Manages the livox_ros_driver2 lifecycle and bridges raw scans into
the Module port pipeline for downstream consumers (SLAM, terrain, etc.).

Blueprint usage::

    bp.add(LidarModule)
    bp.wire("LidarModule", "scan", "SlamBridgeModule", "raw_scan")
"""

from __future__ import annotations

import logging

import numpy as np

from core.module import Module
from core.msgs.sensor import PointCloud2
from core.registry import register
from core.stream import Out

from .lidar import Lidar

logger = logging.getLogger(__name__)


@register("driver", "lidar_mid360")
class LidarModule(Module, layer=1):
    """Livox MID-360 driver module.

    Starts livox_ros_driver2 and exposes raw point cloud on the ``scan`` port.
    Downstream modules (SLAM, terrain analysis) subscribe to this port.

    Config (robot_config.yaml)::

        lidar:
          lidar_ip: "192.168.1.115"
          host_ip:  "192.168.1.5"
          publish_freq: 10.0
    """

    scan: Out[PointCloud2]

    def __init__(self, ip: str | None = None, **kw):
        super().__init__(**kw)
        self._lidar = Lidar(ip=ip)

    def setup(self) -> None:
        self._lidar.on_cloud(self._on_cloud)
        self._lidar.connect()

    def teardown(self) -> None:
        self._lidar.disconnect()

    def _on_cloud(self, pts: np.ndarray) -> None:
        """Forward numpy (N, 4) cloud to the scan port as PointCloud2."""
        # Wrap in a minimal PointCloud2 compatible with downstream consumers.
        msg = PointCloud2(
            width=len(pts),
            height=1,
            data=pts.tobytes(),
            point_step=16,           # 4 × float32
            row_step=16 * len(pts),
            fields=["x", "y", "z", "intensity"],
        )
        self.scan.send(msg)
