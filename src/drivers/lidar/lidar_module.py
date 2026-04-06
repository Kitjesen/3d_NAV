"""LidarModule — Livox MID-360 as a first-class Blueprint Module.

Decouples LiDAR from SLAM: LiDAR is an independent hardware resource
that can be started, stopped, and subscribed to without running SLAM.

Blueprint usage::

    from drivers.lidar import LidarModule

    bp.add(LidarModule)                             # default config
    bp.add(LidarModule, ip="192.168.1.120")         # override IP
    bp.wire("LidarModule", "scan", "TerrainModule", "cloud")

Stack factory usage (in blueprints/stacks/)::

    from drivers.lidar import LidarModule
    bp.add(LidarModule, ip=ip)
    # SLAMModule no longer starts the driver — it subscribes via DDS topics
    # that LidarModule's native driver publishes.
"""

from __future__ import annotations

import logging
from typing import Any, Dict, Optional

from core.module import Module
from core.msgs.sensor import Imu, PointCloud2
from core.registry import register
from core.stream import Out

from .lidar import Lidar, LidarState

logger = logging.getLogger(__name__)


@register("driver", "lidar_mid360", description="Livox MID-360 LiDAR driver")
class LidarModule(Module, layer=1):
    """Livox MID-360 driver as a Module in the Blueprint system.

    Owns the Livox driver lifecycle (start/stop/health) and bridges raw
    point cloud + IMU data into Module output ports.

    Ports:
        scan (Out[PointCloud2]): Raw LiDAR point cloud per frame.
        imu  (Out[Imu]):         IMU readings from the LiDAR unit.
        alive (Out[bool]):       Driver health status.

    Config from robot_config.yaml::

        lidar:
          lidar_ip: "192.168.1.115"
          host_ip:  "192.168.1.5"
          publish_freq: 10.0
    """

    scan:  Out[PointCloud2]
    imu:   Out[Imu]
    alive: Out[bool]

    def __init__(
        self,
        ip: Optional[str] = None,
        scan_topic: str = "/lidar/scan",
        imu_topic: str = "/imu/data",
        **kw,
    ):
        super().__init__(**kw)
        self._lidar = Lidar(ip=ip, scan_topic=scan_topic, imu_topic=imu_topic)

    def setup(self) -> None:
        self._lidar.on_cloud(self._on_cloud)
        self._lidar.on_imu(self._on_imu)

    def start(self) -> None:
        super().start()
        try:
            self._lidar.connect()
            self.alive.publish(True)
        except Exception as e:
            self.alive.publish(False)
            logger.error("LidarModule start failed: %s", e)

    def stop(self) -> None:
        self._lidar.disconnect()
        self.alive.publish(False)
        super().stop()

    def _on_cloud(self, pts) -> None:
        """Forward numpy (N,4) → PointCloud2 on the scan port."""
        cloud = PointCloud2.from_numpy(pts, frame_id="livox_frame")
        self.scan.publish(cloud)

    def _on_imu(self, imu_msg: Imu) -> None:
        """Forward Imu to the imu port."""
        self.imu.publish(imu_msg)

    def health(self) -> Dict[str, Any]:
        """Report driver health for monitoring."""
        base = super().port_summary()
        h = self._lidar.health
        base["lidar"] = h.to_dict()
        return base

    def __repr__(self) -> str:
        return f"LidarModule(ip={self._lidar.ip!r}, state={self._lidar.state.value})"
