"""SLAM — build maps, localize, get odometry."""

from __future__ import annotations

import logging
import os
import time
from typing import Optional

import numpy as np

logger = logging.getLogger(__name__)


class SLAM:
    """SLAM / Localization — wraps SLAMModule + SlamBridgeModule.

    Usage::

        slam = SLAM(lidar)             # or SLAM() for bridge-only
        slam.start()
        pose = slam.get_pose()         # (x, y, z, yaw)
        cloud = slam.get_map_cloud()   # numpy (N,3)
        slam.save_map("building_a")
        slam.stop()
    """

    def __init__(self, lidar=None, mode: str = "fastlio2"):
        """
        Args:
            lidar: LiDAR instance (optional, SLAM manages its own if None)
            mode: "fastlio2" (mapping), "localizer" (localization), "bridge" (external)
        """
        self._lidar = lidar
        self._mode = mode
        self._slam_module = None
        self._bridge = None
        self._latest_odom = None
        self._latest_cloud = None
        self._started = False

    def start(self) -> SLAM:
        if self._started:
            return self
        try:
            from slam.slam_bridge_module import SlamBridgeModule
            from slam.slam_module import SLAMModule

            if self._mode != "bridge":
                self._slam_module = SLAMModule(backend=self._mode)
                self._slam_module.setup()

            self._bridge = SlamBridgeModule()
            self._bridge.setup()
            self._bridge.odometry._add_callback(self._on_odom)
            self._bridge.map_cloud._add_callback(self._on_cloud)

            if self._slam_module:
                self._slam_module.start()
            self._bridge.start()

            self._started = True
            logger.info("SLAM started (mode=%s)", self._mode)
        except Exception as e:
            logger.error("SLAM start failed: %s", e)
        return self

    def stop(self) -> None:
        if self._bridge:
            self._bridge.stop()
        if self._slam_module:
            self._slam_module.stop()
        self._started = False

    def get_pose(self):
        """Returns (x, y, z, yaw) or None."""
        if self._latest_odom is None:
            return None
        o = self._latest_odom
        return (o.x, o.y, o.z, o.yaw)

    def get_map_cloud(self) -> np.ndarray | None:
        """Latest map point cloud (N,3)."""
        return self._latest_cloud

    def save_map(self, name: str) -> bool:
        """Save current SLAM map to ~/data/nova/maps/<name>/."""
        import subprocess
        map_dir = os.path.join(
            os.environ.get("NAV_MAP_DIR", os.path.expanduser("~/data/nova/maps")),
            name,
        )
        os.makedirs(map_dir, exist_ok=True)
        pcd_path = os.path.join(map_dir, "map.pcd")
        try:
            result = subprocess.run(
                ["ros2", "service", "call", "/pgo/save_maps",
                 "interface/srv/SaveMaps", "{file_path: '%s'}" % pcd_path],
                capture_output=True, text=True, timeout=30,
            )
            if result.returncode == 0:
                logger.info("Map saved: %s", pcd_path)
                self._build_tomogram(name)
                return True
            else:
                logger.error("Map save failed: %s", result.stderr[:200])
                return False
        except Exception as e:
            logger.error("Map save error: %s", e)
            return False

    def _build_tomogram(self, name: str) -> None:
        try:
            import sys
            sys.path.insert(0, os.path.join(
                os.path.dirname(__file__), "..", "global_planning",
                "PCT_planner", "tomography", "scripts"))
            from build_tomogram import build_tomogram_from_pcd
            map_dir = os.path.join(
                os.environ.get("NAV_MAP_DIR", os.path.expanduser("~/data/nova/maps")),
                name,
            )
            build_tomogram_from_pcd(
                os.path.join(map_dir, "map.pcd"),
                os.path.join(map_dir, "tomogram.pickle"),
            )
            logger.info("Tomogram built for %s", name)
        except Exception as e:
            logger.warning("Tomogram build failed: %s", e)

    def _on_odom(self, odom):
        self._latest_odom = odom

    def _on_cloud(self, cloud):
        if cloud.points is not None and len(cloud.points) > 0:
            self._latest_cloud = cloud.points[:, :3]

    @property
    def odom_count(self) -> int:
        return self._bridge.odometry.msg_count if self._bridge else 0

    @property
    def cloud_count(self) -> int:
        return self._bridge.map_cloud.msg_count if self._bridge else 0

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *args):
        self.stop()

    def __repr__(self):
        return "SLAM(mode=%s, running=%s, odom=%d, cloud=%d)" % (
            self._mode, self._started, self.odom_count, self.cloud_count)
