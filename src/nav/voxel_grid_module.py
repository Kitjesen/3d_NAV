"""VoxelGridModule — 3D voxel grid map from LiDAR point cloud.

Maintains a numpy hash-map of voxel occupancy counts, with exponential
decay to forget stale space.  Publishes occupied voxel centres as a
PointCloud2 and a stats dict.

Ports:
  In:  map_cloud (PointCloud2), odometry (Odometry)
  Out: voxel_map (dict), voxel_cloud (PointCloud2)
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Any, Dict, Tuple

import numpy as np

from core.module import Module, skill
from core.msgs.nav import Odometry
from core.msgs.sensor import PointCloud2
from core.registry import register
from core.stream import In, Out

logger = logging.getLogger(__name__)

# Voxel key type: (ix, iy, iz) integer tuple
_VoxelKey = tuple[int, int, int]


@register("map", "voxel", description="3D voxel grid from LiDAR point cloud")
class VoxelGridModule(Module, layer=2):
    """Accumulate LiDAR returns into a 3-D voxel hash map.

    Each voxel stores an integer hit count.  On every publish cycle the
    counts are decayed by ``(1 - decay_rate)`` so old returns fade out
    and voxels with count < 1 are pruned.  The CPU-only path is safe for
    S100P (no CUDA, no Open3D).

    Column carving: for every occupied (ix, iy) column the module tracks
    the min/max iz so callers can distinguish ground from obstacles.
    """

    map_cloud:   In[PointCloud2]
    odometry:    In[Odometry]

    voxel_map:   Out[dict]
    voxel_cloud: Out[PointCloud2]

    def __init__(
        self,
        voxel_size: float = 0.05,       # metres per voxel edge
        max_range: float = 20.0,         # discard points beyond this range (m)
        min_z: float = -0.5,             # discard points below this Z (m)
        max_z: float = 3.0,              # discard points above this Z (m)
        decay_rate: float = 0.01,        # fraction decayed per publish cycle
        publish_interval: float = 2.0,   # seconds between publishes
        **kw,
    ):
        super().__init__(**kw)
        self._res = float(voxel_size)
        self._max_range = float(max_range)
        self._min_z = float(min_z)
        self._max_z = float(max_z)
        self._decay = float(decay_rate)
        self._interval = float(publish_interval)

        # voxel key → float count (float for smooth decay)
        self._voxels: dict[_VoxelKey, float] = {}
        self._lock = threading.Lock()

        self._robot_xyz = np.zeros(3, dtype=np.float64)
        self._last_publish: float = 0.0

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def setup(self) -> None:
        self.map_cloud.subscribe(self._on_cloud)
        self.odometry.subscribe(self._on_odom)
        # Accept every point cloud but throttle the expensive publish step
        self.map_cloud.set_policy("latest")

    def start(self) -> None:
        super().start()
        self._last_publish = time.time()

    def stop(self) -> None:
        super().stop()

    # ------------------------------------------------------------------
    # Subscribers
    # ------------------------------------------------------------------

    def _on_odom(self, odom: Odometry) -> None:
        self._robot_xyz[0] = odom.x
        self._robot_xyz[1] = odom.y
        self._robot_xyz[2] = odom.z

    def _on_cloud(self, cloud: PointCloud2) -> None:
        if cloud.is_empty:
            return

        pts = cloud.points[:, :3].astype(np.float64)

        # Range filter relative to robot position
        diff = pts - self._robot_xyz
        dist = np.linalg.norm(diff, axis=1)
        mask = (
            (dist <= self._max_range)
            & (pts[:, 2] >= self._min_z)
            & (pts[:, 2] <= self._max_z)
        )
        pts = pts[mask]
        if pts.shape[0] == 0:
            return

        # Convert to integer voxel indices
        ix = np.floor(pts[:, 0] / self._res).astype(np.int64)
        iy = np.floor(pts[:, 1] / self._res).astype(np.int64)
        iz = np.floor(pts[:, 2] / self._res).astype(np.int64)
        keys = zip(ix.tolist(), iy.tolist(), iz.tolist())

        with self._lock:
            for key in keys:
                self._voxels[key] = self._voxels.get(key, 0.0) + 1.0

        now = time.time()
        if now - self._last_publish >= self._interval:
            self._last_publish = now
            self._decay_and_publish()

    # ------------------------------------------------------------------
    # Decay + publish
    # ------------------------------------------------------------------

    def _decay_and_publish(self) -> None:
        """Apply exponential decay, prune dead voxels, then publish."""
        with self._lock:
            factor = 1.0 - self._decay
            to_delete = []
            for key, count in self._voxels.items():
                new_count = count * factor
                if new_count < 1.0:
                    to_delete.append(key)
                else:
                    self._voxels[key] = new_count
            for key in to_delete:
                del self._voxels[key]

            if not self._voxels:
                return

            keys_arr = np.array(list(self._voxels.keys()), dtype=np.float32)

        # Voxel centres = (index + 0.5) * resolution
        centres = (keys_arr + 0.5) * self._res  # (N, 3) float32

        # Column carving: per (ix, iy) find min/max iz
        col_map: dict[tuple, list] = {}
        for _i, (ix, iy, iz) in enumerate(keys_arr.astype(np.int64).tolist()):
            col = (int(ix), int(iy))
            if col not in col_map:
                col_map[col] = [iz, iz]
            else:
                if iz < col_map[col][0]:
                    col_map[col][0] = iz
                if iz > col_map[col][1]:
                    col_map[col][1] = iz

        total = len(self._voxels)
        mem_kb = (total * 64) / 1024  # rough: 3×int64 key + float value

        stats: dict[str, Any] = {
            "total_voxels": total,
            "occupied": total,
            "memory_kb": round(mem_kb, 1),
            "voxel_size": self._res,
            "ts": time.time(),
            "column_count": len(col_map),
        }

        cloud = PointCloud2(points=centres, frame_id="map")
        self.voxel_map.publish(stats)
        self.voxel_cloud.publish(cloud)

    # ------------------------------------------------------------------
    # Skills
    # ------------------------------------------------------------------

    @skill
    def get_voxel_stats(self) -> dict:
        """Return stats about the current voxel map.

        Returns:
            dict with total_voxels, occupied, memory_kb, voxel_size.
        """
        with self._lock:
            total = len(self._voxels)
        mem_kb = (total * 64) / 1024
        return {
            "total_voxels": total,
            "occupied": total,
            "memory_kb": round(mem_kb, 1),
            "voxel_size": self._res,
        }

    @skill
    def clear_voxels(self) -> dict:
        """Reset the entire voxel map.

        Returns:
            dict with cleared count.
        """
        with self._lock:
            count = len(self._voxels)
            self._voxels.clear()
        logger.info("VoxelGrid cleared %d voxels", count)
        return {"cleared": count}

    @skill
    def query_voxel(self, x: float, y: float, z: float) -> dict:
        """Check whether the voxel containing (x, y, z) is occupied.

        Args:
            x: World X coordinate (metres).
            y: World Y coordinate (metres).
            z: World Z coordinate (metres).

        Returns:
            dict with occupied (bool), count (float), voxel_key (list).
        """
        key: _VoxelKey = (
            int(np.floor(x / self._res)),
            int(np.floor(y / self._res)),
            int(np.floor(z / self._res)),
        )
        with self._lock:
            count = self._voxels.get(key, 0.0)
        return {
            "occupied": count >= 1.0,
            "count": count,
            "voxel_key": list(key),
        }

    # ------------------------------------------------------------------
    # Health
    # ------------------------------------------------------------------

    def health(self) -> dict[str, Any]:
        info = super().port_summary()
        with self._lock:
            n = len(self._voxels)
        info["voxel_grid"] = {
            "voxel_size": self._res,
            "max_range": self._max_range,
            "z_range": [self._min_z, self._max_z],
            "decay_rate": self._decay,
            "publish_interval": self._interval,
            "voxel_count": n,
        }
        return info
