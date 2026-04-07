"""ElevationMapModule — real-time elevation (height) map from LiDAR.

Subscribes to SLAMModule.map_cloud (PointCloud2).
Bins 3-D points by XY cell and computes per-cell height statistics:

  min_z     — lowest point per cell  (ground surface estimate)
  max_z     — highest point per cell (obstacle / ceiling top)
  clearance — max_z - min_z          (free vertical space)
  valid     — bool mask (True = at least one point hit this cell)

Ports:
  In:  map_cloud (PointCloud2), odometry (Odometry)
  Out: elevation_map (dict)
"""

from __future__ import annotations

import logging
import time
from typing import Any, Dict

import numpy as np

from core.module import Module
from core.msgs.nav import Odometry
from core.msgs.sensor import PointCloud2
from core.registry import register
from core.stream import In, Out

logger = logging.getLogger(__name__)


@register("map", "elevation", description="Real-time elevation map from LiDAR point cloud")
class ElevationMapModule(Module, layer=2):
    """Per-cell min/max height from LiDAR — robot-centric, sliding window.

    Uses numpy ufunc scatter (np.minimum.at / np.maximum.at) for O(N)
    per-cell min/max without Python loops over cells.

    Consumers
    ---------
    - Terrain cost: slope = (max_z[i] - max_z[j]) / cell_dist — feeds A*
    - Step detection: clearance < robot_height → impassable
    - Foot placement: min_z surface for quadruped planner
    """

    map_cloud:     In[PointCloud2]
    odometry:      In[Odometry]

    elevation_map: Out[dict]

    def __init__(
        self,
        resolution: float = 0.2,
        map_radius: float = 15.0,   # half-width (m) — finer scope than occupancy
        z_floor: float = -0.50,     # discard below-ground noise
        z_ceil: float = 3.00,       # discard above-ceiling noise
        publish_hz: float = 2.0,
        **kw,
    ):
        super().__init__(**kw)
        self._res = resolution
        self._radius = map_radius
        self._z_floor = z_floor
        self._z_ceil = z_ceil
        self._interval = 1.0 / publish_hz
        self._robot_xy = np.zeros(2, dtype=np.float64)
        self._gs = int(2 * map_radius / resolution)

    def setup(self) -> None:
        self.map_cloud.subscribe(self._on_cloud)
        self.odometry.subscribe(self._on_odom)
        self.map_cloud.set_policy("throttle", interval=self._interval)

    def _on_odom(self, odom: Odometry) -> None:
        self._robot_xy[0] = odom.x
        self._robot_xy[1] = odom.y

    def _on_cloud(self, cloud: PointCloud2) -> None:
        if cloud.is_empty:
            return
        pts = cloud.points[:, :3]

        mask = (pts[:, 2] > self._z_floor) & (pts[:, 2] < self._z_ceil)
        pts = pts[mask]
        if pts.shape[0] == 0:
            return

        origin_xy = self._robot_xy - self._radius
        gs = self._gs

        ix = np.floor((pts[:, 0] - origin_xy[0]) / self._res).astype(np.int32)
        iy = np.floor((pts[:, 1] - origin_xy[1]) / self._res).astype(np.int32)
        valid = (ix >= 0) & (ix < gs) & (iy >= 0) & (iy < gs)
        ix, iy, z = ix[valid], iy[valid], pts[valid, 2].astype(np.float32)

        n_cells = gs * gs
        flat = ix * gs + iy  # linear cell index

        # Scatter min/max with numpy ufunc — O(N), no Python cell loop
        min_flat = np.full(n_cells, np.inf,  dtype=np.float32)
        max_flat = np.full(n_cells, -np.inf, dtype=np.float32)
        np.minimum.at(min_flat, flat, z)
        np.maximum.at(max_flat, flat, z)

        hit = (min_flat != np.inf)
        min_flat = np.where(hit, min_flat, np.nan)
        max_flat = np.where(hit, max_flat, np.nan)

        min_z     = min_flat.reshape(gs, gs)
        max_z     = max_flat.reshape(gs, gs)
        valid_mask = hit.reshape(gs, gs)
        clearance  = np.where(valid_mask, max_z - min_z, np.nan)

        self.elevation_map.publish({
            "min_z":      min_z,
            "max_z":      max_z,
            "clearance":  clearance,
            "valid":      valid_mask,
            "resolution": self._res,
            "origin":     origin_xy.tolist(),
            "ts":         time.time(),
            "frame_id":   "map",
        })

    def health(self) -> dict[str, Any]:
        info = super().port_summary()
        info["elevation_map"] = {
            "resolution": self._res,
            "map_radius": self._radius,
            "grid_size":  self._gs,
            "z_range":    [self._z_floor, self._z_ceil],
        }
        return info
