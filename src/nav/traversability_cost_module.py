"""TraversabilityCostModule — fuse obstacle + slope + ESDF into unified cost.

Subscribes to four map modules, produces a single fused traversability grid
that both GlobalPlannerService (via NavigationModule) and LocalPlannerModule
consume.

Fusion formula per cell:
  fused = clamp(
      w_obs   * obstacle_cost
    + w_slope * slope_cost(elevation)
    + w_prox  * proximity_cost(esdf)
  , 0, 100)

Ports:
  In:  costmap (dict), elevation_map (dict), esdf (dict), traversability (dict)
  Out: fused_cost (dict), esdf_field (dict), slope_grid (dict)
"""

from __future__ import annotations

import logging
import time
from typing import Any

import numpy as np

from core.module import Module
from core.registry import register
from core.stream import In, Out

logger = logging.getLogger(__name__)


def _resample_to_grid(
    src: np.ndarray,
    src_origin: np.ndarray,
    src_res: float,
    dst_shape: tuple[int, int],
    dst_origin: np.ndarray,
    dst_res: float,
    fill: float = 0.0,
) -> np.ndarray:
    """Nearest-neighbour resample src grid onto dst grid coordinates."""
    dst_h, dst_w = dst_shape
    # Build dst cell centres in world coords
    ys = dst_origin[1] + (np.arange(dst_h) + 0.5) * dst_res
    xs = dst_origin[0] + (np.arange(dst_w) + 0.5) * dst_res

    # Map to src cell indices
    src_ix = np.floor((xs - src_origin[0]) / src_res).astype(np.int32)
    src_iy = np.floor((ys - src_origin[1]) / src_res).astype(np.int32)

    # 2D index grids
    ix_grid = np.broadcast_to(src_ix[None, :], (dst_h, dst_w))
    iy_grid = np.broadcast_to(src_iy[:, None], (dst_h, dst_w))

    valid = (
        (ix_grid >= 0) & (ix_grid < src.shape[1])
        & (iy_grid >= 0) & (iy_grid < src.shape[0])
    )
    out = np.full(dst_shape, fill, dtype=np.float32)
    out[valid] = src[iy_grid[valid], ix_grid[valid]]
    return out


@register("map", "traversability_cost",
          description="Fuse obstacle + slope + ESDF into unified traversability cost")
class TraversabilityCostModule(Module, layer=2):
    """Unified traversability cost provider.

    Fuses four map layers into a single cost grid consumed by both
    global planner (via NavigationModule) and local planner (via ESDF field).
    Gracefully degrades: if any input is missing, that layer contributes zero cost.
    """

    # Inputs — from four map sources
    costmap:        In[dict]   # OccupancyGridModule: {grid, resolution, origin}
    elevation_map:  In[dict]   # ElevationMapModule: {min_z, max_z, clearance, valid}
    esdf:           In[dict]   # ESDFModule: {distance_field, grad_x, grad_y}
    traversability: In[dict]   # TerrainModule: traversability stats

    # Outputs — unified interface
    fused_cost: Out[dict]   # {grid(float32 0-100), resolution, origin, ts}
    esdf_field: Out[dict]   # {distance_field, grad_x, grad_y, resolution, origin}
    slope_grid: Out[dict]   # {grid(float32 degrees), resolution, origin}

    def __init__(
        self,
        obstacle_weight: float = 1.0,
        slope_weight: float = 0.3,
        proximity_weight: float = 0.2,
        safe_distance: float = 1.5,
        max_slope_deg: float = 30.0,
        publish_hz: float = 2.0,
        **kw,
    ):
        super().__init__(**kw)
        self._w_obs = obstacle_weight
        self._w_slope = slope_weight
        self._w_prox = proximity_weight
        self._safe_dist = safe_distance
        self._max_slope = max_slope_deg
        self._interval = 1.0 / publish_hz

        # Latest data from each source
        self._costmap_data: dict | None = None
        self._elev_data: dict | None = None
        self._esdf_data: dict | None = None
        self._trav_data: dict | None = None
        self._last_publish: float = 0.0

    def setup(self) -> None:
        self.costmap.subscribe(self._on_costmap)
        self.elevation_map.subscribe(self._on_elevation)
        self.esdf.subscribe(self._on_esdf)
        self.traversability.subscribe(self._on_traversability)

    def _on_costmap(self, data: dict) -> None:
        self._costmap_data = data
        self._try_fuse()

    def _on_elevation(self, data: dict) -> None:
        self._elev_data = data

    def _on_esdf(self, data: dict) -> None:
        self._esdf_data = data
        # Forward ESDF immediately for LocalPlanner consumption
        self.esdf_field.publish(data)

    def _on_traversability(self, data: dict) -> None:
        self._trav_data = data

    def _try_fuse(self) -> None:
        """Fuse available layers and publish. Triggered by costmap (primary clock)."""
        now = time.time()
        if now - self._last_publish < self._interval:
            return
        self._last_publish = now

        cm = self._costmap_data
        if cm is None:
            return  # costmap is required as the base grid

        grid = np.asarray(cm["grid"], dtype=np.float32)
        res = cm["resolution"]
        origin = np.array(cm["origin"][:2], dtype=np.float64)
        shape = grid.shape

        # Start with obstacle cost (always available)
        fused = self._w_obs * grid

        # Slope cost from elevation map
        slope_deg = self._compute_slope(shape, origin, res)
        if slope_deg is not None:
            slope_cost = np.clip(slope_deg / self._max_slope, 0, 1) * 100.0
            # Cells above max_slope get hard-blocked
            slope_cost[slope_deg >= self._max_slope] = 100.0
            fused += self._w_slope * slope_cost

            self.slope_grid.publish({
                "grid": slope_deg,
                "resolution": res,
                "origin": origin.tolist(),
                "ts": now,
            })

        # Proximity cost from ESDF
        prox_cost = self._compute_proximity(shape, origin, res)
        if prox_cost is not None:
            fused += self._w_prox * prox_cost

        fused = np.clip(fused, 0, 100).astype(np.float32)

        self.fused_cost.publish({
            "grid": fused,
            "resolution": res,
            "origin": origin.tolist(),
            "ts": now,
        })

    def _compute_slope(
        self,
        dst_shape: tuple[int, int],
        dst_origin: np.ndarray,
        dst_res: float,
    ) -> np.ndarray | None:
        """Compute slope in degrees from elevation grid, resampled to dst grid."""
        elev = self._elev_data
        if elev is None:
            return None

        max_z = elev.get("max_z")
        valid = elev.get("valid")
        if max_z is None or valid is None:
            return None

        max_z = np.asarray(max_z, dtype=np.float32)
        valid = np.asarray(valid, dtype=bool)
        elev_res = elev.get("resolution", 0.2)
        elev_origin = np.array(elev["origin"][:2], dtype=np.float64)

        # Compute slope on elevation grid's own resolution
        # Replace invalid cells with neighbour average to avoid edge artifacts
        z = np.where(valid, max_z, 0.0)
        dzdx = np.gradient(z, elev_res, axis=1)
        dzdy = np.gradient(z, elev_res, axis=0)
        slope_rad = np.arctan(np.sqrt(dzdx**2 + dzdy**2))
        slope_deg = np.degrees(slope_rad).astype(np.float32)
        slope_deg[~valid] = 0.0

        # Resample to costmap grid if dimensions differ
        if slope_deg.shape != dst_shape or not np.allclose(elev_origin, dst_origin):
            slope_deg = _resample_to_grid(
                slope_deg, elev_origin, elev_res,
                dst_shape, dst_origin, dst_res,
                fill=0.0,
            )
        return slope_deg

    def _compute_proximity(
        self,
        dst_shape: tuple[int, int],
        dst_origin: np.ndarray,
        dst_res: float,
    ) -> np.ndarray | None:
        """Repulsive cost from ESDF — decays with distance from obstacles."""
        esdf = self._esdf_data
        if esdf is None:
            return None

        dist = esdf.get("distance_field")
        if dist is None:
            return None

        dist = np.asarray(dist, dtype=np.float32)
        esdf_res = esdf.get("resolution", 0.2)
        esdf_origin = np.array(esdf["origin"][:2], dtype=np.float64)

        cost = np.clip(1.0 - dist / self._safe_dist, 0, 1) * 100.0

        # Resample to costmap grid if dimensions differ
        if cost.shape != dst_shape or not np.allclose(esdf_origin, dst_origin):
            cost = _resample_to_grid(
                cost, esdf_origin, esdf_res,
                dst_shape, dst_origin, dst_res,
                fill=0.0,
            )
        return cost

    def health(self) -> dict[str, Any]:
        info = super().port_summary()
        info["traversability_cost"] = {
            "weights": {
                "obstacle": self._w_obs,
                "slope": self._w_slope,
                "proximity": self._w_prox,
            },
            "safe_distance": self._safe_dist,
            "max_slope_deg": self._max_slope,
            "has_costmap": self._costmap_data is not None,
            "has_elevation": self._elev_data is not None,
            "has_esdf": self._esdf_data is not None,
            "has_traversability": self._trav_data is not None,
        }
        return info
