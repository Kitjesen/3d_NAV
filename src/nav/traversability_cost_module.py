"""TraversabilityCostModule — map layer hub: ESDF relay + slope visualization.

Two core functions:
  1. ESDF → LocalPlanner  — relay ESDF distance field + gradients for smooth
     obstacle avoidance scoring in the CMU local planner. This is the primary
     value on S100P: the local planner gets distance-to-obstacle info it never
     had before, enabling path-score bias toward corridor centers.
  2. Slope → Web          — compute slope grid from ElevationMap and push via
     Gateway SSE for operator situational awareness (green/yellow/red overlay).

Secondary function (dev/sim only):
  3. fused_cost → NavigationModule → A* backend — on dev machines where
     ele_planner.so (PCT) is unavailable, the fused grid gives the Python A*
     terrain awareness (slope hard-block + proximity preference). On S100P
     with PCT active, this fused grid only feeds _find_safe_goal BFS — PCT
     plans on its own pre-built 3D tomogram, not on this grid.

Merge rule: layered max() with hard constraint protection.
  L0  LETHAL(100) / INSCRIBED(99) from OccupancyGrid — never overridden
  L1  slope ≥ max_slope → LETHAL
  L2  slope soft cost 1..97 — only on free cells
  L3  ESDF proximity 0..proximity_cap — only on free cells

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


# W2-7: scipy is required for bilinear resampling. Verified once at module
# import; setup() raises loudly if it's missing (Wave 1 no-silent-fallback
# discipline). Tests patch this flag to simulate the absence.
try:
    import scipy.ndimage
    _SCIPY_AVAILABLE = True
except ImportError:
    _SCIPY_AVAILABLE = False


def _resample_to_grid(
    src: np.ndarray,
    src_origin: np.ndarray,
    src_res: float,
    dst_shape: tuple[int, int],
    dst_origin: np.ndarray,
    dst_res: float,
    fill: float = 0.0,
) -> np.ndarray:
    """Bilinear resample src grid onto dst grid coordinates via scipy.

    W2-7: nearest-neighbour was previously used and produced aliasing at grid
    edges where slope/cost discontinuities confused the planner. Bilinear
    gives continuous interpolated values between source cells.
    """
    if not _SCIPY_AVAILABLE:
        raise RuntimeError(
            "_resample_to_grid requires scipy — call TraversabilityCostModule.setup() "
            "first to see the pre-flight check error."
        )
    from scipy.ndimage import map_coordinates

    dst_h, dst_w = dst_shape
    # Build dst cell centres in world coords
    ys = dst_origin[1] + (np.arange(dst_h) + 0.5) * dst_res
    xs = dst_origin[0] + (np.arange(dst_w) + 0.5) * dst_res

    # Map world coords to fractional src cell indices (row = y-axis, col = x-axis)
    src_row = (ys - src_origin[1]) / src_res - 0.5   # shape (dst_h,)
    src_col = (xs - src_origin[0]) / src_res - 0.5   # shape (dst_w,)

    # 2D coordinate arrays for map_coordinates (row, col order)
    row_grid = np.broadcast_to(src_row[:, None], dst_shape)
    col_grid = np.broadcast_to(src_col[None, :], dst_shape)
    coords = np.array([row_grid.ravel(), col_grid.ravel()])

    out = map_coordinates(
        src.astype(np.float64),
        coords,
        order=1,              # bilinear
        mode="constant",
        cval=float(fill),
    ).reshape(dst_shape).astype(np.float32)
    return out


@register("map", "traversability_cost",
          description="Fuse obstacle + slope + ESDF into unified traversability cost")
class TraversabilityCostModule(Module, layer=2):
    """Map layer hub: ESDF relay + slope visualization + A* fallback fusion.

    Primary (S100P production):
      - Relays ESDF field to LocalPlannerModule for smooth obstacle avoidance
      - Publishes slope grid for Web operator situational awareness
      - fused_cost feeds NavigationModule but PCT ignores it for path planning

    Secondary (dev/sim):
      - fused_cost is the sole cost source for the Python A* planner
      - Gives A* terrain awareness that raw OccupancyGrid alone doesn't have

    Gracefully degrades: if any input is missing, that layer contributes zero.
    max_slope_deg defaults from robot_config.yaml terrain_analysis.slope_max.
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
        safe_distance: float = 1.5,
        max_slope_deg: float | None = None,
        proximity_cap: float = 50.0,
        publish_hz: float = 2.0,
        **kw,
    ):
        super().__init__(**kw)
        self._safe_dist = safe_distance
        self._prox_cap = proximity_cap
        self._interval = 1.0 / publish_hz

        # Read max_slope from robot_config (single source of truth)
        if max_slope_deg is not None:
            self._max_slope = max_slope_deg
        else:
            try:
                from core.config import get_config
                cfg = get_config()
                # terrain_analysis.slope_max is tan(angle); convert to degrees
                import math
                slope_tan = cfg.raw.get("terrain_analysis", {}).get("slope_max", 1.0)
                self._max_slope = math.degrees(math.atan(slope_tan))
            except (ImportError, AttributeError):
                self._max_slope = 45.0  # tan(1.0) = 45°, quadruped limit

        # Latest data from each source
        self._costmap_data: dict | None = None
        self._elev_data: dict | None = None
        self._esdf_data: dict | None = None
        self._trav_data: dict | None = None
        self._last_publish: float = 0.0

    def setup(self) -> None:
        if not _SCIPY_AVAILABLE:
            raise RuntimeError(
                "TraversabilityCostModule requires scipy for bilinear "
                "resampling between elevation/ESDF and costmap grids. "
                "Install with: pip install scipy"
            )
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

    # Cost semantics (match ROS2 nav2 costmap_2d convention):
    #   LETHAL     = 100  — occupied wall, hard constraint, cannot be overridden
    #   INSCRIBED  = 99   — inside inflation radius, treat as impassable
    #   SLOPE_HARD = 98   — slope ≥ max_slope_deg, impassable terrain
    #   1..97             — soft cost (slope preference + ESDF proximity preference)
    #   0                 — free space
    LETHAL    = 100
    INSCRIBED = 99

    def _try_fuse(self) -> None:
        """Layered priority merge. Triggered by costmap (primary clock).

        Layer precedence (high → low):
          L0  Lethal/Inscribed — from OccupancyGrid (obstacle + inflation)
          L1  Slope hard-block — slope ≥ max_slope_deg → LETHAL
          L2  Slope soft cost  — slope mapped to 1..97 (only on free cells)
          L3  Proximity prefer — ESDF distance preference (only on free cells)

        Merge rule: max() per cell with hard constraint protection.
        """
        now = time.time()
        if now - self._last_publish < self._interval:
            return
        self._last_publish = now

        cm = self._costmap_data
        if cm is None:
            return

        obs_grid = np.asarray(cm["grid"], dtype=np.float32)
        res = cm["resolution"]
        origin = np.array(cm["origin"][:2], dtype=np.float64)
        shape = obs_grid.shape

        # L0: Obstacle layer is the base — lethal and inscribed cells are protected
        fused = obs_grid.copy()
        is_hard = obs_grid >= self.INSCRIBED  # lethal + inscribed cells

        # L1+L2: Slope layer
        slope_deg = self._compute_slope(shape, origin, res)
        if slope_deg is not None:
            # Hard-block: slope ≥ max → LETHAL (even if OccupancyGrid says free)
            slope_lethal = slope_deg >= self._max_slope
            fused[slope_lethal & ~is_hard] = self.LETHAL

            # Soft cost: slope < max → map to 1..97, only on non-hard cells
            soft_mask = ~is_hard & ~slope_lethal & (slope_deg > 3.0)
            slope_cost = np.clip(slope_deg / self._max_slope, 0, 0.97) * 100.0
            fused[soft_mask] = np.maximum(fused[soft_mask], slope_cost[soft_mask])

            self.slope_grid.publish({
                "grid": slope_deg,
                "resolution": res,
                "origin": origin.tolist(),
                "ts": now,
            })

        # L3: Proximity preference — only on free-ish cells (cost < INSCRIBED)
        #     This does NOT duplicate inflation (inflation is binary dilation at 0.5m,
        #     proximity is a smooth gradient out to safe_distance=1.5m).
        prox_cost = self._compute_proximity(shape, origin, res)
        if prox_cost is not None:
            free_mask = fused < self.INSCRIBED
            # Cap proximity contribution so it never promotes free→inscribed
            soft_prox = np.clip(prox_cost * (self._prox_cap / 100.0), 0, self._prox_cap)
            fused[free_mask] = np.maximum(fused[free_mask], soft_prox[free_mask])

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
            "merge": "layered_max (LETHAL > INSCRIBED > slope > proximity)",
            "safe_distance": self._safe_dist,
            "max_slope_deg": self._max_slope,
            "proximity_cap": self._prox_cap,
            "layers": {
                "costmap":        self._costmap_data is not None,
                "elevation":      self._elev_data is not None,
                "esdf":           self._esdf_data is not None,
                "traversability": self._trav_data is not None,
            },
        }
        return info
