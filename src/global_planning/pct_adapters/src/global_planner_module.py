"""Planner backends: A* (dev/sim fallback) and native PCT.

FROZEN: this module is stable and should not need changes.
New planner backends (e.g. RRT*) should be registered in separate files
via @register("planner_backend", "name"), not added here.

Used by GlobalPlannerService (nav/global_planner_service.py) via Registry:
    backend = get("planner_backend", "pct")    # native ele_planner.so
    backend = get("planner_backend", "astar")  # dev/sim: pure Python fallback

See docs/02-architecture/PLANNER_SELECTION.md for the selection rationale.

TODOs (low priority, only if needed):
  - A* has no path smoothing (jagged 8-connected grid paths). If visual
    servo or demo needs smoother trajectories, add post-plan Ramer-Douglas-Peucker
    or cubic spline in GlobalPlannerService, not here.
  - A* obstacle_thr is hardcoded 49.9. If OccupancyGridModule changes its
    cost scale, this threshold needs to match.
"""

from __future__ import annotations

import heapq
import logging
import os
from typing import List, Optional, Tuple

import numpy as np

from core.registry import register
from global_planning.PCT_planner_runnable.runtime import load_tomogram_planner

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# PCT backend: native terrain-aware planner
# ---------------------------------------------------------------------------

@register("planner_backend", "pct",
          description="C++ ele_planner via .so, 3D terrain-aware native PCT")
class _PCTBackend:
    """Production planner: TomogramPlanner wrapping ele_planner.so + traj_opt.so.

    Capabilities:
      - 3D A* on multi-slice traversability grid (stairs, ramps)
      - GPMP trajectory optimisation (smooth, kinematically feasible)
      - Gradient-aware elevation planning

    Requires compiled native .so files matching the current Linux arch/Python ABI.
    On import failure logs a clear error and marks itself unavailable; all
    plan() calls then return [] so GlobalPlannerService raises RuntimeError
    and NavigationModule falls back to direct-goal mode.
    """

    def __init__(self, tomogram_path: str = "", obstacle_thr: float = 49.9):
        self._planner = None
        self._tomogram_path = tomogram_path
        self._obstacle_thr = obstacle_thr
        self._available = False
        self._load_error: str = ""

        # 2D ground-floor grid for _find_safe_goal BFS (extracted from tomogram)
        self._grid: np.ndarray | None = None
        self._trav_3d: np.ndarray | None = None
        self._resolution: float = 0.2
        self._origin: np.ndarray = np.zeros(2)
        self._slice_h0: float = 0.0
        self._slice_dh: float = 0.5

        # Live costmap overlay for dynamic obstacle avoidance
        self._costmap: np.ndarray | None = None
        self._costmap_resolution: float = 0.2
        self._costmap_origin: np.ndarray = np.zeros(2)

        self._try_load(tomogram_path)

    def _try_load(self, tomogram_path: str) -> None:
        """Try to import TomogramPlanner and load the tomogram.

        Sets self._available = True only when both the .so and the tomogram
        are successfully loaded.
        """
        tomogram_path = os.path.abspath(tomogram_path)

        if not os.path.exists(tomogram_path):
            self._load_error = (
                f"Tomogram file not found: '{tomogram_path}'. "
                f"Set tomogram= in your profile or copy the map to "
                f"~/data/inovxio/data/maps/active/tomogram.pickle"
            )
            logger.error("PCT backend: %s", self._load_error)
            return

        try:
            planner, runtime_paths = load_tomogram_planner(
                tomogram_path,
                obstacle_thr=self._obstacle_thr,
            )
            self._planner = planner
            self._available = True
            logger.info(
                "PCT ele_planner loaded: %s  lib_dir=%s map_dim=%s slices=%s",
                tomogram_path,
                runtime_paths.lib_dir,
                getattr(planner, "map_dim", "?"),
                getattr(planner, "n_slice", "?"),
            )
            self._extract_grid(tomogram_path)
        except Exception as e:
            self._load_error = f"PCT runtime/loadTomogram failed: {e}"
            logger.exception("PCT backend failed to load tomogram: %s", tomogram_path)

    @property
    def available(self) -> bool:
        return self._available

    def _extract_grid(self, tomogram_path: str) -> None:
        """Extract 2D ground-floor traversability grid from the tomogram pickle.

        This grid is used by GlobalPlannerService._find_safe_goal() BFS to
        verify goals are in free space before sending them to the 3D planner.
        """
        import pickle
        try:
            with open(tomogram_path, "rb") as f:
                raw = pickle.load(f)
        except Exception:
            logger.warning("PCT: could not load tomogram pickle for grid extraction")
            return

        if not isinstance(raw, dict):
            return

        tomo_data = raw.get("data")
        res = float(raw.get("resolution", 0.2))
        self._slice_h0 = float(raw.get("slice_h0", 0.0))
        self._slice_dh = float(raw.get("slice_dh", 0.5))

        if tomo_data is not None and hasattr(tomo_data, "ndim") and tomo_data.ndim == 4:
            # data shape: (5, n_slices, H, W), channel 0 = traversability
            trav_3d = np.asarray(tomo_data[0], dtype=np.float32)
            if "slice_h0" in raw and "slice_dh" in raw:
                trav_3d = np.transpose(trav_3d, (0, 2, 1))
            self._trav_3d = trav_3d
            self._grid = np.asarray(self._trav_3d[0], dtype=np.float32)
            center = np.array(raw.get("center", [0, 0])[:2], dtype=np.float64)
            h, w = self._grid.shape
            self._origin = center - np.array([w * res / 2, h * res / 2])
        else:
            grid = raw.get("grid", raw.get("traversability"))
            if grid is not None:
                self._grid = np.asarray(grid, dtype=np.float32)
            self._trav_3d = None
            self._origin = np.array(raw.get("origin", [0, 0])[:2], dtype=np.float64)

        self._resolution = res
        self._static_grid: np.ndarray | None = None
        if self._grid is not None:
            self._static_grid = self._grid.copy()
            logger.info(
                "PCT: extracted 2D grid for goal safety BFS: shape=%s res=%.3f",
                self._grid.shape, res,
            )

    def update_map(self, grid: np.ndarray, resolution: float = 0.2,
                   origin: np.ndarray | None = None) -> None:
        """Accept live costmap for dynamic obstacle checking in _find_safe_goal.

        The costmap is stored separately and merged with the static tomogram
        grid when GlobalPlannerService calls _find_safe_goal(). The 3D PCT
        planner itself still uses the pre-built tomogram for path planning.
        """
        self._costmap = np.asarray(grid, dtype=np.float32)
        self._costmap_resolution = resolution
        if origin is not None:
            self._costmap_origin = np.array(origin[:2], dtype=np.float64)

        # Merge costmap obstacles into _grid so _find_safe_goal sees them
        if self._grid is not None:
            self._merge_costmap()

    def _merge_costmap(self) -> None:
        """Overlay dynamic costmap obstacles onto the static tomogram grid.

        Resets _grid from _static_grid first, then overlays costmap obstacles.
        Works even when costmap and tomogram have different resolutions/origins.
        """
        if self._costmap is None or self._static_grid is None:
            return

        # Reset to clean static grid before merging
        self._grid = self._static_grid.copy()

        cm = self._costmap
        cm_res = self._costmap_resolution
        cm_ox, cm_oy = self._costmap_origin[0], self._costmap_origin[1]
        g_res = self._resolution
        g_ox, g_oy = self._origin[0], self._origin[1]
        gh, gw = self._grid.shape
        _ch, _cw = cm.shape

        # Find costmap cells that are obstacles
        obs_rows, obs_cols = np.where(cm >= self._obstacle_thr)
        if len(obs_rows) == 0:
            return

        # Convert costmap obstacle cells to world coords, then to grid coords
        world_x = cm_ox + obs_cols * cm_res
        world_y = cm_oy + obs_rows * cm_res
        grid_cols = np.round((world_x - g_ox) / g_res).astype(int)
        grid_rows = np.round((world_y - g_oy) / g_res).astype(int)

        # Filter in-bounds and mark obstacles
        mask = (grid_cols >= 0) & (grid_cols < gw) & (grid_rows >= 0) & (grid_rows < gh)
        valid_rows = grid_rows[mask]
        valid_cols = grid_cols[mask]
        if len(valid_rows) > 0:
            self._grid[valid_rows, valid_cols] = np.maximum(
                self._grid[valid_rows, valid_cols],
                cm[obs_rows[mask], obs_cols[mask]],
            )

    def plan(self, start: np.ndarray, goal: np.ndarray) -> list:
        """Plan a 3D path from start to goal.

        Args:
            start: world coords [x, y, z]
            goal:  world coords [x, y, z]

        Returns:
            List of (x, y, z) tuples; empty list means planning failed.
            Caller (GlobalPlannerService) raises RuntimeError on empty return.
        """
        if self._planner is None:
            logger.error(
                "PCT planner unavailable (%s); cannot plan. "
                "Is this running with native PCT .so files matching this arch/Python ABI?",
                self._load_error,
            )
            return []

        start_pos = np.asarray(start[:2], dtype=np.float64)
        goal_pos  = np.asarray(goal[:2],  dtype=np.float64)
        # Choose heights that land on traversable tomogram slices at each XY.
        # A raw 2-D goal often arrives with z=0, while get_surface_height()
        # can snap to an upper slice whose traversability is a hard barrier.
        start_h = self._select_traversable_height(
            start_pos,
            float(start[2]) if len(start) > 2 else 0.0,
        )
        goal_h = self._select_traversable_height(
            goal_pos,
            float(goal[2]) if len(goal) > 2 else 0.0,
        )

        try:
            result = self._planner.plan(start_pos, goal_pos, start_h, goal_h)
        except Exception:
            logger.exception(
                "PCT plan() raised exception for start=%s goal=%s", start, goal
            )
            return []

        if result is None or len(result) == 0:
            logger.warning(
                "PCT plan() returned no path: start=%s goal=%s", start, goal
            )
            return []

        # result is np.ndarray (N, 3) in world coords [x, y, z]
        try:
            arr = np.asarray(result, dtype=np.float64)
            if arr.ndim == 2 and arr.shape[1] >= 3:
                return [(float(p[0]), float(p[1]), float(p[2])) for p in arr]
            elif arr.ndim == 2 and arr.shape[1] == 2:
                return [(float(p[0]), float(p[1]), goal_h) for p in arr]
            else:
                logger.error("Unexpected PCT result shape: %s", arr.shape)
                return []
        except Exception:
            logger.exception("PCT result conversion failed")
            return []

    def _select_traversable_height(self, pos: np.ndarray, fallback_z: float) -> float:
        """Return a height that maps to a traversable tomogram slice."""
        fallback = float(fallback_z) if np.isfinite(fallback_z) else 0.0
        planner = self._planner
        trav = getattr(planner, "layers_t", None)
        if trav is None:
            trav = getattr(self, "_trav_3d", None)
        if planner is None or trav is None:
            return fallback

        trav = np.asarray(trav, dtype=np.float32)
        if trav.ndim != 3 or trav.shape[0] == 0:
            return fallback

        try:
            idx = planner.pos2idx(np.asarray(pos[:2], dtype=np.float64))
            col = int(np.clip(idx[0], 0, trav.shape[2] - 1))
            row = int(np.clip(idx[1], 0, trav.shape[1] - 1))
        except Exception:
            if self._resolution <= 0:
                return fallback
            col = int(round((float(pos[0]) - self._origin[0]) / self._resolution))
            row = int(round((float(pos[1]) - self._origin[1]) / self._resolution))
            col = int(np.clip(col, 0, trav.shape[2] - 1))
            row = int(np.clip(row, 0, trav.shape[1] - 1))

        elev = getattr(planner, "layers_g", None)
        if elev is not None:
            elev = np.asarray(elev, dtype=np.float32)
            if elev.shape != trav.shape:
                elev = None

        def to_slice(z: float) -> int:
            try:
                raw_slice = round(float(planner.pos2slice(float(z))))
                return int(
                    np.clip(raw_slice, 0, trav.shape[0] - 1)
                )
            except Exception:
                if self._slice_dh == 0:
                    return 0
                return int(
                    np.clip(
                        round((float(z) - self._slice_h0) / self._slice_dh),
                        0,
                        trav.shape[0] - 1,
                    )
                )

        def slice_height(slice_idx: int, preferred: float | None = None) -> float:
            if (
                preferred is not None
                and np.isfinite(preferred)
                and to_slice(preferred) == slice_idx
            ):
                return float(preferred)
            if elev is not None:
                height = float(elev[slice_idx, row, col])
                if np.isfinite(height):
                    return height
            return float(self._slice_h0 + slice_idx * self._slice_dh)

        candidates: list[tuple[int, float | None]] = [(to_slice(fallback), fallback)]
        try:
            surface_h = float(
                planner.get_surface_height(np.asarray(pos[:2], dtype=np.float64))
            )
            if np.isfinite(surface_h):
                candidates.append((to_slice(surface_h), surface_h))
        except Exception:
            pass
        candidates.extend((slice_idx, None) for slice_idx in range(trav.shape[0]))

        seen: set[int] = set()
        for slice_idx, preferred_h in candidates:
            if slice_idx in seen:
                continue
            seen.add(slice_idx)
            cost = float(trav[slice_idx, row, col])
            if np.isfinite(cost) and cost < self._obstacle_thr:
                return slice_height(slice_idx, preferred_h)

        return fallback


# ---------------------------------------------------------------------------
# A* backend: cross-platform fallback for dev/sim (not production)
# ---------------------------------------------------------------------------

@register("planner_backend", "astar",
          description="Pure Python A* on tomogram ground-floor, dev/sim only, NOT for production")
class _AStarBackend:
    """Cross-platform 2D A* for development machines and CI.

    Uses only the ground-floor traversability slice from the tomogram.
    No trajectory optimisation, no 3D terrain awareness.

    This backend exists as a deterministic development/simulation fallback when
    native PCT libraries are unavailable or unsuitable for the current runtime.
    """

    def __init__(self, tomogram_path: str = "", obstacle_thr: float = 49.9):
        self._grid: np.ndarray | None = None
        self._static_grid: np.ndarray | None = None
        self._costmap: np.ndarray | None = None
        self._costmap_resolution = 0.2
        self._costmap_origin = np.zeros(2)
        self._resolution = 0.2
        self._origin = np.zeros(2)
        self._obstacle_thr = obstacle_thr
        if tomogram_path and os.path.exists(tomogram_path):
            self._load_tomogram(tomogram_path)

    def _load_tomogram(self, path: str) -> None:
        import pickle
        with open(path, "rb") as f:
            raw = pickle.load(f)
        if not isinstance(raw, dict):
            logger.error("A* backend: unexpected tomogram format in %s", path)
            return
        tomo_data = raw.get("data")
        res = float(raw.get("resolution", 0.2))
        if tomo_data is not None and hasattr(tomo_data, "ndim") and tomo_data.ndim == 4:
            # data shape: (5, n_slices, H, W), channel 0 = traversability
            grid = np.asarray(tomo_data[0, 0], dtype=np.float32)
            if "slice_h0" in raw and "slice_dh" in raw:
                grid = grid.T
            self._grid = grid
            center = np.array(raw.get("center", [0, 0])[:2], dtype=np.float64)
            h, w = self._grid.shape
            self._origin = center - np.array([w * res / 2, h * res / 2])
        else:
            self._grid = raw.get("grid", raw.get("traversability"))
            if self._grid is not None:
                self._grid = np.asarray(self._grid, dtype=np.float32)
            self._origin = np.array(raw.get("origin", [0, 0])[:2], dtype=np.float64)
        self._resolution = res
        self._static_grid = self._grid.copy() if self._grid is not None else None
        logger.info(
            "A* backend: loaded %s  grid=%s  res=%.3f",
            path,
            self._grid.shape if self._grid is not None else None,
            res,
        )

    def update_map(self, grid: np.ndarray, resolution: float = 0.2,
                   origin: np.ndarray | None = None) -> None:
        """Live costmap update; replaces the static pickle-loaded grid."""
        self._costmap = np.asarray(grid, dtype=np.float32)
        self._costmap_resolution = resolution
        if origin is not None:
            self._costmap_origin = np.array(origin[:2], dtype=np.float64)
        if self._static_grid is None:
            self._grid = self._costmap
            self._resolution = resolution
            if origin is not None:
                self._origin = np.array(origin[:2], dtype=np.float64)
            return
        self._merge_costmap()

    def _merge_costmap(self) -> None:
        if self._costmap is None or self._static_grid is None:
            return
        self._grid = self._static_grid.copy()

        cm = self._costmap
        cm_res = self._costmap_resolution
        cm_ox, cm_oy = self._costmap_origin[0], self._costmap_origin[1]
        g_res = self._resolution
        g_ox, g_oy = self._origin[0], self._origin[1]
        gh, gw = self._grid.shape

        obs_rows, obs_cols = np.where(cm >= self._obstacle_thr)
        if len(obs_rows) == 0:
            return

        world_x = cm_ox + obs_cols * cm_res
        world_y = cm_oy + obs_rows * cm_res
        grid_cols = np.round((world_x - g_ox) / g_res).astype(int)
        grid_rows = np.round((world_y - g_oy) / g_res).astype(int)

        mask = (grid_cols >= 0) & (grid_cols < gw) & (grid_rows >= 0) & (grid_rows < gh)
        valid_rows = grid_rows[mask]
        valid_cols = grid_cols[mask]
        if len(valid_rows) > 0:
            self._grid[valid_rows, valid_cols] = np.maximum(
                self._grid[valid_rows, valid_cols],
                cm[obs_rows[mask], obs_cols[mask]],
            )

    def plan(self, start: np.ndarray, goal: np.ndarray) -> list:
        """Plan a 2D path on the ground-floor traversability grid.

        Returns:
            List of (x, y, z) tuples, or [] if no path found.
        """
        if self._grid is None:
            logger.warning("A* backend: no map loaded")
            return []

        res = self._resolution
        nrows, ncols = self._grid.shape  # grid[row=y, col=x]
        gz = float(goal[2]) if len(goal) > 2 else 0.0

        def world2grid(wx: float, wy: float) -> tuple[int, int]:
            col = int(round((wx - self._origin[0]) / res))
            row = int(round((wy - self._origin[1]) / res))
            return (max(0, min(col, ncols - 1)), max(0, min(row, nrows - 1)))

        def grid2world(col: int, row: int) -> tuple[float, float]:
            return (col * res + self._origin[0], row * res + self._origin[1])

        def is_free(col: int, row: int) -> bool:
            return 0 <= col < ncols and 0 <= row < nrows and self._grid[row, col] < self._obstacle_thr

        sc, sr = world2grid(float(start[0]), float(start[1]))
        gc, gr = world2grid(float(goal[0]),  float(goal[1]))

        if sc == gc and sr == gr:
            return [(float(goal[0]), float(goal[1]), gz)]

        # 8-connected A* with Euclidean heuristic (admissible)
        def heuristic(c: int, r: int) -> float:
            return ((gc - c) ** 2 + (gr - r) ** 2) ** 0.5

        open_q = [(heuristic(sc, sr), 0.0, sc, sr)]
        g_score: dict = {(sc, sr): 0.0}
        came_from: dict = {}

        while open_q:
            _, g, cc, cr = heapq.heappop(open_q)
            if g > g_score.get((cc, cr), float('inf')) + 1e-9:
                continue  # stale entry
            if cc == gc and cr == gr:
                break
            for dc, dr in [(-1, 0), (1, 0), (0, -1), (0, 1),
                            (-1, -1), (-1, 1), (1, -1), (1, 1)]:
                nc, nr = cc + dc, cr + dr
                if not (0 <= nc < ncols and 0 <= nr < nrows):
                    continue
                if not is_free(nc, nr):
                    continue
                if dc and dr and (not is_free(cc + dc, cr) or not is_free(cc, cr + dr)):
                    continue
                step = 1.414 if dc and dr else 1.0
                ng = g + step
                if ng < g_score.get((nc, nr), float('inf')):
                    g_score[(nc, nr)] = ng
                    came_from[(nc, nr)] = (cc, cr)
                    heapq.heappush(open_q, (ng + heuristic(nc, nr), ng, nc, nr))

        if (gc, gr) not in came_from and (gc, gr) != (sc, sr):
            logger.warning(
                "A* backend: no path  start=(%d,%d) goal=(%d,%d)  grid=%s",
                sc, sr, gc, gr, self._grid.shape,
            )
            return []

        # Reconstruct by walking came_from back to start, include start point.
        path_cells: list[tuple[int, int]] = []
        cur = (gc, gr)
        while cur in came_from:
            path_cells.append(cur)
            cur = came_from[cur]
        path_cells.append((sc, sr))  # start point was missing before this fix
        path_cells.reverse()

        result = []
        for col, row in path_cells:
            wx, wy = grid2world(col, row)
            result.append((wx, wy, gz))

        # Ensure last point is exactly the requested goal
        if result and (abs(result[-1][0] - float(goal[0])) > res or
                       abs(result[-1][1] - float(goal[1])) > res):
            result.append((float(goal[0]), float(goal[1]), gz))

        return result
