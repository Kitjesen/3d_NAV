"""Planner backends — A* (dev/sim) and PCT C++ (S100P production).

Used by GlobalPlannerService (nav/global_planner_service.py) via Registry:
    backend = get("planner_backend", "pct")    # S100P: ele_planner.so
    backend = get("planner_backend", "astar")  # dev/sim: pure Python fallback

See docs/02-architecture/PLANNER_SELECTION.md for the selection rationale.
"""

from __future__ import annotations

import heapq
import logging
import os
import sys
from typing import List, Optional, Tuple

import numpy as np

from core.registry import register

logger = logging.getLogger(__name__)

# Paths needed by planner_wrapper.py and the C++ .so bindings.
#
# planner_wrapper.py uses:
#   from lib import a_star, ele_planner, traj_opt   ← needs _PCT_PLANNER in sys.path
#   from utils import *                              ← utils.py is in scripts/
#
# The .so files (ele_planner, a_star, traj_opt) are inside lib/, so lib/ must
# also be on sys.path for Python to find them as extension modules.
_PCT_PLANNER = os.path.normpath(
    os.path.join(os.path.dirname(__file__), "..", "..", "PCT_planner", "planner")
)
_PCT_LIB = os.path.join(_PCT_PLANNER, "lib")
_PCT_SCRIPTS = os.path.join(_PCT_PLANNER, "scripts")


# ---------------------------------------------------------------------------
# PCT backend — production quality, S100P only
# ---------------------------------------------------------------------------

@register("planner_backend", "pct",
          description="C++ ele_planner via .so — 3D terrain-aware, S100P (aarch64) only")
class _PCTBackend:
    """Production planner: TomogramPlanner wrapping ele_planner.so + traj_opt.so.

    Capabilities:
      - 3D A* on multi-slice traversability grid (stairs, ramps)
      - GPMP trajectory optimisation (smooth, kinematically feasible)
      - Gradient-aware elevation planning

    Requires aarch64 compiled .so files — not available on x86_64 dev machines.
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

        self._try_load(tomogram_path)

    def _try_load(self, tomogram_path: str) -> None:
        """Try to import TomogramPlanner and load the tomogram.

        Sets self._available = True only when both the .so and the tomogram
        are successfully loaded.
        """
        # sys.path setup required by planner_wrapper.py:
        #   _PCT_PLANNER  → enables  `from lib import a_star, ele_planner, traj_opt`
        #   _PCT_LIB      → Python finds the .so extension modules inside lib/
        #   _PCT_SCRIPTS  → enables  `from utils import *`  (utils.py lives here)
        for p in [_PCT_PLANNER, _PCT_LIB, _PCT_SCRIPTS]:
            if p not in sys.path:
                sys.path.insert(0, p)

        try:
            from planner_wrapper import TomogramPlanner  # type: ignore
        except ImportError as e:
            self._load_error = (
                f"ele_planner.so not available ({e}). "
                f"This is expected on x86_64 dev machines. "
                f"Use planner='astar' for simulation/development."
            )
            logger.warning("PCT backend unavailable: %s", self._load_error)
            return

        if not tomogram_path or not os.path.exists(tomogram_path):
            self._load_error = (
                f"Tomogram file not found: '{tomogram_path}'. "
                f"Set tomogram= in your profile or copy the map to "
                f"~/data/nova/maps/active/tomogram.pickle"
            )
            logger.error("PCT backend: %s", self._load_error)
            return

        try:
            # TomogramPlanner.__init__ takes a cfg object, not a path.
            # We use loadTomogram() which accepts a path string directly.
            class _MinimalCfg:
                class planner:
                    use_quintic = True
                    max_heading_rate = 10
                    obstacle_thr = 49.9
                class wrapper:
                    tomo_dir = "/"
                    pcd_dir = None

            _MinimalCfg.planner.obstacle_thr = self._obstacle_thr
            planner = TomogramPlanner(_MinimalCfg)
            planner.loadTomogram(tomogram_path)
            self._planner = planner
            self._available = True
            logger.info(
                "PCT ele_planner loaded: %s  map_dim=%s slices=%s",
                tomogram_path,
                getattr(planner, "map_dim", "?"),
                getattr(planner, "n_slice", "?"),
            )
        except Exception as e:
            self._load_error = f"TomogramPlanner.loadTomogram failed: {e}"
            logger.exception("PCT backend failed to load tomogram: %s", tomogram_path)

    @property
    def available(self) -> bool:
        return self._available

    def plan(self, start: np.ndarray, goal: np.ndarray) -> list:
        """Plan a 3D path from start to goal.

        Args:
            start: world coords [x, y, z]
            goal:  world coords [x, y, z]

        Returns:
            List of (x, y, z) tuples — empty list means planning failed.
            Caller (GlobalPlannerService) raises RuntimeError on empty return.
        """
        if self._planner is None:
            logger.error(
                "PCT planner unavailable (%s) — cannot plan. "
                "Is this running on S100P with ele_planner.so compiled?",
                self._load_error,
            )
            return []

        start_pos = np.asarray(start[:2], dtype=np.float64)
        goal_pos  = np.asarray(goal[:2],  dtype=np.float64)
        start_h   = float(start[2]) if len(start) > 2 else 0.0
        goal_h    = float(goal[2])  if len(goal)  > 2 else 0.0

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


# ---------------------------------------------------------------------------
# A* backend — cross-platform fallback for dev/sim (not production)
# ---------------------------------------------------------------------------

@register("planner_backend", "astar",
          description="Pure Python A* on tomogram ground-floor — dev/sim only, NOT for production")
class _AStarBackend:
    """Cross-platform 2D A* for development machines and CI.

    Uses only the ground-floor traversability slice from the tomogram.
    No trajectory optimisation, no 3D terrain awareness.

    This backend exists solely because ele_planner.so is aarch64-only.
    Do NOT use in production (nav/explore profiles on S100P).
    """

    def __init__(self, tomogram_path: str = "", obstacle_thr: float = 49.9):
        self._grid: Optional[np.ndarray] = None
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
            # data shape: (5, n_slices, H, W) — channel 0 = traversability
            self._grid = np.asarray(tomo_data[0, 0], dtype=np.float32)
            center = np.array(raw.get("center", [0, 0])[:2], dtype=np.float64)
            h, w = self._grid.shape
            self._origin = center - np.array([w * res / 2, h * res / 2])
        else:
            self._grid = raw.get("grid", raw.get("traversability"))
            if self._grid is not None:
                self._grid = np.asarray(self._grid, dtype=np.float32)
            self._origin = np.array(raw.get("origin", [0, 0])[:2], dtype=np.float64)
        self._resolution = res
        logger.info(
            "A* backend: loaded %s  grid=%s  res=%.3f",
            path,
            self._grid.shape if self._grid is not None else None,
            res,
        )

    def update_map(self, grid: np.ndarray, resolution: float = 0.2,
                   origin: Optional[np.ndarray] = None) -> None:
        """Live costmap update — replaces the static pickle-loaded grid."""
        self._grid = np.asarray(grid, dtype=np.float32)
        self._resolution = resolution
        if origin is not None:
            self._origin = np.array(origin[:2], dtype=np.float64)

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

        def world2grid(wx: float, wy: float) -> Tuple[int, int]:
            col = int(round((wx - self._origin[0]) / res))
            row = int(round((wy - self._origin[1]) / res))
            return (max(0, min(col, ncols - 1)), max(0, min(row, nrows - 1)))

        def grid2world(col: int, row: int) -> Tuple[float, float]:
            return (col * res + self._origin[0], row * res + self._origin[1])

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
                if self._grid[nr, nc] >= self._obstacle_thr:
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

        # Reconstruct — walk came_from back to start, include start point
        path_cells: List[Tuple[int, int]] = []
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
