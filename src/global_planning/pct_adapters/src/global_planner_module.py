"""Planner backends — A* and PCT (C++ ele_planner).

Used by GlobalPlannerService (nav/global_planner_service.py) via Registry:
    backend = get("planner_backend", "astar")
    backend = get("planner_backend", "pct")
"""

from __future__ import annotations

import logging
import os
import sys
import time
from typing import List, Optional

import numpy as np

from core.registry import register

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Backends (pure algorithm, no ROS2)
# ---------------------------------------------------------------------------

@register("planner_backend", "astar", description="Pure Python A* on tomogram grid")
class _AStarBackend:
    """Pure Python A* on tomogram grid."""

    def __init__(self, tomogram_path: str = "", obstacle_thr: float = 49.9):
        self._grid = None
        self._resolution = 1.0
        self._origin = np.zeros(2)
        self._obstacle_thr = obstacle_thr
        if tomogram_path and os.path.exists(tomogram_path):
            self._load_tomogram(tomogram_path)

    def _load_tomogram(self, path: str):
        import pickle
        with open(path, "rb") as f:
            raw = pickle.load(f)
        if isinstance(raw, dict):
            # Tomogram format: data=(5, N_slices, W, H), first channel = traversability
            tomo_data = raw.get("data")
            if tomo_data is not None and hasattr(tomo_data, "ndim") and tomo_data.ndim == 4:
                # Extract traversability layer (channel 0), use first slice
                self._grid = np.asarray(tomo_data[0, 0], dtype=np.float32)
            else:
                # Legacy format: grid or traversability key
                self._grid = raw.get("grid", raw.get("traversability"))
            self._resolution = raw.get("resolution", 1.0)
            origin = raw.get("center", raw.get("origin", [0, 0]))
            self._origin = np.array(origin[:2], dtype=np.float64)
        logger.info("A* loaded tomogram: %s, grid=%s", path,
                     self._grid.shape if self._grid is not None else None)

    def update_map(self, grid: np.ndarray, resolution: float = 0.2,
                   origin: np.ndarray = None):
        """Live costmap update — replaces the static pickle-loaded grid."""
        self._grid = grid
        self._resolution = resolution
        if origin is not None:
            self._origin = np.array(origin[:2])

    def plan(self, start: np.ndarray, goal: np.ndarray) -> list:
        if self._grid is None:
            logger.warning("A* planner: no map loaded, cannot plan")
            return []
        # World → grid coords
        si = int((start[0] - self._origin[0]) / self._resolution)
        sj = int((start[1] - self._origin[1]) / self._resolution)
        gi = int((goal[0] - self._origin[0]) / self._resolution)
        gj = int((goal[1] - self._origin[1]) / self._resolution)

        h, w = self._grid.shape
        si, sj = max(0, min(si, w-1)), max(0, min(sj, h-1))
        gi, gj = max(0, min(gi, w-1)), max(0, min(gj, h-1))

        # A* search
        import heapq
        open_set = [(0, si, sj)]
        came_from = {}
        g_score = {(si, sj): 0}
        closed = set()

        while open_set:
            _, cx, cy = heapq.heappop(open_set)
            if (cx, cy) in closed:
                continue
            closed.add((cx, cy))

            if cx == gi and cy == gj:
                break

            for dx, dy in [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]:
                nx, ny = cx + dx, cy + dy
                if 0 <= nx < w and 0 <= ny < h and (nx, ny) not in closed:
                    if self._grid[ny, nx] > self._obstacle_thr:
                        continue
                    cost = 1.414 if abs(dx) + abs(dy) == 2 else 1.0
                    ng = g_score[(cx, cy)] + cost
                    if ng < g_score.get((nx, ny), float('inf')):
                        g_score[(nx, ny)] = ng
                        f = ng + ((gi-nx)**2 + (gj-ny)**2)**0.5
                        heapq.heappush(open_set, (f, nx, ny))
                        came_from[(nx, ny)] = (cx, cy)

        # Reconstruct path
        if (gi, gj) not in came_from and (gi, gj) != (si, sj):
            return []
        path = []
        cur = (gi, gj)
        while cur in came_from:
            wx = cur[0] * self._resolution + self._origin[0]
            wy = cur[1] * self._resolution + self._origin[1]
            path.append((wx, wy, goal[2] if len(goal) > 2 else 0.0))
            cur = came_from[cur]
        path.reverse()
        return path


@register("planner_backend", "pct", description="C++ ele_planner via .so (aarch64 only)")
class _PCTBackend:
    """C++ ele_planner via .so (aarch64 only)."""

    def __init__(self, tomogram_path: str = "", obstacle_thr: float = 50):
        self._planner = None
        self._tomogram_path = tomogram_path
        self._obstacle_thr = obstacle_thr
        try:
            sys.path.insert(0, os.path.join(
                os.path.dirname(__file__), "..", "..", "PCT_planner", "planner", "lib"))
            from planner_wrapper import TomogramPlanner
            self._planner = TomogramPlanner(tomogram_path)
            logger.info("PCT ele_planner loaded: %s", tomogram_path)
        except ImportError:
            logger.warning("ele_planner.so not available — use planner='astar' instead")

    def plan(self, start: np.ndarray, goal: np.ndarray) -> list:
        if self._planner is None:
            return []
        try:
            result = self._planner.plan(
                start[0], start[1], start[2] if len(start) > 2 else 0.0,
                goal[0], goal[1], goal[2] if len(goal) > 2 else 0.0,
            )
            if result is not None:
                return [(p[0], p[1], p[2]) for p in result]
        except Exception:
            logger.exception("PCT planning failed")
        return []
