"""GlobalPlannerService — pure-algorithm planning backend wrapper.

Not a Module. Used internally by NavigationModule to separate planning
logic from the mission FSM.
"""
from __future__ import annotations

import logging
import os
import time
from typing import Any, List, Optional, Tuple

import numpy as np

from nav.plan_safety import evaluate_backend_path_safety

logger = logging.getLogger(__name__)


class GlobalPlannerService:
    """Wraps a planner backend: create → plan → downsample.

    Not a Module — no ports, no lifecycle. Just algorithm logic.
    Swap the backend by passing a different planner_name.
    """

    def __init__(
        self,
        planner_name: str = "astar",
        tomogram: str = "",
        obstacle_thr: float = 49.9,
        downsample_dist: float = 2.0,
        plan_safety_policy: str = "observe",
        fallback_planner_name: str = "astar",
    ) -> None:
        self._planner_name = planner_name
        self._tomogram = tomogram
        self._obstacle_thr = obstacle_thr
        self._downsample_dist = downsample_dist
        self._plan_safety_policy = plan_safety_policy
        self._fallback_planner_name = fallback_planner_name
        self._backend = None
        self._fallback_backend = None
        self._last_plan_report: dict[str, Any] = {}
        self._warned_no_grid: bool = False

    def setup(self) -> None:
        """Create the planner backend. Must be called before plan()."""
        self._backend = self._create_backend()

    @property
    def is_ready(self) -> bool:
        return self._backend is not None

    @property
    def has_map(self) -> bool:
        if self._backend is None:
            return False
        grid = getattr(self._backend, "_grid", None)
        return grid is not None and hasattr(grid, "shape") and getattr(grid, "size", 0) > 0

    def plan(
        self,
        start: np.ndarray,
        goal: np.ndarray,
        safe_goal_tolerance: float = 4.0,
    ) -> tuple[list[np.ndarray], float]:
        """Plan a path from start to goal.

        If the goal lands on an obstacle, BFS-searches the nearest free cell
        within safe_goal_tolerance meters (reference: dimos _find_safe_goal).

        Returns:
            (path, plan_ms) where path is list of np.ndarray([x, y, z]).
        Raises:
            RuntimeError if backend not ready or planner returns empty path.
        """
        if self._backend is None:
            raise RuntimeError("GlobalPlannerService: backend not set up")

        # Goal safety: if backend exposes a costmap, verify goal is in free space
        safe_goal = self._find_safe_goal(goal, tolerance=safe_goal_tolerance)
        if safe_goal is not None:
            if not np.array_equal(safe_goal[:2], goal[:2]):
                logger.info(
                    "Goal adjusted: (%.1f,%.1f) → (%.1f,%.1f) (nearest free cell)",
                    goal[0], goal[1], safe_goal[0], safe_goal[1],
                )
            goal = safe_goal

        raw_path, plan_ms = self._plan_with_backend(self._backend, start, goal)

        if not raw_path:
            raise RuntimeError("GlobalPlannerService: planner returned empty path")

        selected_backend = self._backend
        selected_planner = self._planner_name
        selected_path = raw_path
        selected_plan_ms = plan_ms
        selected_safety = self._evaluate_path_safety(selected_backend, selected_path)
        rejected_plans: list[dict[str, Any]] = []
        fallback_reason = ""
        if selected_safety is not None and not selected_safety.get("ok", True):
            fallback_reason = (
                f"{self._planner_name} path_safety failed "
                f"({selected_safety.get('blocked_sample_count', 0)} blocked samples)"
            )
            rejected_plans.append(
                {
                    "planner": self._planner_name,
                    "path_safety": selected_safety,
                    "reason": fallback_reason,
                }
            )
            if self._plan_safety_policy == "reject":
                self._last_plan_report = {
                    "selected_planner": self._planner_name,
                    "selected_path_safety": selected_safety,
                    "rejected_plans": rejected_plans,
                    "fallback_reason": fallback_reason,
                    "policy": self._plan_safety_policy,
                }
                raise RuntimeError(f"GlobalPlannerService: {fallback_reason}")
            if self._plan_safety_policy == "fallback_astar":
                if self._planner_name.lower() == self._fallback_planner_name.lower():
                    self._last_plan_report = {
                        "selected_planner": self._planner_name,
                        "selected_path_safety": selected_safety,
                        "rejected_plans": rejected_plans,
                        "fallback_reason": fallback_reason,
                        "policy": self._plan_safety_policy,
                    }
                    raise RuntimeError(
                        "GlobalPlannerService: plan safety failed and no distinct fallback planner is configured"
                    )
                fb_backend = self._get_fallback_backend()
                fb_path, fb_ms = self._plan_with_backend(fb_backend, start, goal)
                fb_safety = self._evaluate_path_safety(fb_backend, fb_path)
                if fb_path and (fb_safety is None or fb_safety.get("ok", False)):
                    selected_backend = fb_backend
                    selected_planner = self._fallback_planner_name
                    selected_path = fb_path
                    selected_plan_ms += fb_ms
                    selected_safety = fb_safety
                    logger.warning(
                        "GlobalPlannerService: %s; using fallback planner '%s'",
                        fallback_reason,
                        self._fallback_planner_name,
                    )
                else:
                    rejected_plans.append(
                        {
                            "planner": self._fallback_planner_name,
                            "path_safety": fb_safety,
                            "reason": "fallback planner unsafe or empty",
                        }
                    )
                    self._last_plan_report = {
                        "selected_planner": self._planner_name,
                        "selected_path_safety": selected_safety,
                        "rejected_plans": rejected_plans,
                        "fallback_reason": fallback_reason,
                        "policy": self._plan_safety_policy,
                    }
                    raise RuntimeError(
                        "GlobalPlannerService: plan safety failed and fallback planner did not produce a safe path"
                    )
            elif self._plan_safety_policy == "observe":
                logger.warning("GlobalPlannerService: %s", fallback_reason)

        path = self._downsample(selected_path, goal)
        self._last_plan_report = {
            "selected_planner": selected_planner,
            "selected_path_safety": selected_safety,
            "fallback_reason": fallback_reason,
            "rejected_plans": rejected_plans,
            "policy": self._plan_safety_policy,
        }
        logger.info(
            "Planned %d waypoints in %.1fms (planner=%s)",
            len(path), selected_plan_ms, selected_planner,
        )
        return path, selected_plan_ms

    def update_map(
        self,
        grid: np.ndarray,
        resolution: float = 0.2,
        origin: np.ndarray | None = None,
    ) -> None:
        """Push live costmap to the backend (if supported)."""
        if self._backend is not None and hasattr(self._backend, "update_map"):
            self._backend.update_map(grid, resolution=resolution, origin=origin)

    # ------------------------------------------------------------------ #
    # Goal Safety (BFS nearest free cell, ref: dimos _find_safe_goal)      #
    # ------------------------------------------------------------------ #

    def _find_safe_goal(
        self,
        goal: np.ndarray,
        tolerance: float = 4.0,
    ) -> np.ndarray | None:
        """BFS-search nearest free cell if goal is on an obstacle.

        Returns adjusted goal (np.ndarray) or None if no costmap available.
        If goal is already free, returns it unchanged.
        """
        if self._backend is None:
            return None

        # Backend must expose costmap grid + resolution + origin
        grid = getattr(self._backend, "_grid", None)
        resolution = getattr(self._backend, "_resolution", 0.2)
        origin = getattr(self._backend, "_origin", None)

        if grid is None or not hasattr(grid, "shape") or grid.ndim != 2:
            if not self._warned_no_grid:
                logger.warning(
                    "GlobalPlannerService: _find_safe_goal bypassed — "
                    "backend '%s' has no 2D _grid attribute. "
                    "Goal safety checking is DISABLED.",
                    self._planner_name,
                )
                self._warned_no_grid = True
            return None

        # Convert world → grid coordinates
        ox = origin[0] if origin is not None else 0.0
        oy = origin[1] if origin is not None else 0.0
        gx = int(round((goal[0] - ox) / resolution))
        gy = int(round((goal[1] - oy) / resolution))
        h, w = grid.shape

        # If goal is in bounds and free, no adjustment needed
        if 0 <= gx < w and 0 <= gy < h:
            if grid[gy, gx] < self._obstacle_thr:
                gz = float(goal[2]) if len(goal) > 2 else 0.0
                return np.array([goal[0], goal[1], gz])

        # BFS: expand outward from goal cell, find nearest free cell
        from collections import deque
        max_cells = int(tolerance / resolution)
        visited = set()
        queue = deque()
        queue.append((gx, gy, 0))
        visited.add((gx, gy))

        while queue:
            cx, cy, dist = queue.popleft()
            if dist > max_cells:
                break

            if 0 <= cx < w and 0 <= cy < h:
                if grid[cy, cx] < self._obstacle_thr:
                    # Found free cell — convert back to world
                    wx = ox + cx * resolution
                    wy = oy + cy * resolution
                    gz = float(goal[2]) if len(goal) > 2 else 0.0
                    return np.array([wx, wy, gz])

            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                nx, ny = cx + dx, cy + dy
                if (nx, ny) not in visited:
                    visited.add((nx, ny))
                    queue.append((nx, ny, dist + 1))

        logger.warning(
            "No free cell within %.1fm of goal (%.1f, %.1f)",
            tolerance, goal[0], goal[1],
        )
        return None  # No free cell found — let planner try anyway

    # ------------------------------------------------------------------ #
    # Internals                                                            #
    # ------------------------------------------------------------------ #

    def _plan_with_backend(
        self,
        backend: Any,
        start: np.ndarray,
        goal: np.ndarray,
    ) -> tuple[list, float]:
        t0 = time.time()
        raw_path = backend.plan(start, goal)
        return raw_path, (time.time() - t0) * 1000

    def _evaluate_path_safety(self, backend: Any, path: list) -> dict[str, Any] | None:
        if self._plan_safety_policy == "off":
            return None
        return evaluate_backend_path_safety(path, backend, obstacle_thr=self._obstacle_thr)

    def _get_fallback_backend(self):
        if self._fallback_backend is None:
            self._fallback_backend = self._create_backend(self._fallback_planner_name)
        return self._fallback_backend

    @property
    def last_plan_report(self) -> dict[str, Any]:
        return dict(self._last_plan_report)

    def _create_backend(self, name: str | None = None):
        from core.registry import get
        name = (name or self._planner_name).lower()
        # Trigger @register decorators for astar / pct backends
        try:
            import global_planning.pct_adapters.src.global_planner_module
        except ImportError:
            pass
        try:
            BackendCls = get("planner_backend", name)
        except KeyError as err:
            from core.registry import list_plugins
            available = list_plugins("planner_backend")
            raise ValueError(
                f"Unknown planner: '{name}'. Available: {available}"
            ) from err

        tomogram_path = self._tomogram
        if not tomogram_path or not os.path.exists(tomogram_path):
            active_tomo = os.path.join(
                os.environ.get("NAV_MAP_DIR", os.path.expanduser("~/data/inovxio/data/maps")),
                "active", "tomogram.pickle",
            )
            if os.path.exists(active_tomo):
                tomogram_path = active_tomo
                logger.info(
                    "GlobalPlannerService: using active map: %s", tomogram_path
                )

        return BackendCls(tomogram_path, self._obstacle_thr)

    def _downsample(self, path: list, goal: np.ndarray) -> list[np.ndarray]:
        if not path:
            return []
        result = [np.array(path[0][:3])]
        for p in path[1:]:
            pt = (np.array(p[:3]) if len(p) >= 3
                  else np.array([p[0], p[1], 0.0]))
            if np.linalg.norm(pt - result[-1]) >= self._downsample_dist:
                result.append(pt)
        goal_pt = (np.array(goal[:3]) if len(goal) >= 3
                   else np.array([goal[0], goal[1], 0.0]))
        if np.linalg.norm(goal_pt - result[-1]) > 1e-6:
            result.append(goal_pt)
        return result
