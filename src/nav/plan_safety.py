"""Shared path safety checks for global planning and simulation gates."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Iterable, Literal, Sequence

import numpy as np

GridIndexOrder = Literal["yx", "xy"]


@dataclass(frozen=True)
class PlanSafetyGrid:
    """2D traversability grid with an explicit world-to-cell convention.

    `yx` means `grid[row_y, col_x]`, which is the convention used by the
    planner backends. `xy` means `grid[ix, iy]`, which is used by some raw
    generated tomogram artifacts before backend normalization.
    """

    grid: np.ndarray
    resolution: float
    origin: tuple[float, float]
    index_order: GridIndexOrder = "yx"

    def cost_at(self, x: float, y: float) -> float:
        col = int(round((float(x) - self.origin[0]) / self.resolution))
        row = int(round((float(y) - self.origin[1]) / self.resolution))
        if self.index_order == "xy":
            ix, iy = col, row
            if ix < 0 or iy < 0 or ix >= self.grid.shape[0] or iy >= self.grid.shape[1]:
                return float("inf")
            return float(self.grid[ix, iy])
        if row < 0 or col < 0 or row >= self.grid.shape[0] or col >= self.grid.shape[1]:
            return float("inf")
        return float(self.grid[row, col])


def path_distance(path: Sequence[Sequence[float]]) -> float:
    if len(path) < 2:
        return 0.0
    xy = np.asarray([[float(p[0]), float(p[1])] for p in path], dtype=float)
    return float(np.sum(np.linalg.norm(np.diff(xy, axis=0), axis=1)))


def path_samples(
    path: Sequence[Sequence[float]],
    *,
    max_step_m: float,
) -> list[tuple[float, float]]:
    if not path:
        return []
    xy = [(float(p[0]), float(p[1])) for p in path]
    samples: list[tuple[float, float]] = [xy[0]]
    for start, end in zip(xy, xy[1:]):
        dist = math.hypot(end[0] - start[0], end[1] - start[1])
        steps = max(1, int(math.ceil(dist / max(max_step_m, 1e-6))))
        for idx in range(1, steps + 1):
            alpha = idx / steps
            samples.append(
                (
                    start[0] + (end[0] - start[0]) * alpha,
                    start[1] + (end[1] - start[1]) * alpha,
                )
            )
    return samples


def evaluate_plan_safety(
    path: Sequence[Sequence[float]],
    grid: PlanSafetyGrid,
    *,
    obstacle_thr: float,
    max_step_m: float | None = None,
    blocked_sample_limit: int = 10,
) -> dict[str, Any]:
    if not path:
        return {
            "ok": False,
            "max_cost": None,
            "blocked_sample_count": 0,
            "blocked_samples": [],
            "sample_count": 0,
            "resolution": float(grid.resolution),
            "index_order": grid.index_order,
        }
    step = float(max_step_m) if max_step_m is not None else max(float(grid.resolution) * 0.5, 0.05)
    samples = path_samples(path, max_step_m=step)
    costs = [grid.cost_at(x, y) for x, y in samples]
    blocked = [
        {
            "x": round(float(samples[idx][0]), 4),
            "y": round(float(samples[idx][1]), 4),
            "cost": round(float(cost), 4) if math.isfinite(float(cost)) else float("inf"),
        }
        for idx, cost in enumerate(costs)
        if (not math.isfinite(float(cost))) or float(cost) >= obstacle_thr
    ]
    finite_costs = [float(cost) for cost in costs if math.isfinite(float(cost))]
    return {
        "ok": not blocked,
        "max_cost": round(float(max(finite_costs)), 4) if finite_costs else float("inf"),
        "blocked_sample_count": len(blocked),
        "blocked_samples": blocked[:blocked_sample_limit],
        "sample_count": len(samples),
        "resolution": float(grid.resolution),
        "index_order": grid.index_order,
    }


def grid_from_backend(backend: Any) -> PlanSafetyGrid | None:
    grid = getattr(backend, "_grid", None)
    if grid is None or not hasattr(grid, "shape"):
        return None
    arr = np.asarray(grid, dtype=np.float32)
    if arr.ndim != 2:
        return None
    origin = getattr(backend, "_origin", (0.0, 0.0))
    resolution = float(getattr(backend, "_resolution", 0.2))
    return PlanSafetyGrid(
        grid=arr,
        resolution=resolution,
        origin=(float(origin[0]), float(origin[1])),
        index_order="yx",
    )


def grid_from_tomogram(tomogram: dict[str, Any]) -> PlanSafetyGrid:
    data = tomogram.get("data")
    if data is not None:
        arr = np.asarray(data, dtype=np.float32)
        if arr.ndim == 4:
            grid = np.asarray(arr[0, 0], dtype=np.float32)
        elif arr.ndim == 3:
            grid = np.asarray(arr[0], dtype=np.float32)
        elif arr.ndim == 2:
            grid = arr
        else:
            raise ValueError(f"unsupported tomogram data shape: {arr.shape}")
    else:
        raw_grid = tomogram.get("grid", tomogram.get("traversability"))
        if raw_grid is None:
            raise ValueError("tomogram has no data/grid/traversability")
        grid = np.asarray(raw_grid, dtype=np.float32)
    origin = tomogram.get("origin")
    index_order: GridIndexOrder = "xy"
    if origin is None:
        center = np.asarray(tomogram.get("center", [0.0, 0.0])[:2], dtype=float)
        res = float(tomogram.get("resolution", 0.2))
        h, w = grid.shape
        origin_arr = center - np.asarray([w * res / 2.0, h * res / 2.0], dtype=float)
        origin = [float(origin_arr[0]), float(origin_arr[1])]
        index_order = "yx"
    return PlanSafetyGrid(
        grid=np.asarray(grid, dtype=np.float32),
        resolution=float(tomogram.get("resolution", 0.2)),
        origin=(float(origin[0]), float(origin[1])),
        index_order=index_order,
    )


def evaluate_backend_path_safety(
    path: Sequence[Sequence[float]],
    backend: Any,
    *,
    obstacle_thr: float,
) -> dict[str, Any] | None:
    grid = grid_from_backend(backend)
    if grid is None:
        return None
    return evaluate_plan_safety(path, grid, obstacle_thr=obstacle_thr)
