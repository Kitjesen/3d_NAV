from __future__ import annotations

import numpy as np
import pytest

from nav.global_planner_service import GlobalPlannerService
from nav.navigation_module import NavigationModule
from nav.plan_safety import (
    PlanSafetyGrid,
    evaluate_backend_path_safety,
    evaluate_plan_safety,
    grid_from_tomogram,
)


def test_plan_safety_supports_raw_tomogram_xy_order() -> None:
    grid = np.zeros((4, 3), dtype=np.float32)
    grid[1, 0] = 100.0
    tomo = {
        "data": grid.reshape(1, 1, 4, 3),
        "resolution": 1.0,
        "origin": [0.0, 0.0],
    }

    report = evaluate_plan_safety(
        [[0.0, 0.0, 0.0], [2.0, 0.0, 0.0]],
        grid_from_tomogram(tomo),
        obstacle_thr=49.9,
        max_step_m=0.5,
    )

    assert report["ok"] is False
    assert report["blocked_sample_count"] >= 1
    assert report["index_order"] == "xy"


def test_plan_safety_supports_backend_yx_order() -> None:
    class Backend:
        _grid = np.zeros((3, 4), dtype=np.float32)
        _resolution = 1.0
        _origin = np.asarray([0.0, 0.0], dtype=float)

    Backend._grid[0, 1] = 100.0

    report = evaluate_backend_path_safety(
        [[0.0, 0.0, 0.0], [2.0, 0.0, 0.0]],
        Backend(),
        obstacle_thr=49.9,
    )

    assert report is not None
    assert report["ok"] is False
    assert report["index_order"] == "yx"


def test_global_planner_service_can_fallback_after_unsafe_plan(monkeypatch: pytest.MonkeyPatch) -> None:
    class UnsafeBackend:
        available = True
        _grid = np.zeros((3, 3), dtype=np.float32)
        _resolution = 1.0
        _origin = np.asarray([0.0, 0.0], dtype=float)

        def plan(self, start, goal):
            return [[0.0, 1.0, 0.0], [2.0, 1.0, 0.0]]

    UnsafeBackend._grid[1, 1] = 100.0

    class SafeBackend:
        available = True
        _grid = UnsafeBackend._grid
        _resolution = 1.0
        _origin = UnsafeBackend._origin

        def plan(self, start, goal):
            return [[0.0, 1.0, 0.0], [0.0, 2.0, 0.0], [2.0, 2.0, 0.0], [2.0, 1.0, 0.0]]

    svc = GlobalPlannerService(
        planner_name="pct",
        obstacle_thr=49.9,
        downsample_dist=0.1,
        plan_safety_policy="fallback_astar",
        fallback_planner_name="astar",
    )
    svc._backend = UnsafeBackend()
    monkeypatch.setattr(svc, "_create_backend", lambda name=None: SafeBackend())

    path, _plan_ms = svc.plan(
        np.asarray([0.0, 1.0, 0.0], dtype=float),
        np.asarray([2.0, 1.0, 0.0], dtype=float),
        safe_goal_tolerance=0.0,
    )

    assert [[float(p[0]), float(p[1])] for p in path] == [[0.0, 1.0], [0.0, 2.0], [2.0, 2.0], [2.0, 1.0]]
    report = svc.last_plan_report
    assert report["selected_planner"] == "astar"
    assert report["rejected_plans"][0]["planner"] == "pct"
    assert report["selected_path_safety"]["ok"] is True


def test_global_planner_service_reports_when_fallback_is_unsafe(monkeypatch: pytest.MonkeyPatch) -> None:
    class UnsafeBackend:
        _grid = np.zeros((3, 3), dtype=np.float32)
        _resolution = 1.0
        _origin = np.asarray([0.0, 0.0], dtype=float)

        def plan(self, start, goal):
            return [[0.0, 1.0, 0.0], [2.0, 1.0, 0.0]]

    UnsafeBackend._grid[1, 1] = 100.0

    svc = GlobalPlannerService(
        planner_name="pct",
        obstacle_thr=49.9,
        plan_safety_policy="fallback_astar",
        fallback_planner_name="astar",
    )
    svc._backend = UnsafeBackend()
    monkeypatch.setattr(svc, "_create_backend", lambda name=None: UnsafeBackend())

    with pytest.raises(RuntimeError, match="fallback planner did not produce a safe path"):
        svc.plan(
            np.asarray([0.0, 1.0, 0.0], dtype=float),
            np.asarray([2.0, 1.0, 0.0], dtype=float),
            safe_goal_tolerance=0.0,
        )

    report = svc.last_plan_report
    assert report["policy"] == "fallback_astar"
    assert [entry["planner"] for entry in report["rejected_plans"]] == ["pct", "astar"]


def test_global_planner_service_can_reject_unsafe_plan() -> None:
    class UnsafeBackend:
        _grid = np.zeros((3, 3), dtype=np.float32)
        _resolution = 1.0
        _origin = np.asarray([0.0, 0.0], dtype=float)

        def plan(self, start, goal):
            return [[0.0, 1.0, 0.0], [2.0, 1.0, 0.0]]

    UnsafeBackend._grid[1, 1] = 100.0

    svc = GlobalPlannerService(plan_safety_policy="reject", obstacle_thr=49.9)
    svc._backend = UnsafeBackend()

    with pytest.raises(RuntimeError, match="path_safety failed"):
        svc.plan(
            np.asarray([0.0, 1.0, 0.0], dtype=float),
            np.asarray([2.0, 1.0, 0.0], dtype=float),
            safe_goal_tolerance=0.0,
        )


def test_global_planner_service_fallback_policy_rejects_unsafe_astar_without_distinct_fallback() -> None:
    class UnsafeAstarBackend:
        _grid = np.zeros((3, 3), dtype=np.float32)
        _resolution = 1.0
        _origin = np.asarray([0.0, 0.0], dtype=float)

        def plan(self, start, goal):
            return [[0.0, 1.0, 0.0], [2.0, 1.0, 0.0]]

    UnsafeAstarBackend._grid[1, 1] = 100.0

    svc = GlobalPlannerService(
        planner_name="astar",
        plan_safety_policy="fallback_astar",
        fallback_planner_name="astar",
        obstacle_thr=49.9,
    )
    svc._backend = UnsafeAstarBackend()

    with pytest.raises(RuntimeError, match="no distinct fallback planner"):
        svc.plan(
            np.asarray([0.0, 1.0, 0.0], dtype=float),
            np.asarray([2.0, 1.0, 0.0], dtype=float),
            safe_goal_tolerance=0.0,
        )
    assert svc.last_plan_report["selected_path_safety"]["ok"] is False


def test_navigation_module_exposes_plan_safety_policy() -> None:
    nav = NavigationModule(
        planner="pct",
        plan_safety_policy="fallback_astar",
        fallback_planner_name="astar",
    )

    summary = nav.health()["navigation"]

    assert summary["planner"] == "pct"
    assert summary["plan_safety_policy"] == "fallback_astar"
