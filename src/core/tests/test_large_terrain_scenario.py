from __future__ import annotations

import json
import pickle

import numpy as np

from nav.global_planner_service import GlobalPlannerService
from sim.engine.scenarios.large_terrain_assets import build_large_terrain_assets
from sim.scripts.large_terrain_nav_validation import run_validation


def _route(assets, name: str):
    return next(route for route in assets.routes if route.name == name)


def _builder_cell(tomo: dict, x: float, y: float) -> float:
    origin = tomo["origin"]
    res = float(tomo["resolution"])
    ix = int(round((x - origin[0]) / res))
    iy = int(round((y - origin[1]) / res))
    return float(tomo["data"][0, 0, ix, iy])


def test_large_terrain_assets_write_schema_and_route_catalog(tmp_path):
    assets = build_large_terrain_assets(tmp_path)

    assert assets.scene_xml.exists()
    assert assets.tomogram.exists()
    assert assets.map_pcd.exists()
    assert assets.metadata.exists()

    metadata = json.loads(assets.metadata.read_text(encoding="utf-8"))
    assert metadata["name"] == "large_terrain_field"
    assert metadata["shape_xy"] == [121, 81]
    assert len(metadata["obstacles"]) >= 10
    assert {zone["name"] for zone in metadata["terrain_zones"]} >= {
        "rough_gravel_patch",
        "slope_ramp_band",
        "ditch_no_go",
    }
    assert {route["name"] for route in metadata["routes"]} >= {
        "terrain_short",
        "terrain_long",
        "terrain_narrow_gap",
        "terrain_slope_bypass",
        "terrain_complex_slalom",
        "terrain_patrol_loop",
    }


def test_large_terrain_tomogram_marks_obstacles_and_terrain_costs(tmp_path):
    assets = build_large_terrain_assets(tmp_path)

    with assets.tomogram.open("rb") as fh:
        tomo = pickle.load(fh)

    data = tomo["data"]
    assert data.shape == (5, 1, 121, 81)
    assert data[0, 0].max() >= 100.0
    assert _builder_cell(tomo, -3.5, 3.9) == 18.0
    assert _builder_cell(tomo, 4.8, 4.1) == 12.0
    assert _builder_cell(tomo, 4.1, -3.1) >= 100.0
    assert _builder_cell(tomo, -9.5, -5.6) < 49.9
    assert _builder_cell(tomo, 9.4, 5.4) < 49.9


def test_large_terrain_astar_route_uses_central_gate(tmp_path):
    assets = build_large_terrain_assets(tmp_path)
    route = _route(assets, "terrain_long")

    svc = GlobalPlannerService(
        planner_name="astar",
        tomogram=str(assets.tomogram),
        downsample_dist=0.2,
        obstacle_thr=49.9,
    )
    svc.setup()
    path, _plan_ms = svc.plan(
        np.asarray(route.start, dtype=float),
        np.asarray(route.goal, dtype=float),
        safe_goal_tolerance=0.0,
    )

    xy = np.asarray([[float(p[0]), float(p[1])] for p in path], dtype=float)
    assert len(xy) > 20
    assert np.linalg.norm(xy[-1] - np.asarray(route.goal[:2], dtype=float)) < 1e-6

    near_wall_crossing = xy[np.abs(xy[:, 0]) < 0.45]
    assert len(near_wall_crossing) > 0
    assert float(np.min(near_wall_crossing[:, 1])) > -1.30
    assert float(np.max(near_wall_crossing[:, 1])) < 1.15

    direct = float(np.linalg.norm(np.asarray(route.goal[:2]) - np.asarray(route.start[:2])))
    routed = float(np.sum(np.linalg.norm(np.diff(xy, axis=0), axis=1)))
    assert routed > direct * 1.05
    assert routed >= route.min_routed_distance_m


def test_large_terrain_complex_slalom_has_safe_astar_route(tmp_path):
    assets = build_large_terrain_assets(tmp_path)
    route = _route(assets, "terrain_complex_slalom")

    svc = GlobalPlannerService(
        planner_name="astar",
        tomogram=str(assets.tomogram),
        downsample_dist=0.2,
        obstacle_thr=49.9,
    )
    svc.setup()
    path, _plan_ms = svc.plan(
        np.asarray(route.start, dtype=float),
        np.asarray(route.goal, dtype=float),
        safe_goal_tolerance=0.0,
    )

    xy = np.asarray([[float(p[0]), float(p[1])] for p in path], dtype=float)
    assert len(xy) > 30
    assert np.linalg.norm(xy[-1] - np.asarray(route.goal[:2], dtype=float)) < 1e-6
    routed = float(np.sum(np.linalg.norm(np.diff(xy, axis=0), axis=1)))
    assert routed >= route.min_routed_distance_m


def test_large_terrain_validation_report_is_non_motion_and_route_safe(tmp_path):
    report = run_validation(tmp_path, routes=("terrain_long",))

    assert report["ok"] is True
    assert report["simulation_only"] is True
    assert report["real_robot_motion"] is False
    assert report["cmd_vel_sent_to_hardware"] is False
    case = report["cases"][0]
    assert case["route"] == "terrain_long"
    assert case["ok"] is True
    assert case["planning"][0]["feasible"] is True
    assert case["planning"][0]["route_ok"] is True
    assert case["planning"][0]["path_safety"]["ok"] is True
    assert case["planning"][0]["metrics"]["route_distance_m"] >= case["planning"][0]["metrics"]["min_required_route_distance_m"]
    assert case["path_safety"]["ok"] is True
    assert case["gate_crossing"]["passed_gate"] is True


def test_large_terrain_validation_records_blocked_pct_without_faking_success(tmp_path, monkeypatch):
    from sim.scripts import large_terrain_nav_validation as mod

    class BlockedService:
        def __init__(self, planner_name: str, tomogram: str, obstacle_thr: float, downsample_dist: float) -> None:
            self._backend = type(
                "BlockedPCT",
                (),
                {"available": False, "_load_error": "native library missing"},
            )()

        def setup(self) -> None:
            return None

        def plan(self, start, goal, safe_goal_tolerance=0.0):
            raise RuntimeError("GlobalPlannerService: planner returned empty path")

    monkeypatch.setattr(mod, "GlobalPlannerService", BlockedService)
    monkeypatch.setattr(mod, "_pct_runtime_evidence", lambda: {"ok": False, "missing": ["ele_planner"]})

    report = mod.run_validation(tmp_path, routes=("terrain_short",), planners=("pct",))

    assert report["ok"] is False
    plan = report["cases"][0]["planning"][0]
    assert plan["planner"] == "pct"
    assert plan["feasible"] is False
    assert plan["blocked"] is True
    assert plan["native_backend_used"] is False
    assert plan["native_runtime"]["ok"] is False
    assert plan["route_ok"] is False
    assert report["cases"][0]["selection"]["selected_planner"] == ""
    assert report["cases"][0]["selection"]["selected_route_ok"] is False


def test_large_terrain_validation_records_safe_fallback_selection(tmp_path, monkeypatch):
    from sim.scripts import large_terrain_nav_validation as mod

    RealService = mod.GlobalPlannerService

    class MixedService:
        def __init__(self, planner_name: str, tomogram: str, obstacle_thr: float, downsample_dist: float) -> None:
            self._planner_name = planner_name
            self._inner = RealService(
                planner_name="astar",
                tomogram=tomogram,
                obstacle_thr=obstacle_thr,
                downsample_dist=downsample_dist,
            )
            self._backend = type(
                "NativePCT",
                (),
                {"available": True, "_load_error": ""},
            )()

        def setup(self) -> None:
            if self._planner_name == "astar":
                self._inner.setup()
                self._backend = self._inner._backend

        def plan(self, start, goal, safe_goal_tolerance=0.0):
            if self._planner_name == "pct":
                return [[float(start[0]), float(start[1]), 0.0], [float(goal[0]), float(goal[1]), 0.0]], 1.0
            return self._inner.plan(start, goal, safe_goal_tolerance=safe_goal_tolerance)

    monkeypatch.setattr(mod, "GlobalPlannerService", MixedService)
    monkeypatch.setattr(mod, "_pct_runtime_evidence", lambda: {"ok": True, "missing": []})

    report = mod.run_validation(tmp_path, routes=("terrain_long",), planners=("pct", "astar"))

    case = report["cases"][0]
    assert case["ok"] is False
    assert case["planning"][0]["planner"] == "pct"
    assert case["planning"][0]["route_ok"] is False
    assert case["planning"][1]["planner"] == "astar"
    assert case["planning"][1]["route_ok"] is True
    assert case["selection"]["primary_planner"] == "pct"
    assert case["selection"]["selected_planner"] == "astar"
    assert case["selection"]["fallback_used"] is True
