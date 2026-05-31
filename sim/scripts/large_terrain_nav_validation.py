#!/usr/bin/env python3
"""Validate large-terrain navigation assets without robot motion."""

from __future__ import annotations

import argparse
import json
import math
import pickle
import sys
import time
from pathlib import Path
from typing import Any, Iterable

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from nav.global_planner_service import GlobalPlannerService
from nav.plan_safety import evaluate_plan_safety, grid_from_tomogram, path_distance
from global_planning.pct_planner_runnable.runtime import inspect_pct_runtime
from sim.engine.scenarios.large_terrain_assets import (
    LargeTerrainAssets,
    TerrainRoute,
    build_large_terrain_assets,
)


DEFAULT_MATRIX_ROUTES = ("terrain_short", "terrain_long", "terrain_narrow_gap", "terrain_slope_bypass")
DEFAULT_PLANNERS = ("astar",)
SELECTION_POLICY = "first_route_ok_after_primary"


def _not_exercised_algorithm_backends() -> dict[str, dict[str, str]]:
    return {
        "local_planner": {
            "status": "not_exercised",
            "exercised_by": "large_terrain_global_planning_assets",
            "reason": "large_terrain validates global planning assets and path safety only",
        },
        "path_follower": {
            "status": "not_exercised",
            "exercised_by": "large_terrain_global_planning_assets",
            "reason": "large_terrain does not run tracking or cmd_vel generation",
        },
    }


def _route_map(assets: LargeTerrainAssets) -> dict[str, TerrainRoute]:
    return {route.name: route for route in assets.routes}


def _path_distance(path: list[tuple[float, float, float]] | list[list[float]]) -> float:
    return path_distance(path)


def _with_route_endpoints(
    route: TerrainRoute,
    path: list[tuple[float, float, float]] | list[list[float]],
) -> list[list[float]]:
    full = [[float(route.start[0]), float(route.start[1]), float(route.start[2])]]
    full.extend([[float(p[0]), float(p[1]), float(p[2]) if len(p) > 2 else 0.0] for p in path])
    full.append([float(route.goal[0]), float(route.goal[1]), float(route.goal[2])])

    deduped: list[list[float]] = []
    for point in full:
        if not deduped:
            deduped.append(point)
            continue
        if math.hypot(point[0] - deduped[-1][0], point[1] - deduped[-1][1]) > 1e-6:
            deduped.append(point)
    return deduped


def _path_safety(path: list[tuple[float, float, float]] | list[list[float]], tomo: dict[str, Any], *, obstacle_thr: float) -> dict[str, Any]:
    return evaluate_plan_safety(path, grid_from_tomogram(tomo), obstacle_thr=obstacle_thr)


def _gate_crossing(path: list[tuple[float, float, float]] | list[list[float]]) -> dict[str, Any]:
    xy = np.asarray([[float(p[0]), float(p[1])] for p in path], dtype=float)
    if len(xy) == 0:
        return {"checked": False, "passed_gate": False}
    near_wall = xy[np.abs(xy[:, 0]) < 0.45]
    if len(near_wall) == 0:
        return {"checked": True, "passed_gate": False, "min_y_at_wall": None}
    min_y = float(np.min(near_wall[:, 1]))
    max_y = float(np.max(near_wall[:, 1]))
    return {
        "checked": True,
        "passed_gate": bool(min_y > -1.30 and max_y < 1.15),
        "min_y_at_wall": round(min_y, 4),
        "max_y_at_wall": round(max_y, 4),
    }


def _pct_runtime_evidence() -> dict[str, Any]:
    try:
        info = inspect_pct_runtime(ROOT)
        evidence = {
            "ok": bool(info.get("ok")),
            "canonical_arch": info.get("canonical_arch"),
            "python_tag": info.get("python_tag"),
            "lib_dir": info.get("lib_dir"),
            "missing": info.get("missing", []),
            "shared_missing": info.get("shared_missing", []),
            "error": info.get("error", ""),
        }
        for key in (
            "known_good_python_tag",
            "python_abi_matches_known_good",
            "platform_system",
            "os_name",
            "native_binary_format",
            "host_platform_supported",
            "host_platform_blocker",
            "candidate_diagnostics",
            "recommended_build_command",
        ):
            if key in info:
                evidence[key] = info[key]
        return evidence
    except Exception as exc:
        return {"ok": False, "error": str(exc)}


def _service_plan_report(svc: Any) -> dict[str, Any]:
    try:
        report = getattr(svc, "last_plan_report", {}) or {}
    except Exception:
        return {}
    return dict(report) if isinstance(report, dict) else {}


def _planner_value(value: Any, default: str) -> str:
    return str(value or default).lower().strip()


def _plan_with_backend(
    planner_name: str,
    assets: LargeTerrainAssets,
    route: TerrainRoute,
    *,
    obstacle_thr: float,
) -> dict[str, Any]:
    planner_name = planner_name.lower().strip()
    native_runtime = _pct_runtime_evidence() if planner_name == "pct" else None
    environment_blocked = bool(
        planner_name == "pct"
        and isinstance(native_runtime, dict)
        and native_runtime.get("ok") is not True
    )
    svc = GlobalPlannerService(
        planner_name=planner_name,
        tomogram=str(assets.tomogram),
        downsample_dist=0.2,
        obstacle_thr=obstacle_thr,
    )
    try:
        svc.setup()
    except Exception as exc:
        return {
            "planner": planner_name,
            "planner_requested": planner_name,
            "selected_planner": planner_name,
            "fallback_reason": "",
            "plan_safety_policy": "",
            "rejected_plans": [],
            "backend_class": "",
            "feasible": False,
            "blocked": True,
            "error": str(exc),
            "native_backend_used": False,
            "native_runtime": native_runtime,
            "status": "blocked" if environment_blocked else "failed",
            "failure_category": "environment_runtime" if environment_blocked else "planner_runtime",
            "plan_ms": 0.0,
            "start": list(route.start),
            "goal": list(route.goal),
            "path": [],
        }
    backend = svc._backend
    backend_available = bool(getattr(backend, "available", True))
    start_t = time.perf_counter()
    plan_report: dict[str, Any] = {}
    try:
        path, plan_ms = svc.plan(
            np.asarray(route.start, dtype=float),
            np.asarray(route.goal, dtype=float),
            safe_goal_tolerance=0.0,
        )
        plan_report = _service_plan_report(svc)
        elapsed_ms = float(plan_ms if plan_ms is not None else (time.perf_counter() - start_t) * 1000.0)
        error = ""
    except Exception as exc:
        path = []
        plan_report = _service_plan_report(svc)
        elapsed_ms = (time.perf_counter() - start_t) * 1000.0
        error = str(exc)
    selected_planner = _planner_value(plan_report.get("selected_planner"), planner_name)
    primary_planner = _planner_value(plan_report.get("primary_planner"), planner_name)
    selected_backend = backend
    if selected_planner != planner_name:
        selected_backend = getattr(svc, "_fallback_backend", None) or backend
    selected_backend_available = bool(getattr(selected_backend, "available", True))
    native_backend_used = bool(selected_planner == "pct" and selected_backend_available and path)
    status = "passed" if path else "blocked" if environment_blocked else "failed"
    rejected_plans = plan_report.get("rejected_plans", [])
    return {
        "planner": planner_name,
        "planner_requested": primary_planner,
        "selected_planner": selected_planner,
        "fallback_reason": str(plan_report.get("fallback_reason") or ""),
        "plan_safety_policy": str(plan_report.get("policy") or ""),
        "rejected_plans": rejected_plans if isinstance(rejected_plans, list) else [],
        "backend_class": selected_backend.__class__.__name__ if selected_backend is not None else "",
        "backend_requested_class": backend.__class__.__name__ if backend is not None else "",
        "feasible": bool(path),
        "blocked": bool(not path),
        "backend_available": selected_backend_available,
        "backend_requested_available": backend_available,
        "native_backend_used": native_backend_used,
        "native_runtime": native_runtime,
        "status": status,
        "failure_category": (
            ""
            if path
            else "environment_runtime"
            if environment_blocked
            else "planner_runtime"
        ),
        "load_error": str(getattr(backend, "_load_error", "")) if backend is not None else "",
        "error": error,
        "plan_ms": round(float(elapsed_ms), 3),
        "start": list(route.start),
        "goal": list(route.goal),
        "path": [[float(p[0]), float(p[1]), float(p[2]) if len(p) > 2 else 0.0] for p in path],
    }


def _plan_evidence(
    route: TerrainRoute,
    plan: dict[str, Any],
    tomo: dict[str, Any],
    *,
    obstacle_thr: float,
) -> dict[str, Any]:
    path = _with_route_endpoints(route, plan.get("path") or [])
    direct = float(np.linalg.norm(np.asarray(route.goal[:2]) - np.asarray(route.start[:2])))
    routed = _path_distance(path)
    route_distance_tolerance_m = 0.05
    safety = _path_safety(path, tomo, obstacle_thr=obstacle_thr)
    gate = (
        _gate_crossing(path)
        if route.name in {"terrain_long", "terrain_narrow_gap", "terrain_complex_slalom"}
        else {"checked": False}
    )
    ok = (
        bool(plan.get("feasible"))
        and bool(safety["ok"])
        and routed + route_distance_tolerance_m >= route.min_routed_distance_m
        and (not gate.get("checked") or bool(gate.get("passed_gate")))
    )
    return {
        "ok": bool(ok),
        "path": path,
        "metrics": {
            "direct_distance_m": round(direct, 4),
            "route_distance_m": round(routed, 4),
            "min_required_route_distance_m": route.min_routed_distance_m,
            "route_distance_tolerance_m": route_distance_tolerance_m,
        },
        "path_safety": safety,
        "gate_crossing": gate,
    }


def _selection_evidence(planning: list[dict[str, Any]]) -> dict[str, Any]:
    primary = planning[0] if planning else {}
    selected = next(
        (
            plan
            for plan in planning
            if bool(plan.get("feasible")) and bool(plan.get("route_ok"))
        ),
        None,
    )
    rejected = [
        {
            "planner": str(plan.get("planner_requested") or plan.get("planner", "")),
            "selected_planner": str(plan.get("selected_planner") or plan.get("planner", "")),
            "feasible": bool(plan.get("feasible")),
            "route_ok": bool(plan.get("route_ok")),
            "reason": (
                "environment_blocked"
                if plan.get("failure_category") == "environment_runtime"
                else
                "not_feasible"
                if not plan.get("feasible")
                else "unsafe_or_invalid_route"
                if not plan.get("route_ok")
                else ""
            ),
        }
        for plan in planning
        if not (bool(plan.get("feasible")) and bool(plan.get("route_ok")))
    ]
    primary_planner = str(primary.get("planner_requested") or primary.get("planner", ""))
    selected_planner = str(selected.get("selected_planner") or selected.get("planner", "")) if selected else ""
    return {
        "policy": SELECTION_POLICY,
        "primary_planner": primary_planner,
        "selected_planner": selected_planner,
        "selected_route_ok": bool(selected),
        "fallback_used": bool(selected and primary and selected_planner != primary_planner),
        "rejected_planners": rejected,
    }


def run_validation(
    output_dir: str | Path,
    *,
    routes: Iterable[str] = DEFAULT_MATRIX_ROUTES,
    planners: Iterable[str] = DEFAULT_PLANNERS,
    obstacle_thr: float = 49.9,
) -> dict[str, Any]:
    assets = build_large_terrain_assets(output_dir)
    route_by_name = _route_map(assets)
    with assets.tomogram.open("rb") as fh:
        tomo = pickle.load(fh)

    cases = []
    all_ok = True
    planner_names = tuple(dict.fromkeys(planner.lower().strip() for planner in planners if planner.strip()))
    native_runtime = _pct_runtime_evidence() if "pct" in planner_names else None
    environment_blockers: list[str] = []
    if isinstance(native_runtime, dict) and native_runtime.get("ok") is not True:
        environment_blockers.append("PCT native runtime unavailable")
    for route_name in routes:
        if route_name not in route_by_name:
            raise ValueError(f"unknown route {route_name!r}")
        route = route_by_name[route_name]
        planning = [_plan_with_backend(planner, assets, route, obstacle_thr=obstacle_thr) for planner in planner_names]
        plan_evidence = [
            _plan_evidence(route, plan, tomo, obstacle_thr=obstacle_thr)
            for plan in planning
        ]
        for plan, evidence in zip(planning, plan_evidence):
            plan["metrics"] = evidence["metrics"]
            plan["path_safety"] = evidence["path_safety"]
            plan["gate_crossing"] = evidence["gate_crossing"]
            plan["route_ok"] = evidence["ok"]
        primary_idx = next((idx for idx, item in enumerate(planning) if item.get("feasible")), 0)
        primary = planning[primary_idx] if planning else {}
        primary_evidence = plan_evidence[primary_idx] if plan_evidence else {}
        safe_primary_idx = next(
            (
                idx
                for idx, item in enumerate(planning)
                if item.get("feasible") and bool((item.get("path_safety") or {}).get("ok"))
            ),
            None,
        )
        safe_primary = planning[safe_primary_idx] if safe_primary_idx is not None else None
        selection = _selection_evidence(planning)
        case_ok = (
            bool(planning)
            and all(bool(item.get("feasible")) for item in planning)
            and all(bool(item.get("route_ok")) for item in planning)
        )
        all_ok = all_ok and case_ok
        cases.append(
            {
                "route": route.name,
                "description": route.description,
                "ok": case_ok,
                "assets": {
                    "scene_xml": str(assets.scene_xml),
                    "tomogram": str(assets.tomogram),
                    "map_pcd": str(assets.map_pcd),
                    "metadata": str(assets.metadata),
                    "start": list(route.start),
                    "goal": list(route.goal),
                },
                "planning": planning,
                "primary_planner": primary.get("planner", ""),
                "safe_primary_planner": safe_primary.get("planner", "") if safe_primary else "",
                "selection": selection,
                "metrics": primary_evidence.get("metrics", {}),
                "path_safety": primary_evidence.get("path_safety", {}),
                "gate_crossing": primary_evidence.get("gate_crossing", {}),
            }
        )

    return {
        "schema_version": "lingtu.large_terrain_nav_validation.v1",
        "ok": all_ok,
        "simulation_only": True,
        "real_robot_motion": False,
        "cmd_vel_sent_to_hardware": False,
        "validation_level": "global_planning_assets",
        "selection_policy": SELECTION_POLICY,
        "algorithm_backends": _not_exercised_algorithm_backends(),
        "assets": {
            "scene_xml": str(assets.scene_xml),
            "tomogram": str(assets.tomogram),
            "map_pcd": str(assets.map_pcd),
            "metadata": str(assets.metadata),
        },
        "native_runtime": native_runtime,
        "environment_blockers": environment_blockers,
        "routes": list(routes),
        "planners": list(planner_names),
        "cases": cases,
    }


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=ROOT / "artifacts/large_terrain_nav_validation")
    parser.add_argument("--routes", default=",".join(DEFAULT_MATRIX_ROUTES))
    parser.add_argument("--planners", default=",".join(DEFAULT_PLANNERS))
    parser.add_argument("--json-out", type=Path, default=ROOT / "artifacts/large_terrain_nav_validation/report.json")
    parser.add_argument("--obstacle-thr", type=float, default=49.9)
    parser.add_argument("--strict", action="store_true")
    return parser


def main() -> int:
    args = _build_parser().parse_args()
    routes = tuple(item.strip() for item in args.routes.split(",") if item.strip())
    planners = tuple(item.strip() for item in args.planners.split(",") if item.strip())
    report = run_validation(args.output_dir, routes=routes, planners=planners, obstacle_thr=args.obstacle_thr)
    args.json_out.parent.mkdir(parents=True, exist_ok=True)
    args.json_out.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    if args.strict and not report.get("ok"):
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
