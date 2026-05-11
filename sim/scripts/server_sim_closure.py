#!/usr/bin/env python3
"""Summarize LingTu server-side simulation closure reports."""

from __future__ import annotations

import argparse
import json
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable

ROOT = Path(__file__).resolve().parents[2]


@dataclass(frozen=True)
class GateSpec:
    name: str
    description: str
    default_patterns: tuple[str, ...]
    command: str
    evaluator: Callable[[dict[str, Any]], tuple[bool, list[str], dict[str, Any]]]


def _load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _bool_false(report: dict[str, Any], key: str) -> bool:
    return report.get(key) is False


def _safe_float(value: Any, default: float = 999.0) -> float:
    try:
        return float(value)
    except Exception:
        return default


def _frame(report: dict[str, Any], key: str) -> str:
    frames = report.get("frames") or {}
    return str(frames.get(key) or "")


def _eval_large_terrain(report: dict[str, Any]) -> tuple[bool, list[str], dict[str, Any]]:
    blockers: list[str] = []
    if report.get("ok") is not True:
        blockers.append("report.ok is not true")
    if report.get("simulation_only") is not True:
        blockers.append("simulation_only is not true")
    if not _bool_false(report, "real_robot_motion"):
        blockers.append("real_robot_motion is not false")
    if not _bool_false(report, "cmd_vel_sent_to_hardware"):
        blockers.append("cmd_vel_sent_to_hardware is not false")

    cases = list(report.get("cases") or [])
    pct_plans = [
        plan
        for case in cases
        for plan in case.get("planning", [])
        if str(plan.get("planner")).lower() == "pct"
    ]
    native_pct = any(bool(plan.get("native_backend_used")) for plan in pct_plans)
    path_safe = all(bool((case.get("path_safety") or {}).get("ok", True)) for case in cases)
    if not pct_plans:
        blockers.append("no PCT planning case found")
    if not native_pct:
        blockers.append("no native PCT case found")
    if not path_safe:
        blockers.append("path safety failed")

    return not blockers, blockers, {
        "case_count": len(cases),
        "native_pct": native_pct,
        "path_safe": path_safe,
        "routes": [case.get("route") for case in cases],
    }


def _eval_native_pct_mujoco(report: dict[str, Any]) -> tuple[bool, list[str], dict[str, Any]]:
    blockers: list[str] = []
    if report.get("ok") is not True:
        blockers.append("report.ok is not true")
    if report.get("simulation_only") is not True:
        blockers.append("simulation_only is not true")
    if not _bool_false(report, "real_robot_motion"):
        blockers.append("real_robot_motion is not false")
    if not _bool_false(report, "cmd_vel_sent_to_hardware"):
        blockers.append("cmd_vel_sent_to_hardware is not false")
    if report.get("reached_goal") is not True:
        blockers.append("reached_goal is not true")
    if str(report.get("planner") or "").lower() != "pct":
        blockers.append("planner is not pct")
    if report.get("pct_native_backend_used") is not True:
        blockers.append("pct_native_backend_used is not true")
    obstacle_aware = report.get("obstacle_aware") or {}
    if obstacle_aware.get("enabled") is not True:
        blockers.append("obstacle_aware.enabled is not true")
    if int(obstacle_aware.get("metadata_points") or 0) <= 0:
        blockers.append("obstacle_aware.metadata_points missing")
    clearance = report.get("obstacle_clearance") or {}
    if clearance.get("checked") is not True:
        blockers.append("obstacle clearance was not checked")
    if clearance.get("collision") is True:
        blockers.append("collision is true")
    local_path_samples = report.get("local_path_samples") or []
    if not local_path_samples:
        blockers.append("local_path_samples missing")
    trajectory_quality = report.get("trajectory_quality") or {}
    if trajectory_quality.get("ok") is not True:
        blockers.append("trajectory_quality.ok is not true")
    if _safe_float(report.get("final_distance_m")) > 0.8:
        blockers.append("final_distance_m > 0.8")
    if _frame(report, "goal") and _frame(report, "goal") != "map":
        blockers.append("goal frame is not map")
    if _frame(report, "cmd_vel") and _frame(report, "cmd_vel") != "base_link":
        blockers.append("cmd_vel frame is not base_link")

    return not blockers, blockers, {
        "planner": report.get("planner"),
        "pct_native_backend_used": report.get("pct_native_backend_used"),
        "final_distance_m": report.get("final_distance_m"),
        "moved_m": report.get("moved_m"),
        "obstacle_aware": {
            "enabled": obstacle_aware.get("enabled"),
            "metadata_points": obstacle_aware.get("metadata_points"),
        },
        "local_path_sample_count": len(local_path_samples),
        "min_clearance_m": clearance.get("min_clearance_m"),
        "clearance_checked": clearance.get("checked"),
        "collision": clearance.get("collision"),
        "trajectory_quality": {
            "ok": trajectory_quality.get("ok"),
            "p95_lateral_error_m": trajectory_quality.get("p95_lateral_error_m"),
            "final_progress_ratio": trajectory_quality.get("final_progress_ratio"),
        },
        "frames": report.get("frames") or {},
    }


def _eval_dynamic_obstacle_local_planner(report: dict[str, Any]) -> tuple[bool, list[str], dict[str, Any]]:
    blockers: list[str] = []
    if report.get("ok") is not True:
        blockers.append("report.ok is not true")
    if report.get("simulation_only") is not True:
        blockers.append("simulation_only is not true")
    if not _bool_false(report, "real_robot_motion"):
        blockers.append("real_robot_motion is not false")
    if not _bool_false(report, "cmd_vel_sent_to_hardware"):
        blockers.append("cmd_vel_sent_to_hardware is not false")
    if report.get("backend_actual") != "nanobind":
        blockers.append("backend_actual is not nanobind")
    if report.get("dynamic_replan_verified") is not True:
        blockers.append("dynamic_replan_verified is not true")
    if report.get("obstacle_response_verified") is not True:
        blockers.append("obstacle_response_verified is not true")
    if report.get("clear_path_recovery_verified") is not True:
        blockers.append("clear_path_recovery_verified is not true")
    if _safe_float(report.get("min_clearance_m"), default=0.0) < 0.25:
        blockers.append("min_clearance_m < 0.25")

    phases = list(report.get("phases") or [])
    by_name = {str(phase.get("name") or ""): phase for phase in phases}
    for name in ("clear_initial", "obstacle_left", "obstacle_right", "obstacle_center", "clear_recovered"):
        if name not in by_name:
            blockers.append(f"{name} phase missing")
    if by_name.get("obstacle_left", {}).get("avoidance_side") != "right":
        blockers.append("left obstacle did not produce right detour")
    if by_name.get("obstacle_right", {}).get("avoidance_side") != "left":
        blockers.append("right obstacle did not produce left detour")
    if by_name.get("clear_recovered", {}).get("avoidance_side") != "straight":
        blockers.append("clear_recovered did not return straight")
    for phase in phases:
        if int(phase.get("path_count") or 0) < 20:
            blockers.append(f"{phase.get('name') or 'unknown'} path_count < 20")
        if str(phase.get("path_frame_id") or "") != "map":
            blockers.append(f"{phase.get('name') or 'unknown'} path frame is not map")

    return not blockers, blockers, {
        "backend_actual": report.get("backend_actual"),
        "dynamic_replan_verified": report.get("dynamic_replan_verified"),
        "obstacle_response_verified": report.get("obstacle_response_verified"),
        "clear_path_recovery_verified": report.get("clear_path_recovery_verified"),
        "min_clearance_m": report.get("min_clearance_m"),
        "phase_count": len(phases),
        "avoidance_sequence": [
            by_name.get("clear_initial", {}).get("avoidance_side"),
            by_name.get("obstacle_left", {}).get("avoidance_side"),
            by_name.get("obstacle_right", {}).get("avoidance_side"),
            by_name.get("obstacle_center", {}).get("avoidance_side"),
            by_name.get("clear_recovered", {}).get("avoidance_side"),
        ],
        "frames": report.get("frames") or {},
    }


def _eval_fastlio2_live(report: dict[str, Any]) -> tuple[bool, list[str], dict[str, Any]]:
    blockers: list[str] = []
    for key in (
        "ok",
        "simulation_only",
        "live_mujoco_lidar_verified",
        "live_mujoco_imu_verified",
        "slam_algorithm_output_verified",
        "bridge_verified",
    ):
        if report.get(key) is not True:
            blockers.append(f"{key} is not true")
    if not _bool_false(report, "real_robot_motion"):
        blockers.append("real_robot_motion is not false")
    if report.get("cmd_vel_sent_to_hardware") is not False:
        blockers.append("cmd_vel_sent_to_hardware is not false")
    if _frame(report, "published_lidar") and _frame(report, "published_lidar") != "body":
        blockers.append("published_lidar frame is not body")

    return not blockers, blockers, {
        "outputs": report.get("outputs") or {},
        "states_seen": report.get("states_seen") or [],
        "point_count": report.get("point_count") or {},
        "frames": report.get("frames") or {},
    }


def _eval_policy_nav(report: dict[str, Any]) -> tuple[bool, list[str], dict[str, Any]]:
    blockers: list[str] = []
    if report.get("passed") is not True:
        blockers.append("report.passed is not true")
    if report.get("simulation_only") is not True:
        blockers.append("simulation_only is not true")
    if not _bool_false(report, "real_robot_motion"):
        blockers.append("real_robot_motion is not false")
    if not _bool_false(report, "cmd_vel_sent_to_hardware"):
        blockers.append("cmd_vel_sent_to_hardware is not false")

    checks = list(report.get("checks") or [])
    policy_loaded = any(bool(check.get("policy_loaded")) for check in checks)
    nav_checks = [check for check in checks if check.get("mode") == "full_stack_policy_nav"]
    nav_passed = any(bool(check.get("passed")) for check in nav_checks)
    if not policy_loaded:
        blockers.append("no ONNX policy loaded")
    if not nav_passed:
        blockers.append("full_stack_policy_nav did not pass")
    for nav_check in nav_checks:
        seen = nav_check.get("seen") or {}
        if int(seen.get("direct_fallback", 0)) != 0:
            blockers.append("full_stack_policy_nav used direct_goal_fallback")
        for key in ("local_path", "path_follower_cmd", "mux_cmd", "waypoints"):
            if int(seen.get(key, 0)) <= 0:
                blockers.append(f"full_stack_policy_nav missing {key}")
    for check in checks:
        if check.get("cmd_vel_sent_to_hardware") is not False:
            blockers.append(f"{check.get('mode')} did not report hardware-safe cmd_vel")

    last_nav = nav_checks[-1] if nav_checks else {}
    return not blockers, blockers, {
        "check_count": len(checks),
        "policy_loaded": policy_loaded,
        "nav_passed": nav_passed,
        "policy_paths": [check.get("policy_path") for check in checks if check.get("policy_path")],
        "nav_seen": last_nav.get("seen") or {},
        "nav_state": last_nav.get("nav_state"),
        "nav_dist_to_goal_m": last_nav.get("dist_to_goal_m"),
        "frames": [check.get("frames") for check in checks if check.get("frames")],
    }


def _eval_gateway_dry_run(report: dict[str, Any]) -> tuple[bool, list[str], dict[str, Any]]:
    blockers: list[str] = []
    if report.get("ok") is not True:
        blockers.append("report.ok is not true")
    if report.get("simulation_only") is not True:
        blockers.append("simulation_only is not true")
    if not _bool_false(report, "real_robot_motion"):
        blockers.append("real_robot_motion is not false")
    if not _bool_false(report, "cmd_vel_sent_to_hardware"):
        blockers.append("cmd_vel_sent_to_hardware is not false")
    published = report.get("published") or {}
    if int(published.get("goal_pose", 0)) != 1:
        blockers.append("goal_pose publish count is not 1")
    if int(published.get("cmd_vel", 0)) != 0:
        blockers.append("cmd_vel was published")

    return not blockers, blockers, {
        "published": published,
        "frames": report.get("frames") or {},
        "target": report.get("target"),
    }


def _eval_routecheck_preflight(report: dict[str, Any]) -> tuple[bool, list[str], dict[str, Any]]:
    blockers: list[str] = []
    if report.get("mode") != "routecheck_non_motion":
        blockers.append("mode is not routecheck_non_motion")
    if report.get("outcome") != "pass":
        blockers.append("outcome is not pass")
    if int(report.get("exit_status") or 0) != 0:
        blockers.append("exit_status is not 0")
    if report.get("non_motion") is not True:
        blockers.append("non_motion is not true")
    if not report.get("map"):
        blockers.append("map is missing")
    goal = report.get("goal") or {}
    for key in ("x", "y", "yaw"):
        if key not in goal:
            blockers.append(f"goal.{key} is missing")

    phases = report.get("phases") or {}
    phase_evidence: dict[str, Any] = {}
    for name in ("baseline", "candidate"):
        phase = phases.get(name) or {}
        phase_evidence[name] = {
            "feasible": phase.get("feasible"),
            "count": phase.get("count"),
            "selected_planner": phase.get("selected_planner") or phase.get("planner"),
            "plan_safety_policy": phase.get("plan_safety_policy"),
            "path_safety_ok": phase.get("path_safety_ok"),
            "active_cmd_source_before": phase.get("active_cmd_source_before"),
            "rejected_plan_count": phase.get("rejected_plan_count"),
        }
        if phase.get("non_motion") is not True:
            blockers.append(f"{name}.non_motion is not true")
        if phase.get("can_accept_goal") is not True:
            blockers.append(f"{name}.can_accept_goal is not true")
        if str(phase.get("active_cmd_source_before") or "none").lower() not in {"none", "null", "-", ""}:
            blockers.append(f"{name}.active_cmd_source_before is not none")
        if phase.get("feasible") is not True:
            blockers.append(f"{name}.feasible is not true")
        if int(phase.get("count") or 0) < 2:
            blockers.append(f"{name}.count < 2")
        if not (phase.get("selected_planner") or phase.get("planner")):
            blockers.append(f"{name}.selected_planner is missing")
        if phase.get("path_safety_ok") is not True:
            blockers.append(f"{name}.path_safety_ok is not true")

    return not blockers, blockers, {
        "map": report.get("map"),
        "goal": goal,
        "outcome": report.get("outcome"),
        "phases": phase_evidence,
        "artifacts": report.get("artifacts") or {},
    }


def _eval_gazebo_runtime(report: dict[str, Any]) -> tuple[bool, list[str], dict[str, Any]]:
    blockers: list[str] = []
    if report.get("ok") is not True:
        blockers.append("report.ok is not true")
    if report.get("simulation_only") is not True:
        blockers.append("simulation_only is not true")
    if not _bool_false(report, "real_robot_motion"):
        blockers.append("real_robot_motion is not false")
    if not _bool_false(report, "cmd_vel_sent_to_hardware"):
        blockers.append("cmd_vel_sent_to_hardware is not false")
    if int(report.get("samples") or 0) <= 0:
        blockers.append("tf samples missing")
    if report.get("odometry_frame_id") != "odom":
        blockers.append("odometry_frame_id is not odom")
    if report.get("odometry_child_frame_id") != "body":
        blockers.append("odometry_child_frame_id is not body")

    frames = report.get("topic_frames") or {}
    samples = report.get("topic_samples") or {}
    point_counts = report.get("point_counts") or {}
    expected = {
        "/nav/map_cloud": "odom",
        "/nav/registered_cloud": "body",
    }
    for topic, frame in expected.items():
        if int(samples.get(topic) or 0) <= 0:
            blockers.append(f"{topic} samples missing")
        if frames.get(topic) != frame:
            blockers.append(f"{topic} frame is not {frame}")
    for topic in ("/nav/map_cloud", "/nav/registered_cloud"):
        if int(point_counts.get(topic) or 0) <= 0:
            blockers.append(f"{topic} points missing")

    return not blockers, blockers, {
        "samples": report.get("samples"),
        "topic_samples": samples,
        "topic_frames": frames,
        "point_counts": point_counts,
    }


def _eval_saved_map_relocalize(report: dict[str, Any]) -> tuple[bool, list[str], dict[str, Any]]:
    blockers: list[str] = []
    if report.get("ok") is not True:
        blockers.append("report.ok is not true")
    if report.get("simulation_only") is not True:
        blockers.append("simulation_only is not true")
    if not _bool_false(report, "real_robot_motion"):
        blockers.append("real_robot_motion is not false")
    if not _bool_false(report, "cmd_vel_sent_to_hardware"):
        blockers.append("cmd_vel_sent_to_hardware is not false")

    contracts = report.get("contracts") or {}
    localizer = contracts.get("localizer") or {}
    if localizer.get("health_source") != "localizer_health_topic":
        blockers.append("localizer health_source is not localizer_health_topic")
    if localizer.get("map_save_source") != "active_map":
        blockers.append("localizer map_save_source is not active_map")
    if localizer.get("recovery_method") != "relocalize_service":
        blockers.append("localizer recovery_method is not relocalize_service")
    if localizer.get("recovery_action") != "relocalize_service":
        blockers.append("localizer recovery_action is not relocalize_service")
    if localizer.get("saved_map_relocalization_supported") is not True:
        blockers.append("localizer saved_map_relocalization_supported is not true")
    for backend in ("fastlio2", "super_lio", "super_lio_relocation"):
        if (contracts.get(backend) or {}).get("saved_map_relocalization_supported") is not False:
            blockers.append(f"{backend} saved_map_relocalization_supported is not false")

    defaults = report.get("default_profiles") or {}
    if defaults.get("navigating") != "localizer":
        blockers.append("navigating default profile is not localizer")
    plans = report.get("plans") or {}
    nav_plan = plans.get("session_navigating_fastlio2") or {}
    if tuple(nav_plan.get("ensure") or ()) != ("slam", "localizer"):
        blockers.append("navigating plan does not ensure slam/localizer")
    switch_plan = plans.get("switch_localizer") or {}
    if tuple(switch_plan.get("ensure") or ()) != ("slam", "localizer"):
        blockers.append("localizer switch plan does not ensure slam/localizer")

    launch_services = report.get("launch_services") or {}
    for service in ("/nav/relocalize", "/nav/relocalize_check", "/nav/global_relocalize", "/nav/saved_map_cloud"):
        if launch_services.get(service) is not True:
            blockers.append(f"{service} launch remap missing")

    bridge = report.get("bridge_status") or {}
    localizer_status = bridge.get("localizer") or {}
    super_lio_status = bridge.get("super_lio") or {}
    if localizer_status.get("backend") != "localizer":
        blockers.append("bridge localizer status backend is not localizer")
    if localizer_status.get("localizer_health") != "LOCKED":
        blockers.append("bridge localizer health is not LOCKED")
    if localizer_status.get("relocalization_state") != "idle":
        blockers.append("bridge localizer relocalization_state is not idle")
    if localizer_status.get("saved_map_relocalization_supported") is not True:
        blockers.append("bridge localizer saved_map_relocalization_supported is not true")
    if super_lio_status.get("saved_map_relocalization_supported") is not False:
        blockers.append("bridge super_lio saved_map_relocalization_supported is not false")
    if super_lio_status.get("relocalization_state") != "unsupported":
        blockers.append("bridge super_lio relocalization_state is not unsupported")

    return not blockers, blockers, {
        "default_profiles": defaults,
        "localizer_contract": {
            "health_source": localizer.get("health_source"),
            "map_save_source": localizer.get("map_save_source"),
            "saved_map_relocalization_supported": localizer.get(
                "saved_map_relocalization_supported"
            ),
            "recovery_method": localizer.get("recovery_method"),
        },
        "launch_services": launch_services,
        "bridge_status": bridge,
    }


def _eval_multifloor_exploration(report: dict[str, Any]) -> tuple[bool, list[str], dict[str, Any]]:
    blockers: list[str] = []
    if report.get("passed") is not True:
        blockers.append("report.passed is not true")
    if report.get("simulation_only") is not True:
        blockers.append("simulation_only is not true")
    if not _bool_false(report, "real_robot_motion"):
        blockers.append("real_robot_motion is not false")
    if report.get("cmd_vel_sent_to_hardware") is not False:
        blockers.append("cmd_vel_sent_to_hardware is not false")
    if report.get("suite") != "multifloor_route_matrix":
        blockers.append("suite is not multifloor_route_matrix")
    if report.get("validation_level") != "kinematic_module_ports":
        blockers.append("validation_level is not kinematic_module_ports")
    if report.get("local_planner_backend_verified") not in {"nanobind", "cmu", "cmu_py"}:
        blockers.append("production local planner backend was not verified")
    if report.get("production_local_planner_required") is not True:
        blockers.append("production_local_planner_required is not true")
    if report.get("production_local_planner_verified") is not True:
        blockers.append("production_local_planner_verified is not true")
    if report.get("frontier_loop_enabled") is not True:
        blockers.append("frontier_loop_enabled is not true")

    required_routes = {"same_floor", "lower_approach", "upper_floor", "cross_floor"}
    routes = set(report.get("routes") or [])
    missing_routes = sorted(required_routes - routes)
    if missing_routes:
        blockers.append(f"missing routes: {', '.join(missing_routes)}")

    exploration = report.get("exploration") or {}
    if exploration.get("ok") is not True:
        blockers.append("exploration.ok is not true")
    if exploration.get("closed_loop") is not True:
        blockers.append("exploration.closed_loop is not true")
    if exploration.get("probe_mode") != "frontier_navigation_closed_loop":
        blockers.append("exploration probe is not frontier_navigation_closed_loop")
    rounds = list(exploration.get("rounds") or [])
    if not rounds:
        blockers.append("frontier closed-loop rounds missing")
    for idx, item in enumerate(rounds):
        if not (item.get("goal") or {}).get("frontier"):
            blockers.append(f"frontier round {idx} missing frontier goal")
        if (item.get("command_flow") or {}).get("ok") is not True:
            blockers.append(f"frontier round {idx} command_flow failed")
        if (item.get("tracking_replay") or {}).get("ok") is not True:
            blockers.append(f"frontier round {idx} tracking_replay failed")

    cases = list(report.get("cases") or [])
    if len(cases) < len(required_routes):
        blockers.append("case_count is lower than required route count")
    for case in cases:
        route = str(case.get("route") or "")
        if case.get("passed") is not True:
            blockers.append(f"{route or 'unknown'} case did not pass")
        if (case.get("lidar_localization") or {}).get("ok") is not True:
            blockers.append(f"{route or 'unknown'} lidar_localization failed")
        if (case.get("native_pct_gate") or {}).get("ok") is not True:
            blockers.append(f"{route or 'unknown'} native_pct_gate failed")
        if (case.get("command_flow") or {}).get("ok") is not True:
            blockers.append(f"{route or 'unknown'} command_flow failed")
        if (case.get("tracking_replay") or {}).get("ok") is not True:
            blockers.append(f"{route or 'unknown'} tracking_replay failed")
        if (case.get("command_flow") or {}).get("cmd_vel_sent_to_hardware") is not False:
            blockers.append(f"{route or 'unknown'} command_flow did not report hardware-safe cmd_vel")
        if (case.get("tracking_replay") or {}).get("cmd_vel_sent_to_hardware") is not False:
            blockers.append(f"{route or 'unknown'} tracking_replay did not report hardware-safe cmd_vel")
    if "cross_floor" in routes:
        cross_floor = next((case for case in cases if case.get("route") == "cross_floor"), {})
        gate = cross_floor.get("native_pct_gate") or {}
        if gate.get("floor_graph_composition_verified") is not True:
            blockers.append("cross_floor floor-graph composition not verified")
        if int(gate.get("native_pct_feasible_segments") or 0) < 2:
            blockers.append("cross_floor does not have two feasible native PCT segments")

    return not blockers, blockers, {
        "case_count": len(cases),
        "routes": sorted(routes),
        "local_planner_backend_verified": report.get("local_planner_backend_verified"),
        "production_local_planner_verified": report.get("production_local_planner_verified"),
        "frontier_loop_enabled": report.get("frontier_loop_enabled"),
        "frontier_rounds": len(rounds),
        "native_pct_gate_passed_count": report.get("native_pct_gate_passed_count"),
        "validation_level": report.get("validation_level"),
    }


GATES: tuple[GateSpec, ...] = (
    GateSpec(
        "multifloor_exploration",
        "Multi-floor route matrix with LiDAR localization contract, local planning, tracking, and frontier closed-loop",
        (
            "artifacts/server_sim_closure/multifloor_exploration/report.json",
            "artifacts/multifloor_nav_validation*/report.json",
        ),
        "PYTHONPATH=src:. python3 sim/scripts/multifloor_nav_validation.py "
        "--route matrix --planners pct,astar --skip-mujoco --frontier-loop "
        "--local-planner-backend nanobind --require-production-local-planner "
        "--output-dir artifacts/server_sim_closure/multifloor_exploration "
        "--json-out artifacts/server_sim_closure/multifloor_exploration/report.json --strict",
        _eval_multifloor_exploration,
    ),
    GateSpec(
        "large_terrain",
        "PCT/native path generation over large static terrain assets",
        (
            "artifacts/server_sim_closure/large_terrain/report.json",
            "artifacts/large_terrain_nav_validation*/report.json",
        ),
        "PYTHONPATH=src:. python3 sim/scripts/large_terrain_nav_validation.py --output-dir artifacts/server_sim_closure/large_terrain --planners pct,astar --json-out artifacts/server_sim_closure/large_terrain/report.json",
        _eval_large_terrain,
    ),
    GateSpec(
        "native_pct_mujoco",
        "Native PCT route through ROS2 localPlanner/pathFollower into MuJoCo kinematic motion",
        (
            "artifacts/server_sim_closure/native_pct_mujoco/report.json",
            "artifacts/native_pct_mujoco_gate*/report.json",
            "artifacts/native_pct_large_terrain*/report.json",
            "artifacts/native_pct_effect/report.json",
            "artifacts/native_pct_effect/*pct*overlay*_report.json",
        ),
        "PYTHONPATH=src:. python3 sim/scripts/native_pct_mujoco_gate.py --source-report artifacts/server_sim_closure/large_terrain/report.json --route terrain_long --planner pct --timeout-s 180 --json-out artifacts/server_sim_closure/native_pct_mujoco/report.json",
        _eval_native_pct_mujoco,
    ),
    GateSpec(
        "dynamic_obstacle_local_planner",
        "Nanobind LocalPlannerModule replans around changing added_obstacles without hardware cmd_vel",
        (
            "artifacts/server_sim_closure/dynamic_obstacle_local_planner/report.json",
            "artifacts/dynamic_obstacle_local_planner*/report.json",
        ),
        "PYTHONPATH=src:. python3 sim/scripts/dynamic_obstacle_local_planner_gate.py "
        "--backend nanobind "
        "--json-out artifacts/server_sim_closure/dynamic_obstacle_local_planner/report.json",
        _eval_dynamic_obstacle_local_planner,
    ),
    GateSpec(
        "fastlio2_live",
        "Fast-LIO2 on live MuJoCo LiDAR/IMU into SlamBridgeModule",
        (
            "artifacts/server_sim_closure/fastlio2_live/report.json",
            "artifacts/mujoco_fastlio2_live*/report.json",
            "artifacts/mujoco_fastlio2_live*/*/report.json",
            "artifacts/fastlio2*/report.json",
        ),
        "PYTHONPATH=src:. python3 sim/scripts/mujoco_fastlio2_live_gate.py --json-out artifacts/server_sim_closure/fastlio2_live/report.json --strict",
        _eval_fastlio2_live,
    ),
    GateSpec(
        "policy_nav",
        "ONNX gait policy with full-stack simulated navigation",
        (
            "artifacts/server_sim_closure/policy_nav/report.json",
            "artifacts/policy_nav*/report.json",
            "artifacts/*policy*/*.json",
            "artifacts/policy_direct_verify.json",
        ),
        "PYTHONPATH=src:. MUJOCO_GL=egl python3 sim/scripts/policy_nav_smoke.py "
        "--direct-duration 4 --goal-distance 0.8 --nav-duration 18 "
        "--nav-local-planner-backend nanobind --nav-path-follower-backend nav_core "
        "--nav-costmap-wait 3 --nav-path-min-speed 0.25 --nav-path-max-speed 0.6 "
        "--nav-waypoint-threshold 0.30 --nav-final-waypoint-threshold 0.25 "
        "--nav-path-goal-tolerance 0.25 --max-nav-dist-to-goal 0.35 "
        "--min-nav-motion 0.35 --nav-max-angular-z 0.15 "
        "--json-out artifacts/server_sim_closure/policy_nav/report.json",
        _eval_policy_nav,
    ),
    GateSpec(
        "gateway_dry_run",
        "Gateway goal/preview command flow without driver or hardware cmd_vel",
        ("artifacts/gateway_goal_dry_run*/report.json", "artifacts/server_sim_closure/gateway_dry_run/report.json"),
        "PYTHONPATH=src:. python3 sim/scripts/gateway_goal_dry_run_gate.py --json-out artifacts/server_sim_closure/gateway_dry_run/report.json --strict",
        _eval_gateway_dry_run,
    ),
    GateSpec(
        "routecheck_preflight",
        "Gateway non-motion route preflight comparing baseline and candidate planning",
        (
            "artifacts/server_sim_closure/routecheck/summary.json",
            "artifacts/super_lio_route_preflight_*/summary.json",
            "artifacts/routecheck*/summary.json",
        ),
        "PYTHONPATH=src:. python3 sim/scripts/routecheck_preflight_gate.py "
        "--map server_sim_demo --goal-x 1.0 --goal-y 0.0 --goal-yaw 0.0 "
        "--json-out artifacts/server_sim_closure/routecheck/summary.json --strict",
        _eval_routecheck_preflight,
    ),
    GateSpec(
        "gazebo_runtime",
        "ROS-native Gazebo TF, odometry, and point-cloud runtime smoke",
        ("artifacts/server_sim_closure/gazebo_runtime/report.json",),
        "bash -lc 'source /opt/ros/humble/setup.bash && "
        "source install/setup.bash 2>/dev/null || true; "
        "PYTHONPATH=src:.:$PYTHONPATH python3 sim/scripts/gazebo_runtime_gate.py "
        "--json-out artifacts/server_sim_closure/gazebo_runtime/report.json'",
        _eval_gazebo_runtime,
    ),
    GateSpec(
        "saved_map_relocalize",
        "Saved-map relocalization contract for navigation mode and localizer bridge status",
        (
            "artifacts/server_sim_closure/saved_map_relocalize/report.json",
            "artifacts/saved_map_relocalize*/report.json",
        ),
        "PYTHONPATH=src:. python3 sim/scripts/saved_map_relocalize_contract_gate.py "
        "--json-out artifacts/server_sim_closure/saved_map_relocalize/report.json --strict",
        _eval_saved_map_relocalize,
    ),
)


def _candidate_matches(patterns: tuple[str, ...]) -> list[Path]:
    seen: set[Path] = set()
    candidates: list[Path] = []
    for pattern in patterns:
        for path in ROOT.glob(pattern):
            if path.is_file() and path not in seen:
                seen.add(path)
                candidates.append(path)
    return sorted(candidates, key=lambda path: path.stat().st_mtime, reverse=True)


def _best_match(spec: GateSpec) -> Path | None:
    candidates = _candidate_matches(spec.default_patterns)
    if not candidates:
        return None
    for path in candidates:
        try:
            ok, _, _ = spec.evaluator(_load_json(path))
        except Exception:
            ok = False
        if ok:
            return path
    return candidates[0]


def summarize(*, report_overrides: dict[str, Path], required: set[str]) -> dict[str, Any]:
    gates: dict[str, Any] = {}
    for spec in GATES:
        path = report_overrides.get(spec.name) or _best_match(spec)
        if path is None:
            gates[spec.name] = {
                "description": spec.description,
                "exists": False,
                "ok": False,
                "status": "missing",
                "blockers": ["report missing"],
                "path": "",
                "command": spec.command,
            }
            continue
        try:
            report = _load_json(path)
            ok, blockers, evidence = spec.evaluator(report)
            gates[spec.name] = {
                "description": spec.description,
                "exists": True,
                "ok": bool(ok),
                "status": "passed" if ok else "failed",
                "blockers": blockers,
                "path": str(path),
                "command": spec.command,
                "evidence": evidence,
            }
        except Exception as exc:
            gates[spec.name] = {
                "description": spec.description,
                "exists": True,
                "ok": False,
                "status": "invalid",
                "blockers": [str(exc)],
                "path": str(path),
                "command": spec.command,
            }

    required_names = required or {spec.name for spec in GATES}
    missing_or_failed = [
        name
        for name in sorted(required_names)
        if not bool((gates.get(name) or {}).get("ok"))
    ]
    verified = {name: bool(item.get("ok")) for name, item in gates.items()}
    return {
        "schema_version": "lingtu.server_sim_closure.v1",
        "ok": not missing_or_failed,
        "simulation_only": True,
        "real_robot_motion": False,
        "cmd_vel_sent_to_hardware": False,
        "generated_at": time.time(),
        "required": sorted(required_names),
        "verified": verified,
        "missing_or_failed": missing_or_failed,
        "gates": gates,
        "remaining_gaps": [
            f"{name}: {', '.join((gates.get(name) or {}).get('blockers') or ['not verified'])}"
            for name in missing_or_failed
        ],
    }


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--large-terrain-report", type=Path, default=None)
    parser.add_argument("--native-pct-mujoco-report", type=Path, default=None)
    parser.add_argument("--dynamic-obstacle-local-planner-report", type=Path, default=None)
    parser.add_argument("--fastlio2-live-report", type=Path, default=None)
    parser.add_argument("--policy-nav-report", type=Path, default=None)
    parser.add_argument("--gateway-dry-run-report", type=Path, default=None)
    parser.add_argument("--routecheck-preflight-report", type=Path, default=None)
    parser.add_argument("--gazebo-runtime-report", type=Path, default=None)
    parser.add_argument("--saved-map-relocalize-report", type=Path, default=None)
    parser.add_argument(
        "--required",
        default=",".join(spec.name for spec in GATES),
        help="Comma-separated gate names required for ok=true",
    )
    parser.add_argument("--json-out", type=Path, default=ROOT / "artifacts/server_sim_closure_summary.json")
    parser.add_argument("--strict", action="store_true")
    return parser


def main() -> int:
    args = _build_parser().parse_args()
    overrides = {
        "large_terrain": args.large_terrain_report,
        "native_pct_mujoco": args.native_pct_mujoco_report,
        "dynamic_obstacle_local_planner": args.dynamic_obstacle_local_planner_report,
        "fastlio2_live": args.fastlio2_live_report,
        "policy_nav": args.policy_nav_report,
        "gateway_dry_run": args.gateway_dry_run_report,
        "routecheck_preflight": args.routecheck_preflight_report,
        "gazebo_runtime": args.gazebo_runtime_report,
        "saved_map_relocalize": args.saved_map_relocalize_report,
    }
    required = {item.strip() for item in args.required.split(",") if item.strip()}
    valid_names = {spec.name for spec in GATES}
    unknown = sorted(required - valid_names)
    if unknown:
        raise SystemExit(f"unknown required gate(s): {', '.join(unknown)}")

    summary = summarize(
        report_overrides={key: value for key, value in overrides.items() if value is not None},
        required=required,
    )
    text = json.dumps(summary, ensure_ascii=False, indent=2, sort_keys=True)
    print(text)
    if args.json_out:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(text + "\n", encoding="utf-8")
    return 0 if summary.get("ok") or not args.strict else 1


if __name__ == "__main__":
    raise SystemExit(main())
