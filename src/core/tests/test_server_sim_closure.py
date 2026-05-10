from __future__ import annotations

import json
from pathlib import Path

import pytest

from sim.scripts import server_sim_closure


def _write_json(path: Path, payload: dict) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload), encoding="utf-8")
    return path


def _complete_multifloor_report() -> dict:
    localization = {"ok": True}
    command_flow = {"ok": True, "cmd_vel_sent_to_hardware": False}
    tracking_replay = {"ok": True, "cmd_vel_sent_to_hardware": False}
    native_pct_gate = {
        "ok": True,
        "floor_graph_composition_verified": False,
        "native_pct_feasible_segments": 1,
    }
    cases = []
    for route in ("same_floor", "lower_approach", "upper_floor"):
        cases.append(
            {
                "route": route,
                "passed": True,
                "lidar_localization": localization,
                "native_pct_gate": native_pct_gate,
                "command_flow": command_flow,
                "tracking_replay": tracking_replay,
            }
        )
    cases.append(
        {
            "route": "cross_floor",
            "passed": True,
            "lidar_localization": localization,
            "native_pct_gate": {
                "ok": True,
                "floor_graph_composition_verified": True,
                "native_pct_feasible_segments": 2,
            },
            "command_flow": command_flow,
            "tracking_replay": tracking_replay,
        }
    )
    return {
        "passed": True,
        "suite": "multifloor_route_matrix",
        "validation_level": "kinematic_module_ports",
        "simulation_only": True,
        "real_robot_motion": False,
        "cmd_vel_sent_to_hardware": False,
        "local_planner_backend_verified": "nanobind",
        "production_local_planner_required": True,
        "production_local_planner_verified": True,
        "frontier_loop_enabled": True,
        "routes": ["same_floor", "lower_approach", "upper_floor", "cross_floor"],
        "native_pct_gate_passed_count": 4,
        "exploration": {
            "ok": True,
            "closed_loop": True,
            "probe_mode": "frontier_navigation_closed_loop",
            "rounds": [
                {
                    "goal": {"frontier": [1.0, 0.0, 0.0]},
                    "command_flow": command_flow,
                    "tracking_replay": tracking_replay,
                }
            ],
        },
        "cases": cases,
    }


def test_gateway_goal_dry_run_gate_publishes_goal_without_cmd_vel():
    pytest.importorskip("fastapi")
    from sim.scripts.gateway_goal_dry_run_gate import run_gate

    report = run_gate(x=1.2, y=-0.4, z=0.0, client_id="pytest")

    assert report["ok"] is True
    assert report["simulation_only"] is True
    assert report["real_robot_motion"] is False
    assert report["cmd_vel_sent_to_hardware"] is False
    assert report["driver_used"] is False
    assert report["frames"]["published_goal"] == "map"
    assert report["frames"]["cmd_vel"] == "not_published"
    assert report["published"]["goal_pose"] == 1
    assert report["published"]["cmd_vel"] == 0


def test_server_sim_closure_finds_default_large_terrain_report_path():
    spec = next(item for item in server_sim_closure.GATES if item.name == "large_terrain")

    assert "artifacts/server_sim_closure/large_terrain/report.json" in spec.default_patterns


def test_server_sim_closure_policy_nav_command_uses_verified_policy_gait_params():
    spec = next(item for item in server_sim_closure.GATES if item.name == "policy_nav")

    assert "--nav-duration 18" in spec.command
    assert "--nav-path-min-speed 0.25" in spec.command
    assert "--nav-path-max-speed 0.6" in spec.command
    assert "--nav-max-angular-z 0.15" in spec.command


def test_server_sim_closure_gazebo_runtime_command_starts_runtime_gate():
    spec = next(item for item in server_sim_closure.GATES if item.name == "gazebo_runtime")

    assert "sim/scripts/gazebo_runtime_gate.py" in spec.command
    assert "tf_contract_smoke.py" not in spec.command
    assert "source /opt/ros/humble/setup.bash" in spec.command


def test_server_sim_closure_dynamic_obstacle_command_uses_nanobind_module_gate():
    spec = next(item for item in server_sim_closure.GATES if item.name == "dynamic_obstacle_local_planner")

    assert "sim/scripts/dynamic_obstacle_local_planner_gate.py" in spec.command
    assert "--backend nanobind" in spec.command
    assert "server_sim_closure/dynamic_obstacle_local_planner/report.json" in spec.command


def test_server_sim_closure_accepts_complete_report_set(tmp_path: Path):
    multifloor = _write_json(tmp_path / "multifloor.json", _complete_multifloor_report())
    large = _write_json(
        tmp_path / "large.json",
        {
            "ok": True,
            "simulation_only": True,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "cases": [
                {
                    "route": "terrain_long",
                    "path_safety": {"ok": True},
                    "planning": [
                        {
                            "planner": "pct",
                            "native_backend_used": True,
                        }
                    ],
                }
            ],
        },
    )
    native = _write_json(
        tmp_path / "native.json",
        {
            "ok": True,
            "simulation_only": True,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "reached_goal": True,
            "final_distance_m": 0.4,
            "planner": "pct",
            "pct_native_backend_used": True,
            "moved_m": 10.0,
            "frames": {"goal": "map", "cmd_vel": "base_link"},
            "obstacle_aware": {
                "enabled": True,
                "metadata_points": 32,
            },
            "obstacle_clearance": {
                "checked": True,
                "collision": False,
                "min_clearance_m": 0.65,
            },
            "local_path_samples": [{"frame_id": "body", "point_count": 8}],
            "trajectory_quality": {
                "ok": True,
                "p95_lateral_error_m": 0.12,
                "final_progress_ratio": 0.99,
            },
        },
    )
    dynamic_obstacles = _write_json(
        tmp_path / "dynamic_obstacles.json",
        {
            "ok": True,
            "simulation_only": True,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "backend_actual": "nanobind",
            "dynamic_replan_verified": True,
            "obstacle_response_verified": True,
            "clear_path_recovery_verified": True,
            "min_clearance_m": 0.42,
            "frames": {"local_path": "map", "cmd_vel": "not_published"},
            "phases": [
                {"name": "clear_initial", "path_count": 101, "path_frame_id": "map", "avoidance_side": "straight"},
                {"name": "obstacle_left", "path_count": 101, "path_frame_id": "map", "avoidance_side": "right"},
                {"name": "obstacle_right", "path_count": 101, "path_frame_id": "map", "avoidance_side": "left"},
                {"name": "obstacle_center", "path_count": 101, "path_frame_id": "map", "avoidance_side": "right"},
                {"name": "clear_recovered", "path_count": 101, "path_frame_id": "map", "avoidance_side": "straight"},
            ],
        },
    )
    fastlio = _write_json(
        tmp_path / "fastlio.json",
        {
            "ok": True,
            "simulation_only": True,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "live_mujoco_lidar_verified": True,
            "live_mujoco_imu_verified": True,
            "slam_algorithm_output_verified": True,
            "bridge_verified": True,
            "outputs": {"fastlio2_odometry": 12},
            "frames": {"published_lidar": "body"},
        },
    )
    policy = _write_json(
        tmp_path / "policy.json",
        {
            "passed": True,
            "simulation_only": True,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "checks": [
                {
                    "mode": "direct_policy",
                    "passed": True,
                    "policy_loaded": True,
                    "policy_path": "/tmp/policy.onnx",
                    "cmd_vel_sent_to_hardware": False,
                },
                {
                    "mode": "full_stack_policy_nav",
                    "passed": True,
                    "policy_loaded": True,
                    "policy_path": "/tmp/policy.onnx",
                    "cmd_vel_sent_to_hardware": False,
                    "seen": {
                        "direct_fallback": 0,
                        "local_path": 4,
                        "path_follower_cmd": 4,
                        "mux_cmd": 4,
                        "waypoints": 1,
                    },
                    "frames": {"goal": "map", "cmd_vel": "base_link"},
                },
            ],
        },
    )
    gateway = _write_json(
        tmp_path / "gateway.json",
        {
            "ok": True,
            "simulation_only": True,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "published": {"goal_pose": 1, "cmd_vel": 0},
            "frames": {"published_goal": "map", "cmd_vel": "not_published"},
        },
    )
    gazebo = _write_json(
        tmp_path / "gazebo.json",
        {
            "ok": True,
            "simulation_only": True,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "samples": 4,
            "odometry_frame_id": "odom",
            "odometry_child_frame_id": "body",
            "topic_samples": {
                "/nav/map_cloud": 3,
                "/nav/registered_cloud": 3,
                "/camera/color/image_raw": 3,
                "/camera/depth/image_raw": 3,
                "/camera/color/camera_info": 3,
            },
            "topic_frames": {
                "/nav/map_cloud": "odom",
                "/nav/registered_cloud": "body",
                "/camera/color/image_raw": "camera_link",
                "/camera/depth/image_raw": "camera_link",
                "/camera/color/camera_info": "camera_link",
            },
            "point_counts": {
                "/nav/map_cloud": 1024,
                "/nav/registered_cloud": 1024,
            },
        },
    )

    summary = server_sim_closure.summarize(
        report_overrides={
            "multifloor_exploration": multifloor,
            "large_terrain": large,
            "native_pct_mujoco": native,
            "dynamic_obstacle_local_planner": dynamic_obstacles,
            "fastlio2_live": fastlio,
            "policy_nav": policy,
            "gateway_dry_run": gateway,
            "gazebo_runtime": gazebo,
        },
        required={
            "multifloor_exploration",
            "large_terrain",
            "native_pct_mujoco",
            "dynamic_obstacle_local_planner",
            "fastlio2_live",
            "policy_nav",
            "gateway_dry_run",
            "gazebo_runtime",
        },
    )

    assert summary["ok"] is True
    assert summary["simulation_only"] is True
    assert summary["real_robot_motion"] is False
    assert summary["cmd_vel_sent_to_hardware"] is False
    assert summary["missing_or_failed"] == []
    assert all(summary["verified"].values())


def test_server_sim_closure_rejects_weak_multifloor_exploration_report(tmp_path: Path):
    weak = _complete_multifloor_report()
    weak["frontier_loop_enabled"] = False
    weak["exploration"] = {
        "ok": True,
        "closed_loop": False,
        "probe_mode": "candidate_only",
        "rounds": [],
    }
    weak["cases"][-1]["native_pct_gate"] = {
        "ok": True,
        "floor_graph_composition_verified": False,
        "native_pct_feasible_segments": 1,
    }
    report_path = _write_json(tmp_path / "weak_multifloor.json", weak)

    summary = server_sim_closure.summarize(
        report_overrides={"multifloor_exploration": report_path},
        required={"multifloor_exploration"},
    )

    assert summary["ok"] is False
    assert summary["verified"]["multifloor_exploration"] is False
    gaps = "\n".join(summary["remaining_gaps"])
    assert "frontier_loop_enabled is not true" in gaps
    assert "exploration.closed_loop is not true" in gaps
    assert "exploration probe is not frontier_navigation_closed_loop" in gaps
    assert "cross_floor floor-graph composition not verified" in gaps


def test_server_sim_closure_reports_remaining_gaps(tmp_path: Path):
    native = _write_json(
        tmp_path / "native.json",
        {
            "ok": True,
            "simulation_only": True,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "reached_goal": True,
            "final_distance_m": 1.2,
            "frames": {"goal": "map", "cmd_vel": "base_link"},
            "obstacle_aware": {"enabled": True, "metadata_points": 24},
            "obstacle_clearance": {"checked": True, "collision": True},
            "local_path_samples": [{"frame_id": "body", "point_count": 4}],
            "trajectory_quality": {"ok": True},
        },
    )

    summary = server_sim_closure.summarize(
        report_overrides={
            "native_pct_mujoco": native,
            "fastlio2_live": tmp_path / "missing_fastlio.json",
        },
        required={"native_pct_mujoco", "fastlio2_live"},
    )

    assert summary["ok"] is False
    assert summary["verified"]["native_pct_mujoco"] is False
    assert "native_pct_mujoco" in summary["missing_or_failed"]
    assert "fastlio2_live" in summary["missing_or_failed"]
    assert any("collision is true" in gap for gap in summary["remaining_gaps"])
    assert any("missing_fastlio.json" in gap for gap in summary["remaining_gaps"])


def test_server_sim_closure_rejects_weak_dynamic_obstacle_report(tmp_path: Path):
    weak = _write_json(
        tmp_path / "dynamic_weak.json",
        {
            "ok": True,
            "simulation_only": True,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "backend_actual": "cmu_py",
            "dynamic_replan_verified": False,
            "obstacle_response_verified": True,
            "clear_path_recovery_verified": False,
            "min_clearance_m": 0.10,
            "phases": [
                {"name": "clear_initial", "path_count": 101, "path_frame_id": "map", "avoidance_side": "straight"},
                {"name": "obstacle_left", "path_count": 101, "path_frame_id": "map", "avoidance_side": "straight"},
                {"name": "obstacle_right", "path_count": 101, "path_frame_id": "map", "avoidance_side": "straight"},
            ],
        },
    )

    summary = server_sim_closure.summarize(
        report_overrides={"dynamic_obstacle_local_planner": weak},
        required={"dynamic_obstacle_local_planner"},
    )

    assert summary["ok"] is False
    assert summary["verified"]["dynamic_obstacle_local_planner"] is False
    gaps = "\n".join(summary["remaining_gaps"])
    assert "backend_actual is not nanobind" in gaps
    assert "dynamic_replan_verified is not true" in gaps
    assert "clear_path_recovery_verified is not true" in gaps
    assert "min_clearance_m < 0.25" in gaps
    assert "left obstacle did not produce right detour" in gaps


def test_server_sim_closure_rejects_non_pct_native_motion_report(tmp_path: Path):
    native = _write_json(
        tmp_path / "native_astar.json",
        {
            "ok": True,
            "simulation_only": True,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "reached_goal": True,
            "final_distance_m": 0.4,
            "planner": "astar",
            "pct_native_backend_used": False,
            "frames": {"goal": "map", "cmd_vel": "base_link"},
            "obstacle_aware": {"enabled": True, "metadata_points": 24},
            "obstacle_clearance": {"checked": True, "collision": False, "min_clearance_m": 0.7},
            "local_path_samples": [{"frame_id": "body", "point_count": 4}],
            "trajectory_quality": {"ok": True},
        },
    )

    summary = server_sim_closure.summarize(
        report_overrides={"native_pct_mujoco": native},
        required={"native_pct_mujoco"},
    )

    assert summary["ok"] is False
    assert summary["verified"]["native_pct_mujoco"] is False
    assert any("planner is not pct" in gap for gap in summary["remaining_gaps"])
    assert any("pct_native_backend_used is not true" in gap for gap in summary["remaining_gaps"])


def test_server_sim_closure_rejects_native_pct_without_obstacle_local_path_evidence(tmp_path: Path):
    native = _write_json(
        tmp_path / "native_weak.json",
        {
            "ok": True,
            "simulation_only": True,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "reached_goal": True,
            "final_distance_m": 0.4,
            "planner": "pct",
            "pct_native_backend_used": True,
            "moved_m": 10.0,
            "frames": {"goal": "map", "cmd_vel": "base_link"},
            "obstacle_aware": {"enabled": False, "metadata_points": 0},
            "obstacle_clearance": {"collision": False, "min_clearance_m": 0.7},
            "local_path_samples": [],
            "trajectory_quality": {"ok": False},
        },
    )

    summary = server_sim_closure.summarize(
        report_overrides={"native_pct_mujoco": native},
        required={"native_pct_mujoco"},
    )

    assert summary["ok"] is False
    assert summary["verified"]["native_pct_mujoco"] is False
    gaps = "\n".join(summary["remaining_gaps"])
    assert "obstacle_aware.enabled is not true" in gaps
    assert "obstacle_aware.metadata_points missing" in gaps
    assert "obstacle clearance was not checked" in gaps
    assert "local_path_samples missing" in gaps
    assert "trajectory_quality.ok is not true" in gaps


def test_server_sim_closure_finds_nested_fastlio2_live_report(tmp_path: Path, monkeypatch):
    nested = _write_json(
        tmp_path / "artifacts" / "mujoco_fastlio2_live" / "factory_scene" / "report.json",
        {
            "ok": True,
            "simulation_only": True,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "live_mujoco_lidar_verified": True,
            "live_mujoco_imu_verified": True,
            "slam_algorithm_output_verified": True,
            "bridge_verified": True,
            "outputs": {"fastlio2_odometry": 8},
            "states_seen": ["TRACKING"],
            "point_count": {"cloud_map": 100},
            "frames": {"published_lidar": "body"},
        },
    )
    monkeypatch.setattr(server_sim_closure, "ROOT", tmp_path)

    summary = server_sim_closure.summarize(report_overrides={}, required={"fastlio2_live"})

    assert summary["ok"] is True
    assert summary["gates"]["fastlio2_live"]["path"] == str(nested)


def test_server_sim_closure_ignores_native_pct_showcase_reports(tmp_path: Path, monkeypatch):
    _write_json(
        tmp_path / "artifacts" / "native_pct_effect" / "report.json",
        {
            "schema_version": "lingtu.native_pct_mujoco_gate.v1",
            "ok": True,
            "simulation_only": True,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "reached_goal": True,
            "final_distance_m": 0.2,
            "planner": "pct",
            "pct_native_backend_used": True,
            "frames": {"goal": "map", "cmd_vel": "base_link"},
            "obstacle_aware": {"enabled": True, "metadata_points": 32},
            "obstacle_clearance": {"checked": True, "collision": False},
            "local_path_samples": [{"frame_id": "body", "point_count": 8}],
            "trajectory_quality": {"ok": True},
        },
    )
    monkeypatch.setattr(server_sim_closure, "ROOT", tmp_path)

    summary = server_sim_closure.summarize(report_overrides={}, required={"native_pct_mujoco"})

    assert summary["ok"] is False
    assert summary["gates"]["native_pct_mujoco"]["exists"] is False
    assert "native_pct_mujoco" in summary["missing_or_failed"]


def test_server_sim_closure_rejects_policy_nav_direct_fallback(tmp_path: Path):
    policy = _write_json(
        tmp_path / "policy.json",
        {
            "passed": True,
            "simulation_only": True,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "checks": [
                {
                    "mode": "direct_policy",
                    "passed": True,
                    "policy_loaded": True,
                    "policy_path": "/tmp/policy.onnx",
                    "cmd_vel_sent_to_hardware": False,
                },
                {
                    "mode": "full_stack_policy_nav",
                    "passed": True,
                    "policy_loaded": True,
                    "policy_path": "/tmp/policy.onnx",
                    "cmd_vel_sent_to_hardware": False,
                    "seen": {
                        "direct_fallback": 1,
                        "local_path": 12,
                        "path_follower_cmd": 12,
                        "mux_cmd": 12,
                        "waypoints": 2,
                    },
                },
            ],
        },
    )

    summary = server_sim_closure.summarize(
        report_overrides={"policy_nav": policy},
        required={"policy_nav"},
    )

    assert summary["ok"] is False
    assert summary["verified"]["policy_nav"] is False
    assert any("direct_goal_fallback" in gap for gap in summary["remaining_gaps"])
