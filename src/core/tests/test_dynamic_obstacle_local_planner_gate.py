from __future__ import annotations

import pytest

from sim.scripts.dynamic_obstacle_local_planner_gate import run_gate


def test_dynamic_obstacle_local_planner_gate_replans_without_hardware_cmd_vel():
    pytest.importorskip("_nav_core")

    report = run_gate(backend="nanobind")

    assert report["ok"] is True
    assert report["simulation_only"] is True
    assert report["real_robot_motion"] is False
    assert report["cmd_vel_sent_to_hardware"] is False
    assert report["backend_actual"] == "nanobind"
    assert report["dynamic_replan_verified"] is True
    assert report["obstacle_response_verified"] is True
    assert report["clear_path_recovery_verified"] is True
    assert report["min_clearance_m"] >= 0.25

    phases = {item["name"]: item for item in report["phases"]}
    assert phases["clear_initial"]["avoidance_side"] == "straight"
    assert phases["obstacle_left"]["avoidance_side"] == "right"
    assert phases["obstacle_right"]["avoidance_side"] == "left"
    assert phases["clear_recovered"]["avoidance_side"] == "straight"
    assert all(item["path_frame_id"] == "map" for item in report["phases"])
