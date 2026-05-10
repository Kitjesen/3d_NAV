from __future__ import annotations

from sim.scripts.saved_map_relocalize_contract_gate import run_gate


def test_saved_map_relocalize_gate_locks_navigation_contract():
    report = run_gate()

    assert report["ok"] is True
    assert report["simulation_only"] is True
    assert report["real_robot_motion"] is False
    assert report["cmd_vel_sent_to_hardware"] is False
    assert report["default_profiles"]["navigating"] == "localizer"
    localizer = report["contracts"]["localizer"]
    assert localizer["health_source"] == "localizer_health_topic"
    assert localizer["map_save_source"] == "active_map"
    assert localizer["saved_map_relocalization_supported"] is True
    assert localizer["recovery_method"] == "relocalize_service"
    for backend in ("fastlio2", "super_lio", "super_lio_relocation"):
        assert report["contracts"][backend]["saved_map_relocalization_supported"] is False
    assert report["plans"]["session_navigating_fastlio2"]["ensure"] == ("slam", "localizer")
    assert report["plans"]["switch_localizer"]["ensure"] == ("slam", "localizer")
    assert all(report["launch_services"].values())
    status = report["bridge_status"]["localizer"]
    assert status["backend"] == "localizer"
    assert status["localizer_health"] == "LOCKED"
    assert status["relocalization_state"] == "idle"
    assert status["saved_map_relocalization_supported"] is True
