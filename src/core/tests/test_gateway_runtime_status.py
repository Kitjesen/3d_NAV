from __future__ import annotations

import asyncio
import json

import pytest


pytest.importorskip("fastapi")


def _endpoint(gateway, path: str):
    gateway.setup()
    return next(route.endpoint for route in gateway._app.routes if route.path == path)


def _payload(response_or_payload):
    if hasattr(response_or_payload, "body"):
        return json.loads(response_or_payload.body)
    return response_or_payload


def test_localization_status_covers_product_states():
    from gateway.gateway_module import GatewayModule
    from gateway.services.runtime_status import build_localization_status

    gateway = GatewayModule()

    payload = build_localization_status(gateway)
    assert payload["state"] == "no_odometry"
    assert payload["has_odometry"] is False
    assert payload["can_relocalize"] is False

    with gateway._state_lock:
        gateway._odom = {"x": 1.0}
        gateway._localization_status = {"state": "TRACKING", "confidence": 0.9}
    payload = build_localization_status(gateway)
    assert payload["state"] == "tracking"
    assert payload["reported_state"] == "TRACKING"

    gateway._icp_quality = 0.03
    with gateway._state_lock:
        gateway._localization_status = {
            "state": "TRACKING",
            "confidence": 0.9,
            "degeneracy": "NONE",
            "localizer_health": "RECOVERED",
        }
    payload = build_localization_status(gateway)
    assert payload["state"] == "ready"
    assert payload["ready"] is True
    assert payload["algorithm_healthy"] is True
    assert payload["pose_fresh"] is True

    gateway._icp_quality = 0.0
    with gateway._state_lock:
        gateway._localization_status = {
            "state": "TRACKING",
            "confidence": 0.89,
            "degeneracy": "NONE",
            "icp_fitness": 0.0,
            "localizer_health": "RECOVERED",
            "localizer_health_source": "localizer_health_topic",
            "localizer_health_fitness": 0.0246,
            "odom_age_ms": 222.4,
        }
    payload = build_localization_status(gateway)
    assert payload["state"] == "ready"
    assert payload["ready"] is True
    assert payload["algorithm_healthy"] is True

    gateway._icp_quality = 0.0
    with gateway._state_lock:
        gateway._localization_status = {
            "state": "TRACKING",
            "confidence": 0.7,
            "degeneracy": "MILD",
            "health_source": "odom_map_cloud",
            "pose_fresh": True,
            "map_cloud_fresh": True,
            "icp_fitness": 0.0,
            "odom_age_ms": 150.0,
            "cloud_age_ms": 120.0,
            "localizer_health": "LIO_TRACKING",
        }
    payload = build_localization_status(gateway)
    assert payload["state"] == "ready"
    assert payload["ready"] is True
    assert payload["algorithm_healthy"] is True
    assert payload["degeneracy"] == "MILD"
    assert payload["reasons"] == []

    gateway._session_mode = "navigating"
    gateway._icp_quality = 0.2
    with gateway._state_lock:
        gateway._localization_status = {"state": "TRACKING", "confidence": 0.9}
    payload = build_localization_status(gateway)
    assert payload["state"] == "ready"
    assert payload["ready"] is True

    with gateway._state_lock:
        gateway._localization_status = {
            "state": "DEGRADED",
            "confidence": 0.4,
            "degeneracy": "MILD",
        }
    payload = build_localization_status(gateway)
    assert payload["state"] == "degraded"
    assert payload["can_relocalize"] is True
    assert "low_confidence" in payload["reasons"]

    with gateway._state_lock:
        gateway._localization_status = {
            "state": "TRACKING",
            "confidence": 0.28,
            "degeneracy": "NONE",
            "icp_fitness": 0.028,
            "odom_age_ms": 1440.0,
            "cloud_age_ms": 120.0,
            "localizer_health": "RECOVERED",
        }
    payload = build_localization_status(gateway)
    assert payload["state"] == "ready"
    assert payload["algorithm_healthy"] is True
    assert payload["pose_fresh"] is True
    assert payload["pose_freshness"] == "fresh"
    assert payload["stale_odometry"] is False
    assert payload["odom_age_ms"] == 1440.0
    assert payload["reasons"] == []

    with gateway._state_lock:
        gateway._localization_status = {
            "state": "TRACKING",
            "confidence": 0.28,
            "degeneracy": "NONE",
            "icp_fitness": 0.028,
            "odom_age_ms": 2500.0,
            "cloud_age_ms": 120.0,
            "localizer_health": "RECOVERED",
        }
    payload = build_localization_status(gateway)
    assert payload["state"] == "degraded"
    assert payload["algorithm_healthy"] is True
    assert payload["pose_fresh"] is False
    assert payload["pose_freshness"] == "stale"
    assert payload["stale_odometry"] is True
    assert payload["odom_age_ms"] == 2500.0
    assert payload["reasons"] == ["reported_state:tracking", "stale_odometry"]

    with gateway._state_lock:
        gateway._localization_status = {"state": "LOST", "confidence": 0.0}
    payload = build_localization_status(gateway)
    assert payload["state"] == "lost"
    assert payload["can_relocalize"] is True

    with gateway._state_lock:
        gateway._localization_status = {
            "state": "TRACKING",
            "confidence": 0.9,
            "localizer_health": "LOST",
        }
    payload = build_localization_status(gateway)
    assert payload["state"] == "lost"
    assert "localizer_health:lost" in payload["reasons"]


def test_localization_status_route_returns_stable_schema():
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    with gateway._state_lock:
        gateway._odom = {"x": 0.0}
        gateway._localization_status = {"state": "TRACKING", "confidence": 0.8}

    payload = asyncio.run(_endpoint(gateway, "/api/v1/localization/status")())

    assert payload["schema_version"] == 1
    assert payload["state"] == "tracking"
    assert payload["has_odometry"] is True
    assert payload["reported_state"] == "TRACKING"


def test_localization_status_exposes_gateway_diagnostic_age():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import LocalizationStatusResponse
    from gateway.services.runtime_status import build_localization_status

    gateway = GatewayModule()
    with gateway._state_lock:
        gateway._odom = {"x": 0.0}
    gateway._on_localization_status({"state": "TRACKING", "confidence": 0.9})

    payload = build_localization_status(gateway)
    model = LocalizationStatusResponse.model_validate(payload)

    assert model.diag_received_ts is not None
    assert model.diag_age_ms is not None
    assert model.diag_age_ms >= 0.0
    assert "_gateway_received_mono" in payload["raw"]


def test_localization_status_exposes_slam_quality_diagnostics():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import LocalizationStatusResponse
    from gateway.services.runtime_status import build_localization_status

    gateway = GatewayModule()
    with gateway._state_lock:
        gateway._odom = {"x": 0.0}
        gateway._localization_status = {
            "state": "DEGRADED",
            "confidence": 0.1,
            "degeneracy": "CRITICAL",
            "icp_fitness": 0.3049,
            "effective_ratio": 1.0,
            "condition_number": 50.4,
            "degenerate_dof_count": 0,
            "pos_cov_trace": 0.000017,
            "ieskf_iter_num": 10,
            "ieskf_converged": False,
            "localizer_health": "DEGRADED",
            "localizer_health_fitness": 0.3049,
            "localizer_health_iter": 11,
            "localizer_health_cov_trace": 0.000019,
        }

    payload = build_localization_status(gateway)
    model = LocalizationStatusResponse.model_validate(payload)

    assert model.state == "degraded"
    assert model.icp_fitness == 0.3049
    assert model.effective_ratio == 1.0
    assert model.condition_number == 50.4
    assert model.degenerate_dof_count == 0
    assert model.pos_cov_trace == 0.000017
    assert model.ieskf_iter_num == 10
    assert model.ieskf_converged is False
    assert model.localizer_health == "DEGRADED"
    assert model.localizer_health_fitness == 0.3049
    assert model.localizer_health_iter == 11
    assert model.localizer_health_cov_trace == 0.000019
    assert payload["raw"]["icp_fitness"] == 0.3049


def test_localization_status_accepts_super_lio_odom_map_health():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import LocalizationStatusResponse
    from gateway.services.runtime_status import build_localization_status

    gateway = GatewayModule()
    gateway._session_mode = "navigating"
    gateway._icp_quality = 0.03
    with gateway._state_lock:
        gateway._odom = {"x": 0.0}
        gateway._localization_status = {
            "backend": "super_lio",
            "state": "TRACKING",
            "confidence": 0.92,
            "health_source": "odom_map_cloud",
            "pose_fresh": True,
            "map_cloud_fresh": True,
            "map_state": "live_map_cloud",
            "map_save_supported": True,
            "map_save_source": "live_map_cloud_snapshot",
            "relocalization_supported": False,
            "saved_map_relocalization_supported": False,
            "restart_recovery_supported": True,
            "recovery_method": "restart_super_lio",
            "relocalization_state": "unsupported",
            "recovery_signal": "NONE",
            "recovery_action": "none",
            "localizer_health": "LIO_TRACKING",
            "odom_age_ms": 120.0,
            "cloud_age_ms": 80.0,
        }

    payload = build_localization_status(gateway)
    model = LocalizationStatusResponse.model_validate(payload)

    assert model.state == "ready"
    assert model.ready is True
    assert model.algorithm_healthy is True
    assert model.backend == "super_lio"
    assert model.health_source == "odom_map_cloud"
    assert model.map_cloud_fresh is True
    assert model.map_save_supported is True
    assert model.map_save_source == "live_map_cloud_snapshot"
    assert model.relocalization_supported is False
    assert model.saved_map_relocalization_supported is False
    assert model.restart_recovery_supported is True
    assert model.recovery_method == "restart_super_lio"
    assert model.can_relocalize is False
    assert model.recovery_signal == "NONE"
    assert model.recovery_action == "none"


def test_localization_status_fills_super_lio_map_save_defaults():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import LocalizationStatusResponse
    from gateway.services.runtime_status import build_localization_status

    gateway = GatewayModule()
    gateway._session_mode = "navigating"
    with gateway._state_lock:
        gateway._odom = {"x": 0.0}
        gateway._localization_status = {
            "backend": "super_lio",
            "state": "TRACKING",
            "confidence": 0.92,
            "health_source": "odom_map_cloud",
            "pose_fresh": True,
            "map_cloud_fresh": True,
            "localizer_health": "LIO_TRACKING",
            "odom_age_ms": 120.0,
            "cloud_age_ms": 80.0,
        }

    payload = build_localization_status(gateway)
    model = LocalizationStatusResponse.model_validate(payload)

    assert model.ready is True
    assert model.map_save_supported is True
    assert model.map_save_source == "live_map_cloud_snapshot"
    assert model.relocalization_supported is False
    assert model.saved_map_relocalization_supported is False
    assert model.restart_recovery_supported is True
    assert model.recovery_method == "restart_super_lio"


def test_localization_status_accepts_super_lio_relocation_contract():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import LocalizationStatusResponse
    from gateway.services.runtime_status import build_localization_status

    gateway = GatewayModule()
    gateway._session_mode = "navigating"
    with gateway._state_lock:
        gateway._odom = {"x": 0.0}
        gateway._localization_status = {
            "backend": "super_lio_relocation",
            "state": "TRACKING",
            "confidence": 0.88,
            "health_source": "odom_map_cloud",
            "pose_fresh": True,
            "map_cloud_fresh": True,
            "map_state": "relocation_map_cloud",
            "map_save_supported": False,
            "map_save_source": "active_map",
            "relocalization_supported": False,
            "saved_map_relocalization_supported": False,
            "restart_recovery_supported": True,
            "recovery_method": "restart_super_lio_relocation",
            "relocalization_state": "unsupported",
            "recovery_signal": "NONE",
            "recovery_action": "none",
            "localizer_health": "LIO_TRACKING",
            "odom_age_ms": 120.0,
            "cloud_age_ms": 80.0,
        }

    payload = build_localization_status(gateway)
    model = LocalizationStatusResponse.model_validate(payload)

    assert model.state == "ready"
    assert model.ready is True
    assert model.backend == "super_lio_relocation"
    assert model.health_source == "odom_map_cloud"
    assert model.map_cloud_fresh is True
    assert model.map_save_supported is False
    assert model.map_save_source == "active_map"
    assert model.relocalization_supported is False
    assert model.saved_map_relocalization_supported is False
    assert model.restart_recovery_supported is True
    assert model.recovery_method == "restart_super_lio_relocation"
    assert model.can_relocalize is False


def test_super_lio_relocation_alias_keeps_conservative_capabilities():
    from gateway.services.runtime_status import backend_capability_defaults

    defaults = backend_capability_defaults("relocation")

    assert defaults["relocalization_supported"] is False
    assert defaults["saved_map_relocalization_supported"] is False
    assert defaults["restart_recovery_supported"] is True
    assert defaults["recovery_method"] == "restart_super_lio_relocation"


def test_super_lio_degraded_state_does_not_offer_relocalize():
    from gateway.gateway_module import GatewayModule
    from gateway.services.runtime_status import build_localization_status

    gateway = GatewayModule()
    with gateway._state_lock:
        gateway._odom = {"x": 0.0}
        gateway._localization_status = {
            "backend": "super_lio",
            "state": "DEGRADED",
            "confidence": 0.2,
            "health_source": "odom_map_cloud",
            "pose_fresh": True,
            "relocalization_supported": False,
            "localizer_health": "LIO_DEGRADED",
        }

    payload = build_localization_status(gateway)

    assert payload["state"] == "degraded"
    assert payload["relocalization_supported"] is False
    assert payload["saved_map_relocalization_supported"] is False
    assert payload["restart_recovery_supported"] is True
    assert payload["recovery_method"] == "restart_super_lio"
    assert payload["can_relocalize"] is False


def test_session_snapshot_exposes_super_lio_backend_capabilities():
    import time

    from gateway.gateway_module import GatewayModule
    from gateway.schemas import SessionResponse

    gateway = GatewayModule()
    gateway._cached_slam_profile = "super_lio"
    gateway._slam_profile_ts = time.time()
    gateway._session_mode = "mapping"
    with gateway._state_lock:
        gateway._localization_status = {
            "backend": "super_lio",
            "state": "TRACKING",
            "health_source": "odom_map_cloud",
            "pose_fresh": True,
            "map_state": "live_map_cloud",
            "map_save_supported": True,
            "map_save_source": "live_map_cloud_snapshot",
            "relocalization_supported": False,
            "saved_map_relocalization_supported": False,
            "restart_recovery_supported": True,
            "recovery_method": "restart_super_lio",
            "relocalization_state": "unsupported",
            "recovery_signal": "NONE",
            "recovery_action": "restart_super_lio",
            "localizer_health": "LIO_TRACKING",
        }

    session = gateway._session_snapshot()
    model = SessionResponse.model_validate(session)

    assert model.slam_profile == "super_lio"
    assert model.localization_backend == "super_lio"
    assert model.health_source == "odom_map_cloud"
    assert model.localizer_ready is True
    assert model.map_save_supported is True
    assert model.map_save_source == "live_map_cloud_snapshot"
    assert model.relocalization_supported is False
    assert model.saved_map_relocalization_supported is False
    assert model.restart_recovery_supported is True
    assert model.recovery_method == "restart_super_lio"
    assert model.relocalization_state == "unsupported"
    assert model.recovery_action == "restart_super_lio"


def test_session_snapshot_exposes_super_lio_relocation_capabilities():
    import time

    from gateway.gateway_module import GatewayModule
    from gateway.schemas import SessionResponse

    gateway = GatewayModule()
    gateway._cached_slam_profile = "super_lio_relocation"
    gateway._slam_profile_ts = time.time()
    gateway._session_mode = "navigating"
    gateway._session_map = "demo"
    with gateway._state_lock:
        gateway._localization_status = {
            "backend": "super_lio_relocation",
            "state": "TRACKING",
            "health_source": "odom_map_cloud",
            "pose_fresh": True,
            "map_state": "relocation_map_cloud",
            "map_save_supported": False,
            "map_save_source": "active_map",
            "relocalization_supported": False,
            "saved_map_relocalization_supported": False,
            "restart_recovery_supported": True,
            "recovery_method": "restart_super_lio_relocation",
            "relocalization_state": "unsupported",
            "recovery_signal": "NONE",
            "recovery_action": "restart_super_lio_relocation",
            "localizer_health": "LIO_TRACKING",
        }

    session = gateway._session_snapshot()
    model = SessionResponse.model_validate(session)

    assert model.slam_profile == "super_lio_relocation"
    assert model.localization_backend == "super_lio_relocation"
    assert model.health_source == "odom_map_cloud"
    assert model.localizer_ready is True
    assert model.map_save_supported is False
    assert model.map_save_source == "active_map"
    assert model.relocalization_supported is False
    assert model.saved_map_relocalization_supported is False
    assert model.restart_recovery_supported is True
    assert model.recovery_method == "restart_super_lio_relocation"
    assert model.relocalization_state == "unsupported"
    assert model.recovery_action == "restart_super_lio_relocation"


def test_navigation_status_reports_mission_path_and_control_source():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import NavigationStatusResponse
    from gateway.services.runtime_status import build_navigation_status

    class FakeMux:
        def health(self):
            return {
                "active_source": "path_follower",
                "sources": {
                    "path_follower": {"active": True, "priority": 40},
                },
            }

    class FakeNavigation:
        def get_navigation_status(self):
            return json.dumps({
                "plan_safety_policy": "reject",
                "last_plan_report": {
                    "selected_planner": "pct",
                    "selected_path_safety": {
                        "ok": False,
                        "blocked_sample_count": 2,
                    },
                    "fallback_reason": "pct path_safety failed",
                    "rejected_plans": [{"planner": "pct", "reason": "unsafe"}],
                    "policy": "reject",
                },
            })

    gateway = GatewayModule()
    gateway._session_mode = "navigating"
    with gateway._state_lock:
        gateway._odom = {"x": 1.0, "y": 2.0, "vx": 0.3, "vy": 0.4}
        gateway._mode = "autonomous"
        gateway._mission = {
            "state": "EXECUTING",
            "planning_frame_id": "map",
            "odom_frame_id": "map",
            "costmap_frame_id": "map",
            "goal_frame_id": "map",
            "wp_index": 2,
            "wp_total": 5,
            "remaining_waypoints": 3,
            "goal": [4.0, 6.0, 0.0],
            "current_waypoint": [2.0, 3.0, 0.0],
            "distance_to_goal_m": 5.0,
            "active_waypoint_distance_m": 1.25,
            "replan_count": 1,
            "speed_scale": 0.5,
            "speed_policy": {
                "scale": 0.5,
                "mode": "cautious",
                "reason": "degeneracy=MILD",
                "source": "localization_degeneracy",
                "applied": True,
            },
            "failure_reason": "",
            "ts": 123.0,
        }
        gateway._last_path = [{"x": 0.0}, {"x": 1.0}, {"x": 2.0}]
        gateway._localization_status = {
            "state": "TRACKING",
            "confidence": 0.9,
            "icp_fitness": 0.03,
        }
    gateway._all_modules = {
        "CmdVelMux": FakeMux(),
        "NavigationModule": FakeNavigation(),
    }

    payload = build_navigation_status(gateway)
    NavigationStatusResponse.model_validate(payload)

    assert payload["state"] == "EXECUTING"
    assert payload["can_accept_goal"] is True
    assert payload["wp_index"] == 2
    assert payload["wp_total"] == 5
    assert payload["replan_count"] == 1
    assert payload["speed_scale"] == 0.5
    assert payload["path"]["points"] == 3
    assert payload["path"]["endpoint"] == "/api/v1/path"
    assert payload["frames"]["planning_frame_id"] == "map"
    assert payload["frames"]["odom_frame_id"] == "map"
    assert payload["frames"]["costmap_frame_id"] == "map"
    assert payload["frames"]["goal_frame_id"] == "map"
    assert payload["frames"]["ok"] is True
    assert payload["frames"]["mismatches"] == []
    assert payload["control"]["mode"] == "autonomous"
    assert payload["control"]["active_cmd_source"] == "path_follower"
    assert payload["control"]["command_owner"] == "navigation"
    assert payload["control"]["source_category"] == "autonomy"
    assert payload["control"]["manual_override"] is False
    assert payload["control"]["preempting_autonomy"] is False
    assert payload["progress"]["fraction"] == 0.4
    assert payload["progress"]["active"] is True
    assert payload["readiness"]["can_execute_autonomy"] is True
    assert payload["readiness"]["session_mode"] == "navigating"
    assert payload["target"]["goal"]["x"] == 4.0
    assert payload["target"]["current_waypoint"]["y"] == 3.0
    assert payload["target"]["distance_to_goal_m"] == 5.0
    assert payload["target"]["active_waypoint_distance_m"] == 1.414
    assert payload["target"]["remaining_waypoints"] == 3
    assert payload["motion"]["current_speed_mps"] == 0.5
    assert payload["motion"]["speed_policy"]["mode"] == "cautious"
    assert payload["motion"]["speed_policy"]["reason"] == "degeneracy=MILD"
    assert payload["feedback"]["next_action"] == "monitor_progress"
    assert payload["diagnostics"]["plan_safety_policy"] == "reject"
    assert payload["diagnostics"]["last_plan_report"]["selected_planner"] == "pct"
    assert (
        payload["diagnostics"]["last_plan_report"]["selected_path_safety"]["ok"]
        is False
    )
    assert payload["reason_codes"] == []
    assert payload["localization"]["degraded"] is False


def test_navigation_status_uses_mission_plan_report_when_module_status_unavailable():
    from gateway.gateway_module import GatewayModule
    from gateway.services.runtime_status import build_navigation_status

    class FakeMux:
        def health(self):
            return {
                "active_source": "path_follower",
                "sources": {
                    "path_follower": {"active": True, "priority": 40},
                },
            }

    gateway = GatewayModule()
    gateway._session_mode = "navigating"
    with gateway._state_lock:
        gateway._odom = {"x": 1.0, "y": 2.0}
        gateway._mode = "autonomous"
        gateway._mission = {
            "state": "PLANNING",
            "planning_frame_id": "map",
            "odom_frame_id": "map",
            "costmap_frame_id": "map",
            "goal_frame_id": "map",
            "plan_safety_policy": "fallback_astar",
            "last_plan_report": {
                "primary_planner": "pct",
                "selected_planner": "astar",
                "fallback_reason": "pct path_safety failed",
                "rejected_plans": [{"planner": "pct", "reason": "unsafe"}],
            },
        }
    gateway._all_modules = {"CmdVelMux": FakeMux()}

    payload = build_navigation_status(gateway)

    assert payload["diagnostics"]["plan_safety_policy"] == "fallback_astar"
    assert payload["diagnostics"]["last_plan_report"]["primary_planner"] == "pct"
    assert payload["diagnostics"]["last_plan_report"]["selected_planner"] == "astar"


def test_navigation_status_blocks_goal_on_odometry_frame_mismatch():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import NavigationStatusResponse
    from gateway.services.runtime_status import build_navigation_status

    class FakeMux:
        def health(self):
            return {"active_source": "none", "sources": {}}

    gateway = GatewayModule()
    gateway._session_mode = "navigating"
    gateway._icp_quality = 0.03
    with gateway._state_lock:
        gateway._odom = {"x": 1.0, "y": 2.0, "frame_id": "odom"}
        gateway._mode = "autonomous"
        gateway._mission = {
            "state": "IDLE",
            "planning_frame_id": "map",
            "costmap_frame_id": "map",
        }
        gateway._localization_status = {"state": "TRACKING", "confidence": 0.9}
    gateway._all_modules = {"CmdVelMux": FakeMux()}

    payload = build_navigation_status(gateway)
    NavigationStatusResponse.model_validate(payload)

    assert payload["frames"]["ok"] is False
    assert payload["frames"]["mismatches"] == [
        {
            "source": "odometry",
            "expected_frame": "map",
            "received_frame": "odom",
        }
    ]
    assert "frame_mismatch_odometry" in payload["reason_codes"]
    assert "frame_mismatch_odometry" in payload["readiness"]["blockers"]
    assert payload["diagnostics"]["frame_mismatches"] == payload["frames"]["mismatches"]
    assert payload["can_accept_goal"] is False
    assert payload["readiness"]["can_execute_autonomy"] is False


def test_gateway_odometry_preserves_frame_for_navigation_status():
    from core.msgs.geometry import Pose, Quaternion, Vector3
    from core.msgs.nav import Odometry
    from gateway.gateway_module import GatewayModule
    from gateway.services.runtime_status import build_navigation_status

    gateway = GatewayModule()
    gateway._session_mode = "navigating"
    gateway._icp_quality = 0.03
    with gateway._state_lock:
        gateway._mode = "autonomous"
        gateway._mission = {"state": "IDLE", "planning_frame_id": "map"}
        gateway._localization_status = {"state": "TRACKING", "confidence": 0.9}

    gateway._on_odometry(Odometry(
        pose=Pose(position=Vector3(1.0, 2.0, 0.0), orientation=Quaternion()),
        frame_id="odom",
        child_frame_id="base_link",
    ))
    payload = build_navigation_status(gateway)

    assert gateway._odom["frame_id"] == "odom"
    assert gateway._odom["child_frame_id"] == "base_link"
    assert payload["frames"]["odom_frame_id"] == "odom"
    assert "frame_mismatch_odometry" in payload["reason_codes"]


def test_gateway_mission_event_pushes_navigation_status_update():
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway._session_mode = "navigating"
    gateway._icp_quality = 0.03
    with gateway._state_lock:
        gateway._odom = {"x": 0.0, "y": 0.0, "frame_id": "map"}
        gateway._mode = "autonomous"
        gateway._localization_status = {"state": "TRACKING", "confidence": 0.9}
    queue = gateway._sse_subscribe()

    try:
        gateway._on_mission({
            "state": "EXECUTING",
            "planning_frame_id": "map",
            "odom_frame_id": "map",
            "costmap_frame_id": "map",
        })
        events = []
        while not queue.empty():
            events.append(queue.get_nowait())
    finally:
        gateway._sse_unsubscribe(queue)

    assert [event["type"] for event in events] == ["mission", "navigation_status"]
    assert events[1]["data"]["state"] == "EXECUTING"
    assert events[1]["data"]["frames"]["ok"] is True


def test_navigation_status_reports_costmap_frame_mismatch():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import NavigationStatusResponse
    from gateway.services.runtime_status import build_navigation_status

    class FakeMux:
        def health(self):
            return {"active_source": "none", "sources": {}}

    gateway = GatewayModule()
    gateway._session_mode = "navigating"
    gateway._icp_quality = 0.03
    with gateway._state_lock:
        gateway._odom = {"x": 1.0, "y": 2.0, "frame_id": "map"}
        gateway._mode = "autonomous"
        gateway._mission = {
            "state": "IDLE",
            "planning_frame_id": "map",
            "odom_frame_id": "map",
            "costmap_frame_id": "odom",
        }
        gateway._localization_status = {"state": "TRACKING", "confidence": 0.9}
    gateway._all_modules = {"CmdVelMux": FakeMux()}

    payload = build_navigation_status(gateway)
    NavigationStatusResponse.model_validate(payload)

    assert payload["frames"]["ok"] is False
    assert payload["frames"]["mismatches"] == [
        {
            "source": "costmap",
            "expected_frame": "map",
            "received_frame": "odom",
        }
    ]
    assert "frame_mismatch_costmap" in payload["reason_codes"]
    assert "frame_mismatch_costmap" in payload["readiness"]["blockers"]
    assert payload["can_accept_goal"] is False


def test_navigation_status_reads_idle_costmap_frame_from_navigation_module():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import NavigationStatusResponse
    from gateway.services.runtime_status import build_navigation_status

    class FakeMux:
        def health(self):
            return {"active_source": "none", "sources": {}}

    class FakeNavigation:
        def get_navigation_status(self):
            return json.dumps({
                "state": "IDLE",
                "planning_frame_id": "map",
                "odom_frame_id": "map",
                "costmap_frame_id": "odom",
            })

    gateway = GatewayModule()
    gateway._session_mode = "navigating"
    gateway._icp_quality = 0.03
    with gateway._state_lock:
        gateway._odom = {"x": 1.0, "y": 2.0, "frame_id": "map"}
        gateway._mode = "autonomous"
        gateway._mission = {"state": "IDLE", "planning_frame_id": "map"}
        gateway._localization_status = {"state": "TRACKING", "confidence": 0.9}
    gateway._all_modules = {
        "CmdVelMux": FakeMux(),
        "NavigationModule": FakeNavigation(),
    }

    payload = build_navigation_status(gateway)
    NavigationStatusResponse.model_validate(payload)

    assert payload["frames"]["costmap_frame_id"] == "odom"
    assert payload["frames"]["ok"] is False
    assert payload["frames"]["mismatches"] == [
        {
            "source": "costmap",
            "expected_frame": "map",
            "received_frame": "odom",
        }
    ]
    assert "frame_mismatch_costmap" in payload["reason_codes"]
    assert "frame_mismatch_costmap" in payload["readiness"]["blockers"]
    assert payload["can_accept_goal"] is False


def test_navigation_status_blocks_goal_when_session_is_not_navigating():
    from gateway.gateway_module import GatewayModule
    from gateway.services.runtime_status import build_navigation_status

    class FakeMux:
        def health(self):
            return {"active_source": "none", "sources": {}}

    gateway = GatewayModule()
    gateway._session_mode = "idle"
    gateway._icp_quality = 0.03
    with gateway._state_lock:
        gateway._odom = {"x": 1.0, "y": 2.0}
        gateway._mode = "autonomous"
        gateway._mission = {"state": "IDLE"}
        gateway._localization_status = {"state": "TRACKING", "confidence": 0.9}
    gateway._all_modules = {"CmdVelMux": FakeMux()}

    payload = build_navigation_status(gateway)

    assert payload["can_accept_goal"] is False
    assert "navigation_session_inactive" in payload["reason_codes"]
    assert "navigation_session_inactive" in payload["readiness"]["blockers"]
    assert payload["readiness"]["session_mode"] == "idle"
    assert payload["feedback"]["next_action"] == "resolve_blockers"


def test_navigation_status_allows_exploring_session_for_external_tare():
    from gateway.gateway_module import GatewayModule
    from gateway.services.runtime_status import build_navigation_status

    class FakeMux:
        def health(self):
            return {"active_source": "path_follower", "sources": {}}

    gateway = GatewayModule()
    gateway._session_mode = "exploring"
    gateway._session_slam_profile = "none"
    gateway._icp_quality = 0.03
    with gateway._state_lock:
        gateway._odom = {"x": 1.0, "y": 2.0}
        gateway._mode = "autonomous"
        gateway._mission = {"state": "IDLE"}
        gateway._localization_status = {
            "state": "TRACKING",
            "confidence": 0.9,
            "pose_fresh": True,
            "odom_age_ms": 100.0,
        }
    gateway._all_modules = {"CmdVelMux": FakeMux()}

    payload = build_navigation_status(gateway)

    assert payload["readiness"]["session_mode"] == "exploring"
    assert "navigation_session_inactive" not in payload["reason_codes"]
    assert "navigation_session_inactive" not in payload["readiness"]["blockers"]


def test_navigation_status_finds_cmd_vel_mux_by_module_class_or_name():
    from gateway.gateway_module import GatewayModule
    from gateway.services.runtime_status import build_navigation_status

    class FakeCmdVelMuxModule:
        def health(self):
            return {
                "active_source": "path_follower",
                "sources": {
                    "path_follower": {"active": True, "priority": 40},
                },
            }

    gateway = GatewayModule()
    with gateway._state_lock:
        gateway._odom = {"x": 1.0, "y": 2.0}
        gateway._mission = {"state": "IDLE"}
        gateway._localization_status = {"state": "TRACKING", "confidence": 0.9}
    gateway._all_modules = {"navigation.CmdVelMuxModule": FakeCmdVelMuxModule()}

    payload = build_navigation_status(gateway)

    assert payload["control"]["cmd_vel_mux"]["available"] is True
    assert payload["control"]["active_cmd_source"] == "path_follower"
    assert payload["control"]["source_category"] == "autonomy"


def test_navigation_status_handles_failed_mission_and_missing_mux():
    from gateway.gateway_module import GatewayModule
    from gateway.services.runtime_status import build_navigation_status

    gateway = GatewayModule()
    with gateway._state_lock:
        gateway._odom = {"x": 0.0}
        gateway._mission = {
            "state": "STUCK",
            "failure_reason": "blocked",
            "speed_scale": 0.25,
        }
        gateway._localization_status = {
            "state": "DEGRADED",
            "confidence": 0.2,
        }

    payload = build_navigation_status(gateway)

    assert payload["state"] == "STUCK"
    assert payload["failure_reason"] == "blocked"
    assert payload["control"]["active_cmd_source"] == "unknown"
    assert payload["control"]["cmd_vel_mux"]["available"] is False
    assert payload["localization"]["degraded"] is True
    assert "mission_stuck" in payload["reason_codes"]
    assert "failure_blocked" in payload["reason_codes"]
    assert "localization_degraded" in payload["reason_codes"]
    assert "cmd_vel_mux_unavailable" in payload["reason_codes"]
    assert payload["diagnostics"]["failure_reason"] == "blocked"


def test_navigation_status_blocks_autonomy_when_pose_is_stale_but_algorithm_healthy():
    from gateway.gateway_module import GatewayModule
    from gateway.services.runtime_status import build_navigation_status

    class FakeMux:
        def health(self):
            return {"active_source": "none", "sources": {}}

    gateway = GatewayModule()
    gateway._session_mode = "navigating"
    gateway._icp_quality = 0.03
    with gateway._state_lock:
        gateway._odom = {"x": 0.0}
        gateway._mission = {"state": "IDLE"}
        gateway._localization_status = {
            "state": "TRACKING",
            "confidence": 0.28,
            "degeneracy": "NONE",
            "icp_fitness": 0.028,
            "odom_age_ms": 2500.0,
            "localizer_health": "RECOVERED",
        }
    gateway._all_modules = {"CmdVelMux": FakeMux()}

    payload = build_navigation_status(gateway)

    assert payload["localization"]["algorithm_healthy"] is True
    assert payload["localization"]["pose_fresh"] is False
    assert "pose_stale" in payload["reason_codes"]
    assert "pose_stale" in payload["readiness"]["blockers"]
    assert payload["can_accept_goal"] is False
    assert payload["readiness"]["can_accept_goal"] is False
    assert payload["readiness"]["can_execute_autonomy"] is False


def test_navigation_status_allows_fresh_pose_with_low_confidence_snapshot():
    from gateway.gateway_module import GatewayModule
    from gateway.services.runtime_status import build_navigation_status

    class FakeMux:
        def health(self):
            return {"active_source": "none", "sources": {}}

    gateway = GatewayModule()
    gateway._session_mode = "navigating"
    gateway._icp_quality = 0.03
    with gateway._state_lock:
        gateway._odom = {"x": 0.0}
        gateway._mission = {"state": "IDLE"}
        gateway._localization_status = {
            "state": "TRACKING",
            "confidence": 0.28,
            "degeneracy": "NONE",
            "icp_fitness": 0.028,
            "odom_age_ms": 1440.0,
            "localizer_health": "RECOVERED",
        }
    gateway._all_modules = {"CmdVelMux": FakeMux()}

    payload = build_navigation_status(gateway)

    assert payload["localization"]["algorithm_healthy"] is True
    assert payload["localization"]["pose_fresh"] is True
    assert payload["localization"]["pose_freshness"] == "fresh"
    assert "pose_stale" not in payload["reason_codes"]
    assert payload["can_accept_goal"] is True
    assert payload["readiness"]["blockers"] == []
    assert payload["readiness"]["can_accept_goal"] is True
    assert payload["readiness"]["can_execute_autonomy"] is True

    session = gateway._session_snapshot()
    assert session["localizer_ready"] is True
    assert session["pose_fresh"] is True
    assert session["pose_freshness"] == "fresh"


def test_navigation_status_treats_mild_degeneracy_as_advisory():
    from gateway.gateway_module import GatewayModule
    from gateway.services.runtime_status import build_navigation_status

    class FakeMux:
        def health(self):
            return {"active_source": "none", "sources": {}}

    gateway = GatewayModule()
    gateway._session_mode = "navigating"
    gateway._icp_quality = 0.0
    with gateway._state_lock:
        gateway._odom = {"x": 0.0}
        gateway._mission = {"state": "IDLE", "speed_scale": 0.7}
        gateway._localization_status = {
            "state": "TRACKING",
            "confidence": 0.7,
            "degeneracy": "MILD",
            "health_source": "odom_map_cloud",
            "pose_fresh": True,
            "map_cloud_fresh": True,
            "icp_fitness": 0.0,
            "odom_age_ms": 150.0,
            "cloud_age_ms": 120.0,
            "localizer_health": "LIO_TRACKING",
        }
    gateway._all_modules = {"CmdVelMux": FakeMux()}

    payload = build_navigation_status(gateway)

    assert payload["can_accept_goal"] is True
    assert payload["localization"]["ready"] is True
    assert payload["localization"]["degraded"] is False
    assert payload["localization"]["degeneracy"] == "MILD"
    assert payload["readiness"]["blockers"] == []
    assert payload["readiness"]["advisories"] == ["localization_mild_degeneracy"]
    assert payload["reason_codes"] == ["localization_mild_degeneracy"]


def test_navigation_status_uses_localizer_health_fitness_when_icp_quality_is_zero():
    from gateway.gateway_module import GatewayModule
    from gateway.services.runtime_status import build_localization_status, build_navigation_status

    class FakeMux:
        def health(self):
            return {"active_source": "none", "sources": {}}

    gateway = GatewayModule()
    gateway._session_mode = "navigating"
    gateway._icp_quality = 0.0
    with gateway._state_lock:
        gateway._odom = {"x": 0.0}
        gateway._mission = {"state": "IDLE"}
        gateway._localization_status = {
            "state": "TRACKING",
            "confidence": 0.92,
            "degeneracy": "NONE",
            "icp_fitness": 0.0,
            "odom_age_ms": 150.0,
            "cloud_age_ms": 140.0,
            "localizer_health": "RECOVERED",
            "localizer_health_source": "localizer_health_topic",
            "localizer_health_fitness": 0.0223,
        }
    gateway._all_modules = {"CmdVelMux": FakeMux()}

    localization = build_localization_status(gateway)
    navigation = build_navigation_status(gateway)
    session = gateway._session_snapshot()

    assert localization["state"] == "ready"
    assert localization["ready"] is True
    assert localization["algorithm_healthy"] is True
    assert localization["reasons"] == []
    assert navigation["can_accept_goal"] is True
    assert navigation["reason_codes"] == []
    assert session["localizer_ready"] is True


def test_localizer_health_topic_recovered_marks_gateway_ready_when_icp_quality_is_zero():
    from gateway.gateway_module import GatewayModule
    from gateway.services.runtime_status import build_localization_status

    gateway = GatewayModule()
    gateway._session_mode = "navigating"
    gateway._icp_quality = 0.0
    with gateway._state_lock:
        gateway._odom = {"x": 0.0}
        gateway._localization_status = {
            "backend": "localizer",
            "state": "TRACKING",
            "confidence": 0.91,
            "degeneracy": "NONE",
            "icp_fitness": 0.0,
            "odom_age_ms": 120.0,
            "cloud_age_ms": 90.0,
            "localizer_health": "RECOVERED",
            "localizer_health_source": "localizer_health_topic",
            "localizer_health_fitness": 0.0215,
            "relocalization_supported": True,
            "saved_map_relocalization_supported": True,
            "restart_recovery_supported": True,
            "recovery_method": "relocalize_service",
            "relocalization_state": "idle",
        }

    payload = build_localization_status(gateway)

    assert payload["state"] == "ready"
    assert payload["ready"] is True
    assert payload["algorithm_healthy"] is True
    assert payload["backend"] == "localizer"
    assert payload["localizer_health"] == "RECOVERED"
    assert payload["localizer_health_source"] == "localizer_health_topic"
    assert payload["localizer_health_fitness"] == 0.0215
    assert payload["relocalization_supported"] is True
    assert payload["saved_map_relocalization_supported"] is True
    assert payload["restart_recovery_supported"] is True
    assert payload["recovery_method"] == "relocalize_service"
    assert payload["relocalization_state"] == "idle"
    assert payload["reasons"] == []


def test_navigation_status_blocks_ready_when_map_cloud_is_stale():
    from gateway.gateway_module import GatewayModule
    from gateway.services.runtime_status import build_localization_status, build_navigation_status

    class FakeMux:
        def health(self):
            return {"active_source": "none", "sources": {}}

    gateway = GatewayModule()
    gateway._session_mode = "navigating"
    gateway._icp_quality = 0.0
    with gateway._state_lock:
        gateway._odom = {"x": 0.0}
        gateway._mission = {"state": "IDLE"}
        gateway._localization_status = {
            "state": "TRACKING",
            "confidence": 0.92,
            "degeneracy": "NONE",
            "icp_fitness": 0.0,
            "odom_age_ms": 150.0,
            "cloud_age_ms": 140.0,
            "map_cloud_fresh": False,
            "localizer_health": "RECOVERED",
            "localizer_health_source": "localizer_health_topic",
            "localizer_health_fitness": 0.0223,
        }
    gateway._all_modules = {"CmdVelMux": FakeMux()}

    localization = build_localization_status(gateway)
    navigation = build_navigation_status(gateway)
    session = gateway._session_snapshot()

    assert localization["state"] == "initializing"
    assert localization["ready"] is False
    assert localization["algorithm_healthy"] is False
    assert localization["reasons"] == ["localizer_not_ready"]
    assert navigation["can_accept_goal"] is False
    assert "localization_initializing" in navigation["reason_codes"]
    assert session["localizer_ready"] is False


def test_navigation_status_blocks_goal_when_super_lio_recovery_signal_is_active():
    from gateway.gateway_module import GatewayModule
    from gateway.services.runtime_status import build_localization_status, build_navigation_status

    class FakeMux:
        def health(self):
            return {"active_source": "none", "sources": {}}

    gateway = GatewayModule()
    gateway._session_mode = "navigating"
    gateway._icp_quality = 0.0
    with gateway._state_lock:
        gateway._odom = {"x": 0.0}
        gateway._mission = {"state": "IDLE"}
        gateway._localization_status = {
            "backend": "super_lio",
            "state": "TRACKING",
            "confidence": 0.92,
            "health_source": "odom_map_cloud",
            "pose_fresh": True,
            "map_cloud_fresh": True,
            "recovery_signal": "LOC_DIVERGED",
            "recovery_action": "restart_super_lio",
            "localizer_health": "LIO_TRACKING",
            "odom_age_ms": 120.0,
            "cloud_age_ms": 80.0,
        }
    gateway._all_modules = {"CmdVelMux": FakeMux()}

    localization = build_localization_status(gateway)
    navigation = build_navigation_status(gateway)

    assert localization["state"] == "degraded"
    assert localization["ready"] is False
    assert localization["algorithm_healthy"] is False
    assert localization["reasons"] == ["recovery_signal:loc_diverged"]
    assert localization["map_save_source"] == "live_map_cloud_snapshot"
    assert navigation["can_accept_goal"] is False
    assert "localization_recovery_active" in navigation["reason_codes"]
    assert "localization_recovery_active" in navigation["readiness"]["blockers"]


def test_goal_route_rejects_stale_localization_without_publishing():
    from gateway.gateway_module import GatewayModule, GoalRequest
    from gateway.schemas import GatewayErrorResponse

    gateway = GatewayModule()
    gateway._session_mode = "navigating"
    gateway._icp_quality = 0.03
    with gateway._state_lock:
        gateway._odom = {"x": 0.0}
        gateway._mission = {"state": "IDLE"}
        gateway._localization_status = {
            "state": "TRACKING",
            "confidence": 0.28,
            "degeneracy": "NONE",
            "icp_fitness": 0.028,
            "odom_age_ms": 2500.0,
            "localizer_health": "RECOVERED",
        }
    sent_goals = []
    gateway.goal_pose._add_callback(sent_goals.append)

    response = asyncio.run(
        _endpoint(gateway, "/api/v1/goal")(
            GoalRequest(
                x=1.0,
                y=2.0,
                request_id="stale-goal",
                client_id="web",
            )
        )
    )
    payload = _payload(response)
    model = GatewayErrorResponse.model_validate(payload)

    assert response.status_code == 409
    assert model.error == "navigation_not_ready"
    assert model.command is not None
    assert model.command.name == "goal"
    assert model.command.accepted is False
    assert model.detail["blockers"] == ["pose_stale"]
    assert sent_goals == []


def test_goal_route_accepts_ready_navigation_goal():
    from gateway.gateway_module import GatewayModule, GoalRequest
    from gateway.schemas import ControlCommandResponse

    class FakeNavigation:
        def __init__(self) -> None:
            self.calls = []

        def preview_plan(self, x, y, z):
            self.calls.append((x, y, z))
            return {"feasible": True}

    gateway = GatewayModule()
    gateway._session_mode = "navigating"
    gateway._icp_quality = 0.03
    with gateway._state_lock:
        gateway._odom = {"x": 0.0}
        gateway._mission = {"state": "IDLE"}
        gateway._localization_status = {
            "state": "TRACKING",
            "confidence": 0.28,
            "degeneracy": "NONE",
            "icp_fitness": 0.028,
            "odom_age_ms": 250.0,
            "localizer_health": "RECOVERED",
        }
    nav = FakeNavigation()
    gateway.on_system_modules({"NavigationModule": nav})
    sent_goals = []
    gateway.goal_pose._add_callback(sent_goals.append)

    payload = asyncio.run(
        _endpoint(gateway, "/api/v1/goal")(
            GoalRequest(
                x=1.0,
                y=2.0,
                request_id="ready-goal",
                client_id="web",
            )
        )
    )
    model = ControlCommandResponse.model_validate(payload)

    assert model.ok is True
    assert model.command.accepted is True
    assert model.goal == [1.0, 2.0, 0.0]
    assert nav.calls == [(1.0, 2.0, 0.0)]
    assert len(sent_goals) == 1


def test_navigation_status_reports_teleop_preemption_for_active_mission():
    from gateway.gateway_module import GatewayModule
    from gateway.services.runtime_status import build_navigation_status

    class FakeMux:
        def health(self):
            return {
                "active_source": "teleop",
                "sources": {
                    "teleop": {
                        "active": True,
                        "priority": 100,
                        "age_ms": 20,
                    },
                    "path_follower": {
                        "active": True,
                        "priority": 40,
                        "age_ms": 25,
                    },
                },
            }

    gateway = GatewayModule()
    with gateway._state_lock:
        gateway._odom = {"x": 1.0}
        gateway._mode = "autonomous"
        gateway._mission = {
            "state": "EXECUTING",
            "wp_index": 1,
            "wp_total": 4,
            "speed_scale": 1.0,
        }
        gateway._localization_status = {"state": "TRACKING", "confidence": 0.9}
    gateway._all_modules = {"CmdVelMux": FakeMux()}

    payload = build_navigation_status(gateway)

    assert payload["control"]["active_cmd_source"] == "teleop"
    assert payload["control"]["command_owner"] == "teleop"
    assert payload["control"]["source_category"] == "manual"
    assert payload["control"]["manual_override"] is True
    assert payload["control"]["preempting_autonomy"] is True
    assert payload["control"]["active_source"]["priority"] == 100
    assert "control_preempted_by_teleop" in payload["reason_codes"]
    assert "control_preempted_by_teleop" in payload["readiness"]["advisories"]


def test_navigation_status_route_returns_stable_schema():
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    with gateway._state_lock:
        gateway._mission = {"state": "IDLE"}

    payload = asyncio.run(_endpoint(gateway, "/api/v1/navigation/status")())

    assert payload["schema_version"] == 1
    assert payload["state"] == "IDLE"
    assert payload["path"]["endpoint"] == "/api/v1/path"
    assert payload["control"]["active_cmd_source"] == "unknown"
    assert "odometry_missing" in payload["reason_codes"]
    assert payload["readiness"]["blockers"] == [
        "odometry_missing",
        "navigation_session_inactive",
    ]


def test_navigation_status_routes_pass_fastapi_response_validation():
    from fastapi.testclient import TestClient

    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway.setup()
    with gateway._state_lock:
        gateway._mission = {"state": "IDLE"}

    client = TestClient(gateway._app)
    for path in ("/api/v1/navigation/status", "/api/v1/navigation"):
        response = client.get(path)

        assert response.status_code == 200
        payload = response.json()
        assert payload["schema_version"] == 1
        assert payload["state"] == "IDLE"
        assert payload["frames"]["planning_frame_id"] == "map"
        assert payload["target"] == {
            "goal": None,
            "current_waypoint": None,
            "distance_to_goal_m": None,
            "active_waypoint_distance_m": None,
            "remaining_waypoints": None,
        }
        assert payload["motion"]["active_cmd_source"] == "unknown"
        assert payload["feedback"]["next_action"] == "resolve_blockers"
        assert payload["feedback"]["blockers"] == [
            "odometry_missing",
            "navigation_session_inactive",
        ]


def test_drift_watchdog_restores_idle_running_localization_services(monkeypatch):
    import core.service_manager as service_manager
    import gateway.gateway_module as gateway_module
    from gateway.gateway_module import GatewayModule

    class FakeServiceManager:
        def __init__(self):
            self.running = {"slam", "localizer"}
            self.calls: list[tuple[str, tuple[str, ...]]] = []

        def is_running(self, service: str) -> bool:
            return service in self.running

        def stop(self, *services: str) -> None:
            self.calls.append(("stop", services))
            self.running.difference_update(services)

        def ensure(self, *services: str) -> None:
            self.calls.append(("ensure", services))
            self.running.update(services)

        def wait_ready(self, *services: str, timeout: float = 15.0) -> bool:
            self.calls.append(("wait_ready", services))
            return True

    fake = FakeServiceManager()
    monkeypatch.setattr(service_manager, "get_service_manager", lambda: fake)
    monkeypatch.setattr(gateway_module.time, "sleep", lambda _: None)

    gateway = GatewayModule()
    gateway._drift_restart_delay_s = 0.0
    gateway._session_mode = "idle"
    with gateway._state_lock:
        gateway._odom = {"x": 999.0}
        gateway._odom_timestamps.append(123.0)

    gateway._drift_restart_do_restart(xy=999.0, y_abs=0.0, v=0.0)

    assert (
        "stop",
        ("slam", "slam_pgo", "localizer", "super_lio", "super_lio_relocation"),
    ) in fake.calls
    assert ("ensure", ("slam", "localizer")) in fake.calls
    assert ("wait_ready", ("slam", "localizer")) in fake.calls
    assert gateway._odom == {}
    assert gateway._odom_timestamps == []


def test_drift_watchdog_restart_noops_after_shutdown(monkeypatch):
    import core.service_manager as service_manager
    from gateway.gateway_module import GatewayModule

    called = False

    def fail_if_called():
        nonlocal called
        called = True
        raise AssertionError("service manager should not be touched during shutdown")

    monkeypatch.setattr(service_manager, "get_service_manager", fail_if_called)

    gateway = GatewayModule()
    gateway._stop_event.set()

    gateway._drift_restart_do_restart(xy=999.0, y_abs=0.0, v=0.0)

    assert called is False


def test_drift_watchdog_restores_idle_running_super_lio_services(monkeypatch):
    import core.service_manager as service_manager
    import gateway.gateway_module as gateway_module
    from gateway.gateway_module import GatewayModule

    class FakeServiceManager:
        def __init__(self):
            self.running = {"super_lio"}
            self.calls: list[tuple[str, tuple[str, ...]]] = []

        def is_running(self, service: str) -> bool:
            return service in self.running

        def stop(self, *services: str) -> None:
            self.calls.append(("stop", services))
            self.running.difference_update(services)

        def ensure(self, *services: str) -> None:
            self.calls.append(("ensure", services))
            self.running.update(services)

        def wait_ready(self, *services: str, timeout: float = 15.0) -> bool:
            self.calls.append(("wait_ready", services))
            return True

    fake = FakeServiceManager()
    monkeypatch.setattr(service_manager, "get_service_manager", lambda: fake)
    monkeypatch.setattr(gateway_module.time, "sleep", lambda _: None)

    gateway = GatewayModule()
    gateway._drift_restart_delay_s = 0.0
    gateway._session_mode = "idle"

    gateway._drift_restart_do_restart(xy=999.0, y_abs=0.0, v=0.0)

    assert (
        "stop",
        ("slam", "slam_pgo", "localizer", "super_lio", "super_lio_relocation"),
    ) in fake.calls
    assert ("ensure", ("lidar", "super_lio")) in fake.calls
    assert ("wait_ready", ("lidar", "super_lio")) in fake.calls


def test_drift_watchdog_restores_idle_running_super_lio_relocation_services(
    monkeypatch,
):
    import core.service_manager as service_manager
    import gateway.gateway_module as gateway_module
    from gateway.gateway_module import GatewayModule

    class FakeServiceManager:
        def __init__(self):
            self.running = {"super_lio_relocation"}
            self.calls: list[tuple[str, tuple[str, ...]]] = []

        def is_running(self, service: str) -> bool:
            return service in self.running

        def stop(self, *services: str) -> None:
            self.calls.append(("stop", services))
            self.running.difference_update(services)

        def ensure(self, *services: str) -> None:
            self.calls.append(("ensure", services))
            self.running.update(services)

        def wait_ready(self, *services: str, timeout: float = 15.0) -> bool:
            self.calls.append(("wait_ready", services))
            return True

    fake = FakeServiceManager()
    monkeypatch.setattr(service_manager, "get_service_manager", lambda: fake)
    monkeypatch.setattr(gateway_module.time, "sleep", lambda _: None)

    gateway = GatewayModule()
    gateway._drift_restart_delay_s = 0.0
    gateway._session_mode = "idle"

    gateway._drift_restart_do_restart(xy=999.0, y_abs=0.0, v=0.0)

    assert (
        "stop",
        ("slam", "slam_pgo", "localizer", "super_lio", "super_lio_relocation"),
    ) in fake.calls
    assert ("ensure", ("lidar", "super_lio_relocation")) in fake.calls
    assert ("wait_ready", ("lidar", "super_lio_relocation")) in fake.calls
