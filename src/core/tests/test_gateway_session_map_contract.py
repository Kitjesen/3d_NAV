from __future__ import annotations

import asyncio
import json
import shutil
import struct
import uuid
from pathlib import Path

import pytest


pytest.importorskip("fastapi")


def _endpoint(gateway, path: str):
    return next(route.endpoint for route in gateway._app.routes if route.path == path)


def _payload(response_or_payload):
    if hasattr(response_or_payload, "body"):
        return json.loads(response_or_payload.body)
    return response_or_payload


def _seed_ready_navigation(gateway):
    with gateway._state_lock:
        gateway._odom = {"x": 0.0, "y": 0.0, "z": 0.0}
        gateway._localization_status = {
            "backend": "super_lio",
            "state": "TRACKING",
            "confidence": 0.9,
            "health_source": "odom_map_cloud",
            "pose_fresh": True,
            "odom_age_ms": 100.0,
            "cloud_age_ms": 100.0,
            "map_cloud_fresh": True,
            "localizer_health": "RECOVERED",
            "recovery_signal": "NONE",
        }


class _JsonRequest:
    def __init__(self, payload: dict):
        self._payload = payload

    async def json(self):
        return self._payload


class _FakeMapResponse:
    def __init__(self):
        self._callbacks = []

    def _add_callback(self, callback):
        self._callbacks.append(callback)


class _FakeMapCommand:
    def __init__(self, response: _FakeMapResponse):
        self.response = response
        self.delivered = []

    def _deliver(self, raw: str):
        command = json.loads(raw)
        self.delivered.append(command)
        for callback in list(self.response._callbacks):
            callback(
                {
                    "action": command["action"],
                    "success": True,
                    "name": command.get("name"),
                }
            )


class _FakeMapManager:
    def __init__(self):
        self.map_response = _FakeMapResponse()
        self.map_command = _FakeMapCommand(self.map_response)


def _write_binary_xyz_pcd(path: Path) -> None:
    header = (
        b"# .PCD v0.7\n"
        b"VERSION 0.7\n"
        b"FIELDS x y z\n"
        b"SIZE 4 4 4\n"
        b"TYPE F F F\n"
        b"COUNT 1 1 1\n"
        b"WIDTH 2\n"
        b"HEIGHT 1\n"
        b"POINTS 2\n"
        b"DATA binary\n"
    )
    path.write_bytes(header + struct.pack("<ffffff", 1, 2, 3, 4, 5, 6))


def test_auth_routes_validate_response_contracts(monkeypatch):
    from gateway import auth
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import AuthCheckResponse, AuthLoginRequest, AuthLoginResponse

    monkeypatch.setattr(auth, "_get_configured_key", lambda: None)

    gateway = GatewayModule()
    gateway.setup()

    check_payload = asyncio.run(_endpoint(gateway, "/api/v1/auth/check")())
    login_response = asyncio.run(
        _endpoint(gateway, "/api/v1/auth/login")(AuthLoginRequest(key=""))
    )

    check = AuthCheckResponse.model_validate(check_payload)
    login = AuthLoginResponse.model_validate(_payload(login_response))

    assert check.auth_required is False
    assert login.ok is True
    assert login.message == "\u8ba4\u8bc1\u672a\u542f\u7528"


def test_auth_login_invalid_key_preserves_legacy_message(monkeypatch):
    from gateway import auth
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import AuthLoginRequest, AuthLoginResponse

    monkeypatch.setattr(auth, "_get_configured_key", lambda: "secret")

    gateway = GatewayModule()
    gateway.setup()

    login_response = asyncio.run(
        _endpoint(gateway, "/api/v1/auth/login")(AuthLoginRequest(key="bad"))
    )
    login = AuthLoginResponse.model_validate(_payload(login_response))

    assert login_response.status_code == 403
    assert login.ok is False
    assert login.message == "Key \u65e0\u6548"


def test_lease_route_validates_success_and_conflict_payloads():
    from gateway.gateway_module import GatewayModule, LeaseRequest
    from gateway.schemas import GatewayErrorResponse, LeaseResponse

    gateway = GatewayModule()
    gateway.setup()
    post_lease = _endpoint(gateway, "/api/v1/lease")

    acquired_payload = asyncio.run(
        post_lease(LeaseRequest(action="acquire", client_id="web", ttl=30.0))
    )
    conflict_response = asyncio.run(
        post_lease(LeaseRequest(action="acquire", client_id="phone", ttl=30.0))
    )
    released_payload = asyncio.run(
        post_lease(LeaseRequest(action="release", client_id="web", ttl=30.0))
    )

    acquired = LeaseResponse.model_validate(acquired_payload)
    conflict_payload = _payload(conflict_response)
    conflict = GatewayErrorResponse.model_validate(conflict_payload)
    released = LeaseResponse.model_validate(released_payload)

    assert acquired.schema_version == 1
    assert acquired.ok is True
    assert acquired.status == "acquired"
    assert acquired.holder == "web"
    assert acquired.active is True
    assert acquired.command.name == "lease"
    assert conflict.schema_version == 1
    assert conflict.ok is False
    assert conflict.error == "lease_conflict"
    assert conflict.command is not None
    assert conflict.command.name == "lease"
    assert conflict.command.accepted is False
    assert released.status == "released"
    assert released.active is False


def test_session_routes_validate_idle_contracts():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import SessionResponse, SessionTransitionResponse

    gateway = GatewayModule()
    gateway.setup()

    session_payload = asyncio.run(_endpoint(gateway, "/api/v1/session")())
    end_payload = asyncio.run(_endpoint(gateway, "/api/v1/session/end")())

    session = SessionResponse.model_validate(session_payload)
    ended = SessionTransitionResponse.model_validate(end_payload)

    assert session.mode == "idle"
    assert session.explorer_available is False
    assert session.explorer_unavailable_reason == "explorer_backend_not_running"
    assert session.explorer_required_profile == "explore_or_tare_explore"
    assert ended.schema_version == 1
    assert ended.ok is True
    assert ended.success is True
    assert ended.ts > 0
    assert ended.session is not None
    assert ended.session.mode == "idle"


def test_session_start_rejects_invalid_mode_with_stable_contract():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import SessionTransitionResponse

    gateway = GatewayModule()
    gateway.setup()

    response = asyncio.run(
        _endpoint(gateway, "/api/v1/session/start")({"mode": "bad"})
    )

    rejected = SessionTransitionResponse.model_validate(_payload(response))

    assert response.status_code == 400
    assert rejected.schema_version == 1
    assert rejected.ok is False
    assert rejected.success is False
    assert rejected.session is None
    assert rejected.ts > 0
    assert "Unknown mode" in (rejected.message or "")


def test_session_start_accepts_legacy_map_field(monkeypatch):
    import core.service_manager as service_manager
    import gateway.gateway_module as gateway_module
    import gateway.routes.session as session_routes
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import SessionTransitionResponse

    class FakeServiceManager:
        def __init__(self):
            self.calls: list[tuple[str, tuple[str, ...]]] = []

        def stop(self, *services: str) -> None:
            self.calls.append(("stop", services))

        def ensure(self, *services: str) -> None:
            self.calls.append(("ensure", services))

        def wait_ready(self, *services: str, timeout: float = 15.0) -> bool:
            self.calls.append(("wait_ready", services))
            return True

    fake_service_manager = FakeServiceManager()
    monkeypatch.setattr(
        service_manager, "get_service_manager", lambda: fake_service_manager
    )
    monkeypatch.setattr(session_routes.os, "symlink", lambda src, dst: None)

    root = Path.cwd() / ".tmp_gateway_tests" / uuid.uuid4().hex
    try:
        monkeypatch.setenv("HOME", str(root))
        monkeypatch.setenv("USERPROFILE", str(root))
        map_root = root / "custom_maps"
        monkeypatch.setenv("NAV_MAP_DIR", str(map_root))
        map_dir = map_root / "demo"
        map_dir.mkdir(parents=True)
        (map_dir / "map.pcd").write_bytes(b"pcd")
        (map_dir / "tomogram.pickle").write_bytes(b"tomogram")

        gateway = GatewayModule()
        gateway.setup()
        monkeypatch.setattr(gateway_module, "active_map_name", lambda: "demo")
        monkeypatch.setattr(gateway, "_spawn_auto_relocalize", lambda _: None)

        payload = asyncio.run(
            _endpoint(gateway, "/api/v1/session/start")(
                {"mode": "navigating", "map": "demo"}
            )
        )

        transition = SessionTransitionResponse.model_validate(payload)
        assert transition.schema_version == 1
        assert transition.ok is True
        assert transition.success is True
        assert transition.ts > 0
        assert transition.session is not None
        assert transition.session.mode == "navigating"
        assert transition.session.active_map == "demo"
        assert transition.session.map_has_pcd is True
        assert transition.session.map_has_tomogram is True
        assert gateway._session_map == "demo"
        assert ("ensure", ("slam", "localizer")) in fake_service_manager.calls
    finally:
        shutil.rmtree(root, ignore_errors=True)


def test_session_start_can_select_super_lio_backend(monkeypatch):
    import core.service_manager as service_manager
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import SessionTransitionResponse

    class FakeServiceManager:
        def __init__(self):
            self.calls: list[tuple[str, tuple[str, ...]]] = []
            self.services = {
                "lidar": "running",
                "slam": "stopped",
                "slam_pgo": "stopped",
                "localizer": "stopped",
                "super_lio": "running",
                "super_lio_relocation": "stopped",
            }

        def stop(self, *services: str) -> None:
            self.calls.append(("stop", services))

        def ensure(self, *services: str) -> None:
            self.calls.append(("ensure", services))

        def wait_ready(self, *services: str, timeout: float = 15.0) -> bool:
            self.calls.append(("wait_ready", services))
            return True

        def status(self, *names):
            return {name: self.services.get(name, "stopped") for name in names}

    fake_service_manager = FakeServiceManager()
    monkeypatch.setattr(
        service_manager, "get_service_manager", lambda: fake_service_manager
    )

    gateway = GatewayModule()
    gateway.setup()

    payload = asyncio.run(
        _endpoint(gateway, "/api/v1/session/start")(
            {"mode": "mapping", "slam_profile": "super_lio"}
        )
    )

    transition = SessionTransitionResponse.model_validate(payload)
    assert transition.schema_version == 1
    assert transition.ok is True
    assert transition.success is True
    assert transition.ts > 0
    assert transition.session is not None
    assert transition.session.mode == "mapping"
    assert transition.session.slam_profile == "super_lio"
    assert gateway._session_slam_profile == "super_lio"
    assert (
        "stop",
        ("slam", "slam_pgo", "localizer", "super_lio_relocation"),
    ) in fake_service_manager.calls
    assert ("ensure", ("lidar", "super_lio")) in fake_service_manager.calls
    assert ("wait_ready", ("lidar", "super_lio")) in fake_service_manager.calls


def test_session_start_can_select_super_lio_relocation_backend(monkeypatch):
    import core.service_manager as service_manager
    import gateway.routes.session as session_routes
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import SessionTransitionResponse

    class FakeServiceManager:
        def __init__(self):
            self.calls: list[tuple[str, tuple[str, ...]]] = []
            self.services = {
                "lidar": "running",
                "slam": "stopped",
                "slam_pgo": "stopped",
                "localizer": "stopped",
                "super_lio": "stopped",
                "super_lio_relocation": "running",
            }

        def stop(self, *services: str) -> None:
            self.calls.append(("stop", services))

        def ensure(self, *services: str) -> None:
            self.calls.append(("ensure", services))

        def wait_ready(self, *services: str, timeout: float = 15.0) -> bool:
            self.calls.append(("wait_ready", services))
            return True

        def status(self, *names):
            return {name: self.services.get(name, "stopped") for name in names}

    fake_service_manager = FakeServiceManager()
    monkeypatch.setattr(
        service_manager, "get_service_manager", lambda: fake_service_manager
    )
    monkeypatch.setattr(session_routes.os, "symlink", lambda src, dst: None)

    root = Path.cwd() / ".tmp_gateway_tests" / uuid.uuid4().hex
    try:
        monkeypatch.setenv("HOME", str(root))
        monkeypatch.setenv("USERPROFILE", str(root))
        map_root = root / "custom_maps"
        monkeypatch.setenv("NAV_MAP_DIR", str(map_root))
        map_dir = map_root / "demo"
        map_dir.mkdir(parents=True)
        (map_dir / "map.pcd").write_bytes(b"pcd")
        (map_dir / "tomogram.pickle").write_bytes(b"tomogram")

        gateway = GatewayModule()
        gateway.setup()
        auto_relocalize_calls: list[str] = []
        monkeypatch.setattr(
            gateway, "_spawn_auto_relocalize", auto_relocalize_calls.append
        )

        payload = asyncio.run(
            _endpoint(gateway, "/api/v1/session/start")(
                {
                    "mode": "navigating",
                    "map_name": "demo",
                    "slam_profile": "super_lio_relocation",
                }
            )
        )

        transition = SessionTransitionResponse.model_validate(payload)
        assert transition.schema_version == 1
        assert transition.ok is True
        assert transition.success is True
        assert transition.ts > 0
        assert transition.session is not None
        assert transition.session.mode == "navigating"
        assert transition.session.slam_profile == "super_lio_relocation"
        assert gateway._session_map == "demo"
        assert gateway._session_slam_profile == "super_lio_relocation"
        assert ("stop", ("slam", "slam_pgo", "localizer", "super_lio")) in (
            fake_service_manager.calls
        )
        assert ("ensure", ("lidar", "super_lio_relocation")) in (
            fake_service_manager.calls
        )
        assert ("wait_ready", ("lidar", "super_lio_relocation")) in (
            fake_service_manager.calls
        )
        assert auto_relocalize_calls == []
    finally:
        shutil.rmtree(root, ignore_errors=True)


def test_map_routes_validate_json_contracts(monkeypatch):
    from gateway.gateway_module import GatewayModule, MapRequest
    from gateway.schemas import (
        GatewayErrorResponse,
        MapLifecycleResponse,
        MapListResponse,
        MapPointsResponse,
    )

    root = Path.cwd() / ".tmp_gateway_tests" / uuid.uuid4().hex
    try:
        map_dir = root / "maps"
        demo = map_dir / "demo"
        demo.mkdir(parents=True)
        _write_binary_xyz_pcd(demo / "map.pcd")
        (demo / "tomogram.pickle").write_bytes(b"tomogram")
        monkeypatch.setenv("NAV_MAP_DIR", str(map_dir))

        gateway = GatewayModule()
        gateway.setup()

        maps_payload = asyncio.run(_endpoint(gateway, "/api/v1/slam/maps")())
        live_points_payload = asyncio.run(_endpoint(gateway, "/api/v1/map/points")())
        saved_points_payload = asyncio.run(
            _endpoint(gateway, "/api/v1/maps/{name}/points")("demo")
        )
        reset_payload = asyncio.run(_endpoint(gateway, "/api/v1/map_cloud/reset")())
        missing_manager_response = asyncio.run(
            _endpoint(gateway, "/api/v1/maps")(MapRequest(action="list"))
        )

        maps = MapListResponse.model_validate(maps_payload)
        live_points = MapPointsResponse.model_validate(live_points_payload)
        saved_points = MapPointsResponse.model_validate(saved_points_payload)
        reset = MapLifecycleResponse.model_validate(reset_payload)
        missing_manager = GatewayErrorResponse.model_validate(
            _payload(missing_manager_response)
        )

        assert [item.name for item in maps.maps] == ["demo"]
        assert maps.schema_version == 1
        assert maps.count == 1
        assert maps.ts > 0
        assert maps.maps[0].has_pcd is True
        assert maps.maps[0].has_tomogram is True
        assert live_points.schema_version == 1
        assert live_points.count == 0
        assert live_points.layout == "xyz_rows"
        assert live_points.frame_id == "map"
        assert live_points.source == "live_map_cloud"
        assert live_points.ts > 0
        assert live_points.points == []
        assert saved_points.schema_version == 1
        assert saved_points.count == 2
        assert saved_points.layout == "flat_xyz"
        assert saved_points.frame_id == "map"
        assert saved_points.source == "saved_map_pcd"
        assert saved_points.name == "demo"
        assert saved_points.ts > 0
        assert saved_points.points == [1, 2, 3, 4, 5, 6]
        assert reset.schema_version == 1
        assert reset.ok is True
        assert reset.success is True
        assert reset.ts > 0
        assert missing_manager.error == "MapManagerModule not running"
    finally:
        shutil.rmtree(root, ignore_errors=True)


def test_slam_maps_uses_guarded_active_map_resolution(monkeypatch, tmp_path):
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import MapListResponse

    demo = tmp_path / "demo"
    other = tmp_path / "other"
    demo.mkdir()
    other.mkdir()
    (demo / "map.pcd").write_bytes(b"pcd")
    (other / "map.pcd").write_bytes(b"pcd")
    monkeypatch.setenv("NAV_MAP_DIR", str(tmp_path))

    gateway = GatewayModule()
    gateway.setup()
    active = tmp_path / "active"
    try:
        active.symlink_to(demo, target_is_directory=True)
    except (NotImplementedError, OSError) as exc:
        pytest.skip(f"filesystem symlinks unavailable: {exc}")

    payload = asyncio.run(_endpoint(gateway, "/api/v1/slam/maps")())
    maps = MapListResponse.model_validate(payload)

    assert maps.active == "demo"
    assert {item.name: item.is_active for item in maps.maps} == {
        "demo": True,
        "other": False,
    }

    active.unlink()
    nested = tmp_path / "nested" / "child"
    nested.mkdir(parents=True)
    active.symlink_to(nested, target_is_directory=True)

    payload = asyncio.run(_endpoint(gateway, "/api/v1/slam/maps")())
    maps = MapListResponse.model_validate(payload)

    assert maps.active == ""
    assert all(item.is_active is False for item in maps.maps)

    active.unlink()
    outside = tmp_path.parent / f"{tmp_path.name}-outside"
    try:
        outside.mkdir()
        active.symlink_to(outside, target_is_directory=True)

        payload = asyncio.run(_endpoint(gateway, "/api/v1/slam/maps")())
        maps = MapListResponse.model_validate(payload)

        assert maps.active == ""
        assert all(item.is_active is False for item in maps.maps)
    finally:
        shutil.rmtree(outside, ignore_errors=True)


def test_map_lifecycle_error_responses_use_stable_envelope(monkeypatch, tmp_path):
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import MapLifecycleResponse, MapNameRequest

    monkeypatch.setenv("NAV_MAP_DIR", str(tmp_path))

    gateway = GatewayModule()
    gateway.setup()

    response = asyncio.run(
        _endpoint(gateway, "/api/v1/map/activate")(MapNameRequest(name="../bad"))
    )
    model = MapLifecycleResponse.model_validate(_payload(response))

    assert response.status_code == 400
    assert model.schema_version == 1
    assert model.ok is False
    assert model.success is False
    assert model.ts > 0
    assert model.message


def test_map_save_falls_back_to_super_lio_live_cloud_snapshot(monkeypatch, tmp_path):
    import numpy as np
    import gateway.routes.maps as map_routes
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import MapLifecycleResponse

    class FakeCompleted:
        stdout = ""
        stderr = "service unavailable"

    monkeypatch.setenv("NAV_MAP_DIR", str(tmp_path))
    monkeypatch.setattr(map_routes.subprocess, "run", lambda *a, **k: FakeCompleted())
    monkeypatch.setattr(
        map_routes,
        "apply_dynamic_filter_step1half",
        lambda _save_dir: {"success": False, "skipped": True},
    )

    gateway = GatewayModule()
    gateway.setup()
    monkeypatch.setattr(gateway, "_get_slam_profile", lambda: "super_lio")
    with gateway._map_cloud_lock:
        gateway._map_points = np.array(
            [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]],
            dtype=np.float32,
        )

    payload = asyncio.run(
        _endpoint(gateway, "/api/v1/map/save")({"name": "super_lio_demo"})
    )
    model = MapLifecycleResponse.model_validate(payload)
    pcd_path = tmp_path / "super_lio_demo" / "map.pcd"
    active_link = tmp_path / "active"

    assert model.schema_version == 1
    assert model.ok is True
    assert model.success is True
    assert model.ts > 0
    assert payload["slam_profile"] == "super_lio"
    assert payload["source"] == "live_map_cloud_snapshot"
    assert payload["relocalization_supported"] is False
    assert payload["saved_map_relocalization_supported"] is False
    assert payload["restart_recovery_supported"] is True
    assert payload["recovery_method"] == "restart_super_lio"
    assert model.warnings is not None
    assert any("Super-LIO: saved live map_cloud snapshot" in item for item in model.warnings)
    assert pcd_path.is_file()
    data = pcd_path.read_bytes()
    assert b"FIELDS x y z" in data
    assert b"DATA binary\n" in data
    body = data.split(b"DATA binary\n", 1)[1]
    assert len(body) == 2 * 3 * 4
    assert struct.unpack("<ffffff", body) == (1.0, 2.0, 3.0, 4.0, 5.0, 6.0)
    assert not active_link.exists()


def test_map_save_rejects_super_lio_relocation_profile(monkeypatch, tmp_path):
    import gateway.routes.maps as map_routes
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import MapLifecycleResponse

    calls = []

    def fake_run(*args, **kwargs):
        calls.append((args, kwargs))
        raise AssertionError("relocation map save should fail before ROS calls")

    monkeypatch.setenv("NAV_MAP_DIR", str(tmp_path))
    monkeypatch.setattr(map_routes.subprocess, "run", fake_run)

    gateway = GatewayModule()
    gateway.setup()
    monkeypatch.setattr(gateway, "_get_slam_profile", lambda: "super_lio_relocation")

    response = asyncio.run(
        _endpoint(gateway, "/api/v1/map/save")({"name": "relocation_demo"})
    )
    payload = _payload(response)
    model = MapLifecycleResponse.model_validate(payload)

    assert response.status_code == 409
    assert model.ok is False
    assert model.success is False
    assert model.name == "relocation_demo"
    assert model.slam_profile == "super_lio_relocation"
    assert model.source == "active_map"
    assert model.map_save_source == "active_map"
    assert model.relocalization_supported is False
    assert model.saved_map_relocalization_supported is False
    assert model.restart_recovery_supported is True
    assert model.recovery_method == "restart_super_lio_relocation"
    assert calls == []
    assert not (tmp_path / "relocation_demo").exists()


def test_maps_route_accepts_legacy_and_canonical_actions():
    from gateway.gateway_module import GatewayModule, MapRequest
    from gateway.schemas import MapLifecycleResponse

    gateway = GatewayModule()
    gateway.setup()
    manager = _FakeMapManager()
    gateway._map_mgr = manager
    post_maps = _endpoint(gateway, "/api/v1/maps")

    use_payload = asyncio.run(post_maps(MapRequest(action="use", name="demo")))
    build_payload = asyncio.run(post_maps(MapRequest(action="build", name="demo")))
    canonical_payload = asyncio.run(
        post_maps(MapRequest(action="build_tomogram", name="demo"))
    )

    for payload in (use_payload, build_payload, canonical_payload):
        model = MapLifecycleResponse.model_validate(payload)
        assert model.schema_version == 1
        assert model.ok is True
        assert model.success is True
        assert model.ts > 0
    assert manager.map_command.delivered == [
        {"action": "set_active", "name": "demo"},
        {"action": "build_tomogram", "name": "demo"},
        {"action": "build_tomogram", "name": "demo"},
    ]


def test_operational_routes_validate_idle_json_contracts():
    from gateway.gateway_module import BitrateRequest, GatewayModule
    from gateway.schemas import (
        BagOperationResponse,
        BagStatusResponse,
        ExplorationStatusResponse,
        GatewayErrorResponse,
        SlamOperationResponse,
        SlamStatusResponse,
        TemporalMemoryResponse,
        WebRTCControlResponse,
        WebRTCStatsResponse,
    )

    gateway = GatewayModule()
    gateway.setup()

    class _TemporalStore:
        def query(self, **_kwargs):
            return [{"label": "door", "confidence": 0.9}]

        def query_semantic(self, *_args, **_kwargs):
            return [{"label": "door", "score": 0.98}]

    gateway._temporal_store = _TemporalStore()

    temporal_payload = asyncio.run(_endpoint(gateway, "/api/v1/memory/temporal")())
    semantic_payload = asyncio.run(
        _endpoint(gateway, "/api/v1/memory/temporal/semantic")(
            {"embedding": [0.1, 0.2], "top_k": 1}
        )
    )
    semantic_response = asyncio.run(
        _endpoint(gateway, "/api/v1/memory/temporal/semantic")({})
    )
    explore_status_payload = asyncio.run(_endpoint(gateway, "/api/v1/explore/status")())
    explore_start_response = asyncio.run(_endpoint(gateway, "/api/v1/explore/start")())
    slam_status_payload = asyncio.run(_endpoint(gateway, "/api/v1/slam/status")())
    slam_switch_response = asyncio.run(
        _endpoint(gateway, "/api/v1/slam/switch")({"profile": "bad"})
    )
    bag_status_payload = asyncio.run(_endpoint(gateway, "/api/v1/bag/status")())
    bag_stop_response = asyncio.run(_endpoint(gateway, "/api/v1/bag/stop")())
    webrtc_stats_payload = asyncio.run(_endpoint(gateway, "/api/v1/webrtc/stats")())
    webrtc_bitrate_response = asyncio.run(
        _endpoint(gateway, "/api/v1/webrtc/bitrate")(BitrateRequest(bps=1_000_000))
    )

    temporal = TemporalMemoryResponse.model_validate(temporal_payload)
    semantic_ok = TemporalMemoryResponse.model_validate(semantic_payload)
    semantic = GatewayErrorResponse.model_validate(_payload(semantic_response))
    explore_status = ExplorationStatusResponse.model_validate(explore_status_payload)
    explore_start = GatewayErrorResponse.model_validate(_payload(explore_start_response))
    slam_status = SlamStatusResponse.model_validate(slam_status_payload)
    slam_switch = SlamOperationResponse.model_validate(_payload(slam_switch_response))
    bag_status = BagStatusResponse.model_validate(bag_status_payload)
    bag_stop = BagOperationResponse.model_validate(_payload(bag_stop_response))
    webrtc_stats = WebRTCStatsResponse.model_validate(webrtc_stats_payload)
    webrtc_bitrate = WebRTCControlResponse.model_validate(
        _payload(webrtc_bitrate_response)
    )

    assert temporal.count == 1
    assert temporal.observations[0]["label"] == "door"
    assert semantic_ok.count == 1
    assert semantic.error == "embedding required"
    assert explore_status.available is False
    assert explore_status.reason == "explorer_backend_not_running"
    assert explore_status.required_profile == "explore_or_tare_explore"
    assert explore_status.supported_profiles == ["explore", "tare_explore"]
    assert explore_start.error == "Exploration backend not running"
    assert explore_start.detail["reason"] == "explorer_backend_not_running"
    assert slam_status.mode in {
        "fastlio2",
        "localizer",
        "slam_only",
        "stopped",
        "super_lio",
        "super_lio_relocation",
    }
    assert slam_switch.schema_version == 1
    assert slam_switch.ok is False
    assert slam_switch.success is False
    assert slam_switch.ts > 0
    assert bag_status.recording is False
    assert bag_stop.error == "not_recording"
    assert webrtc_stats.enabled is False
    assert webrtc_bitrate.error == "webrtc_unavailable"


def test_tare_explorer_is_available_through_exploration_contracts():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import (
        ExplorationCommandResponse,
        ExplorationStatusResponse,
    )

    class TAREExplorerModule:
        def __init__(self):
            self.started = False

        def start_tare_exploration(self):
            self.started = True
            return json.dumps({"status": "started"})

        def stop_tare_exploration(self):
            self.started = False
            return json.dumps({"status": "stopped"})

        def get_tare_status(self):
            return json.dumps(
                {
                    "alive": True,
                    "started": self.started,
                    "waypoint_count": 3,
                    "finished": False,
                }
            )

    gateway = GatewayModule()
    gateway.setup()
    tare = TAREExplorerModule()
    gateway.on_system_modules({"TAREExplorerModule": tare})
    _seed_ready_navigation(gateway)
    gateway._on_tare_stats({"runtime_ms": 12.5})
    gateway._on_exploration_supervisor({"mode": "idle", "reason": "ready"})

    status_payload = asyncio.run(_endpoint(gateway, "/api/v1/explore/status")())
    start_payload = asyncio.run(_endpoint(gateway, "/api/v1/explore/start")())
    running_payload = asyncio.run(_endpoint(gateway, "/api/v1/explore/status")())
    stop_payload = asyncio.run(_endpoint(gateway, "/api/v1/explore/stop")())
    stopped_payload = asyncio.run(_endpoint(gateway, "/api/v1/explore/status")())
    snapshot = gateway._session_snapshot()

    status = ExplorationStatusResponse.model_validate(status_payload)
    started = ExplorationCommandResponse.model_validate(start_payload)
    running = ExplorationStatusResponse.model_validate(running_payload)
    stopped = ExplorationCommandResponse.model_validate(stop_payload)
    final_status = ExplorationStatusResponse.model_validate(stopped_payload)

    assert status.available is True
    assert status.backend == "tare"
    assert status.can_start is True
    assert status.blockers == []
    assert status.exploring is False
    assert status.tare["status"]["alive"] is True
    assert status.tare["status"]["waypoint_count"] == 3
    assert status.tare["stats"]["runtime_ms"] == 12.5
    assert status.supervisor["mode"] == "idle"
    assert started.status["status"] == "started"
    assert running.exploring is True
    assert running.can_start is False
    assert "exploration_already_active" in running.blockers
    assert stopped.status["status"] == "stopped"
    assert final_status.exploring is False
    assert snapshot["explorer_backend"] == "tare"
    assert snapshot["explorer_available"] is True
    assert snapshot["explorer_unavailable_reason"] is None
    assert snapshot["explorer_required_profile"] is None
    assert snapshot["can_start_exploring"] is True


def test_wavefront_explorer_is_available_through_exploration_contracts():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import (
        ExplorationCommandResponse,
        ExplorationStatusResponse,
    )

    class WavefrontFrontierExplorer:
        def __init__(self):
            self.started = False

        def begin_exploration(self):
            self.started = True
            return {"status": "started", "backend": "frontier"}

        def end_exploration(self):
            self.started = False
            return {"status": "stopped", "backend": "frontier"}

        def health(self):
            return {"frontier_count": 7}

    gateway = GatewayModule()
    gateway.setup()
    explorer = WavefrontFrontierExplorer()
    gateway.on_system_modules({"WavefrontFrontierExplorer": explorer})
    _seed_ready_navigation(gateway)

    status_payload = asyncio.run(_endpoint(gateway, "/api/v1/explore/status")())
    start_payload = asyncio.run(_endpoint(gateway, "/api/v1/explore/start")())
    running_payload = asyncio.run(_endpoint(gateway, "/api/v1/explore/status")())
    stop_payload = asyncio.run(_endpoint(gateway, "/api/v1/explore/stop")())
    stopped_payload = asyncio.run(_endpoint(gateway, "/api/v1/explore/status")())

    status = ExplorationStatusResponse.model_validate(status_payload)
    started = ExplorationCommandResponse.model_validate(start_payload)
    running = ExplorationStatusResponse.model_validate(running_payload)
    stopped = ExplorationCommandResponse.model_validate(stop_payload)
    final_status = ExplorationStatusResponse.model_validate(stopped_payload)

    assert status.available is True
    assert status.backend == "frontier"
    assert status.can_start is True
    assert status.blockers == []
    assert status.frontier_count == 7
    assert status.exploring is False
    assert started.status == {"status": "started", "backend": "frontier"}
    assert running.exploring is True
    assert running.can_start is False
    assert "exploration_already_active" in running.blockers
    assert stopped.status == {"status": "stopped", "backend": "frontier"}
    assert final_status.exploring is False
    assert gateway._session_snapshot()["explorer_backend"] == "frontier"


def test_explore_start_rejects_localization_recovery_blocker():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import ExplorationStatusResponse, GatewayErrorResponse

    class WavefrontFrontierExplorer:
        def __init__(self):
            self.started = False

        def begin_exploration(self):
            self.started = True
            return {"status": "started"}

        def health(self):
            return {"frontier_count": 2}

    gateway = GatewayModule()
    gateway.setup()
    explorer = WavefrontFrontierExplorer()
    gateway.on_system_modules({"WavefrontFrontierExplorer": explorer})
    _seed_ready_navigation(gateway)
    with gateway._state_lock:
        gateway._localization_status["recovery_signal"] = "LOC_DIVERGED"

    status_payload = asyncio.run(_endpoint(gateway, "/api/v1/explore/status")())
    start_response = asyncio.run(_endpoint(gateway, "/api/v1/explore/start")())

    status = ExplorationStatusResponse.model_validate(status_payload)
    error = GatewayErrorResponse.model_validate(_payload(start_response))

    assert status.available is True
    assert status.can_start is False
    assert "localization_recovery_active" in status.blockers
    assert start_response.status_code == 409
    assert error.error == "exploration_not_ready"
    assert "localization_recovery_active" in error.detail["blockers"]
    assert explorer.started is False
    assert gateway._exploring is False


def test_exploring_session_start_rejects_localization_recovery_blocker(monkeypatch):
    import core.service_manager as service_manager
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import SessionTransitionResponse

    class FakeServiceManager:
        def __init__(self):
            self.calls: list[tuple[str, tuple[str, ...]]] = []

        def stop(self, *services: str) -> None:
            self.calls.append(("stop", services))

        def ensure(self, *services: str) -> None:
            self.calls.append(("ensure", services))

        def wait_ready(self, *services: str, timeout: float = 15.0) -> bool:
            self.calls.append(("wait_ready", services))
            return True

    class TAREExplorerModule:
        def __init__(self):
            self.started = False

        def start_tare_exploration(self):
            self.started = True
            return {"status": "started"}

        def get_tare_status(self):
            return {"started": self.started}

    fake_service_manager = FakeServiceManager()
    monkeypatch.setattr(
        service_manager, "get_service_manager", lambda: fake_service_manager
    )

    gateway = GatewayModule()
    gateway.setup()
    tare = TAREExplorerModule()
    gateway.on_system_modules({"TAREExplorerModule": tare})
    _seed_ready_navigation(gateway)
    with gateway._state_lock:
        gateway._localization_status["recovery_signal"] = "LOC_DIVERGED"

    response = asyncio.run(
        _endpoint(gateway, "/api/v1/session/start")({"mode": "exploring"})
    )
    started = SessionTransitionResponse.model_validate(_payload(response))

    assert response.status_code == 409
    assert started.ok is False
    assert started.success is False
    assert "localization_recovery_active" in (started.message or "")
    assert started.detail["blockers"] == ["localization_recovery_active"]
    assert fake_service_manager.calls == []
    assert tare.started is False
    assert gateway._session_mode == "idle"


def test_tare_explorer_session_start_end_uses_exploration_backend(monkeypatch):
    import core.service_manager as service_manager
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import SessionTransitionResponse

    class FakeServiceManager:
        def __init__(self):
            self.calls: list[tuple[str, tuple[str, ...]]] = []

        def stop(self, *services: str) -> None:
            self.calls.append(("stop", services))

        def ensure(self, *services: str) -> None:
            self.calls.append(("ensure", services))

        def wait_ready(self, *services: str, timeout: float = 15.0) -> bool:
            self.calls.append(("wait_ready", services))
            return True

    class TAREExplorerModule:
        def __init__(self):
            self.started = False
            self.start_count = 0
            self.stop_count = 0

        def start_tare_exploration(self):
            self.started = True
            self.start_count += 1
            return {"status": "started"}

        def stop_tare_exploration(self):
            self.started = False
            self.stop_count += 1
            return {"status": "stopped"}

        def get_tare_status(self):
            return {"started": self.started}

    fake_service_manager = FakeServiceManager()
    monkeypatch.setattr(
        service_manager, "get_service_manager", lambda: fake_service_manager
    )

    gateway = GatewayModule()
    gateway.setup()
    tare = TAREExplorerModule()
    gateway.on_system_modules({"TAREExplorerModule": tare})
    _seed_ready_navigation(gateway)

    start_payload = asyncio.run(
        _endpoint(gateway, "/api/v1/session/start")({"mode": "exploring"})
    )
    started = SessionTransitionResponse.model_validate(start_payload)

    assert started.ok is True
    assert started.success is True
    assert started.session is not None
    assert started.session.mode == "exploring"
    assert started.session.slam_profile == "fastlio2"
    assert started.session.explorer_backend == "tare"
    assert tare.started is True
    assert tare.start_count == 1
    assert ("stop", ("localizer", "super_lio", "super_lio_relocation")) in (
        fake_service_manager.calls
    )
    assert ("ensure", ("slam", "slam_pgo")) in fake_service_manager.calls
    assert ("wait_ready", ("slam", "slam_pgo")) in fake_service_manager.calls

    end_payload = asyncio.run(_endpoint(gateway, "/api/v1/session/end")())
    ended = SessionTransitionResponse.model_validate(end_payload)

    assert ended.ok is True
    assert ended.success is True
    assert ended.session is not None
    assert ended.session.mode == "idle"
    assert ended.session.explorer_backend == "tare"
    assert tare.started is False
    assert tare.stop_count == 1
    assert (
        "stop",
        ("super_lio_relocation", "super_lio", "slam_pgo", "localizer", "slam"),
    ) in fake_service_manager.calls


def test_slam_status_uses_logical_service_states(monkeypatch):
    import core.service_manager as service_manager
    from gateway.gateway_module import GatewayModule

    class _FakeServiceManager:
        def __init__(self, services):
            self._services = services

        def status(self, *names):
            assert names == (
                "lidar",
                "slam",
                "slam_pgo",
                "localizer",
                "super_lio",
                "super_lio_relocation",
            )
            return dict(self._services)

    gateway = GatewayModule()
    gateway.setup()
    endpoint = _endpoint(gateway, "/api/v1/slam/status")

    monkeypatch.setattr(
        service_manager,
        "get_service_manager",
        lambda: _FakeServiceManager(
            {
                "lidar": "running",
                "slam": "running",
                "slam_pgo": "stopped",
                "localizer": "running",
                "super_lio": "stopped",
                "super_lio_relocation": "stopped",
            }
        ),
    )
    localizer_payload = asyncio.run(endpoint())
    assert localizer_payload["mode"] == "localizer"
    assert localizer_payload["services"]["slam"] == "running"

    monkeypatch.setattr(
        service_manager,
        "get_service_manager",
        lambda: _FakeServiceManager(
            {
                "lidar": "running",
                "slam": "running",
                "slam_pgo": "stopped",
                "localizer": "stopped",
                "super_lio": "stopped",
                "super_lio_relocation": "stopped",
            }
        ),
    )
    fastlio_payload = asyncio.run(endpoint())
    assert fastlio_payload["mode"] == "fastlio2"

    monkeypatch.setattr(
        service_manager,
        "get_service_manager",
        lambda: _FakeServiceManager(
            {
                "lidar": "running",
                "slam": "stopped",
                "slam_pgo": "stopped",
                "localizer": "stopped",
                "super_lio": "running",
                "super_lio_relocation": "stopped",
            }
        ),
    )
    super_lio_payload = asyncio.run(endpoint())
    assert super_lio_payload["mode"] == "super_lio"
    assert super_lio_payload["services"]["super_lio"] == "running"

    monkeypatch.setattr(
        service_manager,
        "get_service_manager",
        lambda: _FakeServiceManager(
            {
                "lidar": "running",
                "slam": "stopped",
                "slam_pgo": "stopped",
                "localizer": "stopped",
                "super_lio": "stopped",
                "super_lio_relocation": "running",
            }
        ),
    )
    relocation_payload = asyncio.run(endpoint())
    assert relocation_payload["mode"] == "super_lio_relocation"
    assert relocation_payload["services"]["super_lio_relocation"] == "running"


def test_slam_switch_can_select_super_lio(monkeypatch):
    import core.service_manager as service_manager
    from gateway.gateway_module import GatewayModule

    class _FakeServiceManager:
        def __init__(self):
            self.calls: list[tuple[str, tuple[str, ...]]] = []

        def stop(self, *names):
            self.calls.append(("stop", names))

        def ensure(self, *names):
            self.calls.append(("ensure", names))

        def wait_ready(self, *names, timeout: float = 15.0):
            self.calls.append(("wait_ready", names))
            return True

    fake = _FakeServiceManager()
    monkeypatch.setattr(service_manager, "get_service_manager", lambda: fake)

    gateway = GatewayModule()
    gateway.setup()
    endpoint = _endpoint(gateway, "/api/v1/slam/switch")

    payload = asyncio.run(endpoint({"profile": "super_lio"}))

    assert payload["schema_version"] == 1
    assert payload["ok"] is True
    assert payload["success"] is True
    assert payload["ts"] > 0
    assert payload["profile"] == "super_lio"
    assert ("stop", ("slam", "slam_pgo", "localizer", "super_lio_relocation")) in (
        fake.calls
    )
    assert ("ensure", ("lidar", "super_lio")) in fake.calls
    assert ("wait_ready", ("lidar", "super_lio")) in fake.calls


def test_slam_switch_can_select_super_lio_relocation(monkeypatch):
    import core.service_manager as service_manager
    from gateway.gateway_module import GatewayModule

    class _FakeServiceManager:
        def __init__(self):
            self.calls: list[tuple[str, tuple[str, ...]]] = []

        def stop(self, *names):
            self.calls.append(("stop", names))

        def ensure(self, *names):
            self.calls.append(("ensure", names))

        def wait_ready(self, *names, timeout: float = 15.0):
            self.calls.append(("wait_ready", names))
            return True

    fake = _FakeServiceManager()
    monkeypatch.setattr(service_manager, "get_service_manager", lambda: fake)

    gateway = GatewayModule()
    gateway.setup()
    endpoint = _endpoint(gateway, "/api/v1/slam/switch")

    payload = asyncio.run(endpoint({"profile": "super_lio_reloc"}))

    assert payload["schema_version"] == 1
    assert payload["ok"] is True
    assert payload["success"] is True
    assert payload["ts"] > 0
    assert payload["profile"] == "super_lio_relocation"
    assert ("stop", ("slam", "slam_pgo", "localizer", "super_lio")) in fake.calls
    assert ("ensure", ("lidar", "super_lio_relocation")) in fake.calls
    assert ("wait_ready", ("lidar", "super_lio_relocation")) in fake.calls


def test_super_lio_relocalize_endpoints_fail_fast_without_ros_call(monkeypatch):
    import subprocess

    from gateway.gateway_module import GatewayModule

    def fail_run(*_args, **_kwargs):
        raise AssertionError("ROS relocalization service should not be called")

    monkeypatch.setattr(subprocess, "run", fail_run)

    gateway = GatewayModule()
    gateway.setup()
    gateway._localization_status = {
        "localization_backend": "super_lio",
        "saved_map_relocalization_supported": False,
        "recovery_method": "restart_super_lio",
    }

    auto_response = asyncio.run(
        _endpoint(gateway, "/api/v1/slam/auto_relocalize")()
    )
    relocalize_response = asyncio.run(
        _endpoint(gateway, "/api/v1/slam/relocalize")(
            {"map_name": "demo", "x": 1.0, "y": 2.0, "yaw": 0.3}
        )
    )

    auto_payload = _payload(auto_response)
    relocalize_payload = _payload(relocalize_response)
    assert auto_response.status_code == 409
    assert relocalize_response.status_code == 409
    assert auto_payload["schema_version"] == 1
    assert relocalize_payload["schema_version"] == 1
    assert auto_payload["ok"] is False
    assert relocalize_payload["ok"] is False
    assert auto_payload["success"] is False
    assert relocalize_payload["success"] is False
    assert auto_payload["ts"] > 0
    assert relocalize_payload["ts"] > 0
    assert "unsupported" in auto_payload["message"]
    assert "unsupported" in relocalize_payload["message"]


def test_super_lio_relocation_relocalize_endpoints_fail_fast_without_ros_call(
    monkeypatch,
):
    import subprocess

    from gateway.gateway_module import GatewayModule

    def fail_run(*_args, **_kwargs):
        raise AssertionError("ROS relocalization service should not be called")

    monkeypatch.setattr(subprocess, "run", fail_run)

    gateway = GatewayModule()
    gateway.setup()
    gateway._localization_status = {
        "localization_backend": "super_lio_relocation",
        "saved_map_relocalization_supported": False,
        "recovery_method": "restart_super_lio_relocation",
    }

    auto_response = asyncio.run(
        _endpoint(gateway, "/api/v1/slam/auto_relocalize")()
    )
    relocalize_response = asyncio.run(
        _endpoint(gateway, "/api/v1/slam/relocalize")(
            {"map_name": "demo", "x": 1.0, "y": 2.0, "yaw": 0.3}
        )
    )

    auto_payload = _payload(auto_response)
    relocalize_payload = _payload(relocalize_response)
    assert auto_response.status_code == 409
    assert relocalize_response.status_code == 409
    assert auto_payload["schema_version"] == 1
    assert relocalize_payload["schema_version"] == 1
    assert auto_payload["ok"] is False
    assert relocalize_payload["ok"] is False
    assert auto_payload["success"] is False
    assert relocalize_payload["success"] is False
    assert auto_payload["ts"] > 0
    assert relocalize_payload["ts"] > 0
    assert "unsupported" in auto_payload["message"]
    assert "unsupported" in relocalize_payload["message"]


def test_localizer_relocalize_keeps_ros_service_path(monkeypatch, tmp_path):
    import subprocess

    from gateway.gateway_module import GatewayModule

    calls = []
    map_dir = tmp_path / "maps"
    (map_dir / "demo").mkdir(parents=True)
    (map_dir / "demo" / "map.pcd").write_text("pcd", encoding="utf-8")
    monkeypatch.setenv("NAV_MAP_DIR", str(map_dir))

    def fake_run(args, **kwargs):
        calls.append((args, kwargs))
        return subprocess.CompletedProcess(args, 0, stdout="success=True\n", stderr="")

    monkeypatch.setattr(subprocess, "run", fake_run)

    gateway = GatewayModule()
    gateway.setup()
    calls.clear()
    gateway._localization_status = {
        "backend": "localizer",
        "saved_map_relocalization_supported": True,
    }
    gateway._persist_last_nav_pose = lambda *_args, **_kwargs: None

    payload = asyncio.run(
        _endpoint(gateway, "/api/v1/slam/relocalize")(
            {"map_name": "demo", "x": 1.0, "y": 2.0, "yaw": 0.3}
        )
    )

    assert payload["schema_version"] == 1
    assert payload["ok"] is True
    assert payload["success"] is True
    assert payload["ts"] > 0
    assert payload["message"] == "Relocalized to demo"
    assert calls
    assert any("/nav/relocalize" in " ".join(args) for args, _kwargs in calls)


def test_temporal_memory_response_accepts_observation_rows():
    from gateway.schemas import TemporalMemoryResponse

    payload = {
        "observations": [{"label": "door", "score": 0.92}],
        "count": 1,
    }

    response = TemporalMemoryResponse.model_validate(payload)

    assert response.count == 1
    assert response.observations[0]["label"] == "door"
