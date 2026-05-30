from __future__ import annotations

import time


def test_slam_module_health_reports_backend_status_contract():
    from slam.slam_module import SLAMModule

    module = SLAMModule(backend="fastlio2")

    info = module.health()["slam"]

    assert info["configured_backend"] == "fastlio2"
    assert info["backend"] == "fastlio2"
    assert info["degraded"] is False
    assert info["degraded_reason"] == ""


def test_slam_bridge_health_reports_configured_and_effective_backend():
    from slam.slam_bridge_module import SlamBridgeModule

    module = SlamBridgeModule(backend_profile="bridge")
    module._current_backend_profile = lambda: "localizer"

    info = module.health()["localization"]

    assert info["configured_backend"] == "bridge"
    assert info["backend"] == "localizer"
    assert info["degraded"] is False
    assert info["degraded_reason"] == ""


def test_slam_bridge_localization_status_reports_backend_status_contract():
    from slam.slam_bridge_module import SlamBridgeModule

    module = SlamBridgeModule(
        backend_profile="bridge",
        odom_timeout=0.5,
        cloud_timeout=0.5,
        watchdog_hz=50,
    )
    module._current_backend_profile = lambda: "localizer"
    received = []
    module.localization_status._add_callback(received.append)
    module._mark_odom_received()
    module._mark_cloud_received()

    module.start()
    try:
        deadline = time.time() + 1.0
        while time.time() < deadline and not any(
            item.get("state") == "TRACKING" for item in received
        ):
            time.sleep(0.02)
    finally:
        module.stop()

    tracking = next(item for item in received if item["state"] == "TRACKING")
    assert tracking["configured_backend"] == "bridge"
    assert tracking["backend"] == "localizer"
    assert tracking["degraded"] is False
    assert tracking["degraded_reason"] == ""
