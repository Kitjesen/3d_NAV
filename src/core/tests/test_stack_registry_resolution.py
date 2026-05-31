from __future__ import annotations

from core.module import Module
from core.registry import clear, register, restore, snapshot


def _entry_classes(bp) -> list[type]:
    return [entry.module_cls for entry in bp._entries]


def test_safety_stack_prefers_registered_modules():
    from core.blueprints.stacks.safety import safety

    saved = snapshot()
    try:
        clear()

        @register("safety", "ring")
        class FakeSafetyRing(Module, layer=0):
            pass

        @register("safety", "cmd_vel_mux")
        class FakeCmdVelMux(Module, layer=0):
            pass

        @register("safety", "geofence")
        class FakeGeofence(Module, layer=0):
            pass

        classes = _entry_classes(safety())

        assert classes == [FakeSafetyRing, FakeCmdVelMux, FakeGeofence]
    finally:
        restore(saved)


def test_lidar_stack_prefers_registered_mid360_module():
    from core.blueprints.stacks.lidar import lidar

    saved = snapshot()
    try:
        clear()

        @register("lidar", "mid360")
        class FakeLidar(Module, layer=1):
            pass

        bp = lidar(ip="192.0.2.10")

        assert _entry_classes(bp) == [FakeLidar]
        assert bp._entries[0].config == {"ip": "192.0.2.10"}
    finally:
        restore(saved)


def test_gateway_stack_prefers_registered_interface_modules():
    from core.blueprints.stacks.gateway import gateway

    saved = snapshot()
    try:
        clear()

        @register("gateway", "fastapi")
        class FakeGateway(Module, layer=6):
            pass

        @register("mcp", "server")
        class FakeMcp(Module, layer=6):
            pass

        @register("teleop", "default")
        class FakeTeleop(Module, layer=6):
            pass

        @register("webrtc", "aiortc")
        class FakeWebRtc(Module, layer=6):
            pass

        @register("visualization", "rerun")
        class FakeRerun(Module, layer=6):
            pass

        bp = gateway(port=5051, mcp_port=8091, enable_teleop=True, enable_rerun=True)

        assert _entry_classes(bp) == [
            FakeGateway,
            FakeMcp,
            FakeTeleop,
            FakeWebRtc,
            FakeRerun,
        ]
        assert bp._entries[0].config == {"port": 5051}
        assert bp._entries[1].config == {"port": 8091}
        assert bp._entries[2].config == {"port": 5051}
        assert bp._entries[4].config == {"web_port": 9090}
    finally:
        restore(saved)

