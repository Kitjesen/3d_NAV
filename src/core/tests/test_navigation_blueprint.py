#!/usr/bin/env python3
"""Tests for the navigation system blueprint (new module architecture)."""

from __future__ import annotations

import pytest

from core.blueprint import Blueprint, SystemHandle
from core.blueprints.navigation import navigation_blueprint

EXPECTED_MODULES = {
    "ThunderDriver",
    "NavigationModule",
    "SafetyRingModule",
}


class TestNavigationBlueprint:

    def test_returns_blueprint(self):
        bp = navigation_blueprint()
        assert isinstance(bp, Blueprint)

    def test_build_creates_system_handle(self):
        handle = navigation_blueprint().build()
        assert isinstance(handle, SystemHandle)

    def test_all_modules_present(self):
        handle = navigation_blueprint().build()
        names = set(handle.modules.keys())
        assert names == EXPECTED_MODULES

    def test_module_count(self):
        handle = navigation_blueprint().build()
        assert len(handle.modules) == 3

    def test_connections_exist(self):
        handle = navigation_blueprint().build()
        assert len(handle.connections) > 0

    def test_explicit_wire_stop_cmd(self):
        handle = navigation_blueprint().build()
        found = any(
            out_mod == "SafetyRingModule" and out_port == "stop_cmd"
            and in_mod == "ThunderDriver" and in_port == "stop_signal"
            for out_mod, out_port, in_mod, in_port in handle.connections
        )
        assert found, "stop_cmd -> ThunderDriver.stop_signal not wired"

    def test_odometry_auto_wired(self):
        handle = navigation_blueprint().build()
        odom_targets = [
            in_mod
            for out_mod, out_port, in_mod, in_port in handle.connections
            if out_port == "odometry" and in_port == "odometry"
        ]
        assert len(odom_targets) >= 2, f"odometry only wired to {odom_targets}"

    def test_start_stop_lifecycle(self):
        handle = navigation_blueprint().build()
        handle.start()
        assert handle.started

        for name, mod in handle.modules.items():
            assert mod.running, f"{name} not running after start()"

        handle.stop()
        assert not handle.started

    def test_data_flow_odometry(self):
        from core.msgs.nav import Odometry
        from core.msgs.geometry import Pose, Twist, Vector3, Quaternion

        handle = navigation_blueprint().build()
        handle.start()

        dog = handle.get_module("ThunderDriver")
        safety = handle.get_module("SafetyRingModule")

        assert safety.ports_in["odometry"].msg_count == 0

        odom = Odometry(
            pose=Pose(
                position=Vector3(1.0, 2.0, 0.0),
                orientation=Quaternion(0, 0, 0, 1),
            ),
            twist=Twist(
                linear=Vector3(0.5, 0, 0),
                angular=Vector3(0, 0, 0),
            ),
        )
        dog.odometry.publish(odom)

        assert safety.ports_in["odometry"].msg_count == 1
        handle.stop()

    def test_data_flow_stop_cmd(self):
        handle = navigation_blueprint().build()
        handle.start()

        safety = handle.get_module("SafetyRingModule")
        dog = handle.get_module("ThunderDriver")

        assert dog.ports_in["stop_signal"].msg_count == 0
        safety.stop_cmd.publish(2)
        assert dog.ports_in["stop_signal"].msg_count == 1

        handle.stop()

    def test_config_passthrough(self):
        handle = navigation_blueprint(
            dog_host="10.0.0.42", dog_port=9999
        ).build()
        dog = handle.get_module("ThunderDriver")
        assert dog._dog_host == "10.0.0.42"
        assert dog._dog_port == 9999

    def test_wiring_diagram_printable(self):
        handle = navigation_blueprint().build()
        health = handle.health()

        assert health["module_count"] == 3
        assert health["connection_count"] > 0
        assert len(health["startup_order"]) == 3


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])
