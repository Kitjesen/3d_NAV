#!/usr/bin/env python3
"""Tests for the navigation system blueprint."""

from __future__ import annotations

import pytest

from core.blueprint import Blueprint, SystemHandle
from core.blueprints.navigation import navigation_blueprint

EXPECTED_MODULES = {
    "ThunderDriver",
    "SafetyModule",
    "EvaluatorModule",
    "DialogueModule",
    "PathAdapterModule",
    "MissionArcModule",
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
        assert len(handle.modules) == 6

    def test_connections_exist(self):
        handle = navigation_blueprint().build()
        assert len(handle.connections) > 0

    def test_explicit_wire_stop_cmd_to_han_dog(self):
        handle = navigation_blueprint().build()
        found = any(
            out_mod == "SafetyModule" and out_port == "stop_cmd"
            and in_mod == "ThunderDriver" and in_port == "stop_signal"
            for out_mod, out_port, in_mod, in_port in handle.connections
        )
        assert found, "stop_cmd -> ThunderDriver.stop_signal not wired"

    def test_explicit_wire_stop_cmd_to_mission_arc(self):
        handle = navigation_blueprint().build()
        found = any(
            out_mod == "SafetyModule" and out_port == "stop_cmd"
            and in_mod == "MissionArcModule" and in_port == "stop_signal"
            for out_mod, out_port, in_mod, in_port in handle.connections
        )
        assert found, "stop_cmd -> MissionArcModule.stop_signal not wired"

    def test_odometry_auto_wired(self):
        handle = navigation_blueprint().build()
        odom_targets = [
            in_mod
            for out_mod, out_port, in_mod, in_port in handle.connections
            if out_port == "odometry" and in_port == "odometry"
        ]
        assert len(odom_targets) >= 3, f"odometry only wired to {odom_targets}"

    def test_adapter_status_wired(self):
        handle = navigation_blueprint().build()
        found = any(
            out_port == "adapter_status" and in_port == "adapter_status"
            and in_mod == "MissionArcModule"
            for out_mod, out_port, in_mod, in_port in handle.connections
        )
        assert found, "adapter_status not wired to MissionArcModule"

    def test_safety_state_wired_to_dialogue(self):
        handle = navigation_blueprint().build()
        found = any(
            out_port == "safety_state" and in_mod == "DialogueModule"
            for _, out_port, in_mod, _ in handle.connections
        )
        assert found, "safety_state not wired to DialogueModule"

    def test_execution_eval_wired_to_dialogue(self):
        handle = navigation_blueprint().build()
        found = any(
            out_port == "execution_eval" and in_mod == "DialogueModule"
            for _, out_port, in_mod, _ in handle.connections
        )
        assert found, "execution_eval not wired to DialogueModule"

    def test_no_unexpected_layer_violations(self):
        handle = navigation_blueprint().build()
        health = handle.health()
        violations = health.get("layer_violations", [])
        # The Safety <-> HanDog feedback cycle is expected and valid:
        # SafetyModule.stop_cmd -> ThunderDriver.stop_signal (command)
        # ThunderDriver.odometry -> SafetyModule.odometry (sensing)
        unexpected = [v for v in violations if "SafetyModule" not in v or "ThunderDriver" not in v]
        assert unexpected == [], f"Unexpected violations: {unexpected}"

    def test_startup_order_layers(self):
        handle = navigation_blueprint().build()
        order = handle.health()["startup_order"]

        safety_idx = order.index("SafetyModule")
        dialogue_idx = order.index("DialogueModule")
        assert safety_idx < dialogue_idx, (
            f"SafetyModule (L0) at {safety_idx} must precede "
            f"DialogueModule (L6) at {dialogue_idx}"
        )

        dog_idx = order.index("ThunderDriver")
        eval_idx = order.index("EvaluatorModule")
        assert dog_idx < eval_idx, (
            f"ThunderDriver (L1) at {dog_idx} must precede "
            f"EvaluatorModule (L2) at {eval_idx}"
        )

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
        safety = handle.get_module("SafetyModule")

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

        safety = handle.get_module("SafetyModule")
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

        assert health["module_count"] == 6
        assert health["connection_count"] > 0
        assert len(health["startup_order"]) == 6

        # Print wiring diagram for debugging (visible with -s flag)
        print("\n=== Navigation Blueprint Wiring Diagram ===")
        for out_mod, out_port, in_mod, in_port in handle.connections:
            print(f"  {out_mod}.{out_port} -> {in_mod}.{in_port}")
        for name, info in health["modules"].items():
            layer = info.get("layer", "?")
            print(f"  [{name}] L{layer}")
        print("=== End ===\n")


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])
