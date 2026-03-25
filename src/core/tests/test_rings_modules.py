"""Tests for three-ring cognitive architecture hive Modules."""

import math
import time

import numpy as np
import pytest

from core import Module, In, Out, Blueprint, autoconnect
from core.msgs.geometry import Pose, PoseStamped, Quaternion, Twist, Vector3
from core.msgs.nav import Odometry, Path
from core.msgs.semantic import (
    DialogueState, ExecutionEval, MissionStatus, SafetyState,
)
from nav.rings.nav_rings.safety_module import SafetyModule, SafetyLevel, LinkStatus
from nav.rings.nav_rings.evaluator_module import EvaluatorModule, Assessment
from nav.rings.nav_rings.dialogue_module import DialogueModule


def make_odom(x=0.0, y=0.0, z=0.0, vx=0.0, vy=0.0, yaw=0.0) -> Odometry:
    q = Quaternion.from_euler(0, 0, yaw)
    return Odometry(
        pose=Pose(position=Vector3(x, y, z), orientation=q),
        twist=Twist(linear=Vector3(vx, vy, 0), angular=Vector3(0, 0, 0)),
    )


def make_path(points) -> Path:
    poses = [PoseStamped(pose=Pose(position=Vector3(x, y, 0))) for x, y in points]
    return Path(poses=poses)


class TestSafetyModulePorts:
    def test_in_ports(self):
        mod = SafetyModule()
        for name in ["odometry","terrain_hb","cmd_vel","stop_signal","slow_down","watchdog","planner_status","loc_quality"]:
            assert name in mod.ports_in
    def test_out_ports(self):
        mod = SafetyModule()
        assert "safety_state" in mod.ports_out
        assert "stop_cmd" in mod.ports_out
    def test_port_types(self):
        mod = SafetyModule()
        assert mod.ports_in["odometry"].msg_type is Odometry
        assert mod.ports_in["cmd_vel"].msg_type is Twist
        assert mod.ports_in["stop_signal"].msg_type is int
        assert mod.ports_out["safety_state"].msg_type is SafetyState
        assert mod.ports_out["stop_cmd"].msg_type is int
    def test_layer(self):
        assert SafetyModule._layer == 0
    def test_config(self):
        mod = SafetyModule(odom_timeout_sec=5.0)
        assert mod._odom_timeout == 5.0

class TestSafetyModuleLogic:
    def test_ok_no_signals(self):
        mod = SafetyModule()
        mod.setup()
        state = mod.tick()
        assert state.level == "OK"
        assert state.issues == []
    def test_stop_level_2_estop(self):
        mod = SafetyModule()
        mod.setup()
        mod.stop_signal._deliver(2)
        state = mod.tick()
        assert state.level == "ESTOP"
        assert any("stop=2" in i for i in state.issues)
    def test_stop_level_1_warn(self):
        mod = SafetyModule()
        mod.setup()
        mod.stop_signal._deliver(1)
        assert mod.tick().level == "WARN"
    def test_stuck_warn(self):
        mod = SafetyModule()
        mod.setup()
        mod.planner_status._deliver("STUCK")
        state = mod.tick()
        assert state.level == "WARN"
        assert any("STUCK" in i for i in state.issues)
    def test_warn_stuck(self):
        mod = SafetyModule()
        mod.setup()
        mod.planner_status._deliver("WARN_STUCK")
        assert mod.tick().level == "WARN"
    def test_slow_down_degraded(self):
        mod = SafetyModule()
        mod.setup()
        mod.slow_down._deliver(2)
        assert mod.tick().level == "DEGRADED"
    def test_watchdog_warn(self):
        mod = SafetyModule()
        mod.setup()
        mod.watchdog._deliver(True)
        assert mod.tick().level == "WARN"
    def test_low_loc_quality(self):
        mod = SafetyModule(loc_quality_warn_threshold=0.5)
        mod.setup()
        mod.loc_quality._deliver(0.2)
        assert mod.tick().level == "WARN"
    def test_odom_heartbeat(self):
        mod = SafetyModule(odom_timeout_sec=0.05)
        mod.setup()
        mod.odometry._deliver(make_odom(1.0, 2.0))
        assert mod._links["slam"].last_seen > 0
    def test_stale_slam_critical(self):
        mod = SafetyModule(odom_timeout_sec=0.01)
        mod.setup()
        mod.odometry._deliver(make_odom())
        time.sleep(0.02)
        assert mod.tick().level == "DANGER"
    def test_stale_terrain_noncritical(self):
        mod = SafetyModule(terrain_timeout_sec=0.01)
        mod.setup()
        mod.terrain_hb._deliver(time.monotonic())
        time.sleep(0.02)
        assert mod.tick().level == "DEGRADED"
    def test_navigating_flag(self):
        mod = SafetyModule()
        mod.setup()
        mod.cmd_vel._deliver(Twist(linear=Vector3(0.5, 0.0, 0.0)))
        assert mod.navigating is True
    def test_not_navigating(self):
        mod = SafetyModule()
        mod.setup()
        mod.cmd_vel._deliver(Twist(linear=Vector3(0.0, 0.0, 0.0)))
        assert mod.navigating is False
    def test_escalation(self):
        mod = SafetyModule(odom_timeout_sec=0.01, escalation_holdoff_sec=0.0)
        mod.setup()
        mod.cmd_vel._deliver(Twist(linear=Vector3(0.5, 0.0, 0.0)))
        mod.odometry._deliver(make_odom())
        time.sleep(0.02)
        stop_cmds = []
        mod.stop_cmd._add_callback(stop_cmds.append)
        mod.tick()
        assert stop_cmds == [2]
    def test_escalation_holdoff(self):
        mod = SafetyModule(odom_timeout_sec=0.01, escalation_holdoff_sec=10.0)
        mod.setup()
        mod.cmd_vel._deliver(Twist(linear=Vector3(0.5, 0.0, 0.0)))
        mod.odometry._deliver(make_odom())
        time.sleep(0.02)
        stop_cmds = []
        mod.stop_cmd._add_callback(stop_cmds.append)
        mod.tick()
        assert stop_cmds == [2]
        stop_cmds.clear()
        mod.tick()
        assert stop_cmds == []
    def test_publishes_safety_state(self):
        mod = SafetyModule()
        mod.setup()
        received = []
        mod.safety_state._add_callback(received.append)
        mod.tick()
        assert len(received) == 1
        assert isinstance(received[0], SafetyState)
    def test_multiple_signals_highest_wins(self):
        mod = SafetyModule()
        mod.setup()
        mod.slow_down._deliver(1)
        mod.planner_status._deliver("STUCK")
        mod.stop_signal._deliver(2)
        assert mod.tick().level == "ESTOP"

class TestEvaluatorModulePorts:
    def test_in_ports(self):
        mod = EvaluatorModule()
        for name in ["odometry","path","waypoint","cmd_vel"]:
            assert name in mod.ports_in
    def test_out_ports(self):
        mod = EvaluatorModule()
        assert "execution_eval" in mod.ports_out
    def test_port_types(self):
        mod = EvaluatorModule()
        assert mod.ports_in["odometry"].msg_type is Odometry
        assert mod.ports_in["path"].msg_type is Path
        assert mod.ports_in["waypoint"].msg_type is Vector3
        assert mod.ports_in["cmd_vel"].msg_type is Twist
        assert mod.ports_out["execution_eval"].msg_type is ExecutionEval
    def test_layer(self):
        assert EvaluatorModule._layer == 2


class TestEvaluatorModuleLogic:
    def test_idle_without_path(self):
        mod = EvaluatorModule()
        mod.setup()
        assert mod.tick().assessment == "IDLE"
    def test_on_track(self):
        mod = EvaluatorModule()
        mod.setup()
        mod.path._deliver(make_path([(0, 0), (10, 0)]))
        mod.odometry._deliver(make_odom(1.0, 0.0, vx=0.5))
        r = mod.tick()
        assert r.assessment == "ON_TRACK"
        assert r.cross_track_error < 0.01
    def test_drifting(self):
        mod = EvaluatorModule(cross_track_warn=1.0)
        mod.setup()
        mod.path._deliver(make_path([(0, 0), (10, 0)]))
        mod.odometry._deliver(make_odom(5.0, 3.0, vx=0.5))
        r = mod.tick()
        assert r.assessment == "DRIFTING"
        assert r.cross_track_error >= 1.0
    def test_cross_track_error(self):
        mod = EvaluatorModule()
        mod.setup()
        mod.path._deliver(make_path([(0, 0), (10, 0)]))
        mod.odometry._deliver(make_odom(5.0, 2.0))
        assert abs(mod.cross_track_error() - 2.0) < 0.01
    def test_distance_to_goal(self):
        mod = EvaluatorModule()
        mod.setup()
        mod.path._deliver(make_path([(0, 0), (10, 0)]))
        mod.odometry._deliver(make_odom(3.0, 0.0))
        assert abs(mod.distance_to_goal() - 7.0) < 0.01
    def test_heading_error(self):
        mod = EvaluatorModule()
        mod.setup()
        mod.path._deliver(make_path([(0, 0), (10, 0)]))
        mod.odometry._deliver(make_odom(0.0, 0.0, yaw=math.pi / 2))
        assert abs(mod.heading_error() - (-math.pi / 2)) < 0.1
    def test_assess_regressing(self):
        mod = EvaluatorModule()
        mod.setup()
        mod.path._deliver(make_path([(0, 0), (10, 0)]))
        assert mod.assess(0.5, 0.1, 5.0) == Assessment.REGRESSING
    def test_assess_stalled(self):
        mod = EvaluatorModule(progress_window_sec=2.0)
        mod.setup()
        mod.path._deliver(make_path([(0, 0), (10, 0)]))
        assert mod.assess(0.5, 0.0, 3.0) == Assessment.STALLED
    def test_publishes(self):
        mod = EvaluatorModule()
        mod.setup()
        received = []
        mod.execution_eval._add_callback(received.append)
        mod.tick()
        assert len(received) == 1
        assert isinstance(received[0], ExecutionEval)
    def test_path_reset(self):
        mod = EvaluatorModule()
        mod.setup()
        mod.path._deliver(make_path([(0, 0), (10, 0)]))
        mod.odometry._deliver(make_odom(5.0, 0.0))
        mod.tick()
        mod.path._deliver(make_path([(0, 0), (0, 10)]))
        assert mod._last_distance == float("inf")

class TestDialogueModulePorts:
    def test_in_ports(self):
        mod = DialogueModule()
        for name in ["safety_state","execution_eval","mission_status","planner_status","odometry","instruction"]:
            assert name in mod.ports_in
    def test_out_ports(self):
        mod = DialogueModule()
        assert "dialogue_state" in mod.ports_out
    def test_port_types(self):
        mod = DialogueModule()
        assert mod.ports_in["safety_state"].msg_type is SafetyState
        assert mod.ports_in["execution_eval"].msg_type is ExecutionEval
        assert mod.ports_in["mission_status"].msg_type is MissionStatus
        assert mod.ports_out["dialogue_state"].msg_type is DialogueState
    def test_layer(self):
        assert DialogueModule._layer == 6


class TestDialogueModuleLogic:
    def test_idle_default(self):
        mod = DialogueModule()
        mod.setup()
        state = mod.tick()
        assert state.mission_state == "IDLE"
        assert state.safety == "OK"
    def test_instruction_captured(self):
        mod = DialogueModule()
        mod.setup()
        mod.instruction._deliver("go to kitchen")
        assert mod.tick().understood == "go to kitchen"
    def test_safety_aggregated(self):
        mod = DialogueModule()
        mod.setup()
        mod.safety_state._deliver(SafetyState(level="DANGER", issues=["SLAM lost"]))
        state = mod.tick()
        assert state.safety == "DANGER"
        assert state.issue is not None
        assert "SLAM" in state.issue
    def test_eval_drifting(self):
        mod = DialogueModule()
        mod.setup()
        mod.execution_eval._deliver(ExecutionEval(assessment="DRIFTING", cross_track_error=2.5, distance_to_goal=8.0))
        state = mod.tick()
        assert state.issue is not None
        assert "2.5" in state.issue
    def test_mission_executing(self):
        mod = DialogueModule()
        mod.setup()
        mod.mission_status._deliver(MissionStatus(state="EXECUTING", progress_pct=50.0))
        mod.execution_eval._deliver(ExecutionEval(assessment="ON_TRACK", distance_to_goal=5.0))
        state = mod.tick()
        assert state.mission_state == "EXECUTING"
        assert state.progress_pct == 50.0
        assert state.distance_m == 5.0
    def test_position_from_odom(self):
        mod = DialogueModule()
        mod.setup()
        mod.odometry._deliver(make_odom(3.5, 7.2))
        state = mod.tick()
        assert abs(state.position_x - 3.5) < 0.01
        assert abs(state.position_y - 7.2) < 0.01
    def test_planner_failed(self):
        mod = DialogueModule()
        mod.setup()
        mod.planner_status._deliver("FAILED")
        assert mod.tick().issue is not None
    def test_publishes(self):
        mod = DialogueModule()
        mod.setup()
        received = []
        mod.dialogue_state._add_callback(received.append)
        mod.tick()
        assert len(received) == 1
        assert isinstance(received[0], DialogueState)
    def test_safety_priority(self):
        mod = DialogueModule()
        mod.setup()
        mod.safety_state._deliver(SafetyState(level="DANGER", issues=["link down"]))
        mod.execution_eval._deliver(ExecutionEval(assessment="DRIFTING", cross_track_error=2.0))
        state = mod.tick()
        assert "link down" in state.issue
    def test_eta(self):
        mod = DialogueModule()
        mod.setup()
        mod.mission_status._deliver(MissionStatus(state="EXECUTING"))
        mod.execution_eval._deliver(ExecutionEval(assessment="ON_TRACK", distance_to_goal=10.0, progress_rate=-0.5))
        state = mod.tick()
        assert state.eta_sec == 20

class TestAutoconnectWiring:
    def test_safety_to_dialogue(self):
        system = Blueprint().add(SafetyModule).add(DialogueModule).auto_wire().build()
        system.start()
        safety = system.get_module("SafetyModule")
        dialogue = system.get_module("DialogueModule")
        safety.stop_signal._deliver(2)
        safety.tick()
        state = dialogue.tick()
        assert state.safety == "ESTOP"
        system.stop()
    def test_evaluator_to_dialogue(self):
        system = Blueprint().add(EvaluatorModule).add(DialogueModule).auto_wire().build()
        system.start()
        evaluator = system.get_module("EvaluatorModule")
        dialogue = system.get_module("DialogueModule")
        evaluator.tick()
        dialogue.tick()
        assert dialogue._eval is not None
        assert dialogue._eval.assessment == "IDLE"
        system.stop()
    def test_three_ring_system(self):
        system = Blueprint().add(SafetyModule).add(EvaluatorModule).add(DialogueModule).auto_wire().build()
        system.start()
        conns = system.connections
        safety_conn = [c for c in conns if c[0] == "SafetyModule" and c[2] == "DialogueModule"]
        assert len(safety_conn) >= 1
        eval_conn = [c for c in conns if c[0] == "EvaluatorModule" and c[2] == "DialogueModule"]
        assert len(eval_conn) >= 1
        system.stop()
    def test_startup_order(self):
        system = Blueprint().add(DialogueModule).add(EvaluatorModule).add(SafetyModule).auto_wire().build()
        order = system.health()["startup_order"]
        si = order.index("SafetyModule")
        ei = order.index("EvaluatorModule")
        di = order.index("DialogueModule")
        assert si < ei < di


class TestEndToEndRings:
    def test_full_pipeline_safe(self):
        system = Blueprint().add(SafetyModule).add(EvaluatorModule).add(DialogueModule).auto_wire().build()
        system.start()
        system.get_module("SafetyModule").tick()
        system.get_module("EvaluatorModule").tick()
        state = system.get_module("DialogueModule").tick()
        assert state.safety == "OK"
        assert state.mission_state == "IDLE"
        assert state.issue is None
        system.stop()
    def test_full_pipeline_danger(self):
        system = Blueprint().add(SafetyModule).add(EvaluatorModule).add(DialogueModule).auto_wire().build()
        system.start()
        safety = system.get_module("SafetyModule")
        safety.stop_signal._deliver(2)
        safety.tick()
        system.get_module("EvaluatorModule").tick()
        state = system.get_module("DialogueModule").tick()
        assert state.safety == "ESTOP"
        assert state.issue is not None
        system.stop()
    def test_msg_counts(self):
        system = Blueprint().add(SafetyModule).add(DialogueModule).auto_wire().build()
        system.start()
        safety = system.get_module("SafetyModule")
        dialogue = system.get_module("DialogueModule")
        for _ in range(5):
            safety.tick()
        assert safety.safety_state.msg_count == 5
        assert dialogue.safety_state.msg_count == 5
        system.stop()
    def test_health_check(self):
        system = Blueprint().add(SafetyModule).add(EvaluatorModule).add(DialogueModule).auto_wire().build()
        system.start()
        h = system.health()
        assert h["module_count"] == 3
        assert h["started"] is True
        assert h["connection_count"] >= 2
        system.stop()
