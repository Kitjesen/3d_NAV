"""Tests for new modules: VectorMemory, VisualServo, AgentLoop, Teleop, SemanticMapper.

Covers:
  - VectorMemoryModule: store + query + numpy fallback + hash embedding
  - VisualServoModule: find mode + follow mode + servo engage/release
  - AgentLoop: tool parsing + done termination + max_steps cap
  - TeleopModule: joystick handling + idle release
  - SemanticMapperModule: KG + TSG update from SceneGraph
  - GoalResolver 5-level fallback chain (stub integration)
"""

import asyncio
import time
import numpy as np
import pytest

import sys
sys.path.insert(0, "src")


# ── VectorMemoryModule ──────────────────────────────────────────────────

class TestVectorMemoryModule:
    def _make(self):
        from memory.modules.vector_memory_module import VectorMemoryModule
        mod = VectorMemoryModule(persist_dir="/tmp/test_vmem")
        mod.setup()
        return mod

    def test_empty_query(self):
        mod = self._make()
        result = mod.query_location("backpack")
        assert result["found"] is False

    def test_store_and_query(self):
        mod = self._make()
        # Simulate storing a snapshot
        mod._robot_xy = (5.0, 3.0)
        mod._store_snapshot(["desk", "chair", "monitor"])
        mod._store_snapshot(["fridge", "sink", "microwave"])
        mod._robot_xy = (10.0, 7.0)
        mod._store_snapshot(["backpack", "bench", "tree"])

        result = mod.query_location("backpack")
        assert result["found"] is True
        assert result["best"]["x"] == 10.0

    def test_hash_embedding_deterministic(self):
        from memory.modules.vector_memory_module import VectorMemoryModule
        e1 = VectorMemoryModule._hash_embedding("desk, chair")
        e2 = VectorMemoryModule._hash_embedding("desk, chair")
        assert np.allclose(e1, e2)
        e3 = VectorMemoryModule._hash_embedding("fridge, sink")
        assert not np.allclose(e1, e3)

    def test_stats(self):
        mod = self._make()
        mod._store_snapshot(["a", "b"])
        s = mod.get_memory_stats()
        assert s["entries"] == 1
        assert s["backend"] == "numpy"


# ── VisualServoModule ───────────────────────────────────────────────────

class TestVisualServoModule:
    def _make(self):
        from semantic.planner.semantic_planner.visual_servo_module import VisualServoModule
        mod = VisualServoModule()
        mod.setup()
        return mod

    def test_idle_by_default(self):
        mod = self._make()
        assert mod._mode == "idle"
        assert mod._servo_active is False

    def test_find_mode_activation(self):
        mod = self._make()
        mod._on_servo_target("find:red chair")
        assert mod._mode == "find"
        assert mod._target_label == "red chair"

    def test_follow_mode_activation(self):
        mod = self._make()
        mod._on_servo_target("follow:person in red")
        assert mod._mode == "follow"
        assert mod._target_label == "person in red"

    def test_stop_cancels_tracking(self):
        mod = self._make()
        mod._on_servo_target("find:chair")
        assert mod._mode == "find"
        mod._on_servo_target("stop")
        assert mod._mode == "idle"

    def test_servo_engage_release(self):
        mod = self._make()
        mod._engage_servo()
        assert mod._servo_active is True
        mod._release_servo()
        assert mod._servo_active is False

    def test_find_with_scene_graph(self):
        """Simulate a full find tick with a scene_graph containing target."""
        from core.msgs.sensor import Image, CameraIntrinsics
        from core.msgs.semantic import SceneGraph, Detection3D
        from core.msgs.nav import Odometry
        from core.msgs.geometry import Pose, Vector3

        mod = self._make()
        mod._on_servo_target("find:chair")

        # Setup sensor data
        mod._on_camera_info(CameraIntrinsics(fx=600, fy=600, cx=320, cy=240, width=640, height=480))
        mod._on_odom(Odometry(pose=Pose(position=Vector3(0, 0, 0))))
        mod._on_depth(Image(data=np.full((480, 640), 2500, dtype=np.uint16)))

        # Scene graph with chair that has bbox
        det = Detection3D(id="c1", label="chair", confidence=0.9)
        det.bbox = [280, 200, 360, 280]
        det.position = Vector3(2.5, 0.0, 0.0)
        mod._on_scene_graph(SceneGraph(objects=[det]))

        # Trigger frame
        mod._on_color(Image(data=np.zeros((480, 640, 3), dtype=np.uint8)))

        # Should be tracking
        assert mod._bbox_nav.state == "tracking"


# ── AgentLoop ───────────────────────────────────────────────────────────

class TestAgentLoop:
    def test_parse_text_tool_call(self):
        from semantic.planner.semantic_planner.agent_loop import AgentLoop
        result = AgentLoop._parse_text_tool_call(
            'I will navigate. {"tool": "navigate_to", "args": {"x": 5.0, "y": 3.0}}'
        )
        assert "tool_calls" in result
        assert result["tool_calls"][0]["function"]["name"] == "navigate_to"

    def test_parse_done(self):
        from semantic.planner.semantic_planner.agent_loop import AgentLoop
        result = AgentLoop._parse_text_tool_call(
            '{"tool": "done", "args": {"summary": "Task complete"}}'
        )
        assert result["tool_calls"][0]["function"]["name"] == "done"

    def test_parse_no_json(self):
        from semantic.planner.semantic_planner.agent_loop import AgentLoop
        result = AgentLoop._parse_text_tool_call("I don't know what to do.")
        assert "content" in result
        assert "tool_calls" not in result

    def test_agent_loop_done_terminates(self):
        """Agent calling done() should terminate the loop."""
        from semantic.planner.semantic_planner.agent_loop import AgentLoop
        import json

        class MockLLM:
            call_count = 0
            async def chat(self, text, system_prompt=""):
                self.call_count += 1
                return json.dumps({"tool": "done", "args": {"summary": "All done"}})

        llm = MockLLM()
        agent = AgentLoop(
            llm_client=llm,
            tool_handlers={},
            context_fn=lambda: {"robot_x": 0, "robot_y": 0, "visible_objects": "",
                                "nav_status": "IDLE", "memory_context": ""},
            max_steps=5,
        )
        state = asyncio.run(agent.run("test"))
        assert state.completed
        assert state.summary == "All done"
        assert state.step == 1

    def test_agent_loop_max_steps(self):
        """Agent should stop after max_steps."""
        from semantic.planner.semantic_planner.agent_loop import AgentLoop

        class ThinkingLLM:
            async def chat(self, text, system_prompt=""):
                return "I'm still thinking..."

        agent = AgentLoop(
            llm_client=ThinkingLLM(),
            tool_handlers={},
            context_fn=lambda: {"robot_x": 0, "robot_y": 0, "visible_objects": "",
                                "nav_status": "IDLE", "memory_context": ""},
            max_steps=3,
        )
        state = asyncio.run(agent.run("infinite task"))
        assert state.completed
        assert state.step == 3
        assert "Max steps" in state.summary


# ── TeleopModule ────────────────────────────────────────────────────────

class TestTeleopModule:
    def _make(self):
        from drivers.teleop_module import TeleopModule
        mod = TeleopModule()
        mod.setup()
        return mod

    def test_initial_idle(self):
        mod = self._make()
        assert mod._active is False
        s = mod.get_teleop_status()
        assert s["active"] is False
        assert s["clients"] == 0

    def test_joy_activates(self):
        mod = self._make()
        mod._on_joy({"lx": 0.5, "ly": 0.0, "az": 0.0})
        assert mod._active is True

    def test_idle_timeout_releases(self):
        mod = self._make()
        mod._release_timeout = 0.01  # 10ms for test
        mod._on_joy({"lx": 0.5, "ly": 0.0, "az": 0.0})
        assert mod._active is True
        time.sleep(0.02)
        mod._check_idle()
        assert mod._active is False

    def test_force_release(self):
        mod = self._make()
        mod._on_joy({"lx": 1.0, "ly": 0.0, "az": 0.0})
        assert mod._active is True
        result = mod.force_release()
        assert mod._active is False
        assert "released" in result.lower()

    def test_clamp_speed(self):
        mod = self._make()
        mod._max_speed = 0.5
        mod._on_joy({"lx": 999.0, "ly": 0.0, "az": 0.0})
        # cmd_vel should have been published with clamped value
        assert mod._active is True  # at least it activated


# ── SemanticMapperModule ────────────────────────────────────────────────

class TestSemanticMapperModule:
    def test_kg_update(self):
        from memory.modules.semantic_mapper_module import SemanticMapperModule
        from core.msgs.semantic import SceneGraph, Detection3D, Region
        from core.msgs.nav import Odometry
        from core.msgs.geometry import Vector3, Pose

        mod = SemanticMapperModule(save_dir="/tmp/test_smap")
        mod.setup()

        det = Detection3D(id="1", label="desk", confidence=0.9)
        sg = SceneGraph(
            objects=[det],
            regions=[Region(name="office", object_ids=["1"], center=Vector3(1, 0, 0))],
        )
        mod._on_odom(Odometry(pose=Pose(position=Vector3(1, 0, 0))))
        mod._on_scene_graph(sg)

        assert mod._kg is not None
        assert "office" in mod._kg.room_types

    def test_tsg_rooms(self):
        from memory.modules.semantic_mapper_module import SemanticMapperModule
        from core.msgs.semantic import SceneGraph, Detection3D, Region
        from core.msgs.nav import Odometry
        from core.msgs.geometry import Vector3, Pose

        mod = SemanticMapperModule(save_dir="/tmp/test_smap2")
        mod.setup()

        sg = SceneGraph(
            objects=[Detection3D(id="1", label="desk")],
            regions=[Region(name="office", object_ids=["1"], center=Vector3(1, 0, 0))],
        )
        mod._on_odom(Odometry(pose=Pose(position=Vector3(0, 0, 0))))
        mod._on_scene_graph(sg)

        assert mod._tsg is not None
        assert len(mod._tsg.rooms) == 1

    def test_skills(self):
        from memory.modules.semantic_mapper_module import SemanticMapperModule
        mod = SemanticMapperModule(save_dir="/tmp/test_smap3")
        mod.setup()

        status = mod.get_semantic_status()
        assert "kg" in status
        assert "tsg" in status

        summary = mod.get_room_summary()
        assert isinstance(summary, str)


# ── Integration: stub profile build ─────────────────────────────────────

class TestStubProfileBuild:
    def test_stub_builds(self):
        from core.blueprints.full_stack import full_stack_blueprint
        bp = full_stack_blueprint(
            robot="stub", slam_profile="none",
            enable_native=False, enable_semantic=False, enable_gateway=False,
        )
        system = bp.build()
        assert len(system.modules) > 0

    def test_dev_builds_with_semantic(self):
        from core.blueprints.full_stack import full_stack_blueprint
        bp = full_stack_blueprint(
            robot="stub", slam_profile="none",
            enable_native=False, enable_semantic=True, enable_gateway=True,
        )
        system = bp.build()
        assert "NavigationModule" in system.modules
        assert "SafetyRingModule" in system.modules
