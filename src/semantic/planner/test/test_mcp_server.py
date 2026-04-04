"""test_mcp_server.py — MCPServerModule unit tests

Tests gateway/mcp_server.py (Module-First implementation).
Does not start an HTTP server; no ROS2 or LLM required.

Coverage:
  - Port declarations (In/Out types)
  - setup() subscription registration
  - Telemetry callbacks (_on_odom / _on_sg / _on_safety / _on_mission)
  - on_system_modules() dynamic @skill registration
  - Calling tools via _tool_registry (same dispatch as JSON-RPC tools/call)
  - health() summary
"""

import json
import os
import sys

# Path setup
_here = os.path.dirname(os.path.abspath(__file__))
_repo = os.path.abspath(os.path.join(_here, "..", "..", "..", ".."))
_src = os.path.join(_repo, "src")
for _p in [_repo, _src]:
    if _p not in sys.path:
        sys.path.insert(0, _p)

from core.stream import In, Out
from core.msgs.geometry import PoseStamped
from core.msgs.nav import Odometry
from core.msgs.semantic import SceneGraph, SafetyState
from gateway.mcp_server import MCPServerModule


def _make_module(**kw) -> MCPServerModule:
    mod = MCPServerModule(**kw)
    mod.setup()
    return mod


def _make_odom(x=1.0, y=2.0, z=0.0):
    from core.msgs.geometry import Pose, Vector3

    od = Odometry()
    od.pose = Pose(position=Vector3(x, y, z))
    return od


def _make_scene_graph():
    from core.msgs.semantic import Detection3D, Region
    from core.msgs.geometry import Vector3

    return SceneGraph(
        objects=[Detection3D(id="1", label="chair", position=Vector3(1, 2, 0))],
        regions=[Region(name="office", object_ids=["1"], center=Vector3(1, 2, 0))],
    )


class TestMCPServerModulePorts:
    def test_layer(self):
        assert MCPServerModule._layer == 6

    def test_in_ports(self):
        mod = MCPServerModule()
        assert isinstance(mod.odometry, In)
        assert isinstance(mod.scene_graph, In)
        assert isinstance(mod.safety_state, In)
        assert isinstance(mod.mission_status, In)

    def test_out_ports(self):
        mod = MCPServerModule()
        assert isinstance(mod.goal_pose, Out)
        assert isinstance(mod.cmd_vel, Out)
        assert isinstance(mod.stop_cmd, Out)
        assert isinstance(mod.instruction, Out)
        assert isinstance(mod.mode_cmd, Out)

    def test_default_port(self):
        mod = MCPServerModule()
        assert mod._port == 8090

    def test_custom_port(self):
        mod = MCPServerModule(port=9000)
        assert mod._port == 9000


class TestMCPServerStateCache:
    def test_odom_cached(self):
        mod = _make_module()
        mod._on_odom(_make_odom(3.5, 7.2))
        assert mod._odom is not None
        assert abs(mod._odom["x"] - 3.5) < 1e-6
        assert abs(mod._odom["y"] - 7.2) < 1e-6

    def test_scene_graph_cached(self):
        mod = _make_module()
        sg = _make_scene_graph()
        mod._on_sg(sg)
        data = json.loads(mod._sg_json)
        assert "objects" in data or "object_count" in data or data

    def test_safety_state_cached(self):
        mod = _make_module()
        ss = SafetyState(level="safe", issues=[])
        mod._on_safety(ss)
        assert mod._safety is not None

    def test_mission_status_cached(self):
        mod = _make_module()
        mod._on_mission({"state": "NAVIGATING", "goal": [1, 2]})
        assert mod._mission is not None
        assert mod._mission["state"] == "NAVIGATING"


class TestMCPDynamicSkills:
    def test_skills_from_module_registered(self):
        mod = _make_module()

        from core.module import Module, skill

        class FakeNav(Module, layer=5):
            @skill
            def navigate_to(self, x: float, y: float) -> str:
                """Go to (x, y)."""
                return json.dumps({"status": "ok", "x": x, "y": y})

        mod.on_system_modules({"FakeNav": FakeNav()})

        assert "navigate_to" in mod._tool_registry
        assert any(t["name"] == "navigate_to" for t in mod._tool_list)

    def test_dynamic_tool_descriptor(self):
        mod = _make_module()

        from core.module import Module, skill

        class FakeSkillMod(Module, layer=4):
            @skill
            def find_object(self, label: str) -> str:
                """Detect and navigate to an object."""
                return label

        mod.on_system_modules({"FakeSkillMod": FakeSkillMod()})
        tool = next(t for t in mod._tool_list if t["name"] == "find_object")
        assert "description" in tool
        assert "inputSchema" in tool

    def test_empty_system_modules(self):
        mod = _make_module()
        mod.on_system_modules({})
        assert mod._tool_registry == {}
        assert mod._tool_list == []


class TestMCPToolCall:
    def setup_method(self):
        self.mod = _make_module()
        self.mod.on_system_modules({"MCPServerModule": self.mod})
        self.mod._on_odom(_make_odom(1.0, 2.0))
        self.mod._on_sg(_make_scene_graph())

    def test_stop_via_registry(self):
        stops = []
        self.mod.stop_cmd._add_callback(stops.append)
        fn = self.mod._tool_registry["stop"]
        data = json.loads(fn())
        assert data.get("status") == "stopped"
        assert stops == [2]

    def test_get_robot_position_via_registry(self):
        fn = self.mod._tool_registry["get_robot_position"]
        data = json.loads(fn())
        assert abs(data["x"] - 1.0) < 1e-6

    def test_get_scene_graph_via_registry(self):
        fn = self.mod._tool_registry["get_scene_graph"]
        parsed = json.loads(fn())
        assert isinstance(parsed, (dict, list))

    def test_send_instruction_publishes_via_registry(self):
        published = []
        self.mod.instruction.subscribe(lambda t: published.append(t))
        fn = self.mod._tool_registry["send_instruction"]
        data = json.loads(fn(text="go to kitchen"))
        assert data.get("status") == "sent"
        assert published == ["go to kitchen"]

    def test_set_mode_via_registry(self):
        fn = self.mod._tool_registry["set_mode"]
        data = json.loads(fn(mode="manual"))
        assert data.get("mode") == "manual"

    def test_unknown_tool_not_in_registry(self):
        assert self.mod._tool_registry.get("nonexistent_tool_xyz") is None


class TestMCPHealth:
    def test_health_contains_mcp_section(self):
        mod = _make_module()
        h = mod.health()
        assert isinstance(h, dict)
        assert "mcp" in h
