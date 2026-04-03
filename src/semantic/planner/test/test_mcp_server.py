"""test_mcp_server.py — MCPServerModule unit tests

Tests the MCPServerModule in gateway/mcp_server.py (Module-First implementation).
Does not start an HTTP server, requires no ROS2, and requires no real LLM.

Coverage:
  - Port declarations (In/Out types)
  - setup() subscription registration
  - _on_odom / _on_sg / _on_safety / _on_mission state caching
  - on_system_modules() dynamic skill registration
  - _execute_tool() return structure for all built-in tools
  - health() summary format
"""

import json
import sys
import os

# Path setup
_here = os.path.dirname(os.path.abspath(__file__))
_repo = os.path.abspath(os.path.join(_here, "..", "..", "..", ".."))
_src = os.path.join(_repo, "src")
for _p in [_repo, _src]:
    if _p not in sys.path:
        sys.path.insert(0, _p)

import pytest
from unittest.mock import MagicMock

from core.stream import In, Out
from core.msgs.geometry import PoseStamped, Twist
from core.msgs.nav import Odometry
from core.msgs.semantic import SceneGraph, SafetyState
from gateway.mcp_server import MCPServerModule


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

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


# ---------------------------------------------------------------------------
# 1. Port declarations
# ---------------------------------------------------------------------------

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


# ---------------------------------------------------------------------------
# 2. State caching
# ---------------------------------------------------------------------------

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
        assert "objects" in data or "object_count" in data or data  # non-empty JSON

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


# ---------------------------------------------------------------------------
# 3. Dynamic skill registration (on_system_modules)
# ---------------------------------------------------------------------------

class TestMCPDynamicSkills:
    def test_skills_from_module_registered(self):
        """on_system_modules must register @skill methods into _skill_registry."""
        mod = _make_module()

        from core.module import Module, skill

        class FakeNav(Module, layer=5):
            @skill
            def navigate_to(self, x: float, y: float) -> str:
                """Go to (x, y)."""
                return f"navigating to {x},{y}"

        nav = FakeNav()
        mod.on_system_modules({"FakeNav": nav})

        assert "navigate_to" in mod._skill_registry
        assert len(mod._dynamic_tools) >= 1

    def test_dynamic_tool_descriptor(self):
        """Each dynamic tool descriptor must contain name, description, and inputSchema."""
        mod = _make_module()

        from core.module import Module, skill

        class FakeSkillMod(Module, layer=4):
            @skill
            def find_object(self, label: str) -> str:
                """Detect and navigate to an object."""
                return label

        mod.on_system_modules({"FakeSkillMod": FakeSkillMod()})
        tool = next(t for t in mod._dynamic_tools if t["name"] == "find_object")
        assert "description" in tool
        assert "inputSchema" in tool

    def test_empty_system_modules(self):
        """No modules → skill_registry empty, no crash."""
        mod = _make_module()
        mod.on_system_modules({})
        assert mod._skill_registry == {}
        assert mod._dynamic_tools == []


# ---------------------------------------------------------------------------
# 4. _execute_tool built-in tools
# ---------------------------------------------------------------------------

class TestMCPExecuteTool:
    def setup_method(self):
        self.mod = _make_module()
        self.mod._on_odom(_make_odom(1.0, 2.0))
        self.mod._on_sg(_make_scene_graph())

    def test_navigate_to_returns_status(self):
        result = self.mod._execute_tool("navigate_to", {"x": 3.0, "y": 4.0})
        data = json.loads(result)
        assert data.get("status") == "navigating"
        assert data["goal"] == [3.0, 4.0]

    def test_stop_returns_stopped(self):
        result = self.mod._execute_tool("stop", {})
        data = json.loads(result)
        assert data.get("status") == "stopped"

    def test_get_navigation_status_with_odom(self):
        result = self.mod._execute_tool("get_navigation_status", {})
        data = json.loads(result)
        assert "position" in data

    def test_get_robot_position(self):
        result = self.mod._execute_tool("get_robot_position", {})
        data = json.loads(result)
        assert "x" in data
        assert abs(data["x"] - 1.0) < 1e-6

    def test_get_scene_graph_returns_json(self):
        result = self.mod._execute_tool("get_scene_graph", {})
        parsed = json.loads(result)
        assert isinstance(parsed, (dict, list))

    def test_send_instruction_publishes(self):
        published = []
        self.mod.instruction.subscribe(lambda t: published.append(t))
        result = self.mod._execute_tool("send_instruction", {"text": "go to kitchen"})
        data = json.loads(result)
        assert data.get("status") == "sent"
        assert published == ["go to kitchen"]

    def test_navigate_to_publishes_goal_pose(self):
        published = []
        self.mod.goal_pose.subscribe(lambda p: published.append(p))
        self.mod._execute_tool("navigate_to", {"x": 5.0, "y": 6.0})
        assert len(published) == 1
        assert isinstance(published[0], PoseStamped)

    def test_set_mode(self):
        result = self.mod._execute_tool("set_mode", {"mode": "manual"})
        data = json.loads(result)
        assert "mode" in data

    def test_unknown_tool_returns_error(self):
        result = self.mod._execute_tool("nonexistent_tool_xyz", {})
        assert "error" in result.lower() or "unknown" in result.lower()


# ---------------------------------------------------------------------------
# 5. health()
# ---------------------------------------------------------------------------

class TestMCPHealth:
    def test_health_contains_mcp_section(self):
        mod = _make_module()
        h = mod.health()
        assert isinstance(h, dict)
        assert "mcp" in h or "port" in str(h) or "running" in str(h)
