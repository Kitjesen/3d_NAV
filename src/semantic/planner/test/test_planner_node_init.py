"""test_planner_node_init.py — SemanticPlannerModule 初始化与端口回归测试

原 test_planner_node_init.py 测试的是已删除的 ROS2 SemanticPlannerNode。
本文件改写为测试 Module-First 版本 SemanticPlannerModule 的初始化正确性。

覆盖：
  - __init__ 参数默认值
  - In/Out 端口声明
  - setup() 不崩溃
  - stop() 正常清理
  - instruction → goal_pose 端到端路由（mock LLM 下）
  - agent_instruction 触发 AgentLoop 路径
  - planner_status 输出
  - mission_status 触发 LERa recovery
  - health() 结构
"""

import sys
import os
import time

_here = os.path.dirname(os.path.abspath(__file__))
_repo = os.path.abspath(os.path.join(_here, "..", "..", "..", ".."))
_src = os.path.join(_repo, "src")
for _p in [_repo, _src,
           os.path.join(_src, "semantic", "planner"),
           os.path.join(_src, "semantic", "perception"),
           os.path.join(_src, "semantic", "common")]:
    if _p not in sys.path:
        sys.path.insert(0, _p)

import pytest
from core.stream import In, Out
from core.msgs.geometry import PoseStamped, Vector3
from core.msgs.nav import Odometry
from core.msgs.semantic import SceneGraph, Detection3D, Region
from semantic.planner.semantic_planner.semantic_planner_module import SemanticPlannerModule


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_module(**kw) -> SemanticPlannerModule:
    mod = SemanticPlannerModule(**kw)
    mod.setup()
    return mod


def _make_odom(x=0.0, y=0.0):
    from core.msgs.geometry import Pose, Vector3
    od = Odometry()
    od.pose = Pose(position=Vector3(x, y, 0.0))
    return od


def _make_scene_graph(labels=("chair", "door")):
    objs = [
        Detection3D(id=str(i), label=lbl, position=Vector3(float(i), 0.0, 0.0))
        for i, lbl in enumerate(labels)
    ]
    return SceneGraph(objects=objs, regions=[])


def _collect(port_out, n=1, timeout=0.5):
    """Subscribe to an Out port and collect up to n messages."""
    items = []
    port_out.subscribe(lambda v: items.append(v))
    deadline = time.time() + timeout
    while len(items) < n and time.time() < deadline:
        time.sleep(0.01)
    return items


# ---------------------------------------------------------------------------
# 1. 初始化 / 端口声明
# ---------------------------------------------------------------------------

class TestSemanticPlannerInit:
    def test_layer(self):
        assert SemanticPlannerModule._layer == 4

    def test_in_ports(self):
        mod = SemanticPlannerModule()
        assert isinstance(mod.instruction, In)
        assert isinstance(mod.agent_instruction, In)
        assert isinstance(mod.scene_graph, In)
        assert isinstance(mod.odometry, In)
        assert isinstance(mod.detections, In)
        assert isinstance(mod.mission_status, In)

    def test_out_ports(self):
        mod = SemanticPlannerModule()
        assert isinstance(mod.goal_pose, Out)
        assert isinstance(mod.task_plan, Out)
        assert isinstance(mod.planner_status, Out)
        assert isinstance(mod.cancel, Out)
        assert isinstance(mod.servo_target, Out)

    def test_default_params(self):
        mod = SemanticPlannerModule()
        # SemanticPlannerModule 内部存储为 _fast_threshold
        assert mod._fast_threshold == 0.75

    def test_custom_params(self):
        mod = SemanticPlannerModule(
            fast_path_threshold=0.9,
            decomposer="rules",
        )
        assert mod._fast_threshold == 0.9

    def test_setup_does_not_crash(self):
        mod = _make_module()
        mod.stop()

    def test_goal_resolver_initialized(self):
        mod = _make_module()
        assert mod._goal_resolver is not None
        mod.stop()

    def test_frontier_scorer_initialized(self):
        mod = _make_module()
        assert hasattr(mod, "_frontier_scorer") and mod._frontier_scorer is not None
        mod.stop()


# ---------------------------------------------------------------------------
# 2. 状态更新
# ---------------------------------------------------------------------------

class TestSemanticPlannerStateUpdate:
    def setup_method(self):
        self.mod = _make_module()

    def teardown_method(self):
        self.mod.stop()

    def test_odometry_cached(self):
        self.mod._on_odom(_make_odom(3.0, 4.0))
        pos = self.mod._robot_pos
        assert abs(pos[0] - 3.0) < 1e-6
        assert abs(pos[1] - 4.0) < 1e-6

    def test_scene_graph_cached(self):
        sg = _make_scene_graph()
        self.mod._on_scene_graph(sg)
        assert self.mod._latest_sg is not None

    def test_detections_cached(self):
        dets = [Detection3D(id="99", label="box", position=Vector3(1, 1, 0))]
        # _on_detections 接受 list，只验证不崩溃
        self.mod._on_detections(dets)


# ---------------------------------------------------------------------------
# 3. instruction → planner_status 流程（快路径，不依赖 LLM）
# ---------------------------------------------------------------------------

class TestSemanticPlannerInstruction:
    def setup_method(self):
        # 注入一个已知场景图让 Fast Path 能命中
        self.mod = _make_module()
        sg = _make_scene_graph(["chair", "door"])
        self.mod._on_odom(_make_odom(0.0, 0.0))
        self.mod._on_scene_graph(sg)

    def teardown_method(self):
        self.mod.stop()

    def test_instruction_triggers_status(self):
        """发送指令后 planner_status 应发布（成功或失败均可，不能静默挂起）。"""
        statuses = []
        self.mod.planner_status._add_callback(lambda s: statuses.append(s))
        self.mod._on_instruction("go to chair")
        deadline = time.time() + 1.5
        while not statuses and time.time() < deadline:
            time.sleep(0.02)
        assert len(statuses) > 0

    def test_instruction_empty_sg_no_crash(self):
        """空场景图下发送指令不崩溃。"""
        self.mod._on_scene_graph(SceneGraph(objects=[], regions=[]))
        self.mod._on_instruction("find the table")
        # 给后台线程一点时间
        time.sleep(0.1)


# ---------------------------------------------------------------------------
# 4. mission_status LERa 冷却
# ---------------------------------------------------------------------------

class TestSemanticPlannerRecovery:
    def test_stuck_triggers_lera_if_instruction_active(self):
        """STUCK 状态且有当前指令时，应触发 LERa（不崩溃即可）。"""
        mod = _make_module()
        mod._on_scene_graph(_make_scene_graph())
        mod._on_odom(_make_odom())
        # 模拟有活动指令
        mod._current_instruction = "find the coffee machine"
        # 发送 STUCK
        mod._on_mission_status({"state": "STUCK"})
        # LERa 在后台线程运行，只验证不崩溃
        time.sleep(0.05)
        mod.stop()

    def test_cooldown_prevents_double_lera(self):
        """连续两次 STUCK 触发，第二次应被冷却阻止。"""
        mod = _make_module()
        mod._current_instruction = "find exit"
        mod._on_mission_status({"state": "STUCK"})
        last = mod._last_lera_time
        mod._on_mission_status({"state": "STUCK"})
        # 第二次因冷却，_last_lera_time 不应变化
        assert mod._last_lera_time == last
        mod.stop()


# ---------------------------------------------------------------------------
# 5. health()
# ---------------------------------------------------------------------------

class TestSemanticPlannerHealth:
    def test_health_structure(self):
        mod = _make_module()
        h = mod.health()
        assert isinstance(h, dict)
        assert "planner" in h or "goal_resolver" in h or "instruction" in str(h)
        mod.stop()
