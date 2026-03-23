"""Tests for lingtu.core — Module / Stream / Transport / Blueprint 框架。

覆盖:
- Module 端口扫描 (In/Out from annotations)
- Out.publish → In._deliver 通过 _add_callback
- LocalTransport pub/sub
- Blueprint auto_wire 按 (name, type) 匹配
- 端到端: Source.Out → Sink.In 数据流
- autoconnect 合并多个 Blueprint
- Module.blueprint() 工厂方法
- 层级依赖校验
- SystemHandle 生命周期
- 消息统计 (msg_count, last_ts, latest)
"""

import time

import pytest

from src.core.stream import In, LocalTransport, Out
from src.core.module import Module
from src.core.blueprint import Blueprint, SystemHandle, autoconnect
from src.core.msgs.geometry import PoseStamped, Vector3
from src.core.msgs.semantic import SceneGraph, Detection3D, GoalResult


# ============================================================================
# Test Fixtures — 测试用模块定义
# ============================================================================

class SourceModule(Module, layer=1):
    """模拟感知源: 输出场景图和位姿。"""
    scene_graph: Out[SceneGraph]
    pose: Out[PoseStamped]


class SinkModule(Module, layer=4):
    """模拟规划消费者: 接收场景图和位姿。"""
    scene_graph: In[SceneGraph]
    pose: In[PoseStamped]


class DetectorModule(Module, layer=3):
    """模拟检测器: 输入位姿 → 输出检测结果和场景图。"""
    pose: In[PoseStamped]
    detections: Out[Detection3D]
    scene_graph: Out[SceneGraph]


class PlannerModule(Module, layer=5):
    """模拟规划器: 输入场景图 → 输出目标。"""
    scene_graph: In[SceneGraph]
    goal: Out[GoalResult]


class NoLayerModule(Module):
    """无层级标签的模块。"""
    data: Out[Vector3]


class SetupTrackModule(Module, layer=2):
    """跟踪 setup/start/stop 调用的模块。"""
    output: Out[Vector3]
    input_port: In[Vector3]

    def __init__(self, **config):
        super().__init__(**config)
        self.setup_called = False
        self.start_called = False
        self.stop_called = False

    def setup(self):
        self.setup_called = True
        self.input_port.subscribe(self._on_data)

    def start(self):
        super().start()
        self.start_called = True

    def stop(self):
        super().stop()
        self.stop_called = True

    def _on_data(self, msg: Vector3):
        # Echo: 收到数据后发布
        self.output.publish(msg)


# ============================================================================
# TestOut — 输出端口
# ============================================================================

class TestOut:
    def test_basic_publish(self):
        """Out.publish 调用已注册的回调。"""
        port = Out("test", SceneGraph)
        received = []
        port._add_callback(received.append)

        sg = SceneGraph()
        port.publish(sg)

        assert len(received) == 1
        assert received[0] is sg  # 零拷贝

    def test_multiple_callbacks(self):
        """Out 支持多个回调 (扇出)。"""
        port = Out("test", int)
        r1, r2 = [], []
        port._add_callback(r1.append)
        port._add_callback(r2.append)

        port.publish(42)

        assert r1 == [42]
        assert r2 == [42]

    def test_msg_count_and_ts(self):
        """发布后 msg_count 递增，last_ts 更新。"""
        port = Out("counter", int)
        assert port.msg_count == 0
        assert port.last_ts == 0.0

        port.publish(1)
        assert port.msg_count == 1
        assert port.last_ts > 0

        ts1 = port.last_ts
        port.publish(2)
        assert port.msg_count == 2
        assert port.last_ts >= ts1

    def test_remove_callback(self):
        """_remove_callback 后不再收到消息。"""
        port = Out("test", int)
        received = []
        port._add_callback(received.append)
        port.publish(1)
        assert len(received) == 1

        port._remove_callback(received.append)
        port.publish(2)
        assert len(received) == 1  # 没有新增

    def test_properties(self):
        """Out 属性正确。"""
        port = Out("my_port", SceneGraph)
        assert port.name == "my_port"
        assert port.msg_type is SceneGraph
        assert port.callback_count == 0

        port._add_callback(lambda x: None)
        assert port.callback_count == 1

    def test_callback_exception_doesnt_stop_delivery(self):
        """一个回调异常不影响其他回调。"""
        port = Out("test", int)
        received = []

        def bad_cb(msg):
            raise RuntimeError("boom")

        port._add_callback(bad_cb)
        port._add_callback(received.append)

        port.publish(99)
        assert received == [99]


# ============================================================================
# TestIn — 输入端口
# ============================================================================

class TestIn:
    def test_deliver(self):
        """_deliver 调用已注册的回调。"""
        port = In("test", SceneGraph)
        received = []
        port.subscribe(received.append)

        sg = SceneGraph()
        port._deliver(sg)

        assert len(received) == 1
        assert received[0] is sg

    def test_latest(self):
        """latest 保存最新消息。"""
        port = In("test", int)
        assert port.latest is None

        port._deliver(10)
        assert port.latest == 10

        port._deliver(20)
        assert port.latest == 20

    def test_msg_count(self):
        """_deliver 递增 msg_count。"""
        port = In("test", int)
        assert port.msg_count == 0

        port._deliver(1)
        port._deliver(2)
        assert port.msg_count == 2

    def test_connected_property(self):
        """connected 反映是否有注册回调。"""
        port = In("test", int)
        assert not port.connected

        port.subscribe(lambda x: None)
        assert port.connected

    def test_deliver_without_subscriber(self):
        """无订阅者时 _deliver 不报错，仍更新 latest。"""
        port = In("test", int)
        port._deliver(42)  # 不应报错
        assert port.latest == 42
        assert port.msg_count == 1

    def test_callback_exception_doesnt_crash(self):
        """回调异常不影响 In 状态更新。"""
        port = In("test", int)

        def bad_cb(msg):
            raise ValueError("oops")

        port.subscribe(bad_cb)
        port._deliver(5)
        assert port.msg_count == 1
        assert port.latest == 5


# ============================================================================
# TestOutToIn — Out → In 直连
# ============================================================================

class TestOutToIn:
    def test_direct_connection(self):
        """Out._add_callback(In._deliver) 实现直连。"""
        out = Out("scene_graph", SceneGraph)
        inp = In("scene_graph", SceneGraph)

        out._add_callback(inp._deliver)

        sg = SceneGraph(objects=[Detection3D(label="chair")])
        out.publish(sg)

        assert inp.latest is sg
        assert inp.msg_count == 1
        assert out.msg_count == 1

    def test_fan_out(self):
        """一个 Out 连接多个 In。"""
        out = Out("pose", PoseStamped)
        in1 = In("pose", PoseStamped)
        in2 = In("pose", PoseStamped)

        out._add_callback(in1._deliver)
        out._add_callback(in2._deliver)

        p = PoseStamped()
        out.publish(p)

        assert in1.latest is p
        assert in2.latest is p


# ============================================================================
# TestLocalTransport
# ============================================================================

class TestLocalTransport:
    def test_pub_sub(self):
        """基本发布/订阅。"""
        t = LocalTransport()
        received = []
        t.subscribe("topic_a", received.append)

        t.publish("topic_a", 42)
        assert received == [42]

    def test_multiple_subscribers(self):
        """同一 topic 多个订阅者。"""
        t = LocalTransport()
        r1, r2 = [], []
        t.subscribe("t", r1.append)
        t.subscribe("t", r2.append)

        t.publish("t", "hello")
        assert r1 == ["hello"]
        assert r2 == ["hello"]

    def test_isolated_topics(self):
        """不同 topic 互不影响。"""
        t = LocalTransport()
        ra, rb = [], []
        t.subscribe("a", ra.append)
        t.subscribe("b", rb.append)

        t.publish("a", 1)
        t.publish("b", 2)

        assert ra == [1]
        assert rb == [2]

    def test_no_subscriber(self):
        """无订阅者发布不报错。"""
        t = LocalTransport()
        t.publish("empty", "data")  # 不应报错

    def test_close_clears_all(self):
        """close 清空所有订阅。"""
        t = LocalTransport()
        t.subscribe("x", lambda m: None)
        assert t.subscriber_count("x") == 1

        t.close()
        assert t.subscriber_count("x") == 0
        assert t.topics == []

    def test_unsubscribe(self):
        """取消订阅后不再收到消息。"""
        t = LocalTransport()
        received = []
        t.subscribe("t", received.append)
        t.publish("t", 1)
        assert len(received) == 1

        t.unsubscribe("t", received.append)
        t.publish("t", 2)
        assert len(received) == 1

    def test_subscriber_count(self):
        """subscriber_count 正确统计。"""
        t = LocalTransport()
        assert t.subscriber_count("t") == 0

        cb1 = lambda m: None
        cb2 = lambda m: None
        t.subscribe("t", cb1)
        t.subscribe("t", cb2)
        assert t.subscriber_count("t") == 2

    def test_topics_list(self):
        """topics 返回活跃 topic。"""
        t = LocalTransport()
        t.subscribe("a", lambda m: None)
        t.subscribe("b", lambda m: None)
        assert set(t.topics) == {"a", "b"}

    def test_transport_protocol(self):
        """LocalTransport 满足 Transport 协议。"""
        from src.core.stream import Transport
        t = LocalTransport()
        assert isinstance(t, Transport)

    def test_callback_exception_isolated(self):
        """一个回调异常不影响其他回调。"""
        t = LocalTransport()
        received = []

        def bad(msg):
            raise RuntimeError("fail")

        t.subscribe("t", bad)
        t.subscribe("t", received.append)

        t.publish("t", "ok")
        assert received == ["ok"]


# ============================================================================
# TestOutWithTransport — Out 绑定 Transport
# ============================================================================

class TestOutWithTransport:
    def test_bind_transport(self):
        """Out 绑定 Transport 后，publish 同时发送到传输层。"""
        transport = LocalTransport()
        out = Out("scene_graph", SceneGraph)
        out._bind_transport(transport, "/nav/scene_graph")

        received_local = []
        received_transport = []
        out._add_callback(received_local.append)
        transport.subscribe("/nav/scene_graph", received_transport.append)

        sg = SceneGraph()
        out.publish(sg)

        assert len(received_local) == 1
        assert len(received_transport) == 1
        assert received_local[0] is sg
        assert received_transport[0] is sg


# ============================================================================
# TestModule — 模块端口扫描
# ============================================================================

class TestModule:
    def test_port_scanning_out(self):
        """Module 自动扫描 Out[T] 标注。"""
        mod = SourceModule()
        assert "scene_graph" in mod.ports_out
        assert "pose" in mod.ports_out
        assert mod.ports_out["scene_graph"].msg_type is SceneGraph
        assert mod.ports_out["pose"].msg_type is PoseStamped

    def test_port_scanning_in(self):
        """Module 自动扫描 In[T] 标注。"""
        mod = SinkModule()
        assert "scene_graph" in mod.ports_in
        assert "pose" in mod.ports_in
        assert mod.ports_in["scene_graph"].msg_type is SceneGraph
        assert mod.ports_in["pose"].msg_type is PoseStamped

    def test_mixed_ports(self):
        """Module 同时有 In 和 Out。"""
        mod = DetectorModule()
        assert "pose" in mod.ports_in
        assert "detections" in mod.ports_out
        assert "scene_graph" in mod.ports_out

    def test_port_instances_are_attrs(self):
        """端口实例可通过属性访问。"""
        mod = SourceModule()
        assert isinstance(mod.scene_graph, Out)
        assert isinstance(mod.pose, Out)
        assert mod.scene_graph.name == "scene_graph"

    def test_layer_attribute(self):
        """layer 参数正确设置。"""
        assert SourceModule._layer == 1
        assert SinkModule._layer == 4
        assert NoLayerModule._layer is None

        mod = SourceModule()
        assert mod.layer == 1

    def test_config_passthrough(self):
        """配置参数正确传递。"""
        mod = SourceModule(camera_id=0, fps=30)
        assert mod._config == {"camera_id": 0, "fps": 30}

    def test_lifecycle(self):
        """Module 生命周期: setup → start → stop。"""
        mod = SourceModule()
        assert not mod.running

        mod.setup()
        mod.start()
        assert mod.running

        mod.stop()
        assert not mod.running

    def test_all_ports(self):
        """all_ports 返回所有端口。"""
        mod = DetectorModule()
        all_p = mod.all_ports
        assert "pose" in all_p
        assert "detections" in all_p
        assert "scene_graph" in all_p

    def test_port_summary(self):
        """port_summary 返回结构化信息。"""
        mod = SourceModule()
        summary = mod.port_summary()
        assert summary["module"] == "SourceModule"
        assert summary["layer"] == 1
        assert "scene_graph" in summary["ports_out"]
        assert summary["ports_out"]["scene_graph"]["type"] == "SceneGraph"

    def test_repr(self):
        """Module repr 包含端口名。"""
        mod = SourceModule()
        r = repr(mod)
        assert "SourceModule" in r
        assert "scene_graph" in r

    def test_blueprint_factory(self):
        """Module.blueprint() 返回 Blueprint。"""
        bp = SourceModule.blueprint(camera_id=0)
        assert isinstance(bp, Blueprint)


# ============================================================================
# TestBlueprint — 蓝图编排
# ============================================================================

class TestBlueprint:
    def test_add_and_build(self):
        """add + build 创建模块实例。"""
        system = Blueprint().add(SourceModule).build()
        assert "SourceModule" in system.modules

    def test_duplicate_module_name_raises(self):
        """重复模块名报错。"""
        with pytest.raises(ValueError, match="already contains"):
            Blueprint().add(SourceModule).add(SourceModule)

    def test_alias(self):
        """alias 允许同类型多实例。"""
        system = (
            Blueprint()
            .add(SourceModule, alias="source_1")
            .add(SourceModule, alias="source_2")
            .build()
        )
        assert "source_1" in system.modules
        assert "source_2" in system.modules

    def test_explicit_wire(self):
        """wire 显式连接 Out → In。"""
        system = (
            Blueprint()
            .add(SourceModule)
            .add(SinkModule)
            .wire("SourceModule", "scene_graph", "SinkModule", "scene_graph")
            .build()
        )

        src = system.get_module("SourceModule")
        sink = system.get_module("SinkModule")

        sg = SceneGraph(objects=[Detection3D(label="table")])
        src.scene_graph.publish(sg)

        assert sink.scene_graph.latest is sg

    def test_wire_unknown_module_raises(self):
        """wire 引用不存在的模块报错。"""
        bp = Blueprint().add(SourceModule)
        with pytest.raises(ValueError, match="unknown"):
            bp.wire("SourceModule", "scene_graph", "NonExistent", "scene_graph").build()

    def test_wire_unknown_port_raises(self):
        """wire 引用不存在的端口报错。"""
        bp = Blueprint().add(SourceModule).add(SinkModule)
        with pytest.raises(ValueError, match="no Out port"):
            bp.wire("SourceModule", "nonexistent", "SinkModule", "scene_graph").build()

    def test_wire_type_mismatch_raises(self):
        """wire 类型不匹配报错。"""
        bp = Blueprint().add(SourceModule).add(SinkModule)
        with pytest.raises(TypeError, match="type mismatch"):
            bp.wire("SourceModule", "scene_graph", "SinkModule", "pose").build()

    def test_auto_wire_by_name_and_type(self):
        """auto_wire 按 (name, msg_type) 自动匹配。"""
        system = (
            Blueprint()
            .add(SourceModule)
            .add(SinkModule)
            .auto_wire()
            .build()
        )

        src = system.get_module("SourceModule")
        sink = system.get_module("SinkModule")

        # scene_graph: Out[SceneGraph] ↔ In[SceneGraph] — 同名同类型，应连接
        sg = SceneGraph()
        src.scene_graph.publish(sg)
        assert sink.scene_graph.latest is sg

        # pose: Out[PoseStamped] ↔ In[PoseStamped] — 同名同类型，应连接
        p = PoseStamped()
        src.pose.publish(p)
        assert sink.pose.latest is p

    def test_auto_wire_no_self_connect(self):
        """auto_wire 不会自连接 (同一模块的 Out→In)。"""
        system = (
            Blueprint()
            .add(DetectorModule)
            .auto_wire()
            .build()
        )
        det = system.get_module("DetectorModule")
        # DetectorModule 有 In[PoseStamped] 和 Out[SceneGraph]
        # 名称不同或同模块，不应自连接
        det.scene_graph.publish(SceneGraph())
        # 没有其他模块的 In[SceneGraph]，所以不会被连到自己

    def test_auto_wire_skips_explicit(self):
        """auto_wire 跳过已有显式连接的 In。"""
        received = []

        system = (
            Blueprint()
            .add(SourceModule, alias="src1")
            .add(SourceModule, alias="src2")
            .add(SinkModule)
            .wire("src1", "scene_graph", "SinkModule", "scene_graph")
            .auto_wire()
            .build()
        )

        sink = system.get_module("SinkModule")
        src1 = system.get_module("src1")
        src2 = system.get_module("src2")

        # scene_graph 已显式连到 src1
        sg1 = SceneGraph(objects=[Detection3D(label="from_src1")])
        src1.scene_graph.publish(sg1)
        assert sink.scene_graph.latest is sg1

    def test_build_with_custom_transport(self):
        """build 接受自定义 Transport。"""
        transport = LocalTransport()
        system = Blueprint().add(SourceModule).build(transport=transport)

        src = system.get_module("SourceModule")
        # Transport 应被绑定到 Out 端口
        assert src.scene_graph._transport is transport

    def test_connections_recorded(self):
        """build 后 connections 记录所有连接。"""
        system = (
            Blueprint()
            .add(SourceModule)
            .add(SinkModule)
            .wire("SourceModule", "scene_graph", "SinkModule", "scene_graph")
            .build()
        )
        conns = system.connections
        assert len(conns) >= 1
        assert ("SourceModule", "scene_graph", "SinkModule", "scene_graph") in conns

    def test_merge(self):
        """merge 合并两个 Blueprint。"""
        bp1 = Blueprint().add(SourceModule)
        bp2 = Blueprint().add(SinkModule)
        bp1.merge(bp2)

        system = bp1.auto_wire().build()
        assert "SourceModule" in system.modules
        assert "SinkModule" in system.modules

    def test_merge_conflict_raises(self):
        """merge 名称冲突报错。"""
        bp1 = Blueprint().add(SourceModule)
        bp2 = Blueprint().add(SourceModule)
        with pytest.raises(ValueError, match="exists in both"):
            bp1.merge(bp2)


# ============================================================================
# TestAutoconnect — 多蓝图合并
# ============================================================================

class TestAutoconnect:
    def test_basic_autoconnect(self):
        """autoconnect 合并并自动连接。"""
        bp = autoconnect(
            SourceModule.blueprint(),
            SinkModule.blueprint(),
        )
        system = bp.build()

        src = system.get_module("SourceModule")
        sink = system.get_module("SinkModule")

        sg = SceneGraph(objects=[Detection3D(label="desk")])
        src.scene_graph.publish(sg)
        assert sink.scene_graph.latest is sg

    def test_autoconnect_three_modules(self):
        """autoconnect 三个模块的链路。"""
        bp = autoconnect(
            SourceModule.blueprint(),
            DetectorModule.blueprint(),
            PlannerModule.blueprint(),
        )
        system = bp.build()

        src = system.get_module("SourceModule")
        planner = system.get_module("PlannerModule")
        detector = system.get_module("DetectorModule")

        # SourceModule.pose → DetectorModule.pose (同名同类型)
        p = PoseStamped()
        src.pose.publish(p)
        assert detector.pose.latest is p


# ============================================================================
# TestSystemHandle — 运行时管理
# ============================================================================

class TestSystemHandle:
    def test_start_stop(self):
        """start/stop 调用所有模块的生命周期方法。"""
        system = (
            Blueprint()
            .add(SetupTrackModule, alias="tracker")
            .build()
        )
        mod = system.get_module("tracker")

        assert not mod.setup_called
        assert not mod.start_called

        system.start()
        assert mod.setup_called
        assert mod.start_called
        assert system.started

        system.stop()
        assert mod.stop_called
        assert not system.started

    def test_double_start_no_error(self):
        """重复 start 不报错。"""
        system = Blueprint().add(SourceModule).build()
        system.start()
        system.start()  # 不应报错
        system.stop()

    def test_stop_without_start(self):
        """未 start 直接 stop 不报错。"""
        system = Blueprint().add(SourceModule).build()
        system.stop()  # 不应报错

    def test_get_module(self):
        """get_module 按名称获取。"""
        system = Blueprint().add(SourceModule).build()
        mod = system.get_module("SourceModule")
        assert isinstance(mod, SourceModule)

    def test_get_module_unknown_raises(self):
        """get_module 未知名称报错。"""
        system = Blueprint().add(SourceModule).build()
        with pytest.raises(KeyError, match="Unknown module"):
            system.get_module("NonExistent")

    def test_health(self):
        """health 返回系统健康信息。"""
        system = (
            Blueprint()
            .add(SourceModule)
            .add(SinkModule)
            .auto_wire()
            .build()
        )
        system.start()

        src = system.get_module("SourceModule")
        src.scene_graph.publish(SceneGraph())

        h = system.health()
        assert h["started"] is True
        assert h["module_count"] == 2
        assert h["connection_count"] >= 1
        assert h["total_messages_out"] >= 1
        assert "SourceModule" in h["modules"]

        system.stop()

    def test_startup_order_respects_deps(self):
        """启动顺序: 上游模块 (低层) 先于下游模块 (高层)。"""
        system = (
            Blueprint()
            .add(SinkModule)     # L4
            .add(SourceModule)   # L1
            .auto_wire()
            .build()
        )
        order = system.health()["startup_order"]
        src_idx = order.index("SourceModule")
        sink_idx = order.index("SinkModule")
        assert src_idx < sink_idx  # L1 先于 L4

    def test_repr(self):
        """SystemHandle repr。"""
        system = Blueprint().add(SourceModule).build()
        r = repr(system)
        assert "stopped" in r
        system.start()
        r = repr(system)
        assert "running" in r
        system.stop()


# ============================================================================
# TestEndToEnd — 端到端数据流
# ============================================================================

class TestEndToEnd:
    def test_source_to_sink_full_pipeline(self):
        """完整流水线: Source → Sink，使用真实消息类型。"""
        system = (
            Blueprint()
            .add(SourceModule)
            .add(SinkModule)
            .auto_wire()
            .build()
        )
        system.start()

        src = system.get_module("SourceModule")
        sink = system.get_module("SinkModule")

        # 发送带内容的 SceneGraph
        sg = SceneGraph(
            objects=[
                Detection3D(id="obj_1", label="chair", confidence=0.9,
                            position=Vector3(1.0, 2.0, 0.5)),
                Detection3D(id="obj_2", label="table", confidence=0.85,
                            position=Vector3(3.0, 4.0, 0.7)),
            ],
            frame_id="map",
        )
        src.scene_graph.publish(sg)

        # 验证到达
        assert sink.scene_graph.latest is sg
        assert sink.scene_graph.latest.objects[0].label == "chair"
        assert len(sink.scene_graph.latest.objects) == 2

        # 发送 PoseStamped
        pose = PoseStamped(frame_id="map")
        src.pose.publish(pose)
        assert sink.pose.latest is pose

        system.stop()

    def test_echo_module_data_flow(self):
        """SetupTrackModule 收到数据后 echo 输出。"""
        class VecSource(Module):
            output: Out[Vector3]

        class VecSink(Module):
            input_port: In[Vector3]

        system = (
            Blueprint()
            .add(VecSource, alias="src")
            .add(SetupTrackModule, alias="echo")
            .add(VecSink, alias="sink")
            .wire("src", "output", "echo", "input_port")
            .wire("echo", "output", "sink", "input_port")
            .build()
        )
        system.start()

        src = system.get_module("src")
        sink = system.get_module("sink")

        v = Vector3(1.0, 2.0, 3.0)
        src.output.publish(v)

        # echo 应该将消息转发到 sink
        assert sink.input_port.latest is v

        system.stop()

    def test_multi_hop_pipeline(self):
        """多跳流水线: Source → Detector → Planner (混合 auto_wire + 显式 wire)。

        SourceModule 和 DetectorModule 都有 Out[SceneGraph] 名为 scene_graph，
        auto_wire 会因歧义跳过 PlannerModule.scene_graph，因此需要显式连接。
        """
        system = (
            Blueprint()
            .add(SourceModule)
            .add(DetectorModule)
            .add(PlannerModule)
            # 显式: Detector.scene_graph → Planner.scene_graph (解决歧义)
            .wire("DetectorModule", "scene_graph", "PlannerModule", "scene_graph")
            # auto_wire 处理无歧义的: Source.pose → Detector.pose
            .auto_wire()
            .build()
        )
        system.start()

        src = system.get_module("SourceModule")
        detector = system.get_module("DetectorModule")
        planner = system.get_module("PlannerModule")

        # Source.pose → Detector.pose (auto-wired, 唯一匹配)
        p = PoseStamped(frame_id="odom")
        src.pose.publish(p)
        assert detector.pose.latest is p

        # Detector.scene_graph → Planner.scene_graph (显式连接)
        sg = SceneGraph(objects=[Detection3D(label="door")])
        detector.scene_graph.publish(sg)
        assert planner.scene_graph.latest is sg

        system.stop()

    def test_msg_counts_accumulate(self):
        """消息计数在整个流水线中正确累积。"""
        system = (
            Blueprint()
            .add(SourceModule)
            .add(SinkModule)
            .auto_wire()
            .build()
        )
        system.start()

        src = system.get_module("SourceModule")
        sink = system.get_module("SinkModule")

        for i in range(10):
            src.scene_graph.publish(SceneGraph())

        assert src.scene_graph.msg_count == 10
        assert sink.scene_graph.msg_count == 10

        system.stop()


# ============================================================================
# TestLayerDeps — 层级依赖
# ============================================================================

class TestLayerDeps:
    def test_no_layer_module_no_violation(self):
        """无层级标签模块不触发校验。"""
        system = (
            Blueprint()
            .add(NoLayerModule)
            .add(SinkModule)
            .auto_wire()
            .build()
        )
        h = system.health()
        assert h["layer_violations"] == []

    def test_normal_layer_flow(self):
        """正常层级流 (L1 → L4) 无违规。"""
        system = (
            Blueprint()
            .add(SourceModule)    # L1
            .add(SinkModule)      # L4
            .auto_wire()
            .build()
        )
        h = system.health()
        assert h["layer_violations"] == []
