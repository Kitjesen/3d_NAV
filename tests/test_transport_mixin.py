"""
Transport Mixin + Agent TaggedLocations 集成测试

测试范围:
  1. TransportMixin 初始化和参数处理
  2. SHM Publisher/Subscriber 基本功能
  3. Agent → TaggedLocations 三层降级
  4. Sim CLI 参数解析
"""
import json
import os
import struct
import sys
import tempfile
import threading
import time
import unittest

# 确保 repo root 在 path 中
_repo = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if _repo not in sys.path:
    sys.path.insert(0, _repo)


# ============================================================
# 1. Transport 核心测试
# ============================================================

class TestTransportCore(unittest.TestCase):
    """Transport 抽象层基本测试。"""

    def test_transport_strategy_enum(self):
        from transport.core import TransportStrategy
        self.assertEqual(TransportStrategy.DDS.value, "dds")
        self.assertEqual(TransportStrategy.SHM.value, "shm")
        self.assertEqual(TransportStrategy.DUAL.value, "dual")
        self.assertEqual(TransportStrategy.AUTO.value, "auto")

    def test_topic_config_defaults(self):
        from transport.core import TopicConfig, TransportStrategy
        tc = TopicConfig(name="/test/topic")
        self.assertEqual(tc.name, "/test/topic")
        self.assertIsNone(tc.msg_type)
        self.assertEqual(tc.strategy, TransportStrategy.DDS)
        self.assertEqual(tc.qos_depth, 10)
        self.assertFalse(tc.reliable)

    def test_shm_name_sanitization(self):
        from transport.shm_transport import _shm_name
        self.assertEqual(_shm_name("/camera/color/image_raw"), "lingtu_camera_color_image_raw")
        self.assertEqual(_shm_name("/nav/odometry"), "lingtu_nav_odometry")
        self.assertEqual(_shm_name("simple"), "lingtu_simple")


class TestSHMTransport(unittest.TestCase):
    """SHM 传输层功能测试。"""

    def test_shm_publish_subscribe_bytes(self):
        """SHM 发布 bytes，轮询订阅者收到。"""
        from transport.shm_transport import SHMTransport
        from transport.core import TopicConfig

        transport = SHMTransport()
        topic = TopicConfig(name="/test/shm_bytes", buffer_size=1024)

        received = []
        def callback(data, ts):
            received.append(data)

        pub = transport.create_publisher(topic)
        sub = transport.create_subscriber(topic, callback)

        # 发布
        payload = b"hello shm transport"
        pub.publish(payload)

        # 等待轮询线程读到
        time.sleep(0.05)

        transport.close()

        self.assertGreaterEqual(len(received), 1)
        self.assertEqual(received[0], payload)

    def test_shm_publish_numpy(self):
        """SHM 发布 numpy array。"""
        try:
            import numpy as np
        except ImportError:
            self.skipTest("numpy not available")

        from transport.shm_transport import SHMTransport
        from transport.core import TopicConfig

        transport = SHMTransport()
        topic = TopicConfig(name="/test/shm_numpy", buffer_size=4096)

        received = []
        def callback(data, ts):
            received.append(data)

        pub = transport.create_publisher(topic)
        sub = transport.create_subscriber(topic, callback)

        arr = np.zeros((10, 10), dtype=np.float32)
        arr[5, 5] = 42.0
        pub.publish(arr)

        time.sleep(0.05)
        transport.close()

        self.assertGreaterEqual(len(received), 1)
        recovered = np.frombuffer(received[0], dtype=np.float32).reshape(10, 10)
        self.assertAlmostEqual(recovered[5, 5], 42.0)

    def test_shm_transport_name(self):
        from transport.shm_transport import SHMTransport
        t = SHMTransport()
        self.assertEqual(t.name, "shm")
        t.close()


class TestDualTransport(unittest.TestCase):
    """Dual 传输层测试 (SHM + DDS mock)。"""

    def test_dual_publish_calls_both(self):
        from transport.dual_transport import DualPublisher
        from transport.core import TopicConfig

        topic = TopicConfig(name="/test/dual")
        calls = {"shm": 0, "dds": 0}

        class MockPub:
            def __init__(self, name):
                self._topic = topic
                self._name = name
            def publish(self, msg):
                calls[self._name] += 1
            def close(self):
                pass

        dual = DualPublisher(MockPub("shm"), MockPub("dds"))
        dual.publish(b"test")

        self.assertEqual(calls["shm"], 1)
        self.assertEqual(calls["dds"], 1)


# ============================================================
# 2. TransportMixin 测试 (mock ROS2 Node)
# ============================================================

class TestTransportMixin(unittest.TestCase):
    """TransportMixin 参数和初始化测试 (无需 ROS2)。"""

    def test_mixin_exports(self):
        """TransportMixin 可从 transport 包导入。"""
        from transport import TransportMixin
        self.assertTrue(hasattr(TransportMixin, 'init_transport'))
        self.assertTrue(hasattr(TransportMixin, 'create_fast_publisher'))
        self.assertTrue(hasattr(TransportMixin, 'create_fast_subscriber'))
        self.assertTrue(hasattr(TransportMixin, 'shutdown_transport'))
        self.assertTrue(hasattr(TransportMixin, 'transport_name'))


# ============================================================
# 3. Agent → TaggedLocations 集成测试
# ============================================================

class TestAgentTaggedLocations(unittest.TestCase):
    """Agent 三层降级导航测试。"""

    def setUp(self):
        self._tmpdir = tempfile.mkdtemp()
        self._tags_path = os.path.join(self._tmpdir, "tags.json")
        # 预写标签
        tags = [
            {"name": "体育馆", "position": [25.0, 30.0, 0.0], "yaw": None},
            {"name": "入口", "position": [5.0, 2.0, 0.0], "yaw": 1.57},
        ]
        with open(self._tags_path, "w", encoding="utf-8") as f:
            json.dump(tags, f, ensure_ascii=False)

    def test_tagged_location_store_load(self):
        """TaggedLocationStore 加载 JSON。"""
        sys.path.insert(0, os.path.join(_repo, "src", "semantic_planner"))
        from semantic_planner.tagged_locations import TaggedLocationStore

        store = TaggedLocationStore(self._tags_path)
        self.assertEqual(len(store.list_all()), 2)

        result = store.query("体育馆")
        self.assertIsNotNone(result)
        self.assertEqual(result["position"][0], 25.0)

    def test_tagged_location_fuzzy_query(self):
        """模糊查询：指令包含标签名。"""
        sys.path.insert(0, os.path.join(_repo, "src", "semantic_planner"))
        from semantic_planner.tagged_locations import TaggedLocationStore

        store = TaggedLocationStore(self._tags_path)

        # "去体育馆" 包含 "体育馆"
        result = store.query_fuzzy("去体育馆")
        self.assertIsNotNone(result)
        self.assertEqual(result["name"], "体育馆")

        # "大门" 不匹配任何标签
        result = store.query_fuzzy("大门")
        self.assertIsNone(result)

    def test_tagged_location_tag_and_save(self):
        """标记新地点并持久化。"""
        sys.path.insert(0, os.path.join(_repo, "src", "semantic_planner"))
        from semantic_planner.tagged_locations import TaggedLocationStore

        store = TaggedLocationStore(self._tags_path)
        store.tag("餐厅", 10.0, 15.0, 0.0, yaw=3.14)
        store.save()

        # 重新加载验证
        store2 = TaggedLocationStore(self._tags_path)
        self.assertEqual(len(store2.list_all()), 3)
        r = store2.query("餐厅")
        self.assertAlmostEqual(r["position"][0], 10.0)
        self.assertAlmostEqual(r["yaw"], 3.14)

    def test_three_tier_degradation_logic(self):
        """三层降级: tagged → instruction → semantic。"""
        sys.path.insert(0, os.path.join(_repo, "src", "semantic_planner"))
        from semantic_planner.tagged_locations import TaggedLocationStore

        store = TaggedLocationStore(self._tags_path)

        # 第一层: 精确匹配标签 → 返回坐标
        tagged = store.query("体育馆")
        self.assertIsNotNone(tagged)
        self.assertEqual(tagged["name"], "体育馆")

        # 第一层 (模糊): 指令中包含标签名
        tagged = store.query("体育馆") or store.query_fuzzy("导航到体育馆")
        self.assertIsNotNone(tagged)

        # 第二层: 未知目标 → 无标签匹配 → 应转发给语义规划器
        tagged = store.query("未知目标") or store.query_fuzzy("未知目标")
        self.assertIsNone(tagged)  # 触发第二层


# ============================================================
# 4. Sim CLI 参数解析测试
# ============================================================

class TestSimCLI(unittest.TestCase):
    """Sim CLI 参数解析测试。"""

    def test_parse_default_args(self):
        from simulate.cli import _parse_args
        args = _parse_args([])
        self.assertEqual(args.engine, "mujoco")
        self.assertEqual(args.world, "factory")
        self.assertFalse(args.headless)
        self.assertIsNone(args.scenario)
        self.assertFalse(args.nav)
        self.assertFalse(args.semantic)

    def test_parse_nav_flag(self):
        from simulate.cli import _parse_args
        args = _parse_args(["--nav"])
        self.assertTrue(args.nav)
        self.assertFalse(args.semantic)

    def test_parse_semantic_flag(self):
        from simulate.cli import _parse_args
        args = _parse_args(["--semantic"])
        self.assertTrue(args.semantic)

    def test_parse_scenario_with_instruction(self):
        from simulate.cli import _parse_args
        args = _parse_args([
            "--scenario", "semantic_nav",
            "--instruction", "导航到大门",
            "--max-time", "60",
        ])
        self.assertEqual(args.scenario, "semantic_nav")
        self.assertEqual(args.instruction, "导航到大门")
        self.assertEqual(args.max_time, 60.0)

    def test_parse_goal_coordinates(self):
        from simulate.cli import _parse_args
        args = _parse_args(["--scenario", "navigation", "--goal", "10,5"])
        self.assertEqual(args.goal, "10,5")


# ============================================================
# 5. Factory 测试
# ============================================================

class TestTransportFactory(unittest.TestCase):
    """Transport factory 策略选择测试。"""

    def test_create_shm_transport(self):
        from transport.factory import create_transport
        from transport.core import TransportStrategy
        t = create_transport(TransportStrategy.SHM)
        self.assertEqual(t.name, "shm")
        t.close()

    def test_create_dds_requires_node(self):
        from transport.factory import create_transport
        from transport.core import TransportStrategy
        with self.assertRaises(ValueError):
            create_transport(TransportStrategy.DDS, ros_node=None)

    def test_create_dual_requires_node(self):
        from transport.factory import create_transport
        from transport.core import TransportStrategy
        with self.assertRaises(ValueError):
            create_transport(TransportStrategy.DUAL, ros_node=None)


if __name__ == "__main__":
    unittest.main(verbosity=2)
