"""
test_sdk.py — MapPilot SDK 集成测试

验证 NavigationSDK + PerceptionSDK 在无 ROS2 环境下可用。
"""

import json
import unittest

import numpy as np

# semantic_common 仅在 S100P ROS2 环境可用
try:
    import semantic_common
    _HAS_SEMANTIC_COMMON = True
except ImportError:
    _HAS_SEMANTIC_COMMON = False

_NEED_BACKEND = unittest.skipUnless(
    _HAS_SEMANTIC_COMMON,
    "需要 semantic_common (仅 S100P ROS2 环境)"
)


class TestPerceptionSDK(unittest.TestCase):
    """PerceptionSDK 基本功能测试。"""

    def setUp(self):
        from mappilot_sdk import PerceptionSDK
        self.sdk = PerceptionSDK()

    @_NEED_BACKEND
    def test_is_available(self):
        self.assertTrue(self.sdk.is_available)

    def test_get_empty_scene_graph(self):
        sg_json = self.sdk.get_scene_graph_json()
        data = json.loads(sg_json)
        self.assertIn("objects", data)

    @_NEED_BACKEND
    def test_update_from_detections(self):
        from mappilot_sdk.mocks import make_mock_detection
        dets = [
            make_mock_detection("chair", (1.0, 0.0, 0.5), seed=1),
            make_mock_detection("desk", (3.0, 0.0, 0.8), seed=2),
        ]
        count = self.sdk.update_from_detections(dets)
        self.assertGreaterEqual(count, 2)

    @_NEED_BACKEND
    def test_get_safety_level_known(self):
        level = self.sdk.get_safety_level("fire extinguisher")
        self.assertEqual(level, "caution")

    def test_get_safety_level_unknown(self):
        level = self.sdk.get_safety_level("banana_xyz")
        self.assertEqual(level, "safe")

    @_NEED_BACKEND
    def test_check_safety_dangerous(self):
        result = self.sdk.check_safety("electrical_panel", "pick")
        self.assertIsNotNone(result)
        self.assertEqual(result["response"], "block")

    def test_check_safety_safe(self):
        result = self.sdk.check_safety("chair", "sit")
        self.assertIsNone(result)

    @_NEED_BACKEND
    def test_get_affordances_chair(self):
        affordances = self.sdk.get_affordances("chair")
        self.assertIn("sittable", affordances)

    @_NEED_BACKEND
    def test_enrich_known(self):
        props = self.sdk.enrich_object("fire extinguisher")
        self.assertTrue(props["kg_matched"])
        self.assertIn("affordances", props)

    def test_enrich_unknown(self):
        props = self.sdk.enrich_object("xyz_unknown_widget")
        self.assertFalse(props["kg_matched"])

    @_NEED_BACKEND
    def test_clear_resets_tracker(self):
        from mappilot_sdk.mocks import make_mock_detection
        det = make_mock_detection("chair", (1.0, 0.0, 0.0), seed=1)
        self.sdk.update_from_detections([det])
        self.sdk.clear()
        self.assertEqual(self.sdk.get_object_count(), 0)


class TestNavigationSDK(unittest.TestCase):
    """NavigationSDK 基本功能测试。"""

    def setUp(self):
        from mappilot_sdk import NavigationSDK
        self.sdk = NavigationSDK()

    @_NEED_BACKEND
    def test_is_available(self):
        self.assertTrue(self.sdk.is_available)

    def test_update_topology(self):
        from mappilot_sdk import SceneGraphInput
        sg = SceneGraphInput(
            rooms=[
                {"room_id": 0, "name": "corridor", "center": {"x": 0, "y": 0},
                 "semantic_labels": ["door"]},
            ],
            topology_edges=[],
        )
        self.sdk.update_topology(sg)  # Should not raise

    def test_record_robot_position(self):
        from mappilot_sdk import SceneGraphInput
        sg = SceneGraphInput(
            rooms=[
                {"room_id": 0, "name": "corridor", "center": {"x": 0, "y": 0},
                 "semantic_labels": ["door"]},
            ],
        )
        self.sdk.update_topology(sg)
        self.sdk.record_robot_position(0.5, 0.0)  # Should not raise

    def test_get_exploration_targets_empty(self):
        targets = self.sdk.get_exploration_targets("找椅子")
        self.assertIsInstance(targets, list)

    def test_get_topology_summary(self):
        summary = self.sdk.get_topology_summary()
        self.assertIsInstance(summary, str)


class TestMocks(unittest.TestCase):
    """Mock infrastructure tests."""

    def test_mock_scene_graph_valid_json(self):
        from mappilot_sdk.mocks import make_mock_scene_graph
        sg_json = make_mock_scene_graph(["chair", "desk"])
        data = json.loads(sg_json)
        self.assertEqual(len(data["objects"]), 2)
        self.assertEqual(data["objects"][0]["label"], "chair")

    def test_mock_scene_graph_default(self):
        from mappilot_sdk.mocks import make_mock_scene_graph
        sg_json = make_mock_scene_graph()
        data = json.loads(sg_json)
        self.assertGreater(len(data["objects"]), 0)

    def test_mock_scene_graph_has_clip_features(self):
        from mappilot_sdk.mocks import make_mock_scene_graph
        sg_json = make_mock_scene_graph(["chair"])
        data = json.loads(sg_json)
        feat = data["objects"][0]["clip_feature"]
        self.assertEqual(len(feat), 512)

    def test_mock_llm_client(self):
        import asyncio
        from mappilot_sdk.mocks import MockLLMClient
        llm = MockLLMClient(response='{"object_id": "obj_0", "confidence": 0.9}')
        loop = asyncio.new_event_loop()
        result = loop.run_until_complete(
            llm.chat([{"role": "user", "content": "test"}])
        )
        loop.close()
        self.assertEqual(llm.call_count, 1)
        data = json.loads(result)
        self.assertEqual(data["object_id"], "obj_0")

    def test_mock_ros2_node(self):
        from mappilot_sdk.mocks import MockROS2Node
        node = MockROS2Node("test_node")
        node.declare_parameter("test_param", 1.5)
        val = node.get_parameter("test_param").as_double()
        self.assertAlmostEqual(val, 1.5)

    def test_mock_publisher(self):
        from mappilot_sdk.mocks import MockROS2Node
        node = MockROS2Node()
        pub = node.create_publisher(None, "/test_topic")
        pub.publish({"data": "hello"})
        self.assertEqual(len(pub.published), 1)
        self.assertEqual(pub.last["data"], "hello")

    @_NEED_BACKEND
    def test_mock_detection(self):
        from mappilot_sdk.mocks import make_mock_detection
        det = make_mock_detection("chair", (1.0, 2.0, 0.5), seed=42)
        self.assertEqual(det.label, "chair")
        self.assertAlmostEqual(det.position[0], 1.0)
        self.assertEqual(det.features.shape, (512,))
        self.assertAlmostEqual(float(np.linalg.norm(det.features)), 1.0, places=5)


class TestSceneGraphInput(unittest.TestCase):
    """SceneGraphInput dataclass tests."""

    def test_from_json(self):
        from mappilot_sdk.mocks import make_mock_scene_graph
        from mappilot_sdk import SceneGraphInput
        sg_json = make_mock_scene_graph(["chair"])
        sg = SceneGraphInput.from_json(sg_json)
        self.assertEqual(len(sg.objects), 1)

    def test_to_dict_roundtrip(self):
        from mappilot_sdk import SceneGraphInput
        sg = SceneGraphInput(
            objects=[{"id": "obj_0", "label": "chair"}],
            regions=[{"name": "office", "object_ids": ["obj_0"]}],
        )
        d = sg.to_dict()
        sg2 = SceneGraphInput.from_dict(d)
        self.assertEqual(sg2.objects[0]["label"], "chair")

    def test_empty_construction(self):
        from mappilot_sdk import SceneGraphInput
        sg = SceneGraphInput()
        self.assertEqual(len(sg.objects), 0)
        self.assertEqual(sg.to_dict()["objects"], [])


if __name__ == "__main__":
    unittest.main()
