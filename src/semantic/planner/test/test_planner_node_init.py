"""
test_planner_node_init.py — SemanticPlannerNode 初始化回归测试 (Round 14)

验证之前修复的关键属性在 __init__ 后正确存在，不会导致 AttributeError。
无需 ROS2 环境，使用 unittest.mock 模拟所有 ROS2 依赖。

覆盖:
  - _follow_person_pub 在 __init__ 后存在（不是 None）
  - _monitor_timer 在 __init__ 后已创建
  - _make_twist_stamped() 返回 TwistStamped 对象（不崩溃）
  - FOLLOW_PERSON 模式不会因缺少属性而 AttributeError
  - _person_tracker 在 __init__ 后可用
  - _pub_cmd_vel 在 __init__ 后存在（TwistStamped 发布器）
"""

import json
import sys
import types
import unittest
from unittest.mock import MagicMock, patch


# ---------------------------------------------------------------------------
#  Fake ROS2 Node 基类 — 捕获 declare_parameter 的默认值
# ---------------------------------------------------------------------------

class FakeNode:
    """替代 rclpy.node.Node 的假节点。
    declare_parameter(name, default) 记录参数;
    get_parameter(name) 返回之前记录的值。
    """

    def __init__(self, name="test_node", **kwargs):
        self._params = {}

    def declare_parameter(self, name, default=None):
        self._params[name] = default

    def get_parameter(self, name):
        val = self._params.get(name, "")
        param = MagicMock()
        param.value = val
        return param

    def create_subscription(self, msg_type, topic, callback, qos):
        return MagicMock()

    def create_publisher(self, msg_type, topic, qos_or_depth):
        return MagicMock()

    def create_timer(self, period, callback):
        return MagicMock()

    def get_logger(self):
        return MagicMock()

    def get_clock(self):
        clock = MagicMock()
        clock.now.return_value.to_msg.return_value = MagicMock()
        return clock


# ---------------------------------------------------------------------------
#  模拟 ROS2 消息类型
# ---------------------------------------------------------------------------

def _make_mock_twist_stamped():
    ts = MagicMock()
    ts.header = MagicMock()
    ts.header.frame_id = ""
    ts.header.stamp = MagicMock()
    ts.twist = MagicMock()
    ts.twist.linear = MagicMock(x=0.0, y=0.0, z=0.0)
    ts.twist.angular = MagicMock(x=0.0, y=0.0, z=0.0)
    return ts


def _setup_mock_modules():
    """注册所有 mock ROS2 模块到 sys.modules。"""
    mock_twist_cls = MagicMock(side_effect=_make_mock_twist_stamped)
    mock_pose_cls = MagicMock(side_effect=lambda: MagicMock())

    mock_geometry = MagicMock()
    mock_geometry.msg.TwistStamped = mock_twist_cls
    mock_geometry.msg.PoseStamped = mock_pose_cls

    mock_nav = MagicMock()
    mock_sensor = MagicMock()
    mock_std = MagicMock()
    mock_std.msg.String = MagicMock

    # rclpy 模块: 用 FakeNode 替代 Node
    mock_rclpy = MagicMock()
    mock_rclpy_node = types.ModuleType("rclpy.node")
    mock_rclpy_node.Node = FakeNode

    mock_rclpy_qos = MagicMock()
    mock_rclpy_qos.QoSProfile = MagicMock(return_value=MagicMock())
    mock_rclpy_qos.ReliabilityPolicy = MagicMock()
    mock_rclpy_qos.HistoryPolicy = MagicMock()

    mods = {
        "rclpy": mock_rclpy,
        "rclpy.node": mock_rclpy_node,
        "rclpy.action": MagicMock(),
        "rclpy.action.client": MagicMock(),
        "rclpy.qos": mock_rclpy_qos,
        "geometry_msgs": mock_geometry,
        "geometry_msgs.msg": mock_geometry.msg,
        "nav_msgs": mock_nav,
        "nav_msgs.msg": mock_nav.msg,
        "sensor_msgs": mock_sensor,
        "sensor_msgs.msg": mock_sensor.msg,
        "std_msgs": mock_std,
        "std_msgs.msg": mock_std.msg,
        "nav2_msgs": MagicMock(),
        "nav2_msgs.action": MagicMock(),
    }
    for k, v in mods.items():
        sys.modules[k] = v
    return mods


def _build_node():
    """构造一个 mock 过的 SemanticPlannerNode 实例。"""
    _setup_mock_modules()

    # 清除缓存的 planner_node 模块以触发重新 import
    for key in list(sys.modules.keys()):
        if "planner_node" in key:
            del sys.modules[key]

    from semantic_planner.planner_node import SemanticPlannerNode
    node = SemanticPlannerNode()
    return node


class TestPlannerNodeInit(unittest.TestCase):
    """SemanticPlannerNode.__init__ 回归测试。"""

    @classmethod
    def setUpClass(cls):
        """一次性构造 node 实例。"""
        cls.node = _build_node()

    def test_follow_person_pub_exists(self):
        """验证 _follow_person_pub 在 __init__ 后存在（不是 None）。
        修复前此属性缺失，导致 FOLLOW_PERSON 模式 AttributeError。
        """
        self.assertTrue(hasattr(self.node, "_follow_person_pub"))
        self.assertIsNotNone(self.node._follow_person_pub)

    def test_monitor_timer_exists(self):
        """验证 _monitor_timer 在 __init__ 后已创建。
        修复前此属性在某些参数组合下未被赋值。
        """
        self.assertTrue(hasattr(self.node, "_monitor_timer"))
        self.assertIsNotNone(self.node._monitor_timer)

    def test_make_twist_stamped_returns_valid_msg(self):
        """验证 _make_twist_stamped() 返回 TwistStamped 对象（不崩溃）。
        修复前使用 Twist 而非 TwistStamped，导致类型不匹配。
        """
        msg = self.node._make_twist_stamped(linear_x=1.0, angular_z=0.5)
        self.assertIsNotNone(msg)
        # 验证 header 和 twist 属性存在
        self.assertTrue(hasattr(msg, "header"))
        self.assertTrue(hasattr(msg, "twist"))

    def test_follow_person_mode_no_attribute_error(self):
        """验证 FOLLOW_PERSON 模式所需的全部属性都存在。
        修复前 _follow_mode / _follow_target_label / _person_tracker 可能缺失。
        """
        required_attrs = [
            "_follow_mode",
            "_follow_target_label",
            "_follow_timeout",
            "_follow_start_time",
            "_person_tracker",
            "_follow_person_pub",
            "_pub_cmd_vel",
        ]
        for attr in required_attrs:
            self.assertTrue(
                hasattr(self.node, attr),
                f"Missing attribute: {attr}",
            )

    def test_person_tracker_initialized(self):
        """验证 _person_tracker 初始化后具有 follow_distance 属性。"""
        tracker = self.node._person_tracker
        self.assertIsNotNone(tracker)
        self.assertTrue(hasattr(tracker, "follow_distance"))

    def test_pub_cmd_vel_exists(self):
        """验证 _pub_cmd_vel 发布器在 __init__ 后存在（TwistStamped 类型）。"""
        self.assertTrue(hasattr(self.node, "_pub_cmd_vel"))
        self.assertIsNotNone(self.node._pub_cmd_vel)


if __name__ == "__main__":
    unittest.main()
