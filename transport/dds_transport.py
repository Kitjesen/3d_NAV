"""
DDS Transport — ROS2 CycloneDDS

标准 ROS2 发布/订阅的薄包装，保持现有代码兼容。
默认传输后端，支持全部 ROS2 QoS 策略。
"""

import logging
from typing import Any, Callable, Optional

from .core import Publisher, Subscriber, TopicConfig, Transport

logger = logging.getLogger(__name__)


class DDSPublisher(Publisher):
    """ROS2 DDS Publisher 包装"""

    def __init__(self, topic: TopicConfig, ros_node):
        super().__init__(topic)
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

        qos = QoSProfile(depth=topic.qos_depth)
        if topic.reliable:
            qos.reliability = ReliabilityPolicy.RELIABLE
            qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        else:
            qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self._pub = ros_node.create_publisher(topic.msg_type, topic.name, qos)

    def publish(self, msg: Any) -> None:
        self._pub.publish(msg)

    def close(self) -> None:
        pass  # ROS2 node 销毁时自动清理


class DDSSubscriber(Subscriber):
    """ROS2 DDS Subscriber 包装"""

    def __init__(self, topic: TopicConfig, callback: Callable, ros_node):
        super().__init__(topic, callback)
        from rclpy.qos import QoSProfile, ReliabilityPolicy

        qos = QoSProfile(depth=topic.qos_depth)
        if topic.reliable:
            qos.reliability = ReliabilityPolicy.RELIABLE
        else:
            qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self._sub = ros_node.create_subscription(
            topic.msg_type, topic.name, callback, qos
        )

    def start(self) -> None:
        pass  # ROS2 spin 驱动

    def close(self) -> None:
        pass


class DDSTransport(Transport):
    """ROS2 CycloneDDS 传输层"""

    def __init__(self, ros_node):
        self._node = ros_node

    def create_publisher(self, topic: TopicConfig) -> DDSPublisher:
        return DDSPublisher(topic, self._node)

    def create_subscriber(self, topic: TopicConfig, callback: Callable) -> DDSSubscriber:
        return DDSSubscriber(topic, callback, self._node)

    def close(self) -> None:
        pass  # ROS2 node 管理生命周期

    @property
    def name(self) -> str:
        return "dds"
