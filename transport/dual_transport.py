"""
Dual Transport — SHM + DDS 双写

发布时同时写 SHM (低延迟) 和 DDS (工具兼容)。
适用于需要同时满足实时性和 ROS2 工具访问的话题。
"""

import logging
from typing import Any, Callable

from .core import Publisher, Subscriber, TopicConfig, Transport

logger = logging.getLogger(__name__)


class DualPublisher(Publisher):
    """双写发布者: SHM + DDS"""

    def __init__(self, shm_pub: Publisher, dds_pub: Publisher):
        super().__init__(shm_pub._topic)
        self._shm = shm_pub
        self._dds = dds_pub

    def publish(self, msg: Any) -> None:
        # SHM 快速路径 (对 bytes/numpy 直接写)
        self._shm.publish(msg)
        # DDS 兼容路径 (对 ROS2 msg 原样发)
        self._dds.publish(msg)

    def close(self) -> None:
        self._shm.close()
        self._dds.close()


class DualSubscriber(Subscriber):
    """双读订阅者: 优先 SHM，DDS 作为 fallback"""

    def __init__(self, shm_sub: Subscriber, dds_sub: Subscriber):
        super().__init__(shm_sub._topic, shm_sub._callback)
        self._shm = shm_sub
        self._dds = dds_sub
        self._use_shm = True

    def start(self) -> None:
        self._shm.start()
        # DDS 也启动但只在 SHM 失败时用
        self._dds.start()

    def close(self) -> None:
        self._shm.close()
        self._dds.close()


class DualTransport(Transport):
    """SHM + DDS 双通道传输"""

    def __init__(self, shm_transport: Transport, dds_transport: Transport):
        self._shm = shm_transport
        self._dds = dds_transport

    def create_publisher(self, topic: TopicConfig) -> DualPublisher:
        return DualPublisher(
            self._shm.create_publisher(topic),
            self._dds.create_publisher(topic),
        )

    def create_subscriber(self, topic: TopicConfig, callback: Callable) -> DualSubscriber:
        return DualSubscriber(
            self._shm.create_subscriber(topic, callback),
            self._dds.create_subscriber(topic, callback),
        )

    def close(self) -> None:
        self._shm.close()
        self._dds.close()

    @property
    def name(self) -> str:
        return "dual(shm+dds)"
