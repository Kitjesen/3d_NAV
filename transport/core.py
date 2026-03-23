"""
传输层抽象 — 极简接口，上层节点只调 publish()/subscribe()

设计:
  Publisher  — 发布消息到指定话题
  Subscriber — 从指定话题接收消息，触发回调
  Transport  — 创建 Publisher/Subscriber 的工厂

所有传输后端 (DDS/SHM/LCM) 实现相同接口，可在运行时切换。
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum
from typing import Any, Callable


class TransportStrategy(Enum):
    """传输策略"""
    DDS = "dds"
    SHM = "shm"
    AUTO = "auto"
    DUAL = "dual"


@dataclass
class TopicConfig:
    """话题配置"""
    name: str
    msg_type: Any = None
    strategy: TransportStrategy = TransportStrategy.DDS
    buffer_size: int = 0
    qos_depth: int = 10
    reliable: bool = False


class Publisher(ABC):
    """发布者"""

    def __init__(self, topic: TopicConfig):
        self._topic = topic

    @property
    def topic_name(self) -> str:
        return self._topic.name

    @abstractmethod
    def publish(self, msg: Any) -> None:
        ...

    def close(self) -> None:
        pass


class Subscriber(ABC):
    """订阅者"""

    def __init__(self, topic: TopicConfig, callback: Callable):
        self._topic = topic
        self._callback = callback

    @property
    def topic_name(self) -> str:
        return self._topic.name

    @abstractmethod
    def start(self) -> None:
        ...

    def close(self) -> None:
        pass


class Transport(ABC):
    """传输层工厂"""

    @abstractmethod
    def create_publisher(self, topic: TopicConfig) -> Publisher:
        ...

    @abstractmethod
    def create_subscriber(self, topic: TopicConfig, callback: Callable) -> Subscriber:
        ...

    @abstractmethod
    def close(self) -> None:
        ...

    @property
    @abstractmethod
    def name(self) -> str:
        ...
