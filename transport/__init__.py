"""
LingTu Transport Layer — 可插拔通信抽象

支持多种传输后端，上层节点通过统一接口通信，不感知底层实现。

后端:
  DDSTransport    — ROS2 CycloneDDS (默认，兼容 ROS2 生态)
  SHMTransport    — POSIX SharedMemory (同机高速通道)
  DualTransport   — 双写模式 (SHM + DDS 并行)

工厂:
  create_transport()    — 按策略创建传输实例
  create_publisher()    — 快捷创建发布者
  create_subscriber()   — 快捷创建订阅者
"""

from .core import Transport, Publisher, Subscriber, TransportStrategy, TopicConfig
from .factory import create_publisher, create_subscriber, create_transport
from .ros2_mixin import TransportMixin

__all__ = [
    "Transport",
    "Publisher",
    "Subscriber",
    "TransportStrategy",
    "TopicConfig",
    "TransportMixin",
    "create_publisher",
    "create_subscriber",
    "create_transport",
]
