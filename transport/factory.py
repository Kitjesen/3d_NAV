"""
传输工厂 — 按策略创建合适的传输实例

策略选择:
  DDS  — 所有通信走 ROS2 (默认)
  SHM  — 同机通信走共享内存
  AUTO — 自动判断: 大消息走 SHM，小消息走 DDS
  DUAL — SHM + DDS 双写 (低延迟 + 工具兼容)
"""

import logging
from typing import Any, Callable, Optional

from .core import (
    Publisher,
    Subscriber,
    TopicConfig,
    Transport,
    TransportStrategy,
)

logger = logging.getLogger(__name__)

_default_transport: Optional[Transport] = None
_shm_transport: Optional[Transport] = None


def create_transport(
    strategy: TransportStrategy = TransportStrategy.DDS,
    ros_node=None,
) -> Transport:
    """创建传输实例"""

    if strategy == TransportStrategy.DDS:
        from .dds_transport import DDSTransport
        if ros_node is None:
            raise ValueError("DDSTransport 需要 ros_node 参数")
        return DDSTransport(ros_node)

    elif strategy == TransportStrategy.SHM:
        from .shm_transport import SHMTransport
        return SHMTransport()

    elif strategy == TransportStrategy.DUAL:
        from .dds_transport import DDSTransport
        from .shm_transport import SHMTransport
        from .dual_transport import DualTransport
        if ros_node is None:
            raise ValueError("DualTransport 需要 ros_node 参数")
        return DualTransport(SHMTransport(), DDSTransport(ros_node))

    elif strategy == TransportStrategy.AUTO:
        # AUTO: 优先 SHM (同机)，fallback DDS
        from .shm_transport import SHMTransport
        try:
            return SHMTransport()
        except Exception:
            from .dds_transport import DDSTransport
            if ros_node is None:
                raise ValueError("AUTO fallback 到 DDS 需要 ros_node")
            return DDSTransport(ros_node)

    raise ValueError(f"未知策略: {strategy}")


def create_publisher(
    topic_name: str,
    msg_type: Any = None,
    strategy: TransportStrategy = TransportStrategy.DDS,
    ros_node=None,
    **kwargs,
) -> Publisher:
    """快捷创建发布者"""
    topic = TopicConfig(
        name=topic_name,
        msg_type=msg_type,
        strategy=strategy,
        **kwargs,
    )
    transport = create_transport(strategy, ros_node)
    return transport.create_publisher(topic)


def create_subscriber(
    topic_name: str,
    msg_type: Any = None,
    callback: Callable = None,
    strategy: TransportStrategy = TransportStrategy.DDS,
    ros_node=None,
    **kwargs,
) -> Subscriber:
    """快捷创建订阅者"""
    topic = TopicConfig(
        name=topic_name,
        msg_type=msg_type,
        strategy=strategy,
        **kwargs,
    )
    transport = create_transport(strategy, ros_node)
    return transport.create_subscriber(topic, callback)
