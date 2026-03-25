"""core.transport.factory — create the appropriate transport instance by strategy.

Strategy selection:
  DDS  — all communication via ROS2 (default)
  SHM  — same-machine communication via shared memory
  AUTO — auto-select: large messages via SHM, small via DDS
  DUAL — SHM + DDS dual-write (low latency + tool compatibility)
"""

import logging
from typing import Any, Callable, Optional

from .abc import (
    Publisher,
    Subscriber,
    TopicConfig,
    TransportABC,
    TransportStrategy,
)

logger = logging.getLogger(__name__)

_default_transport: Optional[TransportABC] = None
_shm_transport: Optional[TransportABC] = None


def create_transport(
    strategy: TransportStrategy = TransportStrategy.DDS,
    ros_node=None,
) -> TransportABC:
    """Create a transport instance by strategy."""

    if strategy == TransportStrategy.DDS:
        from .dds import DDSTransport
        return DDSTransport()

    elif strategy == TransportStrategy.SHM:
        from .shm import SHMTransport
        return SHMTransport()

    elif strategy == TransportStrategy.DUAL:
        from .dds import DDSTransport
        from .shm import SHMTransport
        from .dual import DualTransport
        return DualTransport(SHMTransport(), DDSTransport())

    elif strategy == TransportStrategy.AUTO:
        # AUTO: prefer SHM (same machine), fallback to DDS
        from .shm import SHMTransport
        try:
            return SHMTransport()
        except Exception:
            from .dds import DDSTransport
            if ros_node is None:
                raise ValueError("AUTO fallback to DDS requires ros_node")
            return DDSTransport(ros_node)

    raise ValueError(f"Unknown strategy: {strategy}")


def create_publisher(
    topic_name: str,
    msg_type: Any = None,
    strategy: TransportStrategy = TransportStrategy.DDS,
    ros_node=None,
    **kwargs,
) -> Publisher:
    """Shortcut to create a publisher."""
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
    """Shortcut to create a subscriber."""
    topic = TopicConfig(
        name=topic_name,
        msg_type=msg_type,
        strategy=strategy,
        **kwargs,
    )
    transport = create_transport(strategy, ros_node)
    return transport.create_subscriber(topic, callback)
