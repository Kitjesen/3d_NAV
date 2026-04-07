"""core.transport — Unified transport layer for LingTu navigation system.

Consolidates all transport implementations under core:

  Transport (Protocol)  — generic publish/subscribe/close interface
  LocalTransport        — in-process zero-copy bus (testing & single-process)
  TransportABC          — ABC factory for ROS2-style backends
  Publisher / Subscriber — ABC message endpoints
  TransportStrategy     — backend selection enum
  TopicConfig           — per-topic configuration

Backends (conditionally imported when deps are available):
  SHMTransport   — POSIX SharedMemory (high-speed same-machine)
  DDSTransport   — ROS2 CycloneDDS
  DualTransport  — SHM + DDS dual-write

Factory:
  create_transport()  — instantiate a backend by strategy
  create_publisher()  — shortcut for one-off publisher
  create_subscriber() — shortcut for one-off subscriber
"""

# --- always-available core types ---
from .abc import (
    Publisher,
    Subscriber,
    TopicConfig,
    TransportABC,
    TransportStrategy,
)
from .adapter import TransportAdapter
from .factory import create_publisher, create_subscriber, create_transport
from .local import LocalTransport, Transport

# --- conditional backend imports ---
try:
    from .shm import SHMPublisher, SHMSubscriber, SHMTransport
except ImportError:
    pass

try:
    from .dds import DDSPublisher, DDSSubscriber, DDSTransport
except ImportError:
    pass

try:
    from .dual import DualPublisher, DualSubscriber, DualTransport
except ImportError:
    pass

__all__ = [
    "LocalTransport",
    "Publisher",
    "Subscriber",
    "TopicConfig",
    "Transport",
    "TransportABC",
    "TransportAdapter",
    "TransportStrategy",
    "create_publisher",
    "create_subscriber",
    "create_transport",
]
