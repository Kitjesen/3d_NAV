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
from .local import Transport, LocalTransport
from .abc import (
    TransportABC,
    Publisher,
    Subscriber,
    TransportStrategy,
    TopicConfig,
)
from .factory import create_publisher, create_subscriber, create_transport
from .adapter import TransportAdapter

# --- conditional backend imports ---
try:
    from .shm import SHMTransport, SHMPublisher, SHMSubscriber
except ImportError:
    pass

try:
    from .dds import DDSTransport, DDSPublisher, DDSSubscriber
except ImportError:
    pass

try:
    from .dual import DualTransport, DualPublisher, DualSubscriber
except ImportError:
    pass

__all__ = [
    "Transport",
    "LocalTransport",
    "TransportABC",
    "Publisher",
    "Subscriber",
    "TransportStrategy",
    "TopicConfig",
    "create_transport",
    "create_publisher",
    "create_subscriber",
    "TransportAdapter",
]
