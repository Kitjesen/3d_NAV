"""core.transport — Unified transport layer for LingTu navigation system.

Consolidates all transport implementations under core:

  Transport (Protocol)  — generic publish/subscribe/close interface
  LocalTransport        — in-process zero-copy bus (testing & single-process)
  TransportABC          — ABC factory for ROS2-style backends
  Publisher / Subscriber — ABC message endpoints
  TransportStrategy     — backend selection enum
  TopicConfig           — per-topic configuration

Backends (conditionally imported):
  SHMTransport   — POSIX SharedMemory (high-speed same-machine)
  DDSTransport   — ROS2 CycloneDDS (default, ecosystem-compatible)
  DualTransport  — SHM + DDS dual-write

Factory:
  create_transport()  — instantiate a backend by strategy
  create_publisher()  — shortcut for one-off publisher
  create_subscriber() — shortcut for one-off subscriber

ROS2 integration:
  TransportMixin — mixin giving any ROS2 Node fast-transport helpers
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
from .ros2_mixin import TransportMixin

# --- conditional backend imports ---
try:
    from .shm import SHMTransport, SHMPublisher, SHMSubscriber
except ImportError:
    pass  # numpy not available

try:
    from .dds import DDSTransport, DDSPublisher, DDSSubscriber
except ImportError:
    pass  # rclpy not available

try:
    from .dual import DualTransport, DualPublisher, DualSubscriber
except ImportError:
    pass  # depends on SHM + DDS

__all__ = [
    # Protocol-based
    "Transport",
    "LocalTransport",
    # ABC-based
    "TransportABC",
    "Publisher",
    "Subscriber",
    "TransportStrategy",
    "TopicConfig",
    # Factory
    "create_transport",
    "create_publisher",
    "create_subscriber",
    # Adapter
    "TransportAdapter",
    # ROS2 mixin
    "TransportMixin",
    # Backends (conditional)
    "SHMTransport",
    "DDSTransport",
    "DualTransport",
]
