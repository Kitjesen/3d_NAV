"""core.transport.local — Transport Protocol and LocalTransport (in-process bus).

Transport Protocol:
    Any object implementing publish / subscribe / close can serve as a
    transport backend (ROS2, DDS, SHM, gRPC, etc.).

LocalTransport:
    Zero-copy in-process synchronous bus for single-process mode and testing.
    Thread-safe: internal Lock protects the subscription map.
"""

from __future__ import annotations

import logging
import threading
from collections.abc import Callable
from typing import Any, Dict, List, Protocol, runtime_checkable

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Transport Protocol
# ---------------------------------------------------------------------------

@runtime_checkable
class Transport(Protocol):
    """Transport layer abstract protocol.

    Any object implementing publish / subscribe / close qualifies as a
    transport backend, including ROS2, DDS, SHM, gRPC, etc.
    """

    def publish(self, topic: str, msg: Any) -> None:
        """Publish a message to a topic."""
        ...

    def subscribe(self, topic: str, cb: Callable[[Any], None]) -> None:
        """Subscribe to a topic; invoke *cb* on each received message."""
        ...

    def close(self) -> None:
        """Release resources."""
        ...


# ---------------------------------------------------------------------------
# LocalTransport — in-process zero-copy bus
# ---------------------------------------------------------------------------

class LocalTransport:
    """In-process synchronous bus for single-process mode and testing.

    Messages are passed by direct reference (zero-copy), with no
    serialization overhead.  Thread-safe via an internal Lock.
    """

    def __init__(self) -> None:
        self._bus: dict[str, list[Callable[[Any], None]]] = {}
        self._lock = threading.Lock()

    def publish(self, topic: str, msg: Any) -> None:
        """Publish a message, synchronously invoking all subscriber callbacks."""
        with self._lock:
            callbacks = list(self._bus.get(topic, []))
        for cb in callbacks:
            try:
                cb(msg)
            except Exception:
                logger.exception("LocalTransport callback error on topic '%s'", topic)

    def subscribe(self, topic: str, cb: Callable[[Any], None]) -> None:
        """Subscribe to *topic*."""
        with self._lock:
            self._bus.setdefault(topic, []).append(cb)

    def unsubscribe(self, topic: str, cb: Callable[[Any], None]) -> None:
        """Unsubscribe from *topic*."""
        with self._lock:
            cbs = self._bus.get(topic)
            if cbs and cb in cbs:
                cbs.remove(cb)

    def close(self) -> None:
        """Clear all subscriptions."""
        with self._lock:
            self._bus.clear()

    @property
    def topics(self) -> list[str]:
        """Return a list of currently active topics."""
        with self._lock:
            return list(self._bus.keys())

    def subscriber_count(self, topic: str) -> int:
        """Return the number of subscribers on *topic*."""
        with self._lock:
            return len(self._bus.get(topic, []))

    def __repr__(self) -> str:
        return f"LocalTransport(topics={len(self._bus)})"
