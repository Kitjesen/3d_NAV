"""lingtu.core.stream — Typed data-flow ports and pluggable transport layer.

Inspired by the dimos Stream/Transport pattern, simplified for LingTu:

- Transport (Protocol) — inter-process / cross-node communication abstraction
- LocalTransport       — in-process zero-copy bus (testing & single-process)
- Out[T]               — output port, publishes messages
- In[T]                — input port, receives messages

Design principles:
  1. Zero external dependencies (pure Python typing + threading)
  2. Decoupled from the msgs layer — T can be any type
  3. Transport is optional — Out/In work in pure local-callback mode

Transport and LocalTransport are now defined in core.transport.local and
re-exported here for backward compatibility.
"""

from __future__ import annotations

import logging
import time
import threading
from typing import (
    Any,
    Callable,
    Generic,
    List,
    Optional,
    TypeVar,
)

from .transport.local import Transport, LocalTransport  # canonical location

logger = logging.getLogger(__name__)

T = TypeVar("T")


# ---------------------------------------------------------------------------
# Out[T] — typed output port
# ---------------------------------------------------------------------------

class Out(Generic[T]):
    """Typed output port.

    Two message delivery paths:
      1. Local callback chain — registered via _add_callback(), called by publish()
      2. External transport — bound via _bind_transport(), additionally sent by publish()

    Counters: msg_count (total published), last_ts (last publish timestamp).
    """

    __slots__ = (
        "_name", "_msg_type", "owner", "_callbacks", "_transport",
        "_transport_topic", "_msg_count", "_last_ts", "_lock",
        "_rate_window_start", "_rate_window_count", "_rate_hz",
        "_publish_errors",
    )

    def __init__(self, msg_type: type, name: str, owner: Any = None) -> None:
        self._name = name
        self._msg_type = msg_type
        self.owner = owner
        self._callbacks: List[Callable[[T], None]] = []
        self._transport: Optional[Transport] = None
        self._transport_topic: Optional[str] = None
        self._msg_count: int = 0
        self._last_ts: float = 0.0
        self._lock = threading.Lock()
        # Rate estimation (2-second sliding window)
        self._rate_window_start: float = 0.0
        self._rate_window_count: int = 0
        self._rate_hz: float = 0.0
        # Error counting
        self._publish_errors: int = 0

    # -- core API ------------------------------------------------------------------

    def publish(self, msg: T) -> None:
        """Publish a message to all local subscribers and external transport."""
        now = time.time()
        with self._lock:
            self._msg_count += 1
            self._last_ts = now
            # Rate estimation: reset window every 2 seconds
            self._rate_window_count += 1
            elapsed = now - self._rate_window_start
            if elapsed >= 2.0:
                self._rate_hz = self._rate_window_count / elapsed if elapsed > 0 else 0.0
                self._rate_window_start = now
                self._rate_window_count = 0
            cbs = list(self._callbacks)
        for cb in cbs:
            try:
                cb(msg)
            except Exception:
                self._publish_errors += 1
                logger.exception("Out[%s] callback error", self._name)
        # External transport
        if self._transport and self._transport_topic:
            try:
                self._transport.publish(self._transport_topic, msg)
            except Exception:
                self._publish_errors += 1
                logger.exception("Out[%s] transport publish error", self._name)

    # -- public API ----------------------------------------------------------------

    def subscribe(self, cb: Callable[[T], Any]) -> Callable[[], None]:
        """Subscribe to this output. Returns an unsubscribe callable.

        Compatible with dimos Out.subscribe() API.
        """
        self._add_callback(cb)
        def unsubscribe() -> None:
            self._remove_callback(cb)
        return unsubscribe

    # -- wiring API (called by Blueprint) -----------------------------------------

    def _add_callback(self, cb: Callable[[T], None]) -> None:
        """Register a local callback (for Out→In direct wiring)."""
        with self._lock:
            self._callbacks.append(cb)

    def _remove_callback(self, cb: Callable[[T], None]) -> None:
        """Remove a local callback."""
        with self._lock:
            if cb in self._callbacks:
                self._callbacks.remove(cb)

    def _bind_transport(self, transport: Transport, topic: Optional[str] = None) -> None:
        """Bind an external transport layer. topic defaults to the port name."""
        self._transport = transport
        self._transport_topic = topic or self._name

    def _clear_callbacks(self) -> None:
        """Remove all callbacks and transport binding. Called on Module.stop()."""
        with self._lock:
            self._callbacks.clear()
        self._transport = None
        self._transport_topic = None

    # -- properties ----------------------------------------------------------------

    @property
    def name(self) -> str:
        return self._name

    @property
    def msg_type(self) -> type:
        return self._msg_type

    @property
    def msg_count(self) -> int:
        return self._msg_count

    @property
    def last_ts(self) -> float:
        return self._last_ts

    @property
    def rate_hz(self) -> float:
        """Estimated publish rate in Hz (2-second window)."""
        return self._rate_hz

    @property
    def publish_errors(self) -> int:
        """Number of callback/transport errors during publish."""
        return self._publish_errors

    @property
    def callback_count(self) -> int:
        with self._lock:
            return len(self._callbacks)

    @property
    def stale_ms(self) -> float:
        """Milliseconds since last publish. -1 if never published."""
        if self._last_ts == 0.0:
            return -1.0
        return (time.time() - self._last_ts) * 1000.0

    def __repr__(self) -> str:
        transport_str = f", transport={self._transport_topic}" if self._transport else ""
        rate_str = f", {self._rate_hz:.1f}Hz" if self._rate_hz > 0 else ""
        return f"Out('{self._name}', {self._msg_type.__name__}, n={self._msg_count}{rate_str}{transport_str})"


# ---------------------------------------------------------------------------
# In[T] — typed input port
# ---------------------------------------------------------------------------

class In(Generic[T]):
    """Typed input port.

    Receives messages via _deliver() (called by Out callbacks or Transport).
    Supports two delivery policies:

    - "all" (default): every message triggers the callback immediately.
    - "latest": only the most recent message is kept. If the callback is
      still running from a previous delivery, new messages update ``latest``
      but do NOT re-enter the callback. This prevents slow consumers
      (e.g. LLM at 2s) from being overwhelmed by fast publishers (e.g. IMU
      at 50Hz). The consumer reads ``self.latest`` when ready.

    Set the policy after construction via ``set_policy("latest")``.
    """

    __slots__ = (
        "_name", "_msg_type", "owner", "_callback", "_msg_count",
        "_last_ts", "_latest", "_lock", "_policy", "_in_callback",
        "_drop_count", "_throttle_interval", "_last_deliver_ts",
        "_sample_n", "_sample_counter", "_buffer_size", "_buffer",
        "_rate_window_start", "_rate_window_count", "_rate_hz",
        "_deliver_count", "_callback_errors",
        "_max_callback_ms", "_total_callback_ms",
    )

    def __init__(self, msg_type: type, name: str, owner: Any = None) -> None:
        self._name = name
        self._msg_type = msg_type
        self.owner = owner
        self._callback: Optional[Callable[[T], None]] = None
        self._msg_count: int = 0
        self._last_ts: float = 0.0
        self._latest: Optional[T] = None
        self._lock = threading.Lock()
        self._policy: str = "all"
        self._in_callback: bool = False
        self._drop_count: int = 0
        # Throttle state
        self._throttle_interval: float = 0.0
        self._last_deliver_ts: float = 0.0
        # Sample state
        self._sample_n: int = 1
        self._sample_counter: int = 0
        # Buffer state
        self._buffer_size: int = 1
        self._buffer: List = []
        # Rate + latency stats
        self._rate_window_start: float = 0.0
        self._rate_window_count: int = 0
        self._rate_hz: float = 0.0
        self._deliver_count: int = 0  # messages actually delivered to callback
        self._callback_errors: int = 0
        self._max_callback_ms: float = 0.0
        self._total_callback_ms: float = 0.0

    # -- Core API ----------------------------------------------------------------

    def subscribe(self, cb: Callable[[T], None]) -> None:
        """Register message callback. Raises RuntimeError if already subscribed."""
        if self._callback is not None:
            raise RuntimeError(
                f"In[{self._name}] already subscribed. "
                f"Each input port accepts exactly one subscriber."
            )
        self._callback = cb

    def _clear_subscriber(self) -> None:
        """Remove callback. Called on Module.stop() to break reference cycles."""
        self._callback = None

    def set_policy(self, policy: str, **kwargs) -> None:
        """Set delivery policy with optional parameters.

        Policies:
            "all"      — deliver every message (default)
            "latest"   — drop if callback busy (non-reentrant)
            "throttle" — max N messages/sec (interval=seconds between delivers)
            "sample"   — deliver every Nth message (n=skip count)
            "buffer"   — collect N messages, deliver as batch list (size=batch size)

        Usage in Module.setup()::

            self.image.set_policy("latest")               # drop old frames
            self.imu.set_policy("throttle", interval=0.1) # max 10Hz
            self.lidar.set_policy("sample", n=5)          # every 5th scan
            self.detections.set_policy("buffer", size=10)  # batch of 10
        """
        valid = ("all", "latest", "throttle", "sample", "buffer")
        if policy not in valid:
            raise ValueError(f"Unknown policy '{policy}', expected one of {valid}")
        self._policy = policy
        if policy == "throttle":
            self._throttle_interval = kwargs.get("interval", 0.1)
        elif policy == "sample":
            self._sample_n = max(1, kwargs.get("n", 2))
            self._sample_counter = 0
        elif policy == "buffer":
            self._buffer_size = max(1, kwargs.get("size", 10))
            self._buffer = []

    def _deliver(self, msg: T) -> None:
        """Deliver a message (called by Out callback or Transport)."""
        now = time.time()
        self._msg_count += 1
        self._last_ts = now
        self._latest = msg

        # Rate estimation (2-second sliding window)
        self._rate_window_count += 1
        elapsed = now - self._rate_window_start
        if elapsed >= 2.0:
            self._rate_hz = self._rate_window_count / elapsed if elapsed > 0 else 0.0
            self._rate_window_start = now
            self._rate_window_count = 0

        if not self._callback:
            return

        # -- Policy: latest (drop if busy) --
        if self._policy == "latest" and self._in_callback:
            self._drop_count += 1
            return

        # -- Policy: throttle (rate limit) --
        if self._policy == "throttle":
            if now - self._last_deliver_ts < self._throttle_interval:
                self._drop_count += 1
                return
            self._last_deliver_ts = now

        # -- Policy: sample (every Nth) --
        if self._policy == "sample":
            self._sample_counter += 1
            if self._sample_counter < self._sample_n:
                self._drop_count += 1
                return
            self._sample_counter = 0

        # -- Policy: buffer (collect batch) --
        if self._policy == "buffer":
            self._buffer.append(msg)
            if len(self._buffer) < self._buffer_size:
                return
            batch = list(self._buffer)
            self._buffer.clear()
            self._deliver_count += 1
            self._in_callback = True
            t0 = time.time()
            try:
                self._callback(batch)
            except Exception:
                self._callback_errors += 1
                logger.exception("In[%s] buffer callback error", self._name)
            finally:
                dt_ms = (time.time() - t0) * 1000.0
                self._total_callback_ms += dt_ms
                if dt_ms > self._max_callback_ms:
                    self._max_callback_ms = dt_ms
                self._in_callback = False
            return

        # -- Policy: all / latest / throttle / sample → deliver single --
        self._deliver_count += 1
        self._in_callback = True
        t0 = time.time()
        try:
            self._callback(msg)
        except Exception:
            self._callback_errors += 1
            logger.exception("In[%s] callback error", self._name)
        finally:
            dt_ms = (time.time() - t0) * 1000.0
            self._total_callback_ms += dt_ms
            if dt_ms > self._max_callback_ms:
                self._max_callback_ms = dt_ms
            self._in_callback = False

    # -- properties ----------------------------------------------------------------

    @property
    def name(self) -> str:
        return self._name

    @property
    def msg_type(self) -> type:
        return self._msg_type

    @property
    def msg_count(self) -> int:
        return self._msg_count

    @property
    def last_ts(self) -> float:
        return self._last_ts

    @property
    def latest(self) -> Optional[T]:
        """Most recent message, or None if nothing received yet."""
        return self._latest

    @property
    def connected(self) -> bool:
        """True if a callback has been registered."""
        return self._callback is not None

    @property
    def policy(self) -> str:
        """Delivery policy: "all" or "latest"."""
        return self._policy

    @property
    def drop_count(self) -> int:
        """Number of messages dropped due to backpressure (latest policy)."""
        return self._drop_count

    @property
    def rate_hz(self) -> float:
        """Estimated receive rate in Hz (2-second window)."""
        return self._rate_hz

    @property
    def deliver_count(self) -> int:
        """Number of messages actually delivered to callback."""
        return self._deliver_count

    @property
    def callback_errors(self) -> int:
        """Number of exceptions raised by callback."""
        return self._callback_errors

    @property
    def max_callback_ms(self) -> float:
        """Maximum callback execution time in milliseconds."""
        return self._max_callback_ms

    @property
    def avg_callback_ms(self) -> float:
        """Average callback execution time in milliseconds."""
        if self._deliver_count == 0:
            return 0.0
        return self._total_callback_ms / self._deliver_count

    @property
    def stale_ms(self) -> float:
        """Milliseconds since last message. -1 if never received."""
        if self._last_ts == 0.0:
            return -1.0
        return (time.time() - self._last_ts) * 1000.0

    def __repr__(self) -> str:
        status = "connected" if self._callback else "idle"
        policy_str = f", policy={self._policy}" if self._policy != "all" else ""
        drop_str = f", dropped={self._drop_count}" if self._drop_count else ""
        rate_str = f", {self._rate_hz:.1f}Hz" if self._rate_hz > 0 else ""
        return f"In('{self._name}', {self._msg_type.__name__}, n={self._msg_count}, {status}{policy_str}{drop_str}{rate_str})"
