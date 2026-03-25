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
# Out[T] — 输出端口
# ---------------------------------------------------------------------------

class Out(Generic[T]):
    """类型化输出端口。

    支持两种消息传递路径:
    1. 本地回调链 — 通过 _add_callback() 注册，publish() 直接调用
    2. 外部传输层 — 通过 _bind_transport() 绑定，publish() 时额外发送

    统计:
    - msg_count: 已发布消息总数
    - last_ts: 最后一次发布时间戳 (time.time())
    """

    __slots__ = (
        "_name", "_msg_type", "_callbacks", "_transport",
        "_transport_topic", "_msg_count", "_last_ts", "_lock",
    )

    def __init__(self, name: str, msg_type: type) -> None:
        self._name = name
        self._msg_type = msg_type
        self._callbacks: List[Callable[[T], None]] = []
        self._transport: Optional[Transport] = None
        self._transport_topic: Optional[str] = None
        self._msg_count: int = 0
        self._last_ts: float = 0.0
        self._lock = threading.Lock()

    # -- 核心 API ----------------------------------------------------------------

    def publish(self, msg: T) -> None:
        """发布一条消息到所有本地订阅者和外部传输层。"""
        # 本地回调
        with self._lock:
            self._msg_count += 1
            self._last_ts = time.time()
            cbs = list(self._callbacks)
        for cb in cbs:
            try:
                cb(msg)
            except Exception:
                logger.exception("Out[%s] callback error", self._name)
        # 外部传输
        if self._transport and self._transport_topic:
            try:
                self._transport.publish(self._transport_topic, msg)
            except Exception:
                logger.exception("Out[%s] transport publish error", self._name)

    # -- 连接 API (由 Blueprint 调用) --------------------------------------------

    def _add_callback(self, cb: Callable[[T], None]) -> None:
        """注册本地回调 (用于 Out→In 直连)。"""
        with self._lock:
            self._callbacks.append(cb)

    def _remove_callback(self, cb: Callable[[T], None]) -> None:
        """移除本地回调。"""
        with self._lock:
            if cb in self._callbacks:
                self._callbacks.remove(cb)

    def _bind_transport(self, transport: Transport, topic: Optional[str] = None) -> None:
        """绑定外部传输层。topic 默认使用端口名。"""
        self._transport = transport
        self._transport_topic = topic or self._name

    def _clear_callbacks(self) -> None:
        """Remove all callbacks and transport binding. Called on Module.stop()."""
        with self._lock:
            self._callbacks.clear()
        self._transport = None
        self._transport_topic = None

    # -- 属性 -------------------------------------------------------------------

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
    def callback_count(self) -> int:
        with self._lock:
            return len(self._callbacks)

    def __repr__(self) -> str:
        transport_str = f", transport={self._transport_topic}" if self._transport else ""
        return f"Out('{self._name}', {self._msg_type.__name__}, n={self._msg_count}{transport_str})"


# ---------------------------------------------------------------------------
# In[T] — 输入端口
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
        "_name", "_msg_type", "_callback", "_msg_count",
        "_last_ts", "_latest", "_lock", "_policy", "_in_callback",
        "_drop_count", "_throttle_interval", "_last_deliver_ts",
        "_sample_n", "_sample_counter", "_buffer_size", "_buffer",
    )

    def __init__(self, name: str, msg_type: type) -> None:
        self._name = name
        self._msg_type = msg_type
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
        self._msg_count += 1
        self._last_ts = time.time()
        self._latest = msg

        if not self._callback:
            return

        # -- Policy: latest (drop if busy) --
        if self._policy == "latest" and self._in_callback:
            self._drop_count += 1
            return

        # -- Policy: throttle (rate limit) --
        if self._policy == "throttle":
            now = time.time()
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
            self._in_callback = True
            try:
                self._callback(batch)
            except Exception:
                logger.exception("In[%s] buffer callback error", self._name)
            finally:
                self._in_callback = False
            return

        # -- Policy: all / latest / throttle / sample → deliver single --
        self._in_callback = True
        try:
            self._callback(msg)
        except Exception:
            logger.exception("In[%s] callback error", self._name)
        finally:
            self._in_callback = False

    # -- 属性 -------------------------------------------------------------------

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

    def __repr__(self) -> str:
        status = "connected" if self._callback else "idle"
        policy_str = f", policy={self._policy}" if self._policy != "all" else ""
        drop_str = f", dropped={self._drop_count}" if self._drop_count else ""
        return f"In('{self._name}', {self._msg_type.__name__}, n={self._msg_count}, {status}{policy_str}{drop_str})"
