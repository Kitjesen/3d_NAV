"""lingtu.core.stream — 类型化数据流端口与可插拔传输层。

受 dimos Stream/Transport 模式启发，简化为 LingTu 导航系统所需的核心：

- Transport (Protocol) — 进程间 / 跨节点通信抽象
- LocalTransport       — 进程内零拷贝总线 (用于测试和单进程仿真)
- Out[T]               — 输出端口，发布消息
- In[T]                — 输入端口，接收消息

设计原则：
  1. 零外部依赖 (纯 Python typing + threading)
  2. 与 msgs 层解耦 — T 可以是任意类型
  3. Transport 可选 — Out/In 可纯本地回调模式使用
"""

from __future__ import annotations

import logging
import threading
import time
from typing import (
    Any,
    Callable,
    Dict,
    Generic,
    List,
    Optional,
    Protocol,
    TypeVar,
    runtime_checkable,
)

logger = logging.getLogger(__name__)

T = TypeVar("T")


# ---------------------------------------------------------------------------
# Transport Protocol
# ---------------------------------------------------------------------------

@runtime_checkable
class Transport(Protocol):
    """传输层抽象协议。

    任何实现 publish / subscribe / close 的对象均可作为传输后端，
    包括 ROS2、DDS、SHM、gRPC 等。
    """

    def publish(self, topic: str, msg: Any) -> None:
        """发布消息到指定 topic。"""
        ...

    def subscribe(self, topic: str, cb: Callable[[Any], None]) -> None:
        """订阅指定 topic，收到消息时调用 cb。"""
        ...

    def close(self) -> None:
        """释放资源。"""
        ...


# ---------------------------------------------------------------------------
# LocalTransport — 进程内零拷贝总线
# ---------------------------------------------------------------------------

class LocalTransport:
    """进程内同步总线，用于单进程模式和测试。

    消息零拷贝传递（直接引用传递），无序列化开销。
    线程安全：内部使用 Lock 保护订阅列表。
    """

    def __init__(self) -> None:
        self._bus: Dict[str, List[Callable[[Any], None]]] = {}
        self._lock = threading.Lock()

    def publish(self, topic: str, msg: Any) -> None:
        """发布消息，同步调用所有订阅者回调。"""
        with self._lock:
            callbacks = list(self._bus.get(topic, []))
        for cb in callbacks:
            try:
                cb(msg)
            except Exception:
                logger.exception("LocalTransport callback error on topic '%s'", topic)

    def subscribe(self, topic: str, cb: Callable[[Any], None]) -> None:
        """订阅 topic。"""
        with self._lock:
            self._bus.setdefault(topic, []).append(cb)

    def unsubscribe(self, topic: str, cb: Callable[[Any], None]) -> None:
        """取消订阅。"""
        with self._lock:
            cbs = self._bus.get(topic)
            if cbs and cb in cbs:
                cbs.remove(cb)

    def close(self) -> None:
        """清空所有订阅。"""
        with self._lock:
            self._bus.clear()

    @property
    def topics(self) -> List[str]:
        """返回当前活跃的 topic 列表。"""
        with self._lock:
            return list(self._bus.keys())

    def subscriber_count(self, topic: str) -> int:
        """返回指定 topic 的订阅者数量。"""
        with self._lock:
            return len(self._bus.get(topic, []))

    def __repr__(self) -> str:
        return f"LocalTransport(topics={len(self._bus)})"


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
        self._msg_count += 1
        self._last_ts = time.time()
        # 本地回调
        with self._lock:
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
    """类型化输入端口。

    使用 subscribe(cb) 注册消息处理回调。
    消息通过 _deliver(msg) 送达（通常由 Out._add_callback 或 Transport 触发）。

    支持两种接收模式:
    1. 被动回调 — subscribe(cb) 后，由上游 Out 或 Transport 推送
    2. 缓冲最新值 — latest 属性保存最近一条消息
    """

    __slots__ = (
        "_name", "_msg_type", "_callback", "_msg_count",
        "_last_ts", "_latest", "_lock",
    )

    def __init__(self, name: str, msg_type: type) -> None:
        self._name = name
        self._msg_type = msg_type
        self._callback: Optional[Callable[[T], None]] = None
        self._msg_count: int = 0
        self._last_ts: float = 0.0
        self._latest: Optional[T] = None
        self._lock = threading.Lock()

    # -- 核心 API ----------------------------------------------------------------

    def subscribe(self, cb: Callable[[T], None]) -> None:
        """注册消息处理回调。后注册会覆盖先前回调。"""
        self._callback = cb

    def _deliver(self, msg: T) -> None:
        """投递消息（由 Out 回调或 Transport 调用）。"""
        self._msg_count += 1
        self._last_ts = time.time()
        self._latest = msg
        if self._callback:
            try:
                self._callback(msg)
            except Exception:
                logger.exception("In[%s] callback error", self._name)

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
        """最近收到的一条消息，未收到过则为 None。"""
        return self._latest

    @property
    def connected(self) -> bool:
        """是否已注册回调。"""
        return self._callback is not None

    def __repr__(self) -> str:
        status = "connected" if self._callback else "idle"
        return f"In('{self._name}', {self._msg_type.__name__}, n={self._msg_count}, {status})"
