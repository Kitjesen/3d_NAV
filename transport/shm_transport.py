"""
SharedMemory Transport — 同机高速通道

机制:
  - POSIX 共享内存双缓冲
  - 控制块: 序列号 + 时间戳 + 数据长度
  - 轮询读取，支持 bytes / numpy / pickle 消息

性能: 900KB 图像 ~200us (DDS ~1300us)
限制: 仅同机通信
"""

import logging
import multiprocessing.shared_memory as pyshm
import struct
import threading
import time
from typing import Any, Callable, Optional

import numpy as np

from .core import Publisher, Subscriber, TopicConfig, Transport

logger = logging.getLogger(__name__)

# 控制块: [seq:8bytes][timestamp:8bytes][data_len:8bytes] = 24 bytes
CTRL_SIZE = 24
DEFAULT_BUF_SIZE = 4 * 1024 * 1024  # 4MB (enough for 1080p RGB)


def _shm_name(topic: str) -> str:
    """话题名 → 共享内存名 (合法化)"""
    return "lingtu_" + topic.replace("/", "_").strip("_")


class SHMPublisher(Publisher):
    """共享内存发布者 — 写入双缓冲"""

    def __init__(self, topic: TopicConfig):
        super().__init__(topic)
        buf_size = topic.buffer_size or DEFAULT_BUF_SIZE
        self._total_size = CTRL_SIZE + buf_size
        self._shm_name = _shm_name(topic.name)
        self._seq = 0

        # 创建或连接共享内存
        try:
            self._shm = pyshm.SharedMemory(
                name=self._shm_name, create=True, size=self._total_size
            )
            self._owner = True
        except FileExistsError:
            self._shm = pyshm.SharedMemory(name=self._shm_name, create=False)
            self._owner = False

        logger.info(f"[SHM-Pub] {topic.name} → {self._shm_name} ({buf_size // 1024}KB)")

    def publish(self, msg: Any) -> None:
        """发布消息 (bytes 或 numpy array)"""
        if isinstance(msg, np.ndarray):
            data = msg.tobytes()
        elif isinstance(msg, bytes):
            data = msg
        elif isinstance(msg, memoryview):
            data = bytes(msg)
        else:
            import pickle
            data = pickle.dumps(msg)

        data_len = len(data)
        if CTRL_SIZE + data_len > self._total_size:
            logger.warning(
                f"[SHM-Pub] {self._shm_name}: data {data_len} > buffer {self._total_size - CTRL_SIZE}, truncating"
            )
            data = data[: self._total_size - CTRL_SIZE]
            data_len = len(data)

        self._seq += 1
        buf = self._shm.buf

        # 写控制块
        struct.pack_into("<QdQ", buf, 0, self._seq, time.time(), data_len)
        # 写数据
        buf[CTRL_SIZE : CTRL_SIZE + data_len] = data

    def close(self) -> None:
        try:
            self._shm.close()
            if self._owner:
                self._shm.unlink()
        except Exception:
            pass


class SHMSubscriber(Subscriber):
    """共享内存订阅者 — 轮询读取"""

    def __init__(self, topic: TopicConfig, callback: Callable, poll_interval: float = 0.0005):
        super().__init__(topic, callback)
        self._shm_name = _shm_name(topic.name)
        self._poll_interval = poll_interval
        self._last_seq = 0
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._shm: Optional[pyshm.SharedMemory] = None

    def start(self) -> None:
        """启动轮询线程"""
        if self._running:
            return

        # 等待发布者创建共享内存
        for _ in range(50):  # 最多等 5 秒
            try:
                self._shm = pyshm.SharedMemory(name=self._shm_name, create=False)
                break
            except FileNotFoundError:
                time.sleep(0.1)

        if self._shm is None:
            logger.warning(f"[SHM-Sub] {self._shm_name} not found, waiting...")
            return

        self._running = True
        self._thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._thread.start()
        logger.info(f"[SHM-Sub] {self._topic.name} ← {self._shm_name}")

    def _poll_loop(self) -> None:
        buf = self._shm.buf
        while self._running:
            seq, ts, data_len = struct.unpack_from("<QdQ", buf, 0)
            if seq > self._last_seq and data_len > 0:
                self._last_seq = seq
                data = bytes(buf[CTRL_SIZE : CTRL_SIZE + data_len])
                try:
                    self._callback(data, ts)
                except Exception as e:
                    logger.error(f"[SHM-Sub] callback error: {e}")
            time.sleep(self._poll_interval)

    def close(self) -> None:
        self._running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        if self._shm:
            try:
                self._shm.close()
            except Exception:
                pass


class SHMTransport(Transport):
    """SharedMemory 传输层"""

    def __init__(self):
        self._publishers: list = []
        self._subscribers: list = []

    def create_publisher(self, topic: TopicConfig) -> SHMPublisher:
        pub = SHMPublisher(topic)
        self._publishers.append(pub)
        return pub

    def create_subscriber(self, topic: TopicConfig, callback: Callable) -> SHMSubscriber:
        sub = SHMSubscriber(topic, callback)
        sub.start()
        self._subscribers.append(sub)
        return sub

    def close(self) -> None:
        for s in self._subscribers:
            s.close()
        for p in self._publishers:
            p.close()
        self._subscribers.clear()
        self._publishers.clear()

    @property
    def name(self) -> str:
        return "shm"
