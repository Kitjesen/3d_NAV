"""
仿真场景基类

所有场景继承此类，实现 setup / is_complete / is_success 三个方法。
"""
import time
from abc import ABC, abstractmethod
from typing import Any, Dict, Optional


class Scenario(ABC):
    """导航仿真场景抽象基类。

    属性:
        name        场景唯一标识
        description 场景描述
        max_time    最大运行时间 (秒)，超时视为失败
    """

    name: str = "base"
    description: str = ""
    max_time: float = 120.0

    def __init__(self):
        self._start_time: Optional[float] = None
        self._end_time: Optional[float] = None
        self._metrics: Dict[str, Any] = {}

    # ── 生命周期 ──────────────────────────────────────────────────────────────

    def setup(self, engine) -> None:
        """场景初始化: 放置机器人、发布目标、设置障碍物等。

        Args:
            engine: 仿真引擎实例 (实现 SimEngine 接口)
        """
        self._start_time = time.time()
        self._metrics = {}

    @abstractmethod
    def is_complete(self, engine) -> bool:
        """判断场景是否结束 (成功或失败均返回 True)."""

    @abstractmethod
    def is_success(self, engine) -> bool:
        """判断场景是否成功完成."""

    def teardown(self, engine) -> None:
        """场景结束后的清理工作 (可选覆写)."""
        self._end_time = time.time()

    # ── 辅助方法 ──────────────────────────────────────────────────────────────

    def is_timeout(self) -> bool:
        """是否超过最大运行时间."""
        if self._start_time is None:
            return False
        return (time.time() - self._start_time) > self.max_time

    def elapsed(self) -> float:
        """已运行时间 (秒)."""
        if self._start_time is None:
            return 0.0
        end = self._end_time if self._end_time is not None else time.time()
        return end - self._start_time

    def get_metrics(self) -> Dict[str, Any]:
        """返回场景指标，子类应在 is_complete 返回 True 前填充 _metrics."""
        return {
            "name": self.name,
            "elapsed_sec": round(self.elapsed(), 2),
            "success": self.is_success(None) if self._end_time else None,
            **self._metrics,
        }

    # ── 内部工具 ──────────────────────────────────────────────────────────────

    def _distance_2d(self, pos_a, pos_b) -> float:
        """计算 XY 平面距离."""
        import math
        return math.sqrt((pos_a[0] - pos_b[0]) ** 2 +
                         (pos_a[1] - pos_b[1]) ** 2)
