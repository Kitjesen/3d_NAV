"""
语义导航测试场景

发送自然语言指令，等待机器人到达目标物体附近。
成功条件: 机器人进入 goal_radius 范围内的任意目标物体。
失败条件: 超时或收到 FAILED 状态。
"""
import json
import time
from typing import List, Optional, Tuple

import numpy as np

from .base import Scenario


class SemanticNavScenario(Scenario):
    """语义导航测试场景。

    用法:
        scenario = SemanticNavScenario(
            instruction="导航到工厂大门",
            target_labels=["door", "gate", "大门"],
            target_positions=[(25.0, 10.0)],   # 可选: 已知目标位置
            goal_radius=2.0,
            max_time=180.0,
        )
    """

    name = "semantic_nav"
    description = "语义导航: 发送自然语言指令，检查是否到达目标物体附近"

    def __init__(
        self,
        instruction: str,
        target_labels: Optional[List[str]] = None,
        target_positions: Optional[List[Tuple[float, float]]] = None,
        goal_radius: float = 2.0,
        max_time: float = 180.0,
    ):
        super().__init__()
        self.instruction = instruction
        self.target_labels = target_labels or []
        self.target_positions = target_positions or []
        self.goal_radius = goal_radius
        self.max_time = max_time

        self._success = False
        self._failure_reason: str = ""
        self._status_history: List[dict] = []
        self._last_planner_status: str = ""

        # ROS2 订阅 (延迟初始化)
        self._status_sub = None
        self._planner_sub = None

    # ── 生命周期 ──────────────────────────────────────────────────────────────

    def setup(self, engine) -> None:
        super().setup(engine)

        # 订阅语义规划器状态
        self._subscribe_status(engine)

        # 发送语义指令
        self._send_instruction(engine)

        print(
            f"[SemanticNavScenario] instruction='{self.instruction}' "
            f"targets={self.target_labels} timeout={self.max_time}s"
        )

    def is_complete(self, engine) -> bool:
        if self.is_timeout():
            self._failure_reason = "timeout"
            return True

        if self._success:
            return True

        # 检查规划器失败状态
        if self._last_planner_status == "FAILED":
            self._failure_reason = "planner_failed"
            return True

        if engine is None:
            return False

        # 检查是否到达已知目标位置
        if self.target_positions:
            state = engine.get_robot_state()
            if state is not None:
                pos = state.position
                for tp in self.target_positions:
                    if self._distance_2d(pos, tp) <= self.goal_radius:
                        self._success = True
                        self._metrics["reached_target"] = list(tp)
                        return True

        return False

    def is_success(self, engine) -> bool:
        return self._success

    def get_metrics(self):
        metrics = super().get_metrics()
        metrics["instruction"] = self.instruction
        metrics["target_labels"] = self.target_labels
        metrics["goal_radius_m"] = self.goal_radius
        metrics["planner_status_final"] = self._last_planner_status
        if self._failure_reason:
            metrics["failure_reason"] = self._failure_reason
        return metrics

    # ── 内部 ──────────────────────────────────────────────────────────────────

    def _send_instruction(self, engine):
        """发布语义指令到 /nav/semantic/instruction."""
        try:
            from std_msgs.msg import String

            node = getattr(engine, "_bridge_node", None)
            if node is None:
                return

            pub = node.create_publisher(String, "/nav/semantic/instruction", 10)
            msg = String()
            msg.data = self.instruction
            # 延迟 1 秒发送，等导航栈就绪
            import threading
            def _delayed():
                time.sleep(1.0)
                pub.publish(msg)
                print(f"[SemanticNavScenario] instruction sent: '{self.instruction}'")
            threading.Thread(target=_delayed, daemon=True).start()
        except Exception as e:
            print(f"[SemanticNavScenario] instruction send failed: {e}")

    def _subscribe_status(self, engine):
        """订阅语义状态和规划器状态话题."""
        try:
            from std_msgs.msg import String

            node = getattr(engine, "_bridge_node", None)
            if node is None:
                return

            def _semantic_status_cb(msg):
                try:
                    data = json.loads(msg.data)
                    self._status_history.append(data)
                    # 检查语义规划器是否报告成功
                    phase = data.get("phase", "")
                    if phase in ("goal_reached", "succeeded"):
                        self._success = True
                except Exception:
                    pass

            def _planner_status_cb(msg):
                self._last_planner_status = msg.data.strip()

            self._status_sub = node.create_subscription(
                String, "/nav/semantic/status", _semantic_status_cb, 10)
            self._planner_sub = node.create_subscription(
                String, "/nav/planner_status", _planner_status_cb, 10)
        except Exception as e:
            print(f"[SemanticNavScenario] status subscribe failed: {e}")
