"""
VoI (Value of Information) 驱动的快慢推理与再感知调度器 (BA-HSG §3.4.4)。

将 "何时慢推理 / 何时再感知 / 何时继续执行" 形式化为信息价值优化问题,
替代固定的 2m 触发启发式。

三种高层动作:
  a0 = continue:    继续 Nav2 执行一小段 Δd
  a1 = reperceive:  原地或小范围执行多方向观察, 更新场景图
  a2 = slow_reason: 触发慢推理 (LLM) 做房间/组/对象重判定

效用函数:
  U(a) = ΔE[S] - λ_t·ΔT(a) - λ_e·ΔE(a) - λ_d·Δd(a)

其中 ΔE[S] 近似为:
  k · (H(P(goal)) - E[H(P(goal) | new_obs)])
  用经验模型估计: credibility_gap, match_uncertainty, goal_variance
"""

import logging
import math
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, Optional

logger = logging.getLogger(__name__)


class SchedulerAction(Enum):
    """VoI 调度器的高层动作。"""
    CONTINUE = "continue"
    REPERCEIVE = "reperceive"
    SLOW_REASON = "slow_reason"


@dataclass
class SchedulerState:
    """调度器输入状态摘要。"""
    # 目标信念
    target_credibility: float = 0.5
    target_existence_prob: float = 0.6
    target_position_var: float = 1.0
    # 场景图统计
    match_count: int = 0
    total_objects: int = 0
    # 导航状态
    distance_to_goal: float = 5.0
    nav_accumulated_dist: float = 0.0
    distance_since_last_reperception: float = 0.0
    # 资源
    slow_reason_count: int = 0       # 本 episode 已调用慢推理次数
    reperception_count: int = 0      # 本 episode 已再感知次数
    time_elapsed: float = 0.0        # 本 episode 已用时间 (s)
    # 历史
    last_reperception_time: float = 0.0
    last_slow_reason_time: float = 0.0


@dataclass
class VoIConfig:
    """VoI 调度器可调参数。"""
    # 成本权重
    lambda_t: float = 0.3           # 时延惩罚
    lambda_e: float = 0.1           # 能耗惩罚
    lambda_d: float = 0.05          # 里程惩罚

    # 动作成本估计 (Jetson Orin NX 实测基准)
    cost_reperceive_time: float = 3.0    # 再感知耗时 (s)
    cost_reperceive_energy: float = 5.0  # 再感知能耗 (J)
    cost_slow_time: float = 5.0          # 慢推理耗时 (s)
    cost_slow_energy: float = 8.0        # 慢推理能耗 (J)
    cost_continue_dist: float = 1.0      # 继续行进距离 (m)

    # 冷却时间
    reperception_cooldown: float = 5.0   # 再感知最小间隔 (s)
    slow_reason_cooldown: float = 15.0   # 慢推理最小间隔 (s)

    # 收益估计参数
    k_info_gain: float = 1.0        # 信息增益缩放
    min_distance_for_trigger: float = 0.5  # 最小移动距离才考虑触发 (m)

    # 阈值
    credibility_safe: float = 0.7   # 可信度 > 此值 → 倾向 continue
    credibility_danger: float = 0.3 # 可信度 < 此值 → 强制 reperceive


class VoIScheduler:
    """VoI 信息价值调度器。
    
    核心决策逻辑:
    1. 根据当前信念状态估计每个动作的信息增益 ΔE[S]
    2. 减去各动作的执行成本 (时间, 能耗, 里程)
    3. 考虑冷却约束
    4. 选择效用最高的动作
    """

    def __init__(self, config: Optional[VoIConfig] = None):
        self._config = config or VoIConfig()
        self._decision_log: list = []

    def decide(self, state: SchedulerState) -> SchedulerAction:
        """根据当前状态选择最优动作。"""
        cfg = self._config
        now = time.time()

        # ── 硬约束: 冷却时间 ──
        can_reperceive = (now - state.last_reperception_time) >= cfg.reperception_cooldown
        can_slow = (now - state.last_slow_reason_time) >= cfg.slow_reason_cooldown

        # ── 安全规则: 极低可信度 → 强制再感知 ──
        if state.target_credibility < cfg.credibility_danger and can_reperceive:
            action = SchedulerAction.REPERCEIVE
            self._log_decision(state, action, "danger_threshold")
            return action

        # ── 安全规则: 高可信度 + 接近目标 → 继续 ──
        if (state.target_credibility > cfg.credibility_safe
                and state.distance_to_goal < 3.0):
            action = SchedulerAction.CONTINUE
            self._log_decision(state, action, "safe_and_close")
            return action

        # ── 最小移动距离检查 ──
        if state.distance_since_last_reperception < cfg.min_distance_for_trigger:
            action = SchedulerAction.CONTINUE
            self._log_decision(state, action, "insufficient_movement")
            return action

        # ── VoI 效用计算 ──
        u_continue = self._utility_continue(state)
        u_reperceive = self._utility_reperceive(state) if can_reperceive else -999.0
        u_slow = self._utility_slow_reason(state) if can_slow else -999.0

        utilities = {
            SchedulerAction.CONTINUE: u_continue,
            SchedulerAction.REPERCEIVE: u_reperceive,
            SchedulerAction.SLOW_REASON: u_slow,
        }

        best_action = max(utilities, key=utilities.get)

        self._log_decision(
            state, best_action,
            f"U_cont={u_continue:.3f}, U_rep={u_reperceive:.3f}, "
            f"U_slow={u_slow:.3f}",
        )
        return best_action

    def _utility_continue(self, state: SchedulerState) -> float:
        """continue 动作的效用: 无信息增益, 低成本。"""
        cfg = self._config
        # 信息增益: 行进中自动检测可能发现新物体
        delta_s = 0.05 * (1.0 - state.target_credibility)
        cost = cfg.lambda_d * cfg.cost_continue_dist
        return cfg.k_info_gain * delta_s - cost

    def _utility_reperceive(self, state: SchedulerState) -> float:
        """reperceive 动作的效用: 中等信息增益, 中等成本。"""
        cfg = self._config
        # 信息增益: 与当前不确定性成正比
        uncertainty = 1.0 - state.target_credibility
        position_info = min(1.0, state.target_position_var / 2.0)
        delta_s = (0.4 * uncertainty + 0.3 * position_info)

        # 如果匹配数为 0, 再感知价值更高
        if state.match_count == 0:
            delta_s += 0.3

        cost = (cfg.lambda_t * cfg.cost_reperceive_time
                + cfg.lambda_e * cfg.cost_reperceive_energy)
        return cfg.k_info_gain * delta_s - cost

    def _utility_slow_reason(self, state: SchedulerState) -> float:
        """slow_reason 动作的效用: 高信息增益, 高成本。"""
        cfg = self._config
        # 慢推理在 credibility 中等、场景复杂时最有价值
        uncertainty = 1.0 - state.target_credibility
        scene_complexity = min(1.0, state.total_objects / 20.0)
        delta_s = 0.5 * uncertainty + 0.2 * scene_complexity

        # 已多次慢推理 → 边际收益递减
        diminishing = 1.0 / (1.0 + 0.3 * state.slow_reason_count)
        delta_s *= diminishing

        cost = (cfg.lambda_t * cfg.cost_slow_time
                + cfg.lambda_e * cfg.cost_slow_energy)
        return cfg.k_info_gain * delta_s - cost

    def _log_decision(
        self, state: SchedulerState, action: SchedulerAction, reason: str
    ) -> None:
        entry = {
            "time": time.time(),
            "action": action.value,
            "cred": round(state.target_credibility, 3),
            "dist_to_goal": round(state.distance_to_goal, 2),
            "reason": reason,
        }
        self._decision_log.append(entry)
        # 保留最近 50 条
        if len(self._decision_log) > 50:
            self._decision_log = self._decision_log[-50:]
        logger.debug("VoI decision: %s", entry)

    @property
    def decision_stats(self) -> Dict[str, int]:
        """统计各动作的决策次数。"""
        stats: Dict[str, int] = {"continue": 0, "reperceive": 0, "slow_reason": 0}
        for entry in self._decision_log:
            action = entry.get("action", "continue")
            stats[action] = stats.get(action, 0) + 1
        return stats
