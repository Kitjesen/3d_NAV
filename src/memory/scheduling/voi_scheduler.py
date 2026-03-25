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

其中 ΔE[S] 使用 Shannon 信息论计算:
  - 信念建模: Beta(α, β), 其中 p = α/(α+β) = target_credibility
  - H_before = 二值熵 H(p) = -p·log₂(p) - (1-p)·log₂(1-p)
  - E[H_after] = 估计观测后的期望熵 (虚拟观测更新 Beta 参数)
  - delta_info = H_before - E[H_after]  (信息增益, 单位: bits)
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
    # Beta 分布参数 (可选, 未提供时从 credibility 推导)
    belief_alpha: float = 0.0
    belief_beta: float = 0.0
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

    # 虚拟观测强度 (用于估计 E[H_after])
    reperceive_obs_strength: float = 2.0   # 再感知等效观测次数
    slow_reason_obs_strength: float = 5.0  # 慢推理等效观测次数
    continue_obs_strength: float = 0.2     # 继续行进的偶然发现强度

    # 默认 Beta 先验浓度 (当 alpha/beta 未提供时)
    default_belief_concentration: float = 4.0


class VoIScheduler:
    """VoI 信息价值调度器 (Shannon 信息论版本)。

    核心决策逻辑:
    1. 将目标信念建模为 Beta(α, β) 分布
    2. 用 Shannon 二值熵 H(p) 度量当前不确定性
    3. 估计每个动作的信息增益 ΔH = H_before - E[H_after]
    4. 减去各动作的执行成本 (时间, 能耗, 里程)
    5. 考虑冷却约束和安全规则
    6. 选择效用最高的动作
    """

    def __init__(self, config: Optional[VoIConfig] = None):
        self._config = config or VoIConfig()
        self._decision_log: list = []

    @staticmethod
    def _binary_entropy(p: float) -> float:
        """二值 Shannon 熵 H(p) = -p·log₂(p) - (1-p)·log₂(1-p)。

        p ∈ (0, 1), 返回值 ∈ [0, 1] bits。
        边界: H(0) = H(1) = 0, H(0.5) = 1.0。
        """
        if p <= 0.0 or p >= 1.0:
            return 0.0
        return -(p * math.log2(p) + (1.0 - p) * math.log2(1.0 - p))

    @staticmethod
    def _beta_variance(alpha: float, beta: float) -> float:
        """Beta(α, β) 分布的方差 = αβ / ((α+β)²(α+β+1))。"""
        s = alpha + beta
        if s <= 0:
            return 0.25  # 最大不确定性
        return (alpha * beta) / (s * s * (s + 1.0))

    def _resolve_belief(self, state: SchedulerState) -> tuple:
        """从 SchedulerState 推导 Beta 分布参数 (α, β) 和均值 p。

        如果 state.belief_alpha > 0 且 state.belief_beta > 0, 直接使用;
        否则从 target_credibility 和默认浓度推导。
        """
        if state.belief_alpha > 0 and state.belief_beta > 0:
            alpha = state.belief_alpha
            beta = state.belief_beta
        else:
            # 从 credibility 反推: p = α/(α+β), 令 α+β = concentration
            p = max(0.01, min(0.99, state.target_credibility))
            conc = self._config.default_belief_concentration
            alpha = p * conc
            beta = (1.0 - p) * conc
        p = alpha / (alpha + beta)
        return alpha, beta, p

    def _info_gain(
        self, alpha: float, beta: float, obs_strength: float
    ) -> float:
        """计算观测的期望信息增益 (bits)。

        使用 Beta 分布方差衡量不确定性:
          U_before = Var[Beta(α, β)]
          U_after  = Var[Beta(α + n·p, β + n·(1-p))]  (期望后验)
          gain = (U_before - U_after) / U_before * H(p)

        方差减少比例 × 二值熵 = 信息增益 (归一化到 [0, 1] bits)。
        弱先验 (小 α+β) 每次观测方差减少更多 → 信息增益更大。
        """
        p = alpha / (alpha + beta)
        var_before = self._beta_variance(alpha, beta)
        if var_before <= 1e-12:
            return 0.0  # 已确定, 无信息可获

        alpha_post = alpha + obs_strength * p
        beta_post = beta + obs_strength * (1.0 - p)
        var_after = self._beta_variance(alpha_post, beta_post)

        # 方差缩减比例 ∈ [0, 1]
        var_reduction = max(0.0, (var_before - var_after) / var_before)

        # 乘以当前二值熵作为信息量的上界
        return var_reduction * self._binary_entropy(p)

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
        """continue 动作的效用: 微弱信息增益 (行进中偶然发现), 低成本。"""
        cfg = self._config
        alpha, beta, _p = self._resolve_belief(state)

        delta_info = self._info_gain(alpha, beta, cfg.continue_obs_strength)

        cost = cfg.lambda_d * cfg.cost_continue_dist
        return cfg.k_info_gain * delta_info - cost

    def _utility_reperceive(self, state: SchedulerState) -> float:
        """reperceive 动作的效用: 中等信息增益, 中等成本。

        位置方差和匹配缺失提供额外增益。
        """
        cfg = self._config
        alpha, beta, _p = self._resolve_belief(state)

        delta_info = self._info_gain(alpha, beta, cfg.reperceive_obs_strength)

        # 位置不确定性: 高方差 → 再感知可减少空间熵
        position_entropy = 0.5 * math.log2(
            max(1.0, 2.0 * math.pi * math.e * state.target_position_var)
        )
        # 归一化到 [0, 0.3] (position_var=2.0 时 ≈ 0.3)
        position_bonus = min(0.3, position_entropy * 0.1)

        # 如果匹配数为 0, 再感知价值更高 (信息论: 零观测 = 最大先验熵)
        no_match_bonus = 0.3 if state.match_count == 0 else 0.0

        total_gain = delta_info + position_bonus + no_match_bonus

        cost = (cfg.lambda_t * cfg.cost_reperceive_time
                + cfg.lambda_e * cfg.cost_reperceive_energy)
        return cfg.k_info_gain * total_gain - cost

    def _utility_slow_reason(self, state: SchedulerState) -> float:
        """slow_reason 动作的效用: 高信息增益, 高成本。

        场景复杂度和边际收益递减影响最终增益。
        """
        cfg = self._config
        alpha, beta, _p = self._resolve_belief(state)

        delta_info = self._info_gain(alpha, beta, cfg.slow_reason_obs_strength)

        # 场景复杂度: 物体越多, LLM 推理越有价值 (更多关系可发现)
        scene_complexity = min(1.0, state.total_objects / 20.0)
        complexity_bonus = 0.2 * scene_complexity

        total_gain = delta_info + complexity_bonus

        # 已多次慢推理 → 边际收益递减 (信息论: 重复观测的边际信息趋零)
        diminishing = 1.0 / (1.0 + 0.3 * state.slow_reason_count)
        total_gain *= diminishing

        cost = (cfg.lambda_t * cfg.cost_slow_time
                + cfg.lambda_e * cfg.cost_slow_energy)
        return cfg.k_info_gain * total_gain - cost

    def _log_decision(
        self, state: SchedulerState, action: SchedulerAction, reason: str
    ) -> None:
        alpha, beta, p = self._resolve_belief(state)
        entry = {
            "time": time.time(),
            "action": action.value,
            "cred": round(state.target_credibility, 3),
            "entropy_bits": round(self._binary_entropy(p), 4),
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
