"""
目标解析器 — 将自然语言指令 + 场景图 → 3D 目标坐标。

核心流程 (VLingNav 2026 双进程 + ESCA 选择性 Grounding):

  ┌─ Fast Path (System 1, 无需 LLM):
  │   场景图直接匹配 → 高置信度 → 直接输出坐标
  │   参考: VLingNav (arXiv 2601.08665) AdaCoT 机制
  │         OmniNav (ICLR 2026) Fast-Slow 系统
  │
  └─ Slow Path (System 2, 调用 LLM):
      ESCA 选择性 Grounding → 过滤场景图 → LLM 推理
      参考: ESCA/SGCLIP (NeurIPS 2025) 选择性 grounding
            AdaNav (ICLR 2026) 不确定性自适应
      可选: GPT-4o Vision 视觉确认 (VLMnav 2024)

为什么 Fast Path 重要:
  - VLingNav 发现 70%+ 的导航步骤用 System 1 即可完成
  - OmniNav 的 Fast 模块支持 5 Hz 控制频率
  - 省去 LLM API 调用 → 延迟从 ~2s 降到 ~10ms, API 费用降低 90%
"""

import asyncio
import json
import logging
import math
import re
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np

from .llm_client import LLMClientBase, LLMError, LLMConfig, create_llm_client
from .prompt_templates import (
    build_goal_resolution_prompt,
    build_exploration_prompt,
    build_vision_grounding_prompt,
)

try:
    from semantic_perception.topology_graph import TopologySemGraph
except ImportError:
    TopologySemGraph = None

logger = logging.getLogger(__name__)


# ── 多源置信度权重 (AdaNav 不确定性融合) ──
WEIGHT_LABEL_MATCH = 0.35       # 标签文本匹配
WEIGHT_CLIP_SIM = 0.35          # CLIP 视觉-语言相似度
WEIGHT_DETECTOR_SCORE = 0.15    # 检测器置信度
WEIGHT_SPATIAL_HINT = 0.15      # 空间关系提示命中


@dataclass
class GoalResult:
    """目标解析结果。"""
    action: str                    # "navigate" | "explore"
    target_x: float = 0.0
    target_y: float = 0.0
    target_z: float = 0.0
    target_label: str = ""
    confidence: float = 0.0
    reasoning: str = ""
    is_valid: bool = False
    error: str = ""
    path: str = ""                 # "fast" | "slow" — 标记走了哪条路径
    candidate_id: int = -1         # BA-HSG: 候选物体 ID
    frame_id: str = "map"          # 坐标帧 (默认 map, 与 planner_node 一致)


@dataclass
class TargetHypothesis:
    """BA-HSG 多假设目标信念 (§3.4.3)。"""
    object_id: int
    label: str
    position: List[float]          # [x, y, z]
    score: float                   # Fast Path fused score
    credibility: float             # BA-HSG composite credibility
    room_match: float              # room-instruction compatibility
    posterior: float = 0.0         # P(this is the true target | history)
    verified: bool = False         # 是否已到达验证过
    rejected: bool = False         # 是否已被拒绝


class TargetBeliefManager:
    """BA-HSG 多假设目标信念管理器 (§3.4.3)。
    
    维护候选目标的后验分布，支持贝叶斯更新和期望代价选择。
    解决多实例歧义问题 (e.g., 多把椅子, 多个门)。
    """

    def __init__(self, gamma1: float = 1.0, gamma2: float = 0.5, gamma3: float = 0.3):
        self._hypotheses: List[TargetHypothesis] = []
        self._gamma1 = gamma1  # fused score weight
        self._gamma2 = gamma2  # credibility weight
        self._gamma3 = gamma3  # room match weight
        self._accept_threshold = 0.7  # 后验阈值 → 确认目标

    def init_from_candidates(
        self,
        candidates: List[Dict],
        instruction: str = "",
    ) -> None:
        """从 Fast Path 候选列表初始化后验。"""
        self._hypotheses = []
        for c in candidates:
            belief = c.get("belief", {})
            h = TargetHypothesis(
                object_id=c.get("id", -1),
                label=c.get("label", ""),
                position=c.get("position", [0, 0, 0]),
                score=c.get("fused_score", 0.5),
                credibility=belief.get("credibility", 0.5) if isinstance(belief, dict) else 0.5,
                room_match=c.get("room_match", 0.5),
            )
            self._hypotheses.append(h)
        self._compute_posterior()

    def _compute_posterior(self) -> None:
        """计算归一化后验: p_i ∝ exp(γ1·score + γ2·cred + γ3·room)。"""
        if not self._hypotheses:
            return
        import math
        log_scores = []
        for h in self._hypotheses:
            if h.rejected:
                log_scores.append(-100.0)
            else:
                log_scores.append(
                    self._gamma1 * h.score
                    + self._gamma2 * h.credibility
                    + self._gamma3 * h.room_match
                )
        max_ls = max(log_scores)
        exp_scores = [math.exp(ls - max_ls) for ls in log_scores]
        total = sum(exp_scores) or 1.0
        for h, es in zip(self._hypotheses, exp_scores):
            h.posterior = es / total

    def bayesian_update(self, object_id: int, detected: bool, clip_sim: float = 0.5) -> None:
        """贝叶斯验证更新: 到达候选附近后观测结果 (§3.4.3)。"""
        for h in self._hypotheses:
            if h.object_id == object_id:
                if detected and clip_sim > 0.7:
                    likelihood = 0.9
                elif detected:
                    likelihood = 0.4 + 0.3 * clip_sim
                else:
                    likelihood = 0.1
                    h.rejected = True
                h.posterior *= likelihood
                h.verified = True
            else:
                # 其他候选: 如果目标被确认在 object_id, 其他后验降低
                if detected and clip_sim > 0.7:
                    h.posterior *= 0.3
        # 重新归一化
        total = sum(h.posterior for h in self._hypotheses) or 1.0
        for h in self._hypotheses:
            h.posterior /= total

    def select_next_target(
        self,
        robot_position: Optional[List[float]] = None,
        beta: float = 0.5,
        rho: float = 0.2,
    ) -> Optional[TargetHypothesis]:
        """期望代价选择: argmin(nav_cost - β·posterior + ρ·info_gain) (§3.4.3)。"""
        active = [h for h in self._hypotheses if not h.rejected and not h.verified]
        if not active:
            # 所有候选已验证或拒绝 → 返回后验最高的已验证候选
            verified = [h for h in self._hypotheses if h.verified and not h.rejected]
            return max(verified, key=lambda h: h.posterior) if verified else None

        best = None
        best_utility = -float("inf")
        for h in active:
            nav_cost = 0.0
            if robot_position:
                dx = h.position[0] - robot_position[0]
                dy = h.position[1] - robot_position[1]
                nav_cost = (dx ** 2 + dy ** 2) ** 0.5
            # 信息增益: 到达该候选后能区分其他候选的能力
            info_gain = 0.0
            for h2 in active:
                if h2.object_id != h.object_id:
                    d = sum((a - b) ** 2 for a, b in zip(h.position, h2.position)) ** 0.5
                    if d < 3.0:
                        info_gain += 0.3  # 附近有其他候选 → 一次验证可区分多个
            utility = beta * h.posterior - nav_cost / 10.0 + rho * info_gain
            if utility > best_utility:
                best_utility = utility
                best = h
        return best

    @property
    def best_hypothesis(self) -> Optional[TargetHypothesis]:
        """后验最高的未拒绝假设。"""
        active = [h for h in self._hypotheses if not h.rejected]
        return max(active, key=lambda h: h.posterior) if active else None

    @property
    def is_converged(self) -> bool:
        """后验是否已收敛 (最高后验 > 阈值)。"""
        best = self.best_hypothesis
        return best is not None and best.posterior > self._accept_threshold

    @property
    def num_active(self) -> int:
        return sum(1 for h in self._hypotheses if not h.rejected)


class GoalResolver:
    """
    目标解析器: VLingNav 双进程 + ESCA 选择性 Grounding + AdaNav 置信度融合。
    """

    def __init__(
        self,
        primary_config: LLMConfig,
        fallback_config: Optional[LLMConfig] = None,
        confidence_threshold: float = 0.6,
        fast_path_threshold: float = 0.75,   # Fast 路径最低置信度
        max_replan_attempts: int = 3,
    ):
        self._primary = create_llm_client(primary_config)
        self._fallback = (
            create_llm_client(fallback_config) if fallback_config else None
        )
        self._confidence_threshold = confidence_threshold
        self._fast_path_threshold = fast_path_threshold
        self._max_replan_attempts = max_replan_attempts

        # 探索状态
        self._explored_directions: List[Dict[str, float]] = []
        self._explore_step_count = 0
        self._visited_room_ids: set = set()

        # BA-HSG: 多假设目标信念管理器
        self._belief_manager = TargetBeliefManager()

        # 创新4: 语义先验引擎 (拓扑感知探索)
        from .semantic_prior import SemanticPriorEngine
        self._semantic_prior_engine = SemanticPriorEngine()

        # 创新5: 拓扑语义图 (Topology Semantic Graph)
        self._tsg: Optional["TopologySemGraph"] = None
        if TopologySemGraph is not None:
            self._tsg = TopologySemGraph()

    # ================================================================
    #  Fast Path — System 1 (VLingNav / OmniNav)
    # ================================================================

    def fast_resolve(
        self,
        instruction: str,
        scene_graph_json: str,
        robot_position: Optional[Dict[str, float]] = None,
        clip_encoder=None,
    ) -> Optional[GoalResult]:
        """
        Fast Path: 场景图直接匹配, 无需 LLM。

        参考:
          - VLingNav (2026): AdaCoT — 简单情况用 System 1
          - OmniNav (ICLR 2026): Fast 模块 5 Hz waypoint
          - AdaNav (ICLR 2026): 高确定性 → 跳过深度推理

        匹配算法:
          1. 从场景图提取物体列表
          2. 提取指令主语 (目标物体) 与修饰语 (空间参考物)
          3. 多源评分: 主语匹配 + CLIP + 检测分 + 空间关系
          4. 最高分 > fast_path_threshold → 直接返回目标
          5. 否则 → 返回 None, 交给 Slow Path

        Args:
            instruction: 用户指令
            scene_graph_json: 场景图 JSON
            robot_position: 当前位置
            clip_encoder: CLIP 编码器 (可选)

        Returns:
            GoalResult or None (None = 需要 Slow Path)
        """
        try:
            sg = json.loads(scene_graph_json)
        except (json.JSONDecodeError, TypeError):
            return None

        objects = sg.get("objects", [])
        if not objects:
            return None

        # GAP: 逐层 CLIP 筛选 (FSR-VLN 路线 A) — 先筛 Room 再筛 Object
        allowed_obj_ids = self._get_object_ids_in_top_rooms(
            instruction, sg, clip_encoder, top_k=2
        )
        if allowed_obj_ids is not None:  # None = 未启用/无 rooms
            objects = [o for o in objects if o.get("id") in allowed_obj_ids]
            if not objects:
                return None

        relations = sg.get("relations", [])
        inst_lower = instruction.lower()

        # ── 提取指令关键词 ──
        keywords = self._extract_keywords(instruction)

        # ── 提取主语 (目标) 与修饰语 (空间参考物) ──
        # "find chair near the door" → subject="chair", modifier="door"
        # "去门旁边的椅子"           → subject="椅子",  modifier="门"
        subject_labels, modifier_labels = self._parse_instruction_roles(
            inst_lower, keywords, [o.get("label", "").lower() for o in objects]
        )

        # ── 对每个物体打分 (AdaNav 多源置信度) ──
        scored: List[Tuple[dict, float, str]] = []

        for obj in objects:
            label = obj.get("label", "").lower()
            score = obj.get("score", 0.5)
            det_count = obj.get("detection_count", 1)

            # 源 1: 标签文本匹配 — 区分主语 vs 修饰语
            label_score = 0.0
            is_subject = False  # 是否匹配为指令的主语 (真正目标)

            # 主语匹配 (目标物体, 高分)
            for subj in subject_labels:
                if subj == label:
                    label_score = 1.0
                    is_subject = True
                    break
                if subj in label or label in subj:
                    label_score = max(label_score, 0.9)
                    is_subject = True

            # 如果不是主语, 检查是否是修饰语 (空间参考物, 低分)
            if not is_subject:
                for mod in modifier_labels:
                    if mod == label or mod in label or label in mod:
                        label_score = max(label_score, 0.3)  # 修饰语低分
                        break

            # 通用关键词匹配 (兜底)
            if label_score == 0.0:
                for kw in keywords:
                    if kw in label or label in kw:
                        label_score = max(label_score, 0.5)

            if label_score == 0.0:
                continue  # 完全不相关, 跳过

            # 源 2: 检测器置信度 (多次观测 → 更可靠)
            detector_score = min(score, 1.0) * min(det_count / 3, 1.0)

            # 源 3: CLIP 视觉-语言相似度
            clip_score = 0.0
            has_real_clip = False
            if clip_encoder is not None and obj.get("clip_feature") is not None:
                try:
                    clip_feature = np.array(obj.get("clip_feature"))
                    if clip_feature.size > 0:
                        similarities = clip_encoder.text_image_similarity(
                            instruction, [clip_feature]
                        )
                        clip_score = similarities[0] if similarities else 0.0
                        has_real_clip = True
                except Exception as e:
                    logger.warning("CLIP similarity computation failed: %s", e)
            # 无真实CLIP时不使用伪造近似 — 将权重重分配给其他真实信号

            # 源 4: 空间关系提示
            # 修复: 区分主体(要找的)和参考物(用于定位的)
            # "find chair near door" → chair是主体, door是参考物
            # 只有主体应该获得空间关系加分
            spatial_score = 0.0
            for rel in relations:
                obj_id = obj.get("id")
                if rel.get("subject_id") == obj_id or rel.get("object_id") == obj_id:
                    # 判断当前物体是关系的主语还是宾语
                    is_subject = (rel.get("subject_id") == obj_id)

                    related_id = (
                        rel["object_id"] if is_subject
                        else rel["subject_id"]
                    )
                    related_obj = next(
                        (o for o in objects if o.get("id") == related_id), None
                    )
                    if related_obj:
                        related_label = related_obj.get("label", "").lower()

                        # 提取核心词（去掉颜色等修饰词）
                        # "red chair" → "chair", "blue door" → "door"
                        label_core = self._extract_core_noun(label)
                        related_core = self._extract_core_noun(related_label)

                        # 关键修复: 检查核心词是否在指令中
                        # "go to chair near door" 应该匹配 "red chair" near "door"
                        label_in_inst = label_core in inst_lower or label in inst_lower
                        related_in_inst = related_core in inst_lower or related_label in inst_lower

                        if label_in_inst and related_in_inst:
                            # 检查指令中的语义: 哪个是主体，哪个是参考
                            # "find chair near door" → "chair"在"door"前面 → chair是主体
                            label_pos = inst_lower.find(label_core if label_core in inst_lower else label)
                            related_pos = inst_lower.find(related_core if related_core in inst_lower else related_label)

                            if label_pos < related_pos:
                                # 当前物体在前 → 是主体 → 给高分
                                spatial_score = 1.0
                                break
                            else:
                                # 当前物体在后 → 是参考物 → 给低分
                                spatial_score = 0.2

                        # 通用近距离关系加分(保底)
                        elif rel.get("relation") == "near":
                            spatial_score = max(spatial_score, 0.3)

            # 综合评分 (AdaNav 风格加权融合)
            if has_real_clip:
                # 4源完整融合
                fused_score = (
                    WEIGHT_LABEL_MATCH * label_score
                    + WEIGHT_CLIP_SIM * clip_score
                    + WEIGHT_DETECTOR_SCORE * detector_score
                    + WEIGHT_SPATIAL_HINT * spatial_score
                )
            else:
                # 无CLIP: 重分配权重 — 不伪造数据
                # 标签匹配 0.55, 检测器 0.25, 空间 0.20
                fused_score = (
                    0.55 * label_score
                    + 0.25 * detector_score
                    + 0.20 * spatial_score
                )

            clip_tag = f"clip={clip_score:.2f}" if has_real_clip else "clip=N/A"
            reason = (
                f"label={label_score:.1f}, {clip_tag}, "
                f"det={detector_score:.2f}, spatial={spatial_score:.1f} → fused={fused_score:.2f}"
            )
            scored.append((obj, fused_score, reason))

        if not scored:
            return None

        # 取最高分
        scored.sort(key=lambda x: x[1], reverse=True)
        best_obj, best_score, best_reason = scored[0]

        # ── B7: CLIP 属性消歧 (区分 "red chair" vs "blue chair") ──
        # 当存在多个同类型高分候选时, 用 CLIP 做精细排序
        if clip_encoder is not None and len(scored) >= 2:
            top_candidates = [
                (obj, sc, r) for obj, sc, r in scored[:5]
                if sc > best_score * 0.8
            ]
            if len(top_candidates) >= 2:
                # 检查是否同类型 (核心名词相同, 只是属性不同)
                core_labels = [
                    self._extract_core_noun(o.get("label", "")).lower()
                    for o, _, _ in top_candidates
                ]
                if len(set(core_labels)) == 1:
                    # 同类型多个候选: 用 CLIP 区分属性
                    clip_ranked = self._clip_attribute_disambiguate(
                        instruction, top_candidates, clip_encoder
                    )
                    if clip_ranked:
                        best_obj, best_score, best_reason = clip_ranked[0]
                        best_reason += " [CLIP-attr-disambig]"

        # ── 距离衰减 (近距离目标优先) ──
        if robot_position:
            pos = best_obj.get("position", {})
            dx = pos.get("x", 0) - robot_position.get("x", 0)
            dy = pos.get("y", 0) - robot_position.get("y", 0)
            dist = math.sqrt(dx * dx + dy * dy)
            # 如果有相近分数但更近的候选, 考虑切换
            for obj2, sc2, _ in scored[1:3]:
                if sc2 > best_score * 0.9:  # 分数差距 < 10%
                    pos2 = obj2.get("position", {})
                    dx2 = pos2.get("x", 0) - robot_position.get("x", 0)
                    dy2 = pos2.get("y", 0) - robot_position.get("y", 0)
                    dist2 = math.sqrt(dx2 * dx2 + dy2 * dy2)
                    if dist2 < dist * 0.5:  # 近一倍以上 → 切换
                        best_obj, best_score, best_reason = obj2, sc2, _
                        break

        # ── 判断是否够格走 Fast Path ──
        if best_score < self._fast_path_threshold:
            logger.info(
                "Fast path score %.2f < threshold %.2f, deferring to Slow path. "
                "Best: %s (%s)",
                best_score, self._fast_path_threshold,
                best_obj.get("label", "?"), best_reason,
            )
            return None  # 不够确定, 交给 LLM

        # ── BA-HSG: 初始化多假设目标信念 ──
        if len(scored) >= 2:
            candidates_for_belief = []
            for obj_dict, fused_sc, _ in scored[:8]:
                pos_d = obj_dict.get("position", {})
                candidates_for_belief.append({
                    "id": obj_dict.get("id", -1),
                    "label": obj_dict.get("label", ""),
                    "position": [pos_d.get("x", 0), pos_d.get("y", 0), pos_d.get("z", 0)],
                    "fused_score": fused_sc,
                    "belief": obj_dict.get("belief", {}),
                    "room_match": 0.5,
                })
            self._belief_manager.init_from_candidates(candidates_for_belief, instruction)

            # 使用 Belief-GoalNav 多假设选择
            robot_pos = [robot_position.get("x", 0), robot_position.get("y", 0)]
            selected = self._belief_manager.select_next_target(robot_pos)
            if selected and not self._belief_manager.is_converged:
                logger.info(
                    "🎯 BA-HSG multi-hypothesis: %d active candidates, "
                    "top posterior=%.3f (%s)",
                    self._belief_manager.num_active,
                    selected.posterior,
                    selected.label,
                )

        pos = best_obj.get("position", {})
        logger.info(
            "⚡ Fast path hit: '%s' at (%.2f, %.2f), score=%.2f [%s]",
            best_obj.get("label", "?"),
            pos.get("x", 0), pos.get("y", 0),
            best_score, best_reason,
        )

        # 从场景图获取坐标系，默认为 map
        target_frame = sg.get("frame_id", "map")

        return GoalResult(
            action="navigate",
            target_x=pos.get("x", 0),
            target_y=pos.get("y", 0),
            target_z=pos.get("z", 0),
            target_label=best_obj.get("label", ""),
            confidence=best_score,
            reasoning=f"Fast path: {best_reason}",
            is_valid=True,
            path="fast",
            candidate_id=best_obj.get("id", -1),
            frame_id=target_frame,
        )

    @staticmethod
    def _extract_core_noun(label: str) -> str:
        """
        提取标签的核心名词（去掉颜色等修饰词）。

        例如:
            "red chair" → "chair"
            "blue door" → "door"
            "fire extinguisher" → "fire extinguisher" (保持不变)

        Args:
            label: 物体标签

        Returns:
            核心名词
        """
        # 常见颜色词
        colors = {
            "red", "blue", "green", "yellow", "white", "black", "gray", "grey",
            "orange", "purple", "pink", "brown", "cyan", "magenta",
            "红色", "蓝色", "绿色", "黄色", "白色", "黑色", "灰色",
            "橙色", "紫色", "粉色", "棕色", "红", "蓝", "绿", "黄", "白", "黑", "灰"
        }

        # 分词
        words = label.split()

        # 去掉颜色词
        core_words = [w for w in words if w.lower() not in colors]

        if core_words:
            return " ".join(core_words)
        else:
            # 如果全是颜色词，返回原标签
            return label

    @staticmethod
    def _get_object_ids_in_top_rooms(
        instruction: str,
        scene_graph: dict,
        clip_encoder,
        top_k: int = 2,
    ) -> Optional[set]:
        """
        GAP: Room CLIP 筛选 (FSR-VLN 路线 A)。
        用 instruction↔Room 相似度取 top-k 房间，返回其 object_ids 集合。
        无 clip/rooms 时返回 None（不做筛选）。
        """
        if clip_encoder is None:
            return None
        subgraphs = scene_graph.get("subgraphs", [])
        rooms = [s for s in subgraphs if s.get("level") == "room"]
        if len(rooms) < 2:
            return None
        obj_by_id = {
            o.get("id"): o for o in scene_graph.get("objects", [])
            if o.get("id") is not None
        }
        room_texts = []
        for r in rooms:
            labels = r.get("object_labels", [])
            if not labels:
                oids = r.get("object_ids", [])
                labels = [obj_by_id.get(oid, {}).get("label", "") for oid in oids]
            room_texts.append(" ".join(str(l) for l in labels[:10] if l))
        if not room_texts:
            return None
        try:
            sims = clip_encoder.text_text_similarity(instruction, room_texts)
        except Exception:
            return None
        if not sims or len(sims) != len(rooms):
            return None
        ranked = sorted(range(len(rooms)), key=lambda i: sims[i], reverse=True)
        allowed = set()
        for i in ranked[:top_k]:
            allowed.update(rooms[i].get("object_ids", []))
        return allowed

    @staticmethod
    def _clip_attribute_disambiguate(
        instruction: str,
        candidates: List[Tuple[dict, float, str]],
        clip_encoder,
    ) -> List[Tuple[dict, float, str]]:
        """
        B7: 用 CLIP 对同类型多候选做属性消歧。

        当场景中有 "red chair" 和 "blue chair" 而指令说 "找红色椅子" 时,
        CLIP 能通过视觉-语言对齐区分属性。

        Args:
            instruction: 用户指令
            candidates: (object_dict, score, reason) 列表
            clip_encoder: CLIPEncoder 实例

        Returns:
            按 CLIP 相似度重排后的候选列表
        """
        clip_scored = []
        for obj, fused_score, reason in candidates:
            clip_feature = obj.get("clip_feature")
            if clip_feature is not None:
                try:
                    feat = np.array(clip_feature)
                    if feat.size > 0:
                        sims = clip_encoder.text_image_similarity(
                            instruction, [feat]
                        )
                        clip_sim = sims[0] if sims else 0.0
                        # 融合: 70% 原始分 + 30% CLIP属性匹配
                        combined = 0.7 * fused_score + 0.3 * clip_sim
                        new_reason = (
                            f"{reason}, clip_attr={clip_sim:.3f}, "
                            f"combined={combined:.3f}"
                        )
                        clip_scored.append((obj, combined, new_reason))
                        continue
                except Exception:
                    pass
            clip_scored.append((obj, fused_score, reason))

        clip_scored.sort(key=lambda x: x[1], reverse=True)
        return clip_scored

    @staticmethod
    def _extract_keywords(instruction: str) -> List[str]:
        """
        从指令中提取关键词 (使用jieba精确分词)。

        升级说明 (P0任务):
        - 原实现: 简单regex分词，中文按字符组
        - 新实现: jieba精确分词，支持自定义词典
        - 回退: jieba未安装时自动回退到简单分词

        参考: SEMANTIC_NAV_REPORT.md 第11.1节
        """
        import re

        stop_words = {
            "the", "a", "an", "to", "go", "find", "get", "me", "for", "and", "or",
            "is", "at", "in", "on", "near", "next", "by", "of", "with", "from",
            "去", "到", "找", "拿", "的", "在", "旁边", "附近", "那个",
            "请", "帮", "我", "一个", "把", "了", "着", "过",
        }

        # 先分离中英文 — 避免jieba在混合文本上的切分错误
        chinese_parts = re.findall(r'[\u4e00-\u9fff]+', instruction)
        english_parts = re.findall(r'[a-zA-Z]+', instruction.lower())
        chinese_text = " ".join(chinese_parts)

        all_keywords: List[str] = []

        # 英文: 简单分词 (停用词过滤 + 长度过滤)
        for w in english_parts:
            w_lower = w.lower()
            if w_lower not in stop_words and len(w_lower) > 1:
                all_keywords.append(w_lower)

        # 中文: 使用jieba精确分词 (如果有的话)
        if chinese_text:
            try:
                from .chinese_tokenizer import extract_keywords
                zh_keywords = extract_keywords(
                    chinese_text,
                    min_length=2,
                    filter_stopwords=True,
                    keep_colors=True,
                    keep_spatial=True,
                )
                all_keywords.extend(zh_keywords)
            except ImportError:
                # 回退: 中文按连续字符组
                zh_tokens = re.findall(r'[\u4e00-\u9fff]+', chinese_text)
                all_keywords.extend(
                    t for t in zh_tokens if t not in stop_words and len(t) > 1
                )

        return list(set(all_keywords))

    def _parse_instruction_roles(
        self,
        inst_lower: str,
        keywords: List[str],
        scene_labels: List[str],
    ) -> Tuple[List[str], List[str]]:
        """
        从指令中解析主语 (导航目标) 和修饰语 (空间参考物)。

        B6 升级: 三级解析策略
          1. 规则匹配 (regex) — 最快, 处理常见句式
          2. 场景标签顺序匹配 — 兜底
          3. LLM 回退 — 复杂指令 (如 "找到书房里靠窗的那把红色椅子")

        Args:
            inst_lower: 指令小写
            keywords: 提取的关键词
            scene_labels: 场景图中所有物体标签 (小写)

        Returns:
            (subject_labels, modifier_labels)
        """
        # 第 1 级: 规则匹配
        subjects, modifiers = self._parse_roles_regex(inst_lower, keywords, scene_labels)
        if subjects:
            return subjects, modifiers

        # 第 2 级: 场景标签顺序匹配
        subjects, modifiers = self._parse_roles_scene_order(inst_lower, keywords, scene_labels)
        if subjects:
            return subjects, modifiers

        # 跳过 Level 3 LLM: 当关键词和场景标签完全无交集时 (如中英跨语言)
        # LLM 角色解析不会有帮助, 且浪费 API 调用
        has_any_overlap = any(
            kw in lbl or lbl in kw
            for kw in keywords
            for lbl in scene_labels
        )
        if not has_any_overlap:
            logger.debug(
                "Skipping LLM role parsing: no keyword-label overlap "
                "(cross-lingual scenario)"
            )
            return keywords[:], []

        # 第 3 级: LLM 回退 (异步 → 在同步上下文中跑)
        try:
            import asyncio
            loop = asyncio.get_event_loop()
            if loop.is_running():
                import concurrent.futures
                with concurrent.futures.ThreadPoolExecutor() as pool:
                    future = pool.submit(
                        asyncio.run,
                        self._parse_roles_llm(inst_lower, scene_labels),
                    )
                    subjects, modifiers = future.result(timeout=5.0)
            else:
                subjects, modifiers = loop.run_until_complete(
                    self._parse_roles_llm(inst_lower, scene_labels)
                )
            if subjects:
                logger.info("LLM role parsing: subjects=%s, modifiers=%s", subjects, modifiers)
                return subjects, modifiers
        except Exception as e:
            logger.debug("LLM role parsing failed (non-critical): %s", e)

        # 全部当主语
        return keywords[:], []

    @staticmethod
    def _parse_roles_regex(
        inst_lower: str,
        keywords: List[str],
        scene_labels: List[str],
    ) -> Tuple[List[str], List[str]]:
        """第 1 级: 正则匹配常见句式。"""
        subjects: List[str] = []
        modifiers: List[str] = []

        en_patterns = [
            r'\b(?:find|go\s+to|navigate\s+to|locate|get)\s+([\w\s]+?)\s+(?:near|by|beside|next\s+to|behind|in\s+front\s+of|left\s+of|right\s+of|on|under|above|below)\s+(?:the\s+)?([\w\s]+)',
            r'\b([\w]+)\s+(?:near|by|beside|next\s+to)\s+(?:the\s+)?([\w]+)',
        ]

        for pat in en_patterns:
            m = re.search(pat, inst_lower)
            if m:
                subj_str = m.group(1).strip()
                mod_str = m.group(2).strip()
                for lbl in scene_labels:
                    if lbl in subj_str or subj_str in lbl:
                        subjects.append(lbl)
                    if lbl in mod_str or mod_str in lbl:
                        modifiers.append(lbl)
                if subjects:
                    return list(set(subjects)), list(set(modifiers))

        zh_patterns = [
            r'([\u4e00-\u9fff]+?)(?:旁边|附近|左边|右边|前面|后面|上面|下面|对面|里面)的([\u4e00-\u9fff]+)',
            r'(?:去|到|找|找到|寻找)([\u4e00-\u9fff]+)',
        ]

        for i, pat in enumerate(zh_patterns):
            m = re.search(pat, inst_lower)
            if m:
                if i == 0:
                    mod_str = m.group(1)
                    subj_str = m.group(2)
                    for lbl in scene_labels:
                        if lbl in subj_str or subj_str in lbl:
                            subjects.append(lbl)
                        if lbl in mod_str or mod_str in lbl:
                            modifiers.append(lbl)
                    if subjects:
                        return list(set(subjects)), list(set(modifiers))
                else:
                    subj_str = m.group(1)
                    for lbl in scene_labels:
                        if lbl in subj_str or subj_str in lbl:
                            subjects.append(lbl)
                    if subjects:
                        return list(set(subjects)), []

        return [], []

    @staticmethod
    def _parse_roles_scene_order(
        inst_lower: str,
        keywords: List[str],
        scene_labels: List[str],
    ) -> Tuple[List[str], List[str]]:
        """第 2 级: 按指令中出现顺序, 第一个匹配场景物体的词为主语。"""
        found_in_scene = []
        for kw in keywords:
            for lbl in scene_labels:
                if kw in lbl or lbl in kw:
                    found_in_scene.append(lbl)
                    break
        for lbl in scene_labels:
            if lbl in inst_lower and lbl not in found_in_scene:
                found_in_scene.append(lbl)

        if found_in_scene:
            return [found_in_scene[0]], found_in_scene[1:]
        return [], []

    async def _parse_roles_llm(
        self,
        inst_lower: str,
        scene_labels: List[str],
    ) -> Tuple[List[str], List[str]]:
        """
        第 3 级: LLM 解析复杂指令中的主语和修饰语。

        B6 新增: 处理规则无法覆盖的复杂句式, 例如:
          - "找到书房里靠窗的那把红色椅子"
          - "go to the second room and find the cup on the table near the window"
        """
        labels_str = ", ".join(scene_labels[:30])
        messages = [
            {
                "role": "system",
                "content": (
                    "You are a robot navigation instruction parser. "
                    "Given an instruction and a list of visible object labels, "
                    "identify the TARGET objects (what the robot should go to) "
                    "and REFERENCE objects (spatial landmarks used for locating the target). "
                    "Reply in strict JSON: {\"targets\": [...], \"references\": [...]}"
                ),
            },
            {
                "role": "user",
                "content": (
                    f"Instruction: \"{inst_lower}\"\n"
                    f"Visible objects: [{labels_str}]\n"
                    f"Identify targets and references."
                ),
            },
        ]

        response = await self._call_with_fallback(messages)
        if not response:
            return [], []

        try:
            data = self._extract_json(response)
            targets = [
                str(t).lower() for t in data.get("targets", [])
                if isinstance(t, str)
            ]
            references = [
                str(r).lower() for r in data.get("references", [])
                if isinstance(r, str)
            ]
            # 映射回场景标签
            matched_targets = [
                lbl for lbl in scene_labels
                if any(t in lbl or lbl in t for t in targets)
            ]
            matched_refs = [
                lbl for lbl in scene_labels
                if any(r in lbl or lbl in r for r in references)
            ]
            return list(set(matched_targets)), list(set(matched_refs))
        except Exception:
            return [], []

    # ================================================================
    #  BA-HSG: 多假设验证与重选 (§3.4.3)
    # ================================================================

    def verify_and_reselect(
        self,
        object_id: int,
        detected: bool,
        clip_sim: float = 0.5,
        robot_position: Optional[List[float]] = None,
    ) -> Optional[GoalResult]:
        """到达候选目标后执行贝叶斯更新, 如需要则重选目标。
        
        Args:
            object_id: 当前验证的物体 ID
            detected: 到达后是否检测到目标
            clip_sim: CLIP 相似度
            robot_position: 当前机器人位置 [x, y]
            
        Returns:
            新的 GoalResult 如果需要重选, 否则 None (当前目标确认)
        """
        if not self._belief_manager._hypotheses:
            return None

        self._belief_manager.bayesian_update(object_id, detected, clip_sim)

        best = self._belief_manager.best_hypothesis
        if best is None:
            return None

        if self._belief_manager.is_converged and best.verified:
            logger.info(
                "✅ BA-HSG belief converged: '%s' posterior=%.3f",
                best.label, best.posterior,
            )
            return None

        # 需要验证下一个候选
        next_target = self._belief_manager.select_next_target(robot_position)
        if next_target is None:
            return None

        logger.info(
            "🔄 BA-HSG reselect: switching to '%s' (posterior=%.3f, %d active)",
            next_target.label, next_target.posterior,
            self._belief_manager.num_active,
        )
        return GoalResult(
            action="navigate",
            target_x=next_target.position[0],
            target_y=next_target.position[1],
            target_z=next_target.position[2],
            target_label=next_target.label,
            confidence=next_target.posterior,
            reasoning=f"BA-HSG reselect: posterior={next_target.posterior:.3f}",
            is_valid=True,
            path="fast",
            candidate_id=next_target.object_id,
            frame_id="map",  # BA-HSG 多假设验证使用 map 坐标系
        )

    # ================================================================
    #  Slow Path — System 2 (LLM 推理)
    # ================================================================

    async def resolve(
        self,
        instruction: str,
        scene_graph_json: str,
        robot_position: Optional[Dict[str, float]] = None,
        language: str = "zh",
        explore_if_unknown: bool = True,
        clip_encoder=None,
    ) -> GoalResult:
        """
        完整解析 (Fast → Slow 双进程)。

        流程:
          1. 先尝试 Fast Path (VLingNav System 1)
          2. Fast 失败 → 选择性 Grounding 过滤场景图 (ESCA)
          3. 过滤后的场景图 → LLM 推理 (Slow Path)

        Args:
            instruction: 用户自然语言指令
            scene_graph_json: 场景图 JSON
            robot_position: 当前机器人位置
            language: "zh" / "en"
            explore_if_unknown: 目标未知时是否自动探索
            clip_encoder: CLIP 编码器 (可选)

        Returns:
            GoalResult
        """
        # ── Step 1: Fast Path (VLingNav) ──
        fast_result = self.fast_resolve(
            instruction, scene_graph_json, robot_position, clip_encoder
        )
        if fast_result is not None:
            return fast_result

        # ── Step 2: 选择性 Grounding (ESCA + D1 CLIP 排序) ──
        filtered_sg = self._selective_grounding(
            instruction, scene_graph_json, clip_encoder=clip_encoder
        )

        # ── Step 3: Slow Path (LLM) ──
        messages = build_goal_resolution_prompt(
            instruction, filtered_sg, robot_position, language
        )

        response_text = await self._call_with_fallback(messages)
        if response_text is None:
            return GoalResult(
                action="error",
                error="All LLM backends failed",
                is_valid=False,
            )

        result = self._parse_llm_response(response_text, json.loads(filtered_sg) if isinstance(filtered_sg, str) else filtered_sg)
        result.path = "slow"

        # 如果需要探索
        if (
            explore_if_unknown
            and result.action == "explore"
            and robot_position is not None
        ):
            logger.info(
                "Target unknown, generating exploration waypoint. "
                "Reason: %s", result.reasoning,
            )
            self._explored_directions.append(
                {"x": result.target_x, "y": result.target_y}
            )

        return result

    async def generate_exploration_waypoint(
        self,
        instruction: str,
        robot_position: Dict[str, float],
        step_distance: float = 2.0,
        language: str = "zh",
        scene_graph_json: str = "",
    ) -> GoalResult:
        """
        生成拓扑感知探索航点 (创新5 升级: TSG 信息增益探索)。

        双层策略:
          Layer 1 (TSG): 如果拓扑语义图可用, 用 Algorithm 2 (Information Gain)
                         选择最优探索目标, 无需 LLM 调用 (~1ms)
          Layer 2 (LLM): 如果 TSG 不可用/评分过低, fallback 到 LLM 探索建议

        参考:
          - TopoNav (2025): 拓扑图作为空间记忆
          - L3MVN (IROS 2023): LLM-guided frontier scoring
          - SG-Nav: 子图评分插值到 frontier
        """
        # ── Layer 1: TSG 信息增益探索 (创新5) ──
        tsg_result = self._try_tsg_exploration(
            instruction, robot_position, scene_graph_json, step_distance,
        )
        if tsg_result is not None:
            return tsg_result

        # ── Layer 2: LLM 探索建议 (创新4 原方案) ──
        topology_context = None
        semantic_priors = None

        if scene_graph_json:
            try:
                sg = json.loads(scene_graph_json)
                rooms = sg.get("rooms", [])
                topology_edges = sg.get("topology_edges", [])

                if rooms:
                    priors = self._semantic_prior_engine.get_unexplored_priors(
                        target_instruction=instruction,
                        rooms=rooms,
                        topology_edges=topology_edges,
                        visited_room_ids=self._visited_room_ids,
                        current_room_id=self._get_current_room_id(
                            robot_position, rooms
                        ),
                    )
                    if priors:
                        semantic_priors = priors[:5]

                if topology_edges:
                    topo_parts = []
                    room_names = {
                        r.get("room_id", -1): r.get("name", "?")
                        for r in rooms
                    }
                    for te in topology_edges[:8]:
                        fr = te.get("from_room", -1)
                        to = te.get("to_room", -1)
                        fn = room_names.get(fr, f"room_{fr}")
                        tn = room_names.get(to, f"room_{to}")
                        visited_f = "✓" if fr in self._visited_room_ids else "?"
                        visited_t = "✓" if to in self._visited_room_ids else "?"
                        topo_parts.append(
                            f"{fn}[{visited_f}] ↔ {tn}[{visited_t}] ({te.get('type', '?')})"
                        )
                    topology_context = "\n".join(topo_parts)

                # 补充 TSG 拓扑摘要到 LLM 上下文
                if self._tsg:
                    tsg_context = self._tsg.to_prompt_context(language)
                    if tsg_context:
                        topology_context = (
                            (topology_context or "") + "\n" + tsg_context
                        ).strip()
            except (json.JSONDecodeError, TypeError, KeyError):
                pass

        messages = build_exploration_prompt(
            instruction, self._explored_directions, robot_position, language,
            topology_context=topology_context,
            semantic_priors=semantic_priors,
        )

        response_text = await self._call_with_fallback(messages)
        if response_text is None:
            angle = np.random.uniform(0, 2 * np.pi)
            return GoalResult(
                action="explore",
                target_x=robot_position["x"] + step_distance * np.cos(angle),
                target_y=robot_position["y"] + step_distance * np.sin(angle),
                target_z=robot_position.get("z", 0.0),
                reasoning="Random exploration (LLM unavailable)",
                confidence=0.1,
                is_valid=True,
            )

        try:
            data = self._extract_json(response_text)
            direction = data.get("explore_direction", {})
            dx = float(direction.get("x", 0)) - robot_position["x"]
            dy = float(direction.get("y", 0)) - robot_position["y"]
            norm = np.sqrt(dx**2 + dy**2)
            if norm > 0:
                dx = dx / norm * step_distance
                dy = dy / norm * step_distance

            self._explore_step_count += 1
            self._explored_directions.append({
                "x": robot_position["x"] + dx,
                "y": robot_position["y"] + dy,
            })

            return GoalResult(
                action="explore",
                target_x=robot_position["x"] + dx,
                target_y=robot_position["y"] + dy,
                target_z=robot_position.get("z", 0.0),
                reasoning=data.get("reasoning", "LLM exploration suggestion"),
                confidence=0.3,
                is_valid=True,
            )
        except Exception as e:
            logger.warning("Failed to parse exploration response: %s", e)
            return GoalResult(
                action="explore",
                error=str(e),
                is_valid=False,
            )

    def _try_tsg_exploration(
        self,
        instruction: str,
        robot_position: Dict[str, float],
        scene_graph_json: str,
        step_distance: float,
    ) -> Optional["GoalResult"]:
        """
        尝试使用拓扑语义图 (TSG) 选择探索目标。

        如果 TSG 可用且返回高置信度探索目标, 直接返回 GoalResult;
        否则返回 None 让调用者 fallback 到 LLM。
        """
        if self._tsg is None or not scene_graph_json:
            return None

        try:
            sg = json.loads(scene_graph_json)
        except (json.JSONDecodeError, TypeError):
            return None

        # 同步 TSG
        self._tsg.update_from_scene_graph(sg)

        # 注入前沿节点 (from scene graph)
        frontier_nodes = sg.get("frontier_nodes", [])
        if frontier_nodes:
            frontier_points = []
            frontier_sizes = []
            for fn in frontier_nodes:
                pos = fn.get("position", {})
                frontier_points.append(np.array([pos.get("x", 0), pos.get("y", 0)]))
                frontier_sizes.append(fn.get("frontier_size", 2.0))
            self._tsg.update_frontiers_from_costmap(frontier_points, frontier_sizes)

            # 预测前沿房间类型 (利用语义先验)
            for fnode in self._tsg.frontiers:
                nearest_room_id = None
                for edge in self._tsg._adjacency.get(fnode.node_id, []):
                    other_id = (
                        edge.to_id if edge.from_id == fnode.node_id else edge.from_id
                    )
                    other = self._tsg.get_node(other_id)
                    if other and other.node_type == "room":
                        nearest_room_id = other.node_id
                        break
                if nearest_room_id is not None:
                    room_node = self._tsg.get_node(nearest_room_id)
                    if room_node:
                        fnode.predicted_room_type = self._predict_adjacent_room_type(
                            room_node.room_type
                        )

        # 记录机器人位置 (触发房间切换检测)
        room_id = self._tsg.record_robot_position(
            robot_position["x"], robot_position["y"]
        )
        if room_id is not None:
            self._visited_room_ids.add(room_id)

        # 运行 Algorithm 2: Information Gain Exploration
        targets = self._tsg.get_best_exploration_target(
            instruction, self._semantic_prior_engine, top_k=3,
        )

        if not targets:
            return None

        best = targets[0]

        # 只有评分足够高才使用 TSG 结果 (避免低质量探索)
        if best.score < 0.05:
            return None

        dx = best.position[0] - robot_position["x"]
        dy = best.position[1] - robot_position["y"]
        norm = np.sqrt(dx**2 + dy**2)
        if norm > step_distance:
            dx = dx / norm * step_distance
            dy = dy / norm * step_distance

        self._explore_step_count += 1
        self._explored_directions.append({
            "x": robot_position["x"] + dx,
            "y": robot_position["y"] + dy,
        })

        logger.info(
            "TSG exploration: %s (score=%.3f, IG=%.3f, hops=%d) — %s",
            best.node_name, best.score, best.information_gain,
            best.hops, best.reasoning,
        )

        # 从场景图获取坐标系
        frame_id = sg.get("frame_id", "map") if 'sg' in locals() else "map"

        return GoalResult(
            action="explore",
            target_x=robot_position["x"] + dx,
            target_y=robot_position["y"] + dy,
            target_z=robot_position.get("z", 0.0),
            target_label=best.node_name,
            reasoning=f"[TSG-IG] {best.reasoning}",
            confidence=min(0.5, best.score),
            is_valid=True,
            path="fast",
            frame_id=frame_id,
        )

    @staticmethod
    def _predict_adjacent_room_type(current_room_type: str) -> str:
        """基于当前房间类型预测相邻房间类型 (空间常识)。"""
        adjacency_priors = {
            "corridor": "office",
            "office": "corridor",
            "kitchen": "corridor",
            "meeting_room": "corridor",
            "bathroom": "corridor",
            "stairwell": "corridor",
            "lobby": "corridor",
            "storage": "corridor",
        }
        return adjacency_priors.get(current_room_type, "")

    async def vision_grounding(
        self,
        instruction: str,
        scene_graph_json: str,
        image_base64: str,
        language: str = "zh",
    ) -> Dict:
        """
        视觉 grounding — 发送相机帧给 GPT-4o Vision。

        参考 VLMnav (2024): 当场景图匹配置信度不够时,
        直接让 VLM 看图判断目标是否可见。

        使用场景:
          - resolve() 返回低置信度时
          - 探索阶段到达新视角时
          - 用户指令包含视觉属性 ("红色的", "坏掉的") 时

        Args:
            instruction: 用户指令
            scene_graph_json: 场景图 JSON
            image_base64: JPEG base64 编码
            language: "zh" / "en"

        Returns:
            dict: {target_visible, position_in_frame, confidence, reasoning}
        """
        from .llm_client import OpenAIClient

        # Vision 只能用 OpenAI (GPT-4o) 后端
        client = None
        if isinstance(self._primary, OpenAIClient):
            client = self._primary
        elif self._fallback and isinstance(self._fallback, OpenAIClient):
            client = self._fallback

        if client is None or not hasattr(client, "chat_with_image"):
            logger.warning("No OpenAI client available for vision grounding")
            return {"target_visible": False, "confidence": 0.0, "reasoning": "No vision backend"}

        system, user_text = build_vision_grounding_prompt(
            instruction, scene_graph_json, language
        )

        try:
            response = await client.chat_with_image(
                text_prompt=user_text,
                image_base64=image_base64,
                system_prompt=system,
            )
            return self._extract_json(response)
        except Exception as e:
            logger.error("Vision grounding failed: %s", e)
            return {"target_visible": False, "confidence": 0.0, "reasoning": str(e)}

    # ================================================================
    #  选择性 Grounding (ESCA, NeurIPS 2025)
    # ================================================================

    def _selective_grounding(
        self,
        instruction: str,
        scene_graph_json: str,
        max_objects: int = 15,
        max_relations: int = 20,
        clip_encoder=None,
    ) -> str:
        """
        选择性 Grounding: 只给 LLM 与指令相关的场景子图。

        D1 升级: CLIP 语义排序替换纯关键词匹配。

        参考:
          - ESCA / SGCLIP (NeurIPS 2025):
            "selective grounding — identifying only contextually relevant
             objects and relationships"
          - MSGNav (2025): "key subgraph selection enables efficient reasoning"

        策略:
          1. CLIP 语义排序 (如可用): 计算所有物体标签与指令的语义相似度
          2. 关键词匹配 (兜底): 标签含指令关键词的物体 → 必选
          3. 关系链扩展: 必选物体的 1-hop 邻居 → 加入 (SG-Nav)
          4. 区域内物体: 指令提到的区域 → 加入
          5. 限制总数避免 token 爆炸

        Args:
            instruction: 用户指令
            scene_graph_json: 完整场景图 JSON
            max_objects: 最多保留物体数
            max_relations: 最多保留关系数
            clip_encoder: CLIP 编码器 (可选, D1 新增)

        Returns:
            过滤后的场景图 JSON
        """
        try:
            sg = json.loads(scene_graph_json)
        except (json.JSONDecodeError, TypeError):
            return scene_graph_json

        objects = sg.get("objects", [])
        relations = sg.get("relations", [])
        regions = sg.get("regions", [])

        if len(objects) <= max_objects:
            return scene_graph_json

        keywords = self._extract_keywords(instruction)
        inst_lower = instruction.lower()

        # ── D1: CLIP 语义排序 (替代纯关键词) ──
        clip_relevance: Dict[int, float] = {}
        if clip_encoder is not None:
            try:
                labels = [obj.get("label", "") for obj in objects]
                if labels:
                    # 批量计算指令与所有物体标签的语义相似度
                    text_features = []
                    for lbl in labels:
                        text_features.append(lbl)
                    sims = clip_encoder.text_text_similarity(instruction, text_features)
                    if sims is None or len(sims) == 0:
                        # text_text 不可用, 尝试 text_image
                        for idx, obj in enumerate(objects):
                            feat = obj.get("clip_feature")
                            if feat is not None:
                                f = np.array(feat)
                                if f.size > 0:
                                    s = clip_encoder.text_image_similarity(instruction, [f])
                                    if s:
                                        clip_relevance[obj.get("id", idx)] = s[0]
                    else:
                        for idx, (obj, sim) in enumerate(zip(objects, sims)):
                            clip_relevance[obj.get("id", idx)] = float(sim)
            except Exception as e:
                logger.debug("CLIP selective grounding failed (falling back to keywords): %s", e)

        # ── 第 1 轮: CLIP + 关键词联合筛选 ──
        relevant_ids = set()
        relevance_scores: Dict[int, float] = {}

        for obj in objects:
            oid = obj.get("id")
            label = obj.get("label", "").lower()

            # CLIP 相似度
            clip_sim = clip_relevance.get(oid, 0.0)

            # 关键词匹配分
            kw_score = 0.0
            if label in inst_lower or inst_lower in label:
                kw_score = 1.0
            else:
                for kw in keywords:
                    if kw in label or label in kw:
                        kw_score = max(kw_score, 0.8)

            # 综合: 有 CLIP 时 CLIP 权重 0.6 + 关键词 0.4; 无 CLIP 时纯关键词
            if clip_relevance:
                combined = 0.6 * clip_sim + 0.4 * kw_score
            else:
                combined = kw_score

            relevance_scores[oid] = combined
            if combined > 0.3:  # 相关性阈值
                relevant_ids.add(oid)

        # ── 第 2 轮: 关系链 1-hop 扩展 (SG-Nav 层次推理) ──
        hop1_ids = set()
        for rel in relations:
            sid = rel.get("subject_id")
            oid = rel.get("object_id")
            if sid in relevant_ids:
                hop1_ids.add(oid)
            if oid in relevant_ids:
                hop1_ids.add(sid)
        relevant_ids |= hop1_ids

        # ── 第 3 轮: 区域内物体 (如果指令提到了区域) ──
        for region in regions:
            region_name = region.get("name", "").lower()
            if any(kw in region_name for kw in keywords):
                relevant_ids |= set(region.get("object_ids", []))

        # ── 第 4 轮: 如果仍然为空, 取 CLIP 排序 top + 最高分物体 ──
        if not relevant_ids:
            if relevance_scores:
                sorted_by_relevance = sorted(
                    objects,
                    key=lambda o: relevance_scores.get(o.get("id"), 0.0),
                    reverse=True,
                )
            else:
                sorted_by_relevance = sorted(
                    objects,
                    key=lambda o: (o.get("score", 0) * o.get("detection_count", 1)),
                    reverse=True,
                )
            relevant_ids = {o["id"] for o in sorted_by_relevance[:max_objects]}

        # ── 构建过滤后的场景图 ──
        filtered_objects = [
            o for o in objects if o["id"] in relevant_ids
        ][:max_objects]
        filtered_obj_ids = {o["id"] for o in filtered_objects}

        filtered_relations = [
            r for r in relations
            if r.get("subject_id") in filtered_obj_ids
            and r.get("object_id") in filtered_obj_ids
        ][:max_relations]

        filtered_regions = [
            r for r in regions
            if any(oid in filtered_obj_ids for oid in r.get("object_ids", []))
        ]

        # 重建摘要
        summary = sg.get("summary", "")
        if filtered_objects:
            labels = [o["label"] for o in filtered_objects[:10]]
            summary = (
                f"Filtered {len(filtered_objects)}/{len(objects)} relevant objects: "
                + ", ".join(labels)
            )

        result = {
            "timestamp": sg.get("timestamp", 0),
            "object_count": len(filtered_objects),
            "objects": filtered_objects,
            "relations": filtered_relations,
            "regions": filtered_regions,
            "summary": summary,
            "_filter_note": f"ESCA selective grounding: {len(objects)}→{len(filtered_objects)} objects",
        }

        logger.debug(
            "Selective grounding: %d→%d objects, %d→%d relations",
            len(objects), len(filtered_objects),
            len(relations), len(filtered_relations),
        )

        return json.dumps(result, ensure_ascii=False)

    def reset_exploration(self):
        """重置探索状态 (新任务时调用)。"""
        self._explored_directions.clear()
        self._explore_step_count = 0
        self._visited_room_ids.clear()
        if self._tsg is not None:
            self._tsg = TopologySemGraph() if TopologySemGraph is not None else None

    def update_visited_room(self, room_id: int) -> None:
        """标记某房间已探索 (拓扑感知探索用)。"""
        if room_id >= 0:
            self._visited_room_ids.add(room_id)

    @property
    def topology_graph(self) -> Optional["TopologySemGraph"]:
        """获取拓扑语义图实例 (供外部模块访问)。"""
        return self._tsg

    @staticmethod
    def _get_current_room_id(
        robot_position: Dict[str, float],
        rooms: List[Dict],
    ) -> int:
        """根据机器人位置找到当前所在的房间 ID。"""
        if not rooms:
            return -1
        robot_xy = np.array([
            robot_position.get("x", 0.0),
            robot_position.get("y", 0.0),
        ])
        best_id = -1
        best_dist = float("inf")
        for room in rooms:
            center = room.get("center", {})
            rx = float(center.get("x", 0.0))
            ry = float(center.get("y", 0.0))
            dist = float(np.linalg.norm(robot_xy - np.array([rx, ry])))
            if dist < best_dist:
                best_dist = dist
                best_id = room.get("room_id", -1)
        return best_id

    # ================================================================
    #  内部方法
    # ================================================================

    async def _call_with_fallback(
        self, messages: List[Dict[str, str]]
    ) -> Optional[str]:
        """调用主 LLM, 失败则尝试备用。"""
        # 主 LLM
        if self._primary.is_available():
            try:
                return await self._primary.chat(messages)
            except LLMError as e:
                logger.warning("Primary LLM failed: %s", e)

        # 备用 LLM
        if self._fallback and self._fallback.is_available():
            try:
                logger.info("Trying fallback LLM...")
                return await self._fallback.chat(messages)
            except LLMError as e:
                logger.error("Fallback LLM also failed: %s", e)

        return None

    def _parse_llm_response(self, response_text: str, scene_graph: dict = None) -> GoalResult:
        """解析 LLM JSON 响应。"""
        try:
            data = self._extract_json(response_text)

            action = data.get("action", "navigate")
            target = data.get("target", {})

            # 从场景图获取坐标系，默认为 map
            frame_id = "map"
            if scene_graph is not None:
                frame_id = scene_graph.get("frame_id", "map")

            return GoalResult(
                action=action,
                target_x=float(target.get("x", 0)),
                target_y=float(target.get("y", 0)),
                target_z=float(target.get("z", 0)),
                target_label=data.get("target_label", ""),
                confidence=float(data.get("confidence", 0)),
                reasoning=data.get("reasoning", ""),
                is_valid=True,
                frame_id=frame_id,
            )
        except Exception as e:
            logger.error("Failed to parse LLM response: %s\nRaw: %s", e, response_text)
            return GoalResult(
                action="error",
                error=f"Parse error: {e}",
                is_valid=False,
            )

    @staticmethod
    def _extract_json(text: str) -> dict:
        """从 LLM 输出中提取 JSON (处理 markdown 代码块 + 截断修复)。"""
        candidates = []

        # 尝试找 JSON 代码块
        match = re.search(r"```(?:json)?\s*([\s\S]*?)```", text)
        if match:
            candidates.append(match.group(1).strip())

        # 找第一个 { 和最后一个 }
        start = text.find("{")
        end = text.rfind("}")
        if start != -1 and end != -1:
            candidates.append(text[start:end + 1])

        for candidate in candidates:
            try:
                return json.loads(candidate)
            except json.JSONDecodeError:
                pass

        # 截断修复: LLM 输出可能在 reasoning 字段中被截断
        for candidate in candidates:
            try:
                fixed = GoalResolver._fix_truncated_json(candidate)
                if fixed:
                    return json.loads(fixed)
            except json.JSONDecodeError:
                pass

        raise ValueError(f"No JSON found in response: {text[:200]}")

    @staticmethod
    def _fix_truncated_json(text: str) -> Optional[str]:
        """尝试修复被截断的 JSON (常见于 max_tokens 限制)。"""
        required_keys = {"action", "target", "confidence"}
        try:
            json.loads(text)
            return text
        except json.JSONDecodeError:
            pass

        # 检查是否包含必要的键
        if not all(f'"{k}"' in text for k in required_keys):
            return None

        # 尝试在不同位置截断并闭合 JSON
        # 策略: 找到最后一个完整的 key-value 对, 截断 reasoning 字段
        reasoning_match = re.search(r'"reasoning"\s*:\s*"', text)
        if reasoning_match:
            prefix = text[:reasoning_match.start()]
            # 检查 prefix 中是否有足够的字段
            if '"confidence"' in prefix:
                cleaned = prefix.rstrip().rstrip(",")
                if not cleaned.endswith("}"):
                    cleaned += "}"
                try:
                    return json.loads(cleaned)
                except json.JSONDecodeError:
                    pass

        # 策略 2: 暴力闭合
        for trim_pos in range(len(text) - 1, max(len(text) - 200, 0), -1):
            ch = text[trim_pos]
            if ch in (',', '"', '}'):
                snippet = text[:trim_pos]
                # 闭合打开的字符串
                if snippet.count('"') % 2 == 1:
                    snippet += '"'
                # 闭合打开的对象
                open_braces = snippet.count('{') - snippet.count('}')
                snippet += "}" * max(open_braces, 0)
                try:
                    result = json.loads(snippet)
                    if isinstance(result, dict) and required_keys.issubset(result.keys()):
                        return snippet
                except json.JSONDecodeError:
                    continue

        return None
