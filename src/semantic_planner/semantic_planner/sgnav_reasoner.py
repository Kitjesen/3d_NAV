"""
SG-Nav 对齐推理器。

目标:
  1) 从层次场景图中构建子图候选 (room/group)
  2) 对子图进行分层评分 (启发式 + 可选 LLM)
  3) 将子图概率插值到 frontier, 选择探索目标
  4) 维护目标可信度 (re-perception) 以抑制假阳性目标
"""

import json
import logging
import math
import re
from dataclasses import dataclass, field
from typing import Awaitable, Callable, Dict, List, Optional, Tuple

import numpy as np

from .frontier_scorer import Frontier
from .prompt_templates import build_sgnav_subgraph_prompt

logger = logging.getLogger(__name__)


@dataclass
class SubgraphCandidate:
    """层次场景图中的可推理子图。"""

    subgraph_id: str
    level: str
    center: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0]))
    room_id: int = -1
    group_id: int = -1
    object_ids: List[int] = field(default_factory=list)
    object_labels: List[str] = field(default_factory=list)
    relation_count: int = 0

    def to_prompt_dict(self) -> Dict:
        return {
            "subgraph_id": self.subgraph_id,
            "level": self.level,
            "room_id": self.room_id,
            "group_id": self.group_id,
            "center": {
                "x": round(float(self.center[0]), 2),
                "y": round(float(self.center[1]), 2),
            },
            "object_count": len(self.object_ids),
            "object_labels": self.object_labels[:10],
            "relation_count": self.relation_count,
        }


@dataclass
class SubgraphScore:
    """子图评分结果。"""

    subgraph_id: str
    level: str
    score: float
    reason: str
    center: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0]))


@dataclass
class FrontierSelection:
    """frontier 选择结果。"""

    frontier: Frontier
    score: float
    reasoning: str
    subgraph_scores: List[SubgraphScore] = field(default_factory=list)


class SGNavReasoner:
    """SG-Nav 风格子图推理 + frontier 插值 + re-perception。"""

    def __init__(
        self,
        max_subgraphs: int = 6,
        use_llm_reasoning: bool = True,
        heuristic_weight: float = 0.45,
        llm_weight: float = 0.55,
        frontier_base_weight: float = 0.55,
        room_gate_weight: float = 0.25,
        interp_decay_distance: float = 4.0,
        credibility_decay: float = 0.9,
        false_positive_penalty: float = 0.2,
        reject_threshold: float = 0.25,
        min_confidence_for_bypass: float = 0.85,
    ):
        self.max_subgraphs = max_subgraphs
        self.use_llm_reasoning = use_llm_reasoning
        self.heuristic_weight = heuristic_weight
        self.llm_weight = llm_weight
        self.frontier_base_weight = frontier_base_weight
        self.room_gate_weight = min(max(room_gate_weight, 0.0), 1.0)
        self.interp_decay_distance = max(0.5, interp_decay_distance)

        self.credibility_decay = min(max(credibility_decay, 0.0), 0.999)
        self.false_positive_penalty = min(max(false_positive_penalty, 0.0), 0.95)
        self.reject_threshold = min(max(reject_threshold, 0.01), 0.99)
        self.min_confidence_for_bypass = min(max(min_confidence_for_bypass, 0.0), 1.0)

        self._target_credibility: Dict[str, float] = {}

    @property
    def target_credibility(self) -> Dict[str, float]:
        return dict(self._target_credibility)

    def reset(self):
        """重置可信度记忆。"""
        self._target_credibility.clear()

    async def select_frontier(
        self,
        instruction: str,
        scene_graph_json: str,
        robot_position: Dict[str, float],
        frontiers: List[Frontier],
        language: str = "zh",
        llm_chat: Optional[Callable[[List[Dict[str, str]]], Awaitable[Optional[str]]]] = None,
    ) -> Optional[FrontierSelection]:
        """执行 SG-Nav 风格 frontier 选择。"""
        if not frontiers:
            return None

        subgraph_scores = await self.reason_subgraphs(
            instruction=instruction,
            scene_graph_json=scene_graph_json,
            robot_position=robot_position,
            language=language,
            llm_chat=llm_chat,
        )

        best: Optional[Frontier] = None
        best_score = -1.0
        best_details = ""

        for frontier in frontiers:
            sg_signal, top_reason = self._interpolate_to_frontier(frontier, subgraph_scores)
            combined = (
                self.frontier_base_weight * float(frontier.score)
                + (1.0 - self.frontier_base_weight) * sg_signal
            )
            if combined > best_score:
                best_score = combined
                best = frontier
                best_details = (
                    f"base={frontier.score:.2f}, sg={sg_signal:.2f}, combined={combined:.2f}; "
                    f"{top_reason}"
                )

        if best is None:
            return None

        return FrontierSelection(
            frontier=best,
            score=float(best_score),
            reasoning=best_details,
            subgraph_scores=subgraph_scores,
        )

    async def reason_subgraphs(
        self,
        instruction: str,
        scene_graph_json: str,
        robot_position: Dict[str, float],
        language: str = "zh",
        llm_chat: Optional[Callable[[List[Dict[str, str]]], Awaitable[Optional[str]]]] = None,
    ) -> List[SubgraphScore]:
        """对子图执行评分 (heuristic + optional LLM)。"""
        candidates = self._extract_subgraphs(scene_graph_json)
        if not candidates:
            return []

        heuristic_scores, heuristic_reasons = self._score_subgraphs_heuristic(
            instruction=instruction,
            candidates=candidates,
            robot_position=robot_position,
        )

        llm_scores: Dict[str, float] = {}
        if self.use_llm_reasoning and llm_chat is not None:
            llm_scores = await self._score_subgraphs_llm(
                instruction=instruction,
                candidates=candidates,
                language=language,
                llm_chat=llm_chat,
            )

        outputs: List[SubgraphScore] = []
        for c in candidates:
            h = heuristic_scores.get(c.subgraph_id, 0.0)
            l = llm_scores.get(c.subgraph_id)
            if l is None:
                score = h
                reason = f"heuristic={h:.2f}; {heuristic_reasons.get(c.subgraph_id, '')}"
            else:
                score = self.heuristic_weight * h + self.llm_weight * l
                reason = (
                    f"heuristic={h:.2f}, llm={l:.2f}; "
                    f"{heuristic_reasons.get(c.subgraph_id, '')}"
                )

            outputs.append(
                SubgraphScore(
                    subgraph_id=c.subgraph_id,
                    level=c.level,
                    score=float(min(max(score, 0.0), 1.0)),
                    reason=reason,
                    center=c.center,
                )
            )

        outputs.sort(key=lambda s: s.score, reverse=True)
        outputs = self._apply_room_level_gating(outputs, candidates)
        outputs.sort(key=lambda s: s.score, reverse=True)
        return outputs

    def _apply_room_level_gating(
        self,
        outputs: List[SubgraphScore],
        candidates: List[SubgraphCandidate],
    ) -> List[SubgraphScore]:
        """
        room->(group/view/object) 层次门控。

        对非 room 子图的分数加入对应 room 分数, 形成逐层递进效果。
        """
        if not outputs or self.room_gate_weight <= 0.0:
            return outputs

        room_score_by_room_id: Dict[int, float] = {}
        candidate_map = {c.subgraph_id: c for c in candidates}

        for s in outputs:
            c = candidate_map.get(s.subgraph_id)
            if c is None:
                continue
            if s.level == "room" and c.room_id >= 0:
                room_score_by_room_id[c.room_id] = max(
                    room_score_by_room_id.get(c.room_id, 0.0),
                    s.score,
                )

        if not room_score_by_room_id:
            return outputs

        adjusted: List[SubgraphScore] = []
        for s in outputs:
            c = candidate_map.get(s.subgraph_id)
            if c is None or s.level == "room" or c.room_id < 0:
                adjusted.append(s)
                continue

            room_score = room_score_by_room_id.get(c.room_id)
            if room_score is None:
                adjusted.append(s)
                continue

            gated = (1.0 - self.room_gate_weight) * s.score + self.room_gate_weight * room_score
            s.score = float(min(max(gated, 0.0), 1.0))
            s.reason = f"{s.reason}; room_gate={room_score:.2f}"
            adjusted.append(s)

        return adjusted

    def evaluate_target_credibility(
        self,
        target_label: str,
        scene_graph_json: str,
        path_confidence: float,
        confirmed_visible: bool = False,
    ) -> Tuple[bool, float, str]:
        """BA-HSG 信念驱动的目标可信度评估 (§3.4.2)。
        
        使用场景图中物体的 Beta 信念状态 (如果可用) 来替代
        简单的 match_count / score 启发式。同时利用复合可信度
        (existence_prob, view_diversity, freshness, quality)。
        """
        key = (target_label or "").strip().lower()
        if not key:
            return False, 1.0, "empty target label"

        sg = self._parse_scene_graph(scene_graph_json)
        objects = sg.get("objects", []) if isinstance(sg.get("objects"), list) else []

        tokens = self._extract_keywords(key)
        if not tokens:
            tokens = [key]

        match_count = 0
        max_score = 0.0
        max_belief_credibility = 0.0
        sum_existence_prob = 0.0

        for obj in objects:
            if not isinstance(obj, dict):
                continue
            label = str(obj.get("label", "")).lower()
            if not label:
                continue
            if any(t in label or label in t for t in tokens):
                match_count += 1
                try:
                    max_score = max(max_score, float(obj.get("score", 0.0)))
                except (TypeError, ValueError):
                    pass
                # BA-HSG: 使用 belief 字段中的概率信息
                belief = obj.get("belief", {})
                if isinstance(belief, dict):
                    p_exist = belief.get("P_exist", 0.5)
                    obj_cred = belief.get("credibility", 0.5)
                    sum_existence_prob += p_exist
                    max_belief_credibility = max(max_belief_credibility, obj_cred)

        # BA-HSG: 混合传统证据与信念状态
        if max_belief_credibility > 0:
            # 有 BA-HSG 信念 → 使用更丰富的证据模型
            avg_exist = sum_existence_prob / max(match_count, 1)
            evidence = (
                0.3 * min(1.0, match_count / 3.0)
                + 0.3 * max_belief_credibility
                + 0.2 * avg_exist
                + 0.2 * min(1.0, max(path_confidence, 0.0))
            )
        else:
            # 回退到传统证据计算 (兼容无信念的场景图)
            evidence = (
                0.4 * min(1.0, match_count / 3.0)
                + 0.4 * min(1.0, max_score)
                + 0.2 * min(1.0, max(path_confidence, 0.0))
            )

        if confirmed_visible:
            evidence = max(evidence, 0.9)

        prev = self._target_credibility.get(key, 0.5)
        cred = prev * self.credibility_decay + evidence * (1.0 - self.credibility_decay)

        if match_count == 0 and not confirmed_visible:
            cred -= self.false_positive_penalty

        cred = min(max(cred, 0.0), 1.0)
        self._target_credibility[key] = cred

        reject = (
            cred < self.reject_threshold
            and path_confidence < self.min_confidence_for_bypass
            and not confirmed_visible
        )
        reason = (
            f"cred={cred:.2f}, matches={match_count}, max_score={max_score:.2f}, "
            f"belief_cred={max_belief_credibility:.2f}, "
            f"path_conf={path_confidence:.2f}, visible={confirmed_visible}"
        )
        return reject, cred, reason

    def _extract_subgraphs(self, scene_graph_json: str) -> List[SubgraphCandidate]:
        sg = self._parse_scene_graph(scene_graph_json)

        raw = sg.get("subgraphs", [])
        if isinstance(raw, list) and raw:
            parsed = self._parse_raw_subgraphs(raw)
            if parsed:
                return parsed[: self.max_subgraphs]

        # fallback: 用 rooms/regions + objects 构造子图
        objects = sg.get("objects", []) if isinstance(sg.get("objects", []), list) else []
        relations = sg.get("relations", []) if isinstance(sg.get("relations", []), list) else []
        obj_by_id = {
            int(o.get("id", -1)): o
            for o in objects
            if isinstance(o, dict) and isinstance(o.get("id"), (int, float))
        }

        room_like = sg.get("rooms", [])
        if not isinstance(room_like, list) or not room_like:
            room_like = sg.get("regions", []) if isinstance(sg.get("regions", []), list) else []

        candidates: List[SubgraphCandidate] = []

        if isinstance(room_like, list) and room_like:
            for idx, r in enumerate(room_like):
                if not isinstance(r, dict):
                    continue
                object_ids = []
                for oid in r.get("object_ids", []):
                    try:
                        object_ids.append(int(oid))
                    except (TypeError, ValueError):
                        continue

                labels = []
                for oid in object_ids:
                    obj = obj_by_id.get(oid)
                    if obj:
                        labels.append(str(obj.get("label", "")))

                center_xy = self._to_center_xy(r.get("center", {}))
                room_id = self._to_int(r.get("room_id"), default=self._to_int(r.get("region_id"), idx))

                rel_count = 0
                object_id_set = set(object_ids)
                for rel in relations:
                    if not isinstance(rel, dict):
                        continue
                    sid = self._to_int(rel.get("subject_id"), default=-1)
                    oid = self._to_int(rel.get("object_id"), default=-1)
                    if sid in object_id_set and oid in object_id_set:
                        rel_count += 1

                candidates.append(
                    SubgraphCandidate(
                        subgraph_id=f"room_{room_id}",
                        level="room",
                        center=center_xy,
                        room_id=room_id,
                        object_ids=object_ids,
                        object_labels=labels,
                        relation_count=rel_count,
                    )
                )

        # 若没有 room-like 信息, 用全局 fallback 子图
        if not candidates:
            labels = [str(o.get("label", "")) for o in objects if isinstance(o, dict)]
            obj_ids = [
                int(o.get("id", i))
                for i, o in enumerate(objects)
                if isinstance(o, dict)
            ]
            center = self._compute_objects_center(objects)
            candidates.append(
                SubgraphCandidate(
                    subgraph_id="room_global",
                    level="room",
                    center=center,
                    room_id=0,
                    object_ids=obj_ids,
                    object_labels=labels,
                    relation_count=len(relations),
                )
            )

        candidates.sort(key=lambda c: (len(c.object_ids), c.relation_count), reverse=True)
        return candidates[: self.max_subgraphs]

    def _parse_raw_subgraphs(self, raw_subgraphs: List[Dict]) -> List[SubgraphCandidate]:
        parsed: List[SubgraphCandidate] = []
        for sg in raw_subgraphs:
            if not isinstance(sg, dict):
                continue
            subgraph_id = str(sg.get("subgraph_id", ""))
            if not subgraph_id:
                continue
            labels = sg.get("object_labels", [])
            if not isinstance(labels, list):
                labels = []
            object_ids = sg.get("object_ids", [])
            if not isinstance(object_ids, list):
                object_ids = []

            parsed.append(
                SubgraphCandidate(
                    subgraph_id=subgraph_id,
                    level=str(sg.get("level", "room")),
                    center=self._to_center_xy(sg.get("center", {})),
                    room_id=self._to_int(sg.get("room_id"), default=-1),
                    group_id=self._to_int(sg.get("group_id"), default=-1),
                    object_ids=[self._to_int(x, default=-1) for x in object_ids],
                    object_labels=[str(x) for x in labels if str(x)],
                    relation_count=self._to_int(sg.get("relation_count"), default=0),
                )
            )

        return parsed

    def _score_subgraphs_heuristic(
        self,
        instruction: str,
        candidates: List[SubgraphCandidate],
        robot_position: Dict[str, float],
    ) -> Tuple[Dict[str, float], Dict[str, str]]:
        keywords = self._extract_keywords(instruction)
        relation_words = {
            "near", "left", "right", "front", "behind", "on", "above", "below",
            "旁", "附近", "左", "右", "前", "后", "上", "下",
        }

        robot_xy = np.array(
            [
                float(robot_position.get("x", 0.0)),
                float(robot_position.get("y", 0.0)),
            ],
            dtype=np.float64,
        )

        scores: Dict[str, float] = {}
        reasons: Dict[str, str] = {}

        for cand in candidates:
            label_text = " ".join(cand.object_labels).lower()

            hit_count = 0
            for kw in keywords:
                if kw in label_text:
                    hit_count += 1

            keyword_score = hit_count / max(len(keywords), 1)
            relation_score = min(1.0, cand.relation_count / 6.0)

            dist = float(np.linalg.norm(cand.center - robot_xy))
            distance_score = math.exp(-dist / self.interp_decay_distance)

            richness_score = min(1.0, len(cand.object_ids) / 8.0)

            relation_hint = any(w in instruction.lower() for w in relation_words)
            relation_bonus = 0.1 if relation_hint and cand.relation_count > 0 else 0.0

            score = (
                0.55 * keyword_score
                + 0.2 * relation_score
                + 0.15 * distance_score
                + 0.1 * richness_score
                + relation_bonus
            )
            score = min(max(score, 0.0), 1.0)

            scores[cand.subgraph_id] = score
            reasons[cand.subgraph_id] = (
                f"kw={keyword_score:.2f}, rel={relation_score:.2f}, "
                f"dist={distance_score:.2f}, rich={richness_score:.2f}"
            )

        return scores, reasons

    async def _score_subgraphs_llm(
        self,
        instruction: str,
        candidates: List[SubgraphCandidate],
        language: str,
        llm_chat: Callable[[List[Dict[str, str]]], Awaitable[Optional[str]]],
    ) -> Dict[str, float]:
        messages = build_sgnav_subgraph_prompt(
            instruction=instruction,
            subgraphs=[c.to_prompt_dict() for c in candidates],
            language=language,
        )

        try:
            response = await llm_chat(messages)
            if not response:
                return {}
            data = self._extract_json(response)
            raw_scores = data.get("subgraph_scores", [])
            if not isinstance(raw_scores, list):
                return {}

            out: Dict[str, float] = {}
            for item in raw_scores:
                if not isinstance(item, dict):
                    continue
                sid = str(item.get("subgraph_id", "")).strip()
                if not sid:
                    continue
                try:
                    score = float(item.get("score", 0.0))
                except (TypeError, ValueError):
                    continue
                out[sid] = min(max(score, 0.0), 1.0)
            return out
        except Exception as e:
            logger.debug("SG-Nav LLM subgraph scoring failed: %s", e)
            return {}

    def _interpolate_to_frontier(
        self,
        frontier: Frontier,
        subgraph_scores: List[SubgraphScore],
    ) -> Tuple[float, str]:
        if not subgraph_scores:
            return 0.0, "no subgraph score"

        weighted_sum = 0.0
        weight_sum = 0.0
        nearest: Optional[Tuple[SubgraphScore, float]] = None

        for sg in subgraph_scores:
            dist = float(np.linalg.norm(frontier.center_world - sg.center))
            w = math.exp(-dist / self.interp_decay_distance)
            weighted_sum += sg.score * w
            weight_sum += w

            if nearest is None or dist < nearest[1]:
                nearest = (sg, dist)

        signal = weighted_sum / max(weight_sum, 1e-6)
        signal = min(max(signal, 0.0), 1.0)

        if nearest is None:
            return signal, "no nearest subgraph"

        near_sg, near_dist = nearest
        detail = (
            f"nearest={near_sg.subgraph_id}({near_sg.level}), "
            f"sg_score={near_sg.score:.2f}, dist={near_dist:.2f}m"
        )
        return signal, detail

    def _parse_scene_graph(self, scene_graph_json: str) -> Dict:
        try:
            data = json.loads(scene_graph_json)
            if isinstance(data, dict):
                return data
        except (json.JSONDecodeError, TypeError):
            pass
        return {}

    @staticmethod
    def _compute_objects_center(objects: List[Dict]) -> np.ndarray:
        points = []
        for obj in objects:
            if not isinstance(obj, dict):
                continue
            pos = obj.get("position", {})
            if not isinstance(pos, dict):
                continue
            try:
                x = float(pos.get("x", 0.0))
                y = float(pos.get("y", 0.0))
            except (TypeError, ValueError):
                continue
            if np.isfinite(x) and np.isfinite(y):
                points.append([x, y])

        if not points:
            return np.array([0.0, 0.0], dtype=np.float64)
        arr = np.asarray(points, dtype=np.float64)
        return arr.mean(axis=0)

    @staticmethod
    def _to_int(value, default: int = 0) -> int:
        try:
            return int(value)
        except (TypeError, ValueError):
            return default

    @staticmethod
    def _to_center_xy(center_obj) -> np.ndarray:
        if not isinstance(center_obj, dict):
            return np.array([0.0, 0.0], dtype=np.float64)
        try:
            x = float(center_obj.get("x", 0.0))
            y = float(center_obj.get("y", 0.0))
        except (TypeError, ValueError):
            return np.array([0.0, 0.0], dtype=np.float64)

        if not np.isfinite(x) or not np.isfinite(y):
            return np.array([0.0, 0.0], dtype=np.float64)
        return np.array([x, y], dtype=np.float64)

    @staticmethod
    def _extract_keywords(text: str) -> List[str]:
        if not text:
            return []

        stop_words = {
            "the", "a", "an", "to", "go", "find", "near", "with", "at", "of",
            "去", "找", "到", "在", "附近", "的", "一个", "一下", "please",
        }
        tokens = re.findall(r"[A-Za-z0-9_]+|[\u4e00-\u9fff]{1,4}", text.lower())
        dedup = []
        for t in tokens:
            if t in stop_words or len(t) <= 1:
                continue
            if t not in dedup:
                dedup.append(t)
        return dedup

    @staticmethod
    def _extract_json(text: str) -> Dict:
        if not text:
            return {}

        text = text.strip()
        try:
            return json.loads(text)
        except json.JSONDecodeError:
            pass

        fenced = re.search(r"```(?:json)?\s*(.*?)\s*```", text, re.S | re.I)
        if fenced:
            try:
                return json.loads(fenced.group(1))
            except json.JSONDecodeError:
                pass

        start = text.find("{")
        end = text.rfind("}")
        if start >= 0 and end > start:
            candidate = text[start : end + 1]
            try:
                return json.loads(candidate)
            except json.JSONDecodeError:
                return {}
        return {}
