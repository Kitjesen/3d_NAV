"""
语义先验引擎 — 房间功能→物体联想 + 拓扑感知探索评分。

参考论文:
  - Hydra (RSS 2022): 层次3D场景图, Places→Rooms 社区检测, 房间拓扑连通
  - Concept-Guided Exploration (2025): Room + Door 作为自治概念, 层次约束传播
  - SG-Nav (NeurIPS 2024): 子图推理 + LLM 常识
  - L3MVN (ICRA 2024): LLM-guided 拓扑探索

核心思想 (创新4: Topology-Aware Semantic Exploration):
  1. 语义联想: 房间类型 → 可能包含的物体 (带概率)
  2. 物体→房间反向推理: "找灭火器" → 走廊/楼梯间概率最高
  3. 拓扑感知: 哪些房间已探索、哪些未探索、房间之间如何连通
  4. 探索策略: 优先去"语义先验最高 + 尚未探索"的方向
"""

import logging
import re
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Set, Tuple

logger = logging.getLogger(__name__)


# ══════════════════════════════════════════════════════════════════
#  房间类型→物体 语义先验知识库
# ══════════════════════════════════════════════════════════════════

ROOM_OBJECT_PRIORS: Dict[str, Dict[str, float]] = {
    "corridor": {
        "door": 0.95, "sign": 0.80, "fire extinguisher": 0.70,
        "elevator": 0.40, "stairs": 0.35, "trash can": 0.30,
        "light": 0.85, "handrail": 0.25, "emergency exit": 0.40,
        "hallway": 0.90, "camera": 0.30,
    },
    "office": {
        "desk": 0.95, "chair": 0.95, "computer": 0.85, "monitor": 0.85,
        "keyboard": 0.80, "mouse": 0.80, "phone": 0.60, "printer": 0.40,
        "bookshelf": 0.50, "cabinet": 0.60, "trash can": 0.70,
        "whiteboard": 0.30, "lamp": 0.50, "cup": 0.40, "pen": 0.30,
    },
    "kitchen": {
        "refrigerator": 0.90, "sink": 0.85, "microwave": 0.75,
        "oven": 0.60, "kettle": 0.70, "table": 0.80, "chair": 0.70,
        "cabinet": 0.80, "trash can": 0.85, "bottle": 0.60,
        "cup": 0.65, "dish": 0.50, "cutting board": 0.30,
    },
    "meeting_room": {
        "table": 0.95, "chair": 0.95, "projector": 0.70, "screen": 0.65,
        "whiteboard": 0.80, "monitor": 0.50, "phone": 0.40, "door": 0.90,
        "clock": 0.30, "speaker": 0.25, "camera": 0.20,
    },
    "bathroom": {
        "toilet": 0.90, "sink": 0.90, "mirror": 0.85, "door": 0.95,
        "trash can": 0.60, "soap dispenser": 0.50, "paper towel": 0.40,
        "hand dryer": 0.30, "light": 0.80,
    },
    "stairwell": {
        "stairs": 0.95, "railing": 0.90, "handrail": 0.90,
        "fire extinguisher": 0.75, "door": 0.85, "sign": 0.70,
        "emergency exit": 0.60, "light": 0.75,
    },
    "lobby": {
        "sofa": 0.70, "reception": 0.50, "desk": 0.50, "chair": 0.60,
        "door": 0.90, "sign": 0.60, "plant": 0.40, "elevator": 0.50,
        "tv": 0.30, "magazine": 0.20, "clock": 0.25,
    },
    "storage": {
        "shelf": 0.90, "cabinet": 0.85, "box": 0.80, "door": 0.90,
        "rack": 0.60, "tool": 0.30, "broom": 0.25, "bucket": 0.20,
    },
    "lab": {
        "desk": 0.85, "computer": 0.80, "monitor": 0.75,
        "equipment": 0.60, "cabinet": 0.50, "chair": 0.80,
        "whiteboard": 0.40, "printer": 0.30, "safety equipment": 0.25,
    },
    "classroom": {
        "desk": 0.95, "chair": 0.95, "blackboard": 0.80, "projector": 0.60,
        "screen": 0.50, "book": 0.40, "clock": 0.35, "speaker": 0.20,
    },
}

# 中文→英文房间类型映射 (用于跨语言匹配)
ZH_ROOM_MAP = {
    "走廊": "corridor", "过道": "corridor", "通道": "corridor",
    "办公室": "office", "工位": "office",
    "厨房": "kitchen", "茶水间": "kitchen",
    "会议室": "meeting_room",
    "卫生间": "bathroom", "洗手间": "bathroom", "厕所": "bathroom",
    "楼梯间": "stairwell", "楼梯": "stairwell",
    "大厅": "lobby", "门厅": "lobby", "前台": "lobby",
    "储物间": "storage", "仓库": "storage", "杂物间": "storage",
    "实验室": "lab",
    "教室": "classroom",
}

# 中文→英文物体类型映射 (常用)
ZH_OBJECT_MAP = {
    "灭火器": "fire extinguisher", "门": "door", "椅子": "chair",
    "桌子": "desk", "电脑": "computer", "显示器": "monitor",
    "冰箱": "refrigerator", "微波炉": "microwave", "水壶": "kettle",
    "马桶": "toilet", "镜子": "mirror", "洗手台": "sink",
    "楼梯": "stairs", "扶手": "railing", "沙发": "sofa",
    "柜子": "cabinet", "架子": "shelf", "垃圾桶": "trash can",
    "白板": "whiteboard", "投影仪": "projector", "电梯": "elevator",
    "标志": "sign", "指示牌": "sign", "植物": "plant",
    "打印机": "printer", "书架": "bookshelf", "电话": "phone",
    "时钟": "clock", "灯": "light", "杯子": "cup",
}


@dataclass
class RoomPrior:
    """单个房间的语义先验评分结果。"""
    room_id: int
    room_name: str
    room_type: str
    prior_score: float
    matched_objects: List[str] = field(default_factory=list)
    is_visited: bool = False
    reasoning: str = ""


@dataclass
class TopologyEdge:
    """房间间拓扑连通边。"""
    from_room_id: int
    to_room_id: int
    edge_type: str              # "door" | "traversal" | "proximity"
    mediator_label: str = ""    # 连通介质的标签 (如 "door")
    mediator_position: Optional[Dict[str, float]] = None
    distance: float = 0.0
    traversal_count: int = 0    # 机器人实际穿越次数

    def to_dict(self) -> Dict:
        d = {
            "from_room": self.from_room_id,
            "to_room": self.to_room_id,
            "type": self.edge_type,
            "distance": round(self.distance, 2),
        }
        if self.mediator_label:
            d["mediator"] = self.mediator_label
        if self.mediator_position:
            d["mediator_pos"] = self.mediator_position
        if self.traversal_count > 0:
            d["traversals"] = self.traversal_count
        return d


class SemanticPriorEngine:
    """
    语义先验引擎: 物体→房间联想 + 房间→物体预期。

    核心方法:
      - predict_target_rooms(): 给定目标描述, 预测最可能的房间类型
      - score_rooms_for_target(): 给定具体房间列表+目标, 评分
      - get_unexplored_priors(): 结合拓扑图, 推荐未探索但高先验的方向
    """

    def __init__(
        self,
        room_priors: Optional[Dict[str, Dict[str, float]]] = None,
    ):
        self._priors = room_priors or ROOM_OBJECT_PRIORS
        self._inverse_index = self._build_inverse_index()

    def _build_inverse_index(self) -> Dict[str, List[Tuple[str, float]]]:
        """物体→[(房间类型, 概率)] 反向索引。"""
        idx: Dict[str, List[Tuple[str, float]]] = {}
        for room_type, objects in self._priors.items():
            for obj_label, prob in objects.items():
                key = obj_label.lower()
                if key not in idx:
                    idx[key] = []
                idx[key].append((room_type, prob))
        for key in idx:
            idx[key].sort(key=lambda x: x[1], reverse=True)
        return idx

    def predict_target_rooms(
        self,
        target_instruction: str,
    ) -> List[Tuple[str, float, str]]:
        """
        预测目标最可能在哪种房间类型中。

        Returns:
            [(room_type, probability, reasoning)] 按概率降序
        """
        keywords = self._extract_target_keywords(target_instruction)
        if not keywords:
            return []

        room_scores: Dict[str, float] = {}
        room_reasons: Dict[str, List[str]] = {}

        for kw in keywords:
            candidates = self._inverse_index.get(kw, [])
            for room_type, prob in candidates:
                old = room_scores.get(room_type, 0.0)
                room_scores[room_type] = max(old, prob)
                if room_type not in room_reasons:
                    room_reasons[room_type] = []
                room_reasons[room_type].append(f"{kw}→{prob:.0%}")

        results = [
            (rt, score, "; ".join(room_reasons.get(rt, [])))
            for rt, score in room_scores.items()
        ]
        results.sort(key=lambda x: x[1], reverse=True)
        return results

    def score_rooms_for_target(
        self,
        target_instruction: str,
        rooms: List[Dict],
        visited_room_ids: Optional[Set[int]] = None,
    ) -> List[RoomPrior]:
        """
        对具体房间列表评分: "这个目标最可能在哪个已知房间里?"

        Args:
            target_instruction: 目标描述
            rooms: 场景图中的 rooms 列表
            visited_room_ids: 已探索过的房间 ID 集合

        Returns:
            RoomPrior 列表 (按 prior_score 降序)
        """
        visited = visited_room_ids or set()
        keywords = self._extract_target_keywords(target_instruction)
        if not keywords:
            return []

        results: List[RoomPrior] = []
        for room in rooms:
            room_id = room.get("room_id", -1)
            room_name = room.get("name", "unknown")
            room_type = self._normalize_room_type(room_name)

            priors = self._priors.get(room_type, {})
            matched = []
            max_prior = 0.0

            for kw in keywords:
                for obj_label, prob in priors.items():
                    if kw in obj_label.lower() or obj_label.lower() in kw:
                        matched.append(obj_label)
                        max_prior = max(max_prior, prob)

            existing_labels = [
                str(l).lower() for l in room.get("semantic_labels", [])
            ]
            direct_match = any(
                kw in el or el in kw
                for kw in keywords for el in existing_labels
            )
            if direct_match:
                max_prior = max(max_prior, 0.95)

            is_visited = room_id in visited
            exploration_bonus = 0.0 if is_visited else 0.15

            score = min(1.0, max_prior + exploration_bonus)

            reasoning_parts = []
            if room_type in self._priors:
                reasoning_parts.append(f"type={room_type}")
            if matched:
                reasoning_parts.append(f"prior_match={','.join(matched[:3])}")
            if direct_match:
                reasoning_parts.append("direct_label_match")
            if not is_visited:
                reasoning_parts.append("unvisited(+0.15)")

            results.append(RoomPrior(
                room_id=room_id,
                room_name=room_name,
                room_type=room_type,
                prior_score=score,
                matched_objects=matched,
                is_visited=is_visited,
                reasoning="; ".join(reasoning_parts),
            ))

        results.sort(key=lambda r: r.prior_score, reverse=True)
        return results

    def get_unexplored_priors(
        self,
        target_instruction: str,
        rooms: List[Dict],
        topology_edges: List[Dict],
        visited_room_ids: Set[int],
        current_room_id: int = -1,
    ) -> List[Dict]:
        """
        结合拓扑图推荐未探索方向: 语义先验 + 可达性。

        Returns:
            [{room_id, room_name, prior_score, reachable_from, hops, reasoning}]
        """
        room_priors = self.score_rooms_for_target(
            target_instruction, rooms, visited_room_ids,
        )

        adjacency: Dict[int, List[int]] = {}
        for edge in topology_edges:
            fr = edge.get("from_room", -1)
            to = edge.get("to_room", -1)
            if fr >= 0 and to >= 0:
                adjacency.setdefault(fr, []).append(to)
                adjacency.setdefault(to, []).append(fr)

        # BFS from current_room to compute hop distances
        hop_dist: Dict[int, int] = {}
        if current_room_id >= 0:
            queue = [current_room_id]
            hop_dist[current_room_id] = 0
            while queue:
                node = queue.pop(0)
                for nb in adjacency.get(node, []):
                    if nb not in hop_dist:
                        hop_dist[nb] = hop_dist[node] + 1
                        queue.append(nb)

        recommendations = []
        for rp in room_priors:
            if rp.is_visited:
                continue
            if rp.prior_score < 0.1:
                continue

            hops = hop_dist.get(rp.room_id, -1)

            reachability_factor = 1.0
            if hops > 0:
                reachability_factor = 1.0 / (1.0 + 0.2 * hops)
            elif hops < 0:
                reachability_factor = 0.3  # not reachable via topology

            combined = rp.prior_score * reachability_factor

            recommendations.append({
                "room_id": rp.room_id,
                "room_name": rp.room_name,
                "room_type": rp.room_type,
                "prior_score": round(rp.prior_score, 3),
                "combined_score": round(combined, 3),
                "hops": hops,
                "reasoning": rp.reasoning,
            })

        recommendations.sort(key=lambda r: r["combined_score"], reverse=True)
        return recommendations

    def get_room_expected_objects(self, room_name: str) -> Dict[str, float]:
        """获取某房间类型的预期物体清单。"""
        room_type = self._normalize_room_type(room_name)
        return dict(self._priors.get(room_type, {}))

    def get_exploration_summary(
        self,
        target_instruction: str,
        rooms: List[Dict],
        visited_room_ids: Set[int],
    ) -> str:
        """生成拓扑探索摘要 (给 LLM 消费)。"""
        predictions = self.predict_target_rooms(target_instruction)
        if not predictions:
            return "No semantic prior available for this target."

        parts = []
        parts.append(f"Target: {target_instruction}")
        parts.append("Predicted room types (by semantic prior):")
        for rt, prob, reason in predictions[:5]:
            parts.append(f"  - {rt}: {prob:.0%} ({reason})")

        visited_types = set()
        unvisited_types = set()
        for room in rooms:
            rt = self._normalize_room_type(room.get("name", ""))
            if room.get("room_id", -1) in visited_room_ids:
                visited_types.add(rt)
            else:
                unvisited_types.add(rt)

        if visited_types:
            parts.append(f"Visited room types: {', '.join(visited_types)}")
        if unvisited_types:
            parts.append(f"Unvisited room types: {', '.join(unvisited_types)}")

        return "\n".join(parts)

    # ── 内部方法 ──

    def _extract_target_keywords(self, instruction: str) -> List[str]:
        """提取目标关键词 (支持中英文)。"""
        text = instruction.lower().strip()
        keywords: List[str] = []

        # 中文→英文映射
        for zh, en in ZH_OBJECT_MAP.items():
            if zh in text:
                keywords.append(en)

        # 英文关键词
        stop = {
            "the", "a", "an", "to", "go", "find", "near", "with", "at",
            "of", "please", "where", "is", "i", "want", "need", "look",
            "去", "找", "到", "在", "附近", "的", "一个", "一下", "有",
            "什么", "哪里", "帮", "我", "看看", "旁边", "那个",
        }
        tokens = re.findall(r"[a-z_]+(?:\s+[a-z_]+)?", text)
        for tok in tokens:
            if tok not in stop and len(tok) > 1:
                if tok not in keywords:
                    keywords.append(tok)

        return keywords

    @staticmethod
    def _normalize_room_type(room_name: str) -> str:
        """将房间名称归一化到标准类型。"""
        name = room_name.lower().strip()

        # 中文映射
        for zh, en in ZH_ROOM_MAP.items():
            if zh in name:
                return en

        # 英文直接匹配
        for room_type in ROOM_OBJECT_PRIORS:
            if room_type in name:
                return room_type

        # 模糊匹配
        if any(w in name for w in ["hall", "passage", "way"]):
            return "corridor"
        if any(w in name for w in ["work", "desk"]):
            return "office"
        if any(w in name for w in ["cook", "food"]):
            return "kitchen"
        if any(w in name for w in ["meet", "conference"]):
            return "meeting_room"
        if any(w in name for w in ["rest", "wash", "wc"]):
            return "bathroom"
        if any(w in name for w in ["stair", "step"]):
            return "stairwell"
        if any(w in name for w in ["store", "ware"]):
            return "storage"

        return "unknown"
