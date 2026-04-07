"""
工业知识图谱 — 物体属性、安全约束、功能可供性 (Affordance)。

参考论文:
  - ConceptBot (ICLR 2026 submission): KG + LLM 任务分解, 超 SayCan 56%
  - AdaptBot (ICRA 2025): LLM + KG + Human-in-Loop 协同
  - SafeMind (2025): 事实/因果/时间三类安全约束
  - OpenFunGraph (CVPR 2025 Highlight): 功能性 3D 场景图
  - DovSG (IEEE RA-L 2025): 动态开放词汇 3D 场景图
  - LOVON (2025-07): 四足开放词汇导航, Unitree 验证

设计:
  - 双层知识: 静态通用知识 (内置) + 动态场景知识 (运行时学习)
  - 安全优先: 任何操作前先查 KG 安全约束
  - 开放词汇: 未知物体通过 CLIP 相似度映射到最近已知概念
  - 场景联想: 房间类型 → 预期物体, 用于引导探索

静态数据定义见 kg_data.py (概念/关系/安全约束)。
"""

import json
import logging
import math
from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, FrozenSet, List, Optional, Set, Tuple

import numpy as np

logger = logging.getLogger(__name__)


# ════════════════════════════════════════════════════════════════
#  数据类型定义
# ════════════════════════════════════════════════════════════════

class SafetyLevel(Enum):
    """安全等级 (SafeMind 三类约束 + 分级)。"""
    SAFE = "safe"                   # 可自由交互
    CAUTION = "caution"             # 需确认后交互
    DANGEROUS = "dangerous"         # 需授权/专业人员
    FORBIDDEN = "forbidden"         # 机器人禁止触碰


class AffordanceType(Enum):
    """物体可供性类型 (OpenFunGraph CVPR 2025)。"""
    GRASPABLE = "graspable"
    OPENABLE = "openable"
    CLOSABLE = "closable"
    PUSHABLE = "pushable"
    SITTABLE = "sittable"
    SUPPORTIVE = "supportive"       # 可放置物品
    CONTAINABLE = "containable"     # 可容纳物品
    SWITCHABLE = "switchable"       # 可开关
    READABLE = "readable"           # 可读取信息
    INSPECTABLE = "inspectable"     # 可巡检
    CLIMBABLE = "climbable"
    PASSABLE = "passable"           # 可通过 (门/走廊)


class RelationType(Enum):
    """知识关系类型 (ConceptBot / ConceptNet)。"""
    IS_A = "is_a"                   # 椅子 IS_A 家具
    HAS_PROPERTY = "has_property"   # 灭火器 HAS_PROPERTY 重量3-8kg
    USED_FOR = "used_for"           # 灭火器 USED_FOR 灭火
    LOCATED_IN = "located_in"       # 打印机 LOCATED_IN 办公室
    CAPABLE_OF = "capable_of"       # 门 CAPABLE_OF 打开/关闭
    DANGEROUS_IF = "dangerous_if"   # 配电箱 DANGEROUS_IF 触碰
    REQUIRES = "requires"           # 灭火器 REQUIRES 拔保险销
    PART_OF = "part_of"             # 按键 PART_OF 电梯
    RELATED_TO = "related_to"       # 灭火器 RELATED_TO 消防通道


@dataclass
class ObjectConcept:
    """物体概念节点 (KG 中的核心实体)。"""
    concept_id: str                  # 唯一标识, e.g. "fire_extinguisher"
    names_zh: list[str]              # 中文名称列表 (含别名)
    names_en: list[str]              # 英文名称列表 (含别名)
    category: str                    # 上位类别: furniture, safety, electronics, etc.
    safety_level: SafetyLevel = SafetyLevel.SAFE
    affordances: set[AffordanceType] = field(default_factory=set)
    typical_locations: list[str] = field(default_factory=list)
    weight_kg: tuple[float, float] = (0.0, 0.0)  # (min, max) 估算
    size_class: str = "medium"       # small / medium / large / fixed
    properties: dict[str, str] = field(default_factory=dict)
    safety_notes: list[str] = field(default_factory=list)
    clip_aliases: list[str] = field(default_factory=list)  # CLIP 匹配用的额外描述


@dataclass
class KGRelation:
    """知识图谱关系边。"""
    source: str                      # concept_id
    relation: RelationType
    target: str                      # concept_id 或属性字符串
    confidence: float = 1.0
    note: str = ""


@dataclass
class SafetyConstraint:
    """安全约束规则 (SafeMind 事实/因果/时间三类)。"""
    constraint_id: str
    constraint_type: str             # "factual" | "causal" | "temporal"
    concept_id: str                  # 涉及的物体概念
    action: str                      # 触发动作: "pick", "push", "approach", etc.
    condition: str                   # 触发条件描述
    response: str                    # "block" | "warn" | "confirm" | "limit_speed"
    message_zh: str                  # 中文警告信息
    message_en: str                  # 英文警告信息
    max_approach_distance: float = 0.0  # 最小安全距离 (m), 0 = 无限制


# ════════════════════════════════════════════════════════════════
#  工业知识图谱
# ════════════════════════════════════════════════════════════════

class IndustrialKnowledgeGraph:
    """
    工业环境知识图谱 (ConceptBot + SafeMind 融合)。

    静态知识: 预定义的物体概念、关系、安全约束 (来自 kg_data.py)
    动态知识: 运行时通过观察学习的新概念和场景关联
    开放词汇: 未知物体通过名称/CLIP 映射到最近已知概念
    """

    def __init__(self):
        self._concepts: dict[str, ObjectConcept] = {}
        self._relations: list[KGRelation] = []
        self._safety_constraints: list[SafetyConstraint] = []
        self._name_index_zh: dict[str, str] = {}   # 中文名 → concept_id
        self._name_index_en: dict[str, str] = {}   # 英文名 → concept_id
        self._category_index: dict[str, list[str]] = {}  # category → [concept_id]

        self._build_industrial_knowledge()

    # ── 查询接口 ──

    def lookup(self, name: str) -> ObjectConcept | None:
        """按名称查找物体概念 (中英文、别名、模糊匹配)。"""
        name_lower = name.lower().strip()
        if not name_lower:
            return None

        # 1. 精确匹配
        cid = self._name_index_en.get(name_lower) or self._name_index_zh.get(name_lower)
        if cid:
            return self._concepts.get(cid)

        # 2. 子串匹配 (中文尤为重要: "灭火器" in "干粉灭火器")
        for zh_name, cid in self._name_index_zh.items():
            if name_lower in zh_name or zh_name in name_lower:
                return self._concepts.get(cid)
        for en_name, cid in self._name_index_en.items():
            if name_lower in en_name or en_name in name_lower:
                return self._concepts.get(cid)

        # 3. concept_id 直接匹配
        if name_lower in self._concepts:
            return self._concepts[name_lower]

        return None

    def get_safety_level(self, name: str) -> SafetyLevel:
        """查询物体安全等级。"""
        concept = self.lookup(name)
        return concept.safety_level if concept else SafetyLevel.SAFE

    def get_affordances(self, name: str) -> set[AffordanceType]:
        """查询物体可供性。"""
        concept = self.lookup(name)
        return concept.affordances if concept else set()

    def check_safety(self, target: str, action: str) -> SafetyConstraint | None:
        """
        检查操作安全性 (SafeMind 约束检查)。

        Args:
            target: 目标物体名称
            action: 意图动作 ("pick", "place", "approach", "push", "inspect" 等)

        Returns:
            触发的安全约束, 或 None 表示安全
        """
        concept = self.lookup(target)
        if concept is None:
            return None

        for constraint in self._safety_constraints:
            if constraint.concept_id != concept.concept_id:
                continue
            if constraint.action == action or constraint.action == "*":
                return constraint

        return None

    def get_relations(self, concept_id: str) -> list[KGRelation]:
        """获取概念的所有关系。"""
        return [r for r in self._relations if r.source == concept_id]

    def get_typical_locations(self, name: str) -> list[str]:
        """查询物体典型位置 (辅助 FIND 指令定向搜索)。"""
        concept = self.lookup(name)
        return concept.typical_locations if concept else []

    def get_category_members(self, category: str) -> list[ObjectConcept]:
        """按类别查询所有物体。"""
        cids = self._category_index.get(category, [])
        return [self._concepts[cid] for cid in cids if cid in self._concepts]

    def is_graspable(self, name: str) -> bool:
        """判断物体是否可抓取。"""
        affordances = self.get_affordances(name)
        return AffordanceType.GRASPABLE in affordances

    def is_dangerous(self, name: str) -> bool:
        """判断物体是否危险。"""
        level = self.get_safety_level(name)
        return level in (SafetyLevel.DANGEROUS, SafetyLevel.FORBIDDEN)

    def enrich_object_properties(self, label: str) -> dict[str, any]:
        """
        为检测到的物体补充 KG 知识 (ConceptBot OPE 模块)。

        用于 InstanceTracker — 每个 TrackedObject 创建/更新时,
        查 KG 补充结构化属性。
        """
        concept = self.lookup(label)
        if concept is None:
            return {
                "kg_matched": False,
                "safety_level": SafetyLevel.SAFE.value,
                "affordances": [],
                "category": "unknown",
            }

        return {
            "kg_matched": True,
            "concept_id": concept.concept_id,
            "safety_level": concept.safety_level.value,
            "affordances": [a.value for a in concept.affordances],
            "category": concept.category,
            "typical_locations": concept.typical_locations,
            "weight_range": list(concept.weight_kg),
            "size_class": concept.size_class,
            "safety_notes": concept.safety_notes,
            "properties": concept.properties,
        }

    def query_by_affordance(self, affordance: AffordanceType) -> list[ObjectConcept]:
        """按可供性查询 (OpenFunGraph 功能查询)。"""
        return [
            c for c in self._concepts.values()
            if affordance in c.affordances
        ]

    def get_all_concepts(self) -> list[ObjectConcept]:
        """返回所有概念列表。"""
        return list(self._concepts.values())

    def get_clip_vocabulary(self) -> list[str]:
        """导出 CLIP 匹配词汇表 (供 YOLO-World 动态类名)。"""
        vocab = []
        for concept in self._concepts.values():
            vocab.extend(concept.names_en)
            vocab.extend(concept.clip_aliases)
        return list(set(vocab))

    def to_json(self) -> str:
        """导出知识图谱为 JSON (供 LLM prompt / 调试)。"""
        data = {
            "concepts": {},
            "relations": [],
            "safety_constraints": [],
        }
        for cid, c in self._concepts.items():
            data["concepts"][cid] = {
                "names_zh": c.names_zh,
                "names_en": c.names_en,
                "category": c.category,
                "safety_level": c.safety_level.value,
                "affordances": [a.value for a in c.affordances],
                "typical_locations": c.typical_locations,
            }
        for r in self._relations:
            data["relations"].append({
                "source": r.source,
                "relation": r.relation.value,
                "target": r.target,
            })
        for sc in self._safety_constraints:
            data["safety_constraints"].append({
                "id": sc.constraint_id,
                "type": sc.constraint_type,
                "concept": sc.concept_id,
                "action": sc.action,
                "response": sc.response,
                "message_zh": sc.message_zh,
            })
        return json.dumps(data, ensure_ascii=False, indent=2)

    # ── 动态知识扩展 ──

    def register_concept(self, concept: ObjectConcept) -> None:
        """运行时注册新概念 (场景中发现的未知物体)。"""
        self._concepts[concept.concept_id] = concept
        for name in concept.names_zh:
            self._name_index_zh[name.lower()] = concept.concept_id
        for name in concept.names_en:
            self._name_index_en[name.lower()] = concept.concept_id
        self._category_index.setdefault(concept.category, []).append(concept.concept_id)
        logger.info("KG: registered new concept '%s' (%s)", concept.concept_id, concept.category)

    def register_constraint(self, constraint: SafetyConstraint) -> None:
        """运行时注册新安全约束。"""
        self._safety_constraints.append(constraint)
        logger.info("KG: registered safety constraint '%s'", constraint.constraint_id)

    # ════════════════════════════════════════════════════════
    #  内置工业知识库
    # ════════════════════════════════════════════════════════

    def _build_industrial_knowledge(self) -> None:
        """从 kg_data 加载静态知识，并构建索引。"""
        from .knowledge_data import build_all_knowledge
        build_all_knowledge(self._concepts, self._relations, self._safety_constraints)
        self._build_indices()

        logger.info(
            "KG built: %d concepts, %d relations, %d safety constraints, %d categories",
            len(self._concepts), len(self._relations),
            len(self._safety_constraints), len(self._category_index),
        )

    def _build_indices(self) -> None:
        """构建名称和类别索引。"""
        for cid, concept in self._concepts.items():
            for name in concept.names_zh:
                self._name_index_zh[name.lower()] = cid
            for name in concept.names_en:
                self._name_index_en[name.lower()] = cid
            self._category_index.setdefault(concept.category, []).append(cid)

    # ════════════════════════════════════════════════════════
    #  开放词汇映射 (DovSG / LOVON 风格)
    # ════════════════════════════════════════════════════════

    def map_unknown_to_concept(
        self,
        label: str,
        clip_embedding: np.ndarray | None = None,
        clip_encoder=None,
        similarity_threshold: float = 0.25,
    ) -> ObjectConcept | None:
        """
        将未知物体映射到最近的已知概念 (DovSG 开放词汇)。

        三级匹配策略:
          1. 名称精确/子串匹配 (最快)
          2. CLIP 语义相似度 (向量级)
          3. 类别关键词模糊匹配 (兜底)
        """
        # Level 1: 名称匹配
        direct = self.lookup(label)
        if direct is not None:
            return direct

        # Level 2: CLIP 语义相似度 (批量编码, 避免逐别名调用)
        if clip_embedding is not None and clip_encoder is not None:
            all_aliases: list[str] = []
            alias_to_concept: list[ObjectConcept] = []
            for concept in self._concepts.values():
                for alias in concept.names_en + concept.clip_aliases:
                    all_aliases.append(alias)
                    alias_to_concept.append(concept)

            if all_aliases:
                try:
                    text_feats = clip_encoder.encode_text(all_aliases)
                    if text_feats.size > 0:
                        q = clip_embedding / (np.linalg.norm(clip_embedding) + 1e-8)
                        norms = np.linalg.norm(text_feats, axis=1, keepdims=True) + 1e-8
                        text_feats_norm = text_feats / norms
                        sims = text_feats_norm @ q
                        best_idx = int(np.argmax(sims))
                        best_sim = float(sims[best_idx])
                        if best_sim >= similarity_threshold:
                            best_concept = alias_to_concept[best_idx]
                            logger.info(
                                "OV mapping: '%s' → '%s' (sim=%.3f, alias='%s')",
                                label, best_concept.concept_id, best_sim,
                                all_aliases[best_idx],
                            )
                            return best_concept
                except Exception as e:
                    logger.warning("CLIP batch mapping failed: %s", e)

        # Level 3: 类别关键词模糊匹配
        label_lower = label.lower()
        category_hints = {
            "safety": ["fire", "alarm", "exit", "emergency", "extinguish", "aid",
                        "灭火", "报警", "出口", "急救", "安全"],
            "furniture": ["chair", "desk", "table", "shelf", "sofa", "bed", "bench",
                          "椅", "桌", "柜", "沙发", "架"],
            "electronics": ["computer", "monitor", "screen", "printer", "phone", "camera",
                            "电脑", "显示", "屏幕", "打印", "摄像"],
            "structure": ["door", "window", "wall", "stairs", "elevator", "pillar",
                          "门", "窗", "墙", "楼梯", "电梯", "柱"],
            "industrial": ["conveyor", "forklift", "crane", "valve", "pipe", "tank",
                           "传送", "叉车", "起重", "阀", "管", "罐"],
            "vehicle": ["car", "truck", "cart", "wagon", "车", "推车"],
            "medical": ["stretcher", "wheelchair", "medicine", "担架", "轮椅", "药"],
            "outdoor": ["tree", "bench", "hydrant", "pole", "fence",
                         "树", "长椅", "路灯", "围栏"],
        }
        for cat, hints in category_hints.items():
            if any(h in label_lower for h in hints):
                members = self.get_category_members(cat)
                if members:
                    return members[0]

        return None

    def get_room_expected_objects(self, room_type: str) -> list[str]:
        """
        查询房间类型的预期物体 (引导探索, Concept-Guided Exploration 风格)。

        用于 EXPLORE 指令时, 预测某个房间类型应该有哪些物体,
        帮助引导探索方向和验证房间分类。
        """
        room_objects = {
            "office": ["desk", "chair", "computer", "monitor", "printer", "cabinet",
                        "whiteboard", "lamp", "trash_bin", "plant", "backpack",
                        "cup", "bottle", "water_dispenser"],
            "kitchen": ["refrigerator", "cup", "bottle", "trash_bin", "lamp",
                         "microwave", "sink", "water_dispenser"],
            "break_room": ["refrigerator", "microwave", "water_dispenser",
                            "cup", "bottle", "vending_machine", "chair", "desk",
                            "sofa", "trash_bin"],
            "corridor": ["door", "fire_extinguisher", "fire_alarm", "safety_sign",
                          "emergency_exit", "fire_hydrant", "trash_bin",
                          "fire_door", "vending_machine", "water_dispenser"],
            "meeting_room": ["desk", "chair", "projector", "whiteboard",
                              "monitor", "television"],
            "bathroom": ["door", "trash_bin", "mirror", "sink", "toilet"],
            "bedroom": ["bed", "lamp", "mirror", "cabinet", "desk", "chair"],
            "living_room": ["sofa", "television", "lamp", "plant", "shelf"],
            "lobby": ["sofa", "plant", "door", "elevator", "aed",
                       "robot_charging_station", "safety_sign",
                       "umbrella_stand", "vending_machine"],
            "stairwell": ["stairs", "railing", "fire_extinguisher", "emergency_exit",
                           "safety_sign", "door", "fire_door"],
            "storage": ["shelf", "cabinet", "box", "pallet"],
            "server_room": ["server_rack", "electrical_panel", "fire_extinguisher",
                             "fire_alarm", "hvac_unit"],
            "warehouse": ["pallet", "forklift", "shelf", "conveyor", "box",
                           "safety_sign", "fire_extinguisher", "safety_helmet",
                           "safety_vest", "spill_kit"],
            "lab": ["desk", "cabinet", "gas_cylinder", "fire_blanket",
                     "first_aid_kit", "fire_extinguisher", "eye_wash_station",
                     "safety_sign"],
            "parking": ["pillar", "fire_extinguisher", "emergency_exit",
                         "safety_sign", "traffic_cone"],
            "outdoor": ["tree", "bench_outdoor", "street_light", "fire_hydrant_outdoor",
                          "traffic_cone", "fence", "manhole_cover"],
            "elevator_hall": ["elevator", "door", "plant", "safety_sign"],
            "factory": ["conveyor", "crane", "control_panel", "safety_sign",
                          "safety_helmet", "safety_vest", "forklift", "pallet",
                          "fire_extinguisher", "eye_wash_station"],
            "hospital": ["wheelchair", "stretcher", "medicine_cabinet",
                          "oxygen_supply", "first_aid_kit", "aed"],
            "entrance": ["door", "umbrella_stand", "safety_sign", "fire_door"],
            "utility_room": ["electrical_panel", "pipe", "valve", "generator",
                              "hvac_unit"],
            "laundry": ["washing_machine", "sink"],
        }
        room_lower = room_type.lower().replace(" ", "_")
        return room_objects.get(room_lower, [])

    def get_manipulation_info(self, target: str, action: str) -> dict:
        """
        获取操作信息 (PICK/PLACE 前的可行性判断)。

        ConceptBot 的 URP 模块: 判断机器人能否完成操作,
        如果不能, 返回原因。
        """
        concept = self.lookup(target)
        if concept is None:
            return {
                "feasible": True,
                "confidence": 0.3,
                "reason": "unknown_object",
                "notes": [],
            }

        result = {
            "feasible": True,
            "confidence": 0.8,
            "reason": "ok",
            "notes": [],
            "safety": concept.safety_level.value,
            "weight_range": list(concept.weight_kg),
            "size_class": concept.size_class,
        }

        # 安全检查
        if concept.safety_level == SafetyLevel.FORBIDDEN:
            result["feasible"] = False
            result["reason"] = "forbidden_object"
            result["notes"] = concept.safety_notes
            return result

        if concept.safety_level == SafetyLevel.DANGEROUS:
            result["feasible"] = False
            result["reason"] = "dangerous_object"
            result["notes"] = concept.safety_notes
            return result

        if action == "pick":
            if AffordanceType.GRASPABLE not in concept.affordances:
                result["feasible"] = False
                result["reason"] = "not_graspable"
                result["notes"].append(f"{target} is {concept.size_class}, not graspable")
                return result

            max_weight = concept.weight_kg[1]
            ROBOT_MAX_PAYLOAD_KG = 5.0
            if max_weight > ROBOT_MAX_PAYLOAD_KG:
                result["feasible"] = False
                result["reason"] = "too_heavy"
                result["notes"].append(
                    f"weight up to {max_weight}kg exceeds robot payload {ROBOT_MAX_PAYLOAD_KG}kg"
                )
                return result

            if concept.size_class in ("large", "fixed"):
                result["feasible"] = False
                result["reason"] = "too_large"
                return result

        elif action == "place":
            pass

        constraint = self.check_safety(target, action)
        if constraint:
            if constraint.response == "block":
                result["feasible"] = False
                result["reason"] = "safety_blocked"
                result["notes"].append(constraint.message_en)
            elif constraint.response in ("warn", "confirm"):
                result["confidence"] = 0.5
                result["notes"].append(constraint.message_en)

        return result

    def get_stats(self) -> dict:
        """返回 KG 统计信息。"""
        return {
            "total_concepts": len(self._concepts),
            "total_relations": len(self._relations),
            "total_safety_constraints": len(self._safety_constraints),
            "categories": {k: len(v) for k, v in self._category_index.items()},
            "dangerous_count": sum(
                1 for c in self._concepts.values()
                if c.safety_level in (SafetyLevel.DANGEROUS, SafetyLevel.FORBIDDEN)
            ),
        }
