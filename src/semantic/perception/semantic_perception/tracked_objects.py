"""
tracked_objects.py — 数据类、常量、辅助函数 (从 instance_tracker.py 提取)

包含:
  - 所有空间/信念/安全常量
  - SpatialRelation, Region, GroupNode, RoomNode, FloorNode
  - PhantomNode, RoomTypePosterior, BeliefMessage, ViewNode
  - TrackedObject (含所有 update/belief 方法)
  - infer_room_type(), GROUP_KEYWORDS
"""

import math
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np

# ── 空间关系常量 (参考 SG-Nav 的关系类型) ──

RELATION_NEAR_THRESHOLD = 1.5      # 米, 小于此距离视为 "near"
RELATION_ON_THRESHOLD = 0.3        # 米, z 轴差异小于此视为 "on" (物体在另一个上面)
REGION_CLUSTER_RADIUS = 3.0        # 米, 同一区域的聚类半径
FLOOR_HEIGHT = 3.0                 # 米, 楼层高度估算 (用于 z 坐标→楼层映射)
FLOOR_MERGE_TOLERANCE = 0.8        # 米, 楼层 z 坐标合并容差

# ── BA-HSG 信念参数 (Belief-Aware Hierarchical Scene Graph) ──
BELIEF_SIGMA_BASE = 0.05           # 位置基准噪声 (m)
BELIEF_SIGMA_DEPTH_COEFF = 0.02    # 深度比例噪声系数
BELIEF_NEG_EVIDENCE_WEIGHT = 0.5   # 负面证据权重 (未检测到时)
BELIEF_FRESHNESS_TAU = 30.0        # 时间衰减常数 (s)
BELIEF_REPROJ_KAPPA = 5.0          # 重投影误差衰减常数 (px)
BELIEF_ROOM_BOOST = 0.3            # 房间信念对新物体的提升幅度
BELIEF_LATERAL_SHARE = 0.1         # 空间邻居信念分享系数

# ── KG-Augmented Loopy Belief Propagation (论文级升级) ──
# 参考: Belief Scene Graphs (ICRA 2024), Commonsense BSG (2025)
# 创新: 用 KG 结构化先验替代 GCN 训练, 多轮迭代替代单次推理
BP_MAX_ITERATIONS = 3              # Loopy BP 最大迭代轮数
BP_CONVERGENCE_EPS = 0.005         # 收敛判据: max |Δ P_exist| < ε
BP_KG_PRIOR_BOOST = 1.5           # KG 先验: 期望物体在该房间时 α 提升量
BP_KG_UNEXPECTED_PENALTY = 0.3    # KG 约束: 非期望物体 β 增加量 (温和怀疑)
BP_ROOM_TO_OBJ_WEIGHT = 0.6       # 下行传播强度: 房间后验→物体先验
BP_OBJ_TO_ROOM_WEIGHT = 0.8       # 上行传播强度: 物体证据→房间类型后验
BP_LATERAL_DECAY = 0.7            # 横向传播衰减: 距离越远信息越弱
BP_PHANTOM_BASE_ALPHA = 0.8       # Phantom (blind) 节点初始 α
BP_PHANTOM_MIN_ROOM_CONFIDENCE = 0.4  # 房间类型置信度 > 此值才生成 phantom

# ── Safety-Aware Differential Credibility (安全感知差异化阈值) ──
# 创新: 根据物体危险等级动态调整所需可信度阈值
# 双阈值策略: 导航避障用低阈值 (宁可信其有), 交互操作用高阈值 (严格确认)
SAFETY_THRESHOLDS_NAVIGATION = {
    "safe": 0.25,
    "caution": 0.15,       # 更低! 检测到可能危险物 → 尽早避障
    "dangerous": 0.10,     # 极低: 一点迹象就触发避障
    "forbidden": 0.05,     # 几乎零容忍: 微弱信号即报警
}
SAFETY_THRESHOLDS_INTERACTION = {
    "safe": 0.40,
    "caution": 0.60,       # 需要更多确认
    "dangerous": 0.80,     # 严格确认才允许靠近
    "forbidden": 0.95,     # 几乎不可能允许交互
}
# Dangerous 物体 Alpha 初始保守系数 (贝叶斯先验偏保守)
SAFETY_PRIOR_ALPHA_SCALE = {
    "safe": 1.0,
    "caution": 1.2,        # 略微提升 → 更快确认 (宁可误报)
    "dangerous": 1.5,      # 明显提升 → 危险物体更快被 "相信存在"
    "forbidden": 2.0,      # 最高 → 极少量证据即确认 (保护性偏见)
}

# ── 规则 Room 命名映射 (创新1: 增强启发式, 替代 area_with_ 拼接) ──
ROOM_TYPE_RULES = {
    "corridor": {
        "keywords": ["door", "sign", "corridor", "hallway", "exit", "门", "走廊", "出口"],
        "min_match": 1,
        "priority": 2,
    },
    "office": {
        "keywords": ["desk", "chair", "computer", "monitor", "keyboard", "mouse", "办公", "桌", "电脑"],
        "min_match": 2,
        "priority": 3,
    },
    "kitchen": {
        "keywords": ["refrigerator", "sink", "microwave", "oven", "kettle", "冰箱", "厨房", "水壶"],
        "min_match": 1,
        "priority": 3,
    },
    "meeting_room": {
        "keywords": ["table", "chair", "screen", "projector", "whiteboard", "会议", "投影"],
        "min_match": 2,
        "priority": 3,
    },
    "bathroom": {
        "keywords": ["toilet", "sink", "mirror", "卫生间", "洗手", "镜"],
        "min_match": 1,
        "priority": 3,
    },
    "stairwell": {
        "keywords": ["stairs", "staircase", "railing", "楼梯", "扶手"],
        "min_match": 1,
        "priority": 3,
    },
    "lobby": {
        "keywords": ["sofa", "reception", "lobby", "大厅", "沙发", "前台"],
        "min_match": 1,
        "priority": 2,
    },
    "storage": {
        "keywords": ["shelf", "cabinet", "box", "storage", "储物", "柜", "架"],
        "min_match": 2,
        "priority": 1,
    },
}


def infer_room_type(labels: List[str]) -> str:
    """基于物体标签推断 Room 类型 (增强规则命名)。

    比 'area_with_door_chair' 更可读, 且不需要 LLM 调用。
    当 LLM 命名不可用时作为 fallback。
    """
    labels_lower = [l.lower() for l in labels]
    best_type = ""
    best_priority = -1
    best_matches = 0

    for room_type, rule in ROOM_TYPE_RULES.items():
        matches = sum(
            1 for kw in rule["keywords"]
            if any(kw in l for l in labels_lower)
        )
        if matches >= rule["min_match"]:
            score = matches * 10 + rule["priority"]
            if score > best_priority:
                best_priority = score
                best_type = room_type
                best_matches = matches

    if best_type:
        return best_type

    # Fallback: 用最常见物体标签
    if labels:
        from collections import Counter
        common = Counter(labels).most_common(2)
        return f"area_{'_'.join(l for l, _ in common)}"
    return "unknown_area"


GROUP_KEYWORDS = {
    "safety": ["fire", "extinguisher", "alarm", "灭火", "应急"],
    "furniture": ["chair", "desk", "table", "sofa", "cabinet", "shelf", "椅", "桌", "柜"],
    "structure": ["door", "window", "stairs", "elevator", "hall", "corridor", "门", "窗", "楼梯"],
    "electronics": ["monitor", "screen", "computer", "tv", "phone", "显示", "电脑", "电视"],
    "utility": ["trash", "bin", "bottle", "refrigerator", "sink", "lamp", "垃圾", "冰箱", "灯"],
}


@dataclass
class SpatialRelation:
    """物体间空间关系 (SG-Nav 风格)。"""
    subject_id: int
    relation: str                  # "near" | "left_of" | "right_of" | "in_front_of" | "behind" | "on" | "above" | "below"
    object_id: int
    distance: float = 0.0


ROOM_NAMING_STABILITY_COUNT = 3       # Region 内物体数 >= 此值时可触发 LLM 命名
ROOM_NAMING_STABILITY_SEC = 10.0      # Region 内物体集合稳定持续 N 秒后触发 LLM 命名


@dataclass
class Region:
    """空间区域 (SG-Nav 层次场景图的房间级)。"""
    region_id: int
    center: np.ndarray             # [x, y] 区域中心
    object_ids: List[int] = field(default_factory=list)
    name: str = ""                 # 自动命名或 LLM 命名
    llm_named: bool = False        # 是否已经被 LLM 命名


@dataclass
class GroupNode:
    """层次场景图的 group 层节点。"""

    group_id: int
    room_id: int
    name: str
    center: np.ndarray
    object_ids: List[int] = field(default_factory=list)
    semantic_labels: List[str] = field(default_factory=list)


@dataclass
class RoomNode:
    """层次场景图的 room 层节点。"""

    room_id: int
    name: str
    center: np.ndarray
    object_ids: List[int] = field(default_factory=list)
    group_ids: List[int] = field(default_factory=list)
    semantic_labels: List[str] = field(default_factory=list)
    llm_named: bool = False        # 是否由 LLM 命名
    clip_feature: Optional[np.ndarray] = None   # OneMap: room 级 CLIP 聚合特征
    feature_count: int = 0                       # 已融合的物体特征数


@dataclass
class FloorNode:
    """层次场景图的 floor 层节点 (SPADE IROS 2025 + HOV-SG 层次)。

    场景图层次: Floor → Room → Group → Object
    楼层通过物体 z 坐标聚类推断, 每 FLOOR_HEIGHT 为一层。
    """

    floor_id: int
    floor_level: int               # 楼层号: 0=地面层, 1=二楼, -1=地下
    z_range: Tuple[float, float]   # (z_min, z_max) 该楼层物体 z 坐标范围
    room_ids: List[int] = field(default_factory=list)
    object_ids: List[int] = field(default_factory=list)
    center_z: float = 0.0         # 楼层 z 中心


@dataclass
class PhantomNode:
    """盲节点 (Blind Node): 由 KG 先验 + 房间类型后验推断的未见但期望存在的物体。

    参考 Belief Scene Graphs (ICRA 2024) 的 blind node 概念,
    但用 KG 结构化先验替代 GCN 训练 — 无需训练数据, 可解释, 可扩展。

    生成条件:
      1. 房间类型后验置信度 > BP_PHANTOM_MIN_ROOM_CONFIDENCE
      2. KG.get_room_expected_objects(room_type) 包含此物体
      3. 该物体在该房间内尚未被检测到
    """
    phantom_id: int
    label: str                         # 期望物体类别
    room_id: int                       # 所属房间
    room_type: str                     # 推断的房间类型
    position: np.ndarray               # 预测位置 (房间中心 + KG 偏移)
    belief_alpha: float                # 存在性先验 α
    belief_beta: float = 1.0           # 存在性先验 β
    kg_prior_strength: float = 0.0     # KG 先验强度 P(object | room_type)
    safety_level: str = "safe"
    source: str = "kg_room_prior"      # 来源标记

    @property
    def existence_prob(self) -> float:
        return self.belief_alpha / (self.belief_alpha + self.belief_beta)


@dataclass
class RoomTypePosterior:
    """房间类型贝叶斯后验 — 多假设追踪。

    不同于 BSG 直接用标签, 也不同于 HOV-SG 用 CLIP 投票,
    我们维护 P(room_type | observed_objects) 的完整后验分布。

    更新公式 (对数域避免下溢):
      log P(t | O) ∝ log P(t) + Σ_i log P(o_i | t)
    其中 P(o_i | t) 来自 KG.get_room_expected_objects(t) 的匹配情况。
    """
    room_id: int
    hypotheses: Dict[str, float] = field(default_factory=dict)
    _log_posteriors: Dict[str, float] = field(default_factory=dict)

    @property
    def best_type(self) -> str:
        if not self.hypotheses:
            return "unknown"
        return max(self.hypotheses, key=self.hypotheses.get)

    @property
    def best_confidence(self) -> float:
        if not self.hypotheses:
            return 0.0
        return max(self.hypotheses.values())

    @property
    def entropy(self) -> float:
        """信息熵 — 衡量房间类型不确定性, 高熵 = 需要更多探索。"""
        if not self.hypotheses:
            return 0.0
        vals = list(self.hypotheses.values())
        total = sum(vals)
        if total <= 0:
            return 0.0
        h = 0.0
        for v in vals:
            p = v / total
            if p > 1e-10:
                h -= p * math.log2(p)
        return h


@dataclass
class BeliefMessage:
    """信念传播消息 (loopy BP 的 message 抽象)。

    在层次场景图中, 消息沿以下方向流动:
      Object → Room (上行): 物体存在性证据聚合为房间类型后验
      Room → Object (下行): 房间类型后验调整物体先验
      Object ↔ Object (横向): 空间邻居信念共享
      Room → Phantom (生成): 房间期望但未见物体 → 产生 phantom node
    """
    source_type: str      # "object" | "room" | "phantom"
    source_id: int
    target_type: str      # "object" | "room" | "phantom"
    target_id: int
    message_type: str     # "existence" | "room_type" | "spatial_context"
    delta_alpha: float = 0.0
    delta_beta: float = 0.0
    weight: float = 1.0


@dataclass
class ViewNode:
    """关键视角节点 (FSR-VLN 风格的 view 层近似实现)。"""

    view_id: int
    position: np.ndarray
    timestamp: float
    room_id: int = -1
    object_ids: List[int] = field(default_factory=list)
    key_labels: List[str] = field(default_factory=list)


@dataclass
class TrackedObject:
    """全局跟踪的单个物体实例 (BA-HSG + USS-Nav 点云融合)。"""
    object_id: int
    label: str
    position: np.ndarray          # [x, y, z] world frame (点云质心)
    best_score: float
    detection_count: int = 1
    last_seen: float = 0.0        # timestamp
    features: np.ndarray = field(default_factory=lambda: np.array([]))
    region_id: int = -1           # 所属区域 (SG-Nav 层次)
    extent: np.ndarray = field(default_factory=lambda: np.array([0.2, 0.2, 0.2]))
    # USS-Nav: 物体累积点云 (N, 3) world frame
    points: np.ndarray = field(default_factory=lambda: np.empty((0, 3)))

    # ── BA-HSG 信念状态 ──
    # Beta(α, β) 存在性分布: P(exists) ~ Beta(α, β)
    belief_alpha: float = 1.5     # 初始偏乐观
    belief_beta: float = 1.0
    # 位置不确定性 (各向同性 Gaussian σ²)
    position_variance: float = 1.0  # 初始高不确定性 (m²)
    # 负面证据: 在预期视域内连续未检测到的次数
    miss_streak: int = 0
    # 复合可信度 (缓存值, 由 compute_credibility() 更新)
    credibility: float = 0.5

    # ── KG 知识增强 (ConceptBot / OpenFunGraph) ──
    kg_concept_id: str = ""        # 匹配的 KG 概念 ID
    safety_level: str = "safe"     # safe / caution / dangerous / forbidden
    affordances: List[str] = field(default_factory=list)  # graspable, openable, ...
    functional_properties: Dict = field(default_factory=dict)  # KG 补充属性
    floor_level: int = 0           # 所在楼层

    # ── KG-Augmented Prior (论文核心创新) ──
    # 与 BSG (ICRA 2024) 的 GCN 学习不同, 我们用 KG 结构化先验直接注入 Beta 参数
    kg_prior_alpha: float = 0.0    # KG 注入的先验 α (累计, 可分解来源)
    kg_prior_source: str = ""      # 先验来源追踪 ("room_type:office", "co_occurrence:desk")
    is_kg_expected: bool = False   # 此物体是否为当前房间类型的 KG 期望物体
    safety_nav_threshold: float = 0.25   # 导航避障用的安全阈值 (动态设置)
    safety_interact_threshold: float = 0.40  # 交互操作用的安全阈值

    # ── 来源标签 (observed vs kg_prior) ──
    source: str = "observed"           # "observed" | "kg_prior" | "loaded"
    last_observed_time: float = 0.0    # 最后一次被真实相机检测到的时间
    is_simulated: bool = False         # 标记是否为模拟/先验数据

    def update(self, det, alpha: float = 0.3) -> None:
        """用新检测更新位置、置信度、点云和信念状态 (USS-Nav + BA-HSG)。"""
        # USS-Nav: 融合点云 → 更新质心位置
        if hasattr(det, 'points') and det.points is not None and len(det.points) > 0:
            self._fuse_pointcloud(det.points)
            self.position = np.mean(self.points, axis=0) if len(self.points) > 0 else det.position.copy()
            self.position_variance = max(0.01, self.position_variance * 0.8)
        else:
            depth = float(np.linalg.norm(det.position[:2]))
            obs_var = (BELIEF_SIGMA_BASE + BELIEF_SIGMA_DEPTH_COEFF * depth) ** 2
            new_var = 1.0 / (1.0 / max(self.position_variance, 1e-6) + 1.0 / max(obs_var, 1e-6))
            self.position = new_var * (
                self.position / max(self.position_variance, 1e-6)
                + det.position / max(obs_var, 1e-6)
            )
            self.position_variance = new_var

        self.best_score = max(self.best_score, det.score)
        self.detection_count += 1
        self.last_seen = time.time()
        self.miss_streak = 0

        safety_scale = SAFETY_PRIOR_ALPHA_SCALE.get(self.safety_level, 1.0)
        self.belief_alpha += 1.0 * safety_scale

        if det.features.size > 0:
            self.features = self._fuse_feature(det.features)
        self._update_extent(det)
        self._update_credibility()

    def _fuse_pointcloud(
        self, new_points: np.ndarray, max_total: int = 1024,
    ) -> None:
        """
        USS-Nav: 增量融合新观测点云到物体累积点云。

        策略: 合并 → 体素降采样 → 限制最大点数。
        """
        if new_points is None or len(new_points) == 0:
            return
        if self.points is None or len(self.points) == 0:
            self.points = new_points.copy()
        else:
            self.points = np.vstack([self.points, new_points])

        from .projection import _voxel_downsample, POINTCLOUD_VOXEL_SIZE
        self.points = _voxel_downsample(self.points, POINTCLOUD_VOXEL_SIZE, max_total)

    def _fuse_feature(self, new_feature: np.ndarray) -> np.ndarray:
        """多视角 CLIP 特征融合 (ConceptGraphs 风格 EMA + L2 normalize)。

        创新1 补强: 每次新观测以 α=0.3 权重融入历史特征 (0.7),
        并做 L2 归一化保证特征在单位球面上。当检测质量 (best_score) 较高时
        给予新观测更大权重, 质量低时减小权重, 对齐 ConceptGraphs 论文的
        multi-view feature aggregation 思路。

        质量感知 α 计算:
          base_alpha = 0.3
          quality_factor = clamp(best_score / 0.8, 0.5, 1.5)
          alpha = min(0.5, base_alpha * quality_factor)
        """
        nf = np.asarray(new_feature, dtype=np.float64)
        if nf.size == 0:
            return self.features

        nf_norm = np.linalg.norm(nf)
        if nf_norm > 0:
            nf = nf / nf_norm

        if self.features.size == 0:
            return nf

        of = np.asarray(self.features, dtype=np.float64)
        of_norm = np.linalg.norm(of)
        if of_norm > 0:
            of = of / of_norm

        # ConceptGraphs 风格: 固定 base_alpha = 0.3, 质量感知调节
        base_alpha = 0.3
        quality_factor = max(0.5, min(1.5, self.best_score / 0.8))
        alpha = min(0.5, base_alpha * quality_factor)

        fused = (1.0 - alpha) * of + alpha * nf
        fused_norm = np.linalg.norm(fused)
        if fused_norm > 0:
            fused = fused / fused_norm
        return fused

    def _update_extent(self, det) -> None:
        """从 2D bbox 和深度估算 3D 包围盒半径。"""
        if det.depth <= 0 or det.bbox_2d.size < 4:
            return
        x1, y1, x2, y2 = det.bbox_2d
        bbox_w = max(float(x2 - x1), 1.0)
        bbox_h = max(float(y2 - y1), 1.0)
        # 使用真实 fx (由 update() 传入), 否则 fallback 到 600
        fx_approx = getattr(det, '_intrinsics_fx', 0.0)
        if fx_approx <= 0:
            fx_approx = 600.0
        extent_x = (bbox_w / fx_approx) * det.depth * 0.5
        extent_y = extent_x  # 对称近似
        extent_z = (bbox_h / fx_approx) * det.depth * 0.5
        new_ext = np.array([
            max(extent_x, 0.05),
            max(extent_y, 0.05),
            max(extent_z, 0.05),
        ])
        # EMA 平滑尺寸
        self.extent = 0.3 * new_ext + 0.7 * self.extent

    # ── BA-HSG 信念方法 ──

    @property
    def existence_prob(self) -> float:
        """Beta 后验均值: P(exists) = α / (α + β)。"""
        return self.belief_alpha / (self.belief_alpha + self.belief_beta)

    @property
    def existence_uncertainty(self) -> float:
        """Beta 后验方差 (不确定性指标)。"""
        a, b = self.belief_alpha, self.belief_beta
        return (a * b) / ((a + b) ** 2 * (a + b + 1))

    def record_miss(self) -> None:
        """在预期视域内未检测到该物体 → 负面证据。"""
        self.miss_streak += 1
        self.belief_beta += BELIEF_NEG_EVIDENCE_WEIGHT
        self._update_credibility()

    def _update_credibility(self) -> None:
        """计算复合可信度 C — 安全感知版 (BA-HSG v2)。

        与 BSG (ICRA 2024) 的区别: 我们的可信度计算融入了
        KG 先验强度和安全等级, 而不仅仅是检测统计量。

        公式:
          C = w_exist × P_exist + w_view × view_div + w_fresh × freshness
              + w_det × det_score + w_kg × kg_factor
        其中 kg_factor = min(kg_prior_alpha / 3, 0.3) 反映 KG 对该物体的信心。
        """
        dt = time.time() - self.last_seen if self.last_seen > 0 else 999.0
        view_diversity = 1.0 - 1.0 / max(self.detection_count, 1)
        freshness = math.exp(-dt / BELIEF_FRESHNESS_TAU)
        kg_factor = min(self.kg_prior_alpha / 3.0, 0.3) if self.kg_prior_alpha > 0 else 0.0

        self.credibility = (
            0.35 * self.existence_prob
            + 0.15 * view_diversity
            + 0.15 * freshness
            + 0.15 * min(self.best_score, 1.0)
            + 0.20 * kg_factor
        )

        # 动态设置安全阈值
        self.safety_nav_threshold = SAFETY_THRESHOLDS_NAVIGATION.get(
            self.safety_level, 0.25)
        self.safety_interact_threshold = SAFETY_THRESHOLDS_INTERACTION.get(
            self.safety_level, 0.40)

    @property
    def is_confirmed_for_navigation(self) -> bool:
        """导航层是否应将此物体视为障碍/标志 (低阈值, 宁可信其有)。"""
        return self.credibility >= self.safety_nav_threshold

    @property
    def is_confirmed_for_interaction(self) -> bool:
        """交互层是否允许与此物体进行操作 (高阈值, 严格确认)。"""
        return self.credibility >= self.safety_interact_threshold

    def to_belief_dict(self) -> dict:
        """导出信念状态 (供 LLM prompt 和日志使用)。"""
        d = {
            "P_exist": round(self.existence_prob, 3),
            "sigma_pos": round(math.sqrt(self.position_variance), 3),
            "credibility": round(self.credibility, 3),
            "detections": self.detection_count,
            "miss_streak": self.miss_streak,
            "confirmed_nav": self.is_confirmed_for_navigation,
            "confirmed_interact": self.is_confirmed_for_interaction,
        }
        if self.kg_concept_id:
            d["kg_concept"] = self.kg_concept_id
            d["safety"] = self.safety_level
            d["affordances"] = self.affordances
        if self.kg_prior_alpha > 0:
            d["kg_prior_alpha"] = round(self.kg_prior_alpha, 3)
            d["kg_prior_source"] = self.kg_prior_source
            d["is_kg_expected"] = self.is_kg_expected
        if self.floor_level != 0:
            d["floor"] = self.floor_level
        return d
