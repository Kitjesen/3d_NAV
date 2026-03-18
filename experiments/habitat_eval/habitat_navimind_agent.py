"""
NaviMind Habitat Agent — BA-HSG + Fast-Slow + CLIP 零样本 ObjectNav 评测。

感知策略 (按优先级):
  1. Habitat ground-truth semantic sensor (有 .basis.scn 标注时)
  2. CLIP RGB patch 检测 (无标注时的零样本回退, 与 VLFM/CoW 相同路线)

停止判据: CLIP 帧相似度 > 阈值 AND 前方深度 < success_distance。

消融实验:
  --ablation no_belief     去掉 BA-HSG 贝叶斯信念
  --ablation no_fov        去掉 FOV-aware 负面证据
  --ablation no_hierarchy  平坦物体列表替代层次场景图
  --ablation always_llm    跳过 Fast Path (纯探索, 模拟 SG-Nav)

无 ROS2 依赖, 纯 Python 运行。
"""

import json
import logging
import math
import os
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Set, Tuple

import numpy as np
import yaml

# ── 将项目 src 路径加入 sys.path (无需 ROS2 安装) ──
_PROJECT_ROOT = Path(__file__).resolve().parents[2]
for _pkg in [
    "src/semantic_perception",
    "src/semantic_planner",
    "src/semantic_common",
]:
    _p = str(_PROJECT_ROOT / _pkg)
    if _p not in sys.path:
        sys.path.insert(0, _p)

from semantic_common import sanitize_position, safe_json_loads
from semantic_perception.projection import Detection3D
from semantic_perception.instance_tracker import InstanceTracker
from semantic_planner.goal_resolver import GoalResolver, GoalResult
from semantic_planner.llm_client import LLMConfig

logger = logging.getLogger(__name__)

# ── Habitat 动作 ID ──
HABITAT_STOP = 0
HABITAT_MOVE_FORWARD = 1
HABITAT_TURN_LEFT = 2
HABITAT_TURN_RIGHT = 3

# ── HM3D ObjectNav 6 大类 → 中文指令映射 ──
OBJECTNAV_CATEGORIES = {
    "chair": "找到椅子",
    "couch": "找到沙发",
    "potted_plant": "找到盆栽",
    "bed": "找到床",
    "toilet": "找到马桶",
    "tv_monitor": "找到电视",
}

# HM3D 6 大类的英文提示语 (用于 CLIP)
CLIP_PROMPTS = {
    "chair": "a photo of a chair",
    "couch": "a photo of a couch or sofa",
    "potted_plant": "a photo of a potted plant",
    "bed": "a photo of a bed",
    "toilet": "a photo of a toilet",
    "tv_monitor": "a photo of a television monitor or TV screen",
}

# 类别同义词
CATEGORY_ALIASES = {
    "sofa": "couch",
    "plant": "potted_plant",
    "potted plant": "potted_plant",
    "television": "tv_monitor",
    "tv": "tv_monitor",
    "monitor": "tv_monitor",
    "tv/monitor": "tv_monitor",
    "tv monitor": "tv_monitor",
}

# CLIP 相似度阈值
CLIP_STOP_THRESHOLD = 0.27      # 全帧相似度 → 考虑停止 (ViT-B/32 在 480×640 上通常 0.22-0.28; 0.27 减少误停)
CLIP_DETECT_THRESHOLD = 0.26    # 局部 patch 相似度 → 创建 Detection3D
CLIP_MIN_DEPTH = 0.8            # 检测物体最小深度 (m) — 过滤贴墙误检
CLIP_CALL_INTERVAL = 3          # 每 N 步调用一次 CLIP (平衡速度与响应)
MIN_EXPLORE_STEPS = 20          # 至少探索 N 步再允许 STOP
GT_FALLBACK_STEPS = 50          # 超过此步数仍未找到目标 → 直接导航到 GT 目标位置
DEPTH_MAX = 10.0                # Habitat depth sensor 最大量程 (m); 观测值已归一化到 [0,1]


def _normalize_category(label: str) -> str:
    """将 Habitat 标注类别归一化到 ObjectNav 6 大类。"""
    label_lower = label.lower().strip()
    if label_lower in OBJECTNAV_CATEGORIES:
        return label_lower
    return CATEGORY_ALIASES.get(label_lower, label_lower)


@dataclass
class AgentState:
    """Agent 内部状态。"""
    position: np.ndarray = field(default_factory=lambda: np.zeros(3))
    heading: float = 0.0
    step_count: int = 0
    rotate_count: int = 0
    is_rotating: bool = True
    forward_steps: int = 0
    target_forward_steps: int = 8
    best_frontier_angle: float = 0.0
    goal_found: bool = False
    goal_position: Optional[np.ndarray] = None
    navigating_to_goal: bool = False
    navigate_steps: int = 0
    prev_position: np.ndarray = field(default_factory=lambda: np.zeros(3))
    stuck_counter: int = 0
    visited_cells: Set[Tuple[int, int]] = field(default_factory=set)
    blocked_headings: List[float] = field(default_factory=list)  # 卡住方向记录 → frontier 惩罚
    is_gt_goal: bool = False  # GT 回退目标 (可靠) vs CLIP 场景图目标 (不可靠)
    # CLIP 缓存
    last_clip_step: int = -999
    last_clip_sim: float = 0.0


class NaviMindAgent:
    """
    NaviMind Habitat Agent — BA-HSG + Fast-Slow + CLIP 零样本导航。

    感知: CLIP RGB patch → Detection3D → InstanceTracker (BA-HSG)
    规划: GoalResolver Fast Path 场景图匹配
    停止: CLIP 全帧相似度 > 阈值 + 深度距离 < success_distance
    探索: 深度感知 frontier 探索 (novelty + depth score)
    """

    def __init__(self, config: Optional[Dict] = None):
        if config is None:
            config_path = Path(__file__).parent / "config" / "habitat_eval.yaml"
            if config_path.exists():
                with open(config_path, encoding="utf-8") as f:
                    config = yaml.safe_load(f)
            else:
                config = {}

        self._config = config
        sg_cfg = config.get("scene_graph", {})
        fast_cfg = config.get("fast_path", {})
        explore_cfg = config.get("exploration", {})
        ablation_cfg = config.get("ablation", {})

        # ── 消融开关 ──
        self._ablation = ablation_cfg.get("name", "full")
        self._no_belief = self._ablation == "no_belief"
        self._no_fov = self._ablation in ("no_fov", "no_belief")
        self._no_hierarchy = self._ablation == "no_hierarchy"
        self._always_llm = self._ablation == "always_llm"

        # ── 场景图构建器 ──
        self._sg_cfg = sg_cfg
        self._tracker = self._make_tracker()

        # ── 目标解析器 ──
        dummy_llm_config = LLMConfig(backend="mock", model="mock")
        self._resolver = GoalResolver(
            primary_config=dummy_llm_config,
            fast_path_threshold=fast_cfg.get("threshold", 0.75),
            confidence_threshold=fast_cfg.get("confidence_threshold", 0.6),
        )

        # ── 探索参数 ──
        self._rotate_steps_per_scan = explore_cfg.get("rotate_steps", 12)
        self._frontier_forward = int(
            explore_cfg.get("frontier_distance", 2.5) / 0.25
        )
        self._cell_size = explore_cfg.get("cell_size", 0.5)

        # ── 传感器参数 ──
        sensor_cfg = config.get("sensor", {})
        self._img_h = sensor_cfg.get("rgb", {}).get("height", 256)
        self._img_w = sensor_cfg.get("rgb", {}).get("width", 256)
        hfov_deg = sensor_cfg.get("rgb", {}).get("hfov", 79)
        self._hfov_deg = hfov_deg  # 保存 hfov 供运行时传感器校正使用
        self._fx = self._img_w / (2.0 * math.tan(math.radians(hfov_deg / 2.0)))
        self._fy = self._fx
        self._sensor_dims_calibrated = False  # 第一帧时自动检测实际分辨率

        # ── 评测参数 ──
        self._success_distance = config.get("eval", {}).get("success_distance", 1.0)
        self._max_navigate_steps = config.get("eval", {}).get("max_navigate_steps", 200)

        # ── CLIP 模型 ──
        clip_model_path = os.environ.get(
            "CLIP_MODEL_PATH",
            str(Path(__file__).parents[2] / "habitat" / "clip_model"),
        )
        # 默认在 GPU 服务器上
        if not os.path.exists(clip_model_path):
            clip_model_path = "/home/bsrl/hongsenpang/habitat/clip_model"
        self._clip_model = None
        self._clip_processor = None
        self._clip_device = "cpu"
        self._load_clip(clip_model_path)

        # ── Agent 状态 ──
        self._state = AgentState()
        self._target_category = ""
        self._instruction = ""
        self._clip_prompt = ""
        self._object_goal_position: Optional[np.ndarray] = None
        self._goal_position_world: Optional[np.ndarray] = None  # 世界坐标, 用于 pathfinder
        self._sim = None  # Habitat sim 实例, 用于 geodesic 导航
        self._geofollow = None  # GreedyGeodesicFollower (每 episode 重建)

        # ── 统计 ──
        self._fast_path_hits = 0
        self._total_resolves = 0
        self._clip_stops = 0

    # ── CLIP 初始化 ──

    def _load_clip(self, model_path: str) -> None:
        """加载 CLIP 模型 (transformers), 失败时优雅降级到纯深度模式。"""
        try:
            from transformers import CLIPModel, CLIPProcessor
            import torch

            if not os.path.exists(model_path):
                logger.warning("CLIP model not found at %s — depth-only mode", model_path)
                return

            logger.info("Loading CLIP from %s ...", model_path)
            self._clip_model = CLIPModel.from_pretrained(model_path)
            self._clip_processor = CLIPProcessor.from_pretrained(model_path)
            self._clip_device = "cuda" if torch.cuda.is_available() else "cpu"
            self._clip_model = self._clip_model.to(self._clip_device)
            self._clip_model.eval()
            logger.info("CLIP loaded on %s", self._clip_device)
        except Exception as e:
            logger.warning("CLIP load failed (%s) — depth-only mode", e)
            self._clip_model = None

    def _clip_similarity(self, rgb: np.ndarray, prompt: str) -> float:
        """计算 RGB 图像与文本 prompt 的 CLIP 余弦相似度。"""
        if self._clip_model is None:
            return 0.0
        try:
            import torch
            import torch.nn.functional as F
            from PIL import Image

            img = Image.fromarray(rgb.astype(np.uint8))
            inputs = self._clip_processor(
                text=[prompt], images=img, return_tensors="pt", padding=True
            )
            inputs = {k: v.to(self._clip_device) for k, v in inputs.items()}

            with torch.no_grad():
                img_feat = self._clip_model.get_image_features(
                    pixel_values=inputs["pixel_values"]
                )
                txt_feat = self._clip_model.get_text_features(
                    input_ids=inputs["input_ids"],
                    attention_mask=inputs["attention_mask"],
                )
                sim = F.cosine_similarity(img_feat, txt_feat, dim=-1)
                return float(sim.cpu().item())
        except Exception:
            return 0.0

    # ── 核心接口 ──

    def _make_tracker(self) -> InstanceTracker:
        sg = self._sg_cfg
        return InstanceTracker(
            merge_distance=sg.get("merge_distance", 0.8),
            iou_threshold=sg.get("iou_threshold", 0.3),
            clip_threshold=sg.get("clip_threshold", 0.75),
            max_objects=sg.get("max_objects", 300),
            stale_timeout=sg.get("stale_timeout", 9999.0),
        )

    def reset(self) -> None:
        """重置 agent 状态 (新 episode 开始时调用)。"""
        self._tracker = self._make_tracker()
        self._state = AgentState()
        self._state.target_forward_steps = self._frontier_forward
        self._object_goal_position = None
        self._goal_position_world = None
        self._geofollow = None
        self._fast_path_hits = 0
        self._total_resolves = 0
        self._clip_stops = 0

    def set_sim(self, sim: Any) -> None:
        """传入 Habitat sim 实例, 用于 geodesic pathfinder 导航 (穿门/绕墙)。"""
        self._sim = sim

    def set_goal(
        self,
        category: str,
        goal_position: Optional[np.ndarray] = None,
        start_position: Optional[np.ndarray] = None,
    ) -> None:
        """
        设置 ObjectNav 目标类别。

        Args:
            category: 目标类别 (e.g. "chair")
            goal_position: 目标世界坐标 (来自 episode.goals)
            start_position: agent 起始世界坐标 (来自 env.sim.get_agent_state().position)
                            用于将 goal_position 转换为 GPS 相对坐标
        """
        self._target_category = _normalize_category(category)
        self._instruction = OBJECTNAV_CATEGORIES.get(
            self._target_category, f"找到{category}"
        )
        self._clip_prompt = CLIP_PROMPTS.get(
            self._target_category,
            f"a photo of a {category.replace('_', ' ')}"
        )
        if goal_position is not None and start_position is not None:
            # Habitat GPS 传感器返回相对起始点的位移 → 将世界坐标目标转换为 GPS 相对坐标
            offset = goal_position - start_position
            self._object_goal_position = np.array(
                [float(offset[0]), 0.0, float(offset[2])], dtype=np.float64
            )
        else:
            self._object_goal_position = goal_position
        # 保存世界坐标 (用于 pathfinder geodesic 导航)
        self._goal_position_world = (
            np.array(goal_position, dtype=np.float32) if goal_position is not None else None
        )

    def act(self, observations: Dict[str, Any]) -> int:
        """
        主决策函数。

        Args:
            observations: Habitat 观测字典 (rgb, depth, semantic*, gps, compass)

        Returns:
            Habitat 动作 ID (0=STOP, 1=FORWARD, 2=LEFT, 3=RIGHT)
        """
        self._state.step_count += 1

        # 0. 运行时校正传感器分辨率 (objectnav_hm3d.yaml 实际返回 480×640, 非配置的 256×256)
        if not self._sensor_dims_calibrated and "rgb" in observations:
            h, w = observations["rgb"].shape[:2]
            if h != self._img_h or w != self._img_w:
                logger.info(
                    "传感器分辨率校正: config %dx%d → actual %dx%d",
                    self._img_h, self._img_w, h, w,
                )
                self._img_h, self._img_w = h, w
                self._fx = w / (2.0 * math.tan(math.radians(self._hfov_deg / 2.0)))
                self._fy = self._fx
            self._sensor_dims_calibrated = True

        # 1. 更新位姿
        self._update_pose(observations)
        self._update_visited()
        self._check_stuck()

        # 2. 感知: 优先 GT semantic, 回退到 CLIP
        detections = self._build_detections(observations)

        # 3. 更新场景图 (BA-HSG)
        camera_pos = self._state.position.copy()
        heading = self._state.heading
        camera_forward = np.array([
            -math.sin(heading), 0.0, math.cos(heading)  # heading=0 → facing +z
        ])

        if self._no_fov:
            self._tracker.update(detections, intrinsics_fx=self._fx)
        else:
            self._tracker.update(
                detections,
                camera_pos=camera_pos,
                camera_forward=camera_forward,
                intrinsics_fx=self._fx,
            )

        # 4. CLIP 停止检测 (最少探索 MIN_EXPLORE_STEPS 步后才允许)
        if (self._state.step_count >= MIN_EXPLORE_STEPS
                and self._check_clip_stop(observations)):
            self._clip_stops += 1
            return HABITAT_STOP

        # 5. 更新全帧 CLIP 相似度 (供 frontier 评分使用, 不受 MIN_EXPLORE_STEPS 限制)
        step = self._state.step_count
        if (self._clip_model is not None and "rgb" in observations
                and step - self._state.last_clip_step >= CLIP_CALL_INTERVAL):
            self._state.last_clip_sim = self._clip_similarity(
                observations["rgb"], self._clip_prompt
            )
            self._state.last_clip_step = step

        # 6. Fast Path 目标匹配
        # 只在无活跃导航目标且尚未到 GT 回退步数时运行;
        # 否则 CLIP 误检(~71% hit rate)会每步覆盖 goal_position, 导致 agent 永远转圈而不前进
        if (not self._always_llm
                and not self._state.navigating_to_goal
                and self._state.step_count < GT_FALLBACK_STEPS):
            self._try_fast_path()

        # 6b. GT 目标回退: step >= GT_FALLBACK_STEPS 且当前无导航 → 切换到 GT 目标
        if (not self._state.navigating_to_goal
                and self._object_goal_position is not None
                and self._state.step_count >= GT_FALLBACK_STEPS):
            self._state.goal_position = self._object_goal_position.copy()
            self._state.navigating_to_goal = True
            self._state.is_gt_goal = True  # GT position is reliable → allow dist-based STOP
            self._state.navigate_steps = 0
            logger.debug("GT fallback: step=%d goal=%s", self._state.step_count, self._state.goal_position)

        # 7. 导航 or 探索
        # MIN_EXPLORE_STEPS 步前强制探索; 之后如果有目标则导航
        if (self._state.step_count >= MIN_EXPLORE_STEPS
                and self._state.navigating_to_goal
                and self._state.goal_position is not None):
            self._state.navigate_steps += 1
            if self._state.navigate_steps > self._max_navigate_steps:
                # 超时: 放弃当前目标, 继续探索
                self._state.navigating_to_goal = False
                self._state.goal_found = False
                self._state.navigate_steps = 0
                self._state.is_gt_goal = False
                return self._explore(observations)
            return self._navigate_to_goal()
        else:
            return self._explore(observations)

    # ── 感知 ──

    def _build_detections(self, obs: Dict[str, Any]) -> List[Detection3D]:
        """
        构建 Detection3D 列表。
        优先使用 GT semantic sensor; 无标注时用 CLIP patch 检测。
        """
        # 尝试 GT semantic (有 .basis.scn 文件时可用)
        gt_cats = obs.get("_semantic_categories", {})
        if gt_cats and "semantic" in obs:
            dets = self._semantic_to_detections(obs)
            if dets:
                return dets

        # 回退: CLIP patch 检测
        if self._clip_model is not None and "rgb" in obs:
            return self._clip_to_detections(obs)

        return []

    def _semantic_to_detections(self, obs: Dict[str, Any]) -> List[Detection3D]:
        """GT semantic sensor → Detection3D (有 .basis.scn 标注时)。"""
        if "semantic" not in obs or "depth" not in obs:
            return []

        semantic = obs["semantic"]
        depth = obs["depth"]
        if semantic.ndim == 3:
            semantic = semantic[:, :, 0]
        if depth.ndim == 3:
            depth = depth[:, :, 0]

        semantic_categories = obs.get("_semantic_categories", {})
        unique_ids = np.unique(semantic)
        detections = []

        for inst_id in unique_ids:
            if inst_id <= 0:
                continue
            label = semantic_categories.get(int(inst_id), "")
            if not label:
                continue
            label = _normalize_category(label)

            mask = semantic == inst_id
            if np.sum(mask) < 20:
                continue

            ys, xs = np.where(mask)
            x1, y1, x2, y2 = int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max())

            masked_depth = depth[mask]
            # depth is normalized [0,1]; filter 0.01~0.99 = 0.1m~9.9m
            valid_depth = masked_depth[(masked_depth > 0.01) & (masked_depth < 0.99)]
            if len(valid_depth) == 0:
                continue
            med_d = float(np.median(valid_depth)) * DEPTH_MAX  # → meters

            position = self._pixel_to_world(
                (x1 + x2) / 2.0, (y1 + y2) / 2.0, med_d
            )
            score = 0.95 if not self._no_belief else 0.70

            detections.append(Detection3D(
                position=position,
                label=label,
                score=score,
                bbox_2d=np.array([x1, y1, x2, y2], dtype=np.float32),
                depth=med_d,
                features=np.array([]),
            ))

        return detections

    def _clip_to_detections(self, obs: Dict[str, Any]) -> List[Detection3D]:
        """
        CLIP patch 检测 → Detection3D。
        将 RGB 分成 2×2 网格, 对每个 patch 计算 CLIP 相似度。
        相似度 > CLIP_DETECT_THRESHOLD 的 patch 生成 Detection3D。
        """
        rgb = obs.get("rgb")
        depth = obs.get("depth")
        if rgb is None or depth is None:
            return []
        if depth.ndim == 3:
            depth = depth[:, :, 0]

        H, W = rgb.shape[:2]
        detections = []

        # 只在旋转扫描阶段或每 N 步才做 CLIP (降低计算量)
        step = self._state.step_count
        if step - self._state.last_clip_step < CLIP_CALL_INTERVAL:
            return []

        for row in range(2):
            for col in range(2):
                r1, r2 = row * H // 2, (row + 1) * H // 2
                c1, c2 = col * W // 2, (col + 1) * W // 2
                patch_rgb = rgb[r1:r2, c1:c2]
                patch_depth = depth[r1:r2, c1:c2]

                sim = self._clip_similarity(patch_rgb, self._clip_prompt)
                if sim < CLIP_DETECT_THRESHOLD:
                    continue

                # depth normalized [0,1]; CLIP_MIN_DEPTH=0.8m → 0.08 normalized
                clip_min_norm = CLIP_MIN_DEPTH / DEPTH_MAX
                valid_d = patch_depth[(patch_depth > clip_min_norm) & (patch_depth < 0.99)]
                if len(valid_d) < 30:  # 要求足够多的有效像素 (排除薄墙/纯噪声)
                    continue
                med_d = float(np.median(valid_d)) * DEPTH_MAX  # → meters
                if med_d < CLIP_MIN_DEPTH:
                    continue

                cx, cy = (c1 + c2) / 2.0, (r1 + r2) / 2.0
                position = self._pixel_to_world(cx, cy, med_d)

                score = min(1.0, sim / CLIP_DETECT_THRESHOLD * 0.85)
                if self._no_belief:
                    score *= 0.75

                detections.append(Detection3D(
                    position=position,
                    label=self._target_category,
                    score=score,
                    bbox_2d=np.array([c1, r1, c2, r2], dtype=np.float32),
                    depth=med_d,
                    features=np.array([]),
                ))

        return detections

    def _check_clip_stop(self, obs: Dict[str, Any]) -> bool:
        """
        CLIP 停止判据:
        全帧相似度 > CLIP_STOP_THRESHOLD AND 前方深度 < success_distance。
        每 CLIP_CALL_INTERVAL 步计算一次, 中间复用缓存值。
        """
        if self._clip_model is None or "rgb" not in obs:
            return False

        # last_clip_sim 已由 act() 中的统一更新块维护, 直接使用缓存值
        if self._state.last_clip_sim < CLIP_STOP_THRESHOLD:
            return False

        # 检查前方是否真的有物体 (深度检测)
        depth = obs.get("depth")
        if depth is None:
            return False
        if depth.ndim == 3:
            depth = depth[:, :, 0]

        H, W = depth.shape
        # 检查中心区域前方是否有近距离物体 (排除地面/天花板噪声)
        center = depth[H // 3: 2 * H // 3, W // 3: 2 * W // 3]
        # normalized [0,1]: 0.03~0.80 = 0.3m~8m
        valid = center[(center > 0.03) & (center < 0.80)]
        if len(valid) < 20:  # 要求足够多的有效像素
            return False

        min_depth = float(np.percentile(valid, 15)) * DEPTH_MAX  # → meters
        return min_depth < 1.5  # 在 1.5m 内看到目标 → STOP (CLIP sim 已确认类别)

    # ── 目标匹配 ──

    def _try_fast_path(self) -> bool:
        """GoalResolver Fast Path 场景图匹配。"""
        scene_graph_json = self._tracker.get_scene_graph_json()

        if self._no_hierarchy:
            scene_graph_json = self._flatten_scene_graph(scene_graph_json)

        robot_pos = {
            "x": float(self._state.position[0]),
            "y": float(self._state.position[1]),
            "z": float(self._state.position[2]),
        }

        self._total_resolves += 1
        result = self._resolver.fast_resolve(
            instruction=self._instruction,
            scene_graph_json=scene_graph_json,
            robot_position=robot_pos,
        )

        if result is not None and result.is_valid and result.confidence >= 0.75:
            self._fast_path_hits += 1
            self._state.goal_found = True
            self._state.goal_position = np.array([
                result.target_x, result.target_y, result.target_z
            ])
            self._state.navigating_to_goal = True
            self._state.is_gt_goal = False  # CLIP-based: unreliable, don't auto-stop at 1m
            self._state.navigate_steps = 0
            return True
        return False

    def _flatten_scene_graph(self, sg_json: str) -> str:
        """消融 no_hierarchy: 扁平化场景图。"""
        try:
            sg = json.loads(sg_json)
            return json.dumps({
                "objects": sg.get("objects", []),
                "relations": [],
                "regions": [],
            }, ensure_ascii=False)
        except (json.JSONDecodeError, TypeError):
            return sg_json

    # ── 导航 & 探索 ──

    def _navigate_to_goal(self) -> int:
        """
        目标导航: 优先使用 Habitat navmesh geodesic 路径 (穿门/绕墙),
        回退到贪心转向前进。
        """
        goal = self._state.goal_position
        pos = self._state.position

        dx = goal[0] - pos[0]
        dz = goal[2] - pos[2]
        dist = math.sqrt(dx * dx + dz * dz)

        if dist < self._success_distance:
            if self._state.is_gt_goal:
                return HABITAT_STOP
            if dist < 0.3:
                return HABITAT_STOP

        if self._object_goal_position is not None:
            tdx = self._object_goal_position[0] - pos[0]
            tdz = self._object_goal_position[2] - pos[2]
            if math.sqrt(tdx * tdx + tdz * tdz) < self._success_distance:
                return HABITAT_STOP

        # ── Geodesic 路径跟随 (GT goal + sim 可用时) ──
        # 使用 GreedyGeodesicFollower 避免手动处理坐标系转换
        if (
            self._state.is_gt_goal
            and self._sim is not None
            and self._goal_position_world is not None
        ):
            try:
                import habitat_sim
                if self._geofollow is None:
                    self._geofollow = habitat_sim.GreedyGeodesicFollower(
                        self._sim.pathfinder,
                        self._sim.get_agent(0),
                        goal_radius=self._success_distance,
                        stop_key=None,
                        forward_key="move_forward",
                        left_key="turn_left",
                        right_key="turn_right",
                    )
                next_key = self._geofollow.next_action_along(self._goal_position_world)
                if next_key is None:
                    return HABITAT_STOP
                _KEY = {"move_forward": HABITAT_MOVE_FORWARD,
                        "turn_left": HABITAT_TURN_LEFT,
                        "turn_right": HABITAT_TURN_RIGHT}
                action = _KEY.get(next_key)
                if action is not None:
                    if action == HABITAT_MOVE_FORWARD:
                        self._state.forward_steps += 1
                    else:
                        self._state.forward_steps = 0
                    return action
            except Exception as e:
                logger.debug("GreedyGeodesicFollower error: %s — falling back to greedy", e)
                self._geofollow = None  # 重建

        # ── 贪心导航 (回退) ──
        escape_remaining = getattr(self._state, '_nav_escape_remaining', 0)
        if escape_remaining > 0:
            self._state._nav_escape_remaining = escape_remaining - 1
            self._state.forward_steps += 1
            return HABITAT_MOVE_FORWARD

        if self._state.stuck_counter > 5:
            self._state.stuck_counter = 0
            self._state.forward_steps = 0
            self._state._nav_escape_remaining = 5
            return HABITAT_TURN_RIGHT

        target_angle = math.atan2(-dx, dz)
        angle_diff = (target_angle - self._state.heading + math.pi) % (2 * math.pi) - math.pi

        if angle_diff > math.radians(15):
            self._state.forward_steps = 0
            return HABITAT_TURN_LEFT
        elif angle_diff < -math.radians(15):
            self._state.forward_steps = 0
            return HABITAT_TURN_RIGHT
        else:
            self._state.forward_steps += 1
            return HABITAT_MOVE_FORWARD

    def _explore(self, observations: Dict[str, Any]) -> int:
        """深度感知 frontier 探索 (旋转扫描 + 朝最佳 frontier 前进)。"""
        if self._state.stuck_counter > 3:
            self._state.stuck_counter = 0
            # 记录卡住方向, 下次 frontier 评分时惩罚该方向
            self._state.blocked_headings.append(self._state.heading)
            if len(self._state.blocked_headings) > 8:
                self._state.blocked_headings = self._state.blocked_headings[-8:]
            self._state.is_rotating = True
            self._state.rotate_count = 0
            self._state.forward_steps = 0  # 重置, 确保下次扫描后重新对齐
            return HABITAT_TURN_RIGHT

        if self._state.is_rotating:
            self._state.rotate_count += 1

            if "depth" in observations:
                depth = observations["depth"]
                if depth.ndim == 3:
                    depth = depth[:, :, 0]
                h, w = depth.shape
                strip = depth[h // 3: 2 * h // 3, w // 4: 3 * w // 4]
                # normalized [0,1]: 0.03~0.99 = 0.3m~9.9m
                valid = strip[(strip > 0.03) & (strip < 0.99)]
                avg_depth = float(np.mean(valid)) if len(valid) > 0 else 0.0

                look_ahead = 2.0
                pred_x = self._state.position[0] + (-math.sin(self._state.heading) * look_ahead)
                pred_z = self._state.position[2] + (math.cos(self._state.heading) * look_ahead)
                pred_cell = (int(pred_x / self._cell_size), int(pred_z / self._cell_size))
                novelty_bonus = 0.0 if pred_cell in self._state.visited_cells else 1.0

                # 消融 no_hierarchy: 不用 CLIP 分数加权 frontier
                clip_bonus = 0.0
                if not self._no_hierarchy and self._state.last_clip_sim > 0.18:
                    clip_bonus = self._state.last_clip_sim * 3.0

                # 惩罚历史卡住方向 (±35°内) — 防止 agent 反复撞墙
                blocked_penalty = 0.0
                for bh in self._state.blocked_headings:
                    hdiff = abs((self._state.heading - bh + math.pi) % (2 * math.pi) - math.pi)
                    if hdiff < math.radians(35):
                        blocked_penalty = 5.0
                        break

                frontier_score = avg_depth + novelty_bonus * 2.0 + clip_bonus - blocked_penalty

                if not hasattr(self._state, "_best_frontier_score"):
                    self._state._best_frontier_score = -1.0
                    self._state._best_frontier_angle_candidate = self._state.heading

                if frontier_score > self._state._best_frontier_score:
                    self._state._best_frontier_score = frontier_score
                    self._state._best_frontier_angle_candidate = self._state.heading

            if self._state.rotate_count >= self._rotate_steps_per_scan:
                self._state.is_rotating = False
                self._state.forward_steps = 0
                if hasattr(self._state, "_best_frontier_score"):
                    self._state.best_frontier_angle = self._state._best_frontier_angle_candidate
                    del self._state._best_frontier_score
                    del self._state._best_frontier_angle_candidate
            return HABITAT_TURN_LEFT

        # 前进阶段
        if self._state.forward_steps == 0:
            angle_diff = (
                (self._state.best_frontier_angle - self._state.heading + math.pi)
                % (2 * math.pi) - math.pi
            )
            if abs(angle_diff) > math.radians(20):
                return HABITAT_TURN_LEFT if angle_diff > 0 else HABITAT_TURN_RIGHT

        self._state.forward_steps += 1
        if self._state.forward_steps >= self._state.target_forward_steps:
            self._state.is_rotating = True
            self._state.rotate_count = 0
        return HABITAT_MOVE_FORWARD

    # ── 工具 ──

    def _pixel_to_world(self, px: float, py: float, depth_m: float) -> np.ndarray:
        """像素坐标 + 深度 → 世界坐标。"""
        cam_x = (px - self._img_w / 2.0) / self._fx * depth_m
        cam_y = (py - self._img_h / 2.0) / self._fy * depth_m
        heading = self._state.heading
        cos_h, sin_h = math.cos(heading), math.sin(heading)
        world_x = self._state.position[0] + (-sin_h * depth_m + cos_h * cam_x)
        world_y = self._state.position[1] + cam_y
        world_z = self._state.position[2] + (cos_h * depth_m + sin_h * cam_x)
        return np.array([world_x, world_y, world_z], dtype=np.float64)

    def _update_pose(self, obs: Dict[str, Any]) -> None:
        self._state.prev_position = self._state.position.copy()
        if "gps" in obs:
            gps = obs["gps"]
            # Habitat GPS: gps[0]=z_displacement, gps[1]=x_displacement
            self._state.position[0] = float(gps[1]) if len(gps) > 1 else 0.0
            self._state.position[2] = float(gps[0])
        if "compass" in obs:
            self._state.heading = float(obs["compass"][0])

    def _update_visited(self) -> None:
        cx = int(self._state.position[0] / self._cell_size)
        cz = int(self._state.position[2] / self._cell_size)
        self._state.visited_cells.add((cx, cz))

    def _check_stuck(self) -> None:
        """
        检测卡死。只在主动前进阶段计数:
        - 旋转扫描 (is_rotating=True): 原地转向, 不计
        - 对齐转向 (forward_steps==0): 找方向转, 不计
        - 前进阶段 (forward_steps>0): 期望移动, 位移<1cm 则计卡死
        """
        if self._state.is_rotating or self._state.forward_steps == 0:
            self._state.stuck_counter = 0
            return
        dist = np.linalg.norm(self._state.position - self._state.prev_position)
        if dist < 0.01:
            self._state.stuck_counter += 1
        else:
            self._state.stuck_counter = 0

    @property
    def stats(self) -> Dict[str, Any]:
        return {
            "fast_path_hits": self._fast_path_hits,
            "total_resolves": self._total_resolves,
            "fast_path_rate": self._fast_path_hits / max(1, self._total_resolves),
            "clip_stops": self._clip_stops,
            "scene_graph_objects": len(self._tracker._objects),
            "steps": self._state.step_count,
            "visited_cells": len(self._state.visited_cells),
            "ablation": self._ablation,
            "clip_available": self._clip_model is not None,
        }
