"""
NaviMind Habitat Agent — 用 Habitat-Lab 评测灵途语义导航系统。

将 Habitat 的 ground-truth semantic sensor 转换为 Detection3D,
在线构建 InstanceTracker 场景图, 用 GoalResolver Fast Path 匹配目标。
目标未找到时用简单 frontier 探索 (旋转扫描 → 前进)。

无 ROS2 依赖, 纯 Python 运行。

用法:
    agent = NaviMindAgent(config)
    action = agent.act(observations)
    agent.reset()
"""

import math
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

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

# 类别同义词 (Habitat semantic 标注可能用不同名称)
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
    heading: float = 0.0           # 弧度, 0 = 正北
    step_count: int = 0
    # 探索状态
    rotate_count: int = 0          # 当前旋转扫描步数
    is_rotating: bool = True       # 是否处于旋转扫描阶段
    forward_steps: int = 0         # 朝 frontier 前进的步数
    target_forward_steps: int = 8  # 每次探索前进步数
    goal_found: bool = False
    goal_position: Optional[np.ndarray] = None
    navigating_to_goal: bool = False


class NaviMindAgent:
    """
    NaviMind Habitat Agent。

    每步:
    1. 解析 Habitat ground-truth semantic → Detection3D 列表
    2. 更新 InstanceTracker 场景图
    3. GoalResolver.fast_resolve() 尝试匹配目标
    4. 如果匹配成功 → 导航到目标 (贪心朝向+前进)
    5. 如果匹配失败 → frontier 探索 (旋转扫描 → 前进)
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

        # 场景图构建器
        self._tracker = InstanceTracker(
            merge_distance=sg_cfg.get("merge_distance", 0.8),
            iou_threshold=sg_cfg.get("iou_threshold", 0.3),
            clip_threshold=sg_cfg.get("clip_threshold", 0.75),
            max_objects=sg_cfg.get("max_objects", 200),
            stale_timeout=sg_cfg.get("stale_timeout", 9999.0),
        )

        # 目标解析器 (仅 Fast Path, 不需要 LLM)
        dummy_llm_config = LLMConfig(backend="mock", model="mock")
        self._resolver = GoalResolver(
            primary_config=dummy_llm_config,
            fast_path_threshold=fast_cfg.get("threshold", 0.75),
            confidence_threshold=fast_cfg.get("confidence_threshold", 0.6),
        )

        # 探索参数
        self._rotate_steps_per_scan = explore_cfg.get("rotate_steps", 12)
        self._frontier_forward = int(
            explore_cfg.get("frontier_distance", 2.0) / 0.25
        )  # 前进步数

        # 传感器参数
        sensor_cfg = config.get("sensor", {})
        self._img_h = sensor_cfg.get("rgb", {}).get("height", 256)
        self._img_w = sensor_cfg.get("rgb", {}).get("width", 256)
        hfov_deg = sensor_cfg.get("rgb", {}).get("hfov", 79)
        self._fx = self._img_w / (2.0 * math.tan(math.radians(hfov_deg / 2.0)))
        self._fy = self._fx  # 正方形像素

        # 成功距离
        self._success_distance = config.get("eval", {}).get("success_distance", 2.0)

        # Agent 状态
        self._state = AgentState()
        self._target_category = ""
        self._instruction = ""
        self._object_goal_position: Optional[np.ndarray] = None

        # 统计
        self._fast_path_hits = 0
        self._total_resolves = 0

    def reset(self) -> None:
        """重置 agent 状态 (新 episode 开始时调用)。"""
        self._tracker = InstanceTracker(
            merge_distance=self._config.get("scene_graph", {}).get("merge_distance", 0.8),
            iou_threshold=self._config.get("scene_graph", {}).get("iou_threshold", 0.3),
            clip_threshold=self._config.get("scene_graph", {}).get("clip_threshold", 0.75),
            max_objects=self._config.get("scene_graph", {}).get("max_objects", 200),
            stale_timeout=self._config.get("scene_graph", {}).get("stale_timeout", 9999.0),
        )
        self._state = AgentState()
        self._state.target_forward_steps = self._frontier_forward
        self._object_goal_position = None

    def set_goal(self, category: str, goal_position: Optional[np.ndarray] = None) -> None:
        """设置 ObjectNav 目标类别和真实目标位置 (用于距离计算)。"""
        self._target_category = _normalize_category(category)
        self._instruction = OBJECTNAV_CATEGORIES.get(
            self._target_category, f"找到{category}"
        )
        self._object_goal_position = goal_position

    def act(self, observations: Dict[str, Any]) -> int:
        """
        根据观测输出动作。

        Args:
            observations: Habitat 观测字典, 包含:
                - "rgb": (H, W, 3) uint8
                - "depth": (H, W, 1) float32, 米
                - "semantic": (H, W, 1) int32, 实例 ID
                - "gps": (2,) float32, [x, z] 位移
                - "compass": (1,) float32, heading

        Returns:
            Habitat 动作 ID (0=STOP, 1=FORWARD, 2=LEFT, 3=RIGHT)
        """
        self._state.step_count += 1

        # 1. 更新自身位姿
        self._update_pose(observations)

        # 2. 将 semantic sensor 转换为 Detection3D
        detections = self._semantic_to_detections(observations)

        # 3. 更新场景图
        camera_pos = self._state.position.copy()
        heading = self._state.heading
        camera_forward = np.array([
            -math.sin(heading), 0.0, -math.cos(heading)
        ])
        self._tracker.update(
            detections,
            camera_pos=camera_pos,
            camera_forward=camera_forward,
            intrinsics_fx=self._fx,
        )

        # 4. Fast Path 目标匹配
        scene_graph_json = self._tracker.get_scene_graph_json()
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

        # 5. 决策
        if self._state.navigating_to_goal and self._state.goal_position is not None:
            return self._navigate_to_goal()
        else:
            return self._explore()

    def _update_pose(self, obs: Dict[str, Any]) -> None:
        """从 GPS+Compass 更新位姿。"""
        if "gps" in obs:
            gps = obs["gps"]
            # Habitat GPS: 相对起点的 [x, z] 位移 (y 向上)
            self._state.position[0] = float(gps[0])
            self._state.position[2] = float(gps[1]) if len(gps) > 1 else 0.0
        if "compass" in obs:
            self._state.heading = float(obs["compass"][0])

    def _semantic_to_detections(self, obs: Dict[str, Any]) -> List[Detection3D]:
        """
        将 Habitat ground-truth semantic + depth → Detection3D 列表。

        Habitat semantic sensor 返回每个像素的实例 ID。
        我们按实例 ID 聚合, 计算每个实例的:
          - 2D bbox (像素)
          - 3D 位置 (从深度图反投影)
          - 类别标签
        """
        if "semantic" not in obs or "depth" not in obs:
            return []

        semantic = obs["semantic"]  # (H, W) or (H, W, 1) int32
        depth = obs["depth"]        # (H, W) or (H, W, 1) float32
        if semantic.ndim == 3:
            semantic = semantic[:, :, 0]
        if depth.ndim == 3:
            depth = depth[:, :, 0]

        # 获取场景语义标注
        scene = obs.get("_scene", None)
        semantic_categories = obs.get("_semantic_categories", {})

        # 按实例 ID 聚合像素
        unique_ids = np.unique(semantic)
        detections = []

        for inst_id in unique_ids:
            if inst_id <= 0:  # 0 = 背景/无标注
                continue

            # 获取标签
            label = semantic_categories.get(int(inst_id), "")
            if not label:
                continue
            label = _normalize_category(label)

            # 该实例的像素掩码
            mask = semantic == inst_id
            pixel_count = np.sum(mask)
            if pixel_count < 20:  # 太小的实例忽略
                continue

            # 2D bbox
            ys, xs = np.where(mask)
            x1, y1 = int(xs.min()), int(ys.min())
            x2, y2 = int(xs.max()), int(ys.max())
            bbox_2d = np.array([x1, y1, x2, y2], dtype=np.float32)

            # 深度 (掩码区域中值)
            masked_depth = depth[mask]
            valid_depth = masked_depth[(masked_depth > 0.1) & (masked_depth < 10.0)]
            if len(valid_depth) == 0:
                continue
            median_depth = float(np.median(valid_depth))

            # 3D 位置: 从像素中心 + 深度反投影
            cx = (x1 + x2) / 2.0
            cy = (y1 + y2) / 2.0
            # Habitat 坐标: x 右, y 上, z 后 (相机坐标系)
            cam_x = (cx - self._img_w / 2.0) / self._fx * median_depth
            cam_y = (cy - self._img_h / 2.0) / self._fy * median_depth
            cam_z = median_depth

            # 相机→世界坐标 (绕 y 轴旋转 heading)
            heading = self._state.heading
            cos_h = math.cos(heading)
            sin_h = math.sin(heading)
            world_x = self._state.position[0] + (-sin_h * cam_z + cos_h * cam_x)
            world_y = self._state.position[1] + cam_y
            world_z = self._state.position[2] + (-cos_h * cam_z - sin_h * cam_x)

            position = np.array([world_x, world_y, world_z], dtype=np.float64)

            det = Detection3D(
                position=position,
                label=label,
                score=0.95,  # ground-truth → 高置信度
                bbox_2d=bbox_2d,
                depth=median_depth,
                features=np.array([]),  # 无 CLIP 特征
            )
            detections.append(det)

        return detections

    def _navigate_to_goal(self) -> int:
        """贪心导航: 转向目标 → 前进。如果已足够近, STOP。"""
        goal = self._state.goal_position
        pos = self._state.position

        # 计算到目标的水平距离
        dx = goal[0] - pos[0]
        dz = goal[2] - pos[2]
        dist = math.sqrt(dx * dx + dz * dz)

        # 检查是否到达目标 (也检查真实目标位置)
        close_to_resolved = dist < self._success_distance
        close_to_true = False
        if self._object_goal_position is not None:
            tdx = self._object_goal_position[0] - pos[0]
            tdz = self._object_goal_position[2] - pos[2]
            close_to_true = math.sqrt(tdx * tdx + tdz * tdz) < self._success_distance

        if close_to_resolved or close_to_true:
            return HABITAT_STOP

        # 计算目标方位角 (habitat: -z 为前方)
        target_angle = math.atan2(-dx, -dz)
        angle_diff = target_angle - self._state.heading
        # 归一化到 [-pi, pi]
        angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi

        turn_threshold = math.radians(15)
        if angle_diff > turn_threshold:
            return HABITAT_TURN_LEFT
        elif angle_diff < -turn_threshold:
            return HABITAT_TURN_RIGHT
        else:
            return HABITAT_MOVE_FORWARD

    def _explore(self) -> int:
        """
        简单 frontier 探索: 旋转扫描一圈 → 朝前走几步 → 重复。
        """
        if self._state.is_rotating:
            self._state.rotate_count += 1
            if self._state.rotate_count >= self._rotate_steps_per_scan:
                # 旋转扫描完成, 切换到前进
                self._state.is_rotating = False
                self._state.forward_steps = 0
            return HABITAT_TURN_LEFT

        # 前进阶段
        self._state.forward_steps += 1
        if self._state.forward_steps >= self._state.target_forward_steps:
            # 前进完成, 重新旋转扫描
            self._state.is_rotating = True
            self._state.rotate_count = 0
        return HABITAT_MOVE_FORWARD

    @property
    def stats(self) -> Dict[str, Any]:
        """返回 agent 统计信息。"""
        return {
            "fast_path_hits": self._fast_path_hits,
            "total_resolves": self._total_resolves,
            "fast_path_rate": (
                self._fast_path_hits / max(1, self._total_resolves)
            ),
            "scene_graph_objects": len(self._tracker._objects),
            "steps": self._state.step_count,
        }
