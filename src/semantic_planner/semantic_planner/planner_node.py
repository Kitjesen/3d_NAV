"""
semantic_planner_node — 语义规划 ROS2 节点 (论文级完整版)

参考论文:
  2026 前沿:
  - VLingNav (arXiv 2601.08665): AdaCoT 双进程推理 → Fast/Slow 路径
  - OmniNav (ICLR 2026): 统一导航 + Fast-Slow 系统 5Hz
  - AdaNav (ICLR 2026): 不确定性自适应推理深度
  - CompassNav (ICLR 2026): 强化微调 + Gap-Aware 奖励

  2025:
  - ESCA/SGCLIP (NeurIPS 2025): 选择性 Grounding → 场景图过滤
  - MTU3D (ICCV 2025): 统一 Grounding + Frontier 选择
  - MSGNav (2025): 多模态 3D 场景图
  - OpenFrontier (2025): FrontierNet + VLM 零样本
  - OrionNav (2025): 四足 + 语义场景图

  2024 基础:
  - SG-Nav (NeurIPS 2024): 层次场景图 + LLM 推理
  - SayCan (Google, 2022): 子目标分解
  - LOVON (2024): 动作原语 + 目标验证
  - VLFM (2023): Frontier 评分探索
  - VLMnav (2024): VLM Vision grounding
  - L3MVN (ICRA 2024): 拓扑记忆

核心流程:
  1. 接收指令 → 任务分解 (SayCan)
  2. 对每个子目标:
     a. FIND:        查场景图, CLIP 匹配
     b. NAVIGATE:    发布目标 PoseStamped
     c. LOOK_AROUND: 原地扫描 (LOVON)
     d. APPROACH:    减速接近
     e. VERIFY:      VLM 视觉验证 (LOVON + VLMnav)
     f. EXPLORE:     Frontier 评分 (VLFM) + 拓扑记忆 (L3MVN)
     g. BACKTRACK:   拓扑记忆回溯 (LOVON)
  3. 子目标失败 → 重试或跳到下一个
  4. 全部完成 → 任务成功

订阅:
  - instruction  (std_msgs/String, JSON)
  - scene_graph  (std_msgs/String, JSON)
  - odometry     (nav_msgs/Odometry)
  - /camera/color/image_raw (sensor_msgs/Image, 可选)
  - /nav/semantic/cancel (std_msgs/String, 任务取消)

发布:
  - resolved_goal (geometry_msgs/PoseStamped)
  - cmd_vel       (geometry_msgs/TwistStamped, 相对话题名)
  - status        (std_msgs/String, JSON)
"""

import asyncio
import json
import math
import re
import threading
import time
import traceback
from enum import Enum
from typing import Optional, Dict

from semantic_common import safe_json_loads

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import String, Empty, Float32

# B5: Nav2 NavigateToPose action
try:
    from nav2_msgs.action import NavigateToPose
    _HAS_NAV2 = True
except ImportError:
    _HAS_NAV2 = False

from .llm_client import LLMConfig
from .goal_resolver import GoalResolver, GoalResult
from .task_decomposer import (
    TaskDecomposer, TaskPlan, SubGoal,
    SubGoalAction, SubGoalStatus,
)
from .topological_memory import TopologicalMemory
from .episodic_memory import EpisodicMemory
from .action_executor import ActionExecutor, ActionCommand
from .frontier_scorer import FrontierScorer
from .exploration_strategy import generate_frontier_goal, extract_frontier_scene_data
from .sgnav_reasoner import SGNavReasoner, FrontierSelection
from .voi_scheduler import VoIScheduler, VoIConfig, SchedulerState, SchedulerAction
from .implicit_fsm_policy import (
    ImplicitFSMPolicy,
    ImplicitFSMObservation,
)
from .person_tracker import PersonTracker
from .bbox_navigator import BBoxNavigator, BBoxNavConfig
from .vlm_bbox_query import query_object_bbox


from .planner_state import PlannerState  # noqa: F811 — 独立模块，避免循环导入
from .nav2_mixin import Nav2Mixin
from .subgoal_mixin import SubgoalMixin
from .bbox_mixin import BBoxNavMixin
from .operational_mixin import OperationalMixin


class SemanticPlannerNode(
    Nav2Mixin,
    SubgoalMixin,
    BBoxNavMixin,
    OperationalMixin,
    Node,
):
    """语义规划 ROS2 节点 (论文级完整版)。"""

    def __init__(self):
        super().__init__("semantic_planner_node")

        # ── 参数声明 ──
        # 主 LLM
        self.declare_parameter("llm.backend", "openai")
        self.declare_parameter("llm.model", "gpt-4o-mini")
        self.declare_parameter("llm.api_key_env", "OPENAI_API_KEY")
        self.declare_parameter("llm.timeout_sec", 10.0)
        self.declare_parameter("llm.max_retries", 2)
        self.declare_parameter("llm.temperature", 0.2)

        # 备用 LLM
        self.declare_parameter("llm_fallback.backend", "qwen")
        self.declare_parameter("llm_fallback.model", "qwen-turbo")
        self.declare_parameter("llm_fallback.api_key_env", "DASHSCOPE_API_KEY")
        self.declare_parameter("llm_fallback.timeout_sec", 15.0)
        self.declare_parameter("llm_fallback.max_retries", 1)

        # 目标解析
        self.declare_parameter("goal_resolution.confidence_threshold", 0.6)
        self.declare_parameter("goal_resolution.replan_on_failure", True)
        self.declare_parameter("goal_resolution.max_replan_attempts", 3)
        self.declare_parameter("goal_resolution.fast_path_threshold", 0.75)

        # 融合权重 (C1, C2 参数化)
        self.declare_parameter("fusion.weight_label_match", 0.35)
        self.declare_parameter("fusion.weight_clip_sim", 0.35)
        self.declare_parameter("fusion.weight_detector_score", 0.15)
        self.declare_parameter("fusion.weight_spatial_hint", 0.15)
        self.declare_parameter("fusion.no_clip_weight_label", 0.55)
        self.declare_parameter("fusion.no_clip_weight_detector", 0.25)
        self.declare_parameter("fusion.no_clip_weight_spatial", 0.20)
        self.declare_parameter("fusion.detector_count_normalize", 3)

        # 探索策略
        self.declare_parameter("exploration.enable", True)
        self.declare_parameter("exploration.strategy", "frontier")
        self.declare_parameter("exploration.max_explore_steps", 20)
        self.declare_parameter("exploration.step_distance", 2.0)
        self.declare_parameter("exploration.costmap_topic", "/nav/costmap")
        self.declare_parameter("exploration.terrain_topic", "/nav/terrain_map")
        self.declare_parameter("exploration.terrain_grid_resolution", 0.2)  # m/cell
        self.declare_parameter("exploration.terrain_grid_half_size", 15.0)  # meters
        self.declare_parameter("exploration.frontier_score_threshold", 0.2)
        self.declare_parameter("exploration.frontier_min_size", 5)
        self.declare_parameter("exploration.frontier_max_count", 10)
        self.declare_parameter("exploration.frontier_cluster_radius_cells", 3)
        self.declare_parameter("exploration.frontier_distance_weight", 0.2)
        self.declare_parameter("exploration.frontier_novelty_weight", 0.3)
        self.declare_parameter("exploration.frontier_language_weight", 0.2)
        self.declare_parameter("exploration.frontier_grounding_weight", 0.3)
        # C6: frontier 评分阈值参数化 (ablation 实验依赖)
        self.declare_parameter("exploration.frontier_novelty_distance", 5.0)
        self.declare_parameter("exploration.frontier_nearby_object_radius", 3.0)
        self.declare_parameter("exploration.frontier_grounding_angle_threshold", 0.7854)
        self.declare_parameter("exploration.frontier_cooccurrence_bonus", 0.25)
        self.declare_parameter("exploration.frontier_grounding_spatial_bonus", 0.1)
        self.declare_parameter("exploration.frontier_grounding_keyword_bonus", 0.4)
        self.declare_parameter("exploration.frontier_grounding_relation_bonus", 0.15)
        # 创新3: Frontier 视觉评分权重 (VLFM 核心)
        self.declare_parameter("exploration.frontier_vision_weight", 0.0)
        # 创新4: 语义先验权重 (Topology-Aware Semantic Exploration)
        self.declare_parameter("exploration.frontier_semantic_prior_weight", 0.15)
        # 创新5: 语义不确定性权重 (RoomTypePosterior.entropy 驱动探索)
        self.declare_parameter("exploration.frontier_semantic_uncertainty_weight", 0.15)
        # R11: USS-Nav TSP + 失败记忆参数
        self.declare_parameter("exploration.frontier_tsp_reorder", True)
        self.declare_parameter("exploration.frontier_tsp_limit", 20)
        self.declare_parameter("exploration.frontier_tsp_ig_radius_cells", 10)
        self.declare_parameter("exploration.frontier_failure_penalty_radius", 3.0)
        self.declare_parameter("exploration.frontier_failure_penalty_decay", 0.7)

        # SG-Nav 对齐参数
        self.declare_parameter("exploration.sgnav.max_subgraphs", 6)
        self.declare_parameter("exploration.sgnav.use_llm_reasoning", True)
        self.declare_parameter("exploration.sgnav.heuristic_weight", 0.45)
        self.declare_parameter("exploration.sgnav.llm_weight", 0.55)
        self.declare_parameter("exploration.sgnav.frontier_base_weight", 0.55)
        self.declare_parameter("exploration.sgnav.room_gate_weight", 0.25)
        self.declare_parameter("exploration.sgnav.interp_decay_distance", 4.0)
        self.declare_parameter("exploration.sgnav.credibility_decay", 0.9)
        self.declare_parameter("exploration.sgnav.false_positive_penalty", 0.2)
        self.declare_parameter("exploration.sgnav.reject_threshold", 0.25)
        self.declare_parameter("exploration.sgnav.min_confidence_for_bypass", 0.85)
        self.declare_parameter("exploration.sgnav.arrival_reperception", True)
        # 创新3: 连续 Re-perception (导航中每 N 米触发)
        self.declare_parameter("exploration.sgnav.continuous_reperception", True)
        self.declare_parameter("exploration.sgnav.reperception_interval_m", 2.0)
        self.declare_parameter("exploration.sgnav.reperception_n_max", 10)
        self.declare_parameter("exploration.sgnav.reperception_s_thresh", 0.8)

        # LOVON 风格隐式 FSM: s_{t+1} = f_theta(obs_t, s_t, instruction)
        self.declare_parameter("fsm.mode", "implicit")            # explicit | implicit
        self.declare_parameter("fsm.implicit.weights_path", "")
        self.declare_parameter("fsm.implicit.strict", False)

        # B5: Nav2 NavigateToPose action 参数
        self.declare_parameter("nav2.use_action_client", True)
        self.declare_parameter("nav2.action_name", "navigate_to_pose")
        self.declare_parameter("nav2.action_timeout_sec", 120.0)

        # SCG 路径规划集成 (默认关闭, 向后兼容)
        self.declare_parameter("scg.enable", False)
        self.declare_parameter("scg.request_topic", "/nav/scg/plan_request")
        self.declare_parameter("scg.result_topic", "/nav/scg/plan_result")
        self.declare_parameter("scg.timeout_sec", 2.0)
        self.declare_parameter("scg.waypoint_interval_sec", 0.5)

        # 安全约束
        self.declare_parameter("safety.max_goal_distance", 50.0)

        # Vision grounding
        self.declare_parameter("vision.enable", True)
        self.declare_parameter("vision.image_topic", "/camera/color/image_raw")
        self.declare_parameter("vision.verify_threshold", 0.5)

        # 执行参数 (C8, C9 参数化)
        self.declare_parameter("execution.arrival_radius", 1.0)
        self.declare_parameter("execution.monitor_hz", 1.0)
        self.declare_parameter("execution.look_around_hz", 10.0)
        self.declare_parameter("execution.default_timeout", 300.0)

        # 拓扑记忆 (C7 参数化)
        self.declare_parameter("topo_memory.new_node_distance", 2.0)
        self.declare_parameter("topo_memory.max_nodes", 500)

        # 语义数据持久化目录 (mapping 时保存, navigation 时加载)
        self.declare_parameter("semantic_data_dir", "")  # 空=不加载/保存

        # 启动后自动发送的指令 (探索模式用, launch 参数传入)
        self.declare_parameter("initial_instruction", "")  # 空=不自动发送

        # Tag 记忆层: 用户手动标记的地点 JSON 文件路径 (空=仅内存，不持久化)
        self.declare_parameter("tagged_locations_path", "")

        # ── 读取参数 ──
        # base_url 参数声明 (YAML 中可选)
        self.declare_parameter("llm.base_url", "")
        self.declare_parameter("llm_fallback.base_url", "")

        # P1-4 fix: 空字符串 launch 参数不应覆盖 YAML 配置
        _backend = self.get_parameter("llm.backend").value
        if not _backend:
            _backend = "kimi"
            self.get_logger().info(
                "llm.backend is empty (launch override), using default: kimi"
            )

        primary_config = LLMConfig(
            backend=_backend,
            model=self.get_parameter("llm.model").value,
            api_key_env=self.get_parameter("llm.api_key_env").value,
            timeout_sec=self.get_parameter("llm.timeout_sec").value,
            max_retries=self.get_parameter("llm.max_retries").value,
            temperature=self.get_parameter("llm.temperature").value,
            base_url=self.get_parameter("llm.base_url").value,
        )

        fallback_config = LLMConfig(
            backend=self.get_parameter("llm_fallback.backend").value,
            model=self.get_parameter("llm_fallback.model").value,
            api_key_env=self.get_parameter("llm_fallback.api_key_env").value,
            timeout_sec=self.get_parameter("llm_fallback.timeout_sec").value,
            max_retries=self.get_parameter("llm_fallback.max_retries").value,
            base_url=self.get_parameter("llm_fallback.base_url").value,
        )

        # SCG 参数
        self._scg_enable = self.get_parameter("scg.enable").value
        self._scg_timeout = self.get_parameter("scg.timeout_sec").value
        self._scg_waypoint_interval = self.get_parameter("scg.waypoint_interval_sec").value

        self._confidence_threshold = self.get_parameter(
            "goal_resolution.confidence_threshold"
        ).value
        self._replan_on_failure = self.get_parameter(
            "goal_resolution.replan_on_failure"
        ).value
        self._max_replan_attempts = self.get_parameter(
            "goal_resolution.max_replan_attempts"
        ).value
        self._explore_enabled = self.get_parameter("exploration.enable").value
        self._exploration_strategy = self.get_parameter("exploration.strategy").value
        self._max_explore_steps = self.get_parameter("exploration.max_explore_steps").value
        self._step_distance = self.get_parameter("exploration.step_distance").value
        self._frontier_score_threshold = self.get_parameter(
            "exploration.frontier_score_threshold"
        ).value
        self._sgnav_use_llm_reasoning = self.get_parameter(
            "exploration.sgnav.use_llm_reasoning"
        ).value
        self._fsm_mode = self.get_parameter("fsm.mode").value
        self._max_goal_distance = self.get_parameter("safety.max_goal_distance").value
        self._arrival_radius = self.get_parameter("execution.arrival_radius").value
        self._monitor_hz = self.get_parameter("execution.monitor_hz").value
        self._vision_verify_threshold = self.get_parameter("vision.verify_threshold").value

        # ── Tag 记忆层初始化 ──
        from .tagged_locations import TaggedLocationStore
        _tagged_path = self.get_parameter("tagged_locations_path").value
        self._tag_store = TaggedLocationStore(_tagged_path)
        self.get_logger().info(
            f"TaggedLocationStore initialized, path='{_tagged_path}', "
            f"entries={len(self._tag_store.list_all())}"
        )

        # ── 目标解析器 ──
        self._resolver = GoalResolver(
            primary_config=primary_config,
            fallback_config=fallback_config,
            confidence_threshold=self._confidence_threshold,
            fast_path_threshold=self.get_parameter(
                "goal_resolution.fast_path_threshold"
            ).value,
            max_replan_attempts=self._max_replan_attempts,
            tagged_location_store=self._tag_store,
        )

        # ── 子模块 ──
        # KG 注入到 TaskDecomposer (安全门 + 可供性验证)
        try:
            from semantic_perception.knowledge_graph import IndustrialKnowledgeGraph
            _kg = IndustrialKnowledgeGraph()
            TaskDecomposer.set_knowledge_graph(_kg)
            self.get_logger().info(f"KG injected into TaskDecomposer ({len(_kg.get_all_concepts())} concepts)")
        except Exception as e:
            self.get_logger().warning(f"KG injection into TaskDecomposer failed (non-critical): {e}")
        self._decomposer = TaskDecomposer()
        self._topo_memory = TopologicalMemory(
            new_node_distance=self.get_parameter("topo_memory.new_node_distance").value,
            max_nodes=self.get_parameter("topo_memory.max_nodes").value,
        )
        # 若 CLIP 编码器已初始化，注入拓扑记忆
        if hasattr(self, '_clip_encoder') and self._clip_encoder is not None:
            self._topo_memory.set_clip_encoder(self._clip_encoder)
        self._episodic_memory = EpisodicMemory(clip_encoder=None)
        self._frontier_scorer = FrontierScorer(
            min_frontier_size=self.get_parameter("exploration.frontier_min_size").value,
            max_frontiers=self.get_parameter("exploration.frontier_max_count").value,
            cluster_radius_cells=self.get_parameter(
                "exploration.frontier_cluster_radius_cells"
            ).value,
            distance_weight=self.get_parameter("exploration.frontier_distance_weight").value,
            novelty_weight=self.get_parameter("exploration.frontier_novelty_weight").value,
            language_weight=self.get_parameter("exploration.frontier_language_weight").value,
            grounding_weight=self.get_parameter("exploration.frontier_grounding_weight").value,
            novelty_distance=self.get_parameter("exploration.frontier_novelty_distance").value,
            nearby_object_radius=self.get_parameter("exploration.frontier_nearby_object_radius").value,
            grounding_angle_threshold=self.get_parameter("exploration.frontier_grounding_angle_threshold").value,
            cooccurrence_bonus=self.get_parameter("exploration.frontier_cooccurrence_bonus").value,
            grounding_spatial_bonus=self.get_parameter("exploration.frontier_grounding_spatial_bonus").value,
            grounding_keyword_bonus=self.get_parameter("exploration.frontier_grounding_keyword_bonus").value,
            grounding_relation_bonus=self.get_parameter("exploration.frontier_grounding_relation_bonus").value,
            vision_weight=self.get_parameter("exploration.frontier_vision_weight").value,
            semantic_prior_weight=self.get_parameter("exploration.frontier_semantic_prior_weight").value,
            semantic_uncertainty_weight=self.get_parameter("exploration.frontier_semantic_uncertainty_weight").value,
            tsp_reorder=self.get_parameter("exploration.frontier_tsp_reorder").value,
            tsp_frontier_limit=self.get_parameter("exploration.frontier_tsp_limit").value,
            tsp_ig_radius_cells=self.get_parameter("exploration.frontier_tsp_ig_radius_cells").value,
        )
        # R11: 从参数覆盖失败记忆惩罚参数
        self._frontier_scorer._failure_penalty_radius = self.get_parameter(
            "exploration.frontier_failure_penalty_radius"
        ).value
        self._frontier_scorer._failure_penalty_decay = self.get_parameter(
            "exploration.frontier_failure_penalty_decay"
        ).value
        # 加载持久化语义数据 (房间-物体 KG, 拓扑记忆)
        self._semantic_data_dir = self.get_parameter("semantic_data_dir").value
        if self._semantic_data_dir:
            self._load_semantic_data(self._semantic_data_dir)

        # 注入语义先验引擎到 frontier 评分器 (创新4: KG 指导探索方向)
        self._frontier_scorer.set_semantic_prior_engine(
            self._resolver._semantic_prior_engine
        )
        # 注入 CLIP 编码器到语义先验引擎 (CLIP text-text 房间类型预测)
        if hasattr(self, '_clip_encoder') and self._clip_encoder is not None:
            self._resolver._semantic_prior_engine.set_clip_encoder(self._clip_encoder)
        self._sgnav_reasoner = SGNavReasoner(
            max_subgraphs=self.get_parameter("exploration.sgnav.max_subgraphs").value,
            use_llm_reasoning=self._sgnav_use_llm_reasoning,
            heuristic_weight=self.get_parameter("exploration.sgnav.heuristic_weight").value,
            llm_weight=self.get_parameter("exploration.sgnav.llm_weight").value,
            frontier_base_weight=self.get_parameter(
                "exploration.sgnav.frontier_base_weight"
            ).value,
            room_gate_weight=self.get_parameter(
                "exploration.sgnav.room_gate_weight"
            ).value,
            interp_decay_distance=self.get_parameter(
                "exploration.sgnav.interp_decay_distance"
            ).value,
            credibility_decay=self.get_parameter(
                "exploration.sgnav.credibility_decay"
            ).value,
            false_positive_penalty=self.get_parameter(
                "exploration.sgnav.false_positive_penalty"
            ).value,
            reject_threshold=self.get_parameter(
                "exploration.sgnav.reject_threshold"
            ).value,
            min_confidence_for_bypass=self.get_parameter(
                "exploration.sgnav.min_confidence_for_bypass"
            ).value,
            reperception_n_max=self.get_parameter(
                "exploration.sgnav.reperception_n_max"
            ).value,
            reperception_s_thresh=self.get_parameter(
                "exploration.sgnav.reperception_s_thresh"
            ).value,
        )
        self._implicit_fsm = ImplicitFSMPolicy(
            weights_path=self.get_parameter("fsm.implicit.weights_path").value,
            strict=self.get_parameter("fsm.implicit.strict").value,
        )
        self._action_executor = ActionExecutor()

        # BA-HSG: VoI 信息价值调度器 (替代固定 2m 触发)
        self._voi_scheduler = VoIScheduler(VoIConfig())
        self._voi_enabled = True
        self._last_voi_reperception_time: float = 0.0
        self._last_voi_slow_time: float = 0.0

        # SCG 路径规划内部状态
        self._scg_result_event = asyncio.Event()
        self._scg_latest_result: Optional[dict] = None

        # 跟随模式标记 (FOLLOW 动作通过现有导航闭环实现)
        self._follow_mode: bool = False
        self._follow_target_label: str = ""
        self._follow_timeout: float = 300.0
        self._follow_start_time: float = 0.0

        # PersonTracker: 实时人体位置追踪 (EMA 平滑 + 速度预测)
        self._person_tracker = PersonTracker(
            follow_distance=1.5,
            lost_timeout=3.0,
        )

        # BBox 视觉伺服导航器 (通用目标追踪, "看到就追" 模式)
        self._bbox_navigator = BBoxNavigator(BBoxNavConfig(
            target_distance=1.5,
        ))
        self._bbox_nav_active = False
        self._bbox_nav_timer = None
        self._robot_yaw: float = 0.0  # 从 odom 四元数提取，供 bbox_nav 使用

        # B5: Nav2 NavigateToPose action client
        self._use_nav2_action = self.get_parameter("nav2.use_action_client").value
        self._nav2_action_timeout = self.get_parameter("nav2.action_timeout_sec").value
        self._nav2_action_client: Optional[ActionClient] = None
        self._nav2_goal_handle: Optional[ClientGoalHandle] = None
        self._nav2_goal_active = False

        if self._use_nav2_action and _HAS_NAV2:
            action_name = self.get_parameter("nav2.action_name").value
            self._nav2_action_client = ActionClient(
                self, NavigateToPose, action_name
            )
            self.get_logger().info(f"Nav2 action client created: {action_name}")
        elif self._use_nav2_action and not _HAS_NAV2:
            self.get_logger().warn(
                "nav2_msgs not available, falling back to PoseStamped publishing"
            )
            self._use_nav2_action = False

        # ── 内部状态 ──
        self._state = PlannerState.IDLE
        self._current_instruction: Optional[str] = None
        self._current_language: str = "zh"
        self._current_explore_if_unknown: bool = True
        self._instruction_timeout: float = self.get_parameter(
            "execution.default_timeout"
        ).value
        self._latest_scene_graph: str = "{}"
        self._robot_position: Optional[Dict[str, float]] = None
        # 语义记忆: 标签地点存储 {name: {"x": float, "y": float, "z": float}}
        self._tagged_locations: Dict[str, Dict[str, float]] = {}
        self._current_goal: Optional[GoalResult] = None
        self._current_plan: Optional[TaskPlan] = None
        self._current_action_cmd: Optional[ActionCommand] = None
        self._replan_count: int = 0
        self._explore_count: int = 0
        self._task_start_time: float = 0.0
        self._failure_reason: str = ""
        self._execution_context: str = ""  # GAP: 闭环反馈 — 成功时累计场景变化
        self._wait_timer = None  # Fix #1: WAIT action one-shot timer handle
        self._nav_goal_timeout: float = 120.0  # Fix #5: per-goal navigation timeout (seconds)
        self._navigation_start_time: float = 0.0  # Fix #5: timestamp when NAVIGATING state entered
        self._last_scene_graph_time: float = 0.0  # Fix #6: timestamp of last scene graph update

        # 创新3: 连续 Re-perception 状态
        self._continuous_reperception = self.get_parameter(
            "exploration.sgnav.continuous_reperception"
        ).value
        self._reperception_interval_m = self.get_parameter(
            "exploration.sgnav.reperception_interval_m"
        ).value
        self._last_reperception_dist: float = 0.0  # 上次触发时的累计距离
        self._nav_accumulated_dist: float = 0.0    # 本次导航累计距离
        self._last_fb_position: Optional[Dict[str, float]] = None  # 上次 feedback 位置

        # 隐式 FSM 内部状态 (LOVON 风格 4-state)
        self._fsm_prev_instruction: str = ""
        self._fsm_mission_state: str = "searching_1"
        self._fsm_search_state: str = "had_searching_1"

        # Vision: 最近的相机帧 (用于 VLM grounding)
        self._latest_image_base64: Optional[str] = None
        self._vision_enabled = self.get_parameter("vision.enable").value

        # 场景图解析缓存: 避免在同一回调内对同一 JSON 字符串多次 json.loads
        # (KeySG / Hydra 启发: 只在数据真正变化时才触发全量解析)
        self._latest_scene_graph_parsed: Optional[dict] = None
        self._latest_scene_graph_hash: int = 0

        # 异步事件循环 (在单独线程中运行 LLM 调用)
        self._loop = asyncio.new_event_loop()
        self._futures_lock = threading.Lock()  # 保护 _pending_futures 的跨线程访问
        self._pending_futures: set = set()  # 防止 asyncio.Task 被 GC 回收
        self._active_mission_future = None  # 当前任务的 Future，新指令到达时取消
        self._async_thread = threading.Thread(
            target=self._run_async_loop, daemon=True
        )
        self._async_thread.start()

        # ── 订阅 ──
        # depth=1: instruction 是事件驱动，只关心最新一条指令
        self._sub_instruction = self.create_subscription(
            String, "instruction", self._instruction_callback, 1
        )
        # depth=2: scene_graph 1Hz 发布，depth=10 会积压 10s 旧数据
        # 通过哈希缓存跳过重复解析，进一步降低无效 CPU 开销
        self._sub_scene_graph = self.create_subscription(
            String, "scene_graph", self._scene_graph_callback, 2
        )
        self._sub_odom = self.create_subscription(
            Odometry, "odometry", self._odom_callback, 5
        )

        self._sub_costmap = None
        self._sub_terrain = None
        self._terrain_grid_resolution = 0.2
        self._terrain_grid_half_size = 15.0
        if self._exploration_strategy in ("frontier", "sg_nav"):
            costmap_topic = self.get_parameter("exploration.costmap_topic").value
            self._sub_costmap = self.create_subscription(
                OccupancyGrid, costmap_topic, self._costmap_callback, 1
            )
            # 订阅 terrain_map PointCloud2, 转为 costmap 供 frontier_scorer 使用
            terrain_topic = self.get_parameter("exploration.terrain_topic").value
            self._terrain_grid_resolution = self.get_parameter(
                "exploration.terrain_grid_resolution"
            ).value
            self._terrain_grid_half_size = self.get_parameter(
                "exploration.terrain_grid_half_size"
            ).value
            self._sub_terrain = self.create_subscription(
                PointCloud2, terrain_topic, self._terrain_to_costmap_callback, 1
            )
            self.get_logger().info(
                f"Frontier exploration enabled, costmap={costmap_topic}, "
                f"terrain={terrain_topic} (grid res={self._terrain_grid_resolution}m, "
                f"half_size={self._terrain_grid_half_size}m)"
            )

        # F1: 任务取消话题
        self._sub_cancel = self.create_subscription(
            String, "/nav/semantic/cancel", self._cancel_callback, 10
        )

        # Tag 记忆层: 接收标记地点指令
        # 消息格式 (JSON):
        #   标记指定坐标: {"name": "体育馆", "x": 25.0, "y": 30.0}
        #   标记当前位置: {"name": "体育馆"}  (不传 x/y 则用当前里程计位置)
        self._sub_tag_location = self.create_subscription(
            String, "/nav/semantic/tag_location", self._tag_location_callback, 10
        )

        # 几何层 stuck_final 信号 → 触发语义层重规划 (pct_path_adapter JSON 事件)
        self._sub_adapter_status = self.create_subscription(
            String, "/nav/adapter_status",
            self._planner_status_callback, 10
        )

        # FOLLOW_PERSON: 接收来自 task_manager 的跟随指令 (JSON)
        # 消息格式: {"target_label": "person", "follow_distance": 1.5,
        #            "timeout_sec": 300, "min_distance": 0.8, "max_distance": 5.0}
        # 空消息 ({}) 表示停止跟随
        self._sub_follow_person_cmd = self.create_subscription(
            String, "/nav/semantic/follow_person_cmd",
            self._follow_person_cmd_callback, 10
        )

        # BBox 视觉伺服导航: "看到目标就追" 模式
        # 消息格式: {"target": "红色椅子"}          — VLM 模式: 用 VLM 在当前帧找目标 bbox
        #           {"target": "red chair", "mode": "vlm"}  — 同上 (显式指定)
        #           {"bbox": [x1,y1,x2,y2], "mode": "direct"}  — 直接给 bbox
        #           {} 或 {"stop": true}            — 停止
        self._sub_bbox_nav = self.create_subscription(
            String, "/nav/semantic/bbox_navigate",
            self._bbox_navigate_callback, 10
        )

        # 可选: 订阅相机图像 (用于 VLM vision grounding)
        if self._vision_enabled:
            image_topic = self.get_parameter("vision.image_topic").value
            self._sub_image = self.create_subscription(
                Image, image_topic, self._image_callback, 1
            )

        # ── 发布 ──
        self._pub_goal = self.create_publisher(PoseStamped, "resolved_goal", 10)
        # A8 修复: 使用相对话题名, 由 launch 文件 remap
        self._pub_cmd_vel = self.create_publisher(TwistStamped, "cmd_vel", 10)
        self._pub_status = self.create_publisher(String, "status", 10)
        self._pub_costmap = self.create_publisher(OccupancyGrid, "costmap_out", 1)  # 供 perception_node SCG 使用
        self._look_around_timer = None

        # FOLLOW_PERSON: 发布跟随参数到 /nav/semantic/follow_person (best-effort)
        _be_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._follow_person_pub = self.create_publisher(
            String, "/nav/semantic/follow_person", _be_qos
        )

        # SCG 路径规划: 发布请求, 订阅结果
        if self._scg_enable:
            scg_req_topic = self.get_parameter("scg.request_topic").value
            scg_res_topic = self.get_parameter("scg.result_topic").value
            self._pub_scg_request = self.create_publisher(String, scg_req_topic, 10)
            self._sub_scg_result = self.create_subscription(
                String, scg_res_topic,
                self._scg_result_callback,
                10,
            )
            self.get_logger().info(
                f"SCG path planning integration enabled: "
                f"req={scg_req_topic}, res={scg_res_topic}"
            )

        # ── 操作类命令发布器 ──
        self._patrol_cmd_pub = self.create_publisher(String, '/nav/patrol/command', 10)
        self._poi_cmd_pub = self.create_publisher(String, '/nav/poi/command', 10)
        self._map_cmd_pub = self.create_publisher(String, '/nav/map/command', 10)
        self._cancel_pub = self.create_publisher(Empty, '/nav/cancel', 10)
        self._speed_pub = self.create_publisher(Float32, '/nav/speed', 10)
        self._response_pub = self.create_publisher(String, '/nav/voice/response', 10)

        # ── 操作类动作集合 (非导航类, 直接路由到命令话题) ──
        self._OPERATIONAL_ACTIONS = {
            SubGoalAction.PATROL,
            SubGoalAction.SAVE_MAP,
            SubGoalAction.SAVE_POI,
            SubGoalAction.SET_SPEED,
            SubGoalAction.RETURN_HOME,
            SubGoalAction.PAUSE,
            SubGoalAction.RESUME,
        }

        # ── 监控定时器 (C9: 参数化频率) ──
        monitor_period = 1.0 / max(self._monitor_hz, 0.1)
        self._monitor_timer = self.create_timer(monitor_period, self._monitor_callback)

        self.get_logger().info(
            f"SemanticPlannerNode started: "
            f"primary_llm={primary_config.backend}/{primary_config.model}, "
            f"fallback={fallback_config.backend}/{fallback_config.model}, "
            f"exploration_strategy={self._exploration_strategy}, "
            f"fsm_mode={self._fsm_mode}, "
            f"implicit_fsm_ready={self._implicit_fsm.is_ready}"
        )

        # ── 自动发送初始指令 (探索模式 launch 参数 target:=...) ──
        _init_instr = self.get_parameter("initial_instruction").value
        if _init_instr:
            self.get_logger().info(
                f"Will auto-send instruction in 3s: '{_init_instr}'"
            )
            self._initial_instruction_timer = self.create_timer(
                3.0, lambda: self._fire_initial_instruction(_init_instr)
            )
        else:
            self._initial_instruction_timer = None

    def _make_twist_stamped(self, linear_x=0.0, linear_y=0.0, angular_z=0.0):
        """构造 TwistStamped 消息 (frame_id='body', stamp=当前时间)."""
        msg = TwistStamped()
        msg.header.frame_id = "body"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = linear_x
        msg.twist.linear.y = linear_y
        msg.twist.angular.z = angular_z
        return msg

    def _fire_initial_instruction(self, instruction: str):
        """启动后自动发送初始指令 (一次性定时器回调)."""
        if self._initial_instruction_timer is not None:
            self._initial_instruction_timer.cancel()
            self._initial_instruction_timer = None
        msg = String()
        msg.data = json.dumps({
            "instruction": instruction,
            "explore_if_unknown": True,
            "language": "zh",
        })
        self.get_logger().info(f"Auto-sending initial instruction: '{instruction}'")
        self._instruction_callback(msg)

    # ================================================================
    #  异步事件循环
    # ================================================================

    def _run_async_loop(self):
        """在单独线程中运行异步事件循环，崩溃后自动重启。"""
        while True:
            asyncio.set_event_loop(self._loop)
            try:
                self._loop.run_forever()
                break  # 正常关闭 (loop.stop() 调用)
            except Exception as e:
                self.get_logger().fatal(
                    f"Async event loop crashed: {e}\n{traceback.format_exc()}"
                )
                # P0-2 fix: 自动重建并重启事件循环
                try:
                    self._loop = asyncio.new_event_loop()
                    self.get_logger().warn("Async event loop restarted after crash")
                except Exception as e2:
                    self.get_logger().fatal(f"Async loop restart failed: {e2}")
                    break

    def _schedule_async(self, coro, is_mission: bool = False):
        """在异步线程中调度协程。保存 Future 引用防止 GC 回收。

        Args:
            coro: 要调度的协程
            is_mission: True = 这是主任务协程 (decompose/replan)，
                       新指令到达时会取消它
        """
        try:
            if self._loop.is_closed():
                self.get_logger().error(
                    "Async loop is closed, cannot schedule coroutine — "
                    "LLM/async operations unavailable"
                )
                return
            future = asyncio.run_coroutine_threadsafe(coro, self._loop)
            with self._futures_lock:
                self._pending_futures.add(future)
                if is_mission:
                    self._active_mission_future = future

            def _on_done(f):
                with self._futures_lock:
                    self._pending_futures.discard(f)
                    if self._active_mission_future is f:
                        self._active_mission_future = None

            future.add_done_callback(_on_done)
        except RuntimeError as e:
            self.get_logger().error(f"Failed to schedule async task: {e}")

    # ================================================================
    #  Callbacks
    # ================================================================

    def _instruction_callback(self, msg: String):
        """
        接收新的自然语言指令。

        流程 (SayCan + SG-Nav):
          1. 解析指令参数
          2. 尝试规则分解 (简单指令不需要 LLM)
          3. 规则分解失败 → LLM 分解
          4. 开始执行第一个子目标
        """
        try:
            data = json.loads(msg.data)
            instruction = data.get("instruction", "")
            language = data.get("language", "zh") or "zh"
            explore = data.get("explore_if_unknown", True)
            timeout = data.get("timeout_sec", self._instruction_timeout)
            radius = data.get("arrival_radius", self._arrival_radius)
            follow_person_mode = data.get("follow_person_mode", False)
            follow_target = data.get("target_label", "person") or "person"
            follow_distance = float(data.get("follow_distance", 1.5))
        except (json.JSONDecodeError, AttributeError) as e:
            # Fix #11: Log JSON parse failure instead of silent fallback
            self.get_logger().warn(f"Instruction JSON parse failed, using raw string: {e}")
            instruction = msg.data
            language = "zh"
            explore = True
            timeout = self._instruction_timeout
            radius = self._arrival_radius
            follow_person_mode = False
            follow_target = "person"
            follow_distance = 1.5

        # 人物跟随模式: 直接构建 FOLLOW 计划，跳过 LLM 分解
        if follow_person_mode:
            self.get_logger().info(
                "👤 Follow person: target='%s', distance=%.1fm, timeout=%.0fs",
                follow_target, follow_distance, timeout,
            )
            self._follow_mode = False  # 先清除旧状态
            self._current_instruction = f"跟随{follow_target}"
            self._current_language = language
            self._task_start_time = time.time()
            self._replan_count = 0
            self._action_executor.reset()
            plan = TaskPlan(
                instruction=self._current_instruction,
                subgoals=[SubGoal(
                    step_id=0,
                    action=SubGoalAction.FOLLOW,
                    target=follow_target,
                    parameters={"timeout": timeout, "follow_distance": follow_distance},
                )],
            )
            self._current_plan = plan
            self._execute_next_subgoal()
            return

        if not instruction:
            self.get_logger().warn("Empty instruction received, ignoring")
            return

        self.get_logger().info(
            f"New semantic instruction: '{instruction}' "
            f"(lang={language}, explore={explore})"
        )

        # 取消旧任务的 async 协程，防止旧 LLM 结果覆盖新指令状态
        with self._futures_lock:
            if self._active_mission_future and not self._active_mission_future.done():
                self._active_mission_future.cancel()
                self.get_logger().info("Cancelled previous mission coroutine")
                self._active_mission_future = None

        # 重置状态
        self._fsm_prev_instruction = self._current_instruction or instruction
        self._current_instruction = instruction
        self._current_language = language
        self._current_explore_if_unknown = explore
        self._instruction_timeout = timeout
        self._arrival_radius = radius
        self._replan_count = 0
        self._explore_count = 0
        self._execution_context = ""
        self._task_start_time = time.time()
        self._resolver.reset_exploration()
        self._frontier_scorer.clear_failure_memory()
        self._sgnav_reasoner.reset()
        self._fsm_mission_state = "searching_1"
        self._fsm_search_state = "had_searching_1"
        self._action_executor.reset()
        self._current_plan = None
        # 重置状态机——确保从 CANCELLED/FAILED 等终态能接受新指令
        if self._state in (PlannerState.CANCELLED, PlannerState.FAILED,
                           PlannerState.COMPLETED):
            self._state = PlannerState.IDLE

        # Step 1: 任务分解 (SayCan)
        plan = self._decomposer.decompose_with_rules(instruction)
        if plan:
            self.get_logger().info(
                f"Rule-based decomposition: {len(plan.subgoals)} subgoals"
            )
            # Check if this is an operational (non-navigation) intent
            if plan.subgoals and plan.subgoals[0].action in self._OPERATIONAL_ACTIONS:
                self.get_logger().info(
                    f"Operational intent detected: {plan.subgoals[0].action.value}, "
                    f"routing to command topics"
                )
                self._current_plan = plan
                self._execute_operational(plan)
                return
            self._current_plan = plan
            self._execute_next_subgoal()
        else:
            self.get_logger().info("Complex instruction, using LLM decomposition")
            self._set_state(PlannerState.DECOMPOSING)
            self._schedule_async(self._llm_decompose(), is_mission=True)

    def _tag_location_callback(self, msg: String):
        """Tag 记忆层: 接收 /nav/semantic/tag_location 消息，存储地点标签。

        消息格式 (JSON):
          标记指定坐标: {"name": "体育馆", "x": 25.0, "y": 30.0}
          标记当前位置: {"name": "体育馆"}  (不传 x/y 则用当前里程计位置)
          删除标签:     {"name": "体育馆", "delete": true}
        """
        try:
            from semantic_common import safe_json_loads
            data = safe_json_loads(msg.data, default=None)
            if not data or "name" not in data:
                self.get_logger().warning(
                    f"tag_location: invalid message (need 'name'): {msg.data!r}"
                )
                return

            name = str(data["name"])

            if data.get("delete"):
                removed = self._tag_store.remove(name)
                self.get_logger().info(
                    f"tag_location: {'removed' if removed else 'not found'} '{name}'"
                )
                self._tag_store.save()
                return

            if "x" in data and "y" in data:
                x = float(data["x"])
                y = float(data["y"])
                z = float(data.get("z", 0.0))
            elif self._robot_position is not None:
                x = self._robot_position["x"]
                y = self._robot_position["y"]
                z = self._robot_position.get("z", 0.0)
            else:
                self.get_logger().warning(
                    "tag_location: no coordinates provided and odometry not yet received"
                )
                return

            self._tag_store.tag(name, x=x, y=y, z=z, yaw=data.get("yaw"))
            self._tag_store.save()
            self.get_logger().info(
                f"tag_location: tagged '{name}' at ({x:.2f}, {y:.2f}, {z:.2f})"
            )
        except Exception as e:
            self.get_logger().error(f"tag_location callback error: {e}")

    def _planner_status_callback(self, msg: String):
        """响应 pct_adapter 的 stuck / stuck_final 信号, 触发语义层重规划。"""
        data = safe_json_loads(msg.data, default=None)
        if data is None:
            return

        event = data.get("event", "")
        if event == "stuck_final":
            now = time.time()
            if now - getattr(self, '_last_stuck_final_log', 0) > 5.0:
                self.get_logger().warn(
                    "[Geometric Stuck Final] 几何层重规划耗尽, 触发语义重规划"
                )
                self._last_stuck_final_log = now
            self._handle_geometric_stuck(data)

    def _scene_graph_callback(self, msg: String):
        """更新最新场景图 + 增量更新房间-物体 KG + 情节记忆。

        优化 (KeySG / Hydra 启发):
          - 通过 hash 检测场景图是否真正变化，未变化则跳过全量解析
          - JSON 只解析一次，结果缓存到 _latest_scene_graph_parsed
          - PersonTracker 直接复用已解析的 dict，消除第二次 json.loads
        """
        self._latest_scene_graph = msg.data
        self._last_scene_graph_time = time.time()  # Fix #6: Track scene graph freshness

        # 哈希检测: 内容未变化时跳过解析（Hydra keyframe-skip 思路）
        new_hash = hash(msg.data)
        if new_hash == self._latest_scene_graph_hash and self._latest_scene_graph_parsed is not None:
            # 场景图无变化，仅 PersonTracker 需要实时更新（跟随模式）
            if self._follow_mode:
                try:
                    self._person_tracker.update(
                        self._latest_scene_graph_parsed.get("objects", [])
                    )
                except (TypeError, KeyError, AttributeError):
                    pass
            return

        # 一次解析，缓存结果
        scene_data = safe_json_loads(msg.data, default=None)
        if scene_data is not None:
            self._latest_scene_graph_parsed = scene_data
            self._latest_scene_graph_hash = new_hash
        else:
            scene_data = self._latest_scene_graph_parsed or {}

        # 增量更新 runtime KG (每次场景图更新都提取房间-物体关系)
        if self._semantic_data_dir and hasattr(self, '_runtime_kg'):
            try:
                from .room_object_kg import extract_room_objects_from_scene_graph
                room_data = extract_room_objects_from_scene_graph(msg.data)
                for room_type, labels, confs in room_data:
                    self._runtime_kg.observe_room(room_type, labels, confs)
            except (ImportError, TypeError, KeyError, ValueError) as e:
                self.get_logger().debug(f"Runtime KG update failed (non-critical): {e}")

        # 注入房间类型后验到 frontier scorer (不确定性驱动探索)
        try:
            rp = scene_data.get("room_posteriors", {})
            if rp:
                self._frontier_scorer.set_room_type_posteriors_from_json(rp)
        except (TypeError, KeyError, AttributeError):
            pass

        # 更新情节记忆（复用已解析的 scene_data）
        try:
            _labels = [obj.get('label', '') for obj in scene_data.get('objects', [])]
            rp = self._robot_position
            if rp:
                _pos = np.array([rp['x'], rp['y'], rp.get('z', 0.0)])
                self._episodic_memory.add(position=_pos, labels=_labels)
        except (TypeError, KeyError, ValueError) as e:
            self.get_logger().debug(f"Episodic memory update failed: {e}")

        # 更新 PersonTracker（复用已解析 dict，消除第二次 json.loads）
        if self._follow_mode:
            try:
                self._person_tracker.update(scene_data.get("objects", []))
            except (TypeError, KeyError, AttributeError):
                pass

    def _image_callback(self, msg: Image):
        """缓存最新相机帧 (用于 VLM vision grounding)。"""
        try:
            import cv2
            import base64
            from cv_bridge import CvBridge

            if not hasattr(self, '_cv_bridge'):
                self._cv_bridge = CvBridge()
            bgr = self._cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            # 压缩为 JPEG base64 (降低分辨率以节省 API 费用)
            h, w = bgr.shape[:2]
            if w > 512:
                scale = 512 / w
                bgr = cv2.resize(bgr, (512, int(h * scale)))

            _, buf = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, 60])
            self._latest_image_base64 = base64.b64encode(buf).decode("utf-8")
        except Exception as e:
            # A3 修复: 不再 `pass`, 记录日志
            self.get_logger().debug(f"Image encoding failed (non-critical): {e}")

    def _odom_callback(self, msg: Odometry):
        """更新机器人位置，并基于 odometry 累计行驶距离触发 continuous_reperception。"""
        p = msg.pose.pose.position
        prev = self._robot_position
        self._robot_position = {"x": p.x, "y": p.y, "z": p.z}

        # 提取 yaw 角，供 BBox 导航器使用
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._robot_yaw = math.atan2(siny_cosp, cosy_cosp)

        # ── Odometry 驱动的连续 Re-perception（替代 Nav2 feedback）──
        try:
            if (
                prev is not None
                and self._continuous_reperception
                and self._state == PlannerState.NAVIGATING
                and self._current_goal
                and self._is_semantic_target(self._current_goal.target_label)
            ):
                dx = p.x - prev["x"]
                dy = p.y - prev["y"]
                self._nav_accumulated_dist += math.sqrt(dx * dx + dy * dy)
                dist_since = self._nav_accumulated_dist - self._last_reperception_dist
                if dist_since >= self._reperception_interval_m:
                    self._last_reperception_dist = self._nav_accumulated_dist
                    self._trigger_continuous_reperception()
        except Exception as e:
            self.get_logger().warn(f"Odom reperception trigger error: {e}")

    def _costmap_callback(self, msg: OccupancyGrid):
        """缓存占据栅格, 供 Frontier 评分探索使用。"""
        try:
            width = int(msg.info.width)
            height = int(msg.info.height)
            if width <= 0 or height <= 0:
                return

            expected = width * height
            if len(msg.data) != expected:
                self.get_logger().warn(
                    f"Invalid costmap data length: {len(msg.data)} != {expected}"
                )
                return

            grid = np.asarray(msg.data, dtype=np.int16).reshape((height, width))
            self._frontier_scorer.update_costmap(
                grid_data=grid,
                resolution=float(msg.info.resolution),
                origin_x=float(msg.info.origin.position.x),
                origin_y=float(msg.info.origin.position.y),
            )
        except Exception as e:
            self.get_logger().warn(f"Costmap parse failed: {e}")

    def _terrain_to_costmap_callback(self, msg: PointCloud2):
        """将 terrain_map PointCloud2 转为 2D 占据栅格, 供 frontier_scorer 使用。

        terrain_analysis 发布的 PointCloud2 包含障碍物点 (PointXYZI, odom 坐标系)。
        以机器人当前位置为中心, 构建局部 2D 栅格:
          - 有障碍物点的格子 → OCCUPIED (100)
          - 在观测范围内无障碍物的格子 → FREE (0)
          - 超出观测范围的格子 → UNKNOWN (-1)
        """
        try:
            if self._robot_position is None:
                return

            robot_x = self._robot_position["x"]
            robot_y = self._robot_position["y"]
            res = self._terrain_grid_resolution
            half = self._terrain_grid_half_size
            grid_size = int(2 * half / res)
            if grid_size <= 0:
                return

            origin_x = robot_x - half
            origin_y = robot_y - half

            # 解析 PointCloud2 → numpy xyz 数组
            points = self._parse_pointcloud2_xyz(msg)
            if points is None or len(points) == 0:
                return

            # 构建栅格: 默认 UNKNOWN
            grid = np.full((grid_size, grid_size), -1, dtype=np.int16)

            px = points[:, 0]
            py = points[:, 1]

            # 将世界坐标转为栅格坐标
            col = ((px - origin_x) / res).astype(np.int32)
            row = ((py - origin_y) / res).astype(np.int32)

            # 过滤越界点
            valid = (row >= 0) & (row < grid_size) & (col >= 0) & (col < grid_size)
            row = row[valid]
            col = col[valid]

            if len(row) == 0:
                return

            # 观测范围 (有数据的区域) 标记为 FREE
            row_min = max(0, int(row.min()))
            row_max = min(grid_size - 1, int(row.max()))
            col_min = max(0, int(col.min()))
            col_max = min(grid_size - 1, int(col.max()))
            grid[row_min:row_max + 1, col_min:col_max + 1] = 0  # FREE

            # 标记障碍物格子
            grid[row, col] = 100  # OCCUPIED

            self._frontier_scorer.update_costmap(
                grid_data=grid,
                resolution=res,
                origin_x=origin_x,
                origin_y=origin_y,
            )

            # 同时发布 OccupancyGrid，供 perception_node SCG 路径规划器使用
            occ = OccupancyGrid()
            occ.header = msg.header
            occ.header.frame_id = "map"
            occ.info.resolution = res
            occ.info.width = grid_size
            occ.info.height = grid_size
            occ.info.origin.position.x = origin_x
            occ.info.origin.position.y = origin_y
            occ.data = grid.flatten().astype(int).tolist()
            self._pub_costmap.publish(occ)

        except Exception as e:
            self.get_logger().warn(f"Terrain->costmap conversion failed: {e}")

    @staticmethod
    def _parse_pointcloud2_xyz(msg: PointCloud2) -> Optional[np.ndarray]:
        """从 PointCloud2 消息提取 xyz 坐标数组。

        支持 PointXYZI (x,y,z,intensity) 和 PointXYZ (x,y,z) 格式。
        返回 (N, 3) float32 数组, 或 None。
        """
        import struct

        if msg.width * msg.height == 0:
            return None

        field_map = {f.name: f for f in msg.fields}
        if "x" not in field_map or "y" not in field_map or "z" not in field_map:
            return None

        x_off = field_map["x"].offset
        y_off = field_map["y"].offset
        z_off = field_map["z"].offset
        point_step = msg.point_step
        n_points = msg.width * msg.height
        data = bytes(msg.data)

        if len(data) < n_points * point_step:
            return None

        # 快速路径: 字段紧密排列 (x=0, y=4, z=8)
        if x_off == 0 and y_off == 4 and z_off == 8 and point_step >= 12:
            if point_step == 12:
                xyz = np.frombuffer(data, dtype=np.float32).reshape(-1, 3)
            else:
                # 有填充字节 (如 PointXYZI, point_step=16 或 32)
                xyz = np.zeros((n_points, 3), dtype=np.float32)
                buf_array = np.frombuffer(data, dtype=np.uint8).reshape(
                    n_points, point_step
                )
                for i, off in enumerate([x_off, y_off, z_off]):
                    xyz[:, i] = np.frombuffer(
                        buf_array[:, off:off + 4].tobytes(), dtype=np.float32
                    )
        else:
            # 通用慢速路径
            xyz = np.zeros((n_points, 3), dtype=np.float32)
            for i in range(n_points):
                base = i * point_step
                xyz[i, 0] = struct.unpack_from('f', data, base + x_off)[0]
                xyz[i, 1] = struct.unpack_from('f', data, base + y_off)[0]
                xyz[i, 2] = struct.unpack_from('f', data, base + z_off)[0]

        # 过滤 NaN/Inf
        valid = np.isfinite(xyz).all(axis=1)
        return xyz[valid]

    # ================================================================
    #  目标解析 (VLingNav 双进程 + ESCA 选择性 Grounding)
    # ================================================================

    async def _resolve_goal(self):
        """
        异步解析目标 — Fast/Slow 双进程。

        参考:
          - VLingNav (2026): AdaCoT 自适应推理
          - OmniNav (ICLR 2026): Fast-Slow 系统
          - ESCA (NeurIPS 2025): 选择性 Grounding
          - AdaNav (ICLR 2026): 不确定性驱动推理深度
        """
        try:
            # Fix #10: Check for empty/stale scene graph before resolving
            if not self._latest_scene_graph or self._latest_scene_graph == "{}":
                sg_age = time.time() - self._last_scene_graph_time if self._last_scene_graph_time > 0 else float('inf')
                if sg_age > 5.0:
                    self.get_logger().warn(
                        f"Scene graph empty/stale (age={sg_age:.1f}s), "
                        "resolve may fall back to exploration"
                    )

            # Fix #4: Wrap resolve() with timeout to prevent indefinite hang
            # Consider @async_timeout when state callback not needed
            try:
                result = await asyncio.wait_for(
                    self._resolver.resolve(
                        instruction=self._current_instruction,
                        scene_graph_json=self._latest_scene_graph,
                        robot_position=self._robot_position,
                        language=self._current_language,
                        explore_if_unknown=self._current_explore_if_unknown,
                    ),
                    timeout=60.0,
                )
            except asyncio.TimeoutError:
                self.get_logger().error("Goal resolution timed out (60s)")
                self._subgoal_failed("resolve_timeout")
                return

            if not result.is_valid:
                self.get_logger().error(f"Goal resolution failed: {result.error}")
                self._subgoal_failed(result.error)
                return

            # LOVON 风格隐式 FSM: 用参数化模型决定状态转移
            if self._fsm_mode == "implicit":
                result = self._apply_implicit_fsm_transition(result)

            # OmniNav 层次子目标: Slow Path 返回房间提示时，先导航到房间中心
            if (
                result.action == "navigate"
                and result.hint_room_center
                and self._robot_position
            ):
                rc = result.hint_room_center
                rx = self._robot_position.get("x", 0.0)
                ry = self._robot_position.get("y", 0.0)
                dist_room = math.hypot(rc[0] - rx, rc[1] - ry)
                dist_final = math.hypot(
                    (result.target_x or 0.0) - rx,
                    (result.target_y or 0.0) - ry,
                )
                # 仅当房间中心在途中（比终点近 1m 以上）时才绕道
                if dist_room > 1.5 and dist_room < dist_final - 1.0:
                    self.get_logger().info(
                        f"OmniNav: room hint '{result.hint_room}' — "
                        f"navigating to room center ({rc[0]:.1f},{rc[1]:.1f}) "
                        f"first (dist={dist_room:.1f}m, final={dist_final:.1f}m)"
                    )
                    result.target_x = float(rc[0])
                    result.target_y = float(rc[1])
                    if len(rc) > 2:
                        result.target_z = float(rc[2])
                    result.hint_room_center = None  # 消费掉，下次 resolve 再找最终目标

            self._current_goal = result

            path_emoji = "⚡" if result.path == "fast" else "🧠"
            self.get_logger().info(
                f"{path_emoji} Goal via {result.path} path: "
                f"'{result.target_label}' conf={result.confidence:.2f}"
            )

            # Slow 路径 + 低置信度 + 有 VLM → 额外视觉验证 (VLMnav)
            if (
                result.path == "slow"
                and result.confidence < self._confidence_threshold
                and self._vision_enabled
                and self._latest_image_base64
            ):
                self.get_logger().info(
                    "Low confidence on slow path, trying vision grounding..."
                )
                try:
                    # Fix #4: Add timeout to vision_grounding call
                    vg_result = await asyncio.wait_for(
                        self._resolver.vision_grounding(
                            instruction=self._current_instruction,
                            scene_graph_json=self._latest_scene_graph,
                            image_base64=self._latest_image_base64,
                            language=self._current_language,
                        ),
                        timeout=30.0,
                    )
                    if vg_result.get("target_visible", False):
                        result.confidence = max(
                            result.confidence, vg_result.get("confidence", 0.5)
                        )
                        self.get_logger().info(
                            f"Vision grounding: target visible, "
                            f"confidence→{result.confidence:.2f}"
                        )
                except asyncio.TimeoutError:
                    self.get_logger().warn("Vision grounding timed out (30s), continuing")
                except Exception as e:
                    # A3 修复: 记录日志, 不静默失败
                    self.get_logger().warn(
                        f"Vision grounding failed (continuing): {e}"
                    )

            if result.action == "navigate":
                # SG-Nav: graph-based re-perception 过滤低可信假阳性目标
                if self._exploration_strategy == "sg_nav":
                    accepted = await self._sgnav_reperception_check(result)
                    if not accepted:
                        self.get_logger().warn(
                            "SG-Nav re-perception rejected target, switching to exploration"
                        )
                        await self._handle_explore_result(
                            GoalResult(
                                action="explore",
                                confidence=max(0.05, result.confidence * 0.5),
                                reasoning="SG-Nav re-perception rejected low-credibility target",
                                is_valid=True,
                                path="sg_nav",
                            )
                        )
                        return

                await self._handle_navigate_result(result)
            elif result.action == "explore":
                await self._handle_explore_result(result)
            else:
                self.get_logger().error(f"Unknown action: {result.action}")
                self._subgoal_failed(f"Unknown action: {result.action}")

        except Exception as e:
            self.get_logger().error(
                f"Goal resolution exception: {e}\n{traceback.format_exc()}"
            )
            self._subgoal_failed(str(e))

    # ================================================================
    #  SCG 路径规划集成
    # ================================================================

    def _scg_result_callback(self, msg: String):
        """接收 perception_node 返回的 SCG 规划结果。"""
        data = safe_json_loads(msg.data, default=None)
        if data is not None:
            self._scg_latest_result = data
        else:
            self.get_logger().warn("SCG result JSON parse error")
            self._scg_latest_result = {"success": False, "waypoints": [], "error": "JSON parse failed"}
        # 从 ROS2 回调线程安全地唤醒 async Event
        if self._loop and not self._loop.is_closed():
            self._loop.call_soon_threadsafe(self._scg_result_event.set)
        else:
            # 降级: 直接设置（可能在 shutdown 期间）
            try:
                self._scg_result_event.set()
            except RuntimeError:
                pass

    async def _plan_with_scg(
        self,
        start_pos: dict,
        goal_pos: tuple,
    ) -> Optional[list]:
        """
        通过 SCG 规划多路径点路径（异步版本，不阻塞 async event loop）。

        Args:
            start_pos: {"x": ..., "y": ..., "z": ...} 机器人当前位置
            goal_pos: (x, y, z) 目标位置

        Returns:
            waypoints 列表 [{"x":..,"y":..,"z":..}, ...] 或 None (失败时)
        """
        if not self._scg_enable:
            return None

        start = [
            start_pos.get("x", 0.0),
            start_pos.get("y", 0.0),
            start_pos.get("z", 0.0),
        ]
        goal = list(goal_pos)

        request_data = json.dumps({"start": start, "goal": goal})
        req_msg = String()
        req_msg.data = request_data

        # 清空旧结果, 重置事件
        self._scg_latest_result = None
        self._scg_result_event.clear()

        # 发布请求
        self._pub_scg_request.publish(req_msg)

        # 异步等待结果（不阻塞 event loop）
        try:
            await asyncio.wait_for(self._scg_result_event.wait(), timeout=self._scg_timeout)
            received = True
        except asyncio.TimeoutError:
            received = False
        if not received or self._scg_latest_result is None:
            self.get_logger().warn(
                f"SCG plan request timed out after {self._scg_timeout}s"
            )
            return None

        result = self._scg_latest_result
        if not result.get("success", False):
            self.get_logger().warn(
                f"SCG plan failed: {result.get('error', 'unknown error')}"
            )
            return None

        waypoints_raw = result.get("waypoints", [])
        if not waypoints_raw:
            return None

        waypoints = [
            {"x": float(wp[0]), "y": float(wp[1]), "z": float(wp[2])}
            for wp in waypoints_raw
            if len(wp) >= 3
        ]

        self.get_logger().info(
            f"SCG path planned: {len(waypoints)} waypoints, "
            f"distance={result.get('distance', 0.0):.2f}m, "
            f"poly_count={result.get('poly_count', 0)}"
        )
        return waypoints

    async def _handle_navigate_result(self, result: GoalResult):
        """处理导航结果 — 发布目标 (B5: 优先用 Nav2 action)。"""
        # 安全检查: 目标距离
        if self._robot_position:
            dist = math.sqrt(
                (result.target_x - self._robot_position["x"]) ** 2
                + (result.target_y - self._robot_position["y"]) ** 2
            )
            if dist > self._max_goal_distance:
                self.get_logger().warn(
                    f"Goal too far ({dist:.1f}m > {self._max_goal_distance}m), clamping"
                )
                scale = self._max_goal_distance / dist
                result.target_x = self._robot_position["x"] + (
                    result.target_x - self._robot_position["x"]
                ) * scale
                result.target_y = self._robot_position["y"] + (
                    result.target_y - self._robot_position["y"]
                ) * scale

        # SCG 路径规划 (若启用且机器人位置已知, 尝试生成多路径点)
        if self._scg_enable and self._robot_position:
            scg_waypoints = await self._plan_with_scg(
                start_pos=self._robot_position,
                goal_pos=(result.target_x, result.target_y, result.target_z),
            )
            if scg_waypoints and len(scg_waypoints) > 1:
                # SCG 成功: 逐个发布中间路径点为 PoseStamped, 最后一个为终点
                self.get_logger().info(
                    f"SCG navigation: publishing {len(scg_waypoints)} waypoints "
                    f"for target '{result.target_label}'"
                )
                self._set_state(PlannerState.NAVIGATING)
                for i, wp in enumerate(scg_waypoints[:-1]):
                    # 发布中间路径点
                    wp_pose = PoseStamped()
                    wp_pose.header.stamp = self.get_clock().now().to_msg()
                    wp_pose.header.frame_id = result.frame_id
                    wp_pose.pose.position.x = wp["x"]
                    wp_pose.pose.position.y = wp["y"]
                    wp_pose.pose.position.z = wp["z"]
                    # 朝向: 指向下一个路径点
                    next_wp = scg_waypoints[i + 1]
                    dx = next_wp["x"] - wp["x"]
                    dy = next_wp["y"] - wp["y"]
                    yaw = math.atan2(dy, dx)
                    wp_pose.pose.orientation.z = math.sin(yaw / 2)
                    wp_pose.pose.orientation.w = math.cos(yaw / 2)
                    self._pub_goal.publish(wp_pose)
                    time.sleep(self._scg_waypoint_interval)
                # 最终目标点
                final_wp = scg_waypoints[-1]
                goal_pose = PoseStamped()
                goal_pose.header.stamp = self.get_clock().now().to_msg()
                goal_pose.header.frame_id = result.frame_id
                goal_pose.pose.position.x = final_wp["x"]
                goal_pose.pose.position.y = final_wp["y"]
                goal_pose.pose.position.z = final_wp["z"]
                if self._robot_position:
                    dx = final_wp["x"] - result.target_x
                    dy = final_wp["y"] - result.target_y
                    yaw = math.atan2(dy, dx)
                    goal_pose.pose.orientation.z = math.sin(yaw / 2)
                    goal_pose.pose.orientation.w = math.cos(yaw / 2)
                else:
                    goal_pose.pose.orientation.w = 1.0
                if self._use_nav2_action and self._nav2_action_client is not None:
                    self._send_nav2_goal(goal_pose, result.target_label or "")
                else:
                    self._pub_goal.publish(goal_pose)
                self.get_logger().info(
                    f"SCG goal published: {result.target_label} via {len(scg_waypoints)} waypoints"
                )
                return
            else:
                # SCG 失败或只有单点: fallback 到原有逻辑
                self.get_logger().warn(
                    "SCG planning failed or returned single point, "
                    "falling back to direct PoseStamped"
                )

        # 构建 PoseStamped (原有逻辑 / SCG fallback)
        goal_pose = PoseStamped()
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.header.frame_id = result.frame_id  # 使用目标解析器返回的坐标系
        goal_pose.pose.position.x = result.target_x
        goal_pose.pose.position.y = result.target_y
        goal_pose.pose.position.z = result.target_z

        if self._robot_position:
            dx = result.target_x - self._robot_position["x"]
            dy = result.target_y - self._robot_position["y"]
            yaw = math.atan2(dy, dx)
            goal_pose.pose.orientation.z = math.sin(yaw / 2)
            goal_pose.pose.orientation.w = math.cos(yaw / 2)
        else:
            goal_pose.pose.orientation.w = 1.0

        # B5: 优先使用 Nav2 action client (有 feedback + 完成/失败回调)
        if self._use_nav2_action and self._nav2_action_client is not None:
            self._send_nav2_goal(goal_pose, result.target_label or "")
        else:
            # 兜底: 直接发布 PoseStamped (无 feedback)
            self._pub_goal.publish(goal_pose)

        self._set_state(PlannerState.NAVIGATING)

        self.get_logger().info(
            f"Goal published: {result.target_label} "
            f"({result.target_x:.2f}, {result.target_y:.2f}, {result.target_z:.2f}) "
            f"confidence={result.confidence:.2f}"
            f"{' [Nav2 action]' if self._use_nav2_action else ' [PoseStamped]'}"
        )

    # ================================================================
    #  监控
    # ================================================================

    def _monitor_callback(self):
        """
        定期监控任务状态。

        职责:
          1. 超时检测
          2. 到达检测 (NAVIGATING / EXPLORING / APPROACHING / BACKTRACKING)
          3. 拓扑记忆更新 (L3MVN)
          4. 发布状态
        """
        if self._state in (
            PlannerState.IDLE, PlannerState.COMPLETED,
            PlannerState.FAILED, PlannerState.CANCELLED,
        ):
            return

        # ── Fix #6: Scene graph staleness check ──
        if self._last_scene_graph_time > 0:
            sg_age = time.time() - self._last_scene_graph_time
            if sg_age > 10.0:
                self.get_logger().warn(
                    f"Scene graph stale ({sg_age:.1f}s > 10s), perception may be down"
                )

        # ── 拓扑记忆更新 (L3MVN) ──
        if self._robot_position:
            sg = safe_json_loads(self._latest_scene_graph, default={})
            try:
                visible_labels = [
                    obj["label"] for obj in sg.get("objects", [])[:10]
                ]
            except (KeyError, TypeError) as e:
                visible_labels = []
                self.get_logger().debug(f"Scene graph parse in monitor: {e}")

            # 从场景图提取当前所在房间
            _current_room_id = -1
            _current_room_name = ""
            try:
                _rooms = sg.get("rooms", [])
                if _rooms and self._robot_position:
                    _rx = self._robot_position.get("x", 0.0)
                    _ry = self._robot_position.get("y", 0.0)
                    _best_dist = float("inf")
                    for _room in _rooms:
                        _rc = _room.get("center", {})
                        _dx = float(_rc.get("x", 0)) - _rx
                        _dy = float(_rc.get("y", 0)) - _ry
                        _d = (_dx * _dx + _dy * _dy) ** 0.5
                        if _d < _best_dist:
                            _best_dist = _d
                            _current_room_id = int(_room.get("room_id", -1))
                            _current_room_name = str(_room.get("name", ""))
                    # 只在合理范围内（8m）才认为在该房间
                    if _best_dist > 8.0:
                        _current_room_id = -1
                        _current_room_name = ""
            except (json.JSONDecodeError, KeyError, TypeError, ValueError) as e:
                self.get_logger().debug(f"Topo memory scene graph parse failed: {e}")

            self._topo_memory.update_position(
                position=np.array([
                    self._robot_position["x"],
                    self._robot_position["y"],
                    self._robot_position.get("z", 0.0),
                ]),
                visible_labels=visible_labels,
                scene_snapshot=json.dumps(
                    {"nearby_labels": visible_labels[:5]}, ensure_ascii=False
                ),
                room_id=_current_room_id,
                room_name=_current_room_name,
            )

        # ── 超时检测 ──
        if self._task_start_time > 0:
            elapsed = time.time() - self._task_start_time
            if elapsed > self._instruction_timeout:
                self.get_logger().warn(
                    f"Semantic nav timeout ({elapsed:.0f}s > "
                    f"{self._instruction_timeout:.0f}s)"
                )
                self._set_state(PlannerState.FAILED, reason="timeout")
                return

        # ── Fix #5: Per-goal navigation timeout (PoseStamped mode) ──
        if self._state == PlannerState.NAVIGATING and self._navigation_start_time > 0:
            nav_elapsed = time.time() - self._navigation_start_time
            if nav_elapsed > self._nav_goal_timeout:
                self.get_logger().warn(
                    f"Navigation goal timeout ({nav_elapsed:.1f}s > {self._nav_goal_timeout}s)"
                )
                self._subgoal_failed("navigation_goal_timeout")
                return

        # ── 到达检测 ──
        arrival_states = (
            PlannerState.NAVIGATING, PlannerState.EXPLORING,
            PlannerState.APPROACHING, PlannerState.BACKTRACKING,
        )
        if self._state in arrival_states:
            if self._robot_position and self._current_goal:
                dist = math.sqrt(
                    (self._current_goal.target_x - self._robot_position["x"]) ** 2
                    + (self._current_goal.target_y - self._robot_position["y"]) ** 2
                )

                if dist < self._arrival_radius:
                    # Fix #9: When Nav2 action is active, let Nav2 result callback
                    # handle completion to avoid double _subgoal_completed race
                    if self._nav2_goal_active:
                        pass  # Let Nav2 result callback handle completion
                    elif self._state == PlannerState.NAVIGATING:
                        self.get_logger().info(
                            f"Arrived at target: {self._current_goal.target_label}"
                        )
                        self._subgoal_completed()

                    elif self._state == PlannerState.EXPLORING:
                        self.get_logger().info(
                            f"Arrived at exploration point {self._explore_count}, "
                            f"re-resolving..."
                        )
                        self._subgoal_completed()

                    elif self._state == PlannerState.APPROACHING:
                        self.get_logger().info("Approach complete")
                        self._subgoal_completed()

                    elif self._state == PlannerState.BACKTRACKING:
                        self.get_logger().info("Backtrack complete")
                        self._subgoal_completed()

        # 发布状态
        self._publish_status()

    # ================================================================
    #  状态管理
    # ================================================================

    def _publish_goal_from_command(self, cmd: ActionCommand):
        """将 ActionCommand 转为 PoseStamped 发布 (B5: 优先 Nav2 action)。"""
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = cmd.frame_id
        goal.pose.position.x = cmd.target_x
        goal.pose.position.y = cmd.target_y
        goal.pose.position.z = cmd.target_z

        goal.pose.orientation.z = math.sin(cmd.target_yaw / 2)
        goal.pose.orientation.w = math.cos(cmd.target_yaw / 2)

        if self._use_nav2_action and self._nav2_action_client is not None:
            self._send_nav2_goal(goal, cmd.command_type)
        else:
            self._pub_goal.publish(goal)
        self._current_action_cmd = cmd

    def _set_state(self, new_state: PlannerState, reason: str = ""):
        """更新状态。"""
        old_state = self._state
        self._state = new_state
        # Fix #5: Record navigation start time for per-goal timeout
        # Always reset when entering NAVIGATING (including re-entry in follow-mode)
        if new_state == PlannerState.NAVIGATING:
            self._navigation_start_time = time.time()
        if new_state == PlannerState.FAILED and reason:
            self._failure_reason = reason
        elif new_state not in (PlannerState.FAILED, PlannerState.CANCELLED):
            self._failure_reason = ""
        if old_state != new_state:
            self.get_logger().info(f"State: {old_state.value} → {new_state.value}")

    def _publish_status(self):
        """发布当前状态 (含任务计划进度)。"""
        # 统一 task_status: 外部消费者只需关注 running/completed/failed/idle
        _RUNNING_STATES = {
            PlannerState.DECOMPOSING, PlannerState.RESOLVING,
            PlannerState.NAVIGATING, PlannerState.LOOKING_AROUND,
            PlannerState.APPROACHING, PlannerState.VERIFYING,
            PlannerState.EXPLORING, PlannerState.BACKTRACKING,
            PlannerState.REPLANNING,
        }
        if self._state in _RUNNING_STATES:
            task_status = "running"
        elif self._state == PlannerState.COMPLETED:
            task_status = "completed"
        elif self._state == PlannerState.FAILED:
            task_status = "failed"
        elif self._state == PlannerState.CANCELLED:
            task_status = "cancelled"
        else:
            task_status = "idle"

        # 子目标进度
        current_step = 0
        total_steps = 0
        if self._current_plan:
            total_steps = len(self._current_plan.subgoals)
            current_step = self._current_plan.current_step

        status = {
            "state": self._state.value,
            "task_status": task_status,
            "failure_reason": self._failure_reason,
            "instruction": self._current_instruction or "",
            "exploration_strategy": self._exploration_strategy,
            "target_label": (
                self._current_goal.target_label if self._current_goal else ""
            ),
            "confidence": (
                round(self._current_goal.confidence, 3) if self._current_goal else 0
            ),
            "is_exploring": self._state == PlannerState.EXPLORING,
            "explore_count": self._explore_count,
            "replan_count": self._replan_count,
            "current_step": current_step,
            "total_steps": total_steps,
            "elapsed_sec": round(time.time() - self._task_start_time, 1)
            if self._task_start_time > 0
            else 0,
            "plan": self._current_plan.to_dict() if self._current_plan else None,
            "topo_memory": {
                "nodes": len(self._topo_memory.nodes),
            },
            "frontier_count": len(getattr(self._frontier_scorer, "_frontiers", [])),
            "sgnav": {
                "credibility_count": len(self._sgnav_reasoner.target_credibility),
            },
            "fsm": {
                "mode": self._fsm_mode,
                "mission_state": self._fsm_mission_state,
                "search_state": self._fsm_search_state,
                "implicit_ready": self._implicit_fsm.is_ready,
            },
        }

        msg = String()
        msg.data = json.dumps(status, ensure_ascii=False)
        self._pub_status.publish(msg)

    # ================================================================
    #  语义数据持久化
    # ================================================================

    def _load_semantic_data(self, data_dir: str) -> None:
        """从目录加载持久化语义数据 (KG + 拓扑记忆 + 场景图缓存)。"""
        import os
        from .room_object_kg import RoomObjectKG

        kg_path = os.path.join(data_dir, "room_object_kg.json")
        topo_path = os.path.join(data_dir, "topo_memory.json")
        sg_cache_path = os.path.join(data_dir, "scene_graph_cache.json")

        # 场景图热启动缓存: 让 Goal Resolver 在感知节点发布第一帧前就有历史数据
        if os.path.exists(sg_cache_path):
            try:
                with open(sg_cache_path, 'r', encoding='utf-8') as f:
                    cached_sg = f.read()
                self._latest_scene_graph = cached_sg
                self._latest_scene_graph_parsed = safe_json_loads(cached_sg, default=None)
                if self._latest_scene_graph_parsed:
                    n_obj = len(self._latest_scene_graph_parsed.get("objects", []))
                    self.get_logger().info(
                        f"[SemanticMap] Scene graph cache loaded ({n_obj} objects), "
                        f"planner ready before first camera frame"
                    )
            except Exception as e:
                self.get_logger().warning(f"[SemanticMap] Failed to load scene graph cache: {e}")

        # 初始化 runtime KG (用于增量收集本次 session 的数据)
        self._runtime_kg = RoomObjectKG()
        if os.path.exists(kg_path):
            self._runtime_kg.load(kg_path)
        self._runtime_kg.start_new_session()

        # 注入 KG 到 GoalResolver 和 FrontierScorer (P1+P2: KG-backed exploration)
        self._resolver.set_room_object_kg(self._runtime_kg)
        self._frontier_scorer.set_room_object_kg(self._runtime_kg)

        # 加载房间-物体 KG → 更新 SemanticPriorEngine
        if os.path.exists(kg_path):
            loaded = self._resolver._semantic_prior_engine.load_learned_priors(kg_path)
            if loaded:
                self.get_logger().info(f"Loaded room-object KG from {kg_path}")
            else:
                self.get_logger().info(f"Room-object KG at {kg_path} empty or failed, using defaults")
        else:
            self.get_logger().info(f"No room-object KG at {kg_path}, using hand-coded priors")

        # 加载拓扑记忆
        if os.path.exists(topo_path):
            if self._topo_memory.load_from_file(topo_path):
                self.get_logger().info(f"Loaded topo memory from {topo_path}")
            else:
                self.get_logger().warning(f"Failed to load topo memory from {topo_path}")

    def _save_semantic_data(self, data_dir: str) -> None:
        """保存语义数据到目录 (shutdown 时调用)。"""
        import os
        os.makedirs(data_dir, exist_ok=True)

        # 保存 runtime KG (已在场景图回调中增量更新)
        try:
            kg = getattr(self, '_runtime_kg', None)
            if kg is not None:
                kg_path = os.path.join(data_dir, "room_object_kg.json")

                # 从拓扑记忆提取房间转换 → 邻接关系
                for from_rid, to_rid in self._topo_memory.get_room_transitions():
                    from_info = self._topo_memory._visited_rooms.get(from_rid, {})
                    to_info = self._topo_memory._visited_rooms.get(to_rid, {})
                    from_name = from_info.get("name", "")
                    to_name = to_info.get("name", "")
                    if from_name and to_name:
                        kg.observe_adjacency(from_name, to_name)

                kg.save(kg_path)
                self.get_logger().info(f"Room-object KG saved to {kg_path}")
        except Exception as e:
            self.get_logger().warning(f"Failed to save room-object KG: {e}")

        # 保存拓扑记忆
        try:
            topo_path = os.path.join(data_dir, "topo_memory.json")
            self._topo_memory.save_to_file(topo_path)
            self.get_logger().info(f"Topo memory saved to {topo_path}")
        except Exception as e:
            self.get_logger().warning(f"Failed to save topo memory: {e}")

        # 保存最新场景图 JSON 缓存 (供下次启动热加载)
        try:
            sg = getattr(self, "_latest_scene_graph", None)
            if sg:
                sg_cache_path = os.path.join(data_dir, "scene_graph_cache.json")
                with open(sg_cache_path, 'w', encoding='utf-8') as f:
                    f.write(sg)
                parsed = getattr(self, "_latest_scene_graph_parsed", None) or {}
                n_obj = len(parsed.get("objects", []))
                self.get_logger().info(f"[SemanticMap] Scene graph cache saved ({n_obj} objects)")
        except Exception as e:
            self.get_logger().warning(f"[SemanticMap] Failed to save scene graph cache: {e}")

    # ================================================================
    #  生命周期
    # ================================================================

    def destroy_node(self):
        """清理 + 保存语义数据。"""
        # Fix #13: Cancel pending async futures before stopping loop.
        # Note: concurrent.futures.Future.cancel() only prevents execution if not yet started.
        # Already-running coroutines will complete or timeout via asyncio.wait_for().
        # The 3s thread join timeout below ensures we don't hang on shutdown.
        with self._futures_lock:
            for future in list(self._pending_futures):
                future.cancel()
            self._pending_futures.clear()
            self._active_mission_future = None
        # 清理 look_around timers
        if hasattr(self, '_look_around_finish_timer') and self._look_around_finish_timer:
            self._look_around_finish_timer.cancel()
        if self._look_around_timer:
            self._look_around_timer.cancel()
        # 清理 BBox 导航定时器
        self._stop_bbox_nav()
        # 保存持久化数据
        if self._semantic_data_dir:
            try:
                self._save_semantic_data(self._semantic_data_dir)
            except Exception as e:
                self.get_logger().warning(f"Failed to save semantic data on shutdown: {e}")

        # 保存 Tag 记忆
        if hasattr(self, "_tag_store"):
            try:
                self._tag_store.save()
            except Exception as e:
                self.get_logger().warning(f"Failed to save tagged locations on shutdown: {e}")

        self._loop.call_soon_threadsafe(self._loop.stop)
        self._async_thread.join(timeout=3.0)
        super().destroy_node()


def main(args=None):
    import signal

    rclpy.init(args=args)
    node = SemanticPlannerNode()

    # 信号处理: systemd nav-semantic.service 用 KillSignal=SIGINT + KillMode=mixed
    # 确保 SIGTERM/SIGINT 都能触发优雅关闭, 避免 16s 后被 SIGKILL
    def _shutdown_handler(signum, frame):
        sig_name = signal.Signals(signum).name
        node.get_logger().info(f"Received {sig_name}, shutting down gracefully...")
        rclpy.shutdown()

    signal.signal(signal.SIGTERM, _shutdown_handler)
    signal.signal(signal.SIGINT, _shutdown_handler)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
