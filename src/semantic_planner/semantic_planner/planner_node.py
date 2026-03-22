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
from .callbacks_mixin import CallbacksMixin
from .resolve_mixin import ResolveMixin
from .state_mixin import StateMixin


class SemanticPlannerNode(
    Nav2Mixin,
    SubgoalMixin,
    BBoxNavMixin,
    OperationalMixin,
    ResolveMixin,
    CallbacksMixin,
    StateMixin,
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
