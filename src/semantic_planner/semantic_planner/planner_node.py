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
  - cmd_vel       (geometry_msgs/Twist, 相对话题名)
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

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import Image
from std_msgs.msg import String

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
from .action_executor import ActionExecutor, ActionCommand
from .frontier_scorer import FrontierScorer
from .exploration_strategy import generate_frontier_goal, extract_frontier_scene_data
from .sgnav_reasoner import SGNavReasoner, FrontierSelection
from .voi_scheduler import VoIScheduler, VoIConfig, SchedulerState, SchedulerAction
from .implicit_fsm_policy import (
    ImplicitFSMPolicy,
    ImplicitFSMObservation,
)


class PlannerState(Enum):
    """规划器状态。"""
    IDLE = "idle"
    DECOMPOSING = "decomposing"
    RESOLVING = "resolving"
    NAVIGATING = "navigating"
    LOOKING_AROUND = "looking_around"
    APPROACHING = "approaching"
    VERIFYING = "verifying"
    EXPLORING = "exploring"
    BACKTRACKING = "backtracking"
    REPLANNING = "replanning"
    COMPLETED = "completed"
    FAILED = "failed"
    CANCELLED = "cancelled"  # F1: 任务取消状态


class SemanticPlannerNode(Node):
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

        # LOVON 风格隐式 FSM: s_{t+1} = f_theta(obs_t, s_t, instruction)
        self.declare_parameter("fsm.mode", "implicit")            # explicit | implicit
        self.declare_parameter("fsm.implicit.weights_path", "")
        self.declare_parameter("fsm.implicit.strict", False)

        # B5: Nav2 NavigateToPose action 参数
        self.declare_parameter("nav2.use_action_client", True)
        self.declare_parameter("nav2.action_name", "navigate_to_pose")
        self.declare_parameter("nav2.action_timeout_sec", 120.0)

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

        # ── 读取参数 ──
        primary_config = LLMConfig(
            backend=self.get_parameter("llm.backend").value,
            model=self.get_parameter("llm.model").value,
            api_key_env=self.get_parameter("llm.api_key_env").value,
            timeout_sec=self.get_parameter("llm.timeout_sec").value,
            max_retries=self.get_parameter("llm.max_retries").value,
            temperature=self.get_parameter("llm.temperature").value,
        )

        fallback_config = LLMConfig(
            backend=self.get_parameter("llm_fallback.backend").value,
            model=self.get_parameter("llm_fallback.model").value,
            api_key_env=self.get_parameter("llm_fallback.api_key_env").value,
            timeout_sec=self.get_parameter("llm_fallback.timeout_sec").value,
            max_retries=self.get_parameter("llm_fallback.max_retries").value,
        )

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

        # ── 目标解析器 ──
        self._resolver = GoalResolver(
            primary_config=primary_config,
            fallback_config=fallback_config,
            confidence_threshold=self._confidence_threshold,
            fast_path_threshold=self.get_parameter(
                "goal_resolution.fast_path_threshold"
            ).value,
            max_replan_attempts=self._max_replan_attempts,
        )

        # ── 子模块 ──
        # KG 注入到 TaskDecomposer (安全门 + 可供性验证)
        try:
            from semantic_perception.knowledge_graph import IndustrialKnowledgeGraph
            _kg = IndustrialKnowledgeGraph()
            TaskDecomposer.set_knowledge_graph(_kg)
            self.get_logger().info("KG injected into TaskDecomposer (%d concepts)", len(_kg.get_all_concepts()))
        except Exception as e:
            self.get_logger().warning("KG injection into TaskDecomposer failed (non-critical): %s", e)
        self._decomposer = TaskDecomposer()
        self._topo_memory = TopologicalMemory(
            new_node_distance=self.get_parameter("topo_memory.new_node_distance").value,
            max_nodes=self.get_parameter("topo_memory.max_nodes").value,
        )
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
        )
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

        # 跟随模式标记 (FOLLOW 动作通过现有导航闭环实现)
        self._follow_mode: bool = False
        self._follow_target_label: str = ""
        self._follow_timeout: float = 300.0
        self._follow_start_time: float = 0.0

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
        self._current_goal: Optional[GoalResult] = None
        self._current_plan: Optional[TaskPlan] = None
        self._current_action_cmd: Optional[ActionCommand] = None
        self._replan_count: int = 0
        self._explore_count: int = 0
        self._task_start_time: float = 0.0
        self._execution_context: str = ""  # GAP: 闭环反馈 — 成功时累计场景变化

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

        # 异步事件循环 (在单独线程中运行 LLM 调用)
        self._loop = asyncio.new_event_loop()
        self._async_thread = threading.Thread(
            target=self._run_async_loop, daemon=True
        )
        self._async_thread.start()

        # ── 订阅 ──
        self._sub_instruction = self.create_subscription(
            String, "instruction", self._instruction_callback, 10
        )
        self._sub_scene_graph = self.create_subscription(
            String, "scene_graph", self._scene_graph_callback, 10
        )
        self._sub_odom = self.create_subscription(
            Odometry, "odometry", self._odom_callback, 10
        )

        self._sub_costmap = None
        if self._exploration_strategy in ("frontier", "sg_nav"):
            costmap_topic = self.get_parameter("exploration.costmap_topic").value
            self._sub_costmap = self.create_subscription(
                OccupancyGrid, costmap_topic, self._costmap_callback, 1
            )
            self.get_logger().info(f"Frontier exploration enabled, costmap={costmap_topic}")

        # F1: 任务取消话题
        self._sub_cancel = self.create_subscription(
            String, "/nav/semantic/cancel", self._cancel_callback, 10
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
        self._pub_cmd_vel = self.create_publisher(Twist, "cmd_vel", 10)
        self._pub_status = self.create_publisher(String, "status", 10)
        self._look_around_timer = None

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

    # ================================================================
    #  异步事件循环
    # ================================================================

    def _run_async_loop(self):
        """在单独线程中运行异步事件循环。"""
        asyncio.set_event_loop(self._loop)
        self._loop.run_forever()

    def _schedule_async(self, coro):
        """在异步线程中调度协程。"""
        asyncio.run_coroutine_threadsafe(coro, self._loop)

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
        except (json.JSONDecodeError, AttributeError):
            instruction = msg.data
            language = "zh"
            explore = True
            timeout = self._instruction_timeout
            radius = self._arrival_radius

        if not instruction:
            self.get_logger().warn("Empty instruction received, ignoring")
            return

        self.get_logger().info(
            f"New semantic instruction: '{instruction}' "
            f"(lang={language}, explore={explore})"
        )

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
        self._sgnav_reasoner.reset()
        self._fsm_mission_state = "searching_1"
        self._fsm_search_state = "had_searching_1"
        self._action_executor.reset()
        self._current_plan = None

        # Step 1: 任务分解 (SayCan)
        plan = self._decomposer.decompose_with_rules(instruction)
        if plan:
            self.get_logger().info(
                f"Rule-based decomposition: {len(plan.subgoals)} subgoals"
            )
            self._current_plan = plan
            self._execute_next_subgoal()
        else:
            self.get_logger().info("Complex instruction, using LLM decomposition")
            self._set_state(PlannerState.DECOMPOSING)
            self._schedule_async(self._llm_decompose())

    def _cancel_callback(self, msg: String):
        """
        F1: 任务取消。

        接收 /nav/semantic/cancel 消息, 停止当前任务。
        """
        reason = msg.data if msg.data else "User requested cancel"
        self.get_logger().info(f"Task cancelled: {reason}")

        # B5: 取消 Nav2 action goal
        if self._nav2_goal_handle is not None and self._nav2_goal_active:
            try:
                self._nav2_goal_handle.cancel_goal_async()
                self.get_logger().info("Nav2 goal cancel requested")
            except Exception as e:
                self.get_logger().debug(f"Nav2 cancel failed: {e}")
            self._nav2_goal_active = False

        # 停止运动
        stop = Twist()
        self._pub_cmd_vel.publish(stop)

        # 停止旋转定时器
        if self._look_around_timer is not None:
            self._look_around_timer.cancel()
            self._look_around_timer = None

        # 重置状态
        self._action_executor.reset()
        self._set_state(PlannerState.CANCELLED)

    def _scene_graph_callback(self, msg: String):
        """更新最新场景图。"""
        self._latest_scene_graph = msg.data

    def _image_callback(self, msg: Image):
        """缓存最新相机帧 (用于 VLM vision grounding)。"""
        try:
            import cv2
            import base64
            from cv_bridge import CvBridge

            bridge = CvBridge()
            bgr = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

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
        """更新机器人位置。"""
        p = msg.pose.pose.position
        self._robot_position = {
            "x": p.x, "y": p.y, "z": p.z,
        }

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

    # ================================================================
    #  任务分解 (SayCan, 异步)
    # ================================================================

    async def _llm_decompose(self):
        """LLM 分解复杂指令 (SayCan / Inner Monologue)。"""
        try:
            # 获取场景摘要
            scene_summary = ""
            try:
                sg = json.loads(self._latest_scene_graph)
                scene_summary = sg.get("summary", "")
            except (json.JSONDecodeError, TypeError) as e:
                # A3 修复: 记录日志
                self.get_logger().debug(f"Scene graph parse failed: {e}")
            if self._execution_context:
                scene_summary = (scene_summary or "") + "\n\n[Execution history]\n" + self._execution_context

            messages = self._decomposer.build_decomposition_prompt(
                self._current_instruction, scene_summary, self._current_language
            )
            response = await self._resolver._call_with_fallback(messages)

            if response:
                plan = self._decomposer.parse_decomposition_response(
                    self._current_instruction, response
                )
            else:
                # LLM 失败, fallback: 简单 NAVIGATE + VERIFY
                plan = TaskPlan(
                    instruction=self._current_instruction,
                    subgoals=[
                        SubGoal(step_id=0, action=SubGoalAction.NAVIGATE,
                                target=self._current_instruction),
                        SubGoal(step_id=1, action=SubGoalAction.VERIFY,
                                target=self._current_instruction),
                    ],
                )

            self.get_logger().info(
                f"LLM decomposition: {len(plan.subgoals)} subgoals: "
                + " → ".join(f"{sg.action.value}({sg.target})" for sg in plan.subgoals)
            )
            self._current_plan = plan
            self._execute_next_subgoal()

        except Exception as e:
            self.get_logger().error(
                f"Decomposition failed: {e}\n{traceback.format_exc()}"
            )
            self._set_state(PlannerState.FAILED)

    # ================================================================
    #  子目标执行引擎 (LOVON 动作原语)
    # ================================================================

    def _execute_next_subgoal(self):
        """推进到下一个子目标并执行。"""
        # F1: 检查是否已取消
        if self._state == PlannerState.CANCELLED:
            return

        if self._current_plan is None:
            self._set_state(PlannerState.FAILED)
            return

        if self._current_plan.is_complete:
            self.get_logger().info("All subgoals completed!")
            self._set_state(PlannerState.COMPLETED)
            return

        if self._current_plan.is_failed:
            self.get_logger().error("Task plan failed (subgoal exhausted retries)")
            self._set_state(PlannerState.FAILED)
            return

        subgoal = self._current_plan.active_subgoal
        if subgoal is None:
            self._set_state(PlannerState.COMPLETED)
            return

        subgoal.status = SubGoalStatus.ACTIVE
        self.get_logger().info(
            f"Executing subgoal [{subgoal.step_id}]: "
            f"{subgoal.action.value}({subgoal.target})"
        )

        # 根据动作类型分发
        if subgoal.action == SubGoalAction.NAVIGATE:
            self._set_state(PlannerState.RESOLVING)
            self._schedule_async(self._resolve_goal())

        elif subgoal.action == SubGoalAction.FIND:
            self._set_state(PlannerState.RESOLVING)
            self._schedule_async(self._resolve_goal())

        elif subgoal.action == SubGoalAction.APPROACH:
            self._execute_approach(subgoal)

        elif subgoal.action == SubGoalAction.VERIFY:
            self._execute_verify(subgoal)

        elif subgoal.action == SubGoalAction.LOOK_AROUND:
            self._execute_look_around(subgoal)

        elif subgoal.action == SubGoalAction.EXPLORE:
            self._set_state(PlannerState.EXPLORING)
            self._schedule_async(self._resolve_goal())

        elif subgoal.action == SubGoalAction.BACKTRACK:
            self._execute_backtrack(subgoal)

        elif subgoal.action == SubGoalAction.STOP:
            self.get_logger().info("⏹ STOP action — cancelling current task")
            self._cancel_navigation()
            self._set_state(PlannerState.IDLE)
            self._advance_subgoal()

        elif subgoal.action == SubGoalAction.PICK:
            self.get_logger().info(
                "🤏 PICK action: target='%s' — resolving position then approaching",
                subgoal.target,
            )
            self._set_state(PlannerState.RESOLVING)
            self._schedule_async(self._resolve_goal())

        elif subgoal.action == SubGoalAction.PLACE:
            self.get_logger().info(
                "📦 PLACE action: target='%s' — navigating to drop location",
                subgoal.target,
            )
            self._set_state(PlannerState.RESOLVING)
            self._schedule_async(self._resolve_goal())

        elif subgoal.action == SubGoalAction.STATUS:
            self.get_logger().info("📊 STATUS query — reporting system state")
            self._publish_status_report(subgoal.target or "all")
            self._advance_subgoal()

        elif subgoal.action == SubGoalAction.FOLLOW:
            # FOLLOW 复用完整导航闭环: resolve → navigate → 到达 → re-resolve → navigate ...
            # 和 NAVIGATE 走同一条路, 区别仅在于 _follow_mode 标记
            self._follow_mode = True
            self._follow_target_label = subgoal.target or "person"
            self._follow_timeout = (
                subgoal.parameters.get("timeout", 300.0) if subgoal.parameters else 300.0
            )
            self._follow_start_time = time.time()
            self.get_logger().info(
                "🏃 Follow mode: target='%s', timeout=%.0fs — "
                "using existing VLN loop (perceive→resolve→navigate→re-perceive)",
                self._follow_target_label, self._follow_timeout,
            )
            self._set_state(PlannerState.RESOLVING)
            self._schedule_async(self._resolve_goal())

        elif subgoal.action == SubGoalAction.WAIT:
            wait_sec = subgoal.parameters.get("wait_sec", 3.0)
            self.get_logger().info(f"Waiting {wait_sec}s...")
            self.create_timer(
                wait_sec,
                lambda: self._subgoal_completed(),
            )

    def _execute_approach(self, subgoal: SubGoal):
        """执行 APPROACH 子目标 (LOVON)。"""
        if not self._robot_position or not self._current_goal:
            self._subgoal_failed("No position or goal for APPROACH")
            return

        target_pos = {
            "x": self._current_goal.target_x,
            "y": self._current_goal.target_y,
            "z": self._current_goal.target_z,
        }

        cmd = self._action_executor.generate_approach_command(
            target_pos, self._robot_position
        )
        self._publish_goal_from_command(cmd)
        self._set_state(PlannerState.APPROACHING)

    def _execute_verify(self, subgoal: SubGoal):
        """
        执行 VERIFY 子目标 (LOVON + VLMnav)。

        A4 修复: 异常时不再默认 success, 而是标记 FAILED 触发重试。
        """
        self._set_state(PlannerState.VERIFYING)

        if self._vision_enabled and self._latest_image_base64:
            self._schedule_async(self._vision_verify(subgoal))
        else:
            # 无 VLM, 基于置信度判断
            if (self._current_goal
                    and self._current_goal.confidence > self._confidence_threshold):
                self.get_logger().info("VERIFY passed (confidence-based, no VLM)")
                self._subgoal_completed()
            else:
                self.get_logger().warn(
                    "VERIFY uncertain (no VLM, low confidence) — passing with warning"
                )
                # 低置信度 + 无 VLM: 仍然通过但记录 warning
                self._subgoal_completed()

    async def _vision_verify(self, subgoal: SubGoal):
        """VLM 视觉验证 (VLMnav 方案)。"""
        try:
            result = await self._resolver.vision_grounding(
                instruction=subgoal.target,
                scene_graph_json=self._latest_scene_graph,
                image_base64=self._latest_image_base64,
                language=self._current_language,
            )

            visible = result.get("target_visible", False)
            confidence = result.get("confidence", 0.0)
            reasoning = result.get("reasoning", "")

            self.get_logger().info(
                f"VERIFY result: visible={visible}, "
                f"confidence={confidence:.2f}, reason={reasoning}"
            )

            if visible and confidence > self._vision_verify_threshold:
                self._subgoal_completed()
            else:
                self.get_logger().warn(
                    f"VERIFY failed: target not confirmed ({reasoning})"
                )
                self._subgoal_failed(f"VLM verification failed: {reasoning}")

        except Exception as e:
            # A4 修复: 异常时不再默认 success
            self.get_logger().error(
                f"Vision verify error: {e}\n{traceback.format_exc()}"
            )
            self._subgoal_failed(f"VLM verify exception: {e}")

    def _publish_status_report(self, query: str):
        """Publish robot status (battery, pose, task, mode) to feedback topic."""
        report = {
            "query": query,
            "state": self._state.value if self._state else "unknown",
            "active_task": str(self._current_plan.instruction) if self._current_plan else None,
            "follow_mode": getattr(self, "_follow_mode", False),
        }
        self.get_logger().info("Status report: %s", report)

    def _execute_look_around(self, subgoal: SubGoal):
        """执行 LOOK_AROUND (LOVON: 原地 360° 扫描)。"""
        cmd = self._action_executor.generate_look_around_command()
        self._current_action_cmd = cmd
        self._set_state(PlannerState.LOOKING_AROUND)

        # 持续发布旋转速度 (C9 参数化)
        look_hz = self.get_parameter("execution.look_around_hz").value
        self._look_around_timer = self.create_timer(
            1.0 / max(look_hz, 1.0), self._publish_look_around_twist
        )

        # 旋转完成后停止并推进子目标
        self.create_timer(
            cmd.timeout_sec,
            lambda: self._finish_look_around(),
        )

    def _publish_look_around_twist(self):
        """发布 LOOK_AROUND 旋转 Twist。"""
        if self._state != PlannerState.LOOKING_AROUND:
            return
        twist = Twist()
        twist.angular.z = (
            self._current_action_cmd.angular_z if self._current_action_cmd else 0.0
        )
        self._pub_cmd_vel.publish(twist)

    def _finish_look_around(self):
        """停止旋转, 完成 LOOK_AROUND 子目标。"""
        stop = Twist()
        self._pub_cmd_vel.publish(stop)

        if self._look_around_timer is not None:
            self._look_around_timer.cancel()
            self._look_around_timer = None

        self._subgoal_completed()

    def _execute_backtrack(self, subgoal: SubGoal):
        """执行 BACKTRACK (LOVON: 回溯到上一位置)。"""
        backtrack_pos = self._topo_memory.get_backtrack_position(steps_back=2)
        if backtrack_pos is not None:
            cmd = self._action_executor.generate_backtrack_command(backtrack_pos)
            self._publish_goal_from_command(cmd)
            self._set_state(PlannerState.BACKTRACKING)
        else:
            self.get_logger().warn("BACKTRACK: no history, skipping")
            self._subgoal_completed()

    def _subgoal_completed(self):
        """当前子目标完成, 推进到下一个。GAP: 闭环反馈 — 成功时累积执行上下文。"""
        if self._current_plan:
            active = self._current_plan.active_subgoal

            # ── 跟随模式: 到达 ≠ 结束, 重新解析目标继续导航 ──
            if (self._follow_mode and active
                    and active.action in (SubGoalAction.NAVIGATE, SubGoalAction.FOLLOW)):
                elapsed = time.time() - self._follow_start_time
                if elapsed > self._follow_timeout:
                    self.get_logger().info(
                        "Follow mode timeout (%.0fs), completing.", elapsed,
                    )
                    self._follow_mode = False
                else:
                    self.get_logger().info(
                        "🏃 Follow mode: arrived near target, re-resolving "
                        "(%.0fs / %.0fs)", elapsed, self._follow_timeout,
                    )
                    # 不 advance, 不 complete — 重新解析同一目标
                    self._set_state(PlannerState.RESOLVING)
                    self._schedule_async(self._resolve_goal())
                    return

            if active:
                self.get_logger().info(
                    f"Subgoal [{active.step_id}] {active.action.value} completed"
                )
            if self._current_goal and active:
                summary = self._build_success_summary(active)
                if summary:
                    self._execution_context = (
                        (self._execution_context + "\n" + summary)
                        if self._execution_context
                        else summary
                    )
                    self.get_logger().debug(f"Execution context: {summary[:80]}...")
            self._current_plan.advance()
            self._action_executor.reset()
            # 退出跟随模式 (FOLLOW subgoal 已 advance)
            if self._follow_mode:
                self._follow_mode = False
        self._execute_next_subgoal()

    def _build_success_summary(self, subgoal) -> str:
        """GAP: 构建子目标成功时的富语义反馈 (Inner Monologue 风格)。"""
        target = getattr(subgoal, "target", "") or ""
        if not target:
            return ""
        try:
            sg = json.loads(self._latest_scene_graph)
            objs = sg.get("objects", [])
            if isinstance(objs, list) and objs:
                labels = [str(o.get("label", "")) for o in objs[:6] if o]
                visible = ", ".join(l for l in labels if l)
                if visible:
                    return f"Reached {target}. Visible: {visible}"
        except (json.JSONDecodeError, TypeError, KeyError):
            pass
        return f"Reached {target}."

    def _subgoal_failed(self, reason: str = ""):
        """当前子目标失败, 尝试重试或放弃。"""
        failed_subgoal: Optional[SubGoal] = None
        if self._current_plan:
            active = self._current_plan.active_subgoal

            # ── 跟随模式: 失败不放弃, 继续搜索 ──
            if (self._follow_mode and active
                    and active.action in (SubGoalAction.NAVIGATE, SubGoalAction.FOLLOW)):
                elapsed = time.time() - self._follow_start_time
                if elapsed < self._follow_timeout:
                    self.get_logger().warn(
                        "🏃 Follow mode: target lost (%s), "
                        "re-resolving (%.0fs / %.0fs)",
                        reason, elapsed, self._follow_timeout,
                    )
                    active.retry_count = 0  # 跟随模式重置重试计数
                    self._set_state(PlannerState.RESOLVING)
                    self._schedule_async(self._resolve_goal())
                    return
                else:
                    self.get_logger().info(
                        "Follow mode timeout (%.0fs), stopping.", elapsed,
                    )
                    self._follow_mode = False

            if active:
                failed_subgoal = active
                self.get_logger().warn(
                    f"Subgoal [{active.step_id}] {active.action.value} failed: {reason}"
                )
            self._current_plan.fail_current()
            self._action_executor.reset()

            # 重试耗尽后触发反馈重规划 (Inner Monologue 风格)
            exhausted = (
                failed_subgoal is not None
                and failed_subgoal.status == SubGoalStatus.FAILED
            )
            can_replan = (
                exhausted
                and self._replan_on_failure
                and self._replan_count < self._max_replan_attempts
            )
            if can_replan:
                self.get_logger().warn(
                    f"Subgoal retries exhausted, triggering replanning "
                    f"({self._replan_count + 1}/{self._max_replan_attempts})"
                )
                self._set_state(PlannerState.REPLANNING)
                self._schedule_async(self._llm_replan(reason, failed_subgoal))
                return

        self._execute_next_subgoal()

    async def _llm_replan(self, reason: str, failed_subgoal: Optional[SubGoal]):
        """基于执行反馈的 LLM 重规划。"""
        try:
            self._replan_count += 1

            # 场景摘要 + 失败反馈
            scene_summary = ""
            try:
                sg = json.loads(self._latest_scene_graph)
                scene_summary = sg.get("summary", "")
            except (json.JSONDecodeError, TypeError):
                scene_summary = ""

            failed_desc = "unknown"
            if failed_subgoal is not None:
                failed_desc = (
                    f"step={failed_subgoal.step_id}, "
                    f"action={failed_subgoal.action.value}, "
                    f"target={failed_subgoal.target}, "
                    f"retry={failed_subgoal.retry_count}/{failed_subgoal.max_retries}"
                )

            ctx = f"[Execution history]\n{self._execution_context}\n\n" if self._execution_context else ""
            feedback_summary = (
                f"{scene_summary}\n"
                f"{ctx}"
                f"[Execution Feedback]\n"
                f"Failed subgoal: {failed_desc}\n"
                f"Failure reason: {reason or 'unknown'}\n"
                "Please replan remaining subgoals from current scene and avoid repeating failed strategy."
            )

            messages = self._decomposer.build_decomposition_prompt(
                self._current_instruction or "",
                feedback_summary,
                self._current_language,
            )

            response = await self._resolver._call_with_fallback(messages)
            if not response:
                self.get_logger().error("Replan failed: no LLM response")
                self._set_state(PlannerState.FAILED)
                return

            new_plan = self._decomposer.parse_decomposition_response(
                self._current_instruction or "", response
            )
            if not new_plan.subgoals:
                self.get_logger().error("Replan failed: empty subgoal list")
                self._set_state(PlannerState.FAILED)
                return

            self.get_logger().info(
                f"Replan #{self._replan_count}: {len(new_plan.subgoals)} subgoals: "
                + " -> ".join(f"{sg.action.value}({sg.target})" for sg in new_plan.subgoals)
            )
            self._current_plan = new_plan
            self._execute_next_subgoal()

        except Exception as e:
            self.get_logger().error(
                f"LLM replan exception: {e}\n{traceback.format_exc()}"
            )
            self._set_state(PlannerState.FAILED)

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
            result = await self._resolver.resolve(
                instruction=self._current_instruction,
                scene_graph_json=self._latest_scene_graph,
                robot_position=self._robot_position,
                language=self._current_language,
                explore_if_unknown=self._current_explore_if_unknown,
            )

            if not result.is_valid:
                self.get_logger().error(f"Goal resolution failed: {result.error}")
                self._subgoal_failed(result.error)
                return

            # LOVON 风格隐式 FSM: 用参数化模型决定状态转移
            if self._fsm_mode == "implicit":
                result = self._apply_implicit_fsm_transition(result)

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
                    vg_result = await self._resolver.vision_grounding(
                        instruction=self._current_instruction,
                        scene_graph_json=self._latest_scene_graph,
                        image_base64=self._latest_image_base64,
                        language=self._current_language,
                    )
                    if vg_result.get("target_visible", False):
                        result.confidence = max(
                            result.confidence, vg_result.get("confidence", 0.5)
                        )
                        self.get_logger().info(
                            f"Vision grounding: target visible, "
                            f"confidence→{result.confidence:.2f}"
                        )
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

                self._handle_navigate_result(result)
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

    def _handle_navigate_result(self, result: GoalResult):
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

        # 构建 PoseStamped
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
    #  B5: Nav2 NavigateToPose action 交互
    # ================================================================

    def _send_nav2_goal(self, goal_pose: PoseStamped, target_label: str):
        """通过 Nav2 action 发送导航目标, 注册 feedback/result 回调。"""
        if not _HAS_NAV2 or self._nav2_action_client is None:
            self._pub_goal.publish(goal_pose)
            return

        # 取消前一个进行中的目标
        if self._nav2_goal_handle is not None and self._nav2_goal_active:
            self.get_logger().info("Cancelling previous Nav2 goal")
            try:
                self._nav2_goal_handle.cancel_goal_async()
            except Exception as e:
                self.get_logger().debug(f"Cancel previous goal failed: {e}")
            self._nav2_goal_active = False

        # 等待 action server (非阻塞, 1秒超时)
        if not self._nav2_action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("Nav2 action server not available, falling back to topic")
            self._pub_goal.publish(goal_pose)
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        # 创新3: 重置连续 Re-perception 状态
        self._nav_accumulated_dist = 0.0
        self._last_reperception_dist = 0.0
        self._last_fb_position = None

        self.get_logger().info(f"Sending Nav2 goal: {target_label}")

        send_future = self._nav2_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._nav2_feedback_callback,
        )
        send_future.add_done_callback(self._nav2_goal_response_callback)

        # 同时发布到 topic (其他节点可能也在监听)
        self._pub_goal.publish(goal_pose)

    def _nav2_goal_response_callback(self, future):
        """Nav2 goal accepted/rejected 回调。"""
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f"Nav2 goal send failed: {e}")
            self._nav2_goal_active = False
            return

        if not goal_handle.accepted:
            self.get_logger().warn("Nav2 goal was rejected")
            self._nav2_goal_active = False
            self._subgoal_failed("Nav2 goal rejected")
            return

        self.get_logger().info("Nav2 goal accepted")
        self._nav2_goal_handle = goal_handle
        self._nav2_goal_active = True

        # 注册结果回调
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._nav2_result_callback)

    def _nav2_feedback_callback(self, feedback_msg):
        """
        Nav2 导航 feedback 回调 (B5 核心: 实时进度)。

        feedback 包含:
          - current_pose: 当前位置
          - navigation_time: 已用时间
          - estimated_time_remaining: 预估剩余时间
          - number_of_recoveries: 恢复次数
          - distance_remaining: 剩余距离
        
        创新3 补强: 每移动 reperception_interval_m (默认 2m) 触发一次
        场景图刷新 + credibility 更新, 实现导航中的连续 Re-perception。
        """
        try:
            fb = feedback_msg.feedback
            dist_remaining = fb.distance_remaining
            nav_time = fb.navigation_time.sec + fb.navigation_time.nanosec * 1e-9
            n_recoveries = fb.number_of_recoveries

            # 从 feedback 提取当前位置
            current_fb_pos = {
                "x": fb.current_pose.pose.position.x,
                "y": fb.current_pose.pose.position.y,
                "z": fb.current_pose.pose.position.z,
            }

            # 累计行驶距离
            if self._last_fb_position is not None:
                dx = current_fb_pos["x"] - self._last_fb_position["x"]
                dy = current_fb_pos["y"] - self._last_fb_position["y"]
                self._nav_accumulated_dist += math.sqrt(dx * dx + dy * dy)
            self._last_fb_position = current_fb_pos

            # 日志 (低频: 仅当距离变化 > 0.5m 时)
            if not hasattr(self, "_last_fb_dist") or abs(self._last_fb_dist - dist_remaining) > 0.5:
                self._last_fb_dist = dist_remaining
                self.get_logger().info(
                    f"Nav2 feedback: dist_remaining={dist_remaining:.2f}m, "
                    f"time={nav_time:.1f}s, recoveries={n_recoveries}, "
                    f"accumulated={self._nav_accumulated_dist:.1f}m"
                )

            # ── 创新3+4: VoI 驱动的连续调度 (BA-HSG §3.4.4) ──
            if (self._continuous_reperception
                    and self._state == PlannerState.NAVIGATING
                    and self._current_goal
                    and self._is_semantic_target(self._current_goal.target_label)):

                if self._voi_enabled:
                    # VoI 调度: 信息价值驱动决策
                    voi_state = self._build_voi_state(dist_remaining)
                    voi_action = self._voi_scheduler.decide(voi_state)

                    if voi_action == SchedulerAction.REPERCEIVE:
                        self._last_reperception_dist = self._nav_accumulated_dist
                        self._last_voi_reperception_time = time.time()
                        self._trigger_continuous_reperception()
                    elif voi_action == SchedulerAction.SLOW_REASON:
                        self._last_voi_slow_time = time.time()
                        self._trigger_voi_slow_reason()
                else:
                    # 回退: 固定 2m 触发
                    dist_since_last = self._nav_accumulated_dist - self._last_reperception_dist
                    if dist_since_last >= self._reperception_interval_m:
                        self._last_reperception_dist = self._nav_accumulated_dist
                        self._trigger_continuous_reperception()

            # 检测异常: 恢复次数过多 → 可能卡住
            if n_recoveries > 3:
                self.get_logger().warn(
                    f"Nav2 too many recoveries ({n_recoveries}), may be stuck"
                )

            # 超时检测 (Nav2 自身也有, 这里做双重保险)
            if nav_time > self._nav2_action_timeout:
                self.get_logger().warn(
                    f"Nav2 navigation timeout ({nav_time:.0f}s > {self._nav2_action_timeout:.0f}s)"
                )
                if self._nav2_goal_handle is not None:
                    self._nav2_goal_handle.cancel_goal_async()

        except Exception as e:
            self.get_logger().debug(f"Nav2 feedback parse error: {e}")

    def _trigger_continuous_reperception(self):
        """
        创新3: 导航中连续 Re-perception。
        
        触发条件: 导航中每移动 reperception_interval_m 米。
        行为: 
          1. 用当前场景图刷新目标 credibility
          2. 如果 credibility 过低, 取消当前导航并转探索
          3. 如果发现更近/更好的匹配, 更新导航目标
        """
        if not self._current_goal:
            return

        self.get_logger().info(
            f"Continuous re-perception triggered at "
            f"{self._nav_accumulated_dist:.1f}m"
        )

        async def _do_reperception():
            try:
                accepted = await self._sgnav_reperception_check(
                    self._current_goal, force_vision_on_arrival=False
                )
                if not accepted:
                    self.get_logger().warn(
                        "Continuous re-perception rejected target, "
                        "cancelling navigation"
                    )
                    # 取消当前 Nav2 goal
                    if self._nav2_goal_handle is not None and self._nav2_goal_active:
                        try:
                            self._nav2_goal_handle.cancel_goal_async()
                        except Exception:
                            pass
                    # 转探索
                    self._subgoal_failed(
                        "Continuous re-perception: target no longer credible"
                    )
                else:
                    self.get_logger().debug(
                        "Continuous re-perception: target still credible"
                    )
            except Exception as e:
                self.get_logger().debug(
                    f"Continuous re-perception error (non-fatal): {e}"
                )

        self._schedule_async(_do_reperception())

    def _build_voi_state(self, distance_remaining: float) -> SchedulerState:
        """构建 VoI 调度器输入状态 (BA-HSG §3.4.4)。"""
        import json as _json

        # 从场景图提取目标信念信息
        target_cred = 0.5
        target_exist = 0.6
        target_var = 1.0
        match_count = 0
        total_objects = 0

        if self._latest_scene_graph and self._current_goal:
            try:
                sg = _json.loads(self._latest_scene_graph)
                objects = sg.get("objects", [])
                total_objects = len(objects)
                target_label = (self._current_goal.target_label or "").lower()
                for obj in objects:
                    label = str(obj.get("label", "")).lower()
                    if target_label and (target_label in label or label in target_label):
                        match_count += 1
                        belief = obj.get("belief", {})
                        if isinstance(belief, dict):
                            target_cred = max(target_cred, belief.get("credibility", 0.5))
                            target_exist = max(target_exist, belief.get("P_exist", 0.6))
                            sigma = belief.get("sigma_pos", 1.0)
                            target_var = min(target_var, sigma * sigma)
            except Exception:
                pass

        return SchedulerState(
            target_credibility=target_cred,
            target_existence_prob=target_exist,
            target_position_var=target_var,
            match_count=match_count,
            total_objects=total_objects,
            distance_to_goal=distance_remaining,
            nav_accumulated_dist=self._nav_accumulated_dist,
            distance_since_last_reperception=(
                self._nav_accumulated_dist - self._last_reperception_dist
            ),
            slow_reason_count=getattr(self, "_voi_slow_count", 0),
            reperception_count=getattr(self, "_voi_rep_count", 0),
            time_elapsed=time.time() - getattr(self, "_episode_start_time", time.time()),
            last_reperception_time=self._last_voi_reperception_time,
            last_slow_reason_time=self._last_voi_slow_time,
        )

    def _trigger_voi_slow_reason(self) -> None:
        """VoI 调度: 触发慢推理重判定 (BA-HSG §3.4.4)。"""
        if not self._current_goal or not self._current_instruction:
            return

        self.get_logger().info(
            "🧠 VoI slow-reason triggered at %.1fm",
            self._nav_accumulated_dist,
        )
        self._voi_slow_count = getattr(self, "_voi_slow_count", 0) + 1

        async def _do_slow_reason():
            try:
                result = await self._resolver.resolve(
                    instruction=self._current_instruction,
                    scene_graph_json=self._latest_scene_graph,
                    robot_position=self._robot_position,
                    language=self._current_language,
                    clip_encoder=getattr(self, "_clip_encoder", None),
                )
                if result.is_valid and result.action == "navigate":
                    if result.confidence > self._current_goal.confidence + 0.1:
                        self.get_logger().info(
                            "🧠 VoI slow-reason found better target: '%s' "
                            "(conf=%.2f > %.2f)",
                            result.target_label,
                            result.confidence,
                            self._current_goal.confidence,
                        )
                        # 取消当前导航并切换到新目标
                        if self._nav2_goal_handle and self._nav2_goal_active:
                            try:
                                self._nav2_goal_handle.cancel_goal_async()
                            except Exception:
                                pass
                        self._current_goal = result
                        # 重新发送 Nav2 目标
                        from geometry_msgs.msg import PoseStamped
                        goal_pose = PoseStamped()
                        goal_pose.header.stamp = self.get_clock().now().to_msg()
                        goal_pose.header.frame_id = result.frame_id  # 使用目标解析器的坐标系
                        goal_pose.pose.position.x = result.target_x
                        goal_pose.pose.position.y = result.target_y
                        goal_pose.pose.position.z = result.target_z
                        goal_pose.pose.orientation.w = 1.0
                        self._send_nav2_goal(goal_pose, result.target_label)
                    else:
                        self.get_logger().debug(
                            "VoI slow-reason: no improvement "
                            "(new=%.2f vs current=%.2f)",
                            result.confidence, self._current_goal.confidence,
                        )
            except Exception as e:
                self.get_logger().debug(f"VoI slow-reason error: {e}")

        self._schedule_async(_do_slow_reason())

    def _nav2_result_callback(self, future):
        """
        Nav2 导航完成/失败结果回调 (B5 核心)。

        根据结果码触发子目标完成/失败:
          - STATUS_SUCCEEDED (4): 到达目标 → _subgoal_completed()
          - STATUS_ABORTED (6):   导航失败 → _subgoal_failed()
          - STATUS_CANCELED (5):  被取消 → 不触发 (可能是我们自己取消的)
        """
        self._nav2_goal_active = False

        try:
            result = future.result()
            status = result.status
        except Exception as e:
            self.get_logger().error(f"Nav2 result error: {e}")
            self._subgoal_failed(f"Nav2 result exception: {e}")
            return

        # rclpy action status codes
        STATUS_SUCCEEDED = 4
        STATUS_CANCELED = 5
        STATUS_ABORTED = 6

        if status == STATUS_SUCCEEDED:
            self.get_logger().info("Nav2: Navigation succeeded")
            # 已到达 — GAP: 到达后 Re-perception (近距离重检测验证, SG-Nav 核心)
            if self._state == PlannerState.NAVIGATING:
                if (self._current_goal
                        and self.get_parameter("exploration.sgnav.arrival_reperception").value
                        and self._is_semantic_target(self._current_goal.target_label)):
                    self._schedule_async(self._arrival_reperception_then_complete())
                    return
                self._subgoal_completed()
            elif self._state == PlannerState.EXPLORING:
                self.get_logger().info("Nav2: Exploration point reached, re-resolving...")
                self._subgoal_completed()
            elif self._state == PlannerState.APPROACHING:
                self.get_logger().info("Nav2: Approach complete")
                self._subgoal_completed()
            elif self._state == PlannerState.BACKTRACKING:
                self.get_logger().info("Nav2: Backtrack complete")
                self._subgoal_completed()
        elif status == STATUS_ABORTED:
            self.get_logger().warn("Nav2: Navigation aborted (failed to reach goal)")
            self._subgoal_failed("Nav2 navigation aborted")
        elif status == STATUS_CANCELED:
            self.get_logger().info("Nav2: Navigation canceled")
            # 不触发 subgoal failure, 可能是我们主动取消
        else:
            self.get_logger().warn(f"Nav2: Unexpected status {status}")
            self._subgoal_failed(f"Nav2 unexpected status: {status}")

    def _apply_implicit_fsm_transition(self, result: GoalResult) -> GoalResult:
        """使用隐式 FSM 预测 mission/search 状态并覆盖动作决策。"""
        obs = self._build_implicit_fsm_observation(result)
        pred = self._implicit_fsm.predict(obs)
        if pred is None:
            return result

        self._fsm_mission_state = pred.mission_state_out
        self._fsm_search_state = pred.search_state_out

        confidence_state = float(np.max(pred.state_prob))
        reason = (
            f"implicit_fsm: mission={pred.mission_state_out}, "
            f"search={pred.search_state_out}, p={confidence_state:.2f}, "
            f"motion={pred.motion_vector.round(2).tolist()}"
        )
        if result.reasoning:
            result.reasoning = f"{result.reasoning} | {reason}"
        else:
            result.reasoning = reason

        # LOVON 4-state 映射到当前 planner 动作
        if pred.mission_state_out in ("searching_1", "searching_0"):
            result.action = "explore"
            result.path = f"{result.path}+implicit" if result.path else "implicit"
            result.is_valid = True

            # 生成一个短程探索航点, 后续仍会走 SG-Nav/frontier/LLM 探索链
            if self._robot_position is not None:
                step = max(0.5, float(self._step_distance))
                direction = 1.0 if pred.mission_state_out == "searching_1" else -1.0
                vx = float(np.clip(pred.motion_vector[0], -1.0, 1.0))
                wz = float(np.clip(pred.motion_vector[2], -1.0, 1.0))

                result.target_x = self._robot_position["x"] + step * (0.6 * vx)
                result.target_y = self._robot_position["y"] + step * (0.6 * direction + 0.4 * wz)
                result.target_z = self._robot_position.get("z", 0.0)
                result.target_label = f"implicit_fsm:{pred.mission_state_out}"

            result.confidence = max(float(result.confidence), confidence_state)

        elif pred.mission_state_out == "success":
            result.action = "navigate"
            result.confidence = max(float(result.confidence), 0.95)
            result.path = f"{result.path}+implicit" if result.path else "implicit"

        elif pred.mission_state_out == "running":
            if result.action not in ("navigate", "explore"):
                result.action = "navigate"
            result.path = f"{result.path}+implicit" if result.path else "implicit"

        return result

    def _build_implicit_fsm_observation(self, result: GoalResult) -> ImplicitFSMObservation:
        """构造隐式 FSM 输入特征。"""
        predicted_object, conf, object_xyn, object_whn = self._extract_detection_for_implicit_fsm(
            result.target_label or self._current_instruction or ""
        )

        return ImplicitFSMObservation(
            mission_instruction_0=self._fsm_prev_instruction or (self._current_instruction or ""),
            mission_instruction_1=self._current_instruction or "",
            mission_object_1=result.target_label or (self._current_instruction or ""),
            predicted_object=predicted_object,
            confidence=max(float(result.confidence), conf),
            object_xyn=np.asarray(object_xyn, dtype=np.float64),
            object_whn=np.asarray(object_whn, dtype=np.float64),
            mission_state_in=self._fsm_mission_state,
            search_state_in=self._fsm_search_state,
        )

    def _extract_detection_for_implicit_fsm(self, target_text: str):
        """从最新场景图提取隐式 FSM 所需检测特征。"""
        try:
            sg = json.loads(self._latest_scene_graph)
        except (json.JSONDecodeError, TypeError):
            return "NULL", 0.0, np.array([0.5, 0.5]), np.array([0.0, 0.0])

        objects = sg.get("objects", [])
        if not isinstance(objects, list) or not objects:
            return "NULL", 0.0, np.array([0.5, 0.5]), np.array([0.0, 0.0])

        tokens = [
            t for t in re.findall(r"[A-Za-z0-9_]+|[\u4e00-\u9fff]{1,4}", target_text.lower())
            if len(t) > 1
        ]

        best_obj = None
        best_score = -1.0
        for obj in objects:
            if not isinstance(obj, dict):
                continue
            label = str(obj.get("label", "")).lower()
            lexical = 0.0
            for t in tokens:
                if t in label or label in t:
                    lexical = max(lexical, 1.0)

            det_score = 0.0
            try:
                det_score = float(obj.get("score", 0.0))
            except (TypeError, ValueError):
                pass

            score = 0.7 * lexical + 0.3 * det_score
            if score > best_score:
                best_score = score
                best_obj = obj

        if best_obj is None or best_score < 0.25:
            return "NULL", 0.0, np.array([0.5, 0.5]), np.array([0.0, 0.0])

        label = str(best_obj.get("label", "NULL"))
        try:
            conf = float(best_obj.get("score", 0.0))
        except (TypeError, ValueError):
            conf = 0.0

        # scene_graph 中通常无归一化 bbox, 用相对方位/距离估计 xyn 和尺寸 proxy
        cxn, cyn = 0.5, 0.5
        size_proxy = min(1.0, max(0.0, conf))
        if self._robot_position is not None:
            pos = best_obj.get("position", {})
            if isinstance(pos, dict) and "x" in pos and "y" in pos:
                try:
                    dx = float(pos.get("x", 0.0)) - self._robot_position["x"]
                    dy = float(pos.get("y", 0.0)) - self._robot_position["y"]
                    dist = math.sqrt(dx * dx + dy * dy)
                    angle = math.atan2(dy, dx)
                    cxn = 0.5 + 0.5 * math.sin(angle)
                    cyn = 0.5 - 0.5 * math.cos(angle)
                    size_proxy = min(1.0, 1.0 / (1.0 + max(dist, 0.0)))
                except (TypeError, ValueError):
                    pass

        whn = np.array([size_proxy, size_proxy], dtype=np.float64)
        xyn = np.array([float(np.clip(cxn, 0.0, 1.0)), float(np.clip(cyn, 0.0, 1.0))])
        return label, float(np.clip(conf, 0.0, 1.0)), xyn, whn

    async def _handle_explore_result(self, result: GoalResult):
        """处理探索结果 (F7: Fast+Slow 都失败时有 fallback)。"""
        if not self._explore_enabled or not self._current_explore_if_unknown:
            self.get_logger().info("Exploration disabled, failing")
            self._set_state(PlannerState.FAILED)
            return

        if self._explore_count >= self._max_explore_steps:
            self.get_logger().info("Max exploration steps reached")
            # F7: 超限后不直接 FAIL, 尝试 BACKTRACK 回起点
            backtrack_pos = self._topo_memory.get_backtrack_position(
                steps_back=self._explore_count
            )
            if backtrack_pos is not None:
                self.get_logger().info("Max explore reached, backtracking to start")
                cmd = self._action_executor.generate_backtrack_command(backtrack_pos)
                self._publish_goal_from_command(cmd)
                self._set_state(PlannerState.BACKTRACKING)
            else:
                self._set_state(PlannerState.FAILED)
            return

        self._explore_count += 1

        if self._robot_position:
            # SG-Nav: 子图推理 + frontier 概率插值
            if self._exploration_strategy == "sg_nav":
                sgnav_result = await self._generate_sgnav_waypoint()
                if sgnav_result is not None and sgnav_result.is_valid:
                    self._current_goal = sgnav_result
                    self._publish_exploration_goal(sgnav_result, source="sg_nav")
                    return

                self.get_logger().debug(
                    "SG-Nav exploration unavailable, fallback to frontier/llm"
                )

                # SG-Nav fallback 1: 纯 frontier
                frontier_result = self._generate_frontier_waypoint()
                if frontier_result is not None and frontier_result.is_valid:
                    self._current_goal = frontier_result
                    self._publish_exploration_goal(frontier_result, source="frontier_fallback")
                    return

            # 优先使用 Frontier 评分探索 (MTU3D 风格闭环)
            if self._exploration_strategy == "frontier":
                frontier_result = self._generate_frontier_waypoint()
                if frontier_result is not None and frontier_result.is_valid:
                    self._current_goal = frontier_result
                    self._publish_exploration_goal(frontier_result, source="frontier")
                    return

                self.get_logger().debug(
                    "Frontier exploration unavailable, fallback to LLM exploration"
                )

            # Fallback: LLM 建议探索方向
            explore_result = await self._resolver.generate_exploration_waypoint(
                instruction=self._current_instruction,
                robot_position=self._robot_position,
                step_distance=self._step_distance,
                language=self._current_language,
            )

            if explore_result.is_valid:
                self._current_goal = explore_result
                self._publish_exploration_goal(explore_result, source="llm")
            else:
                self._set_state(PlannerState.FAILED)

    async def _generate_sgnav_waypoint(self) -> Optional[GoalResult]:
        """用 SG-Nav 子图推理 + frontier 插值生成探索航点。"""
        if self._robot_position is None:
            return None

        robot_xy = np.array([
            self._robot_position["x"],
            self._robot_position["y"],
        ], dtype=np.float64)

        frontiers = self._frontier_scorer.extract_frontiers(robot_xy)
        if not frontiers:
            return None

        scene_objects, scene_relations = extract_frontier_scene_data(self._latest_scene_graph)
        scored_frontiers = self._frontier_scorer.score_frontiers(
            instruction=self._current_instruction or "",
            robot_position=robot_xy,
            visited_positions=self._topo_memory.visited_positions,
            scene_objects=scene_objects,
            scene_relations=scene_relations,
        )

        llm_chat = self._resolver._call_with_fallback if self._sgnav_use_llm_reasoning else None
        selection: Optional[FrontierSelection] = await self._sgnav_reasoner.select_frontier(
            instruction=self._current_instruction or "",
            scene_graph_json=self._latest_scene_graph,
            robot_position=self._robot_position,
            frontiers=scored_frontiers,
            language=self._current_language,
            llm_chat=llm_chat,
        )

        if selection is None:
            return None
        if selection.score < self._frontier_score_threshold:
            return None

        best = selection.frontier
        return GoalResult(
            action="explore",
            target_x=float(best.center_world[0]),
            target_y=float(best.center_world[1]),
            target_z=float(self._robot_position.get("z", 0.0)),
            target_label=f"sgnav_frontier:{best.direction_label}",
            confidence=float(selection.score),
            reasoning=selection.reasoning,
            is_valid=True,
            path="sg_nav",
        )

    def _is_semantic_target(self, target_label: Optional[str]) -> bool:
        """是否为语义目标 (非探索点)，需做到达后 Re-perception。"""
        if not target_label or not isinstance(target_label, str):
            return False
        t = target_label.strip().lower()
        if t.startswith("sgnav_frontier:") or t.startswith("implicit_fsm:"):
            return False
        return True

    async def _arrival_reperception_then_complete(self):
        """
        到达后 Re-perception + BA-HSG 贝叶斯验证 (§3.4.3)。
        
        流程:
          1. 传统 re-perception 检查 (视觉+可信度)
          2. BA-HSG: 贝叶斯更新 → 如果目标被拒绝, 自动重选下一候选
          3. 如果多假设管理器有重选结果 → 导航到新候选
          4. 否则走传统的探索回退
        """
        if not self._current_goal:
            self._subgoal_completed()
            return
        accepted = await self._sgnav_reperception_check(
            self._current_goal, force_vision_on_arrival=True
        )
        if accepted:
            self._subgoal_completed()
        else:
            # BA-HSG: 多假设贝叶斯重选
            candidate_id = getattr(self._current_goal, "candidate_id", -1)
            if candidate_id >= 0 and hasattr(self._resolver, "verify_and_reselect"):
                robot_pos = None
                if self._robot_position:
                    robot_pos = [
                        self._robot_position.get("x", 0),
                        self._robot_position.get("y", 0),
                    ]
                new_goal = self._resolver.verify_and_reselect(
                    object_id=candidate_id,
                    detected=False,
                    clip_sim=0.3,
                    robot_position=robot_pos,
                )
                if new_goal and new_goal.is_valid:
                    self.get_logger().info(
                        "🔄 BA-HSG reselect at arrival: '%s' → '%s'",
                        self._current_goal.target_label,
                        new_goal.target_label,
                    )
                    self._current_goal = new_goal
                    from geometry_msgs.msg import PoseStamped
                    goal_pose = PoseStamped()
                    goal_pose.header.stamp = self.get_clock().now().to_msg()
                    goal_pose.header.frame_id = new_goal.frame_id  # 使用新目标的坐标系
                    goal_pose.pose.position.x = new_goal.target_x
                    goal_pose.pose.position.y = new_goal.target_y
                    goal_pose.pose.position.z = new_goal.target_z
                    goal_pose.pose.orientation.w = 1.0
                    self._send_nav2_goal(goal_pose, new_goal.target_label)
                    return

            self.get_logger().warn(
                "Arrival re-perception rejected target, switching to exploration"
            )
            await self._handle_explore_result(
                GoalResult(
                    action="explore",
                    confidence=0.1,
                    reasoning="Arrival re-perception: target not confirmed at close range",
                    is_valid=True,
                    path="arrival_reperception",
                )
            )

    async def _sgnav_reperception_check(
        self, result: GoalResult, force_vision_on_arrival: bool = False
    ) -> bool:
        """
        SG-Nav graph-based re-perception:
        当目标可信度过低时拒绝该目标并继续探索。
        force_vision_on_arrival: 到达后强制做视觉验证 (近距离重检测)。
        """
        if not result.target_label:
            return True

        run_vision = (
            self._vision_enabled
            and self._latest_image_base64
            and (
                force_vision_on_arrival
                or result.confidence < self._confidence_threshold
            )
        )
        confirmed_visible = False
        if run_vision:
            try:
                vg = await self._resolver.vision_grounding(
                    instruction=result.target_label,
                    scene_graph_json=self._latest_scene_graph,
                    image_base64=self._latest_image_base64,
                    language=self._current_language,
                )
                confirmed_visible = bool(
                    vg.get("target_visible", False)
                    and float(vg.get("confidence", 0.0)) > self._vision_verify_threshold
                )
            except Exception as e:
                self.get_logger().debug(f"SG-Nav re-perception vision check failed: {e}")

        reject, cred, reason = self._sgnav_reasoner.evaluate_target_credibility(
            target_label=result.target_label,
            scene_graph_json=self._latest_scene_graph,
            path_confidence=result.confidence,
            confirmed_visible=confirmed_visible,
        )

        if reject:
            self.get_logger().warn(
                f"SG-Nav re-perception reject '{result.target_label}': {reason}"
            )
            return False

        self.get_logger().debug(
            f"SG-Nav re-perception accept '{result.target_label}': cred={cred:.2f}; {reason}"
        )
        return True

    def _generate_frontier_waypoint(self) -> Optional[GoalResult]:
        """用 Frontier 评分器生成探索航点。"""
        if self._robot_position is None:
            return None
        return generate_frontier_goal(
            frontier_scorer=self._frontier_scorer,
            instruction=self._current_instruction or "",
            robot_position=self._robot_position,
            visited_positions=self._topo_memory.visited_positions,
            scene_graph_json=self._latest_scene_graph,
            score_threshold=self._frontier_score_threshold,
        )

    def _publish_exploration_goal(self, explore_result: GoalResult, source: str):
        """发布探索航点 (B5: 优先 Nav2 action)。"""
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = explore_result.frame_id  # 使用探索结果的坐标系
        goal.pose.position.x = explore_result.target_x
        goal.pose.position.y = explore_result.target_y
        goal.pose.position.z = explore_result.target_z
        goal.pose.orientation.w = 1.0

        if self._use_nav2_action and self._nav2_action_client is not None:
            self._send_nav2_goal(goal, f"explore:{source}")
        else:
            self._pub_goal.publish(goal)
        self._set_state(PlannerState.EXPLORING)

        self.get_logger().info(
            f"Exploration waypoint ({source}) #{self._explore_count}: "
            f"({explore_result.target_x:.2f}, {explore_result.target_y:.2f}) "
            f"conf={explore_result.confidence:.2f}, reason: {explore_result.reasoning}"
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

        # ── 拓扑记忆更新 (L3MVN) ──
        if self._robot_position:
            visible_labels = []
            try:
                sg = json.loads(self._latest_scene_graph)
                visible_labels = [
                    obj["label"] for obj in sg.get("objects", [])[:10]
                ]
            except (json.JSONDecodeError, TypeError, KeyError) as e:
                self.get_logger().debug(f"Scene graph parse in monitor: {e}")

            self._topo_memory.update_position(
                position=np.array([
                    self._robot_position["x"],
                    self._robot_position["y"],
                    self._robot_position.get("z", 0.0),
                ]),
                visible_labels=visible_labels,
                # D7: 结构化截断 — 只保留附近物体的标签
                scene_snapshot=json.dumps(
                    {"nearby_labels": visible_labels[:5]}, ensure_ascii=False
                ),
            )

        # ── 超时检测 ──
        if self._task_start_time > 0:
            elapsed = time.time() - self._task_start_time
            if elapsed > self._instruction_timeout:
                self.get_logger().warn(
                    f"Semantic nav timeout ({elapsed:.0f}s > "
                    f"{self._instruction_timeout:.0f}s)"
                )
                self._set_state(PlannerState.FAILED)
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
                    if self._state == PlannerState.NAVIGATING:
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

    def _set_state(self, new_state: PlannerState):
        """更新状态。"""
        old_state = self._state
        self._state = new_state
        if old_state != new_state:
            self.get_logger().info(f"State: {old_state.value} → {new_state.value}")

    def _publish_status(self):
        """发布当前状态 (含任务计划进度)。"""
        status = {
            "state": self._state.value,
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
    #  生命周期
    # ================================================================

    def destroy_node(self):
        """清理。"""
        self._loop.call_soon_threadsafe(self._loop.stop)
        self._async_thread.join(timeout=3.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SemanticPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
