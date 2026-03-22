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
from .bbox_nav_mixin import BBoxNavMixin
from .operational_mixin import OperationalMixin
from .callbacks_mixin import CallbacksMixin
from .goal_mixin import ResolveMixin
from .state_mixin import StateMixin
from .init_mixin import PlannerInitMixin


class SemanticPlannerNode(
    Nav2Mixin,
    SubgoalMixin,
    BBoxNavMixin,
    OperationalMixin,
    ResolveMixin,
    CallbacksMixin,
    StateMixin,
    PlannerInitMixin,
    Node,
):
    """语义规划 ROS2 节点 (论文级完整版)。"""

    def __init__(self):
        super().__init__("semantic_planner_node")
        self._declare_parameters()
        self._read_parameters()
        self._init_state()
        self._init_components()
        self._create_publishers()
        self._create_subscriptions()

        primary_config = self._primary_llm_config
        fallback_config = self._fallback_llm_config
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
