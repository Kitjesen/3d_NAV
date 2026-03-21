"""运营指令 Mixin — 操作类意图路由 + 取消 + stuck 处理。

从 planner_node.py 提取的方法:
  _execute_operational, _publish_response,
  _follow_person_cmd_callback, _cancel_callback,
  _handle_geometric_stuck
"""

import json
import time

from std_msgs.msg import String, Empty, Float32
from geometry_msgs.msg import PoseStamped

from .task_decomposer import SubGoalAction
from .planner_state import PlannerState


class OperationalMixin:
    """运营指令方法。通过多继承混入 SemanticPlannerNode。"""

    def _execute_operational(self, plan):
        """Execute operational (non-navigation) subgoals by publishing to command topics."""
        for sg in plan.subgoals:
            action = sg.action
            params = sg.parameters

            if action == SubGoalAction.PATROL:
                route = params.get("route", "default")
                cmd = json.dumps({"action": "start", "name": route})
                msg = String()
                msg.data = cmd
                self._patrol_cmd_pub.publish(msg)
                self._publish_response(f"好的，启动巡逻路线「{route}」")
                self.get_logger().info(f"Operational: PATROL route={route}")

            elif action == SubGoalAction.SAVE_MAP:
                cmd = json.dumps({"action": "save"})
                msg = String()
                msg.data = cmd
                self._map_cmd_pub.publish(msg)
                self._publish_response("正在保存地图...")
                self.get_logger().info("Operational: SAVE_MAP")

            elif action == SubGoalAction.SAVE_POI:
                name = params.get("name", sg.target or "未命名")
                x = round(self._robot_position["x"], 2) if self._robot_position else 0.0
                y = round(self._robot_position["y"], 2) if self._robot_position else 0.0
                cmd = json.dumps({"action": "set", "name": name, "x": x, "y": y, "z": 0.0})
                msg = String()
                msg.data = cmd
                self._poi_cmd_pub.publish(msg)
                self._publish_response(f"已标记当前位置为「{name}」")
                self.get_logger().info(f"Operational: SAVE_POI name={name} pos=({x},{y})")

            elif action == SubGoalAction.SET_SPEED:
                speed = float(params.get("value", 1.0))
                msg = Float32()
                msg.data = speed
                self._speed_pub.publish(msg)
                self._publish_response(f"速度已调整为 {speed:.1f} m/s")
                self.get_logger().info(f"Operational: SET_SPEED value={speed}")

            elif action == SubGoalAction.RETURN_HOME:
                # Publish goal to origin or saved home position
                goal = PoseStamped()
                goal.header.frame_id = "map"
                goal.header.stamp = self.get_clock().now().to_msg()
                goal.pose.position.x = 0.0
                goal.pose.position.y = 0.0
                goal.pose.position.z = 0.0
                goal.pose.orientation.w = 1.0
                self._pub_goal.publish(goal)
                self._publish_response("好的，正在返航")
                self.get_logger().info("Operational: RETURN_HOME -> goal(0,0)")

            elif action == SubGoalAction.PAUSE:
                msg = Empty()
                self._cancel_pub.publish(msg)
                self._publish_response("已暂停当前任务")
                self.get_logger().info("Operational: PAUSE")

            elif action == SubGoalAction.RESUME:
                self._publish_response("恢复暂不支持，请重新下达指令")
                self.get_logger().info("Operational: RESUME (not yet supported)")

            elif action == SubGoalAction.STOP:
                msg = Empty()
                self._cancel_pub.publish(msg)
                self._publish_response("已停止")
                self.get_logger().info("Operational: STOP -> cancel")

        # Update state to completed
        self._set_state(PlannerState.COMPLETED)
        self._publish_status()

    def _publish_response(self, text: str):
        """Publish voice/text response for the user."""
        msg = String()
        msg.data = json.dumps({
            "response": text,
            "timestamp": time.time(),
            "instruction": self._current_instruction or "",
        }, ensure_ascii=False)
        self._response_pub.publish(msg)
        self.get_logger().info(f"Response: {text}")

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
        self._pub_cmd_vel.publish(self._make_twist_stamped())

        # 停止旋转定时器
        if self._look_around_timer is not None:
            self._look_around_timer.cancel()
            self._look_around_timer = None

        # FOLLOW_PERSON: 发布空目标停止跟随节点
        if self._follow_mode:
            _stop_msg = String()
            _stop_msg.data = "{}"
            self._follow_person_pub.publish(_stop_msg)
            self._follow_mode = False
            self.get_logger().info("Follow person mode cancelled")

        # 重置状态
        self._follow_mode = False
        self._action_executor.reset()
        self._set_state(PlannerState.CANCELLED)

    def _follow_person_cmd_callback(self, msg: String):
        """
        接收来自 task_manager 的人物跟随指令 (TASK_TYPE_FOLLOW_PERSON = 7)。

        消息格式 (JSON):
          {"target_label": "person", "follow_distance": 1.5,
           "timeout_sec": 300.0, "min_distance": 0.8, "max_distance": 5.0}

        空消息 ({} 或空字符串) 表示停止跟随。
        """
        raw = msg.data.strip() if msg.data else ""

        # 空消息 → 停止跟随
        if not raw or raw in ("{}", "null"):
            if self._follow_mode:
                self.get_logger().info("Follow person: stop command received")
                self._follow_mode = False
                self._pub_cmd_vel.publish(self._make_twist_stamped())
                self._set_state(PlannerState.IDLE)
            return

        try:
            params = json.loads(raw)
        except json.JSONDecodeError as e:
            self.get_logger().warn(f"Follow person cmd JSON parse error: {e}")
            return

        target_label = params.get("target_label", "person") or "person"
        follow_distance = float(params.get("follow_distance", 1.5) or 1.5)
        timeout_sec = float(params.get("timeout_sec", 300.0) or 300.0)
        min_distance = float(params.get("min_distance", 0.8) or 0.8)
        max_distance = float(params.get("max_distance", 5.0) or 5.0)

        self.get_logger().info(
            "Follow person: target='%s', follow_dist=%.1fm, "
            "min=%.1fm, max=%.1fm, timeout=%.0fs",
            target_label, follow_distance, min_distance, max_distance, timeout_sec,
        )

        # 配置 PersonTracker 跟随距离
        self._person_tracker.follow_distance = follow_distance

        # 设置跟随模式内部状态
        self._follow_mode = True
        self._follow_target_label = target_label
        self._follow_timeout = timeout_sec
        self._follow_start_time = time.time()
        self._task_start_time = time.time()
        self._instruction_timeout = timeout_sec + 10.0  # 宽松超时

        # 转发参数到 /nav/semantic/follow_person (供下游跟随控制器使用)
        out_msg = String()
        out_msg.data = json.dumps({
            "target_label": target_label,
            "follow_distance": follow_distance,
            "min_distance": min_distance,
            "max_distance": max_distance,
            "timeout_sec": timeout_sec,
        }, ensure_ascii=False)
        self._follow_person_pub.publish(out_msg)

        # 通过现有 VLN 闭环启动跟随:
        # 构建单步 FOLLOW 任务计划, 使用 _execute_next_subgoal 进入 RESOLVING 状态
        self._current_instruction = f"follow {target_label}"
        self._current_language = "zh"
        self._current_explore_if_unknown = False
        self._replan_count = 0
        self._explore_count = 0
        self._execution_context = ""
        self._resolver.reset_exploration()
        self._action_executor.reset()

        from .task_decomposer import TaskPlan, SubGoal, SubGoalAction
        follow_plan = TaskPlan(
            instruction=self._current_instruction,
            subgoals=[
                SubGoal(
                    step_id=0,
                    action=SubGoalAction.FOLLOW,
                    target=target_label,
                    parameters={"timeout": timeout_sec},
                ),
            ],
        )
        self._current_plan = follow_plan
        self._execute_next_subgoal()

    def _handle_geometric_stuck(self, data: dict):
        """几何层 stuck_final → 调用 _subgoal_failed 触发 LERa 重规划。"""
        # 仅在 NAVIGATING / EXPLORING 等执行态时响应
        executing_states = {
            PlannerState.NAVIGATING,
            PlannerState.EXPLORING,
            PlannerState.APPROACHING,
        }
        if self._state not in executing_states:
            # BUG FIX: 非执行态下节流日志，每 5s 打印一次而非每 100ms
            now = time.time()
            if now - getattr(self, '_last_stuck_ignore_log', 0) > 5.0:
                self.get_logger().info(
                    f"[Geometric Stuck] 忽略 stuck_final, 当前状态: {self._state.value}"
                )
                self._last_stuck_ignore_log = now
            return

        # BUG FIX: 进入 NAVIGATING 后给 local_planner 至少 3s 的 grace period
        if self._navigation_start_time > 0:
            elapsed = time.time() - self._navigation_start_time
            if elapsed < 3.0:
                self.get_logger().debug(
                    f"[Geometric Stuck] Grace period: {elapsed:.1f}s < 3.0s, 忽略"
                )
                return
        reason = (
            f"Path geometrically blocked at waypoint "
            f"{data.get('index', '?')}/{data.get('total', '?')}"
        )
        self._subgoal_failed(reason)


