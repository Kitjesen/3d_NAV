"""Nav2 导航交互 Mixin — NavigateToPose action + 重感知 + 隐式 FSM。

从 planner_node.py 提取的方法:
  _send_nav2_goal, _nav2_goal_response_callback, _nav2_feedback_callback,
  _trigger_continuous_reperception, _do_reperception (内联),
  _build_voi_state, _trigger_voi_slow_reason, _do_slow_reason (内联),
  _nav2_result_callback, _apply_implicit_fsm_transition,
  _build_implicit_fsm_observation, _extract_detection_for_implicit_fsm,
  _handle_explore_result, _generate_sgnav_waypoint, _is_semantic_target,
  _arrival_reperception_then_complete, _sgnav_reperception_check,
  _generate_frontier_waypoint, _publish_exploration_goal
"""

import asyncio
import json
import math
import re
import time
import traceback
from typing import Optional

import numpy as np

from semantic_common import safe_json_loads

from geometry_msgs.msg import PoseStamped

from .goal_resolver import GoalResult
from .exploration_strategy import generate_frontier_goal, extract_frontier_scene_data
from .sgnav_reasoner import FrontierSelection
from .voi_scheduler import SchedulerState, SchedulerAction
from .implicit_fsm_policy import ImplicitFSMObservation
from .planner_state import PlannerState


class Nav2Mixin:
    """Nav2 导航交互方法。通过多继承混入 SemanticPlannerNode。"""

    def _send_nav2_goal(self, goal_pose: PoseStamped, target_label: str):
        """通过 Nav2 action 发送导航目标, 注册 feedback/result 回调。"""
        try:
            from nav2_msgs.action import NavigateToPose
            _HAS_NAV2 = True
        except ImportError:
            _HAS_NAV2 = False

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
                self.get_logger().debug(
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
                    # 转探索（不依赖 Nav2，直接状态机跳转）
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
        # 从场景图提取目标信念信息
        target_cred = 0.5
        target_exist = 0.6
        target_var = 1.0
        match_count = 0
        total_objects = 0

        if self._latest_scene_graph and self._current_goal:
            sg = safe_json_loads(self._latest_scene_graph, default={})
            try:
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
            except (KeyError, TypeError, ValueError) as e:
                self.get_logger().debug(f"VoI scheduler state parse failed: {e}")

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
            "VoI slow-reason triggered at %.1fm",
            self._nav_accumulated_dist,
        )
        self._voi_slow_count = getattr(self, "_voi_slow_count", 0) + 1

        async def _do_slow_reason():
            try:
                # Fix #4: Add timeout to VoI slow-reason resolve call
                result = await asyncio.wait_for(
                    self._resolver.resolve(
                        instruction=self._current_instruction,
                        scene_graph_json=self._latest_scene_graph,
                        robot_position=self._robot_position,
                        language=self._current_language,
                        clip_encoder=getattr(self, "_clip_encoder", None),
                    ),
                    timeout=60.0,
                )
                if result.is_valid and result.action == "navigate":
                    if result.confidence > self._current_goal.confidence + 0.1:
                        self.get_logger().info(
                            "VoI slow-reason found better target: '%s' "
                            "(conf=%.2f > %.2f)",
                            result.target_label,
                            result.confidence,
                            self._current_goal.confidence,
                        )
                        # 取消当前导航并切换到新目标
                        if self._nav2_goal_handle and self._nav2_goal_active:
                            try:
                                self._nav2_goal_handle.cancel_goal_async()
                            except (RuntimeError, AttributeError) as e:
                                self.get_logger().debug(f"cancel_goal_async failed: {e}")
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
        sg = safe_json_loads(self._latest_scene_graph, default={})

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
        """处理探索结果 (F7: Fast+Slow 都失败时有 fallback)。

        架构说明 (USS-Nav style):
          实时探索: FrontierScorer (15Hz 级别, 几何+语义先验+TSP)
          目标确认: SGNavReasoner._sgnav_reperception_check() (按需触发)
          SG-Nav 子图 H-CoT: 仅在 Slow Path 目标推理时调用, 不在探索循环里
        """
        if not self._explore_enabled or not self._current_explore_if_unknown:
            self.get_logger().info("Exploration disabled, failing")
            self._set_state(PlannerState.FAILED, reason="exploration_disabled")
            return

        # ── 探索终止判断: 步数 + 语义信号 ──
        # 除步数限制外, 增加 frontier 新颖性耗尽和房间覆盖度检查
        fail_reason = ""
        if self._explore_count >= self._max_explore_steps:
            fail_reason = "max_steps"
        else:
            # 所有 frontier 评分过低 → 环境已充分探索
            cached_frontiers = getattr(self._frontier_scorer, "_frontiers", [])
            if cached_frontiers and all(
                f.score < 0.15 for f in cached_frontiers
            ):
                fail_reason = "all_explored"

        if fail_reason:
            self.get_logger().info(
                "Exploration terminated: %s (steps=%d/%d)",
                fail_reason, self._explore_count, self._max_explore_steps,
            )
            # F7: 超限后不直接 FAIL, 尝试 BACKTRACK 回起点
            backtrack_pos = self._topo_memory.get_backtrack_position(
                steps_back=self._explore_count
            )
            if backtrack_pos is not None:
                self.get_logger().info(
                    "Exploration done (%s), backtracking to start",
                    fail_reason,
                )
                cmd = self._action_executor.generate_backtrack_command(backtrack_pos)
                self._publish_goal_from_command(cmd)
                self._set_state(PlannerState.BACKTRACKING)
            else:
                self._set_state(PlannerState.FAILED, reason=f"exploration_exhausted:{fail_reason}")
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

                # Fix #14: Promote to info — SG-Nav unavailable is operationally significant
                self.get_logger().info(
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

                # Fix #14: Promote to info — zero frontiers is operationally significant
                self.get_logger().info(
                    "Frontier exploration unavailable (no valid frontiers), fallback to LLM exploration"
                )

            # Fallback: LLM 建议探索方向
            # Fix #4: Add timeout to generate_exploration_waypoint call
            # Consider @async_timeout when state callback not needed
            try:
                explore_result = await asyncio.wait_for(
                    self._resolver.generate_exploration_waypoint(
                        instruction=self._current_instruction,
                        robot_position=self._robot_position,
                        step_distance=self._step_distance,
                        language=self._current_language,
                        scene_graph_json=self._latest_scene_graph,
                    ),
                    timeout=60.0,
                )
            except asyncio.TimeoutError:
                self.get_logger().error("LLM exploration waypoint timed out (60s)")
                self._set_state(PlannerState.FAILED, reason="exploration_timeout")
                return

            if explore_result.is_valid:
                self._current_goal = explore_result
                self._publish_exploration_goal(explore_result, source="llm")
            else:
                self._set_state(PlannerState.FAILED, reason="llm_exploration_invalid")

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

        scene_objects, scene_relations, scene_rooms = extract_frontier_scene_data(
            self._latest_scene_graph
        )
        scored_frontiers = self._frontier_scorer.score_frontiers(
            instruction=self._current_instruction or "",
            robot_position=robot_xy,
            visited_positions=self._topo_memory.visited_positions,
            scene_objects=scene_objects,
            scene_relations=scene_relations,
            scene_rooms=scene_rooms,
        )

        # 收集 L3MVN 前沿描述 + VLingMem 已探索摘要 + 情节记忆 (供 SG-Nav LLM prompt)
        frontier_descs = [f.description for f in scored_frontiers if getattr(f, "description", "")]
        explored = self._topo_memory.get_explored_summaries()
        ep_summary = self._episodic_memory.get_summary()
        if ep_summary:
            explored = [f"[情节记忆] {ep_summary}"] + explored

        llm_chat = self._resolver._call_with_fallback if self._sgnav_use_llm_reasoning else None
        selection: Optional[FrontierSelection] = await self._sgnav_reasoner.select_frontier(
            instruction=self._current_instruction or "",
            scene_graph_json=self._latest_scene_graph,
            robot_position=self._robot_position,
            frontiers=scored_frontiers,
            language=self._current_language,
            llm_chat=llm_chat,
            frontier_descriptions=frontier_descs or None,
            explored_summaries=explored or None,
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
                        "BA-HSG reselect at arrival: '%s' -> '%s'",
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
                # Fix #7: Add timeout to vision_grounding in re-perception check
                # Consider @async_timeout when state callback not needed
                vg = await asyncio.wait_for(
                    self._resolver.vision_grounding(
                        instruction=result.target_label,
                        scene_graph_json=self._latest_scene_graph,
                        image_base64=self._latest_image_base64,
                        language=self._current_language,
                    ),
                    timeout=30.0,
                )
                confirmed_visible = bool(
                    vg.get("target_visible", False)
                    and float(vg.get("confidence", 0.0)) > self._vision_verify_threshold
                )
            except asyncio.TimeoutError:
                self.get_logger().debug("SG-Nav re-perception vision check timed out (30s)")
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


