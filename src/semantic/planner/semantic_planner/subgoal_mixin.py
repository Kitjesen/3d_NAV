"""子目标执行 Mixin — LLM 分解 + LOVON 动作原语 + LERa 恢复。

从 planner_node.py 提取的方法:
  _llm_decompose, _execute_next_subgoal, _on_wait_complete,
  _execute_approach, _execute_verify, _vision_verify,
  _execute_look_around, _publish_look_around_twist, _finish_look_around,
  _execute_backtrack, _subgoal_completed, _build_success_summary,
  _subgoal_failed, _llm_replan
"""

import asyncio
import json
import time
import traceback
from typing import Optional

import numpy as np

from semantic_common import safe_json_loads

from .action_executor import ActionCommand
from .task_decomposer import (
    TaskPlan, SubGoal,
    SubGoalAction, SubGoalStatus,
)
from .planner_state import PlannerState


class SubgoalMixin:
    """子目标执行方法。通过多继承混入 SemanticPlannerNode。"""

    async def _llm_decompose(self):
        """LLM 分解复杂指令 (SayCan / Inner Monologue)。"""
        try:
            # 获取场景摘要
            sg = safe_json_loads(self._latest_scene_graph, default={})
            scene_summary = sg.get("summary", "")
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

        except asyncio.CancelledError:
            # 新指令到达时取消了旧的 decompose — 正常行为，不设 FAILED
            self.get_logger().info("LLM decomposition cancelled (new instruction arrived)")
        except Exception as e:
            self.get_logger().error(
                f"Decomposition failed: {e}\n{traceback.format_exc()}"
            )
            self._set_state(PlannerState.FAILED, reason="decomposition_failed")

    def _execute_next_subgoal(self):
        """推进到下一个子目标并执行。"""
        # F1: 检查是否已取消
        if self._state == PlannerState.CANCELLED:
            return

        if self._current_plan is None:
            self._set_state(PlannerState.FAILED, reason="no_plan")
            return

        if self._current_plan.is_complete:
            self.get_logger().info("All subgoals completed!")
            self._set_state(PlannerState.COMPLETED)
            return

        if self._current_plan.is_failed:
            self.get_logger().error("Task plan failed (subgoal exhausted retries)")
            self._set_state(PlannerState.FAILED, reason="subgoal_retries_exhausted")
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
            self.get_logger().info("STOP action — stopping robot and advancing")
            self._pub_cmd_vel.publish(self._make_twist_stamped())
            self._set_state(PlannerState.IDLE)
            self._subgoal_completed()

        elif subgoal.action == SubGoalAction.PICK:
            self.get_logger().info(
                "PICK action: target='%s' — resolving position then approaching",
                subgoal.target,
            )
            self._set_state(PlannerState.RESOLVING)
            self._schedule_async(self._resolve_goal())

        elif subgoal.action == SubGoalAction.PLACE:
            self.get_logger().info(
                "PLACE action: target='%s' — navigating to drop location",
                subgoal.target,
            )
            self._set_state(PlannerState.RESOLVING)
            self._schedule_async(self._resolve_goal())

        elif subgoal.action == SubGoalAction.STATUS:
            self.get_logger().info("STATUS query — reporting system state")
            self._publish_status_report(subgoal.target or "all")
            self._subgoal_completed()

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
                "Follow mode: target='%s', timeout=%.0fs — "
                "using existing VLN loop (perceive->resolve->navigate->re-perceive)",
                self._follow_target_label, self._follow_timeout,
            )
            self._set_state(PlannerState.RESOLVING)
            self._schedule_async(self._resolve_goal())

        elif subgoal.action == SubGoalAction.WAIT:
            wait_sec = subgoal.parameters.get("wait_sec", 3.0)
            self.get_logger().info(f"Waiting {wait_sec}s...")
            # Fix #1: Store timer handle and cancel in callback to prevent recurring fire
            self._wait_timer = self.create_timer(wait_sec, self._on_wait_complete)

    def _on_wait_complete(self):
        """Fix #1: One-shot WAIT timer callback — cancel recurring timer, then advance."""
        if self._wait_timer is not None:
            self._wait_timer.cancel()
            self._wait_timer = None
        if self._state in (PlannerState.CANCELLED, PlannerState.IDLE,
                           PlannerState.COMPLETED, PlannerState.FAILED):
            return
        self._subgoal_completed()

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
            # Fix #7: Add timeout to vision_grounding in verify path
            # Consider @async_timeout when state callback not needed
            result = await asyncio.wait_for(
                self._resolver.vision_grounding(
                    instruction=subgoal.target,
                    scene_graph_json=self._latest_scene_graph,
                    image_base64=self._latest_image_base64,
                    language=self._current_language,
                ),
                timeout=30.0,
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

        except asyncio.TimeoutError:
            self.get_logger().error("Vision verify timed out (30s)")
            self._subgoal_failed("VLM verify timeout")
        except Exception as e:
            # A4 修复: 异常时不再默认 success
            self.get_logger().error(
                f"Vision verify error: {e}\n{traceback.format_exc()}"
            )
            self._subgoal_failed(f"VLM verify exception: {e}")

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
        # 必须保存引用，否则无法取消 → 多次触发 _subgoal_completed 破坏状态
        self._look_around_finish_timer = self.create_timer(
            cmd.timeout_sec,
            lambda: self._finish_look_around(),
        )

    def _publish_look_around_twist(self):
        """发布 LOOK_AROUND 旋转 TwistStamped。"""
        if self._state != PlannerState.LOOKING_AROUND:
            return
        az = (
            self._current_action_cmd.angular_z if self._current_action_cmd else 0.0
        )
        self._pub_cmd_vel.publish(self._make_twist_stamped(angular_z=az))

    def _finish_look_around(self):
        """停止旋转, 完成 LOOK_AROUND 子目标。"""
        # 防止多次触发: finish timer 和 spin timer 都取消
        if self._state != PlannerState.LOOKING_AROUND:
            return

        self._pub_cmd_vel.publish(self._make_twist_stamped())

        if self._look_around_timer is not None:
            self._look_around_timer.cancel()
            self._look_around_timer = None
        if hasattr(self, '_look_around_finish_timer') and self._look_around_finish_timer is not None:
            self._look_around_finish_timer.cancel()
            self._look_around_finish_timer = None

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

            # ── 跟随模式: 到达 ≠ 结束, 使用 PersonTracker 实时跟随 ──
            if (self._follow_mode and active
                    and active.action in (SubGoalAction.NAVIGATE, SubGoalAction.FOLLOW)):
                elapsed = time.time() - self._follow_start_time
                if elapsed > self._follow_timeout:
                    self.get_logger().info(
                        "Follow mode timeout (%.0fs), completing.", elapsed,
                    )
                    self._follow_mode = False
                else:
                    # 优先使用 PersonTracker 实时航点
                    robot_pos = [
                        self._robot_position.get("x", 0) if self._robot_position else 0,
                        self._robot_position.get("y", 0) if self._robot_position else 0,
                        0.0,
                    ]
                    waypoint = self._person_tracker.get_follow_waypoint(robot_pos)
                    if waypoint:
                        self.get_logger().info(
                            "Follow mode: tracker waypoint (%.1f, %.1f) "
                            "(%.0fs / %.0fs)",
                            waypoint["x"], waypoint["y"],
                            elapsed, self._follow_timeout,
                        )
                        cmd = self._action_executor.generate_navigate_command(
                            waypoint, self._robot_position,
                        )
                        self._publish_goal_from_command(cmd)
                        self._set_state(PlannerState.NAVIGATING)
                    elif self._person_tracker.is_lost():
                        self.get_logger().warn(
                            "Follow mode: PersonTracker lost target, "
                            "falling back to resolve (%.0fs / %.0fs)",
                            elapsed, self._follow_timeout,
                        )
                        self._set_state(PlannerState.RESOLVING)
                        self._schedule_async(self._resolve_goal())
                    else:
                        # tracker 有人但 waypoint 计算失败, fallback
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
                    # 限制执行上下文大小，避免无限增长导致 LLM token 超限
                    if len(self._execution_context) > 4000:
                        lines = self._execution_context.split("\n")
                        self._execution_context = "\n".join(lines[-20:])
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
        sg = safe_json_loads(self._latest_scene_graph, default={})
        objs = sg.get("objects", [])
        if isinstance(objs, list) and objs:
            labels = [str(o.get("label", "")) for o in objs[:6] if o]
            visible = ", ".join(l for l in labels if l)
            if visible:
                return f"Reached {target}. Visible: {visible}"
        return f"Reached {target}."

    def _subgoal_failed(self, reason: str = ""):
        """当前子目标失败, 尝试重试或放弃。"""
        # P0: 记录探索失败位置 → frontier 失败记忆
        if self._state == PlannerState.EXPLORING and self._current_goal:
            fail_pos = np.array([
                self._current_goal.target_x,
                self._current_goal.target_y,
            ])
            self._frontier_scorer.record_frontier_failure(fail_pos)

        failed_subgoal: Optional[SubGoal] = None
        if self._current_plan:
            active = self._current_plan.active_subgoal

            # ── 跟随模式: 失败不放弃, PersonTracker 继续追踪 ──
            if (self._follow_mode and active
                    and active.action in (SubGoalAction.NAVIGATE, SubGoalAction.FOLLOW)):
                elapsed = time.time() - self._follow_start_time
                if elapsed < self._follow_timeout:
                    # 尝试用 PersonTracker 恢复
                    robot_pos = [
                        self._robot_position.get("x", 0) if self._robot_position else 0,
                        self._robot_position.get("y", 0) if self._robot_position else 0,
                        0.0,
                    ]
                    waypoint = self._person_tracker.get_follow_waypoint(robot_pos)
                    if waypoint and not self._person_tracker.is_lost():
                        self.get_logger().warn(
                            "Follow mode: nav failed (%s), but tracker has target "
                            "at (%.1f, %.1f) -- re-navigating",
                            reason, waypoint["x"], waypoint["y"],
                        )
                        active.retry_count = 0
                        cmd = self._action_executor.generate_navigate_command(
                            waypoint, self._robot_position,
                        )
                        self._publish_goal_from_command(cmd)
                        self._set_state(PlannerState.NAVIGATING)
                    else:
                        self.get_logger().warn(
                            "Follow mode: target lost (%s), "
                            "re-resolving (%.0fs / %.0fs)",
                            reason, elapsed, self._follow_timeout,
                        )
                        active.retry_count = 0
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
                # ── LERa 恢复: Look → Explain → Replan ──
                sg = safe_json_loads(self._latest_scene_graph, default={})
                current_labels = [
                    o.get("label", "") for o in sg.get("objects", [])
                ]

                lera_action = self._action_executor.lera_recover(
                    failed_action=(
                        f"{failed_subgoal.action.value}({failed_subgoal.target})"
                        if failed_subgoal else "unknown"
                    ),
                    current_labels=current_labels,
                    original_goal=self._current_instruction or "",
                    failure_count=(
                        failed_subgoal.retry_count if failed_subgoal else 1
                    ),
                    event_loop=self._loop,
                )
                self.get_logger().warn(
                    f"[LERa] recovery={lera_action}, triggering replanning "
                    f"({self._replan_count + 1}/{self._max_replan_attempts})"
                )

                if lera_action == "abort":
                    self.get_logger().warn("[LERa] aborting navigation")
                    self._set_state(PlannerState.FAILED, reason="lera_abort")
                    return

                self._set_state(PlannerState.REPLANNING)
                self._schedule_async(
                    self._llm_replan(reason, failed_subgoal, lera_action),
                    is_mission=True,
                )
                return

        self._execute_next_subgoal()

    async def _llm_replan(
        self,
        reason: str,
        failed_subgoal: Optional[SubGoal],
        lera_action: str = "",
    ):
        """基于执行反馈的 LLM 重规划。"""
        try:
            self._replan_count += 1

            # 场景摘要 + 失败反馈
            sg = safe_json_loads(self._latest_scene_graph, default={})
            scene_summary = sg.get("summary", "")

            failed_desc = "unknown"
            if failed_subgoal is not None:
                failed_desc = (
                    f"step={failed_subgoal.step_id}, "
                    f"action={failed_subgoal.action.value}, "
                    f"target={failed_subgoal.target}, "
                    f"retry={failed_subgoal.retry_count}/{failed_subgoal.max_retries}"
                )

            # LERa 恢复策略提示
            lera_hint = ""
            if lera_action == "expand_search":
                lera_hint = "\n[LERa] Suggested strategy: expand search area, try different directions.\n"
            elif lera_action == "requery_goal":
                lera_hint = "\n[LERa] Suggested strategy: re-interpret the goal, it may be ambiguous.\n"
            elif lera_action == "retry_different_path":
                lera_hint = "\n[LERa] Suggested strategy: try a different approach path.\n"

            ctx = f"[Execution history]\n{self._execution_context}\n\n" if self._execution_context else ""
            feedback_summary = (
                f"{scene_summary}\n"
                f"{ctx}"
                f"{lera_hint}"
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
                self._set_state(PlannerState.FAILED, reason="replan_no_llm_response")
                return

            new_plan = self._decomposer.parse_decomposition_response(
                self._current_instruction or "", response
            )
            if not new_plan.subgoals:
                self.get_logger().error("Replan failed: empty subgoal list")
                self._set_state(PlannerState.FAILED, reason="replan_empty_subgoals")
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
            self._set_state(PlannerState.FAILED, reason="replan_exception")


