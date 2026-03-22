"""目标解析 + SCG 路径规划 Mixin — Fast/Slow 双进程 + SCG 集成。

从 planner_node.py 提取的方法:
  _resolve_goal, _scg_result_callback, _plan_with_scg, _handle_navigate_result
"""

import asyncio
import json
import math
import time
import traceback
from typing import Optional

from semantic_common import safe_json_loads

from geometry_msgs.msg import PoseStamped

from .goal_resolver import GoalResult
from .planner_state import PlannerState


class ResolveMixin:
    """目标解析与 SCG 路径规划方法。通过多继承混入 SemanticPlannerNode。"""

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

    def _scg_result_callback(self, msg):
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

        from std_msgs.msg import String
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
