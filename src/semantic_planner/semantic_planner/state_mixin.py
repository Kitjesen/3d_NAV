"""状态管理 + 监控 + 持久化 Mixin。

从 planner_node.py 提取的方法:
  _monitor_callback, _publish_goal_from_command,
  _set_state, _publish_status,
  _load_semantic_data, _save_semantic_data
"""

import json
import math
import time
from typing import Optional

import numpy as np

from semantic_common import safe_json_loads

from geometry_msgs.msg import PoseStamped

from .action_executor import ActionCommand
from .planner_state import PlannerState


class StateMixin:
    """状态管理、监控和持久化方法。通过多继承混入 SemanticPlannerNode。"""

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
        from std_msgs.msg import String as RosString
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

        msg = RosString()
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
