"""BBox 视觉伺服导航 Mixin — "看到就追" 模式。

从 planner_node.py 提取的方法:
  _bbox_navigate_callback, _start_bbox_nav, _stop_bbox_nav, _bbox_nav_tick
"""

import json

import numpy as np

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

from .vlm_bbox_query import query_object_bbox


class BBoxNavMixin:
    """BBox 视觉伺服导航方法。通过多继承混入 SemanticPlannerNode。"""

    def _bbox_navigate_callback(self, msg: String):
        """
        BBox 视觉伺服导航指令。

        消息格式 (JSON):
          {"target": "红色椅子"}                       — VLM 模式: 用 VLM 在当前帧找目标 bbox
          {"target": "red chair", "mode": "vlm"}       — 同上 (显式指定)
          {"bbox": [x1,y1,x2,y2], "mode": "direct"}   — 直接给 bbox，立即开始追踪
          {}  或 {"stop": true}                        — 停止

        流程 (VLM 模式):
          1. query_object_bbox() 调用视觉 LLM 在最新图像中定位目标
          2. 找到 bbox → _start_bbox_nav() 启动 10Hz 定时器循环
          3. 每帧: bbox_navigator.update() → 发布 TwistStamped
          4. arrived / lost → 停止定时器，停车
        """
        raw = msg.data.strip() if msg.data else ""

        # 停止命令
        if not raw or raw in ("{}", "null"):
            self._stop_bbox_nav()
            return

        try:
            params = json.loads(raw)
        except json.JSONDecodeError as e:
            self.get_logger().warn(f"BBox nav JSON parse error: {e}")
            return

        if params.get("stop", False):
            self._stop_bbox_nav()
            return

        mode = params.get("mode", "vlm")

        if mode == "direct" and "bbox" in params:
            # 直接给 bbox，立即启动
            bbox = params["bbox"]
            if not isinstance(bbox, list) or len(bbox) != 4:
                self.get_logger().warn(f"BBox nav: invalid bbox format: {bbox}")
                return
            self._start_bbox_nav(bbox)

        elif "target" in params:
            # VLM 模式: 异步查询当前图像
            target = params["target"]
            self.get_logger().info(f"BBox nav: VLM querying '{target}'...")

            async def _vlm_query():
                if not self._latest_image_base64:
                    self.get_logger().warn("BBox nav: 无相机图像，无法执行 VLM 查询")
                    return
                try:
                    bbox = await query_object_bbox(
                        self._resolver, self._latest_image_base64, target
                    )
                    if bbox:
                        self.get_logger().info(
                            f"BBox nav: VLM 找到 '{target}' at {bbox}"
                        )
                        self._start_bbox_nav(bbox)
                    else:
                        self.get_logger().info(
                            f"BBox nav: VLM 未在当前帧找到 '{target}'"
                        )
                except Exception as e:
                    self.get_logger().error(f"BBox nav VLM query 失败: {e}")

            self._schedule_async(_vlm_query())

        else:
            self.get_logger().warn(
                f"BBox nav: 消息格式无效 (需要 'target' 或 'bbox' 字段): {raw[:100]}"
            )

    def _start_bbox_nav(self, initial_bbox: list):
        """启动 BBox 视觉伺服导航循环 (10Hz)。"""
        self._bbox_navigator.set_target_bbox(initial_bbox)
        self._bbox_nav_active = True

        # 取消旧定时器（防重入）
        if self._bbox_nav_timer is not None:
            self._bbox_nav_timer.cancel()
        self._bbox_nav_timer = self.create_timer(0.1, self._bbox_nav_tick)
        self.get_logger().info(f"BBox nav 已启动, initial_bbox={initial_bbox}")

    def _stop_bbox_nav(self):
        """停止 BBox 视觉伺服导航，停车。"""
        if self._bbox_nav_timer is not None:
            self._bbox_nav_timer.cancel()
            self._bbox_nav_timer = None
        if not self._bbox_nav_active:
            return
        self._bbox_nav_active = False
        self._bbox_navigator.stop()
        # 停车
        self._pub_cmd_vel.publish(self._make_twist_stamped())
        self.get_logger().info("BBox nav 已停止")

    def _bbox_nav_tick(self):
        """10Hz BBox 导航循环 — 两阶段集成。

        阶段 1 (远程, > servo_takeover_distance):
          bbox → 3D 位置 → 发布为 goal_pose → 规划栈接管 (A* + 局部避障)
        阶段 2 (近程, <= servo_takeover_distance):
          bbox → 3D → PD 视觉伺服 → 直接 cmd_vel (精确接近)
        """
        if not self._bbox_nav_active:
            self._stop_bbox_nav()
            return

        # 机器人当前位姿 (x, y, yaw)
        if self._robot_position is None:
            return
        robot_pose = (
            self._robot_position["x"],
            self._robot_position["y"],
            self._robot_yaw,
        )

        # 获取当前 bbox（沿用上一帧的 _target_bbox）
        current_bbox = self._bbox_navigator._target_bbox
        if current_bbox is None:
            self._bbox_navigator.tick_lost_check()
            if self._bbox_navigator.state == "lost":
                self.get_logger().warn("BBox nav: 目标已丢失 (无初始 bbox)")
                self._stop_bbox_nav()
            return

        # 深度图 + 相机内参
        depth_image = getattr(self, "_latest_depth_image", None)
        intrinsics = getattr(self, "_camera_intrinsics", None)
        if intrinsics is None:
            intrinsics = (600.0, 600.0, 320.0, 240.0)

        if depth_image is None:
            self._bbox_navigator.tick_lost_check()
            if self._bbox_navigator.state == "lost":
                self.get_logger().warn("BBox nav: 无深度图，目标丢失")
                self._stop_bbox_nav()
            return

        # 计算 3D 目标位置
        target_3d = self._bbox_navigator.compute_3d_from_bbox(
            current_bbox, depth_image,
            intrinsics[0], intrinsics[1], intrinsics[2], intrinsics[3],
            robot_pose=robot_pose,
        )
        if target_3d is None:
            return

        # 计算距离
        dx = target_3d[0] - robot_pose[0]
        dy = target_3d[1] - robot_pose[1]
        distance = float(np.sqrt(dx * dx + dy * dy))
        cfg = self._bbox_navigator._config

        # ── 到达判定 ──
        if distance < cfg.arrived_threshold + cfg.target_distance:
            self.get_logger().info(
                f"BBox nav: 已到达目标! distance={distance:.2f}m"
            )
            self._stop_bbox_nav()
            return

        # ── 阶段切换 ──
        if distance > cfg.servo_takeover_distance:
            # 阶段 1: 远程 → 发 goal_pose 给规划栈
            goal_pose = PoseStamped()
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            goal_pose.header.frame_id = "map"
            goal_pose.pose.position.x = float(target_3d[0])
            goal_pose.pose.position.y = float(target_3d[1])
            goal_pose.pose.position.z = 0.0
            goal_pose.pose.orientation.w = 1.0
            self._send_nav2_goal(goal_pose, "bbox_visual_target")
            self.get_logger().debug(
                f"BBox nav [远程]: distance={distance:.1f}m → goal_pose ({target_3d[0]:.1f}, {target_3d[1]:.1f})"
            )
        else:
            # 阶段 2: 近程 → PD 视觉伺服直接控制
            result = self._bbox_navigator.update(
                bbox=current_bbox,
                depth_image=depth_image,
                camera_intrinsics=intrinsics,
                robot_pose=robot_pose,
            )
            state = result["state"]
            if state == "tracking":
                self._pub_cmd_vel.publish(
                    self._make_twist_stamped(
                        linear_x=result["linear_x"],
                        angular_z=result["angular_z"],
                    )
                )
                self.get_logger().debug(
                    f"BBox nav [近程]: distance={distance:.1f}m → lx={result['linear_x']:.2f}, az={result['angular_z']:.2f}"
                )
            elif state == "arrived":
                self.get_logger().info(f"BBox nav: 已到达! distance={result['distance']:.2f}m")
                self._stop_bbox_nav()
            elif state == "lost":
                self.get_logger().warn("BBox nav: 目标丢失")
                self._stop_bbox_nav()
