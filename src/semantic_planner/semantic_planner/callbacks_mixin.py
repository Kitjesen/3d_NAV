"""感知数据回调 Mixin — 场景图/图像/里程计/代价地图/地形回调。

从 planner_node.py 提取的方法:
  _scene_graph_callback, _image_callback, _odom_callback,
  _costmap_callback, _terrain_to_costmap_callback, _parse_pointcloud2_xyz
"""

import math
from typing import Optional

import numpy as np

from semantic_common import safe_json_loads

from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import Image, PointCloud2

from .planner_state import PlannerState


class CallbacksMixin:
    """感知数据回调方法。通过多继承混入 SemanticPlannerNode。"""

    def _scene_graph_callback(self, msg):
        """更新最新场景图 + 增量更新房间-物体 KG + 情节记忆。

        优化 (KeySG / Hydra 启发):
          - 通过 hash 检测场景图是否真正变化，未变化则跳过全量解析
          - JSON 只解析一次，结果缓存到 _latest_scene_graph_parsed
          - PersonTracker 直接复用已解析的 dict，消除第二次 json.loads
        """
        import time
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
            from nav_msgs.msg import OccupancyGrid as OccGrid
            occ = OccGrid()
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
