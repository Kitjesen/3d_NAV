# Inspired by DimOS visualization/rerun/bridge.py, Apache 2.0 License
"""
LingTu Rerun 可视化节点 — 3D 实时展示导航全过程

订阅 ROS2 话题，实时推送到 Rerun Web 查看器。
浏览器打开 http://<robot_ip>:9090 即可看到 3D 实时画面。

可视化内容:
  - 相机 RGB 画面 + 检测结果叠加
  - 深度图
  - 机器人位姿轨迹
  - 场景图物体 (3D bbox + 标签)
  - 全局路径 + 局部路径
  - 地图点云
  - 导航状态面板

用法:
  ros2 run semantic_planner rerun_viewer
  # 浏览器打开 http://192.168.66.190:9090

或通过 CLI:
  lingtu viz
"""

import json
import logging
import math
import struct
import threading
import time
from typing import Optional

import numpy as np

try:
    import rerun as rr
    import rerun.blueprint as rrb
    HAS_RERUN = True
except ImportError:
    HAS_RERUN = False

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import String
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, PointStamped, TwistStamped
from sensor_msgs.msg import Image, PointCloud2

logger = logging.getLogger(__name__)

# QoS: 传感器数据用 BEST_EFFORT
_SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class RerunViewerNode(Node):
    """Rerun 3D 可视化 ROS2 节点。"""

    def __init__(self):
        super().__init__("rerun_viewer")

        if not HAS_RERUN:
            self.get_logger().error("rerun-sdk 未安装: pip install rerun-sdk")
            return

        # ── 参数 ──
        self.declare_parameter("web_port", 9090)
        self.declare_parameter("app_name", "LingTu")
        self.declare_parameter("max_image_fps", 3.0)
        self.declare_parameter("max_pointcloud_fps", 0.5)

        web_port = self.get_parameter("web_port").value
        app_name = self.get_parameter("app_name").value
        self._max_image_interval = 1.0 / self.get_parameter("max_image_fps").value
        self._max_pc_interval = 1.0 / self.get_parameter("max_pointcloud_fps").value

        # ── Rerun 初始化 ──
        rr.init(app_name, spawn=False)
        # gRPC 服务模式 — 客户端连接查看
        grpc_port = web_port
        if hasattr(rr, 'serve_grpc'):
            uri = rr.serve_grpc(grpc_port=grpc_port, server_memory_limit="256MiB")
            self.get_logger().info(
                f"Rerun gRPC: {uri}"
            )
            self.get_logger().info(
                f"查看方式: rerun 'rerun+http://192.168.66.190:{grpc_port}/proxy'"
            )
        else:
            rr.spawn()
        self.get_logger().info(
            f"Rerun Web 查看器: http://0.0.0.0:{web_port}"
        )

        # 布局: 左边 3D 场景，右边相机 RGB + 深度 + 状态
        try:
            rr.send_blueprint(rrb.Blueprint(
                rrb.Horizontal(
                    rrb.Spatial3DView(name="3D Scene", origin="world"),
                    rrb.Vertical(
                        rrb.Spatial2DView(name="Camera RGB", origin="camera/rgb"),
                        rrb.Spatial2DView(name="Depth", origin="camera/depth"),
                        rrb.TextLogView(name="Status", origin="status"),
                    ),
                ),
            ))
        except Exception as e:
            self.get_logger().warn(f"Blueprint setup failed (non-fatal): {e}")

        # ── 节流状态 ──
        self._last_image_time = 0.0
        self._last_depth_time = 0.0
        self._last_pc_time = 0.0
        self._odom_trail = []  # 轨迹点
        self._frame_count = 0

        # ── 订阅 ──
        # 相机
        self.create_subscription(
            Image, "/camera/color/image_raw",
            self._rgb_callback, _SENSOR_QOS
        )
        self.create_subscription(
            Image, "/camera/depth/image_raw",
            self._depth_callback, _SENSOR_QOS
        )
        # 里程计
        self.create_subscription(
            Odometry, "/nav/odometry",
            self._odom_callback, _SENSOR_QOS
        )
        # 路径
        self.create_subscription(
            Path, "/nav/global_path",
            self._global_path_callback, 10
        )
        self.create_subscription(
            Path, "/nav/local_path",
            self._local_path_callback, 10
        )
        # 航点
        self.create_subscription(
            PointStamped, "/nav/way_point",
            self._waypoint_callback, 10
        )
        # 目标
        self.create_subscription(
            PoseStamped, "/nav/goal_pose",
            self._goal_callback, 10
        )
        # 场景图
        self.create_subscription(
            String, "/nav/semantic/scene_graph",
            self._scene_graph_callback, 1
        )
        # 地图点云
        self.create_subscription(
            PointCloud2, "/nav/map_cloud",
            self._map_cloud_callback, _SENSOR_QOS
        )
        # 状态
        self.create_subscription(
            String, "/nav/planner_status",
            self._status_callback, 10
        )
        self.create_subscription(
            String, "/nav/semantic/status",
            self._semantic_status_callback, 10
        )
        # cmd_vel
        self.create_subscription(
            TwistStamped, "/nav/cmd_vel",
            self._cmd_vel_callback, _SENSOR_QOS
        )

        # 状态定时刷新
        self.create_timer(1.0, self._status_timer)

        self.get_logger().info("RerunViewerNode started")

    # ════════════════════════════════════════
    #  相机
    # ════════════════════════════════════════

    def _rgb_callback(self, msg: Image):
        now = time.time()
        if now - self._last_image_time < self._max_image_interval:
            return
        self._last_image_time = now
        self._frame_count += 1

        try:
            h, w = msg.height, msg.width
            if msg.encoding in ("rgb8", "RGB8"):
                arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)
            elif msg.encoding in ("bgr8", "BGR8"):
                arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)
                arr = arr[:, :, ::-1]  # BGR → RGB
            else:
                return

            rr.set_time_sequence("frame", self._frame_count)
            rr.log("camera/rgb", rr.Image(arr))
        except Exception as e:
            self.get_logger().debug(f"RGB log error: {e}")

    def _depth_callback(self, msg: Image):
        now = time.time()
        if now - self._last_depth_time < self._max_image_interval:
            return
        self._last_depth_time = now

        try:
            h, w = msg.height, msg.width
            if msg.encoding == "16UC1":
                arr = np.frombuffer(msg.data, dtype=np.uint16).reshape(h, w)
            elif msg.encoding == "32FC1":
                arr = np.frombuffer(msg.data, dtype=np.float32).reshape(h, w)
            else:
                return

            rr.log("camera/depth", rr.DepthImage(arr, meter=1000.0))
        except Exception as e:
            self.get_logger().debug(f"Depth log error: {e}")

    # ════════════════════════════════════════
    #  里程计 + 轨迹
    # ════════════════════════════════════════

    def _odom_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        # 机器人位置
        rr.log("world/robot", rr.Points3D(
            [[p.x, p.y, p.z + 0.3]],
            colors=[[0, 255, 100]],
            radii=[0.15],
        ))

        # 轨迹
        self._odom_trail.append([p.x, p.y, p.z + 0.05])
        if len(self._odom_trail) > 2000:
            self._odom_trail = self._odom_trail[-1000:]

        if len(self._odom_trail) > 2:
            rr.log("world/trail", rr.LineStrips3D(
                [self._odom_trail],
                colors=[[100, 200, 255, 120]],
                radii=[0.02],
            ))

        # 朝向箭头
        yaw = 2.0 * math.atan2(q.z, q.w)
        dx = 0.5 * math.cos(yaw)
        dy = 0.5 * math.sin(yaw)
        rr.log("world/heading", rr.Arrows3D(
            origins=[[p.x, p.y, p.z + 0.3]],
            vectors=[[dx, dy, 0]],
            colors=[[0, 255, 100]],
            radii=[0.03],
        ))

    # ════════════════════════════════════════
    #  路径 + 航点 + 目标
    # ════════════════════════════════════════

    def _global_path_callback(self, msg: Path):
        if not msg.poses:
            return
        pts = [[p.pose.position.x, p.pose.position.y, p.pose.position.z + 0.1]
               for p in msg.poses]
        rr.log("world/global_path", rr.LineStrips3D(
            [pts], colors=[[50, 200, 50]], radii=[0.04],
        ))

    def _local_path_callback(self, msg: Path):
        if not msg.poses:
            return
        pts = [[p.pose.position.x, p.pose.position.y, p.pose.position.z + 0.15]
               for p in msg.poses]
        rr.log("world/local_path", rr.LineStrips3D(
            [pts], colors=[[255, 200, 50]], radii=[0.03],
        ))

    def _waypoint_callback(self, msg: PointStamped):
        rr.log("world/waypoint", rr.Points3D(
            [[msg.point.x, msg.point.y, msg.point.z + 0.2]],
            colors=[[255, 100, 0]],
            radii=[0.12],
        ))

    def _goal_callback(self, msg: PoseStamped):
        p = msg.pose.position
        rr.log("world/goal", rr.Points3D(
            [[p.x, p.y, p.z + 0.2]],
            colors=[[255, 50, 50]],
            radii=[0.2],
        ))
        rr.log("world/goal_label", rr.TextLog(f"Goal: ({p.x:.1f}, {p.y:.1f})"))

    # ════════════════════════════════════════
    #  场景图 (语义物体)
    # ════════════════════════════════════════

    def _scene_graph_callback(self, msg: String):
        try:
            sg = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        objects = sg.get("objects", [])
        if not objects:
            return

        positions = []
        labels = []
        colors = []

        for obj in objects:
            pos = obj.get("position", [0, 0, 0])
            label = obj.get("label", "?")
            conf = obj.get("confidence", 0.5)

            # position 可能是 list [x,y,z] 或 dict {"x":..,"y":..,"z":..}
            try:
                if isinstance(pos, dict):
                    px, py, pz = float(pos.get("x", 0)), float(pos.get("y", 0)), float(pos.get("z", 0))
                elif isinstance(pos, (list, tuple)) and len(pos) >= 3:
                    px, py, pz = float(pos[0]), float(pos[1]), float(pos[2])
                else:
                    continue
            except (ValueError, TypeError):
                continue

            positions.append([px, py, pz + 0.3])
            labels.append(f"{label} ({conf:.0%})")

            # 颜色按置信度
            g = int(conf * 255)
            colors.append([100, g, 255 - g])

        rr.log("world/objects", rr.Points3D(
            positions,
            colors=colors,
            radii=[0.1] * len(positions),
            labels=labels,
        ))

    # ════════════════════════════════════════
    #  地图点云
    # ════════════════════════════════════════

    def _map_cloud_callback(self, msg: PointCloud2):
        now = time.time()
        if now - self._last_pc_time < self._max_pc_interval:
            return
        self._last_pc_time = now

        try:
            # 简单解析 PointCloud2 (假设 XYZ float32)
            point_step = msg.point_step
            n_points = msg.width * msg.height
            if n_points == 0 or point_step < 12:
                return

            data = np.frombuffer(msg.data, dtype=np.uint8)
            # 限制点数避免 Rerun 卡顿
            max_points = 50000
            stride = max(1, n_points // max_points)

            points = []
            for i in range(0, n_points, stride):
                offset = i * point_step
                if offset + 12 > len(data):
                    break
                x, y, z = struct.unpack_from("fff", data, offset)
                if not (math.isnan(x) or math.isnan(y) or math.isnan(z)):
                    points.append([x, y, z])

            if points:
                rr.log("world/map_cloud", rr.Points3D(
                    points,
                    colors=[[180, 180, 200, 80]],
                    radii=[0.02],
                ))
        except Exception as e:
            self.get_logger().debug(f"Map cloud error: {e}")

    # ════════════════════════════════════════
    #  状态
    # ════════════════════════════════════════

    def _status_callback(self, msg: String):
        rr.log("status/planner", rr.TextLog(f"Planner: {msg.data}"))

    def _semantic_status_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            state = data.get("state", "unknown")
            rr.log("status/semantic", rr.TextLog(f"Semantic: {state}"))
        except:
            rr.log("status/semantic", rr.TextLog(msg.data[:100]))

    def _cmd_vel_callback(self, msg: TwistStamped):
        lx = msg.twist.linear.x
        az = msg.twist.angular.z
        rr.log("status/cmd_vel", rr.TextLog(f"v={lx:.2f} m/s  w={az:.2f} rad/s"))

    def _status_timer(self):
        """每秒刷新状态面板。"""
        trail_len = len(self._odom_trail)
        rr.log("status/info", rr.TextLog(
            f"LingTu v1.8.0 | frames: {self._frame_count} | trail: {trail_len} pts"
        ))


def main(args=None):
    if not HAS_RERUN:
        print("ERROR: rerun-sdk 未安装。运行: pip install rerun-sdk")
        return

    rclpy.init(args=args)
    node = RerunViewerNode()
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
