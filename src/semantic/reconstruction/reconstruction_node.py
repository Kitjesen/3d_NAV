"""
reconstruction_node — 三维重建 ROS2 节点

管道:
  RGB + Depth (同步) → 体素颜色表 → 彩色点云
  场景图 JSON         → 语义标签注入
  定时器 1Hz          → 发布 /nav/reconstruction/semantic_cloud
  save_ply 服务       → 保存 PLY 到 maps/reconstruction/

订阅:
  /camera/color/image_raw        (sensor_msgs/Image)
  /camera/depth/image_rect_raw   (sensor_msgs/Image)
  /camera/color/camera_info      (sensor_msgs/CameraInfo)
  /nav/semantic/scene_graph      (std_msgs/String JSON)

发布:
  /nav/reconstruction/semantic_cloud  (sensor_msgs/PointCloud2)
  /nav/reconstruction/stats           (std_msgs/String JSON)

服务:
  /nav/reconstruction/save_ply        (std_srvs/Trigger)
"""

import json
import os
import time
from typing import Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from std_msgs.msg import String
from std_srvs.srv import Trigger

import message_filters
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from .color_projector import ColorProjector
from .semantic_labeler import SemanticLabeler
from .ply_writer import save_ply_with_labels


# QoS: 点云用 BEST_EFFORT（大数据量，丢帧可接受）
_CLOUD_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    depth=1,
)


class ReconstructionNode(Node):

    def __init__(self):
        super().__init__("reconstruction_node")

        # ── 参数 ──
        self.declare_parameter("publish_rate", 1.0)
        self.declare_parameter("voxel_size", 0.05)
        self.declare_parameter("camera_frame", "camera_color_optical_frame")
        self.declare_parameter("world_frame", "map")
        self.declare_parameter("save_dir", "maps/reconstruction")
        self.declare_parameter("min_points_to_publish", 1000)

        self._publish_rate = self.get_parameter("publish_rate").value
        self._camera_frame = self.get_parameter("camera_frame").value
        self._world_frame = self.get_parameter("world_frame").value
        self._save_dir = self.get_parameter("save_dir").value
        self._min_points = self.get_parameter("min_points_to_publish").value

        # ── 核心模块 ──
        voxel_size = self.get_parameter("voxel_size").value
        self._projector = ColorProjector(voxel_size=voxel_size)
        self._labeler = SemanticLabeler()
        self._bridge = CvBridge()

        # ── TF ──
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # ── 相机内参缓存 ──
        self._intrinsics: Optional[dict] = None  # {fx, fy, cx, cy}

        # ── 订阅（RGB + Depth 同步） ──
        color_sub = message_filters.Subscriber(
            self, Image, "color_image", qos_profile=qos_profile_sensor_data
        )
        depth_sub = message_filters.Subscriber(
            self, Image, "depth_image", qos_profile=qos_profile_sensor_data
        )
        self._sync = message_filters.ApproximateTimeSynchronizer(
            [color_sub, depth_sub], queue_size=5, slop=0.1
        )
        self._sync.registerCallback(self._on_rgbd)

        self.create_subscription(
            CameraInfo, "camera_info", self._on_camera_info,
            qos_profile=qos_profile_sensor_data,
        )
        self.create_subscription(
            String, "scene_graph", self._on_scene_graph, 10
        )

        # ── 发布 ──
        self._cloud_pub = self.create_publisher(
            PointCloud2, "semantic_cloud", _CLOUD_QOS
        )
        self._stats_pub = self.create_publisher(String, "stats", 10)

        # ── 服务 ──
        self.create_service(Trigger, "save_ply", self._on_save_ply)

        # ── 定时器 ──
        period = 1.0 / max(self._publish_rate, 0.1)
        self.create_timer(period, self._publish_cloud)

        self.get_logger().info(
            f"[Reconstruction] 启动 — voxel={voxel_size}m "
            f"rate={self._publish_rate}Hz "
            f"camera_frame={self._camera_frame}"
        )

    # ── 回调 ────────────────────────────────────────────────────

    def _on_camera_info(self, msg: CameraInfo):
        if self._intrinsics is not None:
            return  # 只需第一帧
        k = msg.k  # 3×3 row-major
        self._intrinsics = {
            "fx": k[0], "fy": k[4],
            "cx": k[2], "cy": k[5],
        }
        self.get_logger().info(
            f"[Reconstruction] 相机内参已接收 fx={k[0]:.1f} fy={k[4]:.1f}"
        )

    def _on_scene_graph(self, msg: String):
        n = self._labeler.update_from_scene_graph(msg.data)
        if n > 0:
            self.get_logger().debug(f"[Reconstruction] 场景图更新 {n} 个物体")

    def _on_rgbd(self, color_msg: Image, depth_msg: Image):
        if self._intrinsics is None:
            return

        # 查询相机到世界的 TF
        try:
            tf = self._tf_buffer.lookup_transform(
                self._world_frame,
                self._camera_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
        except Exception:
            return  # TF 未就绪时静默跳过

        # ROS → numpy
        try:
            color_bgr = self._bridge.imgmsg_to_cv2(color_msg, "bgr8")
            depth_mm = self._bridge.imgmsg_to_cv2(depth_msg, "passthrough")
        except Exception as e:
            self.get_logger().warn(f"[Reconstruction] 图像转换失败: {e}")
            return

        # TF 转为 4×4 矩阵
        camera_to_world = _tf_to_matrix(tf.transform)

        n_updated = self._projector.update_from_frame(
            color_bgr=color_bgr,
            depth_mm=depth_mm,
            fx=self._intrinsics["fx"],
            fy=self._intrinsics["fy"],
            cx=self._intrinsics["cx"],
            cy=self._intrinsics["cy"],
            camera_to_world=camera_to_world,
        )

    def _publish_cloud(self):
        xyzrgb = self._projector.get_colored_cloud()
        if len(xyzrgb) < self._min_points:
            return

        # 语义标签注入
        labels = self._labeler.label_cloud(xyzrgb)

        # 发布 PointCloud2
        cloud_msg = _make_xyzrgb_cloud(xyzrgb, self._world_frame, self.get_clock().now().to_msg())
        self._cloud_pub.publish(cloud_msg)

        # 统计信息
        n_labeled = sum(1 for l in labels if l != "background")
        stats = {
            "total_points": len(xyzrgb),
            "labeled_points": n_labeled,
            "objects": self._labeler.object_count,
            "voxels": self._projector.voxel_count,
        }
        self._stats_pub.publish(String(data=json.dumps(stats)))

    def _on_save_ply(self, request, response):
        xyzrgb = self._projector.get_colored_cloud()
        if len(xyzrgb) == 0:
            response.success = False
            response.message = "无点云数据，请先驱动机器人采集"
            return response

        labels = self._labeler.label_cloud(xyzrgb)
        timestamp = int(time.time())
        filepath = os.path.join(self._save_dir, f"reconstruction_{timestamp}.ply")

        try:
            n = save_ply_with_labels(xyzrgb, labels, filepath)
            response.success = True
            response.message = f"已保存 {n} 个点到 {filepath}"
            self.get_logger().info(f"[Reconstruction] PLY 保存完成: {filepath} ({n} pts)")
        except Exception as e:
            response.success = False
            response.message = f"保存失败: {e}"
            self.get_logger().error(f"[Reconstruction] PLY 保存失败: {e}")

        return response


# ── 工具函数 ─────────────────────────────────────────────────

def _tf_to_matrix(transform) -> np.ndarray:
    """ROS Transform → 4×4 numpy 变换矩阵。"""
    t = transform.translation
    r = transform.rotation

    # 四元数 → 旋转矩阵（ROS 使用 x,y,z,w 顺序）
    x, y, z, w = r.x, r.y, r.z, r.w
    mat = np.array([
        [1 - 2*(y*y + z*z),   2*(x*y - w*z),     2*(x*z + w*y),   t.x],
        [2*(x*y + w*z),       1 - 2*(x*x + z*z), 2*(y*z - w*x),   t.y],
        [2*(x*z - w*y),       2*(y*z + w*x),     1 - 2*(x*x+y*y), t.z],
        [0.0,                 0.0,               0.0,              1.0],
    ], dtype=np.float64)
    return mat


def _make_xyzrgb_cloud(xyzrgb: np.ndarray, frame_id: str, stamp) -> PointCloud2:
    """将 (N, 6) XYZRGB 数组打包为 ROS PointCloud2。"""
    n = len(xyzrgb)

    buf = np.zeros(n, dtype=[
        ("x", np.float32), ("y", np.float32), ("z", np.float32),
        ("rgb", np.float32),
    ])
    buf["x"] = xyzrgb[:, 0]
    buf["y"] = xyzrgb[:, 1]
    buf["z"] = xyzrgb[:, 2]

    r = xyzrgb[:, 3].astype(np.uint32)
    g = xyzrgb[:, 4].astype(np.uint32)
    b = xyzrgb[:, 5].astype(np.uint32)
    rgb_uint32 = (r << 16) | (g << 8) | b
    buf["rgb"] = rgb_uint32.view(np.float32)

    msg = PointCloud2()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.height = 1
    msg.width = n
    msg.fields = [
        PointField(name="x",   offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name="y",   offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name="z",   offset=8,  datatype=PointField.FLOAT32, count=1),
        PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 16
    msg.row_step = 16 * n
    msg.data = buf.tobytes()
    msg.is_dense = True
    return msg


def main(args=None):
    rclpy.init(args=args)
    node = ReconstructionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
