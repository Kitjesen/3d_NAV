#!/usr/bin/env python3
"""
最小投影验证：RGB-D 同步 → 深度取样 → 2D→3D → camera→body 坐标。
不依赖 YOLO/CLIP/tracker，只验证坐标链路。

用法: python3 verify_projection.py [--frames 10]

外参配置（body → camera_link）根据实际安装调整:
  - tx/ty/tz: 相机在机体坐标系的位置 (前/左/上)
  - 旋转: 默认相机朝前，Z轴对齐机体X轴
"""
import sys
import time
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
import message_filters


class ProjectionVerifier(Node):
    def __init__(self, max_frames=10):
        super().__init__('projection_verifier')
        self.max_frames = max_frames
        self.frame_count = 0
        self.intrinsics = None

        # ── 1. 发布 body → camera_link 静态 TF ──
        # 根据实际安装位置调整这些值
        # 假设: 相机装在机体前方 0.1m, 正中, 上方 0.15m, 朝前
        self._broadcast_body_to_camera(
            tx=0.10,   # 前方 10cm
            ty=0.00,   # 正中
            tz=0.15,   # 上方 15cm
        )

        # ── 2. 订阅 CameraInfo ──
        self.create_subscription(
            CameraInfo,
            '/camera/color/camera_info',
            self._camera_info_cb,
            10,
        )

        # ── 3. 同步订阅 RGB + Depth ──
        self._sub_color = message_filters.Subscriber(
            self, Image, '/camera/color/image_raw',
        )
        self._sub_depth = message_filters.Subscriber(
            self, Image, '/camera/depth/image_raw',
        )
        self._sync = message_filters.ApproximateTimeSynchronizer(
            [self._sub_color, self._sub_depth],
            queue_size=5,
            slop=0.05,  # 50ms 容差
        )
        self._sync.registerCallback(self._synced_cb)

        self.get_logger().info('等待 CameraInfo + RGB-D 同步帧...')

    def _broadcast_body_to_camera(self, tx, ty, tz):
        """发布 body → camera_link 静态 TF。

        ROS2 相机坐标系: X右, Y下, Z前
        机体坐标系: X前, Y左, Z上

        旋转: body_X→cam_Z, body_Y→cam_-X, body_Z→cam_-Y
        对应旋转矩阵:
          [[0, -1,  0],
           [0,  0, -1],
           [1,  0,  0]]
        四元数: (qx=0.5, qy=-0.5, qz=0.5, qw=0.5)
        """
        broadcaster = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'body'
        t.child_frame_id = 'camera_link'
        t.transform.translation.x = tx
        t.transform.translation.y = ty
        t.transform.translation.z = tz
        # body→camera_link 旋转四元数
        t.transform.rotation.x = 0.5
        t.transform.rotation.y = -0.5
        t.transform.rotation.z = 0.5
        t.transform.rotation.w = 0.5
        broadcaster.sendTransform(t)
        self.get_logger().info(
            f'Published static TF: body → camera_link '
            f'[{tx:.2f}, {ty:.2f}, {tz:.2f}]'
        )

    def _camera_info_cb(self, msg: CameraInfo):
        if self.intrinsics is None:
            k = msg.k
            self.intrinsics = {
                'fx': k[0], 'fy': k[4],
                'cx': k[2], 'cy': k[5],
                'width': msg.width,
                'height': msg.height,
            }
            self.get_logger().info(
                f'CameraInfo: {msg.width}x{msg.height}, '
                f'fx={k[0]:.1f}, fy={k[4]:.1f}, '
                f'cx={k[2]:.1f}, cy={k[5]:.1f}'
            )

    def _synced_cb(self, color_msg: Image, depth_msg: Image):
        if self.intrinsics is None:
            return
        if self.frame_count >= self.max_frames:
            return

        self.frame_count += 1

        # ── 解析 depth 图像 ──
        depth_w = depth_msg.width
        depth_h = depth_msg.height
        color_w = color_msg.width
        color_h = color_msg.height

        if depth_msg.encoding == '16UC1':
            depth_arr = np.frombuffer(depth_msg.data, dtype=np.uint16).reshape(depth_h, depth_w)
            depth_scale = 0.001  # mm → m
        elif depth_msg.encoding == '32FC1':
            depth_arr = np.frombuffer(depth_msg.data, dtype=np.float32).reshape(depth_h, depth_w)
            depth_scale = 1.0
        else:
            self.get_logger().warn(f'Unknown depth encoding: {depth_msg.encoding}')
            return

        # ── 取 5 个采样点 ──
        fx = self.intrinsics['fx']
        fy = self.intrinsics['fy']
        cx = self.intrinsics['cx']
        cy = self.intrinsics['cy']

        # 时间戳差
        t_color = color_msg.header.stamp.sec + color_msg.header.stamp.nanosec * 1e-9
        t_depth = depth_msg.header.stamp.sec + depth_msg.header.stamp.nanosec * 1e-9
        dt_ms = (t_color - t_depth) * 1000

        print(f'\n=== Frame {self.frame_count}/{self.max_frames} ===')
        print(f'Color: {color_w}x{color_h} ({color_msg.encoding})')
        print(f'Depth: {depth_w}x{depth_h} ({depth_msg.encoding})')
        print(f'Timestamp diff: {dt_ms:.1f}ms')

        # 采样点: 中心 + 四角偏移
        sample_points = [
            ('center', depth_w // 2, depth_h // 2),
            ('left',   depth_w // 4, depth_h // 2),
            ('right',  3 * depth_w // 4, depth_h // 2),
            ('top',    depth_w // 2, depth_h // 4),
            ('bottom', depth_w // 2, 3 * depth_h // 4),
        ]

        # body→camera_link 变换矩阵
        # R: body_X→cam_Z, body_Y→cam_-X, body_Z→cam_-Y
        R_body_cam = np.array([
            [0, -1,  0],
            [0,  0, -1],
            [1,  0,  0],
        ], dtype=np.float64)
        t_body_cam = np.array([0.10, 0.00, 0.15])

        # camera→body: R^T, -R^T @ t
        R_cam_body = R_body_cam.T
        t_cam_body = -R_cam_body @ t_body_cam

        print(f'\n{"Point":<8} {"Pixel":<12} {"Depth(m)":<10} {"Camera XYZ (m)":<28} {"Body XYZ (m)":<28}')
        print('-' * 90)

        for name, u, v in sample_points:
            # 取 3x3 邻域中值减少噪声
            v0 = max(0, v - 1)
            v1 = min(depth_h, v + 2)
            u0 = max(0, u - 1)
            u1 = min(depth_w, u + 2)
            patch = depth_arr[v0:v1, u0:u1]
            valid = patch[patch > 0]
            if len(valid) == 0:
                print(f'{name:<8} ({u:4d},{v:4d})  {"NO DEPTH":<10}')
                continue

            depth_raw = float(np.median(valid))
            depth_m = depth_raw * depth_scale

            if depth_m < 0.1 or depth_m > 10.0:
                print(f'{name:<8} ({u:4d},{v:4d})  {depth_m:<10.3f} OUT OF RANGE')
                continue

            # ── 2D → 3D (camera 坐标系) ──
            # 注意: depth 图像分辨率可能和 color 不同
            # 这里用 depth 图像的内参（假设 depth 和 color 内参接近）
            # 精确做法需要 depth camera_info
            x_cam = (u - cx * depth_w / color_w) * depth_m / (fx * depth_w / color_w)
            y_cam = (v - cy * depth_h / color_h) * depth_m / (fy * depth_h / color_h)
            z_cam = depth_m
            p_cam = np.array([x_cam, y_cam, z_cam])

            # ── camera → body ──
            p_body = R_cam_body @ p_cam + t_cam_body

            print(
                f'{name:<8} ({u:4d},{v:4d})  {depth_m:<10.3f} '
                f'[{p_cam[0]:+7.3f}, {p_cam[1]:+7.3f}, {p_cam[2]:+7.3f}]  '
                f'[{p_body[0]:+7.3f}, {p_body[1]:+7.3f}, {p_body[2]:+7.3f}]'
            )

        if self.frame_count >= self.max_frames:
            print(f'\n=== {self.max_frames} frames done ===')
            raise SystemExit(0)


def main():
    max_frames = 5
    if '--frames' in sys.argv:
        idx = sys.argv.index('--frames')
        if idx + 1 < len(sys.argv):
            max_frames = int(sys.argv[idx + 1])

    rclpy.init()
    node = ProjectionVerifier(max_frames=max_frames)
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
