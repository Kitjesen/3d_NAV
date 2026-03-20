#!/usr/bin/env python3
"""
collect_for_milo.py — sunrise 端数据采集脚本

将 Orbbec RGB-D + Fast-LIO2 SLAM 位姿采集为 COLMAP 格式，供 MILo 训练。

输出目录结构（COLMAP 格式）:
  output_dir/
  ├── images/          ← RGB 图像 (frame_000000.jpg, ...)
  ├── sparse/0/
  │   ├── cameras.txt  ← 相机内参 (PINHOLE model)
  │   ├── images.txt   ← 每张图的位姿 (quaternion + translation)
  │   └── points3D.txt ← 空文件（MILo 不需要预先三角化点）
  └── depth_maps/      ← 深度图 (可选，16bit PNG，mm)

用法 (在 sunrise 上运行):
  # 先启动 SLAM + 相机
  ros2 launch navigation_bringup.launch.py

  # 开始采集（手动遥控机器人走一圈）
  python3 tools/collect_for_milo.py --output /tmp/milo_capture --interval 0.5

  # 采集完成后 Ctrl+C，数据打包传到 GPU 服务器
  tar czf /tmp/milo_capture.tar.gz -C /tmp milo_capture
  scp -P 34418 /tmp/milo_capture.tar.gz root@connect.westd.seetacloud.com:/root/

依赖:
  pip3 install numpy opencv-python

ROS2 话题:
  /camera/color/image_raw        (sensor_msgs/Image, BGR8)
  /camera/color/camera_info      (sensor_msgs/CameraInfo)
  /camera/depth/image_rect_raw   (sensor_msgs/Image, uint16 mm) [可选]
  /nav/odometry                  (nav_msgs/Odometry, SLAM 位姿)
"""

import argparse
import os
import sys
import time
from typing import Optional

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import message_filters


class MiLoCollector(Node):

    def __init__(self, output_dir: str, interval: float, save_depth: bool):
        super().__init__("milo_collector")

        self._output_dir = output_dir
        self._interval = interval
        self._save_depth = save_depth
        self._bridge = CvBridge()

        # 输出目录
        self._img_dir = os.path.join(output_dir, "images")
        self._sparse_dir = os.path.join(output_dir, "sparse", "0")
        self._depth_dir = os.path.join(output_dir, "depth_maps")
        os.makedirs(self._img_dir, exist_ok=True)
        os.makedirs(self._sparse_dir, exist_ok=True)
        if save_depth:
            os.makedirs(self._depth_dir, exist_ok=True)

        # 状态
        self._intrinsics: Optional[dict] = None
        self._frame_count = 0
        self._last_capture_time = 0.0
        self._latest_odom: Optional[Odometry] = None

        # 位姿记录 (写入 images.txt)
        self._image_entries = []  # [(qw,qx,qy,qz, tx,ty,tz, filename), ...]

        # ── 订阅 ──
        self.create_subscription(
            CameraInfo, "/camera/color/camera_info",
            self._on_camera_info, qos_profile=qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry, "/nav/odometry",
            self._on_odom, qos_profile=qos_profile_sensor_data,
        )

        # RGB + Depth 同步
        color_sub = message_filters.Subscriber(
            self, Image, "/camera/color/image_raw",
            qos_profile=qos_profile_sensor_data,
        )
        depth_sub = message_filters.Subscriber(
            self, Image, "/camera/depth/image_rect_raw",
            qos_profile=qos_profile_sensor_data,
        )
        self._sync = message_filters.ApproximateTimeSynchronizer(
            [color_sub, depth_sub], queue_size=10, slop=0.1,
        )
        self._sync.registerCallback(self._on_rgbd)

        self.get_logger().info(
            f"[MiLo采集] 启动 — 输出: {output_dir}, 间隔: {interval}s, 深度: {save_depth}"
        )

    def _on_camera_info(self, msg: CameraInfo):
        if self._intrinsics is not None:
            return
        k = msg.k
        self._intrinsics = {
            "fx": k[0], "fy": k[4], "cx": k[2], "cy": k[5],
            "width": msg.width, "height": msg.height,
        }
        self.get_logger().info(
            f"[MiLo采集] 内参: {msg.width}x{msg.height} "
            f"fx={k[0]:.1f} fy={k[4]:.1f}"
        )

    def _on_odom(self, msg: Odometry):
        self._latest_odom = msg

    def _on_rgbd(self, color_msg: Image, depth_msg: Image):
        now = time.time()
        if now - self._last_capture_time < self._interval:
            return
        if self._intrinsics is None or self._latest_odom is None:
            return

        self._last_capture_time = now

        # 转图像
        try:
            color_bgr = self._bridge.imgmsg_to_cv2(color_msg, "bgr8")
        except Exception as e:
            self.get_logger().warn(f"图像转换失败: {e}")
            return

        # 文件名
        fname = f"frame_{self._frame_count:06d}.jpg"
        img_path = os.path.join(self._img_dir, fname)

        # 保存 RGB
        cv2.imwrite(img_path, color_bgr, [cv2.IMWRITE_JPEG_QUALITY, 95])

        # 保存深度 (可选)
        if self._save_depth:
            try:
                depth = self._bridge.imgmsg_to_cv2(depth_msg, "passthrough")
                depth_path = os.path.join(self._depth_dir, f"frame_{self._frame_count:06d}.png")
                cv2.imwrite(depth_path, depth)
            except Exception:
                pass

        # 提取位姿 — COLMAP 使用 world-to-camera 变换
        odom = self._latest_odom
        p = odom.pose.pose.position
        q = odom.pose.pose.orientation  # ROS: x,y,z,w

        # SLAM 输出的是 camera-to-world (odom frame)
        # COLMAP images.txt 需要 world-to-camera: q_wc, t_wc
        # q_wc = conj(q_cw), t_wc = -R_wc @ t_cw
        qw, qx, qy, qz = q.w, -q.x, -q.y, -q.z  # 共轭
        # 旋转矩阵 R_wc (从 world-to-camera 四元数)
        R = _quat_to_rot(qw, qx, qy, qz)
        tx, ty, tz = -R @ np.array([p.x, p.y, p.z])

        self._image_entries.append((qw, qx, qy, qz, tx, ty, tz, fname))
        self._frame_count += 1

        if self._frame_count % 10 == 0:
            self.get_logger().info(f"[MiLo采集] 已采集 {self._frame_count} 帧")

    def save_colmap_files(self):
        """保存 COLMAP sparse 文件。"""
        if self._intrinsics is None:
            self.get_logger().error("未收到相机内参，无法保存")
            return

        intr = self._intrinsics

        # cameras.txt — PINHOLE 模型
        cameras_path = os.path.join(self._sparse_dir, "cameras.txt")
        with open(cameras_path, "w") as f:
            f.write("# Camera list with one line of data per camera:\n")
            f.write("# CAMERA_ID, MODEL, WIDTH, HEIGHT, PARAMS[]\n")
            f.write(f"1 PINHOLE {intr['width']} {intr['height']} "
                    f"{intr['fx']} {intr['fy']} {intr['cx']} {intr['cy']}\n")

        # images.txt — 每张图的位姿
        images_path = os.path.join(self._sparse_dir, "images.txt")
        with open(images_path, "w") as f:
            f.write("# Image list with two lines of data per image:\n")
            f.write("# IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME\n")
            f.write("# POINTS2D[] as (X, Y, POINT3D_ID)\n")
            for i, (qw, qx, qy, qz, tx, ty, tz, fname) in enumerate(self._image_entries):
                image_id = i + 1
                f.write(f"{image_id} {qw:.8f} {qx:.8f} {qy:.8f} {qz:.8f} "
                        f"{tx:.8f} {ty:.8f} {tz:.8f} 1 {fname}\n")
                f.write("\n")  # COLMAP 格式: 空行表示无 2D 点

        # points3D.txt — 空文件
        points_path = os.path.join(self._sparse_dir, "points3D.txt")
        with open(points_path, "w") as f:
            f.write("# 3D point list (empty — MILo does not require pre-triangulation)\n")

        self.get_logger().info(
            f"[MiLo采集] 保存完成: {self._frame_count} 帧\n"
            f"  cameras.txt: {cameras_path}\n"
            f"  images.txt:  {images_path}\n"
            f"  points3D.txt: {points_path}"
        )


def _quat_to_rot(qw, qx, qy, qz):
    """四元数 → 3x3 旋转矩阵。"""
    return np.array([
        [1 - 2*(qy*qy + qz*qz),   2*(qx*qy - qw*qz),     2*(qx*qz + qw*qy)],
        [2*(qx*qy + qw*qz),       1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qw*qx)],
        [2*(qx*qz - qw*qy),       2*(qy*qz + qw*qx),     1 - 2*(qx*qx + qy*qy)],
    ])


def main():
    parser = argparse.ArgumentParser(description="MILo 数据采集 (sunrise 端)")
    parser.add_argument("--output", default="/tmp/milo_capture", help="输出目录")
    parser.add_argument("--interval", type=float, default=0.5,
                        help="采集间隔 (秒)，0.5=2fps, 1.0=1fps")
    parser.add_argument("--no-depth", action="store_true", help="不保存深度图")
    args = parser.parse_args()

    rclpy.init()
    node = MiLoCollector(args.output, args.interval, save_depth=not args.no_depth)

    print(f"\n{'='*50}")
    print(f"  MILo 数据采集已启动")
    print(f"  输出: {args.output}")
    print(f"  间隔: {args.interval}s ({1/args.interval:.0f} fps)")
    print(f"  遥控机器人走一圈，Ctrl+C 结束采集")
    print(f"{'='*50}\n")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    print(f"\n采集结束，保存 COLMAP 文件...")
    node.save_colmap_files()

    n = node._frame_count
    print(f"\n{'='*50}")
    print(f"  完成！共 {n} 帧")
    print(f"  数据目录: {args.output}")
    print(f"")
    print(f"  下一步:")
    print(f"    tar czf /tmp/milo_capture.tar.gz -C /tmp milo_capture")
    print(f"    scp /tmp/milo_capture.tar.gz GPU_SERVER:/root/MILo/data/")
    print(f"{'='*50}\n")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
