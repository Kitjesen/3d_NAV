#!/usr/bin/env python3
"""Normalize private Gazebo/GZ ROS topics into LingTu runtime topics.

This adapter is deliberately simulation-only. It does not publish goals or
commands. Its job is to keep raw Gazebo frames and topics out of `/nav/*` by
owning the simulator boundary:

  /lingtu/gazebo/raw/* -> /nav/odometry, /nav/map_cloud, camera topics
  TF: map -> odom, odom -> body

The point-cloud path treats raw Gazebo lidar points as sensor-frame points,
projects them through the configured lidar->body extrinsic, and then projects
the map cloud through the latest odom->body pose. This keeps `/nav/map_cloud`
odom-fixed and `/nav/registered_cloud` body-relative.
"""

from __future__ import annotations

import copy
import math
import sys
import time
from collections import deque
from dataclasses import dataclass


@dataclass(frozen=True)
class Pose3:
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float


@dataclass(frozen=True)
class TimedPose:
    stamp_sec: float
    pose: Pose3


def _stamp_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _quat_rotate(point: tuple[float, float, float], quat: tuple[float, float, float, float]) -> tuple[float, float, float]:
    """Rotate a point by a normalized xyzw quaternion."""

    x, y, z = point
    qx, qy, qz, qw = quat
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 1e-12:
        return point
    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm

    # q * p * q^-1, expanded to avoid temporary quaternion objects.
    tx = 2.0 * (qy * z - qz * y)
    ty = 2.0 * (qz * x - qx * z)
    tz = 2.0 * (qx * y - qy * x)
    return (
        x + qw * tx + (qy * tz - qz * ty),
        y + qw * ty + (qz * tx - qx * tz),
        z + qw * tz + (qx * ty - qy * tx),
    )


def _transform_xyz(
    point: tuple[float, float, float],
    *,
    translation: tuple[float, float, float],
    rotation_xyzw: tuple[float, float, float, float],
) -> tuple[float, float, float]:
    rx, ry, rz = _quat_rotate(point, rotation_xyzw)
    tx, ty, tz = translation
    return rx + tx, ry + ty, rz + tz


def main() -> int:
    try:
        import rclpy
        from geometry_msgs.msg import TransformStamped
        from nav_msgs.msg import Odometry
        from rclpy.node import Node
        from sensor_msgs.msg import CameraInfo, Image, LaserScan, PointCloud2
        from sensor_msgs_py import point_cloud2
        from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
    except ImportError as exc:
        print(f"Gazebo runtime adapter requires ROS 2 Python packages: {exc}", file=sys.stderr)
        return 2

    from sim.engine.bridge.gazebo_bridge import GazeboBridgeConfig

    class GazeboRuntimeAdapter(Node):
        def __init__(self) -> None:
            super().__init__("lingtu_gazebo_runtime_adapter")
            self._cfg = GazeboBridgeConfig()
            raw = self._cfg.raw_ros_topics()
            self.declare_parameter("raw_lidar_frame", self._cfg.frames.lidar_frame)
            self.declare_parameter("lidar_to_body_x", 0.28)
            self.declare_parameter("lidar_to_body_y", 0.0)
            self.declare_parameter("lidar_to_body_z", 0.20)
            self.declare_parameter("prefer_point_cloud_over_scan", True)
            self.declare_parameter("scan_fallback_after_sec", 0.75)
            self._raw_lidar_frame = str(self.get_parameter("raw_lidar_frame").value)
            self._lidar_to_body = (
                float(self.get_parameter("lidar_to_body_x").value),
                float(self.get_parameter("lidar_to_body_y").value),
                float(self.get_parameter("lidar_to_body_z").value),
            )
            self._prefer_point_cloud_over_scan = bool(
                self.get_parameter("prefer_point_cloud_over_scan").value
            )
            self._scan_fallback_after_sec = float(
                self.get_parameter("scan_fallback_after_sec").value
            )
            self._latest_body_pose = Pose3(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
            self._pose_history: deque[TimedPose] = deque(maxlen=300)
            self._last_point_cloud_wall_time = 0.0

            self._odom_pub = self.create_publisher(Odometry, self._cfg.lingtu_odometry, 10)
            self._map_cloud_pub = self.create_publisher(PointCloud2, self._cfg.lingtu_map_cloud, 2)
            self._registered_cloud_pub = self.create_publisher(
                PointCloud2,
                self._cfg.lingtu_registered_cloud,
                2,
            )
            self._color_pub = self.create_publisher(Image, self._cfg.lingtu_color_image, 2)
            self._depth_pub = self.create_publisher(Image, self._cfg.lingtu_depth_image, 2)
            self._info_pub = self.create_publisher(CameraInfo, self._cfg.lingtu_camera_info, 2)

            self._tf_pub = TransformBroadcaster(self)
            self._static_tf_pub = StaticTransformBroadcaster(self)
            self._publish_static_map_to_odom()

            self.create_subscription(Odometry, raw["odometry"], self._on_odom, 10)
            self.create_subscription(PointCloud2, raw["lidar_points"], self._on_cloud, 2)
            self.create_subscription(LaserScan, raw["lidar_scan"], self._on_scan, 2)
            self.create_subscription(Image, raw["color_image"], self._on_color, 2)
            self.create_subscription(Image, raw["depth_image"], self._on_depth, 2)
            self.create_subscription(CameraInfo, raw["camera_info"], self._on_info, 2)

            self.get_logger().info(
                "Gazebo runtime adapter: raw Gazebo topics -> LingTu /nav topics, "
                "TF map->odom->body"
            )

        def _publish_static_map_to_odom(self) -> None:
            stamp = self.get_clock().now().to_msg()
            tfs = []

            map_to_odom = TransformStamped()
            map_to_odom.header.stamp = stamp
            map_to_odom.header.frame_id = "map"
            map_to_odom.child_frame_id = "odom"
            map_to_odom.transform.rotation.w = 1.0
            tfs.append(map_to_odom)

            body_to_lidar = TransformStamped()
            body_to_lidar.header.stamp = stamp
            body_to_lidar.header.frame_id = "body"
            body_to_lidar.child_frame_id = self._cfg.frames.lidar_frame
            body_to_lidar.transform.translation.x = self._lidar_to_body[0]
            body_to_lidar.transform.translation.y = self._lidar_to_body[1]
            body_to_lidar.transform.translation.z = self._lidar_to_body[2]
            body_to_lidar.transform.rotation.w = 1.0
            tfs.append(body_to_lidar)

            body_to_camera = TransformStamped()
            body_to_camera.header.stamp = stamp
            body_to_camera.header.frame_id = "body"
            body_to_camera.child_frame_id = self._cfg.frames.camera_frame
            body_to_camera.transform.translation.x = 0.36
            body_to_camera.transform.translation.y = 0.0
            body_to_camera.transform.translation.z = 0.22
            body_to_camera.transform.rotation.w = 1.0
            tfs.append(body_to_camera)

            self._static_tf_pub.sendTransform(tfs)

        def _on_odom(self, msg: Odometry) -> None:
            out = copy.deepcopy(msg)
            out.header.frame_id = "odom"
            out.child_frame_id = "body"
            self._odom_pub.publish(out)
            pose = Pose3(
                out.pose.pose.position.x,
                out.pose.pose.position.y,
                out.pose.pose.position.z,
                out.pose.pose.orientation.x,
                out.pose.pose.orientation.y,
                out.pose.pose.orientation.z,
                out.pose.pose.orientation.w,
            )
            self._latest_body_pose = pose
            self._pose_history.append(TimedPose(_stamp_sec(out.header.stamp), pose))

            tf = TransformStamped()
            tf.header.stamp = out.header.stamp
            tf.header.frame_id = "odom"
            tf.child_frame_id = "body"
            tf.transform.translation.x = out.pose.pose.position.x
            tf.transform.translation.y = out.pose.pose.position.y
            tf.transform.translation.z = out.pose.pose.position.z
            tf.transform.rotation = out.pose.pose.orientation
            self._tf_pub.sendTransform(tf)

        def _on_cloud(self, msg: PointCloud2) -> None:
            self._last_point_cloud_wall_time = time.monotonic()
            registered_cloud = self._transform_cloud(msg, target_frame="body")
            if self._point_count(registered_cloud) <= 0:
                return
            self._registered_cloud_pub.publish(registered_cloud)

            map_cloud = self._body_cloud_to_odom(registered_cloud)
            map_cloud.header.frame_id = "odom"
            if self._point_count(map_cloud) <= 0:
                return
            self._map_cloud_pub.publish(map_cloud)

        def _on_scan(self, msg: LaserScan) -> None:
            if (
                self._prefer_point_cloud_over_scan
                and self._last_point_cloud_wall_time > 0.0
                and time.monotonic() - self._last_point_cloud_wall_time
                <= self._scan_fallback_after_sec
            ):
                return
            points: list[tuple[float, float, float]] = []
            angle = float(msg.angle_min)
            for distance in msg.ranges:
                r = float(distance)
                if math.isfinite(r) and float(msg.range_min) <= r <= float(msg.range_max):
                    x = r * math.cos(angle)
                    y = r * math.sin(angle)
                    points.append(
                        _transform_xyz(
                            (x, y, 0.0),
                            translation=self._lidar_to_body,
                            rotation_xyzw=(0.0, 0.0, 0.0, 1.0),
                        )
                    )
                angle += float(msg.angle_increment)

            header = copy.deepcopy(msg.header)
            header.frame_id = "body"
            registered_cloud = point_cloud2.create_cloud_xyz32(header, points)
            self._registered_cloud_pub.publish(registered_cloud)

            map_cloud = self._body_cloud_to_odom(registered_cloud)
            map_cloud.header.frame_id = "odom"
            self._map_cloud_pub.publish(map_cloud)

        def _transform_cloud(self, msg: PointCloud2, *, target_frame: str) -> PointCloud2:
            if self._raw_lidar_frame in {"body", "base_link"}:
                return self._copy_cloud_with_frame(msg, target_frame)
            fields = {field.name for field in msg.fields}
            if not {"x", "y", "z"} <= fields:
                self.get_logger().warn("raw lidar cloud has no x/y/z fields; publishing empty normalized cloud")
                header = copy.deepcopy(msg.header)
                header.frame_id = target_frame
                return point_cloud2.create_cloud(header, msg.fields, [])
            points = []
            names = [field.name for field in msg.fields]
            for raw in point_cloud2.read_points(msg, field_names=names, skip_nans=True):
                values = list(raw)
                x_idx, y_idx, z_idx = names.index("x"), names.index("y"), names.index("z")
                values[x_idx], values[y_idx], values[z_idx] = _transform_xyz(
                    (float(values[x_idx]), float(values[y_idx]), float(values[z_idx])),
                    translation=self._lidar_to_body,
                    rotation_xyzw=(0.0, 0.0, 0.0, 1.0),
                )
                points.append(values)
            header = copy.deepcopy(msg.header)
            header.frame_id = target_frame
            return point_cloud2.create_cloud(header, msg.fields, points)

        def _body_cloud_to_odom(self, msg: PointCloud2) -> PointCloud2:
            pose = self._pose_for_stamp(_stamp_sec(msg.header.stamp))
            names = [field.name for field in msg.fields]
            if not {"x", "y", "z"} <= set(names):
                return self._copy_cloud_with_frame(msg, "odom")
            points = []
            for raw in point_cloud2.read_points(msg, field_names=names, skip_nans=True):
                values = list(raw)
                x_idx, y_idx, z_idx = names.index("x"), names.index("y"), names.index("z")
                values[x_idx], values[y_idx], values[z_idx] = _transform_xyz(
                    (float(values[x_idx]), float(values[y_idx]), float(values[z_idx])),
                    translation=(pose.x, pose.y, pose.z),
                    rotation_xyzw=(pose.qx, pose.qy, pose.qz, pose.qw),
                )
                points.append(values)
            header = copy.deepcopy(msg.header)
            header.frame_id = "odom"
            return point_cloud2.create_cloud(header, msg.fields, points)

        def _pose_for_stamp(self, stamp_sec: float) -> Pose3:
            if not self._pose_history or stamp_sec <= 0.0:
                return self._latest_body_pose
            return min(
                self._pose_history,
                key=lambda item: abs(item.stamp_sec - stamp_sec),
            ).pose

        def _copy_cloud_with_frame(self, msg: PointCloud2, frame_id: str) -> PointCloud2:
            out = copy.deepcopy(msg)
            out.header.frame_id = frame_id
            return out

        @staticmethod
        def _point_count(msg: PointCloud2) -> int:
            return int(msg.width) * int(msg.height)

        def _on_color(self, msg: Image) -> None:
            out = copy.deepcopy(msg)
            out.header.frame_id = "camera_link"
            self._color_pub.publish(out)

        def _on_depth(self, msg: Image) -> None:
            out = copy.deepcopy(msg)
            out.header.frame_id = "camera_link"
            self._depth_pub.publish(out)

        def _on_info(self, msg: CameraInfo) -> None:
            out = copy.deepcopy(msg)
            out.header.frame_id = "camera_link"
            self._info_pub.publish(out)

    rclpy.init(args=None)
    node = GazeboRuntimeAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
