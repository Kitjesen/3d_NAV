"""ROS2SimDriverModule — bridges ROS2 topics from mujoco_ros2_bridge to Module ports.

Subscribes to ROS2: /nav/odometry, /nav/registered_cloud, /camera/color/image_raw
Publishes to ROS2:  /nav/cmd_vel (TwistStamped)

Module ports are identical to MujocoDriverModule for drop-in replacement.

Usage::

    from drivers.sim.ros2_sim_driver import ROS2SimDriverModule
    bp.add(ROS2SimDriverModule,
           node_name="lingtu_ros2_driver",
           spin_rate=100.0)
"""

from __future__ import annotations

import logging
import struct
import threading
import time
from typing import Any, Dict, Optional

import numpy as np

from core.module import Module
from core.stream import In, Out
from core.msgs.geometry import Twist, Vector3, Pose, Quaternion, PoseStamped
from core.msgs.nav import Odometry
from core.msgs.sensor import PointCloud2, Image, ImageFormat, CameraIntrinsics
from core.registry import register

logger = logging.getLogger(__name__)

# PointCloud2 point step when the cloud is XYZI (x,y,z,intensity each float32)
_XYZI_POINT_STEP = 16  # 4 × float32


@register("driver", "sim_ros2", description="ROS2 bridge driver for MuJoCo simulation")
class ROS2SimDriverModule(Module, layer=1):
    """Bridges ROS2 topics from mujoco_ros2_bridge to Module ports.

    Subscribes to ROS2:
      /nav/odometry          — nav_msgs/Odometry   @ 50 Hz
      /nav/registered_cloud  — sensor_msgs/PointCloud2 (XYZI) @ 10 Hz
      /camera/color/image_raw — sensor_msgs/Image (optional, BGR8)

    Publishes to ROS2:
      /nav/cmd_vel  — geometry_msgs/TwistStamped

    Module ports mirror MujocoDriverModule for drop-in replacement.
    """

    # -- Inputs --
    cmd_vel: In[Twist]
    stop_signal: In[int]

    # -- Outputs --
    odometry: Out[Odometry]
    lidar_cloud: Out[PointCloud2]
    map_cloud: Out[PointCloud2]   # alias for lidar_cloud → feeds OccupancyGrid/Terrain
    camera_image: Out[Image]
    depth_image: Out[Image]
    camera_info: Out[CameraIntrinsics]
    goal_pose: Out[PoseStamped]
    alive: Out[bool]

    def __init__(
        self,
        node_name: str = "lingtu_ros2_driver",
        spin_rate: float = 100.0,
        odom_topic: str = "/nav/odometry",
        cloud_topic: str = "/nav/registered_cloud",
        image_topic: str = "/camera/color/image_raw",
        depth_topic: str = "/camera/depth/image_raw",
        info_topic: str = "/camera/color/camera_info",
        cmd_vel_topic: str = "/nav/cmd_vel",
        qos_depth: int = 10,
        **kw,
    ):
        super().__init__(**kw)
        self._node_name = node_name
        self._spin_rate = spin_rate
        self._odom_topic = odom_topic
        self._cloud_topic = cloud_topic
        self._image_topic = image_topic
        self._depth_topic = depth_topic
        self._info_topic = info_topic
        self._cmd_vel_topic = cmd_vel_topic
        self._qos_depth = qos_depth

        self._node = None
        self._pub_cmd_vel = None
        self._spin_thread: Optional[threading.Thread] = None
        self._running = False
        self._stopped = False

        # Latest command, written by Module cmd_vel callback, read by spin thread
        self._cmd_vx = 0.0
        self._cmd_vy = 0.0
        self._cmd_wz = 0.0

    def setup(self):
        self.cmd_vel.subscribe(self._on_cmd_vel)
        self.stop_signal.subscribe(self._on_stop)

        try:
            import rclpy
            from rclpy.node import Node
            from rclpy.qos import QoSProfile, ReliabilityPolicy
            from nav_msgs.msg import Odometry as ROS2Odom
            from sensor_msgs.msg import PointCloud2, Image as ROS2Image, CameraInfo
            from geometry_msgs.msg import TwistStamped

            if not rclpy.ok():
                rclpy.init()

            qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=self._qos_depth)

            self._node = Node(self._node_name)

            # Subscribers
            self._node.create_subscription(
                ROS2Odom, self._odom_topic, self._on_ros2_odom, qos)
            self._node.create_subscription(
                PointCloud2, self._cloud_topic, self._on_ros2_cloud, qos)
            self._node.create_subscription(
                ROS2Image, self._image_topic, self._on_ros2_image, qos)
            self._node.create_subscription(
                ROS2Image, self._depth_topic, self._on_ros2_depth, qos)
            self._node.create_subscription(
                CameraInfo, self._info_topic, self._on_ros2_info, qos)

            # Goal pose subscriber — bridges ROS2 /nav/goal_pose to Module port
            from geometry_msgs.msg import PoseStamped as ROS2PoseStamped
            self._node.create_subscription(
                ROS2PoseStamped, "/nav/goal_pose", self._on_ros2_goal, qos)

            # Publisher
            self._pub_cmd_vel = self._node.create_publisher(
                TwistStamped, self._cmd_vel_topic, qos)

            logger.info(
                "ROS2SimDriverModule: node '%s' created — odom=%s cloud=%s cmd_vel=%s",
                self._node_name, self._odom_topic, self._cloud_topic, self._cmd_vel_topic,
            )

        except ImportError as e:
            logger.error("ROS2SimDriverModule: rclpy not available: %s", e)
        except Exception as e:
            logger.error("ROS2SimDriverModule: setup failed: %s", e)

    def start(self):
        super().start()
        if self._node is None:
            self.alive.publish(False)
            return

        self._running = True
        self._spin_thread = threading.Thread(
            target=self._spin_loop, name="ros2_sim_spin", daemon=True)
        self._spin_thread.start()
        self.alive.publish(True)
        logger.info("ROS2SimDriverModule: spin loop started at %.0f Hz", self._spin_rate)

    def stop(self):
        self._running = False
        if self._spin_thread:
            self._spin_thread.join(timeout=3.0)
            self._spin_thread = None
        if self._node:
            try:
                self._node.destroy_node()
            except Exception:
                pass
            self._node = None
        self.alive.publish(False)
        super().stop()

    # -- ROS2 callbacks (called from spin thread) ----------------------------

    def _on_ros2_odom(self, msg) -> None:
        """Convert ROS2 nav_msgs/Odometry → Module Odometry and publish."""
        odom = Odometry(
            pose=Pose(
                position=Vector3(
                    msg.pose.pose.position.x,
                    msg.pose.pose.position.y,
                    msg.pose.pose.position.z,
                ),
                orientation=Quaternion(
                    msg.pose.pose.orientation.x,
                    msg.pose.pose.orientation.y,
                    msg.pose.pose.orientation.z,
                    msg.pose.pose.orientation.w,
                ),
            ),
            twist=Twist(
                linear=Vector3(
                    msg.twist.twist.linear.x,
                    msg.twist.twist.linear.y,
                    msg.twist.twist.linear.z,
                ),
                angular=Vector3(
                    msg.twist.twist.angular.x,
                    msg.twist.twist.angular.y,
                    msg.twist.twist.angular.z,
                ),
            ),
            ts=time.time(),
            frame_id=msg.header.frame_id or "odom",
        )
        self.odometry.publish(odom)

    def _on_ros2_cloud(self, msg) -> None:
        """Convert ROS2 sensor_msgs/PointCloud2 (XYZI, 16 bytes/pt) → Module PointCloud2."""
        try:
            n_pts = msg.width * msg.height
            if n_pts == 0:
                return

            raw = bytes(msg.data)
            step = msg.point_step

            if step == _XYZI_POINT_STEP:
                # Fast path: XYZI packed as 4×float32 — extract XYZ columns only
                arr = np.frombuffer(raw, dtype=np.float32).reshape(n_pts, 4)
                pts = np.ascontiguousarray(arr[:, :3])
            else:
                # Generic path: parse x/y/z field offsets from PointFields
                offsets = _parse_xyz_offsets(msg.fields)
                if offsets is None:
                    logger.warning("ROS2SimDriverModule: cannot parse PointCloud2 fields")
                    return
                ox, oy, oz = offsets
                pts = np.zeros((n_pts, 3), dtype=np.float32)
                for i in range(n_pts):
                    base = i * step
                    pts[i, 0] = struct.unpack_from("<f", raw, base + ox)[0]
                    pts[i, 1] = struct.unpack_from("<f", raw, base + oy)[0]
                    pts[i, 2] = struct.unpack_from("<f", raw, base + oz)[0]

            # Filter out invalid (NaN/Inf) points
            valid = np.isfinite(pts).all(axis=1)
            pts = pts[valid]

            if pts.shape[0] == 0:
                return

            cloud = PointCloud2(
                points=pts,
                frame_id=msg.header.frame_id or "body",
                ts=time.time(),
            )
            self.lidar_cloud.publish(cloud)
            self.map_cloud.publish(cloud)
        except Exception as e:
            logger.error("ROS2SimDriverModule: cloud conversion error: %s", e)

    def _on_ros2_image(self, msg) -> None:
        """Convert ROS2 sensor_msgs/Image → Module Image."""
        try:
            encoding = getattr(msg, "encoding", "bgr8").lower()
            h, w = msg.height, msg.width
            raw = bytes(msg.data)

            if encoding in ("bgr8", "rgb8"):
                arr = np.frombuffer(raw, dtype=np.uint8).reshape(h, w, 3)
                fmt = ImageFormat.BGR if encoding == "bgr8" else ImageFormat.RGB
            elif encoding in ("mono8", "8uc1"):
                arr = np.frombuffer(raw, dtype=np.uint8).reshape(h, w)
                fmt = ImageFormat.GRAY
            elif encoding in ("32fc1",):
                arr = np.frombuffer(raw, dtype=np.float32).reshape(h, w)
                fmt = ImageFormat.DEPTH_F32
            else:
                # Unknown encoding — treat as raw bytes, best effort
                arr = np.frombuffer(raw, dtype=np.uint8)
                fmt = ImageFormat.GRAY

            self.camera_image.publish(Image(
                data=arr.copy(),
                format=fmt,
                ts=time.time(),
                frame_id=msg.header.frame_id or "camera",
            ))
        except Exception as e:
            logger.error("ROS2SimDriverModule: image conversion error: %s", e)

    def _on_ros2_depth(self, msg) -> None:
        """Convert ROS2 depth image to Module Image."""
        try:
            encoding = getattr(msg, "encoding", "16uc1").lower()
            h, w = msg.height, msg.width
            raw = bytes(msg.data)

            if encoding in ("16uc1",):
                arr = np.frombuffer(raw, dtype=np.uint16).reshape(h, w)
                fmt = ImageFormat.DEPTH_U16
            elif encoding in ("32fc1",):
                arr = np.frombuffer(raw, dtype=np.float32).reshape(h, w)
                fmt = ImageFormat.DEPTH_F32
            else:
                arr = np.frombuffer(raw, dtype=np.uint16).reshape(h, w)
                fmt = ImageFormat.DEPTH_U16

            self.depth_image.publish(Image(
                data=arr.copy(),
                format=fmt,
                ts=time.time(),
                frame_id=msg.header.frame_id or "camera",
            ))
        except Exception as e:
            logger.error("ROS2SimDriverModule: depth conversion error: %s", e)

    def _on_ros2_info(self, msg) -> None:
        """Convert ROS2 CameraInfo to Module CameraIntrinsics."""
        try:
            k = msg.k
            self.camera_info.publish(CameraIntrinsics(
                fx=float(k[0]),
                fy=float(k[4]),
                cx=float(k[2]),
                cy=float(k[5]),
                width=int(msg.width),
                height=int(msg.height),
                depth_scale=1.0,
            ))
        except Exception as e:
            logger.error("ROS2SimDriverModule: camera info conversion error: %s", e)

    def _on_ros2_goal(self, msg) -> None:
        """Convert ROS2 geometry_msgs/PoseStamped → Module PoseStamped → goal_pose port."""
        try:
            p = msg.pose.position
            q = msg.pose.orientation
            self.goal_pose.publish(PoseStamped(
                pose=Pose(
                    position=Vector3(float(p.x), float(p.y), float(p.z)),
                    orientation=Quaternion(float(q.x), float(q.y), float(q.z), float(q.w)),
                ),
                frame_id=msg.header.frame_id or "map",
                ts=time.time(),
            ))
            logger.info("ROS2SimDriverModule: goal received (%.1f, %.1f)", p.x, p.y)
        except Exception as e:
            logger.error("ROS2SimDriverModule: goal conversion error: %s", e)

    # -- Module input callbacks ----------------------------------------------

    def _on_cmd_vel(self, twist: Twist) -> None:
        if self._stopped:
            return
        self._cmd_vx = twist.linear.x if hasattr(twist.linear, "x") else 0.0
        self._cmd_vy = twist.linear.y if hasattr(twist.linear, "y") else 0.0
        self._cmd_wz = twist.angular.z if hasattr(twist.angular, "z") else 0.0
        self._publish_cmd_vel()

    def _on_stop(self, level: int) -> None:
        if level >= 1:
            self._cmd_vx = 0.0
            self._cmd_vy = 0.0
            self._cmd_wz = 0.0
            self._stopped = True
            self._publish_cmd_vel()

    # -- ROS2 publish helpers ------------------------------------------------

    def _publish_cmd_vel(self) -> None:
        """Publish current velocity command to ROS2 /nav/cmd_vel."""
        if self._pub_cmd_vel is None or self._node is None:
            return
        try:
            from geometry_msgs.msg import TwistStamped
            from builtin_interfaces.msg import Time as ROS2Time

            msg = TwistStamped()
            now = time.time()
            msg.header.stamp.sec = int(now)
            msg.header.stamp.nanosec = int((now % 1.0) * 1e9)
            msg.header.frame_id = "body"
            msg.twist.linear.x = self._cmd_vx
            msg.twist.linear.y = self._cmd_vy
            msg.twist.linear.z = 0.0
            msg.twist.angular.x = 0.0
            msg.twist.angular.y = 0.0
            msg.twist.angular.z = self._cmd_wz
            self._pub_cmd_vel.publish(msg)
        except Exception as e:
            logger.warning("ROS2SimDriverModule: cmd_vel publish error: %s", e)

    # -- Spin loop -----------------------------------------------------------

    def _spin_loop(self) -> None:
        """Drive rclpy.spin_once() at spin_rate Hz to deliver ROS2 callbacks."""
        try:
            import rclpy
        except ImportError:
            logger.error("ROS2SimDriverModule: rclpy unavailable in spin loop")
            return

        dt = 1.0 / self._spin_rate
        while self._running and self._node is not None:
            t0 = time.monotonic()
            try:
                rclpy.spin_once(self._node, timeout_sec=0.0)
            except Exception as e:
                logger.error("ROS2SimDriverModule: spin_once error: %s", e)
                break
            elapsed = time.monotonic() - t0
            sleep_time = dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

        logger.info("ROS2SimDriverModule: spin loop ended")

    # -- Health --------------------------------------------------------------

    def health(self) -> Dict[str, Any]:
        info = super().port_summary()
        info["ros2"] = {
            "node": self._node_name,
            "running": self._running,
            "has_node": self._node is not None,
            "spin_rate": self._spin_rate,
            "odom_topic": self._odom_topic,
            "cloud_topic": self._cloud_topic,
            "cmd_vel_topic": self._cmd_vel_topic,
        }
        return info


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _parse_xyz_offsets(fields) -> Optional[tuple]:
    """Extract byte offsets for x, y, z fields from a PointCloud2 field list.

    Returns (offset_x, offset_y, offset_z) or None if any field is missing.
    """
    offsets: Dict[str, int] = {}
    for field in fields:
        name = field.name.lower()
        if name in ("x", "y", "z"):
            offsets[name] = field.offset
    if len(offsets) < 3:
        return None
    return offsets["x"], offsets["y"], offsets["z"]
