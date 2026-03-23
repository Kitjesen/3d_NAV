"""
仿真 ROS2 统一桥接

将仿真引擎的数据转为 ROS2 消息发布，
让导航栈不知道自己在仿真中运行。

发布话题 (和真机完全一致):
  /camera/color/image_raw        sensor_msgs/Image
  /camera/color/camera_info      sensor_msgs/CameraInfo
  /camera/depth/image_raw        sensor_msgs/Image
  /nav/odometry                  nav_msgs/Odometry
  /nav/registered_cloud          sensor_msgs/PointCloud2
  /nav/map_cloud                 sensor_msgs/PointCloud2
  TF: map -> odom -> body -> camera_link

订阅话题:
  /nav/cmd_vel                   geometry_msgs/TwistStamped
"""
import threading
import time
from typing import Optional

import numpy as np


class SimROS2Bridge:
    """将仿真引擎数据桥接到 ROS2 话题。

    engine 须提供:
      engine.get_robot_state()   -> RobotState (pos, quat_wxyz, vel, omega)
      engine.get_lidar_points()  -> np.ndarray shape (N,4) xyzi, world frame
      engine.get_camera_data()   -> CameraData (rgb, depth, K)
      engine.apply_velocity(vx, vy, wz) -> None

    transport:
      enable_shm=True 时，相机图像额外通过 SHM 发布 (raw bytes, ~200μs)。
      自定义高速消费者可通过 SHM 读取，ROS2 工具通过 DDS 读取。
    """

    # 默认发布频率 (Hz)
    ODOM_HZ = 50
    CAMERA_HZ = 15
    CLOUD_HZ = 10

    def __init__(self, engine, node_name: str = "sim_ros2_bridge", enable_shm: bool = False):
        self._engine = engine
        self._node_name = node_name
        self._enable_shm = enable_shm
        self._lock = threading.Lock()
        self._running = False

        # cmd_vel 状态
        self._cmd_vx: float = 0.0
        self._cmd_vy: float = 0.0
        self._cmd_wz: float = 0.0
        self._cmd_time: float = 0.0
        self._cmd_watchdog_sec: float = 0.2

        # SHM 快速通道 (延迟初始化)
        self._shm_rgb_pub = None
        self._shm_depth_pub = None

        # ROS2 对象 (延迟初始化)
        self._node = None
        self._odom_pub = None
        self._cloud_pub = None
        self._cloud_body_pub = None
        self._img_pub = None
        self._info_pub = None
        self._depth_pub = None
        self._tf_broadcaster = None
        self._static_tf_broadcaster = None

        # 消息类型引用
        self._Odometry = None
        self._PointCloud2 = None
        self._PointField = None
        self._TransformStamped = None
        self._Image = None
        self._CameraInfo = None

        self._init_ros2()

    # ── 初始化 ────────────────────────────────────────────────────────────────

    def _init_ros2(self):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from nav_msgs.msg import Odometry
        from geometry_msgs.msg import TwistStamped, TransformStamped
        from sensor_msgs.msg import PointCloud2, PointField, Image, CameraInfo
        from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

        if not rclpy.ok():
            rclpy.init()

        self._node = rclpy.create_node(self._node_name)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        qos_tl = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # 发布者
        self._odom_pub = self._node.create_publisher(
            Odometry, "/nav/odometry", qos)
        self._cloud_pub = self._node.create_publisher(
            PointCloud2, "/nav/map_cloud", qos)
        self._cloud_body_pub = self._node.create_publisher(
            PointCloud2, "/nav/registered_cloud", qos)
        self._img_pub = self._node.create_publisher(
            Image, "/camera/color/image_raw", qos)
        self._depth_pub = self._node.create_publisher(
            Image, "/camera/depth/image_raw", qos)
        self._info_pub = self._node.create_publisher(
            CameraInfo, "/camera/color/camera_info", qos_tl)

        # TF 广播
        self._tf_broadcaster = TransformBroadcaster(self._node)
        self._static_tf_broadcaster = StaticTransformBroadcaster(self._node)

        # 订阅者
        self._node.create_subscription(
            TwistStamped, "/nav/cmd_vel", self._cmd_vel_cb, qos)

        # 保存消息类型
        self._Odometry = Odometry
        self._PointCloud2 = PointCloud2
        self._PointField = PointField
        self._TransformStamped = TransformStamped
        self._Image = Image
        self._CameraInfo = CameraInfo

        # 发布静态 TF: map -> odom (仿真中相同)
        self._publish_static_map_odom()

        # SHM 快速通道 (可选, ~200μs vs DDS ~1300μs for 900KB image)
        if self._enable_shm:
            try:
                import sys, os
                _repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
                if _repo_root not in sys.path:
                    sys.path.insert(0, _repo_root)
                from transport.shm_transport import SHMTransport
                from transport.core import TopicConfig
                _shm = SHMTransport()
                self._shm_rgb_pub = _shm.create_publisher(
                    TopicConfig(name="/camera/color/image_raw", buffer_size=4*1024*1024))
                self._shm_depth_pub = _shm.create_publisher(
                    TopicConfig(name="/camera/depth/image_raw", buffer_size=4*1024*1024))
                self._node.get_logger().info("[SimROS2Bridge] SHM fast channel enabled (camera)")
            except Exception as e:
                self._node.get_logger().warning(f"[SimROS2Bridge] SHM init failed: {e}")
                self._enable_shm = False

        self._node.get_logger().info(
            f"[SimROS2Bridge] node '{self._node_name}' started"
            f"{' [+SHM]' if self._enable_shm else ''}")

    def _publish_static_map_odom(self):
        """发布静态 TF: map -> odom (仿真中视为同一坐标系)."""
        tf = self._TransformStamped()
        tf.header.stamp = self._node.get_clock().now().to_msg()
        tf.header.frame_id = "map"
        tf.child_frame_id = "odom"
        tf.transform.rotation.w = 1.0
        self._static_tf_broadcaster.sendTransform([tf])

    # ── 订阅回调 ──────────────────────────────────────────────────────────────

    def _cmd_vel_cb(self, msg: "TwistStamped"):
        with self._lock:
            self._cmd_vx = msg.twist.linear.x
            self._cmd_vy = msg.twist.linear.y
            self._cmd_wz = msg.twist.angular.z
            self._cmd_time = time.time()

    def _watchdog(self):
        """200ms 无 cmd_vel 则归零."""
        if time.time() - self._cmd_time > self._cmd_watchdog_sec:
            with self._lock:
                self._cmd_vx = 0.0
                self._cmd_vy = 0.0
                self._cmd_wz = 0.0

    # ── 发布方法 ──────────────────────────────────────────────────────────────

    def publish_odometry(self):
        """发布 odometry + TF: odom -> body."""
        state = self._engine.get_robot_state()
        if state is None:
            return

        now = self._node.get_clock().now().to_msg()
        pos = state.position          # (3,) xyz
        q = state.quat_wxyz           # (4,) w,x,y,z
        vel = state.linear_velocity   # (3,)
        omega = state.angular_velocity  # (3,)

        # Odometry
        odom = self._Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "odom"
        odom.child_frame_id = "body"
        odom.pose.pose.position.x = float(pos[0])
        odom.pose.pose.position.y = float(pos[1])
        odom.pose.pose.position.z = float(pos[2])
        # ROS 四元数: x,y,z,w
        odom.pose.pose.orientation.x = float(q[1])
        odom.pose.pose.orientation.y = float(q[2])
        odom.pose.pose.orientation.z = float(q[3])
        odom.pose.pose.orientation.w = float(q[0])
        odom.twist.twist.linear.x = float(vel[0])
        odom.twist.twist.linear.y = float(vel[1])
        odom.twist.twist.linear.z = float(vel[2])
        odom.twist.twist.angular.x = float(omega[0])
        odom.twist.twist.angular.y = float(omega[1])
        odom.twist.twist.angular.z = float(omega[2])
        self._odom_pub.publish(odom)

        # TF: odom -> body
        tf = self._TransformStamped()
        tf.header.stamp = now
        tf.header.frame_id = "odom"
        tf.child_frame_id = "body"
        tf.transform.translation.x = float(pos[0])
        tf.transform.translation.y = float(pos[1])
        tf.transform.translation.z = float(pos[2])
        tf.transform.rotation.x = float(q[1])
        tf.transform.rotation.y = float(q[2])
        tf.transform.rotation.z = float(q[3])
        tf.transform.rotation.w = float(q[0])
        self._tf_broadcaster.sendTransform(tf)

    def publish_pointcloud(self):
        """发布 LiDAR 点云到 /nav/map_cloud 和 /nav/registered_cloud."""
        pts = self._engine.get_lidar_points()  # (N,4) xyzi world frame
        if pts is None or len(pts) == 0:
            return

        now = self._node.get_clock().now().to_msg()
        fields = [
            self._PointField(name="x", offset=0,  datatype=7, count=1),
            self._PointField(name="y", offset=4,  datatype=7, count=1),
            self._PointField(name="z", offset=8,  datatype=7, count=1),
            self._PointField(name="intensity", offset=12, datatype=7, count=1),
        ]
        pts_f32 = pts.astype(np.float32)

        # 世界坐标系 -> /nav/map_cloud
        msg = self._PointCloud2()
        msg.header.stamp = now
        msg.header.frame_id = "odom"
        msg.height = 1
        msg.width = len(pts_f32)
        msg.is_dense = False
        msg.is_bigendian = False
        msg.fields = fields
        msg.point_step = 16
        msg.row_step = 16 * len(pts_f32)
        msg.data = pts_f32.tobytes()
        self._cloud_pub.publish(msg)

        # 机体坐标系 -> /nav/registered_cloud
        state = self._engine.get_robot_state()
        if state is not None:
            body_pos = state.position
            q = state.quat_wxyz  # w,x,y,z
            # 四元数 -> 旋转矩阵
            w, x, y, z = q[0], q[1], q[2], q[3]
            R = np.array([
                [1 - 2*(y*y + z*z), 2*(x*y - w*z),     2*(x*z + w*y)],
                [2*(x*y + w*z),     1 - 2*(x*x + z*z), 2*(y*z - w*x)],
                [2*(x*z - w*y),     2*(y*z + w*x),     1 - 2*(x*x + y*y)],
            ], dtype=np.float32)
            pts_body_xyz = (pts_f32[:, :3] - body_pos.astype(np.float32)) @ R
            pts_body = np.hstack([pts_body_xyz, pts_f32[:, 3:4]])

            msg2 = self._PointCloud2()
            msg2.header.stamp = now
            msg2.header.frame_id = "body"
            msg2.height = 1
            msg2.width = len(pts_body)
            msg2.is_dense = False
            msg2.is_bigendian = False
            msg2.fields = fields
            msg2.point_step = 16
            msg2.row_step = 16 * len(pts_body)
            msg2.data = pts_body.tobytes()
            self._cloud_body_pub.publish(msg2)

    def publish_camera(self):
        """发布相机图像和 camera_info."""
        cam = self._engine.get_camera_data()
        if cam is None:
            return

        now = self._node.get_clock().now().to_msg()

        # RGB 图像
        if cam.rgb is not None:
            h, w = cam.rgb.shape[:2]
            img_msg = self._Image()
            img_msg.header.stamp = now
            img_msg.header.frame_id = "camera_link"
            img_msg.height = h
            img_msg.width = w
            img_msg.encoding = "rgb8"
            img_msg.is_bigendian = False
            img_msg.step = w * 3
            rgb_bytes = cam.rgb.tobytes()
            img_msg.data = rgb_bytes
            self._img_pub.publish(img_msg)
            # SHM 并行发布 (raw bytes, ~200μs)
            if self._shm_rgb_pub is not None:
                self._shm_rgb_pub.publish(rgb_bytes)

        # 深度图像
        if cam.depth is not None:
            h, w = cam.depth.shape[:2]
            depth_msg = self._Image()
            depth_msg.header.stamp = now
            depth_msg.header.frame_id = "camera_link"
            depth_msg.height = h
            depth_msg.width = w
            depth_msg.encoding = "32FC1"
            depth_msg.is_bigendian = False
            depth_msg.step = w * 4
            depth_bytes = cam.depth.astype(np.float32).tobytes()
            depth_msg.data = depth_bytes
            self._depth_pub.publish(depth_msg)
            # SHM 并行发布
            if self._shm_depth_pub is not None:
                self._shm_depth_pub.publish(depth_bytes)

        # CameraInfo
        if cam.K is not None:
            h, w = (cam.rgb.shape[:2] if cam.rgb is not None
                    else cam.depth.shape[:2])
            info = self._CameraInfo()
            info.header.stamp = now
            info.header.frame_id = "camera_link"
            info.height = h
            info.width = w
            K = cam.K.flatten().tolist()
            info.k = K
            self._info_pub.publish(info)

    # ── 控制循环 ──────────────────────────────────────────────────────────────

    def spin_once(self):
        """处理一次 ROS2 回调 (非阻塞)."""
        import rclpy
        rclpy.spin_once(self._node, timeout_sec=0.0)

    def apply_control(self):
        """将当前 cmd_vel 下发给仿真引擎 (含看门狗)."""
        self._watchdog()
        with self._lock:
            vx, vy, wz = self._cmd_vx, self._cmd_vy, self._cmd_wz
        self._engine.apply_velocity(vx, vy, wz)

    def run(
        self,
        odom_hz: float = ODOM_HZ,
        camera_hz: float = CAMERA_HZ,
        cloud_hz: float = CLOUD_HZ,
    ):
        """主循环: 以配置频率发布各话题，直到 stop() 被调用。"""
        self._running = True
        odom_dt = 1.0 / odom_hz
        cam_dt = 1.0 / camera_hz
        cloud_dt = 1.0 / cloud_hz

        last_odom = 0.0
        last_cam = 0.0
        last_cloud = 0.0

        while self._running:
            now = time.time()

            self.spin_once()
            self.apply_control()

            if now - last_odom >= odom_dt:
                self.publish_odometry()
                last_odom = now

            if now - last_cam >= cam_dt:
                self.publish_camera()
                last_cam = now

            if now - last_cloud >= cloud_dt:
                self.publish_pointcloud()
                last_cloud = now

            # 短暂休眠避免 CPU 满载 (限速到最快频率)
            time.sleep(min(odom_dt, cam_dt, cloud_dt) * 0.5)

    def stop(self):
        """停止主循环."""
        self._running = False

    def destroy(self):
        """销毁 ROS2 节点."""
        if self._node is not None:
            self._node.destroy_node()
