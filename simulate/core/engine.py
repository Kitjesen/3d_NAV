"""仿真引擎抽象基类 — 引擎无关的统一接口"""
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

import numpy as np


@dataclass
class RobotState:
    """机器人完整运动状态快照."""

    position: np.ndarray          # [x, y, z] 世界坐标系
    orientation: np.ndarray       # [x, y, z, w] 四元数 (ROS 顺序)
    linear_velocity: np.ndarray   # [vx, vy, vz] 世界坐标系
    angular_velocity: np.ndarray  # [wx, wy, wz] 世界坐标系
    joint_positions: np.ndarray   # (16,) 腿关节位置 rad，MuJoCo 顺序
    joint_velocities: np.ndarray  # (16,) 腿关节速度 rad/s
    imu_gyro: np.ndarray          # (3,) 机体坐标系陀螺仪 rad/s
    imu_projected_gravity: np.ndarray  # (3,) 机体坐标系重力投影（归一化）

    def __post_init__(self):
        self.position = np.asarray(self.position, dtype=np.float64)
        self.orientation = np.asarray(self.orientation, dtype=np.float64)
        self.linear_velocity = np.asarray(self.linear_velocity, dtype=np.float64)
        self.angular_velocity = np.asarray(self.angular_velocity, dtype=np.float64)
        self.joint_positions = np.asarray(self.joint_positions, dtype=np.float64)
        self.joint_velocities = np.asarray(self.joint_velocities, dtype=np.float64)
        self.imu_gyro = np.asarray(self.imu_gyro, dtype=np.float64)
        self.imu_projected_gravity = np.asarray(self.imu_projected_gravity, dtype=np.float64)


@dataclass
class CameraData:
    """相机帧数据."""

    rgb: np.ndarray       # (H, W, 3) uint8
    depth: np.ndarray     # (H, W) float32 单位：米
    intrinsics: tuple     # (fx, fy, cx, cy)
    timestamp: float = 0.0


@dataclass
class VelocityCommand:
    """速度指令 (对应 ROS TwistStamped)."""

    linear_x: float = 0.0   # 前进速度 m/s
    linear_y: float = 0.0   # 侧移速度 m/s
    angular_z: float = 0.0  # 转向角速度 rad/s


class SimEngine(ABC):
    """仿真引擎抽象基类.

    所有具体引擎 (MuJoCo, Isaac, Gazebo 等) 都实现此接口，
    上层代码只依赖 SimEngine，不感知底层实现。
    """

    def __init__(self) -> None:
        self._running: bool = False
        self._sim_time: float = 0.0

    # ──────────────────────────────────────────────────────────────
    # 生命周期
    # ──────────────────────────────────────────────────────────────

    @abstractmethod
    def load(self, xml_path: str, **kwargs: Any) -> None:
        """加载场景 XML / URDF.

        Args:
            xml_path: 场景描述文件路径
            **kwargs: 引擎特定参数
        """

    @abstractmethod
    def reset(self) -> RobotState:
        """重置仿真到初始状态，返回初始机器人状态."""

    @abstractmethod
    def close(self) -> None:
        """释放仿真资源."""

    # ──────────────────────────────────────────────────────────────
    # 仿真步进
    # ──────────────────────────────────────────────────────────────

    @abstractmethod
    def step(self, cmd: Optional[VelocityCommand] = None) -> RobotState:
        """推进一个控制周期 (physics + policy).

        Args:
            cmd: 速度指令，None 表示保持上一指令

        Returns:
            步进后的机器人状态
        """

    @abstractmethod
    def step_physics(self, n_steps: int = 1) -> None:
        """纯物理步进，不执行策略推理.

        Args:
            n_steps: 步进次数
        """

    # ──────────────────────────────────────────────────────────────
    # 状态读取
    # ──────────────────────────────────────────────────────────────

    @abstractmethod
    def get_robot_state(self) -> RobotState:
        """读取当前机器人状态."""

    @abstractmethod
    def get_camera_data(self, camera_name: str = "front_camera") -> Optional[CameraData]:
        """读取相机帧 (RGB + 深度).

        Args:
            camera_name: MuJoCo XML 中定义的相机名称

        Returns:
            CameraData 或 None（如果相机不存在）
        """

    @abstractmethod
    def get_lidar_points(self) -> np.ndarray:
        """读取当前 LiDAR 扫描点云.

        Returns:
            (N, 3) float32 世界坐标系点云，或 (N, 4) 含 intensity
        """

    # ──────────────────────────────────────────────────────────────
    # 控制接口
    # ──────────────────────────────────────────────────────────────

    @abstractmethod
    def set_cmd_vel(self, cmd: VelocityCommand) -> None:
        """设置速度指令（异步，由 step() 在下一帧应用）."""

    @abstractmethod
    def set_joint_positions(self, positions: np.ndarray) -> None:
        """直接设置关节目标位置 (绕过 ONNX 策略).

        Args:
            positions: (16,) 关节目标位置，MuJoCo 顺序
        """

    # ──────────────────────────────────────────────────────────────
    # 场景操作
    # ──────────────────────────────────────────────────────────────

    @abstractmethod
    def set_robot_pose(self, position: np.ndarray, orientation: np.ndarray) -> None:
        """传送机器人到指定位姿.

        Args:
            position: [x, y, z]
            orientation: [x, y, z, w] 四元数
        """

    @abstractmethod
    def add_obstacle(self, name: str, shape: str, size: List[float],
                     position: List[float], rgba: Optional[List[float]] = None) -> None:
        """运行时动态添加障碍物.

        Args:
            name: 障碍物名称（唯一）
            shape: "box" | "sphere" | "cylinder"
            size: 形状参数（box: [lx,ly,lz], sphere: [r], cylinder: [r,h]）
            position: [x, y, z]
            rgba: 颜色 [r, g, b, a]，默认灰色
        """

    # ──────────────────────────────────────────────────────────────
    # 属性
    # ──────────────────────────────────────────────────────────────

    @property
    def sim_time(self) -> float:
        """仿真时间（秒）."""
        return self._sim_time

    @property
    def is_running(self) -> bool:
        """仿真是否在运行."""
        return self._running

    @property
    @abstractmethod
    def dt(self) -> float:
        """仿真时间步长（秒）."""

    @property
    @abstractmethod
    def control_dt(self) -> float:
        """策略控制周期（秒），通常为 physics_dt × n_substeps."""
