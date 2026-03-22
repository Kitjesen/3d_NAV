"""传感器配置 — CameraConfig / LidarConfig / IMUConfig."""
from dataclasses import dataclass, field
from typing import List, Optional, Tuple


@dataclass
class CameraConfig:
    """相机配置（对应 MuJoCo camera 元素）.

    MuJoCo 相机参数参考：
      https://mujoco.readthedocs.io/en/stable/XMLreference.html#body-camera
    """

    name: str = "front_camera"      # MuJoCo XML 中的相机名称
    width: int = 640                # 渲染宽度 px
    height: int = 480               # 渲染高度 px
    fovy: float = 60.0              # 垂直视场角 °（MuJoCo 默认 45°）
    render_depth: bool = True       # 是否渲染深度图
    depth_near: float = 0.1         # 深度最近距离 m
    depth_far: float = 10.0         # 深度最远距离 m
    fps: float = 30.0               # 期望帧率 Hz

    @property
    def intrinsics(self) -> Tuple[float, float, float, float]:
        """计算相机内参 (fx, fy, cx, cy).

        基于 MuJoCo fovy 和分辨率的简化计算（等效于 OpenCV 针孔模型）。
        """
        import math
        fy = self.height / (2.0 * math.tan(math.radians(self.fovy) / 2.0))
        # MuJoCo 相机纵横比由 width/height 决定，fovx 自动匹配
        fovx = 2.0 * math.degrees(math.atan(math.tan(math.radians(self.fovy) / 2.0)
                                             * self.width / self.height))
        fx = self.width / (2.0 * math.tan(math.radians(fovx) / 2.0))
        cx = self.width / 2.0
        cy = self.height / 2.0
        return (fx, fy, cx, cy)


@dataclass
class LidarConfig:
    """LiDAR 配置（Livox MID-360 参数）.

    参考 sim/sensors/livox_mid360.py 的参数定义。
    """

    body_name: str = "lidar_link"      # MuJoCo 挂载体名称
    sensor_name: str = "lidar_mid360"  # ray_caster plugin 传感器名（方案 A）

    # 扫描参数（方案 B fallback）
    n_rays: int = 6400              # 每帧射线数（真实 MID-360 每帧 ~20000）
    vfov_min_deg: float = -7.0      # 垂直视场角下界 °
    vfov_max_deg: float = 52.0      # 垂直视场角上界 °
    range_min: float = 0.10         # 最近有效距离 m
    range_max: float = 70.0         # 最远有效距离 m
    add_noise: bool = True          # 是否添加测距噪声
    noise_std: float = 0.02         # 测距噪声标准差 m
    fps: float = 10.0               # 扫描频率 Hz

    # OmniPerception 真实扫描模式
    mid360_npy_path: Optional[str] = None  # 若不为 None 则用真实扫描模式
    samples_per_frame: int = 20000          # 每帧采样数（真实模式）

    # Geom 过滤（只检测环境 geom，不检测机器人自身）
    geom_group: int = 1             # MuJoCo geomgroup bitmask


@dataclass
class IMUConfig:
    """IMU 配置（从 MuJoCo freejoint 状态提取）.

    与 brainstem ImuService 输出格式一致。
    """

    body_name: str = "base_link"    # IMU 挂载体名称（通常是 base_link）
    gyro_scale: float = 0.25        # 陀螺仪缩放系数（匹配 brainstem）
    freq_hz: float = 200.0          # IMU 输出频率（MuJoCo 中每步都可读取）
    add_noise: bool = False         # 是否添加 IMU 噪声
    gyro_noise_std: float = 0.01    # 陀螺仪噪声 rad/s
    accel_noise_std: float = 0.05   # 加速度计噪声 m/s²
