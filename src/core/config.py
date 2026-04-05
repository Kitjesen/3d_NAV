"""Unified configuration loader.

Reads config/robot_config.yaml as single source of truth.
Modules access config via: cfg = load_config(); cfg.speed.max_speed

Environment variable LINGTU_CONFIG_PATH overrides the default config path.
"""

import os
import yaml
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Optional

_REPO_ROOT = Path(__file__).resolve().parent.parent.parent
_DEFAULT_CONFIG = _REPO_ROOT / "config" / "robot_config.yaml"


@dataclass
class SpeedConfig:
    """Velocity limits for the robot."""
    max_linear: float = 1.0
    max_angular: float = 1.0
    max_speed: float = 0.875
    autonomy_speed: float = 0.875


@dataclass
class GeometryConfig:
    """Physical dimensions and sensor offsets."""
    vehicle_height: float = 0.5
    vehicle_width: float = 0.6
    vehicle_length: float = 1.0
    sensor_offset_x: float = 0.3
    sensor_offset_y: float = 0.0


@dataclass
class DriverConfig:
    """Brainstem CMS connection and control parameters."""
    dog_host: str = "127.0.0.1"
    dog_port: int = 13145
    control_rate: float = 50.0
    auto_enable: bool = True
    auto_standup: bool = True
    reconnect_interval: float = 3.0
    slam_reset_interval: float = 5.0


@dataclass
class SafetyConfig:
    """Safety thresholds and timeouts."""
    obstacle_height_thre: float = 0.2
    ground_height_thre: float = 0.1
    stop_distance: float = 0.8
    slow_distance: float = 2.0
    deadman_timeout_ms: float = 300.0
    cmd_vel_timeout_ms: float = 200.0
    tilt_limit_deg: float = 30.0


@dataclass
class CameraConfig:
    """Camera mounting extrinsics and default intrinsics.

    Extrinsics define the body→camera static transform (position + rotation).
    Intrinsics are factory defaults; runtime CameraInfo from ROS2 overrides them.
    Distortion uses Brown-Conrady (plumb_bob) model: k1, k2, p1, p2, k3.
    """
    # Extrinsics: camera position in body frame (m)
    position_x: float = 0.15
    position_y: float = 0.0
    position_z: float = 0.45
    # Extrinsics: camera rotation relative to body forward (rad)
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    # Default intrinsics (overridden by ROS2 CameraInfo at runtime)
    fx: float = 615.0
    fy: float = 615.0
    cx: float = 320.0
    cy: float = 240.0
    width: int = 640
    height: int = 480
    depth_scale: float = 0.001
    # Distortion coefficients (Brown-Conrady / plumb_bob)
    dist_k1: float = 0.0
    dist_k2: float = 0.0
    dist_p1: float = 0.0
    dist_p2: float = 0.0
    dist_k3: float = 0.0

    @property
    def T_body_camera(self) -> 'np.ndarray':
        """4x4 body→camera static transform from factory calibration.

        Uses cv2.Rodrigues for rotation (falls back to numpy if cv2 unavailable).
        Rotation is specified as ZYX Euler angles (yaw, pitch, roll) in radians.
        """
        import numpy as np
        rvec = np.array([self.roll, self.pitch, self.yaw], dtype=np.float64)
        try:
            import cv2
            R, _ = cv2.Rodrigues(rvec)
        except ImportError:
            # Fallback: if angles are all zero, R is identity (common case)
            if np.allclose(rvec, 0):
                R = np.eye(3)
            else:
                # Rodrigues formula: R = I + sin(θ)·K + (1-cos(θ))·K²
                theta = np.linalg.norm(rvec)
                k = rvec / theta
                K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
                R = np.eye(3) + np.sin(theta) * K + (1 - np.cos(theta)) * (K @ K)
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [self.position_x, self.position_y, self.position_z]
        return T


@dataclass
class LidarConfig:
    """LiDAR sensor mounting extrinsics and publish settings.

    offset_x/y/z + roll/pitch/yaw define the body→lidar static transform
    used by static_transform_publisher and C++ coordinate transforms.
    """
    frame_id: str = "livox_frame"
    publish_freq: float = 10.0
    # Network (Livox MID-360). These are consumed by the Livox driver JSON generator.
    lidar_ip: str = "192.168.1.115"
    host_ip: str = "192.168.1.5"
    offset_x: float = -0.011
    offset_y: float = -0.02329
    offset_z: float = 0.04412
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    camera_offset_z: float = 0.0


@dataclass
class RobotConfig:
    """Top-level robot configuration, mirroring config/robot_config.yaml.

    Sub-configs hold typed fields for the most commonly accessed sections.
    The full parsed YAML dict is available via ``raw`` for sections not
    explicitly modelled (e.g. terrain, local_planner, pct_planner).
    """
    speed: SpeedConfig = field(default_factory=SpeedConfig)
    geometry: GeometryConfig = field(default_factory=GeometryConfig)
    driver: DriverConfig = field(default_factory=DriverConfig)
    safety: SafetyConfig = field(default_factory=SafetyConfig)
    lidar: LidarConfig = field(default_factory=LidarConfig)
    camera: CameraConfig = field(default_factory=CameraConfig)
    raw: Dict[str, Any] = field(default_factory=dict)


def _fill_dataclass(cls, data: Dict[str, Any]):
    """Instantiate a dataclass, ignoring keys not present in its fields."""
    field_names = {f.name for f in cls.__dataclass_fields__.values()}
    filtered = {k: v for k, v in data.items() if k in field_names}
    return cls(**filtered)


def load_config(path: Optional[str] = None) -> RobotConfig:
    """Load robot config from YAML.

    Resolution order for the config file path:
    1. Explicit ``path`` argument.
    2. ``LINGTU_CONFIG_PATH`` environment variable.
    3. ``config/robot_config.yaml`` relative to repo root.

    Returns a ``RobotConfig`` with defaults if the file is missing.
    """
    config_path = path or os.environ.get("LINGTU_CONFIG_PATH") or str(_DEFAULT_CONFIG)

    try:
        with open(config_path, "r", encoding="utf-8") as fh:
            raw = yaml.safe_load(fh) or {}
    except (FileNotFoundError, OSError):
        raw = {}

    return RobotConfig(
        speed=_fill_dataclass(SpeedConfig, raw.get("speed", {})),
        geometry=_fill_dataclass(GeometryConfig, raw.get("geometry", {})),
        driver=_fill_dataclass(DriverConfig, raw.get("driver", {})),
        safety=_fill_dataclass(SafetyConfig, raw.get("safety", {})),
        lidar=_fill_dataclass(LidarConfig, raw.get("lidar", {})),
        camera=_fill_dataclass(CameraConfig, raw.get("camera", {})),
        raw=raw,
    )


# ---------------------------------------------------------------------------
# Singleton accessor
# ---------------------------------------------------------------------------

_config: Optional[RobotConfig] = None


def get_config() -> RobotConfig:
    """Return the singleton ``RobotConfig``, loading on first call."""
    global _config
    if _config is None:
        _config = load_config()
    return _config


def reset_config() -> None:
    """Clear the singleton so the next ``get_config()`` reloads from disk.

    Primarily useful for tests.
    """
    global _config
    _config = None
