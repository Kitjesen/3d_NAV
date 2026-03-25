"""Robot configuration — RobotConfig."""
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np


@dataclass
class RobotConfig:
    """NOVA Dog robot simulation configuration.

    Single source of truth: physical parameters, initial pose, policy path,
    and joint mapping are all defined here.
    Corresponds to the sim section in config/robot_config.yaml.
    """

    # Model files
    robot_xml: str = ""           # robot.xml path (relative to sim/ or absolute)
    policy_onnx: str = ""         # policy.onnx path

    # Initial pose
    init_position: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.35])
    init_orientation_wxyz: List[float] = field(default_factory=lambda: [1.0, 0.0, 0.0, 0.0])

    # Physical parameters (from global CLAUDE.md memory)
    pd_kp: float = 65.0           # position gain (hip)
    pd_kv: float = 5.0            # velocity gain (damping)

    # Policy parameters (consistent with brainstem StandardObservationBuilder)
    # Extracted from sim/bridge/nova_nav_bridge.py
    action_scale: List[float] = field(default_factory=lambda: [
        0.125, 0.25, 0.25,   # FR: hip, thigh, calf
        0.125, 0.25, 0.25,   # FL
        0.125, 0.25, 0.25,   # RR
        0.125, 0.25, 0.25,   # RL
        5.0, 5.0, 5.0, 5.0  # foot
    ])
    imu_gyro_scale: float = 0.25
    joint_vel_scale: float = 0.05
    policy_freq_hz: float = 50.0  # policy inference frequency
    obs_dim: int = 57
    history_len: int = 5

    # Standing pose (Dart order, use as-is, no sign flip)
    standing_pose: List[float] = field(default_factory=lambda: [
        -0.1, -0.8,  1.8,     # FR: hip, thigh, calf
         0.1,  0.8, -1.8,     # FL
         0.1,  0.8, -1.8,     # RR
        -0.1, -0.8,  1.8,     # RL
         0.0,  0.0,  0.0, 0.0 # foot
    ])

    # Joint names (MuJoCo order)
    leg_joint_names: List[str] = field(default_factory=lambda: [
        'fr_hip_joint', 'fr_thigh_joint', 'fr_calf_joint', 'fr_foot_joint',
        'fl_hip_joint', 'fl_thigh_joint', 'fl_calf_joint', 'fl_foot_joint',
        'rr_hip_joint', 'rr_thigh_joint', 'rr_calf_joint', 'rr_foot_joint',
        'rl_hip_joint', 'rl_thigh_joint', 'rl_calf_joint', 'rl_foot_joint',
    ])

    # Body names
    base_body_name: str = "base_link"
    lidar_body_name: str = "lidar_link"

    # Actuator offset (first 8 actuators are arm; legs start at index 8)
    leg_act_offset: int = 8

    # Joint order mapping (from nova_nav_bridge.py original constants)
    # MuJoCo (4+4+4+4): FR(h,t,c,f), FL(...), RR(...), RL(...)
    # Dart   (3+3+3+3+4): FR(h,t,c), FL(h,t,c), RR, RL, FR_f, FL_f, RR_f, RL_f
    mj_to_dart: List[int] = field(default_factory=lambda: [
        0, 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14, 3, 7, 11, 15
    ])
    dart_to_mj: List[int] = field(default_factory=lambda: [
        0, 1, 2, 12, 3, 4, 5, 13, 6, 7, 8, 14, 9, 10, 11, 15
    ])

    # Velocity command limits
    max_linear_vel: float = 1.0    # m/s
    max_angular_vel: float = 1.0   # rad/s
    cmd_vel_watchdog_sec: float = 0.2  # zero-out timeout

    def resolve_paths(self, base_dir: Optional[str] = None) -> "RobotConfig":
        """Resolve relative paths to absolute paths.

        Args:
            base_dir: base directory; defaults to sim/ directory
        """
        if base_dir is None:
            base_dir = str(Path(__file__).resolve().parent.parent.parent / "sim")

        base = Path(base_dir)
        if self.robot_xml and not Path(self.robot_xml).is_absolute():
            self.robot_xml = str(base / self.robot_xml)
        if self.policy_onnx and not Path(self.policy_onnx).is_absolute():
            self.policy_onnx = str(base / self.policy_onnx)
        return self

    @property
    def standing_pose_array(self) -> np.ndarray:
        return np.array(self.standing_pose, dtype=np.float64)

    @property
    def action_scale_array(self) -> np.ndarray:
        return np.array(self.action_scale, dtype=np.float64)

    @property
    def mj_to_dart_array(self) -> np.ndarray:
        return np.array(self.mj_to_dart, dtype=np.int32)

    @property
    def dart_to_mj_array(self) -> np.ndarray:
        return np.array(self.dart_to_mj, dtype=np.int32)

    @classmethod
    def default_nova_dog(cls) -> "RobotConfig":
        """Return default NOVA Dog config (paths must be resolved at runtime)."""
        cfg = cls()
        cfg.robot_xml = "robot/robot.xml"
        cfg.policy_onnx = "robot/policy.onnx"
        return cfg
