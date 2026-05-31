"""sim.engine.core — Simulation platform core abstractions"""
from .engine import SimEngine, RobotState, CameraData, VelocityCommand
from .world import WorldConfig, SimWorld
from .robot import RobotConfig
from .sensor import CameraConfig, LidarConfig, IMUConfig
from .robot_model import (
    RobotModel,
    JointConfig,
    SensorLayout,
    ControllerParams,
    NOVA_DOG_MODEL,
    OMNI_CART_MODEL,
    register_robot,
    get_robot,
    list_robots,
    auto_discover,
)

__all__ = [
    "SimEngine",
    "RobotState",
    "CameraData",
    "VelocityCommand",
    "WorldConfig",
    "SimWorld",
    "RobotConfig",
    "CameraConfig",
    "LidarConfig",
    "IMUConfig",
    "RobotModel",
    "JointConfig",
    "SensorLayout",
    "ControllerParams",
    "NOVA_DOG_MODEL",
    "OMNI_CART_MODEL",
    "register_robot",
    "get_robot",
    "list_robots",
    "auto_discover",
]
