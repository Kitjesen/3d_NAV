"""core.msgs — unified message types for inter-module data flow.

All Module In/Out ports use these types. No ROS2 dependency.
"""

from .geometry import Pose, PoseStamped, Quaternion, Transform, Twist, TwistStamped, Vector3
from .nav import OccupancyGrid, Odometry, Path
from .robot import BatteryState, FootForces, JointState, RobotState
from .semantic import (
    Detection3D,
    DialogueState,
    ExecutionEval,
    GoalResult,
    MissionStatus,
    NavigationCommand,
    Region,
    Relation,
    SafetyState,
    SceneGraph,
)
from .sensor import CameraIntrinsics, Image, ImageFormat, Imu, PointCloud2, PointField

__all__ = [
    # geometry
    "Vector3", "Quaternion", "Pose", "PoseStamped",
    "Twist", "TwistStamped", "Transform",
    # nav
    "Odometry", "Path", "OccupancyGrid",
    # sensor
    "Image", "ImageFormat", "CameraIntrinsics", "PointCloud2", "PointField", "Imu",
    # robot
    "JointState", "BatteryState", "FootForces", "RobotState",
    # semantic
    "Detection3D", "Relation", "Region", "SceneGraph",
    "GoalResult", "NavigationCommand", "SafetyState", "MissionStatus",
    "ExecutionEval", "DialogueState",
]
