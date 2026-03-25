"""lingtu.core.msgs — 统一消息类型，所有模块间数据流的契约。"""

from .geometry import Pose, PoseStamped, Quaternion, Twist, Vector3
from .nav import OccupancyGrid, Odometry, Path
from .semantic import (
    Detection3D,
    DialogueState,
    ExecutionEval,
    GoalResult,
    MissionStatus,
    NavigationCommand,
    Relation,
    Region,
    SafetyState,
    SceneGraph,
)

__all__ = [
    # geometry
    "Vector3", "Quaternion", "Pose", "PoseStamped", "Twist",
    # nav
    "Odometry", "Path", "OccupancyGrid",
    # semantic
    "Detection3D", "Relation", "Region", "SceneGraph",
    "GoalResult", "NavigationCommand", "SafetyState", "MissionStatus",
    "ExecutionEval", "DialogueState",
]
