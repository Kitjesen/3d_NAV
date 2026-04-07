"""lingtu.core.spec -- Protocol interfaces for all module categories.

Import any protocol directly::

    from core.spec import Detector, GoalResolver, LLMBackend

All protocols use ``typing.Protocol`` with ``@runtime_checkable`` so that
``isinstance()`` checks work at runtime without requiring explicit
inheritance.
"""

from .driver import RobotDriver
from .llm import LLMBackend
from .memory import EpisodicMemoryLike, SpatialMemory
from .nav import CostMapper, LocalPlanner, NavigationStack, VoxelMapper
from .perception import Detector, Encoder, Tracker
from .planning import GlobalPlanner, GoalResolver, TaskDecomposer
from .safety import SafetyChecker

__all__ = [
    # perception
    "Detector",
    "Encoder",
    "Tracker",
    # planning
    "GoalResolver",
    "GlobalPlanner",
    "TaskDecomposer",
    # driver
    "RobotDriver",
    # memory
    "SpatialMemory",
    "EpisodicMemoryLike",
    # safety
    "SafetyChecker",
    # llm
    "LLMBackend",
    # navigation
    "NavigationStack",
    "VoxelMapper",
    "CostMapper",
    "LocalPlanner",
]
