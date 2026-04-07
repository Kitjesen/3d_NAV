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
    "CostMapper",
    # perception
    "Detector",
    "Encoder",
    "EpisodicMemoryLike",
    "GlobalPlanner",
    # planning
    "GoalResolver",
    # llm
    "LLMBackend",
    "LocalPlanner",
    # navigation
    "NavigationStack",
    # driver
    "RobotDriver",
    # safety
    "SafetyChecker",
    # memory
    "SpatialMemory",
    "TaskDecomposer",
    "Tracker",
    "VoxelMapper",
]
