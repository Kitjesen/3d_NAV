"""lingtu.core.spec -- Protocol interfaces for all module categories.

Import any protocol directly::

    from core.spec import Detector, GoalResolver, LLMBackend

All protocols use ``typing.Protocol`` with ``@runtime_checkable`` so that
``isinstance()`` checks work at runtime without requiring explicit
inheritance.
"""

from .perception import Detector, Encoder, Tracker
from .planning import GlobalPlanner, GoalResolver, TaskDecomposer
from .driver import RobotDriver
from .memory import EpisodicMemoryLike, SpatialMemory
from .safety import SafetyChecker
from .llm import LLMBackend
from .nav import NavigationStack, VoxelMapper, CostMapper, LocalPlanner

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
