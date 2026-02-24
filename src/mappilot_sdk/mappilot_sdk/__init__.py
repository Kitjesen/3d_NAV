"""
MapPilot SDK — ROS2-free autonomous navigation Python interface.

Installable on Windows/macOS/Linux without ROS2:
    pip install -e src/mappilot_sdk

Core classes:
    NavigationSDK     — Goal resolution + frontier scoring pipeline
    PerceptionSDK     — Scene graph building (offline / test mode)
    NavigationTypes   — Pure dataclasses (no ROS2 deps)
"""

from .navigation import NavigationSDK
from .perception import PerceptionSDK
from .types import (
    GoalResult,
    NavigationCommand,
    SceneGraphInput,
)

__all__ = [
    "NavigationSDK",
    "PerceptionSDK",
    "GoalResult",
    "NavigationCommand",
    "SceneGraphInput",
]

__version__ = "1.5.0"
