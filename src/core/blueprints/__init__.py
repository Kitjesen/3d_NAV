"""lingtu.core.blueprints — composable blueprint factories.

Primary entry point:
    full_stack_blueprint(robot="thunder", slam_profile="fastlio2", ...)

Composable API (dimos-style):
    from core.blueprints.stacks import *
    autoconnect(driver("thunder"), slam("localizer"), navigation("astar"), ...).build()

Legacy:
    stub_blueprint() — CI testing
"""

from .full_stack import full_stack_blueprint
from .stacks import driver, gateway, maps, memory, navigation, perception, planner, safety, slam
from .stub import stub_blueprint

__all__ = [
    "driver",
    "full_stack_blueprint",
    "gateway",
    "maps",
    "memory",
    "navigation",
    "perception",
    "planner",
    "safety",
    "slam",
    "stub_blueprint",
]
