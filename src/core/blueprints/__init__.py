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
from .stub import stub_blueprint
from .stacks import driver, slam, maps, perception, memory, planner, navigation, safety, gateway

__all__ = [
    "full_stack_blueprint",
    "stub_blueprint",
    "driver", "slam", "maps", "perception", "memory",
    "planner", "navigation", "safety", "gateway",
]
