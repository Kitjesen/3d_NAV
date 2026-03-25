"""Blueprint factory -- builds navigation stack with pluggable robot driver.

Select driver by name from the registry. All drivers share the same port
interface (cmd_vel In, odometry Out, alive Out) so the rest of the stack
is robot-agnostic.

Usage::

    from core.blueprints.factory import make_blueprint

    handle = make_blueprint(robot="thunder", dog_host="192.168.66.190").build()
    handle = make_blueprint(robot="stub").build()
    handle = make_blueprint(robot="auto").build()
"""

from __future__ import annotations

import logging
import platform
from typing import Any

from core.blueprint import Blueprint
from core.registry import get, list_plugins, auto_select

logger = logging.getLogger(__name__)


def _ensure_drivers_registered():
    """Import driver modules so their @register decorators fire."""
    try:
        import drivers.thunder.han_dog_module  # noqa: F401
    except ImportError:
        pass
    try:
        import core.blueprints.stub  # noqa: F401
    except ImportError:
        pass
    try:
        import core.blueprints.simulation  # noqa: F401
    except ImportError:
        pass


def make_blueprint(robot: str = "auto", **config: Any) -> Blueprint:
    """Build a navigation blueprint with the specified robot driver.

    Uses new module architecture: Driver + NavigationModule + SafetyRingModule.

    Args:
        robot: Driver name from registry ("thunder", "sim_mujoco", "stub",
               "auto" to pick best for current platform).
        **config: Passed to driver and modules.

    Returns:
        Ready-to-build Blueprint.
    """
    _ensure_drivers_registered()

    if robot == "auto":
        arch = platform.machine().lower()
        resolved = auto_select("driver", platform=arch)
        if resolved is None:
            resolved = "stub"
            logger.warning("No driver found for platform %s, falling back to stub", arch)
        logger.info("Auto-selected driver: %s (platform=%s)", resolved, arch)
        robot = resolved

    DriverCls = get("driver", robot)
    available = list_plugins("driver")
    logger.info("Building blueprint: driver=%s (available: %s)", robot, available)

    from nav.navigation_module import NavigationModule
    from nav.safety_ring_module import SafetyRingModule

    bp = Blueprint()

    # Driver — pass only driver-relevant config
    driver_keys = {"dog_host", "dog_port", "sim_host", "sim_port",
                   "max_linear_speed", "max_angular_speed"}
    bp.add(DriverCls, **{k: v for k, v in config.items() if k in driver_keys})

    # Navigation + Safety
    bp.add(NavigationModule, planner=config.get("planner", "astar"))
    bp.add(SafetyRingModule)

    # Cross-name wire
    driver_name = DriverCls.__name__
    bp.wire("SafetyRingModule", "stop_cmd", driver_name, "stop_signal")

    bp.auto_wire()
    return bp
