"""Blueprint factory — builds navigation stack with pluggable robot driver.

Select driver by name from the registry. All drivers share the same port
interface (cmd_vel In, odometry Out, alive Out) so the rest of the stack
is robot-agnostic.

Usage::

    from core.blueprints.factory import make_blueprint

    # Real NOVA quadruped
    handle = make_blueprint(robot="nova_dog", dog_host="192.168.66.190").build()

    # MuJoCo simulation
    handle = make_blueprint(robot="sim_mujoco", sim_host="localhost").build()

    # CI / unit tests
    handle = make_blueprint(robot="stub").build()

    # Auto-select best driver for current platform
    handle = make_blueprint(robot="auto").build()

    # Add a new robot — just register it, no changes here:
    #   @register("driver", "unitree_go2", priority=8)
    #   class UnitreeGo2Module(Module, layer=1):
    #       cmd_vel: In[Twist]
    #       odometry: Out[Odometry]
    #       alive: Out[bool]
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
        import drivers.thunder.han_dog_module  # noqa: F401 — registers ThunderDriver
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


def _get_common_modules():
    """Import and return the non-driver modules shared by all blueprints."""
    from nav.rings.nav_rings.safety_module import SafetyModule
    from nav.rings.nav_rings.evaluator_module import EvaluatorModule
    from nav.rings.nav_rings.dialogue_module import DialogueModule
    from global_planning.pct_adapters.src.path_adapter_module import PathAdapterModule
    from global_planning.pct_adapters.src.mission_arc_module import MissionArcModule
    return SafetyModule, EvaluatorModule, DialogueModule, PathAdapterModule, MissionArcModule


def make_blueprint(robot: str = "auto", **config: Any) -> Blueprint:
    """Build a navigation blueprint with the specified robot driver.

    Args:
        robot: Driver name from registry ("thunder", "sim_mujoco", "stub",
               "auto" to pick best for current platform, or any custom name).
        **config: Passed to driver __init__ (dog_host, sim_host, etc.)
                  and to common modules (max_replan_count, etc.).

    Returns:
        Ready-to-build Blueprint.

    Raises:
        KeyError: If robot name not found in registry.
    """
    _ensure_drivers_registered()

    # Resolve "auto" to best available driver for this platform
    if robot == "auto":
        arch = platform.machine().lower()
        resolved = auto_select("driver", platform=arch)
        if resolved is None:
            resolved = "stub"
            logger.warning("No driver found for platform %s, falling back to stub", arch)
        logger.info("Auto-selected driver: %s (platform=%s)", resolved, arch)
        robot = resolved

    # Get driver class from registry
    DriverCls = get("driver", robot)

    available = list_plugins("driver")
    logger.info("Building blueprint: driver=%s (available: %s)", robot, available)

    # Get common modules
    SafetyModule, EvaluatorModule, DialogueModule, PathAdapterModule, MissionArcModule = (
        _get_common_modules()
    )

    # Build blueprint
    bp = Blueprint()
    bp.add(SafetyModule)
    bp.add(DriverCls, **{k: v for k, v in config.items()
                         if k in ("dog_host", "dog_port", "sim_host", "sim_port",
                                  "max_linear_speed", "max_angular_speed")})
    bp.add(EvaluatorModule)
    bp.add(PathAdapterModule)
    bp.add(MissionArcModule, max_replan_count=config.get("max_replan_count", 3))
    bp.add(DialogueModule)

    # Cross-name wires (stop_cmd -> stop_signal)
    driver_name = DriverCls.__name__
    bp.wire("SafetyModule", "stop_cmd", driver_name, "stop_signal")
    bp.wire("SafetyModule", "stop_cmd", "MissionArcModule", "stop_signal")

    bp.auto_wire()
    return bp
