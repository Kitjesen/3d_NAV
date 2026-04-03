"""Driver stack: robot connection + camera bridge (if needed)."""

from __future__ import annotations

import logging

from core.blueprint import Blueprint
from core.config import get_config
from core.registry import get, auto_select

logger = logging.getLogger(__name__)


def _ensure_drivers_registered():
    for mod in (
        "drivers.thunder.han_dog_module",
        "core.blueprints.stub",
        "drivers.sim.mujoco_driver_module",
        "drivers.sim.ros2_sim_driver",
    ):
        try:
            __import__(mod)
        except ImportError:
            pass


def driver(robot: str = "thunder", **config) -> Blueprint:
    """Driver + optional camera bridge."""
    cfg = get_config()
    bp = Blueprint()

    _ensure_drivers_registered()
    if robot == "auto":
        import platform
        robot = auto_select("driver", platform=platform.machine().lower()) or "stub"

    DriverCls = get("driver", robot)
    driver_config = dict(config)
    driver_config.setdefault("dog_host", cfg.driver.dog_host)
    driver_config.setdefault("dog_port", cfg.driver.dog_port)
    bp.add(DriverCls, **driver_config)

    # Camera bridge moved to perception() stack — only loaded when needed
    return bp


def driver_name(robot: str) -> str:
    """Resolve driver class name for wiring."""
    _ensure_drivers_registered()
    if robot == "auto":
        import platform
        robot = auto_select("driver", platform=platform.machine().lower()) or "stub"
    return get("driver", robot).__name__
