"""SLAM stack: managed (fastlio2/pointlio/localizer) or bridge."""

from __future__ import annotations

import logging

from core.blueprint import Blueprint

logger = logging.getLogger(__name__)


def slam(profile: str = "fastlio2") -> Blueprint:
    """SLAM / localization stack.

    Profiles:
      "fastlio2"/"pointlio" → SLAMModule manages C++ process
      "localizer"           → SLAMModule manages Fast-LIO2 + ICP localizer
      "bridge"              → SlamBridgeModule subscribes to external ROS2
      "none"/""             → empty (stub/dev mode)
    """
    bp = Blueprint()

    if not profile or profile == "none":
        return bp

    if profile == "bridge":
        try:
            from slam.slam_bridge_module import SlamBridgeModule
            bp.add(SlamBridgeModule)
        except ImportError as e:
            logger.warning("SlamBridgeModule not available: %s", e)
    else:
        # Managed mode: SLAMModule manages C++ processes (lio/pgo/livox),
        # SlamBridgeModule bridges ROS2 topics back into Python Module ports.
        try:
            from slam.slam_module import SLAMModule
            bp.add(SLAMModule, backend=profile)
        except ImportError as e:
            logger.warning("SLAMModule not available: %s", e)
        try:
            from slam.slam_bridge_module import SlamBridgeModule
            bp.add(SlamBridgeModule)
        except ImportError:
            pass

    return bp


def slam_module_name(profile: str) -> str:
    """Return the Module class name for SLAM data wiring (odometry + map_cloud).

    Always SlamBridgeModule — it has the Out ports.
    SLAMModule only manages C++ process lifecycle, no Out ports used.
    """
    if not profile or profile == "none":
        return ""
    return "SlamBridgeModule"
