"""SLAM stack: C++ runs as systemd service, Python bridges data via DDS."""

from __future__ import annotations

import logging

from core.blueprint import Blueprint

logger = logging.getLogger(__name__)


def slam(profile: str = "fastlio2") -> Blueprint:
    """SLAM / localization stack.

    C++ SLAM always runs as a separate systemd service (correct DDS isolation).
    Python only bridges ROS2 topics into Module ports.

    Profiles:
      "fastlio2"  → start slam + slam_pgo services (mapping mode)
      "localizer" → start slam + localizer services (navigation mode)
      "bridge"    → subscribe only, assume SLAM already running
      "none"/""   → empty (stub/dev mode)
    """
    bp = Blueprint()

    if not profile or profile == "none":
        return bp

    # Start the right systemd services for this mode
    try:
        from core.service_manager import get_service_manager
        svc = get_service_manager()

        if profile == "fastlio2":
            svc.stop("localizer")  # stop nav mode if running
            svc.ensure("slam", "slam_pgo")
            svc.wait_ready("slam", timeout=10.0)
            logger.info("SLAM mapping services started (slam + pgo)")

        elif profile == "localizer":
            svc.stop("slam_pgo")  # stop map mode if running
            svc.ensure("slam", "localizer")
            svc.wait_ready("slam", "localizer", timeout=10.0)
            logger.info("SLAM localization services started (slam + localizer)")

        elif profile == "bridge":
            pass  # assume SLAM already running

    except Exception as e:
        if profile in ("fastlio2", "localizer"):
            logger.warning(
                "SLAM service manager unavailable (no systemd?): %s. "
                "SLAM C++ nodes will not be started automatically. "
                "On S100P, ensure systemd services are configured. "
                "On dev machines, SLAM is not needed (stub/dev profiles).", e)
        else:
            logger.debug("SLAM service manager: %s", e)

    # Bridge ROS2 topics into Python Module ports
    try:
        from slam.slam_bridge_module import SlamBridgeModule
        bp.add(SlamBridgeModule)
    except ImportError as e:
        logger.warning("SlamBridgeModule not available: %s", e)

    return bp


def slam_module_name(profile: str) -> str:
    """Return the Module class name for SLAM data wiring.

    Always SlamBridgeModule — it has the Out ports.
    """
    if not profile or profile == "none":
        return ""
    return "SlamBridgeModule"
