"""SLAM stack: C++ runs as systemd service, Python bridges data via DDS.

Optionally includes DepthVisualOdomModule for degeneracy-resilient fusion.
When SLAM detects corridor/open-field degeneracy (SEVERE/CRITICAL),
visual odometry from the depth camera selectively fuses into degenerate DOFs.

GNSS fusion: when GnssModule is present in the system (from full_stack.py's
_gnss_bp), its ``gnss_odom`` port auto-wires into SlamBridgeModule.gnss_odom
via Blueprint._do_auto_wire (matches by port_name + msg_type). No explicit
wire() call needed — do not add one here unless the port names diverge.
"""

from __future__ import annotations

import logging

from core.blueprint import Blueprint

logger = logging.getLogger(__name__)


def slam(profile: str = "fastlio2", enable_visual_backup: bool = True) -> Blueprint:
    """SLAM / localization stack.

    C++ SLAM always runs as a separate systemd service (correct DDS isolation).
    Python only bridges ROS2 topics into Module ports.

    Args:
        profile: SLAM backend profile
        enable_visual_backup: Add DepthVisualOdomModule for degeneracy fallback

    Profiles:
      "fastlio2"  → start slam + slam_pgo services (mapping mode)
      "localizer" → start slam + localizer services (navigation mode)
      "bridge"    → subscribe only, assume SLAM already running
      "none"/""   → empty (stub/dev mode)

    GNSS fusion config is read from robot_config.yaml gnss.* section and
    passed to SlamBridgeModule as kwargs:
      gnss.antenna_offset.{x,y,z}  → gnss_antenna_offset (lever-arm)
      gnss.fusion.enabled          → gnss_fusion
      gnss.fusion.alpha_{healthy,degraded}  → gnss_alpha_{healthy,degraded}
      gnss.fusion.rtk_float_scale  → gnss_rtk_float_scale
      gnss.fusion.max_{age_s,std_m} → gnss_max_{age_s,std_m}
      gnss.fusion.residual_{warn_m,warn_duration_s,warn_ratio}
                                    → gnss_residual_*
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
        bp.add(SlamBridgeModule, **_read_gnss_fusion_kwargs())
    except ImportError as e:
        logger.warning("SlamBridgeModule not available: %s", e)

    # Depth camera visual odometry for degeneracy fallback
    if enable_visual_backup:
        try:
            from slam.depth_visual_odom_module import DepthVisualOdomModule
            bp.add(DepthVisualOdomModule)
            logger.info("SLAM stack: DepthVisualOdomModule enabled for degeneracy backup")
        except ImportError as e:
            logger.debug("DepthVisualOdomModule not available: %s", e)

    return bp


def slam_module_name(profile: str) -> str:
    """Return the Module class name for SLAM data wiring.

    Always SlamBridgeModule — it has the Out ports.
    """
    if not profile or profile == "none":
        return ""
    return "SlamBridgeModule"


def _read_gnss_fusion_kwargs() -> dict:
    """Load SlamBridgeModule GNSS fusion kwargs from the typed GnssConfig.

    Returns an empty dict on any failure so SlamBridgeModule falls back to its
    own hardcoded defaults — never blocks SLAM startup because of config quirks.
    """
    try:
        from core.config import get_config
        gnss = get_config().gnss
    except Exception as e:
        logger.debug("GNSS fusion config unavailable: %s", e)
        return {}

    ant = gnss.antenna_offset
    fusion = gnss.fusion
    return {
        "gnss_antenna_offset": (float(ant.x), float(ant.y), float(ant.z)),
        "gnss_fusion":                   bool(fusion.enabled),
        "gnss_alpha_healthy":            float(fusion.alpha_healthy),
        "gnss_alpha_degraded":           float(fusion.alpha_degraded),
        "gnss_rtk_float_scale":          float(fusion.rtk_float_scale),
        "gnss_max_age_s":                float(fusion.max_age_s),
        "gnss_max_std_m":                float(fusion.max_std_m),
        "gnss_residual_warn_m":          float(fusion.residual_warn_m),
        "gnss_residual_warn_duration_s": float(fusion.residual_warn_duration_s),
        "gnss_residual_warn_ratio":      float(fusion.residual_warn_ratio),
    }
