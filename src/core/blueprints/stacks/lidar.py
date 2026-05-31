"""LiDAR stack: Livox MID-360 driver as an independent Module.

Decouples LiDAR from SLAM — LiDAR is a hardware resource that can be
started, monitored, and subscribed to independently.

The native livox_ros_driver2 process is managed by LidarModule:
- Auto-restart on crash (up to 3x)
- Publishes to DDS topics /lidar/scan + /imu/data
- Bridges point cloud + IMU into Module ports for Python consumers

SLAM, terrain analysis, and other consumers subscribe to these DDS
topics independently — no launch file needed.
"""

from __future__ import annotations

import logging

from core.blueprint import Blueprint
from core.blueprints.stacks._registry import stack_module

logger = logging.getLogger(__name__)


def lidar(ip: str | None = None, enabled: bool = True) -> Blueprint:
    """LiDAR driver stack.

    Args:
        ip: LiDAR IP address override (default from robot_config.yaml).
        enabled: Set to False for stub/dev profiles that don't need hardware.

    Returns:
        Blueprint with LidarModule (or empty if disabled).
    """
    bp = Blueprint()

    if not enabled:
        return bp

    try:
        LidarModule = stack_module(
            "lidar",
            "mid360",
            seed_group="lidar",
            fallback="drivers.real.lidar.LidarModule",
        )
    except ImportError as e:
        logger.warning("LiDAR stack: LidarModule not available: %s", e)
        return bp

    kw = {}
    if ip:
        kw["ip"] = ip
    bp.add(LidarModule, alias="LidarModule", **kw)
    logger.info("LiDAR stack: LidarModule added (ip=%s)", ip or "config default")

    return bp
