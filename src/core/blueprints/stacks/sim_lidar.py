"""Simulated LiDAR stack: generate PointCloud2 from MuJoCo XML scene geometry.

Replaces real LiDAR + SLAM for pure-Python simulation.  The output
``map_cloud`` is identical to what SLAMModule publishes, so
OccupancyGridModule and the rest of the navigation pipeline work unchanged.
"""

from __future__ import annotations

import logging

from core.blueprint import Blueprint

logger = logging.getLogger(__name__)


def sim_lidar(scene_xml: str = "", **config) -> Blueprint:
    """Simulated LiDAR from MuJoCo XML scene.

    Args:
        scene_xml: Path to MuJoCo XML world file.

    Returns:
        Blueprint with SimPointCloudProvider (or empty if no scene).
    """
    bp = Blueprint()

    if not scene_xml:
        return bp

    from drivers.sim.sim_pointcloud_provider import SimPointCloudProvider
    bp.add(SimPointCloudProvider, scene_xml=scene_xml, **config)
    logger.info("sim_lidar stack: SimPointCloudProvider(%s)", scene_xml)

    return bp
