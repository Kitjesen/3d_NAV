"""Map stack: OccupancyGrid + ESDF + ElevationMap."""

from __future__ import annotations

import logging

from core.blueprint import Blueprint

logger = logging.getLogger(__name__)


def maps(**config) -> Blueprint:
    """Real-time map layers from LiDAR point cloud."""
    bp = Blueprint()
    try:
        from nav.occupancy_grid_module import OccupancyGridModule
        from nav.esdf_module import ESDFModule
        from nav.elevation_map_module import ElevationMapModule

        bp.add(OccupancyGridModule,
               resolution=config.get("grid_resolution", 0.2),
               map_radius=config.get("grid_radius", 30.0),
               inflation_radius=config.get("inflation_radius", 0.5))
        bp.add(ESDFModule)
        bp.add(ElevationMapModule,
               resolution=config.get("elev_resolution", 0.2),
               map_radius=config.get("elev_radius", 15.0))
    except ImportError as e:
        logger.warning("Map modules not available: %s", e)
    return bp
