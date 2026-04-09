"""Map stack: OccupancyGrid + ESDF + ElevationMap + MapManagerModule."""

from __future__ import annotations

import logging
import os

from core.blueprint import Blueprint

logger = logging.getLogger(__name__)


def maps(**config) -> Blueprint:
    """Real-time map layers from LiDAR point cloud, plus map lifecycle management.

    Params sourced from robot_config.yaml (occupancy_grid / voxel_grid sections).
    MapManagerModule is always included — it handles list/save/use/delete/build
    commands arriving via its map_command In port.
    """
    bp = Blueprint()
    try:
        from core.config import get_config
        from nav.elevation_map_module import ElevationMapModule
        from nav.esdf_module import ESDFModule
        from nav.occupancy_grid_module import OccupancyGridModule
        cfg = get_config()
        og = cfg.raw.get("occupancy_grid", {})

        bp.add(OccupancyGridModule,
               resolution=config.get("grid_resolution", og.get("resolution", 0.2)),
               map_radius=config.get("grid_radius", og.get("map_radius", 30.0)),
               inflation_radius=config.get("inflation_radius", og.get("inflation_radius", 0.5)),
               z_min=og.get("z_min", 0.10),
               z_max=og.get("z_max", 2.00),
               publish_hz=og.get("publish_hz", 2.0))

        from nav.voxel_grid_module import VoxelGridModule
        vg = cfg.raw.get("voxel_grid", {})
        bp.add(VoxelGridModule,
               voxel_size=config.get("voxel_size", vg.get("voxel_size", 0.05)),
               max_range=config.get("voxel_max_range", vg.get("max_range", 20.0)),
               min_z=vg.get("min_z", -0.5),
               max_z=vg.get("max_z", 3.0),
               decay_rate=vg.get("decay_rate", 0.01),
               publish_interval=vg.get("publish_interval", 2.0))

        bp.add(ESDFModule)
        bp.add(ElevationMapModule,
               resolution=config.get("elev_resolution", og.get("resolution", 0.2)),
               map_radius=config.get("elev_radius", 15.0))
    except ImportError as e:
        logger.warning("Map modules not available: %s", e)

    # MapManagerModule: map lifecycle (list/save/use/build/delete).
    # Always included so the REPL and Gateway can issue map_command messages
    # without needing a direct subprocess call.
    try:
        from nav.services.nav_services.map_manager_module import MapManagerModule
        map_dir = config.get(
            "map_dir",
            os.environ.get(
                "NAV_MAP_DIR",
                os.path.expanduser("~/data/inovxio/data/maps"),
            ),
        )
        bp.add(MapManagerModule, map_dir=map_dir)
    except ImportError as e:
        logger.warning("MapManagerModule not available: %s", e)

    return bp
