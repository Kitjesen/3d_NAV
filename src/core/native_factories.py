"""Backward-compat re-export shim — do not add new factories here.

Factories have been moved to their owning subsystems:
  base_autonomy.native_factories  — terrain_analysis, local_planner, path_follower
  slam.native_factories           — slam_fastlio2, slam_pgo, slam_localizer, slam_pointlio, livox_driver
  gateway.native_factories        — grpc_gateway
"""

from base_autonomy.native_factories import (  # noqa: F401
    local_planner,
    path_follower,
    terrain_analysis,
    terrain_analysis_ext,
)
from gateway.native_factories import grpc_gateway  # noqa: F401
from slam.native_factories import (  # noqa: F401
    livox_driver,
    slam_fastlio2,
    slam_localizer,
    slam_pgo,
    slam_pointlio,
)
