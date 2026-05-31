"""Backward-compat re-export shim — do not add new factories here.

Factories have been moved to their owning subsystems:
  base_autonomy.native_factories  — terrain_analysis, local_planner, path_follower
  slam.native_factories           — slam_fastlio2, slam_pgo, slam_localizer, slam_pointlio, livox_driver
  gateway.native_factories        — grpc_gateway
"""

