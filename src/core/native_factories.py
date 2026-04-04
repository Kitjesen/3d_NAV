"""Pre-configured NativeModule factories for all C++ ROS2 nodes.

Each factory reads parameters from RobotConfig (config/robot_config.yaml)
and returns a ready-to-use NativeModule that can be added to a Blueprint.

Usage::

    from core.native_factories import terrain_analysis, slam_fastlio2
    from core.config import get_config

    cfg = get_config()
    bp = Blueprint()
    bp.add(terrain_analysis(cfg), alias="terrain")
    bp.add(slam_fastlio2(cfg), alias="slam")
"""

from __future__ import annotations

import os
from typing import Optional

from .config import RobotConfig, get_config
from .native_module import NativeModule, NativeModuleConfig

# Default install prefix — overridable via RobotConfig.raw["nav_install"]
_DEFAULT_PREFIX = "/opt/nova/lingtu/v1.8.0/install"

_DDS_ENV = {
    "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp",
}


def _prefix(cfg: RobotConfig) -> str:
    return cfg.raw.get("nav_install", _DEFAULT_PREFIX)


def _exe(cfg: RobotConfig, package: str, binary: str) -> str:
    # colcon install layout: prefix/package/lib/package/binary
    colcon_path = os.path.join(_prefix(cfg), package, "lib", package, binary)
    if os.path.exists(colcon_path):
        return colcon_path
    # fallback: prefix/lib/package/binary
    return os.path.join(_prefix(cfg), "lib", package, binary)


def _share(cfg: RobotConfig, package: str, *sub) -> str:
    # colcon: prefix/package/share/package/...
    colcon_path = os.path.join(_prefix(cfg), package, "share", package, *sub)
    if os.path.exists(colcon_path):
        return colcon_path
    # fallback: prefix/share/package/...
    return os.path.join(_prefix(cfg), "share", package, *sub)


# ---------------------------------------------------------------------------
# Autonomy stack (base_autonomy)
# ---------------------------------------------------------------------------

def terrain_analysis(cfg: Optional[RobotConfig] = None) -> NativeModule:
    """Terrain analysis — ground estimation + obstacle detection.

    All parameters sourced from config/robot_config.yaml → terrain section.
    Geometry (vehicleHeight) and safety thresholds come from their own sections.
    """
    cfg = cfg or get_config()
    ta = cfg.raw.get("terrain", {})
    return NativeModule(NativeModuleConfig(
        executable=_exe(cfg, "terrain_analysis", "terrainAnalysis"),
        name="terrain_analysis",
        parameters={
            # geometry
            "vehicleHeight": cfg.geometry.vehicle_height,
            # safety thresholds
            "obstacleHeightThre": cfg.safety.obstacle_height_thre,
            "groundHeightThre":   cfg.safety.ground_height_thre,
            # terrain section
            "terrainVoxelSize":       ta.get("scan_voxel_size",          0.05),
            "useSorting":             ta.get("use_sorting",               True),
            "quantileZ":              ta.get("quantile_z",                0.25),
            "considerDrop":           ta.get("consider_drop",             False),
            "limitGroundLift":        ta.get("limit_ground_lift",         False),
            "maxGroundLift":          ta.get("max_ground_lift",           0.15),
            "clearDyObs":             ta.get("clear_dy_obs",              False),
            "minDyObsDis":            ta.get("min_dy_obs_dis",            0.3),
            "absDyObsRelZThre":       ta.get("abs_dy_obs_rel_z_thre",     0.2),
            "minDyObsVFOV":           ta.get("min_dy_obs_vfov",          -16.0),
            "maxDyObsVFOV":           ta.get("max_dy_obs_vfov",           16.0),
            "minDyObsPointNum":       ta.get("min_dy_obs_point_num",      1),
            "noDataObstacle":         ta.get("no_data_obstacle",          False),
            "noDataBlockSkipNum":     ta.get("no_data_block_skip_num",    0),
            "minBlockPointNum":       ta.get("min_block_point_num",       10),
            "voxelPointUpdateThre":   ta.get("voxel_point_update_thre",   100),
            "voxelTimeUpdateThre":    ta.get("voxel_time_update_thre",    2.0),
            "minRelZ":                ta.get("min_rel_z",                -1.5),
            "maxRelZ":                ta.get("max_rel_z",                 0.2),
            "disRatioZ":              ta.get("dis_ratio_z",               0.2),
            "checkCollision":         ta.get("check_collision",           True),
        },
        remappings={
            "/Odometry":     "/nav/odometry",
            "/cloud_map":    "/nav/map_cloud",
            "/map_clearing": "/nav/map_clearing",
            "/terrain_map":  "/nav/terrain_map",
        },
        env=_DDS_ENV,
    ))


def terrain_analysis_ext(cfg: Optional[RobotConfig] = None) -> NativeModule:
    """Extended terrain analysis — connectivity + 2.5D height map.

    All parameters sourced from config/robot_config.yaml → terrain section.
    """
    cfg = cfg or get_config()
    ta = cfg.raw.get("terrain", {})
    return NativeModule(NativeModuleConfig(
        executable=_exe(cfg, "terrain_analysis_ext", "terrainAnalysisExt"),
        name="terrain_analysis_ext",
        parameters={
            # geometry
            "vehicleHeight": cfg.geometry.vehicle_height,
            # terrain section (ext uses same keys as terrain_analysis where applicable)
            "scanVoxelSize":         ta.get("scan_voxel_size",          0.05),
            "useSorting":            ta.get("use_sorting",               True),
            "quantileZ":             ta.get("quantile_z",                0.25),
            "voxelPointUpdateThre":  ta.get("voxel_point_update_thre",   100),
            "voxelTimeUpdateThre":   ta.get("voxel_time_update_thre",    2.0),
            "lowerBoundZ":           ta.get("min_rel_z",                -1.5),
            "upperBoundZ":           ta.get("max_rel_z",                 0.2),
            "disRatioZ":             ta.get("dis_ratio_z",               0.2),
            "terrainUnderVehicle":   ta.get("terrain_under_vehicle",    -0.2),
            "terrainConnThre":       ta.get("terrain_conn_thre",         0.5),
            "ceilingFilteringThre":  ta.get("ceiling_filtering_thre",    2.0),
            "checkTerrainConn":      ta.get("check_collision",           True),
        },
        remappings={
            "/Odometry":        "/nav/odometry",
            "/cloud_map":       "/nav/map_cloud",
            "/cloud_clearing":  "/nav/cloud_clearing",
            "/terrain_map":     "/nav/terrain_map",
            "/terrain_map_ext": "/nav/terrain_map_ext",
        },
        env=_DDS_ENV,
    ))


def local_planner(cfg: Optional[RobotConfig] = None) -> NativeModule:
    """Local planner — obstacle avoidance + path scoring.

    All parameters sourced from config/robot_config.yaml:
      geometry.*         — vehicle dimensions and sensor offsets
      speed.*            — velocity limits
      safety.*           — obstacle/ground height thresholds
      local_planner.*    — all planning tuning knobs
      autonomy.*         — joy delay timings
    """
    cfg = cfg or get_config()
    lp  = cfg.raw.get("local_planner", {})
    aut = cfg.raw.get("autonomy", {})
    ta  = cfg.raw.get("terrain", {})
    return NativeModule(NativeModuleConfig(
        executable=_exe(cfg, "local_planner", "localPlanner"),
        name="local_planner",
        parameters={
            # geometry
            "vehicleLength":   cfg.geometry.vehicle_length,
            "vehicleWidth":    cfg.geometry.vehicle_width,
            "sensorOffsetX":   cfg.geometry.sensor_offset_x,
            "sensorOffsetY":   cfg.geometry.sensor_offset_y,
            # speed
            "maxSpeed":        cfg.speed.max_speed,
            "autonomySpeed":   cfg.speed.autonomy_speed,
            # safety thresholds
            "obstacleHeightThre": cfg.safety.obstacle_height_thre,
            "groundHeightThre":   cfg.safety.ground_height_thre,
            # local_planner section
            "twoWayDrive":        lp.get("two_way_drive",           True),
            "laserVoxelSize":     lp.get("laser_voxel_size",        0.05),
            "terrainVoxelSize":   ta.get("scan_voxel_size",         0.2),
            "useTerrainAnalysis": True,
            "checkObstacle":      lp.get("check_obstacle",          True),
            "checkRotObstacle":   lp.get("check_rot_obstacle",      False),
            "adjacentRange":      lp.get("path_range_step",         3.5),
            "costHeightThre1":    lp.get("cost_height_thre_1",      0.15),
            "costHeightThre2":    lp.get("cost_height_thre_2",      0.1),
            "useCost":            lp.get("use_cost",                False),
            "slowPathNumThre":    lp.get("slow_path_num_thre",      5),
            "slowGroupNumThre":   lp.get("slow_group_num_thre",     1),
            "pointPerPathThre":   lp.get("point_per_path_thre",     2),
            "dirWeight":          lp.get("dir_weight",              0.02),
            "dirThre":            lp.get("dir_thre",                90.0),
            "dirToVehicle":       lp.get("dir_to_vehicle",          False),
            "minPathRange":       lp.get("min_path_range",          1.0),
            "pathRangeStep":      lp.get("path_range_step",         0.5),
            "pathRangeBySpeed":   lp.get("path_range_by_speed",     True),
            "pathCropByGoal":     lp.get("path_crop_by_goal",       True),
            "freezeAng":          lp.get("freeze_ang",              90.0),
            "freezeTime":         lp.get("freeze_time",             2.0),
            "goalClearRange":     lp.get("goal_clear_range",        0.5),
            "goalBehindRange":    lp.get("goal_behind_range",       0.8),
            # autonomy section
            "autonomyMode":          True,
            "joyToSpeedDelay":       aut.get("joy_to_speed_delay",          2.0),
            "joyToCheckObstacleDelay": aut.get("joy_to_check_obstacle_delay", 5.0),
        },
        remappings={
            "/Odometry":             "/nav/odometry",
            "/cloud_map":            "/nav/map_cloud",
            "/terrain_map":          "/nav/terrain_map",
            "/terrain_map_ext":      "/nav/terrain_map_ext",
            "/way_point":            "/nav/way_point",
            "/speed":                "/nav/speed",
            "/path":                 "/nav/local_path",
            "/stop":                 "/nav/stop",
            "/slow_down":            "/nav/slow_down",
            "/navigation_boundary":  "/nav/navigation_boundary",
            "/added_obstacles":      "/nav/added_obstacles",
            "/check_obstacle":       "/nav/check_obstacle",
        },
        env=_DDS_ENV,
    ))


def path_follower(cfg: Optional[RobotConfig] = None) -> NativeModule:
    """Path follower — Pure Pursuit waypoint tracking + cmd_vel output.

    All parameters sourced from config/robot_config.yaml:
      speed.*         — velocity limits
      control.*       — yaw rate gains
      path_follower.* — lookahead, acceleration, stop/slow thresholds
      autonomy.*      — joy delay timings
    """
    cfg = cfg or get_config()
    pf  = cfg.raw.get("path_follower", {})
    ctl = cfg.raw.get("control", {})
    aut = cfg.raw.get("autonomy", {})
    return NativeModule(NativeModuleConfig(
        executable=_exe(cfg, "local_planner", "pathFollower"),
        name="path_follower",
        parameters={
            # speed
            "maxSpeed":        cfg.speed.max_speed,
            "autonomySpeed":   cfg.speed.autonomy_speed,
            # control section
            "yawRateGain":     ctl.get("yaw_rate_gain",       7.5),
            "stopYawRateGain": ctl.get("stop_yaw_rate_gain",  7.5),
            "maxYawRate":      ctl.get("max_yaw_rate",        45.0),
            "maxAccel":        ctl.get("max_accel",           1.0),
            # path_follower section
            "switchTimeThre":      pf.get("switch_time_thre",      1.0),
            "dirDiffThre":         pf.get("dir_diff_thre",         0.1),
            "pubSkipNum":          pf.get("pub_skip_num",           1),
            "useInclRateToSlow":   pf.get("use_incl_rate_to_slow", False),
            "inclRateThre":        pf.get("incl_rate_thre",        120.0),
            "slowRate1":           pf.get("slow_rate_1",           0.25),
            "slowRate2":           pf.get("slow_rate_2",           0.5),
            "slowRate3":           pf.get("slow_rate_3",           0.75),
            "slowTime1":           pf.get("slow_time_1",           2.0),
            "slowTime2":           pf.get("slow_time_2",           2.0),
            "useInclToStop":       pf.get("use_incl_to_stop",      False),
            "inclThre":            pf.get("incl_thre",             45.0),
            "stopTime":            pf.get("stop_time",             5.0),
            "noRotAtStop":         pf.get("no_rot_at_stop",        False),
            "noRotAtGoal":         pf.get("no_rot_at_goal",        True),
            # autonomy section
            "autonomyMode":        True,
            "joyToSpeedDelay":     aut.get("joy_to_speed_delay", 2.0),
        },
        remappings={
            "/Odometry":       "/nav/odometry",
            "/path":           "/nav/local_path",
            "/cmd_vel":        "/nav/cmd_vel",
            "/planner_status": "/nav/planner_status",
        },
        env=_DDS_ENV,
    ))


# ---------------------------------------------------------------------------
# LiDAR Driver
# ---------------------------------------------------------------------------

def livox_driver(cfg: Optional[RobotConfig] = None) -> NativeModule:
    """Livox MID-360 ROS2 driver — publishes /lidar/scan (CustomMsg) + /imu/data."""
    cfg = cfg or get_config()
    config_path = cfg.raw.get("lidar", {}).get(
        "livox_config",
        _share(cfg, "livox_ros_driver2", "config", "MID360_config.json"),
    )
    return NativeModule(NativeModuleConfig(
        executable=_exe(cfg, "livox_ros_driver2", "livox_ros_driver2_node"),
        name="livox_driver",
        parameters={
            "xfer_format": 1,  # 1=Livox CustomMsg
            "multi_topic": 0,
            "data_src": 0,     # 0=LiDAR hardware
            "publish_freq": 10.0,
            "output_data_type": 0,
            "user_config_path": config_path,
        },
        remappings={
            "/livox/lidar": "/lidar/scan",
            "/livox/imu": "/imu/data",
        },
        env=_DDS_ENV,
        auto_restart=True,
        max_restarts=3,
    ))


# ---------------------------------------------------------------------------
# SLAM
# ---------------------------------------------------------------------------

def slam_fastlio2(cfg: Optional[RobotConfig] = None) -> NativeModule:
    """Fast-LIO2 SLAM — LiDAR-inertial odometry + mapping."""
    cfg = cfg or get_config()
    config_path = cfg.raw.get("slam", {}).get(
        "fastlio2_config",
        _share(cfg, "fastlio2", "config", "lio.yaml"),
    )
    return NativeModule(NativeModuleConfig(
        executable=_exe(cfg, "fastlio2", "lio_node"),
        name="slam_fastlio2",
        parameters={"config_path": config_path},
        remappings={
            "/cloud_registered": "/nav/registered_cloud",
            "/cloud_map": "/nav/map_cloud",
            "/Odometry": "/nav/odometry",
            "/path": "/lio_path",
            "/imu/data": "/nav/imu",
            "/lidar/scan": "/nav/lidar_scan",
            "save_map": "/nav/save_map",
        },
        env=_DDS_ENV,
        auto_restart=True,
        max_restarts=3,
    ))


def slam_pgo(cfg: Optional[RobotConfig] = None) -> NativeModule:
    """PGO node — Pose Graph Optimization for map saving.

    Subscribes to Fast-LIO2 output, provides /pgo/save_maps service.
    Must run alongside Fast-LIO2.
    """
    cfg = cfg or get_config()
    config_path = cfg.raw.get("slam", {}).get(
        "pgo_config",
        _share(cfg, "pgo", "config", "pgo.yaml"),
    )
    return NativeModule(NativeModuleConfig(
        executable=_exe(cfg, "pgo", "pgo_node"),
        name="pgo",
        parameters={"config_path": config_path},
        remappings={
            "/cloud_registered": "/nav/registered_cloud",
            "/Odometry": "/nav/odometry",
        },
        env=_DDS_ENV,
        auto_restart=True,
        max_restarts=3,
    ))


def slam_localizer(cfg: Optional[RobotConfig] = None) -> NativeModule:
    """ICP Localizer — localization against a pre-built map.

    Requires Fast-LIO2 running for real-time LiDAR odometry.
    Localizer does ICP matching against a static PCD map.
    """
    cfg = cfg or get_config()
    config_path = cfg.raw.get("slam", {}).get(
        "localizer_config",
        _share(cfg, "localizer", "config", "localizer.yaml"),
    )
    map_path = cfg.raw.get("slam", {}).get(
        "static_map_path",
        os.path.join(
            os.environ.get("NAV_MAP_DIR", os.path.expanduser("~/data/nova/maps")),
            "active", "map.pcd",
        ),
    )
    return NativeModule(NativeModuleConfig(
        executable=_exe(cfg, "localizer", "localizer_node"),
        name="localizer",
        parameters={
            "config_path": config_path,
            "static_map_path": map_path,
        },
        remappings={
            "/cloud_registered": "/nav/registered_cloud",
            "/Odometry": "/nav/odometry",
            "map_cloud": "/nav/map_cloud",
        },
        env=_DDS_ENV,
        auto_restart=True,
        max_restarts=3,
    ))


def slam_pointlio(cfg: Optional[RobotConfig] = None) -> NativeModule:
    """Point-LIO SLAM — alternative SLAM profile."""
    cfg = cfg or get_config()
    config_path = cfg.raw.get("slam", {}).get(
        "pointlio_config",
        _share(cfg, "pointlio", "config", "pointlio.yaml"),
    )
    return NativeModule(NativeModuleConfig(
        executable=_exe(cfg, "pointlio", "pointlio_node"),
        name="slam_pointlio",
        parameters={"config_path": config_path},
        remappings={
            "/cloud_registered": "/nav/registered_cloud",
            "/cloud_map": "/nav/map_cloud",
            "/Odometry": "/nav/odometry",
            "/imu/data": "/nav/imu",
            "/lidar/scan": "/nav/lidar_scan",
        },
        env=_DDS_ENV,
        auto_restart=True,
        max_restarts=3,
    ))


# ---------------------------------------------------------------------------
# gRPC gateway
# ---------------------------------------------------------------------------

def grpc_gateway(cfg: Optional[RobotConfig] = None) -> NativeModule:
    """gRPC gateway — remote control + telemetry (C++)."""
    cfg = cfg or get_config()
    return NativeModule(NativeModuleConfig(
        executable=_exe(cfg, "remote_monitoring", "grpc_gateway"),
        name="grpc_gateway",
        parameters={
            "grpc_port": cfg.raw.get("grpc", {}).get("port", 50051),
        },
        env=_DDS_ENV,
    ))
