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
_DEFAULT_PREFIX = "/opt/lingtu/nav/install"

_DDS_ENV = {
    "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp",
}


def _prefix(cfg: RobotConfig) -> str:
    return cfg.raw.get("nav_install", _DEFAULT_PREFIX)


def _exe(cfg: RobotConfig, package: str, binary: str) -> str:
    return os.path.join(_prefix(cfg), "lib", package, binary)


# ---------------------------------------------------------------------------
# Autonomy stack (base_autonomy)
# ---------------------------------------------------------------------------

def terrain_analysis(cfg: Optional[RobotConfig] = None) -> NativeModule:
    """Terrain analysis — ground estimation + obstacle detection."""
    cfg = cfg or get_config()
    return NativeModule(NativeModuleConfig(
        executable=_exe(cfg, "terrain_analysis", "terrainAnalysis"),
        name="terrain_analysis",
        parameters={
            "vehicleHeight": cfg.geometry.vehicle_height,
            "terrainVoxelSize": 0.2,
            "obstacleHeightThre": cfg.safety.obstacle_height_thre,
            "groundHeightThre": cfg.safety.ground_height_thre,
            "checkCollision": True,
        },
        remappings={
            "/Odometry": "/nav/odometry",
            "/cloud_map": "/nav/map_cloud",
            "/map_clearing": "/nav/map_clearing",
            "/terrain_map": "/nav/terrain_map",
        },
        env=_DDS_ENV,
    ))


def terrain_analysis_ext(cfg: Optional[RobotConfig] = None) -> NativeModule:
    """Extended terrain analysis — connectivity + 2.5D height map."""
    cfg = cfg or get_config()
    return NativeModule(NativeModuleConfig(
        executable=_exe(cfg, "terrain_analysis_ext", "terrainAnalysisExt"),
        name="terrain_analysis_ext",
        parameters={
            "scanVoxelSize": 0.05,
            "decayTime": 2.0,
            "noDecayDis": 4.0,
            "clearingDis": 8.0,
            "useSorting": True,
            "quantileZ": 0.25,
            "vehicleHeight": cfg.geometry.vehicle_height,
            "voxelPointUpdateThre": 100,
            "voxelTimeUpdateThre": 2.0,
            "lowerBoundZ": -1.5,
            "upperBoundZ": 1.0,
            "disRatioZ": 0.2,
            "checkTerrainConn": True,
            "terrainUnderVehicle": -0.2,
            "terrainConnThre": 0.5,
            "ceilingFilteringThre": 2.0,
            "localTerrainMapRadius": 4.0,
        },
        remappings={
            "/Odometry": "/nav/odometry",
            "/cloud_map": "/nav/map_cloud",
            "/cloud_clearing": "/nav/cloud_clearing",
            "/terrain_map": "/nav/terrain_map",
            "/terrain_map_ext": "/nav/terrain_map_ext",
        },
        env=_DDS_ENV,
    ))


def local_planner(cfg: Optional[RobotConfig] = None) -> NativeModule:
    """Local planner — obstacle avoidance + path scoring."""
    cfg = cfg or get_config()
    return NativeModule(NativeModuleConfig(
        executable=_exe(cfg, "local_planner", "localPlanner"),
        name="local_planner",
        parameters={
            "vehicleLength": cfg.geometry.vehicle_length,
            "vehicleWidth": cfg.geometry.vehicle_width,
            "sensorOffsetX": cfg.geometry.sensor_offset_x,
            "sensorOffsetY": cfg.geometry.sensor_offset_y,
            "twoWayDrive": True,
            "laserVoxelSize": 0.05,
            "terrainVoxelSize": 0.2,
            "useTerrainAnalysis": True,
            "checkObstacle": True,
            "checkRotObstacle": False,
            "adjacentRange": 3.5,
            "obstacleHeightThre": cfg.safety.obstacle_height_thre,
            "groundHeightThre": cfg.safety.ground_height_thre,
            "maxSpeed": cfg.speed.max_speed,
            "autonomyMode": True,
            "autonomySpeed": cfg.speed.autonomy_speed,
            "slopeWeight": 0.0,
            "pathScale": 1.0,
            "minPathScale": 0.75,
            "pathScaleStep": 0.25,
            "pathScaleBySpeed": True,
            "minPathRange": 1.0,
            "pathRangeStep": 0.5,
            "pathRangeBySpeed": True,
            "pathCropByGoal": True,
            "goalClearRange": 0.5,
        },
        remappings={
            "/Odometry": "/nav/odometry",
            "/cloud_map": "/nav/map_cloud",
            "/terrain_map": "/nav/terrain_map",
            "/terrain_map_ext": "/nav/terrain_map_ext",
            "/way_point": "/nav/way_point",
            "/speed": "/nav/speed",
            "/path": "/nav/local_path",
            "/stop": "/nav/stop",
            "/slow_down": "/nav/slow_down",
            "/navigation_boundary": "/nav/navigation_boundary",
            "/added_obstacles": "/nav/added_obstacles",
            "/check_obstacle": "/nav/check_obstacle",
        },
        env=_DDS_ENV,
    ))


def path_follower(cfg: Optional[RobotConfig] = None) -> NativeModule:
    """Path follower — Pure Pursuit waypoint tracking + cmd_vel output."""
    cfg = cfg or get_config()
    return NativeModule(NativeModuleConfig(
        executable=_exe(cfg, "local_planner", "pathFollower"),
        name="path_follower",
        parameters={
            "maxSpeed": cfg.speed.max_speed,
            "autonomyMode": True,
            "autonomySpeed": cfg.speed.autonomy_speed,
        },
        remappings={
            "/Odometry": "/nav/odometry",
            "/path": "/nav/local_path",
            "/cmd_vel": "/nav/cmd_vel",
            "/planner_status": "/nav/planner_status",
        },
        env=_DDS_ENV,
    ))


# ---------------------------------------------------------------------------
# SLAM
# ---------------------------------------------------------------------------

def slam_fastlio2(cfg: Optional[RobotConfig] = None) -> NativeModule:
    """Fast-LIO2 SLAM — LiDAR-inertial odometry + mapping."""
    cfg = cfg or get_config()
    config_path = cfg.raw.get("slam", {}).get(
        "fastlio2_config",
        os.path.join(_prefix(cfg), "share", "fastlio2", "config", "lio.yaml"),
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


def slam_pointlio(cfg: Optional[RobotConfig] = None) -> NativeModule:
    """Point-LIO SLAM — alternative SLAM profile."""
    cfg = cfg or get_config()
    config_path = cfg.raw.get("slam", {}).get(
        "pointlio_config",
        os.path.join(_prefix(cfg), "share", "pointlio", "config", "pointlio.yaml"),
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
