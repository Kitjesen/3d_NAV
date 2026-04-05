"""NativeModule factories for SLAM and LiDAR driver C++ nodes.

Parameters come from config/robot_config.yaml via RobotConfig.
Config paths for SLAM (fastlio2_config, pgo_config, etc.) can be
overridden in robot_config.yaml under the 'slam' section.

Usage::

    from slam.native_factories import slam_fastlio2, slam_localizer, livox_driver
    from core.config import get_config

    cfg = get_config()
    bp.add(slam_fastlio2(cfg), alias="slam")
    bp.add(livox_driver(cfg),  alias="lidar")
"""

from __future__ import annotations

import os
import platform
from typing import Optional

from core.config import RobotConfig, get_config
from core.native_install import DDS_ENV, exe, share

_IS_AARCH64 = platform.machine() in ("aarch64", "arm64")
from core.native_module import NativeModule, NativeModuleConfig
from core.utils.livox_config import ensure_mid360_config_file


def livox_driver(cfg: Optional[RobotConfig] = None) -> NativeModule:
    """Livox MID-360 ROS2 driver — /lidar/scan (CustomMsg) + /imu/data."""
    cfg = cfg or get_config()
    # Enterprise rule: single source of truth.
    # Generate a fresh driver JSON from config/robot_config.yaml (cfg.lidar) to
    # avoid drift between repo files and deployed machines.
    config_path = ensure_mid360_config_file(cfg)
    return NativeModule(NativeModuleConfig(
        executable=exe(cfg, "livox_ros_driver2", "livox_ros_driver2_node"),
        name="livox_driver",
        parameters={
            "xfer_format":      1,    # 1 = Livox CustomMsg
            "multi_topic":      0,    # trimmed driver: single topic only
            "data_src":         0,    # 0 = LiDAR hardware
            "publish_freq":     cfg.lidar.publish_freq,
            "output_data_type": 0,
            "user_config_path": config_path,
        },
        remappings={
            "/livox/lidar": "/lidar/scan",
            "/livox/imu":   "/imu/data",
        },
        env=DDS_ENV,
        auto_restart=True,
        max_restarts=3,
    ))


def slam_fastlio2(cfg: Optional[RobotConfig] = None) -> NativeModule:
    """Fast-LIO2 — LiDAR-inertial odometry + mapping."""
    cfg = cfg or get_config()
    # Auto-select optimized config on S100P (aarch64 ARM CPU)
    default_yaml = "lio_s100p.yaml" if _IS_AARCH64 else "lio.yaml"
    config_path = cfg.raw.get("slam", {}).get(
        "fastlio2_config",
        share(cfg, "fastlio2", "config", default_yaml),
    )
    return NativeModule(NativeModuleConfig(
        executable=exe(cfg, "fastlio2", "lio_node"),
        name="slam_fastlio2",
        parameters={"config_path": config_path},
        remappings={
            "/cloud_registered": "/nav/registered_cloud",
            "/cloud_map":        "/nav/map_cloud",
            "/Odometry":         "/nav/odometry",
            "/path":             "/lio_path",
            "/imu/data":         "/nav/imu",
            "/lidar/scan":       "/nav/lidar_scan",
            "save_map":          "/nav/save_map",
        },
        env=DDS_ENV,
        auto_restart=True,
        max_restarts=3,
    ))


def slam_pgo(cfg: Optional[RobotConfig] = None) -> NativeModule:
    """PGO node — Pose Graph Optimization for map saving.

    Must run alongside Fast-LIO2; provides /pgo/save_maps service.
    """
    cfg = cfg or get_config()
    config_path = cfg.raw.get("slam", {}).get(
        "pgo_config",
        share(cfg, "pgo", "config", "pgo.yaml"),
    )
    return NativeModule(NativeModuleConfig(
        executable=exe(cfg, "pgo", "pgo_node"),
        name="pgo",
        parameters={"config_path": config_path},
        remappings={
            "/cloud_registered": "/nav/registered_cloud",
            "/Odometry":         "/nav/odometry",
        },
        env=DDS_ENV,
        auto_restart=True,
        max_restarts=3,
    ))


def slam_localizer(cfg: Optional[RobotConfig] = None) -> NativeModule:
    """ICP Localizer — localization against a pre-built PCD map.

    Requires Fast-LIO2 for real-time LiDAR odometry.
    """
    cfg = cfg or get_config()
    slam_cfg = cfg.raw.get("slam", {})
    config_path = slam_cfg.get(
        "localizer_config",
        share(cfg, "localizer", "config", "localizer.yaml"),
    )
    map_path = slam_cfg.get(
        "static_map_path",
        os.path.join(
            os.environ.get("NAV_MAP_DIR", os.path.expanduser("~/data/nova/maps")),
            "active", "map.pcd",
        ),
    )
    return NativeModule(NativeModuleConfig(
        executable=exe(cfg, "localizer", "localizer_node"),
        name="localizer",
        parameters={
            "config_path":     config_path,
            "static_map_path": map_path,
        },
        remappings={
            "/cloud_registered": "/nav/registered_cloud",
            "/Odometry":         "/nav/odometry",
            "map_cloud":         "/nav/map_cloud",
        },
        env=DDS_ENV,
        auto_restart=True,
        max_restarts=3,
    ))


def slam_hba(cfg: Optional[RobotConfig] = None) -> NativeModule:
    """HBA — Hierarchical Bundle Adjustment for map refinement.

    Post-processing step: loads PGO patches + poses, runs multi-iteration BA,
    produces refined poses.  Not a real-time module — start on demand after
    mapping is complete, call refine_map / save_poses services, then stop.
    """
    cfg = cfg or get_config()
    config_path = cfg.raw.get("slam", {}).get(
        "hba_config",
        share(cfg, "hba", "config", "hba.yaml"),
    )
    return NativeModule(NativeModuleConfig(
        executable=exe(cfg, "hba", "hba_node"),
        name="hba",
        parameters={"config_path": config_path},
        env=DDS_ENV,
        auto_restart=False,
        max_restarts=0,
    ))


def slam_pointlio(cfg: Optional[RobotConfig] = None) -> NativeModule:
    """Point-LIO SLAM — alternative to Fast-LIO2."""
    cfg = cfg or get_config()
    config_path = cfg.raw.get("slam", {}).get(
        "pointlio_config",
        share(cfg, "pointlio", "config", "pointlio.yaml"),
    )
    return NativeModule(NativeModuleConfig(
        executable=exe(cfg, "pointlio", "pointlio_node"),
        name="slam_pointlio",
        parameters={"config_path": config_path},
        remappings={
            "/cloud_registered": "/nav/registered_cloud",
            "/cloud_map":        "/nav/map_cloud",
            "/Odometry":         "/nav/odometry",
            "/imu/data":         "/nav/imu",
            "/lidar/scan":       "/nav/lidar_scan",
        },
        env=DDS_ENV,
        auto_restart=True,
        max_restarts=3,
    ))
