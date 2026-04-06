"""Full-stack navigation blueprint — composable factory pattern.

Usage::

    # dimos-style one-liner per stack
    system = full_stack_blueprint(robot="thunder", slam_profile="localizer").build()
    system.start()

    # Or compose directly
    from core.blueprints.stacks import *
    system = autoconnect(
        driver("thunder", host="192.168.66.190"),
        slam("localizer"),
        maps(),
        perception("bpu"),
        memory(),
        planner("kimi"),
        navigation("astar"),
        safety(),
        gateway(5050),
    ).build()
"""

from __future__ import annotations

import logging
from typing import Any

from core.blueprint import Blueprint, autoconnect
from core.utils.calibration_check import run_calibration_check

from .stacks import driver, lidar, sim_lidar, slam, maps, perception, memory, navigation, safety, gateway
from .stacks import planner as planner_stack
from .stacks.driver import driver_name
from .stacks.slam import slam_module_name
from .stacks.memory import DEFAULT_SEMANTIC_DIR

logger = logging.getLogger(__name__)


def full_stack_blueprint(
    robot: str = "thunder",
    slam_profile: str = "fastlio2",
    detector: str = "yoloe",
    encoder: str = "mobileclip",
    llm: str = "kimi",
    planner_backend: str = "astar",
    tomogram: str = "",
    gateway_port: int = 5050,
    teleop_port: int = 5050,  # teleop is now on /ws/teleop of the main gateway port
    enable_native: bool = True,
    enable_semantic: bool = True,
    enable_gateway: bool = True,
    enable_teleop: bool = True,
    enable_map_modules: bool = True,
    enable_rerun: bool = False,
    scene_xml: str = "",
    # Legacy alias
    planner: str = "",
    **config: Any,
) -> Blueprint:
    """Build the complete LingTu navigation stack from composable factories.

    Each stack is a factory function returning a Blueprint.
    autoconnect() merges them and auto-wires by (port_name, msg_type).
    """
    # Legacy alias support
    planner_backend = planner or planner_backend
    semantic_save_dir = config.get("semantic_save_dir", DEFAULT_SEMANTIC_DIR)
    _drv = driver_name(robot)

    # ── Startup calibration self-check ─────────────────────────────────
    _needs_camera = enable_semantic or slam_profile not in ("", "none")
    _needs_slam = slam_profile not in ("", "none")
    calib = run_calibration_check(
        require_camera=_needs_camera,
        require_slam=_needs_slam,
    )
    if not calib.ok:
        raise RuntimeError(
            f"Calibration self-check failed ({len(calib.errors)} error(s)): "
            + "; ".join(calib.errors)
        )

    driver_config = dict(config)
    if enable_semantic and _drv == "MujocoDriverModule":
        driver_config.setdefault("enable_camera", True)

    perception_config = dict(config)
    perception_config["_driver_cls_name"] = _drv

    # LiDAR enabled when SLAM needs it (not for stub/dev/sim that don't use real hardware)
    _needs_lidar = slam_profile not in ("", "none", "bridge")
    _lidar_ip = config.get("lidar_ip")

    bp = autoconnect(
        driver(robot, **driver_config),
        lidar(ip=_lidar_ip, enabled=_needs_lidar),
        sim_lidar(scene_xml=scene_xml),
        slam(slam_profile),
        maps(**config)         if enable_map_modules else Blueprint(),
        perception(detector, encoder, **perception_config)
                               if enable_semantic else Blueprint(),
        memory(semantic_save_dir)
                               if enable_semantic else Blueprint(),
        planner_stack(llm, semantic_save_dir)
                               if enable_semantic else Blueprint(),
        navigation(planner_backend, tomogram, enable_native, **config),
        safety(),
        gateway(
            gateway_port,
            teleop_port=teleop_port,
            enable_teleop=enable_teleop,
            enable_rerun=enable_rerun,
        )
                               if enable_gateway else Blueprint(),
    )

    # ── Cross-stack critical wires ──────────────────────────────────────
    _slam = slam_module_name(slam_profile)

    # map_cloud routing — SLAM module publishes map_cloud to map consumers.
    # When SLAM is active, explicitly wire to avoid ambiguity with driver's lidar_cloud.
    # Only wire to modules actually in the blueprint.
    _bp_names = {e.name for e in bp._entries}
    _map_consumers = [
        "OccupancyGridModule", "ElevationMapModule", "TerrainModule",
        "VoxelGridModule", "RerunBridgeModule", "GatewayModule",
    ]
    if _slam:
        for consumer in _map_consumers:
            if consumer in _bp_names:
                bp.wire(_slam, "map_cloud", consumer, "map_cloud")
    elif scene_xml and "SimPointCloudProvider" in _bp_names:
        for consumer in _map_consumers:
            if consumer in _bp_names:
                bp.wire("SimPointCloudProvider", "map_cloud", consumer, "map_cloud")
        # SimPointCloudProvider needs odometry to trigger periodic re-publish
        bp.wire(_drv, "odometry", "SimPointCloudProvider", "odometry")
    else:
        driver_map_port = ""
        if _drv == "ROS2SimDriverModule":
            driver_map_port = "map_cloud"
        elif _drv == "MujocoDriverModule":
            driver_map_port = "lidar_cloud"
        if driver_map_port:
            for consumer in _map_consumers:
                if consumer in _bp_names:
                    bp.wire(_drv, driver_map_port, consumer, "map_cloud")

    # depth_image routing — Driver or CameraBridge provides depth.
    if enable_semantic:
        _camera_src = "CameraBridgeModule" if "CameraBridgeModule" in _bp_names else _drv
        _color_out = "color_image" if _camera_src == "CameraBridgeModule" else "camera_image"
        for consumer in ["PerceptionModule", "ReconstructionModule", "VisualServoModule"]:
            if consumer in _bp_names and _camera_src in _bp_names:
                bp.wire(_camera_src, _color_out, consumer, "color_image")
                bp.wire(_camera_src, "depth_image", consumer, "depth_image")
                bp.wire(_camera_src, "camera_info", consumer, "camera_info")

    # Safety → all actuators
    bp.wire("SafetyRingModule", "stop_cmd", _drv, "stop_signal")
    bp.wire("SafetyRingModule", "stop_cmd", "NavigationModule", "stop_signal")

    # Helper: only wire if both modules exist in the blueprint
    def _w(out_mod, out_port, in_mod, in_port):
        if out_mod in _bp_names and in_mod in _bp_names:
            bp.wire(out_mod, out_port, in_mod, in_port)

    # Localization health → Safety + Navigation + DepthVisualOdom
    _w("SlamBridgeModule", "localization_status", "SafetyRingModule", "localization_status")
    _w("SlamBridgeModule", "localization_status", "NavigationModule", "localization_status")
    _w("SlamBridgeModule", "localization_status", "DepthVisualOdomModule", "localization_status")

    # Visual odometry fusion: DepthVisualOdom → SlamBridge (selective DOF blend)
    _w("DepthVisualOdomModule", "visual_odometry", "SlamBridgeModule", "visual_odom")

    # Depth camera feed to visual odometry
    _camera_for_vodom = "CameraBridgeModule" if "CameraBridgeModule" in _bp_names else _drv
    _color_port = "color_image" if _camera_for_vodom == "CameraBridgeModule" else "camera_image"
    _w(_camera_for_vodom, _color_port, "DepthVisualOdomModule", "color_image")
    _w(_camera_for_vodom, "depth_image", "DepthVisualOdomModule", "depth_image")
    _w(_camera_for_vodom, "camera_info", "DepthVisualOdomModule", "camera_info")

    # Instruction + goal routing — Gateway/MCP both publish instruction/goal_pose,
    # causing auto_wire ambiguity. Explicitly fan-in both sources to consumers.
    _w("GatewayModule", "instruction", "SemanticPlannerModule", "instruction")
    _w("MCPServerModule", "instruction", "SemanticPlannerModule", "instruction")
    _w("GatewayModule", "instruction", "NavigationModule", "instruction")
    _w("MCPServerModule", "instruction", "NavigationModule", "instruction")
    _w("GatewayModule", "goal_pose", "NavigationModule", "goal_pose")
    _w("MCPServerModule", "goal_pose", "NavigationModule", "goal_pose")
    _w("SemanticPlannerModule", "goal_pose", "NavigationModule", "goal_pose")

    # cmd_vel monitoring — SafetyRing gets mux output (see CmdVelMux wiring below)

    # Odometry routing — prefer SLAM for NavigationModule, always use driver
    # odometry for local modules that need fast feedback in dev/sim.
    _w("PerceptionModule", "detections_3d", "SemanticPlannerModule", "detections")

    _nav_odom_src = _slam if (_slam and _slam in _bp_names) else _drv
    _w(_nav_odom_src, "odometry", "NavigationModule", "odometry")
    _w(_nav_odom_src, "odometry", "PerceptionModule", "odometry")
    for consumer in [
        "OccupancyGridModule", "ElevationMapModule", "TerrainModule",
        "VoxelGridModule", "WavefrontFrontierExplorer", "RerunBridgeModule",
        "LocalPlannerModule", "PathFollowerModule",
        "SemanticMapperModule", "EpisodicMemoryModule", "TaggedLocationsModule",
        "VectorMemoryModule", "TemporalMemoryModule", "MissionLoggerModule",
        "SemanticPlannerModule", "VisualServoModule",
        "ReconstructionModule", "SafetyRingModule", "GeofenceManagerModule",
        "GatewayModule", "MCPServerModule",
    ]:
        _w(_drv, "odometry", consumer, "odometry")

    # Costmap → NavigationModule (live obstacle data for _find_safe_goal BFS)
    _w("OccupancyGridModule", "costmap", "NavigationModule", "costmap")

    # Mission history — NavigationModule state changes go to MissionLoggerModule
    _w("NavigationModule", "mission_status", "MissionLoggerModule", "mission_status")

    # Goal routing from sim driver
    try:
        from core.registry import get as _get_plugin
        if hasattr(_get_plugin("driver", robot), "goal_pose"):
            bp.wire(_drv, "goal_pose", "NavigationModule", "goal_pose")
    except Exception:
        pass

    # Autonomy chain
    _w("NavigationModule", "waypoint", "LocalPlannerModule", "waypoint")
    _w("TerrainModule", "terrain_map", "LocalPlannerModule", "terrain_map")
    _w("LocalPlannerModule", "local_path", "PathFollowerModule", "local_path")
    _w("LocalPlannerModule", "local_path", "SafetyRingModule", "path")

    # Visual servo — dual channel
    _w("SemanticPlannerModule", "servo_target", "VisualServoModule", "servo_target")
    _w("VisualServoModule", "goal_pose", "NavigationModule", "goal_pose")
    _w("VisualServoModule", "nav_stop", "NavigationModule", "stop_signal")

    # Teleop — joystick → TeleopModule → CmdVelMux, active signal → Nav
    _w("TeleopModule", "teleop_active", "NavigationModule", "teleop_active")

    # ── CmdVelMux: priority-based velocity arbitration ──
    # All cmd_vel sources → CmdVelMux (separate input ports per source)
    _w("TeleopModule",      "cmd_vel",          "CmdVelMux", "teleop_cmd_vel")
    _w("VisualServoModule", "cmd_vel",          "CmdVelMux", "visual_servo_cmd_vel")
    _w("NavigationModule",  "recovery_cmd_vel", "CmdVelMux", "recovery_cmd_vel")
    _w("PathFollowerModule", "cmd_vel",         "CmdVelMux", "path_follower_cmd_vel")
    # CmdVelMux → driver (single cmd_vel output)
    bp.wire("CmdVelMux", "driver_cmd_vel", _drv, "cmd_vel")

    # SafetyRing monitors mux output (sees all sources via winner)
    _w("CmdVelMux", "driver_cmd_vel", "SafetyRingModule", "cmd_vel")

    return bp
