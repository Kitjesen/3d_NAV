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

from .stacks import driver, slam, maps, perception, memory, navigation, safety, gateway
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
    enable_native: bool = True,
    enable_semantic: bool = True,
    enable_gateway: bool = True,
    enable_map_modules: bool = True,
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

    bp = autoconnect(
        driver(robot, **config),
        slam(slam_profile),
        maps(**config)         if enable_map_modules else Blueprint(),
        perception(detector, encoder, **config)
                               if enable_semantic else Blueprint(),
        memory(semantic_save_dir)
                               if enable_semantic else Blueprint(),
        planner_stack(llm, semantic_save_dir)
                               if enable_semantic else Blueprint(),
        navigation(planner_backend, tomogram, enable_native),
        safety(),
        gateway(gateway_port)  if enable_gateway else Blueprint(),
    )

    # ── Cross-stack critical wires ──────────────────────────────────────
    _drv = driver_name(robot)
    _slam = slam_module_name(slam_profile)

    # map_cloud routing — SLAM module publishes map_cloud to map consumers.
    # When SLAM is active, explicitly wire to avoid ambiguity with driver's lidar_cloud.
    if _slam:
        _cloud_consumers = ["OccupancyGridModule", "ElevationMapModule", "TerrainModule"]
        for consumer in _cloud_consumers:
            try:
                bp.wire(_slam, "map_cloud", consumer, "map_cloud")
            except Exception:
                pass

    # depth_image routing — Driver or CameraBridge provides depth.
    if enable_semantic:
        _depth_src = "CameraBridgeModule"
        try:
            from core.registry import get as _get_plugin
            if hasattr(_get_plugin("driver", robot), "depth_image"):
                _depth_src = _drv
        except Exception:
            pass
        for consumer in ["ReconstructionModule", "VisualServoModule"]:
            try:
                bp.wire(_depth_src, "depth_image", consumer, "depth_image")
            except Exception:
                pass

    # Safety → all actuators
    bp.wire("SafetyRingModule", "stop_cmd", _drv, "stop_signal")
    bp.wire("SafetyRingModule", "stop_cmd", "NavigationModule", "stop_signal")

    # Instruction + goal routing — Gateway/MCP both publish instruction/goal_pose,
    # causing auto_wire ambiguity. Explicitly fan-in both sources to consumers.
    if enable_gateway:
        try:
            if enable_semantic:
                bp.wire("GatewayModule", "instruction", "SemanticPlannerModule", "instruction")
                bp.wire("MCPServerModule", "instruction", "SemanticPlannerModule", "instruction")
            bp.wire("GatewayModule", "instruction", "NavigationModule", "instruction")
            bp.wire("MCPServerModule", "instruction", "NavigationModule", "instruction")
            bp.wire("GatewayModule", "goal_pose", "NavigationModule", "goal_pose")
            bp.wire("MCPServerModule", "goal_pose", "NavigationModule", "goal_pose")
        except Exception:
            pass

    # SemanticPlanner goal_pose → Navigation (must be explicit since auto_wire
    # skips In ports that already have explicit wires from Gateway/MCP above)
    if enable_semantic:
        try:
            bp.wire("SemanticPlannerModule", "goal_pose", "NavigationModule", "goal_pose")
        except Exception:
            pass

    # cmd_vel monitoring — SafetyRing needs to see all cmd_vel sources
    try:
        if enable_gateway:
            bp.wire("GatewayModule", "cmd_vel", "SafetyRingModule", "cmd_vel")
        if enable_semantic:
            bp.wire("VisualServoModule", "cmd_vel", "SafetyRingModule", "cmd_vel")
    except Exception:
        pass

    # Odometry routing — when SLAM is active, two sources exist (Driver + SLAM)
    # causing auto_wire ambiguity on ALL odometry consumers.
    # Solution: SLAM → NavigationModule (high accuracy for planning),
    #           Driver → everything else (always available, low latency).
    if _slam:
        bp.wire(_slam, "odometry", "NavigationModule", "odometry")
        # Driver odometry to all other consumers that need it
        _odom_consumers = [
            "OccupancyGridModule", "ElevationMapModule", "TerrainModule",
            "LocalPlannerModule", "PathFollowerModule",
            "SemanticMapperModule", "EpisodicMemoryModule", "TaggedLocationsModule",
            "VectorMemoryModule", "SemanticPlannerModule", "VisualServoModule",
            "ReconstructionModule",
            "SafetyRingModule", "GeofenceManagerModule",
        ]
        if enable_gateway:
            _odom_consumers += ["GatewayModule", "MCPServerModule"]
        for consumer in _odom_consumers:
            try:
                bp.wire(_drv, "odometry", consumer, "odometry")
            except Exception:
                pass  # module not in blueprint for this profile

    # Goal routing from sim driver
    try:
        DriverCls = __import__("core.registry", fromlist=["get"]).get("driver", robot)
        if hasattr(DriverCls, "goal_pose"):
            bp.wire(_drv, "goal_pose", "NavigationModule", "goal_pose")
    except Exception:
        pass

    # Autonomy chain — explicit wires
    if enable_native:
        bp.wire("NavigationModule", "waypoint", "LocalPlannerModule", "waypoint")
        bp.wire("TerrainModule", "terrain_map", "LocalPlannerModule", "terrain_map")
        bp.wire("LocalPlannerModule", "local_path", "PathFollowerModule", "local_path")
        bp.wire("PathFollowerModule", "cmd_vel", _drv, "cmd_vel")

    # Visual servo — dual channel
    if enable_semantic:
        try:
            bp.wire("SemanticPlannerModule", "servo_target", "VisualServoModule", "servo_target")
            bp.wire("VisualServoModule", "goal_pose", "NavigationModule", "goal_pose")
            bp.wire("VisualServoModule", "nav_stop", "NavigationModule", "stop_signal")
            if enable_native:
                bp.wire("VisualServoModule", "cmd_vel", _drv, "cmd_vel")
        except Exception:
            pass

    # Teleop — joystick cmd_vel + pause autonomy
    if enable_gateway:
        try:
            bp.wire("TeleopModule", "cmd_vel", _drv, "cmd_vel")
            bp.wire("TeleopModule", "nav_stop", "NavigationModule", "stop_signal")
        except Exception:
            pass

    return bp
