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
    enable_rerun: bool = False,
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
        navigation(planner_backend, tomogram, enable_native, **config),
        safety(),
        gateway(gateway_port, enable_rerun=enable_rerun)
                               if enable_gateway else Blueprint(),
    )

    # ── Cross-stack critical wires ──────────────────────────────────────
    _drv = driver_name(robot)
    _slam = slam_module_name(slam_profile)

    # map_cloud routing — SLAM module publishes map_cloud to map consumers.
    # When SLAM is active, explicitly wire to avoid ambiguity with driver's lidar_cloud.
    # Only wire to modules actually in the blueprint.
    _bp_names = {e.name for e in bp._entries}
    if _slam:
        for consumer in ["OccupancyGridModule", "ElevationMapModule", "TerrainModule", "VoxelGridModule", "RerunBridgeModule"]:
            if consumer in _bp_names:
                bp.wire(_slam, "map_cloud", consumer, "map_cloud")

    # depth_image routing — Driver or CameraBridge provides depth.
    if enable_semantic:
        _depth_src = "CameraBridgeModule" if "CameraBridgeModule" in _bp_names else _drv
        for consumer in ["ReconstructionModule", "VisualServoModule"]:
            if consumer in _bp_names and _depth_src in _bp_names:
                bp.wire(_depth_src, "depth_image", consumer, "depth_image")

    # Safety → all actuators
    bp.wire("SafetyRingModule", "stop_cmd", _drv, "stop_signal")
    bp.wire("SafetyRingModule", "stop_cmd", "NavigationModule", "stop_signal")

    # Helper: only wire if both modules exist in the blueprint
    def _w(out_mod, out_port, in_mod, in_port):
        if out_mod in _bp_names and in_mod in _bp_names:
            bp.wire(out_mod, out_port, in_mod, in_port)

    # Instruction + goal routing — Gateway/MCP both publish instruction/goal_pose,
    # causing auto_wire ambiguity. Explicitly fan-in both sources to consumers.
    _w("GatewayModule", "instruction", "SemanticPlannerModule", "instruction")
    _w("MCPServerModule", "instruction", "SemanticPlannerModule", "instruction")
    _w("GatewayModule", "instruction", "NavigationModule", "instruction")
    _w("MCPServerModule", "instruction", "NavigationModule", "instruction")
    _w("GatewayModule", "goal_pose", "NavigationModule", "goal_pose")
    _w("MCPServerModule", "goal_pose", "NavigationModule", "goal_pose")
    _w("SemanticPlannerModule", "goal_pose", "NavigationModule", "goal_pose")

    # cmd_vel monitoring — SafetyRing needs to see all cmd_vel sources
    _w("GatewayModule", "cmd_vel", "SafetyRingModule", "cmd_vel")
    _w("VisualServoModule", "cmd_vel", "SafetyRingModule", "cmd_vel")

    # Odometry routing — SLAM → Nav (high accuracy), Driver → everything else
    if _slam and _slam in _bp_names:
        bp.wire(_slam, "odometry", "NavigationModule", "odometry")
        for consumer in [
            "OccupancyGridModule", "ElevationMapModule", "TerrainModule",
            "VoxelGridModule", "WavefrontFrontierExplorer", "RerunBridgeModule",
            "LocalPlannerModule", "PathFollowerModule",
            "SemanticMapperModule", "EpisodicMemoryModule", "TaggedLocationsModule",
            "VectorMemoryModule", "TemporalMemoryModule",
            "SemanticPlannerModule", "VisualServoModule",
            "ReconstructionModule", "SafetyRingModule", "GeofenceManagerModule",
            "GatewayModule", "MCPServerModule",
        ]:
            if consumer in _bp_names:
                bp.wire(_drv, "odometry", consumer, "odometry")

    # Goal routing from sim driver
    try:
        from core.registry import get as _get_plugin
        if hasattr(_get_plugin("driver", robot), "goal_pose"):
            bp.wire(_drv, "goal_pose", "NavigationModule", "goal_pose")
    except Exception:
        pass

    # Autonomy chain
    if enable_native:
        _w("NavigationModule", "waypoint", "LocalPlannerModule", "waypoint")
        _w("TerrainModule", "terrain_map", "LocalPlannerModule", "terrain_map")
        _w("LocalPlannerModule", "local_path", "PathFollowerModule", "local_path")
        _w("PathFollowerModule", "cmd_vel", _drv, "cmd_vel")

    # Visual servo — dual channel
    _w("SemanticPlannerModule", "servo_target", "VisualServoModule", "servo_target")
    _w("VisualServoModule", "goal_pose", "NavigationModule", "goal_pose")
    _w("VisualServoModule", "nav_stop", "NavigationModule", "stop_signal")
    if enable_native:
        _w("VisualServoModule", "cmd_vel", _drv, "cmd_vel")

    # Teleop — joystick cmd_vel + pause autonomy
    _w("TeleopModule", "cmd_vel", _drv, "cmd_vel")
    _w("TeleopModule", "nav_stop", "NavigationModule", "stop_signal")

    return bp
