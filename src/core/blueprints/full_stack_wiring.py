"""Explicit cross-stack wiring for the full LingTu profile.

The module keeps high-risk fan-in/fan-out rules out of the profile factory so
profile graph tests can lock the actual contract of the assembled system.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from core.blueprint import Blueprint

from .stacks.slam import slam_module_name


@dataclass(frozen=True)
class WireSpec:
    out_module: str
    out_port: str
    in_module: str
    in_port: str
    transport: Any = None

    def apply(self, bp: Blueprint) -> None:
        bp.wire(
            self.out_module,
            self.out_port,
            self.in_module,
            self.in_port,
            transport=self.transport,
        )


def _wire_if_present(bp: Blueprint, names: set[str], spec: WireSpec) -> None:
    if (
        spec.out_module in names
        and spec.in_module in names
        and _declares_port(bp, spec.out_module, spec.out_port)
        and _declares_port(bp, spec.in_module, spec.in_port)
    ):
        spec.apply(bp)


def _apply_if_present(bp: Blueprint, names: set[str], specs: list[WireSpec]) -> None:
    for spec in specs:
        _wire_if_present(bp, names, spec)


def _declares_port(bp: Blueprint, module_name: str, port_name: str) -> bool:
    for entry in bp._entries:
        if entry.name != module_name:
            continue
        if entry.instance is not None:
            ports = getattr(entry.instance, "all_ports", {})
            if port_name in ports:
                return True
        for cls in entry.module_cls.__mro__:
            if port_name in getattr(cls, "__annotations__", {}):
                return True
        return hasattr(entry.module_cls, port_name)
    return False


def apply_full_stack_wires(
    bp: Blueprint,
    *,
    robot: str,
    driver_module: str,
    slam_profile: str,
    scene_xml: str = "",
    enable_semantic: bool = True,
) -> Blueprint:
    """Apply explicit cross-stack wires to a composed full-stack Blueprint."""

    drv = driver_module
    slam_module = slam_module_name(slam_profile)
    names = {entry.name for entry in bp._entries}
    camera_src = "CameraBridgeModule" if "CameraBridgeModule" in names else drv
    color_out = "color_image" if camera_src == "CameraBridgeModule" else "camera_image"

    map_consumers = [
        "OccupancyGridModule",
        "ElevationMapModule",
        "TerrainModule",
        "VoxelGridModule",
        "RerunBridgeModule",
        "GatewayModule",
    ]
    if slam_module:
        for consumer in map_consumers:
            _wire_if_present(bp, names, WireSpec(slam_module, "map_cloud", consumer, "map_cloud"))
    elif scene_xml and "SimPointCloudProvider" in names:
        for consumer in map_consumers:
            _wire_if_present(
                bp,
                names,
                WireSpec("SimPointCloudProvider", "map_cloud", consumer, "map_cloud"),
            )
        _wire_if_present(bp, names, WireSpec(drv, "odometry", "SimPointCloudProvider", "odometry"))
    else:
        driver_map_port = ""
        if drv == "ROS2SimDriverModule":
            driver_map_port = "map_cloud"
        elif drv == "MujocoDriverModule":
            driver_map_port = "lidar_cloud"
        if driver_map_port:
            for consumer in map_consumers:
                _wire_if_present(bp, names, WireSpec(drv, driver_map_port, consumer, "map_cloud"))

    if enable_semantic:
        for consumer in ["PerceptionModule", "ReconstructionModule", "VisualServoModule"]:
            _apply_if_present(
                bp,
                names,
                [
                    WireSpec(camera_src, color_out, consumer, "color_image"),
                    WireSpec(camera_src, "depth_image", consumer, "depth_image"),
                    WireSpec(camera_src, "camera_info", consumer, "camera_info"),
                ],
            )

    WireSpec("SafetyRingModule", "stop_cmd", drv, "stop_signal").apply(bp)
    WireSpec("SafetyRingModule", "stop_cmd", "NavigationModule", "stop_signal").apply(bp)
    _apply_if_present(
        bp,
        names,
        [
            WireSpec("GatewayModule", "stop_cmd", drv, "stop_signal"),
            WireSpec("GatewayModule", "stop_cmd", "NavigationModule", "stop_signal"),
            WireSpec("GatewayModule", "cmd_vel", "CmdVelMux", "teleop_cmd_vel"),
        ],
    )

    _apply_if_present(
        bp,
        names,
        [
            WireSpec("SlamBridgeModule", "localization_status", "SafetyRingModule", "localization_status"),
            WireSpec("SlamBridgeModule", "localization_status", "NavigationModule", "localization_status"),
            WireSpec("SlamBridgeModule", "localization_status", "DepthVisualOdomModule", "localization_status"),
            WireSpec("SlamBridgeModule", "localization_status", "GatewayModule", "localization_status"),
            WireSpec("SlamBridgeModule", "map_frame_jump_event", "NavigationModule", "map_frame_jump_event"),
            WireSpec("SlamBridgeModule", "map_frame_jump_event", "LocalPlannerModule", "map_frame_jump_event"),
            WireSpec("SlamBridgeModule", "map_frame_jump_event", "PathFollowerModule", "map_frame_jump_event"),
            WireSpec("DepthVisualOdomModule", "visual_odometry", "SlamBridgeModule", "visual_odom"),
            WireSpec(camera_src, color_out, "DepthVisualOdomModule", "color_image"),
            WireSpec(camera_src, "depth_image", "DepthVisualOdomModule", "depth_image"),
            WireSpec(camera_src, "camera_info", "DepthVisualOdomModule", "camera_info"),
            WireSpec("GatewayModule", "instruction", "SemanticPlannerModule", "instruction"),
            WireSpec("MCPServerModule", "instruction", "SemanticPlannerModule", "instruction"),
            WireSpec("GatewayModule", "instruction", "NavigationModule", "instruction"),
            WireSpec("MCPServerModule", "instruction", "NavigationModule", "instruction"),
            WireSpec("GatewayModule", "goal_pose", "NavigationModule", "goal_pose"),
            WireSpec("GatewayModule", "cancel", "NavigationModule", "cancel"),
            WireSpec("MCPServerModule", "goal_pose", "NavigationModule", "goal_pose"),
            WireSpec("SemanticPlannerModule", "goal_pose", "NavigationModule", "goal_pose"),
            WireSpec("TAREExplorerModule", "exploration_goal", "NavigationModule", "goal_pose"),
            WireSpec("PerceptionModule", "detections_3d", "SemanticPlannerModule", "detections"),
        ],
    )

    nav_odom_src = slam_module if (slam_module and slam_module in names) else drv
    _apply_if_present(
        bp,
        names,
        [
            WireSpec(nav_odom_src, "odometry", "NavigationModule", "odometry"),
            WireSpec(nav_odom_src, "odometry", "PerceptionModule", "odometry"),
        ],
    )
    # Navigation-state consumers must use the same odometry frame as
    # NavigationModule/Gateway. On real profiles this is the SLAM bridge or
    # managed SLAM output; falling back to driver odometry only applies when no
    # localization module exists.
    for consumer in [
        "OccupancyGridModule",
        "ElevationMapModule",
        "TerrainModule",
        "VoxelGridModule",
        "WavefrontFrontierExplorer",
        "RerunBridgeModule",
        "LocalPlannerModule",
        "PathFollowerModule",
        "SemanticMapperModule",
        "EpisodicMemoryModule",
        "TaggedLocationsModule",
        "VectorMemoryModule",
        "TemporalMemoryModule",
        "MissionLoggerModule",
        "SemanticPlannerModule",
        "VisualServoModule",
        "ReconstructionModule",
        "SafetyRingModule",
        "GeofenceManagerModule",
        "MCPServerModule",
    ]:
        _wire_if_present(
            bp,
            names,
            WireSpec(nav_odom_src, "odometry", consumer, "odometry"),
        )
    _wire_if_present(bp, names, WireSpec(nav_odom_src, "odometry", "GatewayModule", "odometry"))

    _apply_if_present(
        bp,
        names,
        [
            WireSpec("OccupancyGridModule", "costmap", "TraversabilityCostModule", "costmap"),
            WireSpec("ElevationMapModule", "elevation_map", "TraversabilityCostModule", "elevation_map"),
            WireSpec("ESDFModule", "esdf", "TraversabilityCostModule", "esdf"),
            WireSpec("TerrainModule", "traversability", "TraversabilityCostModule", "traversability"),
            WireSpec("TraversabilityCostModule", "fused_cost", "NavigationModule", "costmap"),
            WireSpec("TraversabilityCostModule", "esdf_field", "LocalPlannerModule", "esdf"),
            WireSpec("TraversabilityCostModule", "fused_cost", "GatewayModule", "costmap"),
            WireSpec("TraversabilityCostModule", "slope_grid", "GatewayModule", "slope_grid"),
            WireSpec("NavigationModule", "mission_status", "MissionLoggerModule", "mission_status"),
            WireSpec("PerceptionModule", "scene_graph", "GatewayModule", "scene_graph"),
            WireSpec("PerceptionModule", "scene_graph", "MCPServerModule", "scene_graph"),
            WireSpec("PerceptionModule", "scene_graph", "ReconstructionModule", "scene_graph"),
            WireSpec("PerceptionModule", "scene_graph", "SemanticMapperModule", "scene_graph"),
            WireSpec("PerceptionModule", "scene_graph", "EpisodicMemoryModule", "scene_graph"),
            WireSpec("PerceptionModule", "scene_graph", "VectorMemoryModule", "scene_graph"),
            WireSpec("PerceptionModule", "scene_graph", "TemporalMemoryModule", "scene_graph"),
            WireSpec("PerceptionModule", "scene_graph", "SemanticPlannerModule", "scene_graph"),
            WireSpec("PerceptionModule", "scene_graph", "VisualServoModule", "scene_graph"),
        ],
    )

    for recorder in ["DatasetRecorderModule", "ReconKeyframeExporterModule"]:
        _apply_if_present(
            bp,
            names,
            [
                WireSpec(camera_src, color_out, recorder, "color_image"),
                WireSpec(camera_src, "depth_image", recorder, "depth_image"),
                WireSpec(camera_src, "camera_info", recorder, "camera_info"),
                WireSpec(nav_odom_src, "odometry", recorder, "odometry"),
            ],
        )

    _apply_if_present(
        bp,
        names,
        [
            WireSpec("SafetyRingModule", "safety_state", "GatewayModule", "safety_state"),
            WireSpec("SafetyRingModule", "safety_state", "MCPServerModule", "safety_state"),
            WireSpec("NavigationModule", "mission_status", "GatewayModule", "mission_status"),
            WireSpec("NavigationModule", "mission_status", "MCPServerModule", "mission_status"),
            WireSpec("SafetyRingModule", "execution_eval", "GatewayModule", "execution_eval"),
            WireSpec("SafetyRingModule", "dialogue_state", "GatewayModule", "dialogue_state"),
            WireSpec("NavigationModule", "global_path", "GatewayModule", "global_path"),
            WireSpec("NavigationModule", "global_path", "LocalPlannerModule", "global_path"),
            WireSpec("LocalPlannerModule", "local_path", "GatewayModule", "local_path"),
            WireSpec("SemanticPlannerModule", "agent_message", "GatewayModule", "agent_message"),
        ],
    )

    try:
        from core.registry import get as get_plugin

        if hasattr(get_plugin("driver", robot), "goal_pose"):
            WireSpec(drv, "goal_pose", "NavigationModule", "goal_pose").apply(bp)
    except Exception:
        pass

    _apply_if_present(
        bp,
        names,
        [
            WireSpec("NavigationModule", "waypoint", "LocalPlannerModule", "waypoint"),
            WireSpec("NavigationModule", "clear_path", "LocalPlannerModule", "clear_path"),
            WireSpec("TerrainModule", "terrain_map", "LocalPlannerModule", "terrain_map"),
            WireSpec("LocalPlannerModule", "local_path", "PathFollowerModule", "local_path"),
            WireSpec("LocalPlannerModule", "local_path", "SafetyRingModule", "path"),
            WireSpec("SemanticPlannerModule", "servo_target", "VisualServoModule", "servo_target"),
            WireSpec("VisualServoModule", "goal_pose", "NavigationModule", "goal_pose"),
            WireSpec("VisualServoModule", "nav_stop", "NavigationModule", "stop_signal"),
            WireSpec(camera_src, color_out, "TeleopModule", "color_image"),
            WireSpec("PerceptionModule", "scene_graph", "TeleopModule", "scene_graph"),
            WireSpec("TeleopModule", "teleop_active", "NavigationModule", "teleop_active"),
            WireSpec(camera_src, color_out, "WebRTCStreamModule", "color_image"),
            WireSpec("TeleopModule", "cmd_vel", "CmdVelMux", "teleop_cmd_vel"),
            WireSpec("VisualServoModule", "cmd_vel", "CmdVelMux", "visual_servo_cmd_vel"),
            WireSpec("NavigationModule", "recovery_cmd_vel", "CmdVelMux", "recovery_cmd_vel"),
            WireSpec("PathFollowerModule", "cmd_vel", "CmdVelMux", "path_follower_cmd_vel"),
        ],
    )
    _wire_if_present(bp, names, WireSpec("CmdVelMux", "driver_cmd_vel", drv, "cmd_vel"))
    _wire_if_present(bp, names, WireSpec("CmdVelMux", "driver_cmd_vel", "SafetyRingModule", "cmd_vel"))

    return bp
