from core.blueprints.profile_graph import (
    PROFILE_SNAPSHOT_TARGETS,
    graph_for_profile,
    resolve_profile_config,
)
from core.blueprints.stacks.navigation import navigation


REAL_LOCALIZATION_PROFILES = (
    "map",
    "nav",
    "explore",
    "super_lio",
    "super_lio_relocation",
)


def _wire_set(graph):
    return {wire.as_snapshot() for wire in graph.explicit_wires}


def test_profile_graphs_compile_for_primary_profiles():
    for profile in PROFILE_SNAPSHOT_TARGETS:
        graph = graph_for_profile(profile)

        assert "NavigationModule" in graph.modules
        assert "SafetyRingModule" in graph.modules
        assert "GatewayModule" in graph.modules
        assert "MCPServerModule" in graph.modules
        assert not graph.dangling_wires(), profile


def test_profile_graph_snapshot_locks_safety_gateway_and_mux_edges():
    for profile in PROFILE_SNAPSHOT_TARGETS:
        graph = graph_for_profile(profile)
        wires = _wire_set(graph)
        modules = set(graph.modules)

        driver = next(
            module
            for module in modules
            if module.endswith("Driver")
            or module.endswith("DriverModule")
            or module.endswith("DogModule")
        )

        assert f"SafetyRingModule.stop_cmd->{driver}.stop_signal" in wires
        assert "SafetyRingModule.stop_cmd->NavigationModule.stop_signal" in wires
        assert f"GatewayModule.stop_cmd->{driver}.stop_signal" in wires
        assert "GatewayModule.stop_cmd->NavigationModule.stop_signal" in wires
        assert "GatewayModule.cmd_vel->CmdVelMux.teleop_cmd_vel" in wires
        assert "NavigationModule.mission_status->GatewayModule.mission_status" in wires
        assert "NavigationModule.mission_status->MCPServerModule.mission_status" in wires
        assert "NavigationModule.clear_path->LocalPlannerModule.clear_path" in wires
        assert "NavigationModule.global_path->LocalPlannerModule.global_path" in wires
        assert "GatewayModule.cancel->NavigationModule.cancel" in wires
        assert f"CmdVelMux.driver_cmd_vel->{driver}.cmd_vel" in wires


def test_profile_graph_snapshot_locks_mapping_and_algorithm_edges():
    for profile in ("stub", "dev", "sim", "sim_gazebo", "map", "nav", "explore"):
        wires = _wire_set(graph_for_profile(profile))

        assert "OccupancyGridModule.costmap->TraversabilityCostModule.costmap" in wires
        assert "ESDFModule.esdf->TraversabilityCostModule.esdf" in wires
        assert "TraversabilityCostModule.fused_cost->NavigationModule.costmap" in wires
        assert "TraversabilityCostModule.fused_cost->GatewayModule.costmap" in wires


def test_nav_profile_uses_slam_bridge_localization_health_edges():
    wires = _wire_set(graph_for_profile("nav"))

    assert "SlamBridgeModule.localization_status->SafetyRingModule.localization_status" in wires
    assert "SlamBridgeModule.localization_status->NavigationModule.localization_status" in wires
    assert "SlamBridgeModule.localization_status->DepthVisualOdomModule.localization_status" in wires
    assert "SlamBridgeModule.localization_status->GatewayModule.localization_status" in wires
    assert "SlamBridgeModule.map_frame_jump_event->NavigationModule.map_frame_jump_event" in wires
    assert "SlamBridgeModule.map_frame_jump_event->LocalPlannerModule.map_frame_jump_event" in wires
    assert "SlamBridgeModule.map_frame_jump_event->PathFollowerModule.map_frame_jump_event" in wires


def test_s100p_profiles_drive_real_brainstem_driver_by_default():
    for profile in ("map", "nav", "explore"):
        graph = graph_for_profile(profile)

        assert "ThunderDriver" in graph.modules
        assert "ROS2SimDriverModule" not in graph.modules


def test_sim_gazebo_profile_uses_ros2_driver_and_live_odometry_frame():
    graph = graph_for_profile("sim_gazebo")
    wires = _wire_set(graph)
    config = resolve_profile_config("sim_gazebo")

    assert config["robot"] == "sim_ros2"
    assert config["slam_profile"] == "none"
    assert config["planning_frame_id"] == "odom"
    assert config["enable_native"] is False
    assert config["python_autonomy_backend"] == "nanobind"
    assert config["python_path_follower_backend"] == "nav_core"
    assert config["enable_camera"] is True
    assert config["use_driver_camera"] is True
    assert config["cloud_topic"] == "/nav/map_cloud"
    assert "ROS2SimDriverModule" in graph.modules
    assert "CameraBridgeModule" not in graph.modules
    assert "MujocoDriverModule" not in graph.modules
    assert "ThunderDriver" not in graph.modules
    assert "ROS2SimDriverModule.map_cloud->OccupancyGridModule.map_cloud" in wires
    assert "ROS2SimDriverModule.map_cloud->TerrainModule.map_cloud" in wires
    assert "ROS2SimDriverModule.odometry->NavigationModule.odometry" in wires
    assert "SlamBridgeModule" not in graph.modules
    assert not graph.dangling_wires()


def test_sim_profiles_keep_autonomy_inside_module_graph():
    for profile in ("sim", "sim_gazebo"):
        config = resolve_profile_config(profile)
        nav_config = dict(config)
        planner = nav_config.pop("planner", "astar")
        tomogram = nav_config.pop("tomogram", "")
        enable_native = nav_config.pop("enable_native", False)

        bp = navigation(planner, tomogram, enable_native, **nav_config)
        entries = {entry.name: entry for entry in bp._entries}

        assert enable_native is False
        assert entries["TerrainModule"].config["backend"] == "nanobind"
        assert entries["LocalPlannerModule"].config["backend"] == "nanobind"
        assert entries["PathFollowerModule"].config["backend"] == "nav_core"


def test_real_robot_profiles_do_not_auto_actuate_on_startup():
    for profile in ("map", "nav", "super_lio", "super_lio_relocation", "explore", "tare_explore"):
        config = resolve_profile_config(profile)

        assert config["robot"] == "thunder"
        assert config["auto_enable"] is False
        assert config["auto_standup"] is False


def test_navigation_profiles_use_localization_odometry_for_runtime_consumers():
    for profile in REAL_LOCALIZATION_PROFILES:
        source = "SlamBridgeModule"
        graph = graph_for_profile(profile)
        wires = _wire_set(graph)

        assert source in graph.modules
        assert f"{source}.odometry->NavigationModule.odometry" in wires
        assert f"{source}.odometry->GatewayModule.odometry" in wires
        assert f"{source}.odometry->SafetyRingModule.odometry" in wires
        assert f"{source}.odometry->PathFollowerModule.odometry" in wires
        assert f"{source}.odometry->LocalPlannerModule.odometry" in wires
        assert f"{source}.map_cloud->OccupancyGridModule.map_cloud" in wires
        assert f"{source}.map_cloud->VoxelGridModule.map_cloud" in wires
        assert f"{source}.map_cloud->ElevationMapModule.map_cloud" in wires
        assert f"{source}.map_cloud->TerrainModule.map_cloud" in wires
        assert f"{source}.map_cloud->GatewayModule.map_cloud" in wires
        assert f"{source}.localization_status->NavigationModule.localization_status" in wires
        assert f"{source}.localization_status->SafetyRingModule.localization_status" in wires
        assert f"{source}.localization_status->GatewayModule.localization_status" in wires
        if "DepthVisualOdomModule" in graph.modules:
            assert f"{source}.localization_status->DepthVisualOdomModule.localization_status" in wires


def test_super_lio_profiles_wire_bridge_localization_status_to_gateway():
    for profile in ("super_lio", "super_lio_relocation"):
        graph = graph_for_profile(profile)
        wires = _wire_set(graph)

        assert "SlamBridgeModule" in graph.modules
        assert "SlamBridgeModule.localization_status->GatewayModule.localization_status" in wires
        assert "SlamBridgeModule.map_frame_jump_event->NavigationModule.map_frame_jump_event" in wires
        assert "SlamBridgeModule.map_frame_jump_event->LocalPlannerModule.map_frame_jump_event" in wires
        assert "SlamBridgeModule.map_frame_jump_event->PathFollowerModule.map_frame_jump_event" in wires
        assert not graph.dangling_wires(), profile


def test_real_robot_profiles_plan_in_live_odometry_frame():
    profiles = (
        "map",
        "nav",
        "super_lio",
        "super_lio_relocation",
        "explore",
        "tare_explore",
    )
    for profile in profiles:
        config = resolve_profile_config(profile)
        nav_config = dict(config)
        planner = nav_config.pop("planner", "astar")
        tomogram = nav_config.pop("tomogram", "")
        enable_native = nav_config.pop("enable_native", False)
        bp = navigation(
            planner,
            tomogram,
            enable_native,
            **nav_config,
        )
        nav_entry = next(entry for entry in bp._entries if entry.name == "NavigationModule")

        assert nav_entry.config["planning_frame_id"] == "odom"


def test_navigation_plan_safety_policy_is_profile_visible():
    sim_config = resolve_profile_config("sim")
    sim_nav_config = dict(sim_config)
    sim_planner = sim_nav_config.pop("planner", "astar")
    sim_tomogram = sim_nav_config.pop("tomogram", "")
    sim_enable_native = sim_nav_config.pop("enable_native", False)
    sim_bp = navigation(sim_planner, sim_tomogram, sim_enable_native, **sim_nav_config)
    sim_entry = next(entry for entry in sim_bp._entries if entry.name == "NavigationModule")

    assert sim_entry.config["plan_safety_policy"] == "fallback_astar"
    assert sim_entry.config["fallback_planner_name"] == "astar"

    for profile in ("nav", "explore", "tare_explore", "super_lio", "super_lio_relocation"):
        config = resolve_profile_config(profile)
        nav_config = dict(config)
        planner = nav_config.pop("planner", "astar")
        tomogram = nav_config.pop("tomogram", "")
        enable_native = nav_config.pop("enable_native", False)
        bp = navigation(planner, tomogram, enable_native, **nav_config)
        nav_entry = next(entry for entry in bp._entries if entry.name == "NavigationModule")

        assert nav_entry.config["plan_safety_policy"] == "observe"
        assert nav_entry.config["fallback_planner_name"] == "astar"
