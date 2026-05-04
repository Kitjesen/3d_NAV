from core.blueprints.profile_graph import PROFILE_SNAPSHOT_TARGETS, graph_for_profile


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
            if module.endswith("DriverModule") or module.endswith("DogModule")
        )

        assert f"SafetyRingModule.stop_cmd->{driver}.stop_signal" in wires
        assert "SafetyRingModule.stop_cmd->NavigationModule.stop_signal" in wires
        assert "NavigationModule.mission_status->GatewayModule.mission_status" in wires
        assert "NavigationModule.mission_status->MCPServerModule.mission_status" in wires
        assert f"CmdVelMux.driver_cmd_vel->{driver}.cmd_vel" in wires


def test_profile_graph_snapshot_locks_mapping_and_algorithm_edges():
    for profile in ("stub", "dev", "sim", "map", "nav", "explore"):
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


def test_navigation_profiles_use_localization_odometry_for_runtime_consumers():
    expected_sources = {
        "nav": "SlamBridgeModule",
        "map": "SlamBridgeModule",
        "explore": "SlamBridgeModule",
    }

    for profile, source in expected_sources.items():
        wires = _wire_set(graph_for_profile(profile))

        assert f"{source}.odometry->GatewayModule.odometry" in wires
        assert f"{source}.odometry->SafetyRingModule.odometry" in wires
        assert f"{source}.odometry->PathFollowerModule.odometry" in wires
        assert f"{source}.odometry->LocalPlannerModule.odometry" in wires


def test_super_lio_profiles_wire_bridge_localization_status_to_gateway():
    for profile in ("super_lio", "super_lio_relocation"):
        graph = graph_for_profile(profile)
        wires = _wire_set(graph)

        assert "SlamBridgeModule" in graph.modules
        assert "SlamBridgeModule.localization_status->GatewayModule.localization_status" in wires
        assert not graph.dangling_wires(), profile
