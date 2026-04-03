from __future__ import annotations

from core.blueprints.full_stack import full_stack_blueprint


def _has_connection(system, out_mod: str, out_port: str, in_mod: str, in_port: str) -> bool:
    return any(
        c[0] == out_mod and c[1] == out_port and c[2] == in_mod and c[3] == in_port
        for c in system.connections
    )


def test_non_native_navigation_uses_python_autonomy_chain():
    system = full_stack_blueprint(
        robot="stub",
        slam_profile="none",
        enable_native=False,
        enable_semantic=False,
        enable_gateway=False,
        enable_map_modules=False,
    ).build()

    try:
        nav = system.get_module("NavigationModule")
        local_planner = system.get_module("LocalPlannerModule")
        path_follower = system.get_module("PathFollowerModule")

        assert nav._enable_ros2_bridge is False
        assert local_planner._backend == "cmu_py"
        assert path_follower._backend == "pure_pursuit"

        assert _has_connection(system, "StubDogModule", "odometry", "NavigationModule", "odometry")
        assert _has_connection(system, "NavigationModule", "waypoint", "LocalPlannerModule", "waypoint")
        assert _has_connection(system, "LocalPlannerModule", "local_path", "PathFollowerModule", "local_path")
        assert _has_connection(system, "PathFollowerModule", "cmd_vel", "StubDogModule", "cmd_vel")
    finally:
        system.stop()
