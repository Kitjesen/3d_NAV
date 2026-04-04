"""Profile and robot-preset tables for the LingTu CLI."""

from __future__ import annotations

import os

def _resolve_tomogram() -> str:
    """Return the active tomogram path, falling back to the built-in sample map.

    Priority:
      1. $NAV_MAP_DIR/active/tomogram.pickle  (user's real map, built via 'map build')
      2. src/global_planning/PCT_planner/rsc/tomogram/building2_9.pickle  (sample)

    The fallback lets the nav profile start without a user-built map so PCT
    can be verified. In production the real map should be in place.
    """
    active = os.path.join(
        os.environ.get("NAV_MAP_DIR", os.path.expanduser("~/data/nova/maps")),
        "active",
        "tomogram.pickle",
    )
    if os.path.isfile(active):
        return active

    # Fallback: built-in sample tomogram (relative to project root)
    _repo = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    sample = os.path.join(
        _repo,
        "src", "global_planning", "PCT_planner",
        "rsc", "tomogram", "building2_9.pickle",
    )
    return sample


_ACTIVE_TOMOGRAM = _resolve_tomogram()

ROBOT_PRESETS = {
    "stub": dict(robot="stub", slam_profile="none", detector="yoloe", encoder="mobileclip"),
    "sim": dict(robot="sim_mujoco", slam_profile="bridge", detector="yoloe", encoder="mobileclip"),
    "ros2": dict(robot="sim_ros2", slam_profile="bridge", detector="yoloe", encoder="mobileclip"),
    "s100p": dict(robot="sim_ros2", slam_profile="localizer", detector="bpu", encoder="mobileclip"),
    "thunder": dict(
        robot="thunder",
        slam_profile="localizer",
        detector="bpu",
        encoder="mobileclip",
        dog_host="192.168.66.190",
        dog_port=13145,
    ),
}

PROFILES = {
    "map": dict(
        _desc="Build map — SLAM + PGO, then 'map save <name>'",
        _default_robot="s100p",
        slam_profile="fastlio2",
        llm="mock",
        planner="astar",
        enable_native=False,
        enable_semantic=False,
        enable_gateway=True,
        enable_map_modules=True,
        gateway_port=5050,
    ),
    "nav": dict(
        _desc="Navigate with pre-built map (localizer + full stack)",
        _default_robot="s100p",
        llm="qwen",
        planner="pct",          # S100P: use ele_planner.so (3D terrain-aware)
        tomogram=_ACTIVE_TOMOGRAM,
        # enable_native=False: C++ local_planner requires the 'local_planner'
        # ROS2 package installed via colcon. Use Python autonomy chain instead.
        # Switch to True only after running: make build && ros2 pkg list | grep local_planner
        enable_native=False,
        enable_semantic=True,
        enable_gateway=True,
        gateway_port=5050,
    ),
    "explore": dict(
        _desc="Explore unknown area (SLAM + frontier, no map needed)",
        _default_robot="s100p",
        slam_profile="fastlio2",
        llm="qwen",
        planner="pct",          # S100P: use ele_planner.so (3D terrain-aware)
        enable_native=False,    # same reason as nav profile above
        enable_semantic=True,
        enable_gateway=True,
        enable_frontier=True,
        gateway_port=5050,
    ),
    "sim": dict(
        _desc="MuJoCo simulation (full algorithm stack)",
        _default_robot="sim",
        llm="mock",
        planner="astar",
        tomogram="src/global_planning/PCT_planner/rsc/tomogram/building2_9.pickle",
        enable_native=True,
        enable_semantic=True,
        enable_gateway=True,
        gateway_port=5050,
    ),
    "dev": dict(
        _desc="Semantic pipeline dev (no hardware)",
        _default_robot="stub",
        llm="mock",
        planner="astar",
        enable_native=False,
        enable_semantic=True,
        enable_gateway=True,
        gateway_port=5050,
    ),
    "stub": dict(
        _desc="Framework testing (no hardware, no semantic)",
        _default_robot="stub",
        llm="mock",
        planner="astar",
        enable_native=False,
        enable_semantic=False,
        enable_gateway=True,
        gateway_port=5050,
    ),
}
