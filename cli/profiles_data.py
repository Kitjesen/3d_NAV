"""Profile and robot-preset tables for the LingTu CLI."""

from __future__ import annotations

import os


def _default_map_dir() -> str:
    """Return the default on-robot map storage directory.

    Maps are runtime data (often large) and should live outside the repo so they
    survive code updates and can be shared across profiles.

    Order:
      1) $NAV_MAP_DIR, if set
      2) legacy: ~/data/nova/maps (kept for backwards compatibility)
      3) new default: ~/data/lingtu/maps
    """
    env = os.environ.get("NAV_MAP_DIR")
    if env:
        return env

    legacy = os.path.expanduser("~/data/nova/maps")
    if os.path.isdir(legacy):
        return legacy

    return os.path.expanduser("~/data/lingtu/maps")


def _resolve_tomogram() -> str:
    """Return the active tomogram path, falling back to the built-in sample map.

    Priority:
      1. $NAV_MAP_DIR/active/tomogram.pickle  (user's real map, built via 'map build')
      2. src/global_planning/PCT_planner/rsc/tomogram/building2_9.pickle  (sample)

    The fallback lets the nav profile start without a user-built map so PCT
    can be verified. In production the real map should be in place.
    """
    active = os.path.join(
        _default_map_dir(),
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
    # S100P hardware preset (RDK X5, Nash BPU, Livox MID-360)
    "s100p": dict(robot="sim_ros2", slam_profile="localizer", detector="bpu", encoder="mobileclip"),
    "navigate": dict(robot="sim_ros2", slam_profile="localizer", detector="bpu", encoder="mobileclip"),  # alias
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
        _desc="Build a new map of the environment",
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
        _desc="Navigate using a saved map",
        _default_robot="s100p",
        # Bridge mode: SlamBridgeModule subscribes to /nav/odometry + /nav/map_cloud
        # produced by external ROS2 SLAM nodes (lidar.service for the Livox driver,
        # plus a Fast-LIO2/PGO/Localizer chain managed outside lingtu). This profile
        # intentionally overrides the s100p preset's "localizer" because on the real
        # robot the SLAM stack runs as separate systemd units; spawning our own
        # NativeModule SLAM here would race them on the LiDAR USB device.
        slam_profile="bridge",
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
        _desc="Explore unknown area (wavefront frontier under navigation stack)",
        _default_robot="s100p",
        slam_profile="fastlio2",
        llm="qwen",
        planner="pct",          # S100P: use ele_planner.so (3D terrain-aware)
        enable_native=False,    # same reason as nav profile above
        enable_semantic=True,
        enable_gateway=True,
        # WavefrontFrontierExplorer is provided by navigation() stack (via
        # enable_frontier=True), not by exploration() stack — the latter is
        # TARE-only since 1c457f3. Keep exploration_backend="none" so we
        # don't try to spawn a TARE NativeModule on top of wavefront.
        enable_frontier=True,
        exploration_backend="none",
        gateway_port=5050,
    ),
    "tare_explore": dict(
        _desc="Explore via CMU TARE hierarchical planner (needs tare_planner submodule built)",
        _default_robot="s100p",
        slam_profile="fastlio2",
        llm="qwen",
        planner="pct",
        enable_native=False,
        enable_semantic=True,
        enable_gateway=True,
        # Don't add wavefront explorer — TARE stack handles goal generation.
        enable_frontier=False,
        exploration_backend="tare",
        tare_scenario="forest",
        gateway_port=5050,
    ),
    "sim": dict(
        _desc="MuJoCo simulation",
        _default_robot="sim",
        llm="mock",
        planner="astar",
        tomogram="src/global_planning/PCT_planner/rsc/tomogram/building2_9.pickle",
        enable_native=True,
        enable_semantic=True,
        enable_gateway=True,
        gateway_port=5050,
    ),
    "sim_nav": dict(
        _desc="Pure-Python navigation sim (no ROS2/C++)",
        _default_robot="stub",
        slam_profile="none",
        llm="mock",
        planner="astar",
        scene_xml="sim/worlds/building_scene.xml",
        enable_native=False,
        enable_semantic=False,
        enable_gateway=True,
        enable_map_modules=True,
        gateway_port=5050,
        python_autonomy_backend="simple",
        python_path_follower_backend="pid",
    ),
    "dev": dict(
        _desc="Test perception & planning without a robot",
        _default_robot="stub",
        llm="mock",
        planner="astar",
        enable_native=False,
        enable_semantic=True,
        enable_gateway=True,
        gateway_port=5050,
    ),
    "stub": dict(
        _desc="Framework testing only",
        _default_robot="stub",
        llm="mock",
        planner="astar",
        enable_native=False,
        enable_semantic=False,
        enable_gateway=True,
        gateway_port=5050,
    ),
}
