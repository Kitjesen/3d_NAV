"""Runtime endpoint definitions for Dimos-style task/connection split.

Profiles describe what LingTu should do: map, navigate, explore, or run TARE.
Runtime endpoints describe where sensor/state data and command sinks come from:
real robot, MuJoCo, Gazebo, or CMU Unity. Keeping this split explicit lets the
same high-level LingTu stack run against different connection layers.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Mapping


class RuntimeEndpointError(ValueError):
    """Raised when a profile cannot run on a selected runtime endpoint."""


@dataclass(frozen=True)
class RuntimeEndpointSpec:
    name: str
    description: str
    data_source: str
    robot_preset: str
    supported_profiles: tuple[str, ...]
    simulation_only: bool
    external_launcher: str | None = None
    runtime_contract: str | None = None
    config_overrides: Mapping[str, Any] = field(default_factory=dict)
    profile_overrides: Mapping[str, Mapping[str, Any]] = field(default_factory=dict)
    default_actions: Mapping[str, tuple[str, ...]] = field(default_factory=dict)
    record_actions: Mapping[str, tuple[str, ...]] = field(default_factory=dict)

    def require_profile(self, profile: str) -> None:
        if profile not in self.supported_profiles:
            supported = ", ".join(self.supported_profiles)
            raise RuntimeEndpointError(
                f"endpoint '{self.name}' does not support profile '{profile}' "
                f"(supported: {supported})"
            )

    def config_for_profile(self, profile: str) -> dict[str, Any]:
        self.require_profile(profile)
        merged = dict(self.config_overrides)
        merged.update(self.profile_overrides.get(profile, {}))
        merged["_runtime_endpoint"] = self.name
        merged["_endpoint_data_source"] = self.data_source
        if self.external_launcher:
            merged["_external_launcher"] = self.external_launcher
        if self.runtime_contract:
            merged["_runtime_contract"] = self.runtime_contract
        if self.default_actions:
            merged["_external_default_args"] = self.default_actions.get(profile, ("gate",))
        if self.record_actions:
            merged["_external_record_args"] = self.record_actions.get(
                profile,
                self.default_actions.get(profile, ("gate",)),
            )
        return merged


_MUJOCO_LIVE_CONFIG: dict[str, Any] = {
    "slam_profile": "none",
    "llm": "mock",
    "planner": "astar",
    "tomogram": "",
    "plan_safety_policy": "fallback_astar",
    "fallback_planner_name": "astar",
    "enable_semantic": False,
    "enable_gateway": True,
    "enable_teleop": False,
    "enable_map_modules": True,
    "enable_ros2_grid_bridge": True,
    "enable_ros2_path_bridge": True,
    "enable_camera": False,
    "use_driver_camera": False,
    "cloud_topic": "/nav/map_cloud",
    "planning_frame_id": "map",
    "enable_native": False,
    "latch_stop_signal": False,
    "python_autonomy_backend": "nanobind",
    "python_path_follower_backend": "nav_core",
    "run_startup_checks": False,
    "manage_external_services": False,
    "gateway_port": 5050,
}

_GAZEBO_CONFIG: dict[str, Any] = {
    "slam_profile": "none",
    "llm": "mock",
    "planner": "astar",
    "tomogram": "src/global_planning/PCT_planner/rsc/tomogram/building2_9.pickle",
    "plan_safety_policy": "fallback_astar",
    "fallback_planner_name": "astar",
    "enable_semantic": True,
    "enable_gateway": True,
    "enable_teleop": False,
    "enable_map_modules": True,
    "enable_camera": True,
    "use_driver_camera": True,
    "cloud_topic": "/nav/map_cloud",
    "planning_frame_id": "map",
    "enable_frontier": True,
    "exploration_backend": "none",
    "frontier_safe_distance": 0.80,
    "frontier_max_dist": 20.0,
    "frontier_rate": 2.0,
    "enable_native": False,
    "latch_stop_signal": False,
    "python_autonomy_backend": "nanobind",
    "python_path_follower_backend": "nav_core",
    "run_startup_checks": False,
    "manage_external_services": False,
    "gateway_port": 5050,
}

_CMU_UNITY_CONFIG: dict[str, Any] = {
    "slam_profile": "none",
    "llm": "mock",
    "planner": "pct",
    "tomogram": "",
    "plan_safety_policy": "fallback_astar",
    "fallback_planner_name": "astar",
    "safe_goal_tolerance": 0.4,
    "waypoint_threshold": 0.45,
    "final_waypoint_threshold": 0.35,
    "stuck_timeout": 25.0,
    "stuck_dist_thre": 0.08,
    "downsample_dist": 0.6,
    "path_follower_goal_tolerance": 0.35,
    "local_planner_allow_direct_track_fallback": True,
    "local_planner_ignore_near_field_stop": True,
    "local_planner_direct_track_fallback_min_distance_m": 0.3,
    "enable_native": False,
    "enable_semantic": False,
    "enable_gateway": True,
    "enable_teleop": False,
    "enable_map_modules": True,
    "enable_ros2_bridge": True,
    "enable_ros2_path_bridge": True,
    "exploration_backend": "tare_external",
    "exploration_auto_start": True,
    "prefer_path_strategy": True,
    "path_start_tolerance_m": 1.5,
    "path_goal_min_distance_m": 1.0,
    "path_goal_spacing_m": 0.75,
    "tare_fallback_timeout_s": 180.0,
    "allow_direct_goal_fallback": True,
    "direct_goal_fallback_on_planner_failure": True,
    "accept_partial_goal_progress": True,
    "partial_goal_repeat_ignore_window_s": 5.0,
    "external_strategy_path_control": False,
    "external_strategy_start_tolerance_m": 1.5,
    "planning_frame_id": "map",
    "latch_stop_signal": False,
    "run_startup_checks": False,
    "manage_external_services": False,
    "gateway_port": 5050,
}


RUNTIME_ENDPOINTS: dict[str, RuntimeEndpointSpec] = {
    "real_s100p": RuntimeEndpointSpec(
        name="real_s100p",
        description="Physical S100P/Thunder robot endpoint.",
        data_source="real_s100p",
        robot_preset="s100p",
        supported_profiles=(
            "map",
            "nav",
            "explore",
            "tare_explore",
            "super_lio",
            "super_lio_relocation",
        ),
        simulation_only=False,
    ),
    "mujoco_live": RuntimeEndpointSpec(
        name="mujoco_live",
        description="MuJoCo raw MID-360 + IMU endpoint feeding Fast-LIO.",
        data_source="mujoco_fastlio2_live",
        robot_preset="sim_gazebo",
        supported_profiles=("map", "explore", "tare_explore", "sim_mujoco_live"),
        simulation_only=True,
        external_launcher="sim/scripts/launch_mujoco_fastlio2_live.sh",
        runtime_contract="mujoco_fastlio2_live",
        config_overrides=_MUJOCO_LIVE_CONFIG,
        profile_overrides={
            "map": {
                "enable_frontier": False,
                "exploration_backend": "none",
            },
            "explore": {
                "enable_frontier": True,
                "exploration_backend": "none",
                "frontier_safe_distance": 0.80,
                "frontier_max_dist": 25.0,
                "frontier_rate": 2.0,
            },
            "tare_explore": {
                "planner": "astar",
                "planner_backend": "astar",
                "enable_frontier": False,
                "exploration_backend": "tare",
                "exploration_auto_start": False,
                "tare_scenario": "indoor",
                "goal_frame_id": "odom",
                "hold_active_goal_until_terminal": True,
                "max_waypoint_distance_m": 6.0,
                "waypoint_odometry_timeout_s": 5.0,
                "defer_empty_path_planning_failure": True,
                "empty_path_retry_interval_s": 2.0,
                "empty_path_retry_timeout_s": 90.0,
                "planning_frame_id": "odom",
                "occupancy_frame_id": "odom",
                "enable_ros2_grid_bridge": True,
                "enable_ros2_path_bridge": True,
            },
            "sim_mujoco_live": {
                "enable_frontier": True,
                "exploration_backend": "none",
                "frontier_safe_distance": 0.80,
                "frontier_max_dist": 25.0,
                "frontier_rate": 2.0,
            },
        },
        default_actions={
            "map": ("gate",),
            "explore": ("explore",),
            "tare_explore": ("tare",),
            "sim_mujoco_live": ("gate",),
        },
        record_actions={
            "map": ("video",),
            "explore": ("video",),
            "tare_explore": ("tare-video",),
            "sim_mujoco_live": ("video",),
        },
    ),
    "gazebo": RuntimeEndpointSpec(
        name="gazebo",
        description="Gazebo/GZ industrial delivery simulation endpoint.",
        data_source="gazebo_industrial",
        robot_preset="sim_gazebo",
        supported_profiles=("explore", "sim_gazebo", "sim_industrial"),
        simulation_only=True,
        external_launcher="sim/scripts/launch_lingtu_gazebo_industrial_demo.sh",
        runtime_contract="gazebo_industrial",
        config_overrides=_GAZEBO_CONFIG,
        default_actions={
            "explore": ("start", "--gate"),
            "sim_gazebo": ("start", "--gate"),
            "sim_industrial": ("start", "--gate"),
        },
        record_actions={
            "explore": ("start", "--gate", "--rviz"),
            "sim_gazebo": ("start", "--gate", "--rviz"),
            "sim_industrial": ("start", "--gate", "--rviz"),
        },
    ),
    "cmu_unity": RuntimeEndpointSpec(
        name="cmu_unity",
        description="CMU Unity + external TARE/FAR benchmark endpoint.",
        data_source="cmu_unity_external",
        robot_preset="sim_gazebo",
        supported_profiles=("tare_explore", "sim_cmu_tare"),
        simulation_only=True,
        external_launcher="sim/scripts/launch_cmu_unity_lingtu_runtime.sh",
        runtime_contract="cmu_unity_external",
        config_overrides=_CMU_UNITY_CONFIG,
        default_actions={
            "tare_explore": ("gate",),
            "sim_cmu_tare": ("gate",),
        },
        record_actions={
            "tare_explore": ("start", "--gate", "--rviz"),
            "sim_cmu_tare": ("start", "--gate", "--rviz"),
        },
    ),
}


def runtime_endpoint(name: str) -> RuntimeEndpointSpec:
    try:
        return RUNTIME_ENDPOINTS[name]
    except KeyError as exc:
        choices = ", ".join(runtime_endpoint_names())
        raise RuntimeEndpointError(f"unknown runtime endpoint '{name}' (choices: {choices})") from exc


def runtime_endpoint_names() -> tuple[str, ...]:
    return tuple(RUNTIME_ENDPOINTS.keys())


def runtime_endpoint_robot_preset(profile: str, endpoint_name: str) -> str:
    endpoint = runtime_endpoint(endpoint_name)
    endpoint.require_profile(profile)
    return endpoint.robot_preset


def apply_runtime_endpoint_config(
    profile: str,
    config: Mapping[str, Any],
    endpoint_name: str,
) -> dict[str, Any]:
    endpoint = runtime_endpoint(endpoint_name)
    merged = dict(config)
    merged.update(endpoint.config_for_profile(profile))
    return merged
