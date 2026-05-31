"""Central import seeds for built-in registry plugins.

The registry is intentionally explicit: modules register themselves through
``@register`` at import time. This helper keeps those import seeds in one
place so stack factories do not grow their own partial plugin catalogs.
"""

from __future__ import annotations

import importlib
import logging
import sys
from collections.abc import Iterable, Mapping

from core.registry import restore_entries, snapshot

logger = logging.getLogger(__name__)


BUILTIN_PLUGIN_MODULES: Mapping[str, tuple[str, ...]] = {
    "device": (
        "core.devices.manager",
    ),
    "driver": (
        "core.blueprints.stub",
        "drivers.real.thunder.han_dog_module",
        "drivers.real.thunder.connection",
        "drivers.sim.mujoco_driver_module",
        "drivers.sim.ros2_sim_driver",
    ),
    "lidar": (
        "drivers.real.lidar.lidar_module",
    ),
    "camera": (
        "drivers.real.thunder.camera_bridge_module",
    ),
    "teleop": (
        "drivers.teleop_module",
    ),
    "map": (
        "nav.occupancy_grid_module",
        "nav.voxel_grid_module",
        "nav.esdf_module",
        "nav.elevation_map_module",
        "nav.traversability_cost_module",
        "nav.ros2_grid_bridge_module",
        "nav.services.nav_services.map_manager_module",
    ),
    "safety": (
        "nav.safety_ring_module",
        "nav.cmd_vel_mux_module",
        "nav.services.nav_services.geofence_manager_module",
    ),
    "planner_backend": (
        "global_planning.pct_adapters.src.global_planner_module",
    ),
    "navigation": (
        "nav.navigation_module",
        "nav.ros2_waypoint_bridge_module",
        "nav.ros2_path_bridge_module",
        "nav.frontier_explorer_module",
        "nav.traversable_frontier_module",
    ),
    "autonomy": (
        "base_autonomy.modules.terrain_module",
        "base_autonomy.modules.local_planner_module",
        "base_autonomy.modules.path_follower_module",
    ),
    "slam": (
        "slam.slam_module",
        "slam.slam_bridge_module",
        "slam.depth_visual_odom_module",
        "slam.gnss_module",
        "slam.gnss_bridge",
        "slam.ntrip_client_module",
    ),
    "sim_lidar": (
        "drivers.sim.sim_pointcloud_provider",
    ),
    "exploration": (
        "exploration.tare_explorer_module",
        "exploration.tare_ros2_bridge_module",
        "exploration.exploration_supervisor_module",
    ),
    "perception": (
        "semantic.perception.semantic_perception.perception_module",
        "semantic.perception.semantic_perception.detector_module",
        "semantic.perception.semantic_perception.encoder_module",
        "semantic.perception.semantic_perception.api.factory",
    ),
    "reconstruction": (
        "semantic.reconstruction.reconstruction_module",
        "semantic.reconstruction.dataset_recorder_module",
        "semantic.reconstruction.keyframe_exporter_module",
    ),
    "semantic": (
        "semantic.planner.semantic_planner.semantic_planner_module",
        "semantic.planner.semantic_planner.visual_servo_module",
    ),
    "llm": (
        "semantic.planner.semantic_planner.llm_client",
        "semantic.planner.semantic_planner.llm_module",
    ),
    "memory": (
        "memory.modules.semantic_mapper_module",
        "memory.modules.episodic_module",
        "memory.modules.tagged_locations_module",
        "memory.modules.vector_memory_module",
        "memory.modules.temporal_memory_module",
        "memory.modules.mission_logger_module",
        "memory.modules.topological_module",
    ),
    "gateway": (
        "gateway.gateway_module",
        "gateway.mcp_server",
        "gateway.rerun_bridge_module",
    ),
    "visualization": (
        "core.rerun_module",
        "gateway.rerun_bridge_module",
    ),
    "webrtc": (
        "webrtc.webrtc_stream_module",
    ),
}


DEFAULT_BUILTIN_PLUGIN_GROUPS: tuple[str, ...] = (
    "device",
    "driver",
    "lidar",
    "camera",
    "teleop",
    "map",
    "safety",
    "planner_backend",
    "navigation",
    "autonomy",
    "slam",
    "exploration",
    "perception",
    "reconstruction",
    "semantic",
    "llm",
    "memory",
)


def seed_builtin_plugins(
    groups: Iterable[str] | None = None,
    *,
    reload_loaded: bool = False,
    strict: bool = False,
) -> dict[str, dict[str, list[str]]]:
    """Import built-in plugin modules so their ``@register`` decorators run.

    Args:
        groups: Logical seed groups to import. ``None`` imports the safe core
            default groups. Optional runtime surfaces such as Gateway, WebRTC,
            and visualization must be seeded explicitly.
        reload_loaded: Re-run decorators for already imported modules. This is
            mainly useful after tests call ``core.registry.clear()``.
        strict: Re-raise import errors instead of recording them.

    Returns:
        ``{"loaded": {group: [module, ...]}, "failed": {group: ["mod: err"]}}``.
    """
    requested = tuple(groups) if groups is not None else DEFAULT_BUILTIN_PLUGIN_GROUPS
    unknown = [group for group in requested if group not in BUILTIN_PLUGIN_MODULES]
    if unknown:
        available = ", ".join(sorted(BUILTIN_PLUGIN_MODULES))
        raise ValueError(f"Unknown plugin seed group(s): {unknown}. Available: {available}")

    preserved_entries = snapshot()
    report: dict[str, dict[str, list[str]]] = {"loaded": {}, "failed": {}}
    try:
        for group in requested:
            for module_name in BUILTIN_PLUGIN_MODULES[group]:
                try:
                    if reload_loaded and module_name in sys.modules:
                        importlib.reload(sys.modules[module_name])
                    else:
                        importlib.import_module(module_name)
                    report["loaded"].setdefault(group, []).append(module_name)
                except Exception as exc:
                    message = f"{module_name}: {exc.__class__.__name__}: {exc}"
                    report["failed"].setdefault(group, []).append(message)
                    if strict:
                        raise
                    logger.debug("Built-in plugin seed failed for %s", module_name, exc_info=True)
    finally:
        restore_entries(preserved_entries)
    return report
