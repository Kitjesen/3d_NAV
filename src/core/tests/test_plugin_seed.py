from __future__ import annotations

import sys

from core.registry import clear, list_plugins, restore, snapshot


def _restore_import_state(module_names: set[str], before: set[str]) -> None:
    """Remove seed-imported modules that were not loaded before this test."""
    for module_name in module_names:
        if module_name in before:
            continue
        sys.modules.pop(module_name, None)


def test_builtin_plugin_seed_restores_core_plugin_surfaces_after_clear():
    from core.plugin_seed import BUILTIN_PLUGIN_MODULES, seed_builtin_plugins

    saved = snapshot()
    seed_modules = {module for modules in BUILTIN_PLUGIN_MODULES.values() for module in modules}
    modules_before = set(sys.modules)
    try:
        clear()

        report = seed_builtin_plugins(
            groups=(
                "driver",
                "lidar",
                "map",
                "planner_backend",
                "autonomy",
                "slam",
                "exploration",
                "perception",
                "llm",
            ),
            reload_loaded=True,
        )

        assert set(BUILTIN_PLUGIN_MODULES) >= {
            "driver",
            "lidar",
            "map",
            "planner_backend",
            "autonomy",
            "slam",
            "exploration",
            "perception",
            "llm",
        }
        assert set(report) == {"loaded", "failed"}

        assert {"stub", "thunder", "sim_mujoco", "sim_ros2", "nova_dog"} <= set(
            list_plugins("driver")
        )
        assert {"lidar_mid360"} <= set(list_plugins("driver"))
        assert {
            "occupancy_grid",
            "voxel",
            "esdf",
            "elevation",
            "traversability_cost",
        } <= set(list_plugins("map"))
        assert {"pct", "astar"} <= set(list_plugins("planner_backend"))
        assert {"nanobind", "native", "cmu", "simple"} <= set(list_plugins("terrain"))
        assert {"nanobind", "cmu", "cmu_py", "simple"} <= set(
            list_plugins("local_planner")
        )
        assert {"nav_core", "pure_pursuit", "pid"} <= set(
            list_plugins("path_follower")
        )
        assert {"fastlio2", "pointlio", "localizer", "genz"} <= set(
            list_plugins("slam")
        )
        assert {"default"} <= set(list_plugins("slam_bridge"))
        assert {"tare", "supervisor"} <= set(list_plugins("exploration"))
        assert {"yoloe", "yolo_world", "bpu", "sim_scene"} <= set(
            list_plugins("perception_detector")
        )
        assert {"clip", "mobileclip"} <= set(list_plugins("perception_encoder"))
        assert {"openai", "claude", "qwen", "moonshot", "mock"} <= set(
            list_plugins("llm_client")
        )
    finally:
        restore(saved)
        _restore_import_state(seed_modules, modules_before)


def test_builtin_plugin_seed_can_seed_one_group_without_loading_unrelated_groups():
    from core.plugin_seed import BUILTIN_PLUGIN_MODULES, seed_builtin_plugins

    saved = snapshot()
    seed_modules = {module for modules in BUILTIN_PLUGIN_MODULES.values() for module in modules}
    modules_before = set(sys.modules)
    try:
        clear()

        seed_builtin_plugins(groups=("planner_backend",), reload_loaded=True)

        assert {"pct", "astar"} <= set(list_plugins("planner_backend"))
        assert list_plugins("driver") == []
        assert list_plugins("perception_detector") == []
    finally:
        restore(saved)
        _restore_import_state(seed_modules, modules_before)
