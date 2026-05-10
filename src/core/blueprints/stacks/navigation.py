"""Navigation stack: NavigationModule + optional Python autonomy chain."""

from __future__ import annotations

import logging

from core.blueprint import Blueprint

logger = logging.getLogger(__name__)


def navigation(
    planner_backend: str = "astar",
    tomogram: str = "",
    enable_native: bool = True,
    **config,
) -> Blueprint:
    """Global planning + local autonomy (terrain → local planner → path follower)."""
    bp = Blueprint()
    enable_ros2_bridge = bool(config.get("enable_ros2_bridge", False))

    from nav.navigation_module import NavigationModule
    nav_config = {
        key: config[key]
        for key in (
            "obstacle_thr",
            "waypoint_threshold",
            "final_waypoint_threshold",
            "stuck_timeout",
            "stuck_dist_thre",
            "max_replan_count",
            "downsample_dist",
            "allow_direct_goal_fallback",
            "goal_update_epsilon",
            "mission_timeout",
            "planning_frame_id",
            "planning_timeout",
            "preview_timeout",
            "safe_goal_tolerance",
            "plan_safety_policy",
            "fallback_planner_name",
        )
        if key in config
    }
    bp.add(NavigationModule,
           planner=planner_backend,
           tomogram=tomogram,
           enable_ros2_bridge=enable_ros2_bridge,
           **nav_config)

    if config.get("enable_frontier", False):
        try:
            from nav.frontier_explorer_module import WavefrontFrontierExplorer
            bp.add(WavefrontFrontierExplorer,
                   min_frontier_size=config.get("frontier_min_size", 5),
                   safe_distance=config.get("frontier_safe_distance", 1.0),
                   lookahead_distance=config.get("frontier_lookahead", 5.0),
                   max_explored_distance=config.get("frontier_max_dist", 15.0),
                   info_gain_threshold=config.get("frontier_info_gain", 0.03),
                   goal_timeout=config.get("frontier_goal_timeout", 30.0),
                   explore_rate=config.get("frontier_rate", 2.0))
            bp.wire(
                "WavefrontFrontierExplorer",
                "exploration_goal",
                "NavigationModule",
                "goal_pose",
            )
        except ImportError as e:
            logger.warning("FrontierExplorer not available: %s", e)

    try:
        from base_autonomy.modules.autonomy_module import add_autonomy_stack
        path_follower_config = {
            param: config[key]
            for key, param in (
                ("path_follower_max_speed", "max_speed"),
                ("path_follower_lookahead", "lookahead"),
                ("path_follower_goal_tolerance", "goal_tolerance"),
                ("path_follower_min_speed", "min_speed"),
                ("path_follower_max_yaw_rate", "max_yaw_rate"),
            )
            if key in config
        }
        if enable_native:
            add_autonomy_stack(
                bp,
                backend="cmu",
                path_follower_backend="nav_core",
                **path_follower_config,
            )
        else:
            add_autonomy_stack(
                bp,
                backend=config.get("python_autonomy_backend", "nanobind"),
                path_follower_backend=config.get("python_path_follower_backend", "nav_core"),
                **path_follower_config,
            )
    except ImportError as e:
        logger.warning("Autonomy stack not available: %s", e)

    return bp
