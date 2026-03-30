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

    from nav.navigation_module import NavigationModule
    bp.add(NavigationModule,
           planner=planner_backend,
           tomogram=tomogram,
           enable_ros2_bridge=not enable_native)

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
        except ImportError as e:
            logger.warning("FrontierExplorer not available: %s", e)

    if enable_native:
        try:
            from base_autonomy.modules.autonomy_module import add_autonomy_stack
            add_autonomy_stack(bp, backend="cmu", path_follower_backend="nav_core")
        except ImportError as e:
            logger.warning("Autonomy stack not available: %s", e)

    return bp
