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

    if enable_native:
        try:
            from base_autonomy.modules.autonomy_module import add_autonomy_stack
            add_autonomy_stack(bp, backend="cmu", path_follower_backend="nav_core")
        except ImportError as e:
            logger.warning("Autonomy stack not available: %s", e)

    return bp
