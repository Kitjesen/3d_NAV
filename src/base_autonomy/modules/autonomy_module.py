"""AutonomyModule — convenience wrapper for the 3 autonomy sub-modules.

Instead of a monolithic black box, the autonomy stack is now 3 independent Modules:
  - TerrainModule:       traversability analysis (In: odometry, map_cloud → Out: terrain_map)
  - LocalPlannerModule:  obstacle avoidance (In: terrain_map, waypoint → Out: local_path)
  - PathFollowerModule:  path tracking (In: local_path → Out: cmd_vel)

Each is independently pluggable via Registry. AutonomyModule remains as a convenience
to add all 3 at once with a single blueprint call.

Usage::

    # Option A: add individually (more control)
    bp.add(TerrainModule, backend="cmu")
    bp.add(LocalPlannerModule, backend="cmu")
    bp.add(PathFollowerModule, backend="pure_pursuit")

    # Option B: convenience wrapper (same result)
    from base_autonomy import add_autonomy_stack
    add_autonomy_stack(bp, backend="cmu")
"""

from __future__ import annotations

import logging
from typing import Any

from core.blueprint import Blueprint
from core.registry import register

from .local_planner_module import LocalPlannerModule
from .path_follower_module import PathFollowerModule
from .terrain_module import TerrainModule

logger = logging.getLogger(__name__)

__all__ = [
    "LocalPlannerModule",
    "PathFollowerModule",
    "TerrainModule",
    "add_autonomy_stack",
]


def add_autonomy_stack(
    bp: Blueprint,
    backend: str = "cmu",
    path_follower_backend: str = "pure_pursuit",
    **kw,
) -> Blueprint:
    """Add the full autonomy stack (terrain + local_planner + path_follower) to a Blueprint.

    Args:
        bp: Blueprint to add modules to.
        backend: Backend for TerrainModule and LocalPlannerModule ("cmu" or "simple").
        path_follower_backend: Backend for PathFollowerModule ("pure_pursuit" or "pid").

    Returns:
        The same Blueprint (for chaining).
    """
    bp.add(TerrainModule, backend=backend)
    bp.add(LocalPlannerModule, backend=backend)
    bp.add(PathFollowerModule, backend=path_follower_backend, **kw)
    return bp
