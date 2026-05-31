"""Exploration stack: TARE backends.

Since 2026-04-25, wavefront frontier has been removed from this stack
(it lives on in `nav.frontier_explorer_module` for the standalone
NavigationModule frontier_explorer option, but is no longer selectable
from the exploration stack).

Backends
--------
``"tare"`` (default) — CMU TARE hierarchical planner (vendored in-tree at
  ``src/exploration/tare_planner``). C++ ROS2 node launched via
  NativeModule. Requires tare_planner colcon package to be built
  (``scripts/build/fetch_ortools.sh`` + ``scripts/build/build_tare.sh``).
  If the binary is missing the stack raises — no silent fallback — so
  misconfiguration is visible, not covered up.

``"none"`` — empty Blueprint (no exploration).
"""

from __future__ import annotations

import logging
import os

from core.blueprint import Blueprint
from core.blueprints.stacks._registry import stack_module

logger = logging.getLogger(__name__)


def exploration(backend: str = "tare", **kw) -> Blueprint:
    """Build an exploration stack Blueprint.

    Args:
        backend: "tare" | "tare_external" | "none". Wavefront is no longer selectable here;
                 use ``exploration(backend="none")`` + NavigationModule's
                 embedded frontier if you need the old pure-Python BFS.
        **kw:    forwarded to the module constructor.
    """
    bp = Blueprint()

    if not backend or backend == "none":
        return bp

    if backend == "tare":
        if not _add_tare(bp, **kw):
            raise RuntimeError(
                "TARE backend requested but binary not available. "
                "Build it with:\n"
                "  scripts/build/fetch_ortools.sh\n"
                "  scripts/build/build_tare.sh\n"
                "Or disable exploration: exploration(backend='none')."
            )
        return bp

    if backend == "tare_external":
        _add_external_tare_bridge(bp, **kw)
        return bp

    raise ValueError(
        f"Unknown exploration backend {backend!r}. "
        "Options: 'tare' (default), 'tare_external', 'none'. "
        "'wavefront' was removed — see nav.frontier_explorer_module for "
        "standalone use."
    )


def _add_tare(bp: Blueprint, **kw) -> bool:
    """Return True iff TARE successfully added. Checks for the compiled
    binary before pulling in the NativeModule factory."""
    try:
        from core.config import get_config
        from core.native_install import exe
        cfg = get_config()
        binary = exe(cfg, "tare_planner", "tare_planner_node")
        if not os.path.exists(binary):
            logger.warning(
                "TARE binary not found at %s — build with "
                "scripts/build/fetch_ortools.sh + build_tare.sh first", binary)
            return False
    except Exception as e:
        logger.debug("TARE pre-check failed: %s", e)
        return False

    try:
        from exploration.native_factories import tare_explorer

        TAREExplorerModule = stack_module(
            "exploration",
            "tare",
            seed_group="exploration",
            fallback="exploration.tare_explorer_module.TAREExplorerModule",
        )
        ExplorationSupervisorModule = stack_module(
            "exploration",
            "supervisor",
            seed_group="exploration",
            fallback=(
                "exploration.exploration_supervisor_module."
                "ExplorationSupervisorModule"
            ),
        )
        scenario = kw.pop("tare_scenario", None)
        if scenario is None:
            scenario = get_config().raw.get("exploration", {}).get(
                "tare_scenario", "forest")
        bp.add(
            tare_explorer(cfg, scenario=scenario),
            alias="TAREPlannerNativeModule",
        )
        bp.add(TAREExplorerModule, alias="TAREExplorerModule", **_tare_kwargs(kw))
        # Supervisor consolidates tare_stats into supervisor_state +
        # fires exploration_ready once TARE is healthy. Autoconnect wires
        # its ``tare_stats: In[dict]`` to TAREExplorerModule's Out, and
        # its ``supervisor_state: Out[dict]`` plus ``tare_stats`` into
        # Gateway for SSE.
        bp.add(
            ExplorationSupervisorModule,
            alias="ExplorationSupervisorModule",
            **_supervisor_kwargs(kw),
        )
        logger.info(
            "TARE exploration stack enabled (scenario=%s, supervisor=on)",
            scenario,
        )
        return True
    except Exception as e:
        logger.warning("TARE module load failed: %s", e)
        return False


def _add_external_tare_bridge(bp: Blueprint, **kw) -> None:
    """Add the TARE bridge without launching LingTu's native TARE process."""
    TAREExplorerModule = stack_module(
        "exploration",
        "tare",
        seed_group="exploration",
        fallback="exploration.tare_explorer_module.TAREExplorerModule",
    )
    ExplorationSupervisorModule = stack_module(
        "exploration",
        "supervisor",
        seed_group="exploration",
        fallback="exploration.exploration_supervisor_module.ExplorationSupervisorModule",
    )

    kw.setdefault("prefer_path_strategy", False)
    kw.setdefault("configured_backend", "tare_external")
    bp.add(TAREExplorerModule, alias="TAREExplorerModule", **_tare_kwargs(kw))
    bp.add(
        ExplorationSupervisorModule,
        alias="ExplorationSupervisorModule",
        **_supervisor_kwargs(kw),
    )
    logger.info("External TARE exploration bridge enabled (native process off)")


def _tare_kwargs(kw: dict) -> dict:
    """Keep only TAREExplorerModule-relevant kwargs."""
    allowed = {
        "way_point_topic", "path_topic", "runtime_topic",
        "finish_topic", "start_topic",
        "configured_backend",
        "goal_frame_id",
        "way_point_timeout_s", "auto_start",
        "hold_active_goal_until_terminal",
        "max_waypoint_distance_m", "waypoint_odometry_timeout_s",
        "prefer_path_strategy",
        "path_goal_min_distance_m", "path_goal_spacing_m",
        "path_start_tolerance_m", "path_max_goal_count",
        "path_strategy_timeout_s", "path_strategy_fallback_to_waypoint",
        "navigation_goal_match_tolerance_m",
    }
    return {k: v for k, v in kw.items() if k in allowed}


def _supervisor_kwargs(kw: dict) -> dict:
    """Keep only ExplorationSupervisorModule-relevant kwargs."""
    mapping = {
        "tare_warn_timeout_s":     "warn_timeout_s",
        "tare_fallback_timeout_s": "fallback_timeout_s",
        "tare_supervisor_hz":      "poll_hz",
    }
    return {dst: kw[src] for src, dst in mapping.items() if src in kw}
