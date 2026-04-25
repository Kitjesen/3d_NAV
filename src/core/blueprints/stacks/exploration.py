"""Exploration stack: TARE backend only.

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

logger = logging.getLogger(__name__)


def exploration(backend: str = "tare", **kw) -> Blueprint:
    """Build an exploration stack Blueprint.

    Args:
        backend: "tare" | "none". Wavefront is no longer selectable here;
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

    raise ValueError(
        f"Unknown exploration backend {backend!r}. "
        "Options: 'tare' (default), 'none'. "
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
        from exploration.exploration_supervisor_module import (
            ExplorationSupervisorModule,
        )
        from exploration.native_factories import tare_explorer
        from exploration.tare_explorer_module import TAREExplorerModule
        scenario = kw.pop("tare_scenario", None)
        if scenario is None:
            scenario = get_config().raw.get("exploration", {}).get(
                "tare_scenario", "forest")
        bp.add(tare_explorer(cfg, scenario=scenario))
        bp.add(TAREExplorerModule, **_tare_kwargs(kw))
        # Supervisor consolidates tare_stats into supervisor_state +
        # fires exploration_ready once TARE is healthy. Autoconnect wires
        # its ``tare_stats: In[dict]`` to TAREExplorerModule's Out, and
        # its ``supervisor_state: Out[dict]`` plus ``tare_stats`` into
        # Gateway for SSE.
        bp.add(ExplorationSupervisorModule, **_supervisor_kwargs(kw))
        logger.info(
            "TARE exploration stack enabled (scenario=%s, supervisor=on)",
            scenario,
        )
        return True
    except Exception as e:
        logger.warning("TARE module load failed: %s", e)
        return False


def _tare_kwargs(kw: dict) -> dict:
    """Keep only TAREExplorerModule-relevant kwargs."""
    allowed = {
        "way_point_topic", "path_topic", "runtime_topic",
        "finish_topic", "start_topic",
        "way_point_timeout_s", "auto_start",
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
