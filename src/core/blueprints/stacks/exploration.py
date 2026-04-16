"""Exploration stack: swappable frontier / TARE backend.

Two backends share the same output contract — ``Out[PoseStamped]
exploration_goal`` — so downstream Modules (NavigationModule in particular)
don't need to know which one is active.

Backends
--------
``"wavefront"`` — pure-Python BFS frontier (:mod:`nav.frontier_explorer_module`).
  Zero external deps, good for simulation and constrained hardware.

``"tare"``      — CMU TARE hierarchical planner (vendored in-tree at
  ``src/exploration/tare_planner``). C++ ROS2 node launched via
  NativeModule. Needs the tare_planner colcon package to be built
  (``scripts/build/fetch_ortools.sh`` + ``scripts/build/build_tare.sh``).
  When the binary is missing the stack falls back to ``wavefront`` so
  dev/CI doesn't break.

``"none"``      — empty Blueprint (no exploration).
"""

from __future__ import annotations

import logging
import os

from core.blueprint import Blueprint

logger = logging.getLogger(__name__)


def exploration(backend: str = "wavefront", **kw) -> Blueprint:
    """Build an exploration stack Blueprint.

    Args:
        backend: "wavefront" | "tare" | "none".
        **kw:    forwarded to the module constructor of the chosen backend.
    """
    bp = Blueprint()

    if not backend or backend == "none":
        return bp

    if backend == "wavefront":
        _add_wavefront(bp, **kw)
    elif backend == "tare":
        if not _add_tare(bp, **kw):
            logger.warning(
                "TARE backend unavailable — falling back to wavefront "
                "frontier explorer. Build TARE via "
                "scripts/build/fetch_ortools.sh + build_tare.sh on S100P.")
            _add_wavefront(bp, **kw)
    else:
        raise ValueError(
            f"Unknown exploration backend '{backend}'. "
            "Options: wavefront, tare, none."
        )

    return bp


def _add_wavefront(bp: Blueprint, **kw) -> None:
    try:
        from nav.frontier_explorer_module import WavefrontFrontierExplorer
        bp.add(WavefrontFrontierExplorer, **_wavefront_kwargs(kw))
    except ImportError as e:
        logger.warning("WavefrontFrontierExplorer unavailable: %s", e)


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
        from exploration.tare_explorer_module import TAREExplorerModule
        scenario = kw.pop("tare_scenario", None)
        if scenario is None:
            scenario = get_config().raw.get("exploration", {}).get(
                "tare_scenario", "forest")
        bp.add(tare_explorer(cfg, scenario=scenario))
        bp.add(TAREExplorerModule, **_tare_kwargs(kw))
        logger.info("TARE exploration stack enabled (scenario=%s)", scenario)
        return True
    except Exception as e:
        logger.warning("TARE module load failed: %s", e)
        return False


def _wavefront_kwargs(kw: dict) -> dict:
    """Filter TARE-specific kwargs out of the wavefront path."""
    return {k: v for k, v in kw.items()
            if not k.startswith("tare_")
            and k not in ("auto_start",)}


def _tare_kwargs(kw: dict) -> dict:
    """Keep only TAREExplorerModule-relevant kwargs."""
    allowed = {
        "way_point_topic", "path_topic", "runtime_topic",
        "finish_topic", "start_topic",
        "way_point_timeout_s", "auto_start",
    }
    return {k: v for k, v in kw.items() if k in allowed}
