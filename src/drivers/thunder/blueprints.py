"""NOVA Dog blueprints -- three tiers of navigation stack composition.

Usage::

    from drivers.thunder import nova_dog_basic, nova_dog_nav, nova_dog_semantic
    handle = nova_dog_nav(dog_host="192.168.66.190").build()
    handle.start()
"""

from __future__ import annotations

import sys
import os
from typing import Any

_src = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if _src not in sys.path:
    sys.path.insert(0, _src)

from core.blueprint import Blueprint


def _common_modules():
    """Import shared non-driver modules."""
    from nav.rings.nav_rings.safety_module import SafetyModule
    from nav.rings.nav_rings.evaluator_module import EvaluatorModule
    from nav.rings.nav_rings.dialogue_module import DialogueModule
    from global_planning.pct_adapters.src.path_adapter_module import PathAdapterModule
    from global_planning.pct_adapters.src.mission_arc_module import MissionArcModule
    return SafetyModule, EvaluatorModule, DialogueModule, PathAdapterModule, MissionArcModule


def _add_common(bp: Blueprint, driver_name: str, **config) -> Blueprint:
    """Add common modules + standard wiring to a blueprint."""
    Safety, Evaluator, Dialogue, PathAdapter, MissionArc = _common_modules()
    bp.add(Safety)
    bp.add(Evaluator)
    bp.add(PathAdapter)
    bp.add(MissionArc, max_replan_count=config.get("max_replan_count", 3))
    bp.add(Dialogue)
    bp.wire("SafetyModule", "stop_cmd", driver_name, "stop_signal")
    bp.wire("SafetyModule", "stop_cmd", "MissionArcModule", "stop_signal")
    bp.auto_wire()
    return bp


def nova_dog_basic(dog_host: str = "127.0.0.1", dog_port: int = 13145, **kw) -> Blueprint:
    """Basic: driver + safety + path following. No semantic perception."""
    from drivers.thunder.connection import NovaDogConnection
    bp = Blueprint()
    bp.add(NovaDogConnection, dog_host=dog_host, dog_port=dog_port)
    return _add_common(bp, "NovaDogConnection", **kw)


def nova_dog_nav(dog_host: str = "127.0.0.1", dog_port: int = 13145, **kw) -> Blueprint:
    """Navigation: basic + perception + goal resolver + memory."""
    bp = nova_dog_basic(dog_host=dog_host, dog_port=dog_port, **kw)
    try:
        from semantic.perception.semantic_perception.perception_module import PerceptionModule
        from semantic.planner.semantic_planner.goal_resolver_module import GoalResolverModule
        from semantic.planner.semantic_planner.task_decomposer_module import TaskDecomposerModule
        from semantic.planner.semantic_planner.action_executor_module import ActionExecutorModule
        from memory import TopologicalMemoryModule, EpisodicMemoryModule
        bp.add(PerceptionModule)
        bp.add(GoalResolverModule)
        bp.add(TaskDecomposerModule)
        bp.add(ActionExecutorModule)
        bp.add(TopologicalMemoryModule)
        bp.add(EpisodicMemoryModule)
        bp.auto_wire()
    except ImportError:
        pass  # semantic modules optional
    return bp


def nova_dog_semantic(
    dog_host: str = "127.0.0.1",
    dog_port: int = 13145,
    llm_backend: str = "kimi",
    **kw,
) -> Blueprint:
    """Full semantic: nav + frontier exploration + tagged locations."""
    bp = nova_dog_nav(dog_host=dog_host, dog_port=dog_port, **kw)
    try:
        from semantic.planner.semantic_planner.frontier_module import FrontierModule
        from memory import TaggedLocationsModule
        bp.add(FrontierModule)
        bp.add(TaggedLocationsModule)
        bp.auto_wire()
    except ImportError:
        pass
    return bp
