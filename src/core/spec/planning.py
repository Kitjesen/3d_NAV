"""Protocol interfaces for the Planning layer (L4 / L5).

Covers semantic goal resolution, global path planning, and instruction
decomposition.
"""

from __future__ import annotations

from typing import Any, List, Protocol, runtime_checkable


@runtime_checkable
class GoalResolver(Protocol):
    """Semantic goal resolver interface (Fast-Slow dual process).

    The single required method is ``fast_resolve`` -- the fast-path
    scene-graph matching that runs at ~5 Hz.
    """

    def fast_resolve(
        self,
        instruction: str,
        scene_graph_json: str,
    ) -> Any:
        """Attempt to resolve *instruction* against the scene graph.

        Returns a GoalResult on success or ``None`` when the fast path
        cannot resolve with sufficient confidence.
        """
        ...


@runtime_checkable
class GlobalPlanner(Protocol):
    """Global path planner interface.

    Implementations: PCT Planner (C++ ele_planner), pure-Python A*, etc.
    """

    def plan(self, start: Any, goal: Any) -> list:
        """Plan a path from *start* to *goal*.

        Returns a list of waypoints (coordinate tuples or Pose objects).
        """
        ...


@runtime_checkable
class TaskDecomposer(Protocol):
    """Instruction decomposer interface.

    Breaks a natural-language instruction into a sequence of sub-goals
    using rule-based matching (fast) or LLM reasoning (slow).
    """

    def decompose_with_rules(self, instruction: str) -> Any:
        """Decompose *instruction* using rule-based heuristics."""
        ...
