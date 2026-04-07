"""PlannerService — pure planning algorithm orchestration, zero ROS2 dependency.

Three services:
  GoalResolutionService  — Fast/Slow dual-process goal resolution
  FrontierExplorationService — frontier extraction + scoring
  ActionExecutionService — action command generation + LERa recovery

All components injected. Testable with mocks.

Usage::

    svc = GoalResolutionService(resolver=GoalResolver(config))
    result = svc.resolve("find the kitchen", scene_graph_json, robot_pos)

    frontier_svc = FrontierExplorationService(scorer=FrontierScorer())
    best = frontier_svc.evaluate(costmap, robot_pos, "explore", scene_graph)
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Any, Dict, List, Optional

import numpy as np

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# GoalResolutionService
# ---------------------------------------------------------------------------

class GoalResolutionService:
    """Pure goal resolution — Fast Path first, Slow Path fallback.

    Wraps GoalResolver but decouples it from ROS2 node lifecycle.
    The resolver and its LLM clients are injected.
    """

    def __init__(self, resolver: Any):
        """
        Args:
            resolver: GoalResolver instance (has fast_resolve / slow_resolve methods)
        """
        self._resolver = resolver

    def resolve_fast(self, instruction: str, scene_graph_json: str,
                     robot_pos: dict | None = None) -> Any:
        """Fast Path — rule-based scene graph matching (~0.2ms).

        Returns GoalResult or None if confidence too low.
        """
        try:
            return self._resolver.fast_resolve(instruction, scene_graph_json)
        except Exception:
            logger.exception("Fast path resolution failed")
            return None

    async def resolve_slow(self, instruction: str, scene_graph_json: str,
                           robot_pos: dict | None = None) -> Any:
        """Slow Path — LLM reasoning (~2s).

        Returns GoalResult. Requires LLM client configured in resolver.
        """
        try:
            return await self._resolver.slow_resolve(instruction, scene_graph_json)
        except Exception:
            logger.exception("Slow path resolution failed")
            return None

    def resolve(self, instruction: str, scene_graph_json: str,
                robot_pos: dict | None = None) -> Any:
        """Auto — try Fast, return if confident, else None (caller decides on Slow)."""
        result = self.resolve_fast(instruction, scene_graph_json, robot_pos)
        if result is not None and hasattr(result, 'confidence'):
            if result.confidence >= getattr(self._resolver, '_fast_path_threshold', 0.75):
                return result
        return result  # caller can check confidence and escalate to slow

    def resolve_by_tag(self, instruction: str) -> Any:
        """Check tagged locations memory first (instant, no scene graph needed)."""
        if hasattr(self._resolver, '_resolve_by_tag'):
            return self._resolver._resolve_by_tag(instruction)
        return None

    @property
    def resolver(self) -> Any:
        return self._resolver


# ---------------------------------------------------------------------------
# FrontierExplorationService
# ---------------------------------------------------------------------------

class FrontierExplorationService:
    """Frontier extraction + scoring for exploration.

    Wraps FrontierScorer. Injected clip_encoder and semantic_prior are optional.
    """

    def __init__(self, scorer: Any, clip_encoder: Any = None,
                 semantic_prior: Any = None):
        self._scorer = scorer
        if clip_encoder and hasattr(scorer, 'set_clip_encoder'):
            scorer.set_clip_encoder(clip_encoder)
        if semantic_prior and hasattr(scorer, 'set_semantic_prior_engine'):
            scorer.set_semantic_prior_engine(semantic_prior)

    def evaluate(
        self,
        costmap: np.ndarray,
        resolution: float,
        origin_x: float,
        origin_y: float,
        robot_pos: np.ndarray,
        instruction: str = "",
        scene_graph: dict | None = None,
    ) -> Any | None:
        """Extract frontiers, score them, return the best one.

        Returns best Frontier or None if no valid frontiers.
        """
        try:
            self._scorer.update_costmap(costmap, resolution, origin_x, origin_y)
            frontiers = self._scorer.extract_frontiers(robot_pos)
            if not frontiers:
                return None
            self._scorer.score_frontiers(
                instruction=instruction,
                robot_pos=robot_pos,
                scene_graph=scene_graph or {},
            )
            return self._scorer.get_best_frontier()
        except Exception:
            logger.exception("Frontier evaluation failed")
            return None

    def record_failure(self, frontier_pos: np.ndarray) -> None:
        """Record a failed frontier to penalize in future scoring."""
        if hasattr(self._scorer, 'record_failure'):
            self._scorer.record_failure(frontier_pos)

    @property
    def scorer(self) -> Any:
        return self._scorer


# ---------------------------------------------------------------------------
# ActionExecutionService
# ---------------------------------------------------------------------------

class ActionExecutionService:
    """Action command generation + LERa failure recovery.

    Wraps ActionExecutor. LLM client optional (only for LERa recovery).
    """

    def __init__(self, executor: Any, llm_client: Any = None):
        self._executor = executor
        self._llm = llm_client

    def navigate(self, target_pos: np.ndarray, robot_pos: np.ndarray) -> Any:
        """Generate navigate-to-position command."""
        return self._executor.generate_navigate_command(target_pos, robot_pos)

    def approach(self, target_pos: np.ndarray, robot_pos: np.ndarray,
                 stop_distance: float = 1.0) -> Any:
        """Generate approach command (stop at distance)."""
        return self._executor.generate_approach_command(
            target_pos, robot_pos, stop_distance=stop_distance)

    def look_around(self, robot_pos: np.ndarray) -> Any:
        """Generate look-around (360 scan) command."""
        return self._executor.generate_look_around_command(robot_pos)

    async def recover(self, failed_action: str, scene_state: dict,
                      event_loop: Any = None) -> str:
        """LERa 3-step failure recovery using LLM.

        Returns recovery strategy: "retry_different_path" | "expand_search" |
                                   "requery_goal" | "abort"
        """
        if self._llm is None:
            # Rule-based fallback when no LLM
            return "retry_different_path"
        try:
            return await self._executor.lera_recover(
                failed_action=failed_action,
                scene_state=scene_state,
                llm_client=self._llm,
                event_loop=event_loop,
            )
        except Exception:
            logger.exception("LERa recovery failed")
            return "retry_different_path"

    @property
    def executor(self) -> Any:
        return self._executor
