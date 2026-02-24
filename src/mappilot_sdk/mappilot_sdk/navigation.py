"""
navigation.py — NavigationSDK

High-level navigation planning interface. Works without ROS2.

Usage:
    from mappilot_sdk import NavigationSDK, SceneGraphInput

    sdk = NavigationSDK()
    sg = SceneGraphInput.from_json(scene_graph_json)
    result = sdk.resolve_goal("找椅子", sg)
    print(result.position)
"""

from __future__ import annotations

import json
import logging
from typing import Any, Dict, List, Optional

import numpy as np

from .types import GoalResult, NavigationCommand, SceneGraphInput

logger = logging.getLogger(__name__)


class NavigationSDK:
    """
    ROS2-free navigation planning SDK.

    Wraps GoalResolver (Fast-Slow dual process), FrontierScorer,
    and TopologySemGraph without any rclpy dependency.

    All algorithm modules from semantic_planner are pure Python and
    can be imported directly on Windows.

    Args:
        llm_client: Optional LLM client for Slow Path resolution.
            If None, only Fast Path is available.
        fast_path_threshold: Confidence threshold for Fast Path.
        language: Instruction language hint ("zh" | "en").
    """

    def __init__(
        self,
        llm_client=None,
        fast_path_threshold: float = 0.75,
        language: str = "zh",
    ):
        self._llm = llm_client
        self._threshold = fast_path_threshold
        self._language = language
        self._resolver = None
        self._topo_graph = None
        self._initialized = False

        self._try_init()

    def _try_init(self) -> None:
        """Lazily import algorithm modules (handles missing optional deps gracefully)."""
        try:
            from semantic_planner.goal_resolver import GoalResolver
            from semantic_planner.llm_client import LLMConfig
            # Use a minimal config; _primary will be replaced if llm_client provided
            config = LLMConfig(backend="openai")
            self._resolver = GoalResolver(primary_config=config)
            if self._llm is not None:
                self._resolver._primary = self._llm
            self._initialized = True
        except ImportError as e:
            logger.warning("GoalResolver not available: %s. Falling back to stub.", e)

        try:
            from semantic_perception.topology_graph import TopologySemGraph
            self._topo_graph = TopologySemGraph()
        except ImportError as e:
            logger.warning("TopologySemGraph not available: %s", e)

    @property
    def is_available(self) -> bool:
        """True if core algorithm modules are loaded."""
        return self._initialized

    # ── Goal Resolution ──────────────────────────────────────────────────────

    def resolve_goal(
        self,
        instruction: str,
        scene_graph: SceneGraphInput,
        robot_position: Optional[np.ndarray] = None,
    ) -> GoalResult:
        """
        Resolve a natural language instruction to a navigation goal.

        Uses Fast Path (pure rule + CLIP matching, ~0.17ms).
        Falls back to Slow Path (LLM) if Fast Path confidence < threshold.

        Args:
            instruction: Natural language instruction ("找椅子", "go to the desk").
            scene_graph: Current scene graph state.
            robot_position: Optional [x, y, z] robot position for spatial reasoning.

        Returns:
            GoalResult with position (if found) and confidence.
        """
        if not self._initialized or self._resolver is None:
            return GoalResult(
                success=False,
                object_id=None,
                label=None,
                position=None,
                confidence=0.0,
                path="failed",
                reasoning="NavigationSDK not initialized (semantic_planner not installed)",
            )

        sg_json = json.dumps(scene_graph.to_dict())

        try:
            result = self._resolver.fast_resolve(instruction, sg_json)
            if result and result.get("confidence", 0) >= self._threshold:
                pos = result.get("position")
                position = np.array(pos, dtype=np.float32) if pos else None
                return GoalResult(
                    success=True,
                    object_id=result.get("object_id"),
                    label=result.get("label"),
                    position=position,
                    confidence=result.get("confidence", 0.0),
                    path="fast",
                    reasoning=result.get("reasoning", ""),
                )
        except Exception as e:
            logger.warning("Fast Path failed: %s", e)

        # Slow Path fallback
        if self._llm is not None:
            return self._resolve_slow(instruction, sg_json)

        return GoalResult(
            success=False,
            object_id=None,
            label=None,
            position=None,
            confidence=0.0,
            path="failed",
            reasoning="Fast Path below threshold, no LLM configured for Slow Path",
        )

    def _resolve_slow(self, instruction: str, sg_json: str) -> GoalResult:
        """Slow Path via LLM."""
        import asyncio
        try:
            loop = asyncio.get_event_loop()
            if loop.is_closed():
                loop = asyncio.new_event_loop()
            result = loop.run_until_complete(
                self._resolver.resolve(instruction, sg_json)
            )
            pos = result.get("position")
            position = np.array(pos, dtype=np.float32) if pos else None
            return GoalResult(
                success=result.get("object_id") is not None,
                object_id=result.get("object_id"),
                label=result.get("label"),
                position=position,
                confidence=result.get("confidence", 0.0),
                path="slow",
                reasoning=result.get("reasoning", ""),
            )
        except Exception as e:
            logger.error("Slow Path failed: %s", e)
            return GoalResult(
                success=False,
                object_id=None,
                label=None,
                position=None,
                confidence=0.0,
                path="failed",
                reasoning=f"LLM error: {e}",
            )

    # ── Topology / Exploration ────────────────────────────────────────────────

    def update_topology(self, scene_graph: SceneGraphInput) -> None:
        """Update internal topology graph from scene graph."""
        if self._topo_graph is not None:
            self._topo_graph.update_from_scene_graph(scene_graph.to_dict())

    def record_robot_position(self, x: float, y: float) -> Optional[int]:
        """Record robot position for traversal memory. Returns current room_id."""
        if self._topo_graph is not None:
            return self._topo_graph.record_robot_position(x, y)
        return None

    def get_exploration_targets(
        self,
        instruction: str,
        top_k: int = 3,
    ) -> List[Dict[str, Any]]:
        """
        Get ranked exploration targets for an instruction.

        Returns list of dicts with keys: node_id, node_name, position, score, reasoning.
        """
        if self._topo_graph is None:
            return []
        targets = self._topo_graph.get_best_exploration_target(instruction, top_k=top_k)
        return [
            {
                "node_id": t.node_id,
                "node_name": t.node_name,
                "node_type": t.node_type,
                "position": t.position.tolist(),
                "score": t.score,
                "reasoning": t.reasoning,
            }
            for t in targets
        ]

    def get_topology_summary(self, language: str = "zh") -> str:
        """Get human-readable topology summary for LLM prompts."""
        if self._topo_graph is None:
            return "topology graph not available"
        return self._topo_graph.to_prompt_context(language=language)
