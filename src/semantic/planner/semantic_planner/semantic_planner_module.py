"""SemanticPlannerModule — unified semantic planning in one Module.

Replaces 4 separate modules (GoalResolver, Frontier, TaskDecomposer, ActionExecutor).
Internal strategies handle different algorithms.

Pipeline:
  instruction → decompose → resolve goal → explore frontiers → execute action

Ports:
  In:  instruction, scene_graph, odometry, detections
  Out: resolved_goal, frontier_goal, task_plan, goal_pose, planner_status

Strategies:
  decomposer: "rules" | "llm"
  resolver:   "fast_slow" (default, Fast Path + Slow Path)
  explorer:   "frontier" (default, frontier scoring)
  executor:   "lera" (default, LERa recovery)

Usage::

    bp.add(SemanticPlannerModule, decomposer="rules")
    bp.add(LLMModule, backend="kimi")  # separate, wired via Blueprint
"""

from __future__ import annotations

import logging
import time
from typing import Any, Dict, Optional

import numpy as np

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", ".."))

from core.module import Module
from core.stream import In, Out
from core.msgs.geometry import PoseStamped, Pose, Vector3, Quaternion
from core.msgs.nav import Odometry
from core.msgs.semantic import SceneGraph, GoalResult as MsgGoalResult
from core.registry import register

logger = logging.getLogger(__name__)


@register("semantic_planner", "default", description="Unified semantic planner module")
class SemanticPlannerModule(Module, layer=4):
    """Unified semantic planner: decompose → resolve → explore → execute.

    Internally composes GoalResolver, FrontierScorer, TaskDecomposer,
    ActionExecutor. Each is a strategy, not a separate Module.
    """

    # -- Inputs --
    instruction: In[str]
    scene_graph: In[SceneGraph]
    odometry: In[Odometry]
    detections: In[list]

    # -- Outputs --
    resolved_goal: Out[PoseStamped]
    frontier_goal: Out[PoseStamped]
    task_plan: Out[dict]
    planner_status: Out[str]

    def __init__(
        self,
        decomposer: str = "rules",
        fast_path_threshold: float = 0.75,
        frontier_score_threshold: float = 0.2,
        max_frontiers: int = 10,
        approach_distance: float = 0.5,
        **kw,
    ):
        super().__init__(**kw)
        self._decomposer_strategy = decomposer
        self._fast_threshold = fast_path_threshold
        self._frontier_threshold = frontier_score_threshold
        self._max_frontiers = max_frontiers
        self._approach_dist = approach_distance

        # Backends (lazy init)
        self._goal_resolver = None
        self._frontier_scorer = None
        self._task_decomposer = None
        self._action_executor = None

        # State
        self._robot_pos = np.zeros(3)
        self._latest_sg: Optional[str] = None
        self._current_instruction = ""
        self._resolve_count = 0
        self._frontier_count = 0

    def setup(self):
        self._init_backends()
        self.instruction.subscribe(self._on_instruction)
        self.scene_graph.subscribe(self._on_scene_graph)
        self.odometry.subscribe(self._on_odom)
        self.detections.subscribe(self._on_detections)

    def _init_backends(self):
        """Lazy-load algorithm backends."""
        # GoalResolver
        try:
            from semantic_planner.goal_resolver import GoalResolver, GoalResolverConfig
            config = GoalResolverConfig(fast_path_threshold=self._fast_threshold)
            self._goal_resolver = GoalResolver(config)
            logger.info("GoalResolver initialized (threshold=%.2f)", self._fast_threshold)
        except ImportError:
            logger.warning("GoalResolver not available")

        # FrontierScorer
        try:
            from semantic_planner.frontier_scorer import FrontierScorer
            self._frontier_scorer = FrontierScorer()
            logger.info("FrontierScorer initialized")
        except ImportError:
            logger.warning("FrontierScorer not available")

        # TaskDecomposer
        try:
            from semantic_planner.task_decomposer import TaskDecomposer
            self._task_decomposer = TaskDecomposer()
            logger.info("TaskDecomposer initialized (strategy=%s)", self._decomposer_strategy)
        except ImportError:
            logger.warning("TaskDecomposer not available")

        # ActionExecutor
        try:
            from semantic_planner.action_executor import ActionExecutor
            self._action_executor = ActionExecutor()
            logger.info("ActionExecutor initialized")
        except ImportError:
            logger.warning("ActionExecutor not available")

    # -- Input handlers ------------------------------------------------------

    def _on_instruction(self, text: str):
        """New instruction → decompose → resolve."""
        self._current_instruction = text
        self.planner_status.publish("PROCESSING")

        # Step 1: Decompose
        plan = self._decompose(text)
        if plan:
            self.task_plan.publish(plan)

        # Step 2: Try fast resolve against current scene graph
        if self._latest_sg:
            self._try_resolve(text, self._latest_sg)

    def _on_scene_graph(self, sg: SceneGraph):
        """Scene graph update → re-resolve if we have an active instruction."""
        sg_json = sg.to_json() if hasattr(sg, 'to_json') else str(sg)
        self._latest_sg = sg_json

        if self._current_instruction and self._goal_resolver:
            self._try_resolve(self._current_instruction, sg_json)

    def _on_odom(self, odom: Odometry):
        self._robot_pos = np.array([odom.x, odom.y, getattr(odom, 'z', 0.0)])

    def _on_detections(self, dets: list):
        """Detection update — could trigger frontier re-evaluation."""
        pass  # consumed by scene_graph path

    # -- Decomposition -------------------------------------------------------

    def _decompose(self, instruction: str) -> Optional[dict]:
        if self._task_decomposer is None:
            return {"subtasks": [instruction]}
        try:
            if self._decomposer_strategy == "rules":
                return self._task_decomposer.decompose_with_rules(instruction)
            else:
                return {"subtasks": [instruction]}
        except Exception:
            logger.exception("Task decomposition failed")
            return {"subtasks": [instruction]}

    # -- Goal Resolution -----------------------------------------------------

    def _try_resolve(self, instruction: str, sg_json: str):
        if self._goal_resolver is None:
            return
        try:
            result = self._goal_resolver.fast_resolve(instruction, sg_json)
            if result and hasattr(result, 'confidence') and result.confidence >= self._fast_threshold:
                self._resolve_count += 1
                pos = result.position if hasattr(result, 'position') else [0, 0, 0]
                if isinstance(pos, (list, tuple)) and len(pos) >= 2:
                    pose = PoseStamped(
                        pose=Pose(position=Vector3(float(pos[0]), float(pos[1]),
                                                    float(pos[2]) if len(pos) > 2 else 0.0),
                                  orientation=Quaternion(0, 0, 0, 1)),
                        frame_id="map", ts=time.time(),
                    )
                    self.resolved_goal.publish(pose)
                    self.planner_status.publish("RESOLVED")
                    return

            # Fast path failed → try frontier exploration
            self._explore_frontier(instruction)
        except Exception:
            logger.exception("Goal resolution failed")
            self.planner_status.publish("FAILED")

    # -- Frontier Exploration ------------------------------------------------

    def _explore_frontier(self, instruction: str):
        if self._frontier_scorer is None:
            self.planner_status.publish("NO_FRONTIER")
            return
        try:
            best = self._frontier_scorer.get_best_frontier()
            if best and hasattr(best, 'position'):
                pos = best.position
                pose = PoseStamped(
                    pose=Pose(position=Vector3(float(pos[0]), float(pos[1]), 0.0),
                              orientation=Quaternion(0, 0, 0, 1)),
                    frame_id="map", ts=time.time(),
                )
                self.frontier_goal.publish(pose)
                self._frontier_count += 1
                self.planner_status.publish("EXPLORING")
            else:
                self.planner_status.publish("NO_FRONTIER")
        except Exception:
            logger.exception("Frontier exploration failed")

    # -- Health --------------------------------------------------------------

    def health(self) -> Dict[str, Any]:
        info = super().port_summary()
        info["semantic_planner"] = {
            "decomposer": self._decomposer_strategy,
            "resolver": self._goal_resolver is not None,
            "frontier": self._frontier_scorer is not None,
            "executor": self._action_executor is not None,
            "resolves": self._resolve_count,
            "frontier_explores": self._frontier_count,
        }
        return info
