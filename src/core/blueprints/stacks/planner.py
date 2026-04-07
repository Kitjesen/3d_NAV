"""Planner stack: SemanticPlanner + LLM + VisualServo."""

from __future__ import annotations

import logging

from core.blueprint import Blueprint

from .memory import DEFAULT_SEMANTIC_DIR

logger = logging.getLogger(__name__)


def planner(llm: str = "kimi", save_dir: str = "", **config) -> Blueprint:
    """Semantic planning: goal resolution + LLM reasoning + visual servo."""
    bp = Blueprint()
    save_dir = save_dir or DEFAULT_SEMANTIC_DIR

    try:
        from semantic.planner.semantic_planner.llm_module import LLMModule
        from semantic.planner.semantic_planner.semantic_planner_module import SemanticPlannerModule
        bp.add(SemanticPlannerModule, save_dir=save_dir)
        bp.add(LLMModule, backend=llm)
    except ImportError as e:
        logger.warning("Semantic planner not available: %s", e)

    try:
        from semantic.planner.semantic_planner.visual_servo_module import VisualServoModule
        bp.add(VisualServoModule)
    except ImportError:
        pass

    return bp
