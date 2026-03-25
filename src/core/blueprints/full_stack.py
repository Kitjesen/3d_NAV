"""Full-stack navigation blueprint — complete pluggable pipeline.

Every component is swappable via config arguments. One call starts everything.

Usage::

    # Real robot (S100P)
    system = full_stack_blueprint(
        robot="thunder", slam_profile="fastlio2",
        detector="bpu", encoder="mobileclip", llm="kimi",
    ).build()
    system.start()

    # Simulation
    system = full_stack_blueprint(
        robot="stub", detector="yoloe", llm="mock",
    ).build()

    # CI / testing
    system = full_stack_blueprint(
        robot="stub", enable_native=False, llm="mock",
    ).build()

Pipeline::

    SLAM(C++) → Terrain(C++) → LocalPlanner(C++)    ← NativeModule (DDS)
                     ↕ DDS
    Camera → DetectorModule → EncoderModule
                     ↓ callback
    PerceptionService(tracker) → SceneGraph
                     ↓ transport (decoupled)
    GoalResolverModule ←→ LLMModule
                     ↓
    GlobalPlannerModule → PathAdapterModule → MissionArcModule
                     ↓
    ThunderDriver → cmd_vel → Robot
                     ↓
    SafetyModule ←→ EvaluatorModule → DialogueModule
                     ↓
    GatewayModule (HTTP/WS/SSE) → App/Console
"""

from __future__ import annotations

import logging
import os
import sys
from typing import Any

_src_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if _src_dir not in sys.path:
    sys.path.insert(0, _src_dir)

from core.blueprint import Blueprint
from core.config import get_config
from core.registry import get, auto_select

logger = logging.getLogger(__name__)


def _ensure_drivers_registered():
    try:
        import drivers.thunder.han_dog_module  # noqa: F401
    except ImportError:
        pass
    try:
        import core.blueprints.stub  # noqa: F401
    except ImportError:
        pass


def full_stack_blueprint(
    robot: str = "thunder",
    slam_profile: str = "fastlio2",
    detector: str = "yoloe",
    encoder: str = "mobileclip",
    llm: str = "kimi",
    planner: str = "astar",
    tomogram: str = "",
    gateway_port: int = 5050,
    enable_native: bool = True,
    enable_semantic: bool = True,
    enable_gateway: bool = True,
    **config: Any,
) -> Blueprint:
    """Build the complete LingTu navigation stack.

    All components are pluggable via arguments. Modules communicate via
    In/Out ports with 3-tier decoupling (callback / transport / DDS).

    Args:
        robot: Driver ("thunder", "sim_mujoco", "stub", "auto")
        slam_profile: "fastlio2" or "pointlio"
        detector: "yoloe", "yolo_world", "bpu", "grounding_dino"
        encoder: "clip", "mobileclip"
        llm: "kimi", "openai", "claude", "qwen", "mock"
        planner: "astar", "pct"
        tomogram: path to tomogram .pickle for global planner
        gateway_port: FastAPI gateway port
        enable_native: include C++ NativeModule nodes
        enable_semantic: include perception + planning modules
        enable_gateway: include HTTP gateway
        **config: extra config (dog_host, dog_port, etc.)
    """
    cfg = get_config()
    bp = Blueprint()

    # ── Layer 1-2: C++ native nodes (NativeModule, DDS) ─────────────────

    if enable_native:
        from core.native_factories import (
            terrain_analysis, terrain_analysis_ext,
            local_planner, path_follower,
            slam_fastlio2, slam_pointlio,
        )
        bp.add(terrain_analysis(cfg), alias="terrain")
        bp.add(terrain_analysis_ext(cfg), alias="terrain_ext")
        bp.add(local_planner(cfg), alias="local_planner")
        bp.add(path_follower(cfg), alias="path_follower")

        if slam_profile == "pointlio":
            bp.add(slam_pointlio(cfg), alias="slam")
        else:
            bp.add(slam_fastlio2(cfg), alias="slam")

    # ── Layer 1: Robot driver (pluggable via registry) ───────────────────

    _ensure_drivers_registered()
    if robot == "auto":
        import platform
        arch = platform.machine().lower()
        robot = auto_select("driver", platform=arch) or "stub"
        logger.info("Auto-selected driver: %s", robot)

    DriverCls = get("driver", robot)
    bp.add(DriverCls,
           dog_host=config.get("dog_host", cfg.driver.dog_host),
           dog_port=config.get("dog_port", cfg.driver.dog_port))
    driver_name = DriverCls.__name__

    # ── Layer 0: Safety ring ─────────────────────────────────────────────

    from nav.rings.nav_rings.safety_module import SafetyModule
    from nav.rings.nav_rings.evaluator_module import EvaluatorModule
    from nav.rings.nav_rings.dialogue_module import DialogueModule

    bp.add(SafetyModule)
    bp.add(EvaluatorModule)
    bp.add(DialogueModule)

    # ── Layer 3: Perception (pluggable detector + encoder) ───────────────

    if enable_semantic:
        try:
            from semantic.perception.semantic_perception.detector_module import DetectorModule
            from semantic.perception.semantic_perception.encoder_module import EncoderModule

            bp.add(DetectorModule, detector=detector,
                   confidence=config.get("confidence", 0.3))
            bp.add(EncoderModule, encoder=encoder)
        except ImportError:
            logger.warning("Perception modules not available, skipping")

    # ── Layer 4: Planning (pluggable LLM + global planner) ───────────────

    if enable_semantic:
        try:
            from semantic.planner.semantic_planner.llm_module import LLMModule
            from semantic.planner.semantic_planner.goal_resolver_module import GoalResolverModule
            from semantic.planner.semantic_planner.frontier_module import FrontierModule
            from semantic.planner.semantic_planner.action_executor_module import ActionExecutorModule
            from semantic.planner.semantic_planner.task_decomposer_module import TaskDecomposerModule

            bp.add(LLMModule, backend=llm)
            bp.add(GoalResolverModule)
            bp.add(FrontierModule)
            bp.add(ActionExecutorModule)
            bp.add(TaskDecomposerModule)
        except ImportError:
            logger.warning("Planner modules not available, skipping")

    # ── Layer 5: Path orchestration ──────────────────────────────────────

    from global_planning.pct_adapters.src.path_adapter_module import PathAdapterModule
    from global_planning.pct_adapters.src.mission_arc_module import MissionArcModule
    from global_planning.pct_adapters.src.global_planner_module import GlobalPlannerModule

    bp.add(GlobalPlannerModule, planner=planner, tomogram=tomogram)
    bp.add(PathAdapterModule)
    bp.add(MissionArcModule,
           max_replan_count=config.get("max_replan_count", 3))

    # ── Layer 6: Gateway (HTTP/WS/SSE) ───────────────────────────────────

    if enable_gateway:
        try:
            from gateway.gateway_module import GatewayModule
            bp.add(GatewayModule, port=gateway_port)
        except ImportError:
            logger.warning("GatewayModule not available, skipping")

    # ── Wiring: 3-tier decoupling ────────────────────────────────────────

    # Tier 1: Safety — direct callback (zero latency)
    bp.wire("SafetyModule", "stop_cmd", driver_name, "stop_signal")
    bp.wire("SafetyModule", "stop_cmd", "MissionArcModule", "stop_signal")

    # Tier 3: Semantic — transport decoupled (crash isolation)
    if enable_semantic:
        try:
            bp.wire("DetectorModule", "detections",
                    "GoalResolverModule", "detections",
                    transport="local")
        except (ValueError, KeyError):
            pass

    # Tier 2: Everything else — auto_wire (direct callback)
    bp.auto_wire()

    return bp
