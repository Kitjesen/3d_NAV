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

    AutonomyModule (C++: SLAM + Terrain + LocalPlanner + PathFollower)
                         ↕ DDS
    Camera → DetectorModule → EncoderModule
                         ↓ callback
    PerceptionService(tracker) → SceneGraph
                         ↓ transport (decoupled)
    SemanticPlannerModule ←→ LLMModule
                         ↓
    NavigationModule (plan + track + FSM)
                         ↓
    ThunderDriver → cmd_vel → Robot
                         ↓
    SafetyRingModule (reflex + eval + dialogue)
                         ↓
    GatewayModule (HTTP/WS/SSE) → App/Console
    MCPServerModule (16 MCP tools) → AI Agents
"""

from __future__ import annotations

import logging
from typing import Any

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
        enable_native: include C++ AutonomyModule nodes
        enable_semantic: include perception + semantic planning modules
        enable_gateway: include HTTP gateway and MCP server
        **config: extra config (dog_host, dog_port, etc.)
    """
    cfg = get_config()
    bp = Blueprint()

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

    # ── Layer 2: C++ autonomy stack (NativeModule, DDS) ──────────────────

    if enable_native:
        from base_autonomy.modules.autonomy_module import AutonomyModule
        bp.add(AutonomyModule)

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

    # ── Layer 4: Semantic planning (unified module + LLM) ────────────────

    if enable_semantic:
        try:
            from semantic.planner.semantic_planner.semantic_planner_module import SemanticPlannerModule
            from semantic.planner.semantic_planner.llm_module import LLMModule

            bp.add(SemanticPlannerModule)
            bp.add(LLMModule, backend=llm)
        except ImportError:
            logger.warning("Semantic planner modules not available, skipping")

    # ── Layer 5: Navigation (unified plan + track + FSM) ─────────────────

    from nav.navigation_module import NavigationModule
    bp.add(NavigationModule, planner=planner, tomogram=tomogram)

    # ── Layer 0: Safety ring (reflex + eval + dialogue) ──────────────────

    from nav.safety_ring_module import SafetyRingModule
    bp.add(SafetyRingModule)

    # ── Layer 6: Gateway (HTTP/WS/SSE + MCP server) ──────────────────────

    if enable_gateway:
        try:
            from gateway.gateway_module import GatewayModule
            bp.add(GatewayModule, port=gateway_port)
        except ImportError:
            logger.warning("GatewayModule not available, skipping")

        try:
            from gateway.mcp_server import MCPServerModule
            bp.add(MCPServerModule, port=8090)
        except ImportError:
            logger.warning("MCPServerModule not available, skipping")

    # ── Wiring: 3-tier decoupling ────────────────────────────────────────

    # Tier 1: Safety — direct callback (zero latency)
    bp.wire("SafetyRingModule", "stop_cmd", driver_name, "stop_signal")
    bp.wire("SafetyRingModule", "stop_cmd", "NavigationModule", "stop_signal")

    # Tier 3: Semantic — transport decoupled (crash isolation)
    if enable_semantic:
        try:
            bp.wire("DetectorModule", "detections",
                    "SemanticPlannerModule", "detections",
                    transport="local")
        except (ValueError, KeyError):
            pass

    # Tier 2: Everything else — auto_wire (direct callback)
    bp.auto_wire()

    return bp
