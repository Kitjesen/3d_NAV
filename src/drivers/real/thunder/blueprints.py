"""Thunder blueprints — three tiers using new 10-module architecture.

This module is a composition layer that wires together modules from across
the LingTu package hierarchy (nav/, semantic/, gateway/). All cross-package
imports are resolved lazily via stack_module() at factory-call time rather
than at module load time, keeping this file's import-time dependencies
limited to the core framework (Blueprint, stack_module).

Usage::

    from drivers.real.thunder.blueprints import thunder_basic, thunder_nav, thunder_semantic
    system = thunder_nav(dog_host="192.168.66.190").build()
    system.start()
"""

from __future__ import annotations

import os
import sys
from typing import Any

from core.blueprints.stacks._registry import optional_stack_module, stack_module

try:
    from core.blueprint import Blueprint
except ModuleNotFoundError as exc:
    if exc.name != "core":
        raise
    _src = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
    _known_paths = {os.path.normcase(os.path.abspath(path or ".")) for path in sys.path}
    if os.path.normcase(_src) not in _known_paths:
        sys.path.insert(0, _src)
    from core.blueprint import Blueprint


def thunder_basic(dog_host: str = "127.0.0.1", dog_port: int = 13145, **kw) -> Blueprint:
    """Basic: ThunderDriver + SafetyRing + Navigation. No semantic."""
    from drivers.real.thunder.han_dog_module import ThunderDriver

    NavigationModule = stack_module(
        "navigation",
        "default",
        seed_group="navigation",
        fallback="nav.navigation_module.NavigationModule",
    )
    SafetyRingModule = stack_module(
        "safety",
        "ring",
        seed_group="safety",
        fallback="nav.safety_ring_module.SafetyRingModule",
    )

    bp = Blueprint()
    bp.add(ThunderDriver, dog_host=dog_host, dog_port=dog_port)
    bp.add(NavigationModule, planner=kw.get("planner", "astar"))
    bp.add(SafetyRingModule)
    bp.wire("SafetyRingModule", "stop_cmd", "ThunderDriver", "stop_signal")
    bp.auto_wire()
    return bp


def thunder_nav(dog_host: str = "127.0.0.1", dog_port: int = 13145, **kw) -> Blueprint:
    """Navigation: basic + perception + semantic planner."""
    bp = thunder_basic(dog_host=dog_host, dog_port=dog_port, **kw)

    DetectorModule = optional_stack_module(
        "detector",
        "pluggable",
        seed_group="perception",
        fallback="semantic.perception.detector_module.DetectorModule",
    )
    EncoderModule = optional_stack_module(
        "encoder",
        "pluggable",
        seed_group="perception",
        fallback="semantic.perception.encoder_module.EncoderModule",
    )
    SemanticPlannerModule = optional_stack_module(
        "semantic_planner",
        "default",
        seed_group="semantic",
        fallback="semantic.planner.semantic_planner_module.SemanticPlannerModule",
    )
    LLMModule = optional_stack_module(
        "llm",
        "pluggable",
        seed_group="llm",
        fallback="semantic.planner.llm_module.LLMModule",
    )

    # Preserve original all-or-nothing semantics: only add if all 4 resolved
    if DetectorModule and EncoderModule and SemanticPlannerModule and LLMModule:
        bp.add(DetectorModule, detector=kw.get("detector", "yoloe"))
        bp.add(EncoderModule, encoder=kw.get("encoder", "mobileclip"))
        bp.add(SemanticPlannerModule)
        bp.add(LLMModule, backend=kw.get("llm", "mock"))
        bp.auto_wire()

    return bp


def thunder_semantic(
    dog_host: str = "127.0.0.1",
    dog_port: int = 13145,
    **kw,
) -> Blueprint:
    """Full semantic: nav + gateway + MCP server."""
    bp = thunder_nav(dog_host=dog_host, dog_port=dog_port, **kw)

    GatewayModule = optional_stack_module(
        "gateway",
        "fastapi",
        seed_group="gateway",
        fallback="gateway.gateway_module.GatewayModule",
    )
    if GatewayModule is not None:
        bp.add(GatewayModule, port=kw.get("gateway_port", 5050))
        bp.auto_wire()

    MCPServerModule = optional_stack_module(
        "mcp",
        "server",
        seed_group="gateway",
        fallback="gateway.mcp_server.MCPServerModule",
    )
    if MCPServerModule is not None:
        bp.add(MCPServerModule, port=kw.get("mcp_port", 8090))
        bp.auto_wire()

    return bp


# Backward-compatible aliases
nova_dog_basic = thunder_basic
nova_dog_nav = thunder_nav
nova_dog_semantic = thunder_semantic
