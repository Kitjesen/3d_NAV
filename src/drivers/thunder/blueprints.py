"""Thunder blueprints — three tiers using new 10-module architecture.

Usage::

    from drivers.thunder.blueprints import thunder_basic, thunder_nav, thunder_semantic
    system = thunder_nav(dog_host="192.168.66.190").build()
    system.start()
"""

from __future__ import annotations

import sys
import os
from typing import Any

_src = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if _src not in sys.path:
    sys.path.insert(0, _src)

from core.blueprint import Blueprint


def thunder_basic(dog_host: str = "127.0.0.1", dog_port: int = 13145, **kw) -> Blueprint:
    """Basic: ThunderDriver + SafetyRing + Navigation. No semantic."""
    from drivers.thunder.han_dog_module import ThunderDriver
    from nav.navigation_module import NavigationModule
    from nav.safety_ring_module import SafetyRingModule

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
    try:
        from semantic.perception.semantic_perception.detector_module import DetectorModule
        from semantic.perception.semantic_perception.encoder_module import EncoderModule
        from semantic.planner.semantic_planner.semantic_planner_module import SemanticPlannerModule
        from semantic.planner.semantic_planner.llm_module import LLMModule

        bp.add(DetectorModule, detector=kw.get("detector", "yoloe"))
        bp.add(EncoderModule, encoder=kw.get("encoder", "mobileclip"))
        bp.add(SemanticPlannerModule)
        bp.add(LLMModule, backend=kw.get("llm", "mock"))
        bp.auto_wire()
    except ImportError:
        pass
    return bp


def thunder_semantic(
    dog_host: str = "127.0.0.1",
    dog_port: int = 13145,
    **kw,
) -> Blueprint:
    """Full semantic: nav + gateway + MCP server."""
    bp = thunder_nav(dog_host=dog_host, dog_port=dog_port, **kw)
    try:
        from gateway.gateway_module import GatewayModule
        bp.add(GatewayModule, port=kw.get("gateway_port", 5050))
        bp.auto_wire()
    except ImportError:
        pass
    try:
        from gateway.mcp_server import MCPServerModule
        bp.add(MCPServerModule, port=kw.get("mcp_port", 8090))
        bp.auto_wire()
    except ImportError:
        pass
    return bp


# Backward-compatible aliases
nova_dog_basic = thunder_basic
nova_dog_nav = thunder_nav
nova_dog_semantic = thunder_semantic
