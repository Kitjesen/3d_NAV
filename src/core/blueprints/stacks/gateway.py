"""Gateway stack: HTTP/WebSocket/SSE + MCP server + Teleop + Rerun.

All external interfaces share a single uvicorn process on port 5050:
  GatewayModule  — REST /api/v1/* + SSE /api/v1/events + WS /ws/teleop
  TeleopModule   — camera encoder (pushes JPEG to GatewayModule)
  MCPServerModule — JSON-RPC 2.0 at http://host:8090/mcp (separate port)
  RerunBridgeModule — optional Rerun 3D viz (separate port)
"""

from __future__ import annotations

import logging

from core.blueprint import Blueprint

logger = logging.getLogger(__name__)


def gateway(
    port: int = 5050,
    mcp_port: int = 8090,
    *,
    enable_teleop: bool = True,
    enable_rerun: bool = False,
    rerun_port: int = 9090,
    # teleop_port kept for backwards compat but ignored — teleop is on /ws/teleop
    teleop_port: int = 5050,
) -> Blueprint:
    """Build gateway stack: REST+SSE+WS teleop on one port, MCP on another."""
    bp = Blueprint()

    try:
        from gateway.gateway_module import GatewayModule
        bp.add(GatewayModule, port=port)
    except ImportError:
        logger.warning("GatewayModule not available")

    try:
        from gateway.mcp_server import MCPServerModule
        bp.add(MCPServerModule, port=mcp_port)
    except ImportError:
        logger.warning("MCPServerModule not available")

    if enable_teleop:
        try:
            from drivers.teleop_module import TeleopModule
            bp.add(TeleopModule, port=port)  # informational — same port as Gateway
        except ImportError:
            pass

    if enable_rerun:
        try:
            from gateway.rerun_bridge_module import RerunBridgeModule
            bp.add(RerunBridgeModule, web_port=rerun_port)
        except ImportError:
            logger.warning("RerunBridgeModule not available")

    return bp
