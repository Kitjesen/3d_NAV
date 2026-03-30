"""Gateway stack: HTTP/WS/SSE + MCP server + Teleop."""

from __future__ import annotations

import logging

from core.blueprint import Blueprint

logger = logging.getLogger(__name__)


def gateway(port: int = 5050, mcp_port: int = 8090, teleop_port: int = 5060,
            enable_rerun: bool = False, rerun_port: int = 9090) -> Blueprint:
    """External interfaces: REST API + WebSocket + MCP tools + Teleop + Rerun."""
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

    try:
        from drivers.teleop_module import TeleopModule
        bp.add(TeleopModule, port=teleop_port)
    except ImportError:
        pass

    if enable_rerun:
        try:
            from gateway.rerun_bridge_module import RerunBridgeModule
            bp.add(RerunBridgeModule, web_port=rerun_port)
        except ImportError:
            logger.warning("RerunBridgeModule not available")

    return bp
