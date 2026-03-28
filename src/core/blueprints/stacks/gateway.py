"""Gateway stack: HTTP/WS/SSE + MCP server."""

from __future__ import annotations

import logging

from core.blueprint import Blueprint

logger = logging.getLogger(__name__)


def gateway(port: int = 5050, mcp_port: int = 8090) -> Blueprint:
    """External interfaces: REST API + WebSocket + MCP tools."""
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

    return bp
