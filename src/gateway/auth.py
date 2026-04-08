"""API Key authentication middleware for GatewayModule.

Security model:
  - API key set via LINGTU_API_KEY env var or robot_config.yaml
  - If no key configured → auth disabled (dev/testing mode)
  - Protected: /api/*, /ws/*, /mcp
  - Public: /, /docs, /redoc, /openapi.json, static assets

Clients send the key as:
  - Header: X-API-Key: <key>
  - Query param: ?api_key=<key>  (for WebSocket/SSE where headers are hard)
  - Cookie: lingtu_api_key=<key> (set by login page)
"""

from __future__ import annotations

import hashlib
import hmac
import logging
import os

from starlette.middleware.base import BaseHTTPMiddleware
from starlette.requests import Request
from starlette.responses import JSONResponse

logger = logging.getLogger(__name__)

# Paths that never require auth
_PUBLIC_PREFIXES = ("/docs", "/redoc", "/openapi.json", "/favicon")
_PUBLIC_EXACT = {"/", "/api/v1/auth/login", "/api/v1/auth/check"}


def _get_configured_key() -> str | None:
    """Read API key from env or config. Returns None if auth is disabled."""
    key = os.environ.get("LINGTU_API_KEY")
    if key:
        return key
    try:
        from core.config import get_config
        cfg = get_config()
        key = cfg.raw.get("gateway", {}).get("api_key")
        if key:
            return str(key)
    except Exception:
        pass
    return None


class APIKeyMiddleware(BaseHTTPMiddleware):
    """Reject requests without a valid API key.

    If no key is configured (LINGTU_API_KEY not set), all requests pass through.
    """

    def __init__(self, app, api_key: str | None = None):
        super().__init__(app)
        self._key = api_key or _get_configured_key()
        if self._key:
            self._key_hash = hashlib.sha256(self._key.encode()).hexdigest()
            logger.info("API key auth enabled (key hash: %s...)", self._key_hash[:8])
        else:
            self._key_hash = None
            logger.info("API key auth disabled (no LINGTU_API_KEY set)")

    async def dispatch(self, request: Request, call_next):
        # No key configured → pass everything
        if self._key_hash is None:
            return await call_next(request)

        path = request.url.path

        # Public paths
        if path in _PUBLIC_EXACT:
            return await call_next(request)
        for prefix in _PUBLIC_PREFIXES:
            if path.startswith(prefix):
                return await call_next(request)
        # Static assets (dashboard)
        if "." in path.split("/")[-1] and not path.startswith("/api"):
            return await call_next(request)

        # Extract key from header, query, or cookie
        client_key = (
            request.headers.get("X-API-Key")
            or request.query_params.get("api_key")
            or request.cookies.get("lingtu_api_key")
        )

        if not client_key:
            return JSONResponse(
                status_code=401,
                content={"error": "unauthorized", "message": "需要 API Key 认证"},
            )

        # Constant-time comparison
        client_hash = hashlib.sha256(client_key.encode()).hexdigest()
        if not hmac.compare_digest(client_hash, self._key_hash):
            return JSONResponse(
                status_code=403,
                content={"error": "forbidden", "message": "API Key 无效"},
            )

        return await call_next(request)
