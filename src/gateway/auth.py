"""API Key authentication middleware for GatewayModule.

Implemented as a **pure ASGI middleware** (not ``BaseHTTPMiddleware``) so
that it works cleanly with both HTTP and WebSocket scopes. Starlette's
``BaseHTTPMiddleware`` has known issues with WebSocket upgrades in some
versions (returns HTTP 403 before the WebSocket handler is reached, see
https://github.com/encode/starlette/issues/1012 and related).

Security model:
  - API key set via ``LINGTU_API_KEY`` env var or ``robot_config.yaml``
  - If no key configured → auth disabled (dev/testing mode, pass-through)
  - Protected: ``/api/*``, ``/ws/*``, ``/mcp``
  - Public: ``/``, ``/docs``, ``/redoc``, ``/openapi.json``, static assets,
    ``/api/v1/auth/login``, ``/api/v1/auth/check``

Clients send the key as:
  - Header: ``X-API-Key: <key>``
  - Query param: ``?api_key=<key>`` (for WebSocket / SSE where setting
    headers is inconvenient)
  - Cookie: ``lingtu_api_key=<key>`` (set by the login page, HTTP only)

On auth failure:
  - HTTP → JSON response with 401/403
  - WebSocket → ``websocket.close`` with code 4401/4403
"""

from __future__ import annotations

import hashlib
import hmac
import json
import logging
import os
from urllib.parse import parse_qs

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


class APIKeyMiddleware:
    """Pure ASGI middleware for API key authentication.

    Works with both HTTP and WebSocket scopes. When auth is disabled (no
    configured key), every request passes through unchanged.
    """

    def __init__(self, app, api_key: str | None = None):
        self.app = app
        self._key = api_key or _get_configured_key()
        if self._key:
            self._key_hash: str | None = hashlib.sha256(
                self._key.encode()
            ).hexdigest()
            logger.info(
                "API key auth enabled (key hash: %s...)",
                self._key_hash[:8],
            )
        else:
            self._key_hash = None
            logger.info("API key auth disabled (no LINGTU_API_KEY set)")

    async def __call__(self, scope, receive, send):
        # Lifespan and other scope types: pass through.
        if scope["type"] not in ("http", "websocket"):
            return await self.app(scope, receive, send)

        # Auth disabled → everything passes.
        if self._key_hash is None:
            return await self.app(scope, receive, send)

        path = scope.get("path", "")

        # Public paths (exact match).
        if path in _PUBLIC_EXACT:
            return await self.app(scope, receive, send)

        # Public path prefixes (docs, static).
        for prefix in _PUBLIC_PREFIXES:
            if path.startswith(prefix):
                return await self.app(scope, receive, send)

        # Static dashboard assets: files with an extension, not under /api.
        last_segment = path.rsplit("/", 1)[-1] if path else ""
        if "." in last_segment and not path.startswith("/api"):
            return await self.app(scope, receive, send)

        # Extract API key from headers / query / cookies.
        client_key = self._extract_key(scope)

        if not client_key:
            return await self._reject(scope, send, 401, "需要 API Key 认证")

        client_hash = hashlib.sha256(client_key.encode()).hexdigest()
        if not hmac.compare_digest(client_hash, self._key_hash):
            return await self._reject(scope, send, 403, "API Key 无效")

        return await self.app(scope, receive, send)

    # ------------------------------------------------------------------ helpers

    @staticmethod
    def _extract_key(scope) -> str | None:
        """Return the API key from headers, query params, or cookies (or None)."""
        # ASGI headers are a list of (bytes, bytes) tuples.
        raw_headers = scope.get("headers", [])
        headers = {
            k.decode("latin-1").lower(): v.decode("latin-1")
            for k, v in raw_headers
        }

        # 1. X-API-Key header
        key = headers.get("x-api-key")
        if key:
            return key

        # 2. Query param ?api_key=
        query = scope.get("query_string", b"").decode("latin-1")
        if query:
            parsed = parse_qs(query)
            if "api_key" in parsed and parsed["api_key"]:
                return parsed["api_key"][0]

        # 3. Cookie lingtu_api_key=
        cookie_header = headers.get("cookie", "")
        if cookie_header:
            for cookie in cookie_header.split(";"):
                cookie = cookie.strip()
                if cookie.startswith("lingtu_api_key="):
                    return cookie[len("lingtu_api_key=") :]

        return None

    @staticmethod
    async def _reject(scope, send, status_code: int, message: str) -> None:
        """Reject a request. HTTP → JSON body; WebSocket → close with 4xxx."""
        if scope["type"] == "http":
            body = json.dumps(
                {
                    "error": "unauthorized" if status_code == 401 else "forbidden",
                    "message": message,
                }
            ).encode()
            await send(
                {
                    "type": "http.response.start",
                    "status": status_code,
                    "headers": [
                        (b"content-type", b"application/json; charset=utf-8"),
                        (b"content-length", str(len(body)).encode()),
                    ],
                }
            )
            await send(
                {
                    "type": "http.response.body",
                    "body": body,
                }
            )
        else:
            # WebSocket: custom close codes in the 4xxx range.
            close_code = 4401 if status_code == 401 else 4403
            reason = message.encode("utf-8")[:123].decode("utf-8", errors="ignore")
            await send(
                {
                    "type": "websocket.close",
                    "code": close_code,
                    "reason": reason,
                }
            )
