"""Static asset routes for the Gateway dashboard."""

from __future__ import annotations

import logging
import os

logger = logging.getLogger(__name__)


def mount_dashboard_assets(app) -> None:
    """Serve the built React dashboard at the root path when available."""
    web_dist = os.path.normpath(
        os.path.join(os.path.dirname(__file__), "..", "..", "..", "web", "dist")
    )
    if not os.path.isdir(web_dist):
        return

    from starlette.staticfiles import StaticFiles
    from starlette.types import Receive, Scope, Send

    inner_app = StaticFiles(directory=web_dist, html=True)

    async def no_cache_html(scope: Scope, receive: Receive, send: Send) -> None:
        async def send_with_headers(message: dict) -> None:
            if message.get("type") == "http.response.start":
                path = scope.get("path", "")
                if not path.startswith("/assets/"):
                    raw = list(message.get("headers", []))
                    raw.append((b"cache-control", b"no-cache, no-store, must-revalidate"))
                    message = {**message, "headers": raw}
            await send(message)

        await inner_app(scope, receive, send_with_headers)

    app.mount("/", no_cache_html, name="dashboard")
    logger.info("Dashboard served from %s", web_dist)
