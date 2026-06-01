# Routes — FastAPI HTTP/WS/SSE Endpoints

This package provides the route modules that register HTTP, WebSocket, and SSE endpoints on the Gateway's FastAPI application. Each route module handles a specific functional domain.

## Files

- **`app.py`** — FastAPI application factory: creates the ASGI app with middleware, CORS, exception handlers, and route registration.
- **`maps.py`** — Map REST endpoints: list, save, load, delete, and build maps; map lifecycle management.
- **`commands.py`** — Command endpoints: navigate_to, stop, cancel, set_mode; dispatches to NavigationModule.
- **`realtime.py`** — SSE (Server-Sent Events) streaming: real-time robot state, map cloud, and health status push.
- **`camera.py`** — Camera streaming endpoints: JPEG frame fetch and MJPEG stream for WebSocket/HTTP clients.
- **`session.py`** — Session management: start/stop robot session, session state query, and config overrides.
- **`status.py`** — Health and status endpoints: module state, connection status, uptime, and error counts.
- **`diagnostics.py`** — Diagnostic endpoints: system logs, component health checks, and calibration verification.
- **`operations.py`** — Operations endpoints: restart services, toggle features, and env var inspection.
- **`assets.py`** — Static asset serving: frontend assets, robot icons, and UI configuration files.
- **`auth.py`** — Authentication middleware: API key validation and token-based access control for endpoints.
