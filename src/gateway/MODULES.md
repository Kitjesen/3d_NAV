# Gateway — HTTP/WS/SSE API + MCP Server Index

> Files live under `src/gateway/`
> 30+ .py files including subdirectories (excluding tests)

---

## Top-Level (6)

| File | Responsibility |
|------|---------------|
| `__init__.py` | Package init — exports public API symbols |
| `gateway_module.py` | **GatewayModule** — FastAPI HTTP/WS/SSE server, drift watchdog, save hooks, session lifecycle |
| `mcp_server.py` | **MCPServerModule** — MCP JSON-RPC tools auto-discovered from @skill methods |
| `auth.py` | Authentication helpers — token validation, API key checking |
| `schemas.py` | Pydantic request/response schemas for all REST endpoints |
| `map_dashboard.py` | Map dashboard UI — web-based map visualization |
| `rerun_bridge_module.py` | **RerunBridgeModule** — 3D telemetry streamed to Rerun viewer |
| `native_factories.py` | C++ native module factory for gateway-related binaries |

---

## Routes — `routes/` (11)

| File | Responsibility |
|------|---------------|
| `__init__.py` | Package init |
| `app.py` | FastAPI app factory — middleware, exception handlers, CORS |
| `auth.py` | `/api/v1/auth` — login, token refresh |
| `commands.py` | `/api/v1/commands` — navigation commands (go, stop, follow) |
| `status.py` | `/api/v1/status` — robot state, module health |
| `maps.py` | `/api/v1/maps` — map list, save, load, delete |
| `session.py` | `/api/v1/session` — mission session lifecycle |
| `operations.py` | `/api/v1/operations` — operational control endpoints |
| `realtime.py` | `/api/v1/realtime` — SSE event stream for live updates |
| `camera.py` | `/api/v1/camera` — camera stream access |
| `diagnostics.py` | `/api/v1/diagnostics` — system diagnostics endpoints |
| `assets.py` | Static asset serving for web dashboard |

---

## Services — `services/` (13)

| File | Responsibility |
|------|---------------|
| `__init__.py` | Package init |
| `app_bootstrap.py` | Application bootstrap — gateway startup sequence |
| `commands.py` | Command dispatcher — routes incoming commands to modules |
| `control_commands.py` | Control command primitives — go/stop/follow/tag |
| `goal_builder.py` | Goal construction — parse semantic targets into NavGoal |
| `map_paths.py` | Map file path management — locate, resolve, validate map paths |
| `map_safety.py` | Map safety checks — validate map before loading |
| `media_status.py` | Media stream status — camera, recording state |
| `readiness.py` | Readiness probe — health check endpoint logic |
| `runtime_dataflow.py` | Runtime data flow — module event stream aggregation |
| `runtime_status.py` | Runtime status — consolidated robot status report |
| `runtime_switch_plan.py` | Runtime switch plan — profile switching orchestration |
| `safety_status.py` | Safety status — SafetyRing state reporting |
| `state_snapshot.py` | State snapshot — point-in-time system state capture |
| `telemetry_normalizers.py` | Telemetry normalization — unit conversion, frame transforms |
| `traffic.py` | Traffic management — rate limiting, request throttling |

## Scripts — `scripts/` (1)

| File | Responsibility |
|------|---------------|
| `ble_peripheral.py` | BLE peripheral — mobile app companion advertisement |
