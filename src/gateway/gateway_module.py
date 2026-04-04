"""GatewayModule — enterprise-grade FastAPI gateway.

Single uvicorn process, shared port 5050.  All external interfaces live here:
  REST API   — typed Pydantic v2 request/response models, validation errors → 422
  SSE        — thread-safe asyncio.Queue fan-out, one queue per connected client
  WebSocket  — teleop joystick + camera stream (replaces separate TeleopModule WS)
  MCP        — JSON-RPC 2.0 endpoint (served by MCPServerModule on port 8090)

Architecture
------------
Module threads write to _state (protected by RLock) and push events via
push_event() (thread-safe).  FastAPI coroutines read _state and dequeue
events — no shared mutable state between threads and coroutines except
through the explicit synchronisation primitives below.

Endpoints
---------
REST
  POST /api/v1/goal          {x,y,z?,instruction?}
  POST /api/v1/cmd_vel       {vx,vy?,wz}
  POST /api/v1/stop
  POST /api/v1/instruction   {text}
  POST /api/v1/mode          {mode: manual|autonomous|estop}
  POST /api/v1/lease         {action: acquire|release|renew, client_id, ttl?}
  POST /api/v1/maps          {action: list|save|use|build|delete|rename, name?, new_name?}
  GET  /api/v1/state         full snapshot (odom, safety, mission, mode, lease)
  GET  /api/v1/scene_graph
  GET  /api/v1/health
SSE
  GET  /api/v1/events        event stream  (application/x-ndjson, chunked)
WebSocket
  WS   /ws/teleop            {type:joy, lx,ly,az} | {type:stop}
                             ← binary JPEG camera frames

Blueprint usage::

    bp.add(GatewayModule, port=5050)
"""

from __future__ import annotations

import asyncio
import json
import logging
import queue
import threading
import time
from typing import Any, Dict, List, Optional

from pydantic import BaseModel, Field, field_validator

from core.module import Module
from core.stream import In, Out
from core.msgs.geometry import Pose, PoseStamped, Quaternion, Twist, Vector3
from core.msgs.nav import Odometry
from core.msgs.semantic import ExecutionEval, SafetyState, SceneGraph
from core.registry import register

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Pydantic v2 request models
# ---------------------------------------------------------------------------

class GoalRequest(BaseModel):
    x: float
    y: float
    z: float = 0.0
    instruction: Optional[str] = None


class CmdVelRequest(BaseModel):
    vx: float
    vy: float = 0.0
    wz: float

    @field_validator("vx", "wz")
    @classmethod
    def finite(cls, v: float) -> float:
        import math
        if not math.isfinite(v):
            raise ValueError("must be finite")
        return v


class InstructionRequest(BaseModel):
    text: str = Field(min_length=1, max_length=1024)


class ModeRequest(BaseModel):
    mode: str

    @field_validator("mode")
    @classmethod
    def valid_mode(cls, v: str) -> str:
        if v not in ("manual", "autonomous", "estop"):
            raise ValueError(f"mode must be manual|autonomous|estop, got {v!r}")
        return v


class LeaseRequest(BaseModel):
    action: str
    client_id: str = "unknown"
    ttl: float = Field(default=30.0, gt=0, le=3600)

    @field_validator("action")
    @classmethod
    def valid_action(cls, v: str) -> str:
        if v not in ("acquire", "release", "renew"):
            raise ValueError(f"action must be acquire|release|renew, got {v!r}")
        return v


class MapRequest(BaseModel):
    action: str
    name:     Optional[str] = None
    new_name: Optional[str] = None

    @field_validator("action")
    @classmethod
    def valid_action(cls, v: str) -> str:
        allowed = {"list", "save", "use", "build", "delete", "rename"}
        if v not in allowed:
            raise ValueError(f"action must be one of {allowed}")
        return v


# ---------------------------------------------------------------------------
# Lease state
# ---------------------------------------------------------------------------

class _Lease:
    """Simple mutex-protected control lease."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._holder: Optional[str] = None
        self._expiry: float = 0.0

    def acquire(self, client_id: str, ttl: float) -> bool:
        with self._lock:
            now = time.monotonic()
            if self._holder and self._holder != client_id and now < self._expiry:
                return False
            self._holder = client_id
            self._expiry = now + ttl
            return True

    def release(self, client_id: str) -> None:
        with self._lock:
            if self._holder == client_id:
                self._holder = None
                self._expiry = 0.0

    def renew(self, client_id: str, ttl: float) -> bool:
        with self._lock:
            if self._holder == client_id:
                self._expiry = time.monotonic() + ttl
                return True
            return False

    def check(self, client_id: str) -> bool:
        with self._lock:
            if self._holder is None:
                return True
            return self._holder == client_id and time.monotonic() < self._expiry

    def to_dict(self) -> Dict[str, Any]:
        with self._lock:
            now = time.monotonic()
            return {
                "holder":  self._holder,
                "active":  self._holder is not None and now < self._expiry,
                "expires_in": max(0.0, self._expiry - now) if self._holder else 0.0,
            }


# ---------------------------------------------------------------------------
# GatewayModule
# ---------------------------------------------------------------------------

@register("gateway", "fastapi", description="FastAPI gateway: REST+SSE+WebSocket teleop")
class GatewayModule(Module, layer=6):
    """HTTP/WebSocket gateway with typed APIs and thread-safe telemetry.

    In:  odometry, scene_graph, safety_state, mission_status,
         execution_eval, dialogue_state
    Out: goal_pose, cmd_vel, stop_cmd, instruction, mode_cmd
    """

    _run_in_main: bool = True

    # -- Inputs (module → cache) --------------------------------------------
    odometry:       In[Odometry]
    scene_graph:    In[SceneGraph]
    safety_state:   In[SafetyState]
    mission_status: In[dict]
    execution_eval: In[ExecutionEval]
    dialogue_state: In[dict]

    # -- Outputs (client commands → modules) --------------------------------
    goal_pose:   Out[PoseStamped]
    cmd_vel:     Out[Twist]
    stop_cmd:    Out[int]    # 0=clear, 1=soft, 2=hard
    instruction: Out[str]
    mode_cmd:    Out[str]

    def __init__(self, port: int = 5050, host: str = "0.0.0.0", **kw):
        super().__init__(**kw)
        self._port = port
        self._host = host

        # Cached state — written by Module callbacks, read by HTTP handlers.
        # Protected by RLock so callbacks and HTTP handler threads don't race.
        self._state_lock = threading.RLock()
        self._odom:     Optional[Dict] = None
        self._sg_json:  str = "{}"
        self._safety:   Optional[Dict] = None
        self._mission:  Optional[Dict] = None
        self._eval:     Optional[Dict] = None
        self._dialogue: Optional[Dict] = None
        self._mode: str = "manual"

        self._lease = _Lease()

        # SSE fan-out — one asyncio.Queue per connected client.
        # Written from Module threads via push_event() (thread-safe).
        self._sse_lock:   threading.Lock = threading.Lock()
        self._sse_queues: List[asyncio.Queue] = []

        # Teleop state (for /ws/teleop)
        self._teleop_active:    bool = False
        self._teleop_clients:   int  = 0
        self._teleop_last_joy:  float = 0.0
        self._teleop_release_timeout: float = 3.0
        self._teleop_max_speed:  float = 0.5
        self._teleop_max_yaw:    float = 1.0
        self._latest_jpeg:  Optional[bytes] = None
        self._jpeg_lock: threading.Lock = threading.Lock()
        # Injected by TeleopModule.on_system_modules() — publish on its behalf
        self._teleop_cmd_vel_port  = None
        self._teleop_nav_stop_port = None

        # Reference to MapManagerModule (set by on_system_modules)
        self._map_mgr = None

        self._app   = None
        self._server_thread: Optional[threading.Thread] = None

    # -- lifecycle ----------------------------------------------------------

    def setup(self) -> None:
        self.odometry.subscribe(self._on_odometry)
        self.scene_graph.subscribe(self._on_scene_graph)
        self.safety_state.subscribe(self._on_safety)
        self.mission_status.subscribe(self._on_mission)
        self.execution_eval.subscribe(self._on_eval)
        self.dialogue_state.subscribe(self._on_dialogue)
        self._app = self._build_app()

    def start(self) -> None:
        super().start()
        self._server_thread = threading.Thread(
            target=self._run_server, daemon=True, name="gateway"
        )
        self._server_thread.start()
        logger.info("GatewayModule started on %s:%d", self._host, self._port)

    def stop(self) -> None:
        self._server_thread = None
        super().stop()

    def on_system_modules(self, modules: Dict[str, Any]) -> None:
        self._map_mgr = modules.get("MapManagerModule")

    # -- teleop config injection (called by TeleopModule) -------------------

    def configure_teleop(
        self,
        max_speed: float,
        max_yaw: float,
        release_timeout: float,
    ) -> None:
        """Called by TeleopModule during its setup() to share config."""
        self._teleop_max_speed = max_speed
        self._teleop_max_yaw   = max_yaw
        self._teleop_release_timeout = release_timeout

    def push_jpeg(self, jpeg_bytes: bytes) -> None:
        """Called by TeleopModule when a new camera frame is ready."""
        with self._jpeg_lock:
            self._latest_jpeg = jpeg_bytes

    # -- Module subscription callbacks -------------------------------------

    def _on_odometry(self, odom: Odometry) -> None:
        d = {
            "x":  odom.x,
            "y":  odom.y,
            "z":  getattr(odom, "z", 0.0),
            "yaw": getattr(odom, "yaw", 0.0),
            "vx": odom.twist.linear.x  if odom.twist else 0.0,
            "wz": odom.twist.angular.z if odom.twist else 0.0,
            "ts": odom.ts,
        }
        with self._state_lock:
            self._odom = d
        self.push_event({"type": "odometry", "data": d})

    def _on_scene_graph(self, sg: SceneGraph) -> None:
        with self._state_lock:
            self._sg_json = sg.to_json() if hasattr(sg, "to_json") else str(sg)

    def _on_safety(self, state: SafetyState) -> None:
        d = {"level": getattr(state, "level", 0), "ts": time.time()}
        with self._state_lock:
            self._safety = d
        self.push_event({"type": "safety", "data": d})

    def _on_mission(self, status: dict) -> None:
        d = status if isinstance(status, dict) else {"raw": str(status)}
        with self._state_lock:
            self._mission = d
        self.push_event({"type": "mission", "data": d})

    def _on_eval(self, ev: ExecutionEval) -> None:
        d = ev.to_dict() if hasattr(ev, "to_dict") else {"raw": str(ev)}
        with self._state_lock:
            self._eval = d
        self.push_event({"type": "eval", "data": d})

    def _on_dialogue(self, state: dict) -> None:
        d = state if isinstance(state, dict) else {"raw": str(state)}
        with self._state_lock:
            self._dialogue = d
        self.push_event({"type": "dialogue", "data": d})

    # -- SSE fan-out --------------------------------------------------------

    def push_event(self, event: Dict) -> None:
        """Thread-safe: push an event to all connected SSE clients."""
        with self._sse_lock:
            queues = list(self._sse_queues)
        for q in queues:
            try:
                q.put_nowait(event)
            except asyncio.QueueFull:
                pass  # slow client — skip frame rather than block

    def _sse_subscribe(self) -> asyncio.Queue:
        q: asyncio.Queue = asyncio.Queue(maxsize=128)
        with self._sse_lock:
            self._sse_queues.append(q)
        return q

    def _sse_unsubscribe(self, q: asyncio.Queue) -> None:
        with self._sse_lock:
            try:
                self._sse_queues.remove(q)
            except ValueError:
                pass

    # -- teleop internals ---------------------------------------------------

    def _teleop_on_joy(self, lx: float, ly: float, az: float) -> None:
        lx = max(-1.0, min(1.0, lx)) * self._teleop_max_speed
        ly = max(-1.0, min(1.0, ly)) * self._teleop_max_speed
        az = max(-1.0, min(1.0, az)) * self._teleop_max_yaw
        twist = Twist(
            linear=Vector3(x=lx, y=ly, z=0.0),
            angular=Vector3(x=0.0, y=0.0, z=az),
        )
        # Publish on TeleopModule's cmd_vel port (higher priority / correct wiring)
        if self._teleop_cmd_vel_port is not None:
            self._teleop_cmd_vel_port.publish(twist)
        else:
            self.cmd_vel.publish(twist)  # fallback if no TeleopModule present
        self._teleop_last_joy = time.monotonic()
        if not self._teleop_active:
            self._teleop_active = True
            if self._teleop_nav_stop_port is not None:
                self._teleop_nav_stop_port.publish(1)
            logger.debug("GatewayModule: teleop engaged")

    def _teleop_release(self) -> None:
        if self._teleop_active:
            self._teleop_active = False
            zero = Twist()
            if self._teleop_cmd_vel_port is not None:
                self._teleop_cmd_vel_port.publish(zero)
            else:
                self.cmd_vel.publish(zero)
            if self._teleop_nav_stop_port is not None:
                self._teleop_nav_stop_port.publish(0)
            logger.info("GatewayModule: teleop released, autonomy resumed")

    # -- FastAPI app --------------------------------------------------------

    def _build_app(self):
        try:
            from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
            from fastapi.exceptions import RequestValidationError
            from fastapi.middleware.cors import CORSMiddleware
            from fastapi.responses import JSONResponse, StreamingResponse
        except ImportError:
            logger.error("FastAPI not installed — run: pip install fastapi uvicorn")
            return None

        app = FastAPI(
            title="LingTu Gateway",
            version="2.0",
            docs_url="/docs",
            redoc_url="/redoc",
        )
        app.add_middleware(
            CORSMiddleware,
            allow_origins=["*"],
            allow_methods=["*"],
            allow_headers=["*"],
        )
        gw = self

        # -- 422 Validation errors → clean JSON ----------------------------

        @app.exception_handler(RequestValidationError)
        async def validation_error(req: Request, exc: RequestValidationError):
            return JSONResponse(
                status_code=422,
                content={"error": "validation_error", "detail": exc.errors()},
            )

        # ── Navigation ─────────────────────────────────────────────────────

        @app.post("/api/v1/goal", summary="Send navigation goal")
        async def post_goal(body: GoalRequest):
            gw.goal_pose.publish(PoseStamped(
                pose=Pose(
                    position=Vector3(body.x, body.y, body.z),
                    orientation=Quaternion(0, 0, 0, 1),
                ),
                frame_id="map",
                ts=time.time(),
            ))
            if body.instruction:
                gw.instruction.publish(body.instruction)
            return {"status": "ok", "goal": [body.x, body.y, body.z]}

        @app.post("/api/v1/cmd_vel", summary="Direct velocity command")
        async def post_cmd_vel(body: CmdVelRequest):
            gw.cmd_vel.publish(Twist(
                linear=Vector3(body.vx, body.vy, 0),
                angular=Vector3(0, 0, body.wz),
            ))
            return {"status": "ok"}

        @app.post("/api/v1/stop", summary="Emergency stop")
        async def post_stop():
            gw.stop_cmd.publish(2)
            gw.cmd_vel.publish(Twist())
            return {"status": "stopped"}

        @app.post("/api/v1/instruction", summary="Natural language navigation instruction")
        async def post_instruction(body: InstructionRequest):
            gw.instruction.publish(body.text)
            return {"status": "ok", "instruction": body.text}

        # ── Mode ───────────────────────────────────────────────────────────

        @app.post("/api/v1/mode", summary="Switch operating mode")
        async def post_mode(body: ModeRequest):
            with gw._state_lock:
                gw._mode = body.mode
            gw.mode_cmd.publish(body.mode)
            if body.mode == "estop":
                gw.stop_cmd.publish(2)
                gw.cmd_vel.publish(Twist())
            return {"status": "ok", "mode": body.mode}

        # ── Lease ──────────────────────────────────────────────────────────

        @app.post("/api/v1/lease", summary="Acquire/release/renew control lease")
        async def post_lease(body: LeaseRequest):
            if body.action == "acquire":
                ok = gw._lease.acquire(body.client_id, body.ttl)
                if not ok:
                    return JSONResponse(
                        status_code=409,
                        content={"error": "lease_conflict",
                                 "detail": gw._lease.to_dict()},
                    )
                return {"status": "acquired", **gw._lease.to_dict()}

            if body.action == "release":
                gw._lease.release(body.client_id)
                return {"status": "released"}

            # renew
            ok = gw._lease.renew(body.client_id, body.ttl)
            if not ok:
                return JSONResponse(
                    status_code=403,
                    content={"error": "not_lease_holder"},
                )
            return {"status": "renewed", **gw._lease.to_dict()}

        # ── Map management ─────────────────────────────────────────────────

        @app.post("/api/v1/maps", summary="Map lifecycle management")
        async def post_maps(body: MapRequest):
            mgr = gw._map_mgr
            if mgr is None:
                return JSONResponse(
                    status_code=503,
                    content={"error": "MapManagerModule not running"},
                )
            # Deliver command via Module port (synchronous one-shot)
            result: List[Dict] = []

            def _capture(resp: dict) -> None:
                result.append(resp)

            mgr.map_response._add_callback(_capture)
            try:
                cmd = {"action": body.action}
                if body.name:
                    cmd["name"] = body.name
                if body.new_name:
                    cmd["new_name"] = body.new_name
                mgr.map_command._deliver(json.dumps(cmd))
            finally:
                try:
                    mgr.map_response._callbacks.remove(_capture)
                except (ValueError, AttributeError):
                    pass

            resp = result[0] if result else {"success": False, "message": "no response"}
            if not resp.get("success"):
                return JSONResponse(
                    status_code=400,
                    content={"error": resp.get("message", "failed"), "detail": resp},
                )
            return resp

        # ── Telemetry ──────────────────────────────────────────────────────

        @app.get("/api/v1/events",
                 summary="SSE event stream",
                 response_class=StreamingResponse)
        async def sse_events():
            q = gw._sse_subscribe()

            async def _stream():
                try:
                    # Send a snapshot immediately so the client has initial state
                    with gw._state_lock:
                        snapshot = {
                            "type": "snapshot",
                            "data": {
                                "odometry": gw._odom,
                                "safety":   gw._safety,
                                "mission":  gw._mission,
                                "mode":     gw._mode,
                            },
                        }
                    yield f"data: {json.dumps(snapshot)}\n\n"

                    while True:
                        try:
                            event = q.get_nowait()
                        except asyncio.QueueEmpty:
                            # Heartbeat every second to keep connection alive
                            yield f"data: {json.dumps({'type': 'ping', 'ts': time.time()})}\n\n"
                            await asyncio.sleep(1.0)
                            continue
                        yield f"data: {json.dumps(event)}\n\n"
                        await asyncio.sleep(0)  # yield control to event loop
                finally:
                    gw._sse_unsubscribe(q)

            return StreamingResponse(_stream(), media_type="text/event-stream",
                                     headers={"Cache-Control": "no-cache",
                                              "X-Accel-Buffering": "no"})

        @app.get("/api/v1/state", summary="Full robot state snapshot")
        async def get_state():
            with gw._state_lock:
                return {
                    "odometry":  gw._odom,
                    "safety":    gw._safety,
                    "mission":   gw._mission,
                    "eval":      gw._eval,
                    "dialogue":  gw._dialogue,
                    "mode":      gw._mode,
                    "lease":     gw._lease.to_dict(),
                    "teleop": {
                        "active":  gw._teleop_active,
                        "clients": gw._teleop_clients,
                    },
                }

        @app.get("/api/v1/scene_graph", summary="Current scene graph")
        async def get_scene_graph():
            with gw._state_lock:
                sg = gw._sg_json
            return JSONResponse({"scene_graph": sg})

        @app.get("/api/v1/health", summary="Gateway health")
        async def get_health():
            with gw._sse_lock:
                n_sse = len(gw._sse_queues)
            return {
                "gateway":     "running",
                "port":        gw._port,
                "mode":        gw._mode,
                "sse_clients": n_sse,
                "teleop": {
                    "active":  gw._teleop_active,
                    "clients": gw._teleop_clients,
                },
                "has_odom": gw._odom is not None,
                "has_map_mgr": gw._map_mgr is not None,
            }

        # ── WebSocket teleop ───────────────────────────────────────────────

        @app.websocket("/ws/teleop")
        async def ws_teleop(ws: WebSocket):
            await ws.accept()
            gw._teleop_clients += 1
            logger.info("Teleop WS connected (%d clients)", gw._teleop_clients)
            release_timeout = gw._teleop_release_timeout

            async def _send_frames():
                """Push JPEG frames to this client at ~10 fps."""
                while True:
                    with gw._jpeg_lock:
                        frame = gw._latest_jpeg
                    if frame:
                        try:
                            await ws.send_bytes(frame)
                        except Exception:
                            break
                    await asyncio.sleep(0.1)

            async def _check_idle():
                """Release autonomy if joystick goes quiet."""
                while True:
                    await asyncio.sleep(0.5)
                    if (gw._teleop_active
                            and time.monotonic() - gw._teleop_last_joy > release_timeout):
                        gw._teleop_release()

            frame_task = asyncio.create_task(_send_frames())
            idle_task  = asyncio.create_task(_check_idle())
            try:
                while True:
                    msg = await ws.receive()
                    if msg["type"] == "websocket.disconnect":
                        break
                    raw = msg.get("text") or msg.get("bytes", b"").decode()
                    if not raw:
                        continue
                    try:
                        data = json.loads(raw)
                    except json.JSONDecodeError:
                        continue
                    t = data.get("type", "")
                    if t == "joy":
                        gw._teleop_on_joy(
                            float(data.get("lx", 0)),
                            float(data.get("ly", 0)),
                            float(data.get("az", 0)),
                        )
                    elif t == "stop":
                        gw.stop_cmd.publish(2)
                        gw.cmd_vel.publish(Twist())
            except WebSocketDisconnect:
                pass
            finally:
                frame_task.cancel()
                idle_task.cancel()
                gw._teleop_clients = max(0, gw._teleop_clients - 1)
                if gw._teleop_clients == 0:
                    gw._teleop_release()
                logger.info("Teleop WS disconnected (%d clients)", gw._teleop_clients)

        return app

    def _run_server(self) -> None:
        if self._app is None:
            return
        try:
            import uvicorn
            config = uvicorn.Config(
                self._app,
                host=self._host,
                port=self._port,
                log_level="warning",
                loop="asyncio",
                # Production: keep-alive and timeout tuning
                timeout_keep_alive=30,
                ws_max_size=2 * 1024 * 1024,  # 2 MB — enough for 1080p JPEG
            )
            server = uvicorn.Server(config)
            server.run()
        except ImportError:
            logger.error("uvicorn not installed — run: pip install 'uvicorn[standard]'")

    # -- health -------------------------------------------------------------

    def health(self) -> Dict[str, Any]:
        info = super().port_summary()
        with self._sse_lock:
            n_sse = len(self._sse_queues)
        info["gateway"] = {
            "port":        self._port,
            "sse_clients": n_sse,
            "teleop_clients": self._teleop_clients,
            "teleop_active": self._teleop_active,
            "has_odom":    self._odom is not None,
            "has_sg":      self._sg_json != "{}",
        }
        return info
