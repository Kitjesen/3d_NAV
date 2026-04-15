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
Probes
  GET  /health               liveness probe (always 200 if alive)
  GET  /ready                readiness probe (200 if all modules ok, 503 if degraded)
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

import numpy as np
from pydantic import BaseModel, Field, field_validator

from core.module import Module
from core.msgs.geometry import Pose, PoseStamped, Quaternion, Twist, Vector3
from core.msgs.nav import Odometry
from core.msgs.semantic import ExecutionEval, SafetyState, SceneGraph
from core.msgs.sensor import PointCloud2
from core.registry import register
from core.stream import In, Out

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Pydantic v2 request models
# ---------------------------------------------------------------------------

class GoalRequest(BaseModel):
    x: float
    y: float
    z: float = 0.0
    instruction: str | None = None


class ClickNavRequest(BaseModel):
    x: float
    y: float
    z: float = 0.0


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
    name:     str | None = None
    new_name: str | None = None

    @field_validator("action")
    @classmethod
    def valid_action(cls, v: str) -> str:
        allowed = {"list", "save", "use", "build", "delete", "rename"}
        if v not in allowed:
            raise ValueError(f"action must be one of {allowed}")
        return v


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _parse_since(since: str) -> float:
    """Parse a human-readable 'since' string to a Unix timestamp.

    Accepted formats: "1h", "30m", "30min", "5s", "2d", or a bare number
    (treated as seconds-ago).  Returns ``time.time() - delta``.
    """
    import re as _re
    now = time.time()
    m = _re.match(r"^(\d+(?:\.\d+)?)\s*(s|sec|m|min|h|hour|d|day)?", since.strip().lower())
    if m:
        value = float(m.group(1))
        unit = m.group(2) or "s"
        if unit in ("h", "hour"):
            return now - value * 3600
        if unit in ("m", "min"):
            return now - value * 60
        if unit in ("d", "day"):
            return now - value * 86400
        return now - value
    try:
        return now - float(since)
    except ValueError:
        return now - 3600  # fallback: 1 hour


# ---------------------------------------------------------------------------
# Lease state
# ---------------------------------------------------------------------------

class _Lease:
    """Simple mutex-protected control lease."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._holder: str | None = None
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

    def to_dict(self) -> dict[str, Any]:
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
    map_cloud:      In[PointCloud2]
    scene_graph:    In[SceneGraph]
    safety_state:   In[SafetyState]
    mission_status: In[dict]
    execution_eval: In[ExecutionEval]
    dialogue_state: In[dict]
    global_path:    In[list]  # from NavigationModule — list of np.ndarray [x,y,z]
    costmap:        In[dict]  # from OccupancyGridModule — {grid, resolution, origin, ts}

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
        self._odom:     dict | None = None
        self._sg_json:  str = "{}"
        self._safety:   dict | None = None
        self._mission:  dict | None = None
        self._eval:     dict | None = None
        self._dialogue: dict | None = None
        self._mode: str = "manual"
        self._last_path: list[dict] = []

        self._lease = _Lease()

        # SSE fan-out — one asyncio.Queue per connected client.
        # Written from Module threads via push_event() (thread-safe).
        self._sse_lock:   threading.Lock = threading.Lock()
        self._sse_queues: list[asyncio.Queue] = []

        # Teleop: delegate to TeleopModule (set by on_system_modules)
        self._teleop_module = None
        self._teleop_clients:   int  = 0
        self._latest_jpeg:  bytes | None = None
        self._jpeg_lock: threading.Lock = threading.Lock()

        # Reference to MapManagerModule (set by on_system_modules)
        self._map_mgr = None
        # All modules dict (set by on_system_modules)
        self._all_modules: dict[str, Any] = {}

        # Map cloud accumulator for /map/viewer
        self._map_points: np.ndarray | None = None
        self._map_cloud_lock = threading.Lock()
        self._map_cloud_count: int = 0
        self._map_voxel_size: float = 0.15

        # Odometry rate tracking (sliding window for SLAM Hz display)
        self._odom_timestamps: list = []  # last 20 timestamps

        # Lazy TemporalStore handle — shared file with TemporalMemoryModule
        self._temporal_store: Any = None

        # Costmap SSE throttle (publish at ~2Hz regardless of OccupancyGridModule rate)
        self._costmap_throttle: int = 0

        # SceneGraph SSE throttle (~2Hz)
        self._sg_throttle: int = 0

        # SLAM status SSE throttle (~1Hz at 10Hz odom)
        self._slam_status_throttle: int = 0

        # Cached SLAM profile (fastlio2 / localizer / stopped) — refreshed every 5s
        self._cached_slam_profile: str = "—"
        self._slam_profile_ts: float = 0.0

        # Autonomous exploration state + frontier explorer module ref
        self._exploring: bool = False
        self._frontier_explorer: Any = None

        # TaggedLocationsModule ref (set by on_system_modules)
        self._tagged_loc_module: Any = None

        self._app   = None
        self._server_thread: threading.Thread | None = None
        self._defer_server: bool = False  # True → main thread runs uvicorn

    # -- lifecycle ----------------------------------------------------------

    def setup(self) -> None:
        self.odometry.subscribe(self._on_odometry)
        self.map_cloud.subscribe(self._on_map_cloud)
        self.map_cloud.set_policy("latest")
        self.scene_graph.subscribe(self._on_scene_graph)
        self.safety_state.subscribe(self._on_safety)
        self.mission_status.subscribe(self._on_mission)
        self.execution_eval.subscribe(self._on_eval)
        self.dialogue_state.subscribe(self._on_dialogue)
        self.global_path.subscribe(self._on_global_path)
        self.costmap.subscribe(self._on_costmap)
        self._app = self._build_app()

    def start(self) -> None:
        super().start()
        if not self._defer_server:
            self._server_thread = threading.Thread(
                target=self._run_server, daemon=True, name="gateway"
            )
            self._server_thread.start()
        logger.info("GatewayModule started on %s:%d", self._host, self._port)

    def stop(self) -> None:
        self._server_thread = None
        super().stop()

    def on_system_modules(self, modules: dict[str, Any]) -> None:
        self._map_mgr = modules.get("MapManagerModule")
        self._all_modules = modules
        self._frontier_explorer = next(
            (m for m in modules.values()
             if m.__class__.__name__ == "WavefrontFrontierExplorer"),
            None,
        )
        self._tagged_loc_module = next(
            (m for m in modules.values()
             if m.__class__.__name__ == "TaggedLocationsModule"),
            None,
        )

    # -- teleop helpers (delegate to TeleopModule) ---------------------------

    @property
    def _teleop_active(self) -> bool:
        """Proxy for teleop active state (lives in TeleopModule now)."""
        tm = self._teleop_module
        return tm._active if tm is not None else False

    def configure_teleop(
        self,
        max_speed: float,
        max_yaw: float,
        release_timeout: float,
    ) -> None:
        """Called by TeleopModule during setup() — config stored for display."""
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
            # Track for Hz calculation (sliding window of 20)
            self._odom_timestamps.append(time.time())
            if len(self._odom_timestamps) > 20:
                self._odom_timestamps.pop(0)
        self.push_event({"type": "odometry", "data": d})

        # Push slam_status at ~1Hz (every 10 odometry frames)
        self._slam_status_throttle += 1
        if self._slam_status_throttle % 10 == 0:
            with self._map_cloud_lock:
                map_pts = len(self._map_points) if self._map_points is not None else 0
            self.push_event({
                "type": "slam_status",
                "mode": self._get_slam_profile(),
                "slam_hz": self._get_slam_hz_cached(),
                "map_points": map_pts,
                "degeneracy_count": getattr(self, "_degeneracy_count", 0),
            })

    @staticmethod
    def _voxel_downsample(pts: np.ndarray, voxel_size: float) -> np.ndarray:
        """Voxel grid downsample: keep one point per cell (by first occurrence)."""
        if len(pts) == 0:
            return pts
        voxels = np.floor(pts[:, :3] / voxel_size).astype(np.int32)
        _, unique_idx = np.unique(voxels, axis=0, return_index=True)
        return pts[np.sort(unique_idx)]

    def _on_map_cloud(self, cloud: PointCloud2) -> None:
        """Accumulate map cloud with voxel downsampling; push 30k pts to SSE every 5 frames."""
        self._map_cloud_count += 1
        pts = cloud.points  # (N, 3) float32
        if pts is None or len(pts) == 0:
            return
        pts = pts[:, :3].astype(np.float32)
        with self._map_cloud_lock:
            if self._map_points is None:
                self._map_points = pts
            else:
                combined = np.concatenate([self._map_points, pts], axis=0)
                # Voxel-downsample when accumulated cloud grows large
                if len(combined) > 300_000:
                    combined = self._voxel_downsample(combined, self._map_voxel_size)
                self._map_points = combined

        # Push to SSE at ~0.2Hz (every 5 frames)
        if self._map_cloud_count % 5 == 0:
            with self._map_cloud_lock:
                pts_all = self._map_points
            if pts_all is None:
                return
            # Sample ≤30k points for SSE bandwidth
            if len(pts_all) > 30_000:
                idx = np.random.choice(len(pts_all), 30_000, replace=False)
                pts_send = pts_all[idx]
            else:
                pts_send = pts_all
            flat = pts_send[:, :3].astype(np.float32).flatten().tolist()
            self.push_event({"type": "map_cloud", "points": flat, "count": len(pts_send)})

    def _get_slam_profile(self) -> str:
        """Return current SLAM profile (cached 5s): fastlio2 / localizer / stopped."""
        now = time.time()
        if now - self._slam_profile_ts < 5.0:
            return self._cached_slam_profile
        self._slam_profile_ts = now
        try:
            from core.service_manager import get_service_manager
            svc = get_service_manager()
            services = svc.status("slam_pgo", "localizer", "slam")
            if services.get("slam_pgo") in ("running", "active"):
                profile = "fastlio2"
            elif services.get("localizer") in ("running", "active"):
                profile = "localizer"
            elif services.get("slam") in ("running", "active"):
                profile = "slam"
            else:
                profile = "stopped"
            self._cached_slam_profile = profile
        except Exception:
            pass
        return self._cached_slam_profile

    @staticmethod
    def _voxel_downsample(pts: np.ndarray, voxel: float) -> np.ndarray:
        """Fast numpy voxel grid downsample."""
        keys = (pts[:, :3] / voxel).astype(np.int32)
        _, idx = np.unique(keys, axis=0, return_index=True)
        return pts[idx]

    def _on_scene_graph(self, sg: SceneGraph) -> None:
        with self._state_lock:
            self._sg_json = sg.to_json() if hasattr(sg, "to_json") else str(sg)
        # Push detected objects to SSE viewer at ~2Hz
        self._sg_throttle += 1
        if self._sg_throttle % 5 == 0:
            try:
                objects = [
                    {
                        "label": obj.label,
                        "x": round(float(obj.position.x), 2),
                        "y": round(float(obj.position.y), 2),
                        "conf": round(float(obj.confidence), 2),
                    }
                    for obj in sg.objects
                    if obj.label and float(obj.confidence) > 0.25
                ]
                if objects:
                    self.push_event({"type": "scene_graph", "objects": objects})
            except Exception:
                pass

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

    def _on_global_path(self, path: list) -> None:
        # path is list of np.ndarray [x, y, z] from NavigationModule
        points = [
            {"x": float(p[0]), "y": float(p[1]), "z": float(p[2]) if len(p) > 2 else 0.0}
            for p in path
        ]
        with self._state_lock:
            self._last_path = points
        self.push_event({"type": "global_path", "points": points})

    def _on_costmap(self, cm: dict) -> None:
        """Throttle OccupancyGridModule costmap to ~2 Hz and push as SSE."""
        self._costmap_throttle += 1
        if self._costmap_throttle % 5 != 0:
            return
        grid = cm.get("grid")
        if grid is None:
            return
        try:
            import base64 as _b64
            import numpy as _np
            g = _np.clip(grid, 0, 100).astype(_np.uint8)
            cols = int(g.shape[0])
            self.push_event({
                "type":       "costmap",
                "grid_b64":   _b64.b64encode(g.tobytes()).decode(),
                "cols":       cols,
                "resolution": float(cm.get("resolution", 0.1)),
                "origin":     [float(v) for v in cm.get("origin", [0.0, 0.0])],
            })
        except Exception as exc:
            logger.debug("_on_costmap serialize failed: %s", exc)

    # -- SSE fan-out --------------------------------------------------------

    def push_event(self, event: dict) -> None:
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

    # -- teleop internals (forwarded to TeleopModule) -------------------------

    def _teleop_on_joy(self, lx: float, ly: float, az: float) -> None:
        """Forward raw joystick input to TeleopModule via joy_input port."""
        tm = self._teleop_module
        if tm is not None and hasattr(tm, "joy_input"):
            tm.joy_input.publish({"lx": lx, "ly": ly, "az": az})
        else:
            # Fallback: publish directly (no TeleopModule present)
            lx = max(-1.0, min(1.0, lx)) * getattr(self, "_teleop_max_speed", 0.5)
            ly = max(-1.0, min(1.0, ly)) * getattr(self, "_teleop_max_speed", 0.5)
            az = max(-1.0, min(1.0, az)) * getattr(self, "_teleop_max_yaw", 1.0)
            self.cmd_vel.publish(Twist(
                linear=Vector3(x=lx, y=ly, z=0.0),
                angular=Vector3(x=0.0, y=0.0, z=az),
            ))

    def _teleop_release(self) -> None:
        tm = self._teleop_module
        if tm is not None:
            tm.force_release()
        else:
            self.cmd_vel.publish(Twist())

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

        # -- API Key auth (disabled if LINGTU_API_KEY not set) -------------
        from gateway.auth import APIKeyMiddleware
        app.add_middleware(APIKeyMiddleware)

        gw = self

        # -- Auth endpoints (always public) --------------------------------

        @app.post("/api/v1/auth/login", summary="Login with API key")
        async def auth_login(request: Request):
            body = await request.json()
            key = body.get("key", "")
            from gateway.auth import _get_configured_key
            configured = _get_configured_key()
            if not configured:
                return JSONResponse({"ok": True, "message": "认证未启用"})
            import hashlib
            import hmac as _hmac
            if _hmac.compare_digest(
                hashlib.sha256(key.encode()).hexdigest(),
                hashlib.sha256(configured.encode()).hexdigest(),
            ):
                resp = JSONResponse({"ok": True, "message": "登录成功"})
                resp.set_cookie("lingtu_api_key", key, httponly=True, max_age=86400 * 30)
                return resp
            return JSONResponse({"ok": False, "message": "Key 无效"}, status_code=403)

        @app.get("/api/v1/auth/check", summary="Check if auth is required")
        async def auth_check():
            from gateway.auth import _get_configured_key
            configured = _get_configured_key()
            return {"auth_required": configured is not None}

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

        @app.post("/api/v1/navigate/click", summary="Navigate to map-viewer click point")
        async def post_navigate_click(body: ClickNavRequest):
            gw.goal_pose.publish(PoseStamped(
                pose=Pose(
                    position=Vector3(body.x, body.y, body.z),
                    orientation=Quaternion(0, 0, 0, 1),
                ),
                frame_id="map",
                ts=time.time(),
            ))
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

        # ── Memory ─────────────────────────────────────────────────────────

        def _temporal_store():
            """Lazy-open TemporalStore at the same path TemporalMemoryModule uses."""
            if gw._temporal_store is None:
                try:
                    import os as _os
                    from memory.storage.temporal_store import TemporalStore as _TS
                    mem_dir = _os.environ.get(
                        "LINGTU_MEMORY_DIR",
                        _os.path.join(_os.path.expanduser("~"), ".nova", "semantic"),
                    )
                    gw._temporal_store = _TS(_os.path.join(mem_dir, "temporal_memory.db"))
                except Exception as exc:
                    logger.warning("GatewayModule: TemporalStore unavailable: %s", exc)
            return gw._temporal_store

        @app.get("/api/v1/memory/temporal", summary="Query temporal entity observations")
        async def get_temporal_memory(
            label: str | None = None,
            since: str | None = None,
            near_x: float | None = None,
            near_y: float | None = None,
            radius: float | None = None,
            limit: int = 100,
        ):
            since_ts = _parse_since(since) if since else None
            store = _temporal_store()
            if store is None:
                return JSONResponse(
                    status_code=503,
                    content={"error": "temporal_store_unavailable",
                             "detail": "TemporalMemoryModule not running or save_dir not set"},
                )
            loop = asyncio.get_event_loop()
            rows = await loop.run_in_executor(
                None,
                lambda: store.query(
                    label=label,
                    since_ts=since_ts,
                    near_x=near_x,
                    near_y=near_y,
                    radius=radius,
                    limit=max(1, min(limit, 1000)),
                ),
            )
            return {"observations": rows, "count": len(rows)}

        @app.post("/api/v1/memory/temporal/semantic",
                  summary="Semantic similarity search over temporal observations")
        async def post_temporal_semantic(body: dict):
            """Accept {embedding: [float,...], top_k: int, since: str, label: str}.

            Returns observations ordered by CLIP cosine similarity to the
            provided embedding vector.  Used by downstream agents / askme tool.
            """
            import numpy as _np
            raw_emb = body.get("embedding")
            if not raw_emb:
                return JSONResponse(status_code=422,
                                    content={"error": "embedding required"})
            try:
                query_vec = _np.asarray(raw_emb, dtype=_np.float32)
            except Exception as exc:
                return JSONResponse(status_code=422,
                                    content={"error": f"invalid embedding: {exc}"})

            since_ts = _parse_since(body["since"]) if body.get("since") else None
            store = _temporal_store()
            if store is None:
                return JSONResponse(status_code=503,
                                    content={"error": "temporal_store_unavailable"})

            loop = asyncio.get_event_loop()
            rows = await loop.run_in_executor(
                None,
                lambda: store.query_semantic(
                    query_vec,
                    top_k=int(body.get("top_k", 10)),
                    since_ts=since_ts,
                    label=body.get("label") or None,
                ),
            )
            return {"observations": rows, "count": len(rows)}

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
            result: list[dict] = []

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

        # ── Explore ────────────────────────────────────────────────────────

        @app.post("/api/v1/explore/start", summary="Start autonomous frontier exploration")
        async def explore_start():
            fe = gw._frontier_explorer
            if fe is None:
                return JSONResponse(status_code=503,
                                    content={"error": "WavefrontFrontierExplorer not running"})
            loop = asyncio.get_event_loop()
            result = await loop.run_in_executor(None, fe.begin_exploration)
            gw._exploring = True
            gw.push_event({"type": "exploring", "active": True})
            return {"status": result}

        @app.post("/api/v1/explore/stop", summary="Stop autonomous frontier exploration")
        async def explore_stop():
            fe = gw._frontier_explorer
            if fe is None:
                return JSONResponse(status_code=503,
                                    content={"error": "WavefrontFrontierExplorer not running"})
            loop = asyncio.get_event_loop()
            result = await loop.run_in_executor(None, fe.end_exploration)
            gw._exploring = False
            gw.push_event({"type": "exploring", "active": False})
            return {"status": result}

        @app.get("/api/v1/explore/status", summary="Exploration status")
        async def explore_status():
            fe = gw._frontier_explorer
            if fe is None:
                return {"available": False, "exploring": False}
            h = fe.health() if hasattr(fe, "health") else {}
            return {
                "available":      True,
                "exploring":      gw._exploring,
                "frontier_count": h.get("frontier_count", 0),
            }

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

        @app.get("/api/v1/locations", summary="List tagged navigation locations")
        async def get_locations():
            tlm = gw._tagged_loc_module
            if tlm is None:
                return JSONResponse({"locations": []})
            try:
                entries = list(tlm.store._store.values())
                locations = [
                    {
                        "name": e.get("name", ""),
                        "x": round(float(e.get("x", 0.0)), 3),
                        "y": round(float(e.get("y", 0.0)), 3),
                        "z": round(float(e.get("z", 0.0)), 3),
                    }
                    for e in entries
                    if e.get("name")
                ]
            except Exception:
                locations = []
            return JSONResponse({"locations": locations})

        @app.get("/api/v1/path", summary="Latest planned path")
        async def get_path():
            with gw._state_lock:
                return {"path": gw._last_path, "robot": gw._odom}

        @app.get("/api/v1/health", summary="System health overview")
        async def get_health():
            with gw._sse_lock:
                n_sse = len(gw._sse_queues)
            map_pts = gw._map_cloud_count
            slam_hz = gw._get_slam_hz_cached()

            # Sensor & module status summary
            sensors: dict[str, Any] = {}
            modules_ok = 0
            modules_fail = 0
            module_summary: dict[str, str] = {}

            if gw._all_modules:
                for name, mod in gw._all_modules.items():
                    try:
                        h = mod.health() if hasattr(mod, "health") else {}
                        module_summary[name] = "ok"
                        modules_ok += 1

                        # Extract sensor-level info from key modules
                        if "LidarModule" in name:
                            lidar_h = h.get("lidar", {})
                            sensors["lidar"] = {
                                "status": lidar_h.get("state", "unknown"),
                                "ip": lidar_h.get("ip", "?"),
                                "cloud_hz": round(h.get("ports_out", {}).get("scan", {}).get("rate_hz", 0), 1),
                            }
                        elif "CameraBridge" in name:
                            color_out = h.get("ports_out", {}).get("color_image", {})
                            sensors["camera"] = {
                                "status": "streaming" if color_out.get("msg_count", 0) > 0 else "idle",
                                "fps": round(color_out.get("rate_hz", 0), 1),
                                "frames": color_out.get("msg_count", 0),
                            }
                        elif "SlamBridge" in name or "SLAMModule" in name:
                            odom_out = h.get("ports_out", {}).get("odometry", {})
                            sensors["slam"] = {
                                "status": "active" if odom_out.get("msg_count", 0) > 0 else "inactive",
                                "hz": round(odom_out.get("rate_hz", 0), 1),
                            }
                        elif "Navigation" in name:
                            sensors["navigation"] = {
                                "state": h.get("mission_state", "idle"),
                                "replan_count": h.get("replan_count", 0),
                            }
                    except Exception:
                        module_summary[name] = "error"
                        modules_fail += 1

            # ── Brainstem gRPC probe (RobotControl :13145) ──
            def _brainstem_probe():
                import brainstem_api as bapi
                import grpc
                ch = grpc.insecure_channel("127.0.0.1:13145")
                stub = bapi.RobotControlStub(ch)
                state = stub.GetCmsState(bapi.Empty(), timeout=1.0)
                fsm_map = {0: "ZERO", 1: "GROUNDED", 2: "STANDING",
                           3: "WALKING", 4: "TRANSITIONING"}
                info: dict[str, Any] = {
                    "status": "connected",
                    "host": "127.0.0.1:13145",
                    "fsm": fsm_map.get(state.kind, str(state.kind)),
                }
                try:
                    v = stub.GetVoltage(bapi.Empty(), timeout=1.0)
                    if v.values:
                        info["voltage_avg"] = round(sum(v.values) / len(v.values), 1)
                except Exception:
                    pass
                ch.close()
                return info

            brainstem_info: dict[str, Any] = {"status": "not_probed"}
            try:
                loop = asyncio.get_event_loop()
                brainstem_info = await loop.run_in_executor(None, _brainstem_probe)
            except ImportError:
                brainstem_info = {"status": "unavailable",
                                 "reason": "brainstem_api not installed"}
            except Exception as e:
                brainstem_info = {"status": "unreachable",
                                 "host": "127.0.0.1:13145",
                                 "error": str(e)[:120]}

            return {
                "status": "ok" if modules_fail == 0 else "degraded",
                "modules_ok": modules_ok,
                "modules_fail": modules_fail,
                "gateway": {
                    "port": gw._port,
                    "mode": gw._mode,
                    "sse_clients": n_sse,
                },
                "teleop": {
                    "active": gw._teleop_active,
                    "clients": gw._teleop_clients,
                },
                "sensors": sensors,
                "slam_hz": round(slam_hz, 1),
                "map_points": map_pts,
                "has_odom": gw._odom is not None,
                "modules": module_summary,
                "brainstem": brainstem_info,
            }

        # ── Liveness / Readiness probes ────────────────────────────────────

        @app.get("/health", summary="Liveness probe")
        async def liveness_health():
            return {"status": "ok", "ts": time.time()}

        @app.get("/ready", summary="Readiness probe")
        async def readiness_ready():
            if not gw._all_modules:
                return JSONResponse(
                    {"status": "not_started", "modules": {}},
                    status_code=503,
                )

            module_health: dict[str, Any] = {}
            all_ok = True
            for name, mod in gw._all_modules.items():
                try:
                    h = mod.health() if hasattr(mod, "health") else {}
                    module_health[name] = {"ok": True, "detail": h}
                except Exception as e:
                    module_health[name] = {"ok": False, "error": str(e)}
                    all_ok = False

            status_code = 200 if all_ok else 503
            return JSONResponse(
                {
                    "status": "ready" if all_ok else "degraded",
                    "modules": module_health,
                    "ts": time.time(),
                },
                status_code=status_code,
            )

        # ── Dashboard + SLAM Control ──────────────────────────────────────

        @app.get("/dashboard", summary="Map management dashboard")
        async def dashboard():
            from starlette.responses import HTMLResponse

            from gateway.map_dashboard import generate_dashboard_html
            return HTMLResponse(generate_dashboard_html())

        @app.get("/api/v1/slam/status", summary="SLAM service status")
        async def slam_status():
            try:
                from core.service_manager import get_service_manager
                svc = get_service_manager()
                services = svc.status("lidar", "slam", "slam_pgo", "localizer")
            except Exception:
                services = {"lidar": "unknown", "slam": "unknown", "slam_pgo": "unknown", "localizer": "unknown"}

            if services.get("slam_pgo") in ("running", "active"):
                mode = "fastlio2"
            elif services.get("localizer") in ("running", "active"):
                mode = "localizer"
            elif services.get("slam") in ("running", "active"):
                mode = "slam_only"
            else:
                mode = "stopped"
            return {"mode": mode, "services": services}

        @app.get("/api/v1/slam/maps", summary="List maps from filesystem")
        async def slam_maps():
            """Filesystem scan fallback — works even without MapManagerModule."""
            import os
            import pathlib
            map_dir = os.environ.get("NAV_MAP_DIR", os.path.expanduser("~/data/inovxio/data/maps"))
            maps = []
            active_target = ""
            active_link = pathlib.Path(map_dir) / "active"
            if active_link.is_symlink():
                active_target = active_link.resolve().name

            if os.path.isdir(map_dir):
                for d in sorted(os.listdir(map_dir)):
                    full = os.path.join(map_dir, d)
                    if not os.path.isdir(full) or d.startswith("_") or d == "active":
                        continue
                    pcd = os.path.join(full, "map.pcd")
                    has_pcd = os.path.isfile(pcd)
                    patches_dir = os.path.join(full, "patches")
                    patch_count = len(os.listdir(patches_dir)) if os.path.isdir(patches_dir) else 0
                    has_tomogram = os.path.isfile(os.path.join(full, "tomogram.pickle"))
                    size_mb: float | None = None
                    if has_pcd:
                        sz = os.path.getsize(pcd)
                        size_mb = round(sz / 1024 / 1024, 1)
                    maps.append({
                        "name": d,
                        "has_pcd": has_pcd,
                        "has_tomogram": has_tomogram,
                        "is_active": d == active_target,
                        "size_mb": size_mb,
                        "patch_count": patch_count,
                    })
            return {"maps": maps, "active": active_target, "map_dir": map_dir}

        @app.get("/api/v1/maps/{name}/pcd", summary="Serve raw PCD file for inline preview")
        async def get_map_pcd(name: str):
            import os as _os
            import pathlib

            from fastapi import HTTPException
            from fastapi.responses import FileResponse
            map_dir = _os.environ.get("NAV_MAP_DIR", _os.path.expanduser("~/data/inovxio/data/maps"))
            base = pathlib.Path(map_dir).resolve()
            pcd_path = (base / name / "map.pcd").resolve()
            if not str(pcd_path).startswith(str(base)):
                raise HTTPException(status_code=403)
            if not pcd_path.is_file():
                raise HTTPException(status_code=404, detail=f"No PCD for map: {name}")
            return FileResponse(str(pcd_path), media_type="application/octet-stream",
                                filename="map.pcd")

        @app.get("/api/v1/maps/{name}/points", summary="Saved map point cloud as JSON")
        async def get_saved_map_points(name: str, max_points: int = 30000):
            import pathlib
            from fastapi import HTTPException
            map_dir = os.environ.get("NAV_MAP_DIR", os.path.expanduser("~/data/inovxio/data/maps"))
            base = pathlib.Path(map_dir).resolve()
            pcd_path = (base / name / "map.pcd").resolve()
            if not str(pcd_path).startswith(str(base)):
                raise HTTPException(status_code=403)
            if not pcd_path.is_file():
                raise HTTPException(status_code=404, detail=f"Map not found: {name}")
            # Parse PCD binary (same logic as _load_map_for_viewer)
            with open(pcd_path, "rb") as f:
                n_points, point_step = 0, 16
                while True:
                    line = f.readline().decode("ascii", errors="ignore").strip()
                    if "POINTS" in line:
                        n_points = int(line.split()[-1])
                    if "SIZE" in line:
                        point_step = sum(int(s) for s in line.split()[1:])
                    if line.startswith("DATA"):
                        break
                data = f.read(n_points * point_step)
            pts = np.frombuffer(data[:n_points * point_step], dtype=np.float32).reshape(n_points, point_step // 4)[:, :3]
            valid = np.isfinite(pts).all(axis=1)
            pts = pts[valid]
            if len(pts) > 0:
                med = np.median(pts, axis=0)
                pts = pts[np.abs(pts - med).max(axis=1) < 100]
            if len(pts) > max_points:
                idx = np.random.choice(len(pts), max_points, replace=False)
                pts = pts[idx]
            flat = pts[:, :3].astype(np.float32).flatten().tolist()
            return {"count": len(pts), "points": flat}

        @app.post("/api/v1/slam/switch", summary="Hot-switch SLAM profile")
        async def slam_switch(body: dict):
            profile = body.get("profile", "")
            if profile not in ("fastlio2", "localizer", "stop"):
                return JSONResponse({"success": False, "message": f"Unknown profile: {profile}"}, status_code=400)
            try:
                from core.service_manager import get_service_manager
                svc = get_service_manager()
                if profile == "fastlio2":
                    svc.stop("localizer")
                    svc.ensure("slam", "slam_pgo")
                    ok = svc.wait_ready("slam", "slam_pgo", timeout=10.0)
                elif profile == "localizer":
                    svc.stop("slam_pgo")
                    svc.ensure("slam", "localizer")
                    ok = svc.wait_ready("slam", "localizer", timeout=10.0)
                else:
                    svc.stop("slam_pgo", "localizer", "slam")
                    ok = True
                return {"success": ok, "profile": profile, "message": f"Switched to {profile}" if ok else "Services not ready after 10s"}
            except Exception as e:
                return JSONResponse({"success": False, "message": str(e)}, status_code=500)

        @app.post("/api/v1/slam/relocalize", summary="Relocalize against a saved map")
        async def slam_relocalize(body: dict):
            """Call /relocalize ROS2 service to localize robot in a saved map."""
            import os, subprocess
            map_name = body.get("map_name", "")
            x    = float(body.get("x",   0.0))
            y    = float(body.get("y",   0.0))
            yaw  = float(body.get("yaw", 0.0))
            if not map_name:
                return JSONResponse({"success": False, "message": "map_name required"}, status_code=400)
            map_dir  = os.environ.get("NAV_MAP_DIR", os.path.expanduser("~/data/inovxio/data/maps"))
            pcd_path = os.path.join(map_dir, map_name, "map.pcd")
            if not os.path.isfile(pcd_path):
                return JSONResponse({"success": False, "message": f"Map not found: {pcd_path}"}, status_code=404)
            _ros_env = (
                "source /opt/ros/humble/setup.bash && "
                "source ~/data/SLAM/navigation/install/setup.bash 2>/dev/null; "
                "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && "
            )
            try:
                r = subprocess.run(
                    ["bash", "-c",
                     _ros_env +
                     f"ros2 service call /relocalize interface/srv/Relocalize "
                     f"\"{{pcd_path: '{pcd_path}', x: {x}, y: {y}, z: 0.0, "
                     f"yaw: {yaw}, pitch: 0.0, roll: 0.0}}\""],
                    capture_output=True, text=True, timeout=30)
                ok = "success=True" in r.stdout
                msg = r.stdout[-300:] if not ok else f"Relocalized to {map_name}"
                return {"success": ok, "message": msg}
            except Exception as e:
                return JSONResponse({"success": False, "message": str(e)}, status_code=500)

        # ── Map 3D Viewer ─────────────────────────────────────────────────

        @app.get("/api/v1/map/points", summary="Map point cloud as JSON (from ikd-tree snapshot)")
        async def get_map_points(max_points: int = 80000):
            # Read from latest snapshot (same source as viewer)
            with gw._map_cloud_lock:
                pts = gw._map_points
            if pts is None or len(pts) == 0:
                return {"count": 0, "points": []}
            if len(pts) > max_points:
                idx = np.random.choice(len(pts), max_points, replace=False)
                pts = pts[idx]
            return {
                "count": len(pts),
                "bounds": {
                    "x": [float(pts[:, 0].min()), float(pts[:, 0].max())],
                    "y": [float(pts[:, 1].min()), float(pts[:, 1].max())],
                    "z": [float(pts[:, 2].min()), float(pts[:, 2].max())],
                },
                "points": pts[:, :3].tolist(),
            }

        @app.get("/map/viewer", summary="Interactive 3D map viewer")
        async def map_viewer(map: str = ""):
            from starlette.responses import HTMLResponse
            if map:
                html = gw._generate_viewer_from_pcd(map)
            else:
                # Live: snapshot from ikd-tree via save_map to temp file
                html = gw._generate_viewer_live()
            return HTMLResponse(html)

        @app.get("/robot/meshes/{filename}", summary="Serve robot STL mesh files")
        async def serve_robot_mesh(filename: str):
            import os
            from starlette.responses import FileResponse
            from fastapi.responses import JSONResponse
            mesh_dir = os.environ.get(
                "DOG_MESH_DIR",
                os.path.join(os.path.dirname(__file__),
                             "../../../../products/quadruped_ws/dog_arm/meshes"),
            )
            safe_name = os.path.basename(filename)  # prevent path traversal
            path = os.path.join(mesh_dir, safe_name)
            if not os.path.isfile(path):
                return JSONResponse(status_code=404, content={"error": "mesh not found", "name": safe_name})
            return FileResponse(path, media_type="application/octet-stream",
                                headers={"Access-Control-Allow-Origin": "*",
                                         "Cache-Control": "public, max-age=3600"})

        @app.get("/api/v1/camera/snapshot", summary="Camera JPEG snapshot")
        async def camera_snapshot():
            """Grab one JPEG frame via rclpy (fastrtps, matching camera driver)."""
            import os
            import subprocess
            import tempfile

            from starlette.responses import Response
            out = os.path.join(tempfile.gettempdir(), "lingtu_cam_snap.jpg")
            script = os.path.join(tempfile.gettempdir(), "lingtu_cam_snap.py")
            # Write script to avoid shell escaping issues
            with open(script, "w") as f:
                f.write(
                    "import rclpy, sys\n"
                    "from sensor_msgs.msg import CompressedImage\n"
                    "rclpy.init()\n"
                    "n=rclpy.create_node('cam_snap')\n"
                    "msg=[None]\n"
                    "n.create_subscription(CompressedImage,'/camera/color/image_raw/compressed',lambda m:msg.__setitem__(0,m),1)\n"
                    "import time; t=time.time()\n"
                    "while msg[0] is None and time.time()-t<2: rclpy.spin_once(n,timeout_sec=0.1)\n"
                    "n.destroy_node(); rclpy.shutdown()\n"
                    f"open('{out}','wb').write(msg[0].data) if msg[0] else None\n"
                )
            try:
                # Unset RMW to use default fastrtps (camera uses fastrtps)
                env = os.environ.copy()
                env.pop("RMW_IMPLEMENTATION", None)
                subprocess.run(
                    ["bash", "-c", f"source /opt/ros/humble/setup.bash && python3 {script}"],
                    capture_output=True, timeout=6, env=env)
                if os.path.isfile(out) and os.path.getsize(out) > 100:
                    with open(out, "rb") as f:
                        data = f.read()
                    return Response(content=data, media_type="image/jpeg")
                else:
                    return JSONResponse({"error": "no frame"}, status_code=503)
            except Exception as e:
                return JSONResponse({"error": str(e)}, status_code=500)

        @app.post("/api/v1/map/activate", summary="Set active map (symlink)")
        async def activate_map(body: dict):
            import os
            import pathlib
            name = body.get("name", "")
            if not name:
                return JSONResponse({"success": False, "message": "需要 name"}, status_code=400)
            map_dir = os.environ.get("NAV_MAP_DIR", os.path.expanduser("~/data/inovxio/data/maps"))
            target = os.path.join(map_dir, name)
            if not os.path.isdir(target):
                return JSONResponse({"success": False, "message": f"地图不存在: {name}"}, status_code=404)
            active_link = pathlib.Path(map_dir) / "active"
            try:
                if active_link.is_symlink() or active_link.exists():
                    active_link.unlink()
                active_link.symlink_to(name)
                return {"success": True, "active": name}
            except Exception as e:
                return JSONResponse({"success": False, "message": str(e)}, status_code=500)

        @app.post("/api/v1/map/rename", summary="Rename a saved map")
        async def rename_map(body: dict):
            import os
            import pathlib
            old = body.get("old_name", "")
            new = body.get("new_name", "")
            if not old or not new:
                return JSONResponse({"success": False, "message": "需要 old_name 和 new_name"}, status_code=400)
            map_dir = os.environ.get("NAV_MAP_DIR", os.path.expanduser("~/data/inovxio/data/maps"))
            old_path = os.path.join(map_dir, old)
            new_path = os.path.join(map_dir, new)
            if not os.path.isdir(old_path):
                return JSONResponse({"success": False, "message": f"地图不存在: {old}"}, status_code=404)
            if os.path.exists(new_path):
                return JSONResponse({"success": False, "message": f"名称已占用: {new}"}, status_code=409)
            try:
                os.rename(old_path, new_path)
                # Update active symlink if it pointed to old name
                active_link = pathlib.Path(map_dir) / "active"
                if active_link.is_symlink() and active_link.resolve().name == old:
                    active_link.unlink()
                    active_link.symlink_to(new_path)
                return {"success": True, "old_name": old, "new_name": new}
            except Exception as e:
                return JSONResponse({"success": False, "message": str(e)}, status_code=500)

        @app.post("/api/v1/map/save", summary="Save current SLAM map")
        async def save_map_now(body: dict | None = None):
            """One-click map save — calls ROS2 save_map service via subprocess."""
            import os
            import subprocess
            if body is None:
                body = {}
            name = body.get("name", "")
            if not name:
                from datetime import datetime
                name = "map_" + datetime.now().strftime("%Y%m%d_%H%M%S")
            map_dir = os.environ.get("NAV_MAP_DIR", os.path.expanduser("~/data/inovxio/data/maps"))
            save_dir = os.path.join(map_dir, name)
            os.makedirs(save_dir, exist_ok=True)
            pcd_path = os.path.join(save_dir, "map.pcd")
            errors = []

            # Common ROS2 env setup for subprocess calls
            _ros_env = (
                "source /opt/ros/humble/setup.bash && "
                "source ~/data/SLAM/navigation/install/setup.bash 2>/dev/null; "
                "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && "
            )

            # Save via Fast-LIO2
            try:
                r = subprocess.run(
                    ["bash", "-c",
                     _ros_env +
                     f"ros2 service call /nav/save_map interface/srv/SaveMaps "
                     f"\"{{file_path: '{pcd_path}'}}\""],
                    capture_output=True, text=True, timeout=30)
                if "success=True" in r.stdout:
                    pass
                else:
                    errors.append(f"Fast-LIO2: {r.stderr[-200:] if r.stderr else r.stdout[-200:]}")
            except Exception as e:
                errors.append(f"Fast-LIO2: {e}")

            # Save via PGO (patches + poses)
            try:
                r = subprocess.run(
                    ["bash", "-c",
                     _ros_env +
                     f"ros2 service call /pgo/save_maps interface/srv/SaveMaps "
                     f"\"{{file_path: '{save_dir}', save_patches: true}}\""],
                    capture_output=True, text=True, timeout=30)
            except Exception:
                pass  # PGO save is optional

            has_pcd = os.path.isfile(pcd_path)
            if has_pcd:
                size = os.path.getsize(pcd_path)
                return {"success": True, "name": name, "path": save_dir,
                        "size": f"{size/1024/1024:.1f}MB"}
            else:
                return JSONResponse(
                    {"success": False, "name": name, "errors": errors},
                    status_code=500)

        # ── WebSocket teleop ───────────────────────────────────────────────
        # Use Starlette's WebSocketRoute directly (app.add_websocket_route)
        # instead of @app.websocket to bypass FastAPI's APIWebSocketRoute
        # dependency-injection stack, which has a bug in 0.135.x that causes
        # the handshake to be silently closed before the handler body runs.

        from starlette.websockets import WebSocket as StarletteWebSocket
        from starlette.websockets import WebSocketDisconnect as StarletteWebSocketDisconnect

        async def ws_teleop_endpoint(ws: StarletteWebSocket):
            await ws.accept()
            gw._teleop_clients += 1
            tm = gw._teleop_module
            if tm is not None:
                tm.on_client_connect()
            logger.info("Teleop WS connected (%d clients)", gw._teleop_clients)

            async def _send_frames():
                """Push JPEG frames to this client at ~10 fps."""
                while True:
                    with gw._jpeg_lock:
                        frame = gw._latest_jpeg
                    if frame:
                        try:
                            await ws.send_bytes(frame)
                        except Exception as e:
                            logger.debug("teleop frame send failed: %s", e)
                            break
                    await asyncio.sleep(0.1)

            frame_task = asyncio.create_task(_send_frames())
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
                        if tm is not None:
                            tm.force_release()
                        else:
                            gw.cmd_vel.publish(Twist())
            except StarletteWebSocketDisconnect:
                pass
            finally:
                frame_task.cancel()
                gw._teleop_clients = max(0, gw._teleop_clients - 1)
                if tm is not None:
                    tm.on_client_disconnect()
                elif gw._teleop_clients == 0:
                    gw._teleop_release()
                logger.info("Teleop WS disconnected (%d clients)", gw._teleop_clients)

        app.add_websocket_route("/ws/teleop", ws_teleop_endpoint)

        # Serve React dashboard (web/dist/) at root — must be last mount
        import os as _os
        _web_dist = _os.path.normpath(
            _os.path.join(_os.path.dirname(__file__), "..", "..", "web", "dist")
        )
        if _os.path.isdir(_web_dist):
            from starlette.staticfiles import StaticFiles
            app.mount("/", StaticFiles(directory=_web_dist, html=True), name="dashboard")
            logger.info("Dashboard served from %s", _web_dist)

        return app

    def _run_server(self) -> None:
        if self._app is None:
            return
        # Reduce GIL switch interval so uvicorn's event loop gets CPU time
        # even when LiDAR/DDS callbacks are active in other threads.
        # Default is 5ms; 1ms gives the event loop ~10x more scheduling chances.
        import sys
        old_interval = sys.getswitchinterval()
        sys.setswitchinterval(0.001)
        try:
            import uvicorn
            config = uvicorn.Config(
                self._app,
                host=self._host,
                port=self._port,
                log_level="warning",
                loop="uvloop",
                ws="auto",
                lifespan="off",
                timeout_keep_alive=30,
                ws_max_size=2 * 1024 * 1024,  # 2 MB — enough for 1080p JPEG
            )
            server = uvicorn.Server(config)
            logger.info("uvicorn server.run() starting on %s:%d", self._host, self._port)
            server.run()
            logger.error("uvicorn server.run() returned unexpectedly")
        except ImportError:
            logger.error("uvicorn not installed — run: pip install 'uvicorn[standard]'")
        except Exception:
            logger.exception("uvicorn crashed")
        finally:
            sys.setswitchinterval(old_interval)

    # -- health -------------------------------------------------------------

    def health(self) -> dict[str, Any]:
        info = super().port_summary()
        with self._sse_lock:
            n_sse = len(self._sse_queues)
        with self._map_cloud_lock:
            map_pts = len(self._map_points) if self._map_points is not None else 0
        info["gateway"] = {
            "port":        self._port,
            "sse_clients": n_sse,
            "teleop_clients": self._teleop_clients,
            "teleop_active": self._teleop_active,
            "has_odom":    self._odom is not None,
            "has_sg":      self._sg_json != "{}",
            "map_points":  map_pts,
        }
        return info

    # -- SLAM Hz measurement (subprocess-based, cached) -----------------------

    def _get_slam_hz_cached(self) -> float:
        """Measure SLAM topic Hz via background thread, cached for 4s."""
        now = time.time()
        if not hasattr(self, "_slam_hz_value"):
            self._slam_hz_value = 0.0
            self._slam_hz_last_update = 0.0
            self._slam_hz_lock = threading.Lock()

        if now - self._slam_hz_last_update > 4.0:
            with self._slam_hz_lock:
                if now - self._slam_hz_last_update > 4.0:
                    self._slam_hz_last_update = now
                    threading.Thread(target=self._measure_slam_hz, daemon=True).start()
        return round(self._slam_hz_value, 1)

    def _measure_slam_hz(self) -> None:
        """Spawn ros2 topic hz, parse rate, update cache. Runs in background thread."""
        import re
        import subprocess
        try:
            r = subprocess.run(
                ["bash", "-c",
                 "source /opt/ros/humble/setup.bash && "
                 "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && "
                 "timeout 3 ros2 topic hz /nav/odometry 2>&1 | tail -5"],
                capture_output=True, text=True, timeout=5)
            m = re.search(r"average rate: ([\d.]+)", r.stdout)
            if m:
                self._slam_hz_value = float(m.group(1))
        except Exception:
            pass

    # -- Map 3D viewer HTML generation ----------------------------------------

    def _generate_viewer_live(self) -> str:
        """Snapshot ikd-tree to temp PCD, then render. Shows exactly what save_map would produce."""
        import os
        import subprocess
        import tempfile
        tmp = os.path.join(tempfile.gettempdir(), "lingtu_live_snapshot.pcd")
        try:
            subprocess.run(
                ["bash", "-c",
                 "source /opt/ros/humble/setup.bash && "
                 "source ~/data/SLAM/navigation/install/setup.bash 2>/dev/null; "
                 "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && "
                 f"ros2 service call /nav/save_map interface/srv/SaveMaps "
                 f"\"{{file_path: '{tmp}'}}\""],
                capture_output=True, text=True, timeout=15)
        except Exception:
            pass

        if os.path.isfile(tmp) and os.path.getsize(tmp) > 200:
            # Parse temp PCD
            with open(tmp, "rb") as f:
                n_points = 0
                point_step = 16
                while True:
                    line = f.readline().decode("ascii", errors="ignore").strip()
                    if "POINTS" in line:
                        n_points = int(line.split()[-1])
                    if "SIZE" in line:
                        point_step = sum(int(s) for s in line.split()[1:])
                    if line.startswith("DATA"):
                        break
                data = f.read(n_points * point_step)
            pts = np.frombuffer(data[:n_points * point_step], dtype=np.float32).reshape(n_points, point_step // 4)[:, :3]
            valid = np.isfinite(pts).all(axis=1)
            pts = pts[valid]
            with self._map_cloud_lock:
                self._map_points = pts
            return self._generate_viewer_html(robot_pos=self._odom)
        else:
            return "<html><body style='background:#0a0a0f;color:#fff;font-family:monospace;padding:40px'><h2>暂无地图数据</h2><p>开始建图并移动机器人后刷新。</p></body></html>"

    def _generate_viewer_from_pcd(self, map_name: str) -> str:
        """Load a saved PCD file and generate viewer HTML (does NOT touch live data)."""
        import os
        map_dir = os.environ.get("NAV_MAP_DIR", os.path.expanduser("~/data/inovxio/data/maps"))
        pcd_path = os.path.join(map_dir, map_name, "map.pcd")
        if not os.path.isfile(pcd_path):
            return f"<html><body style='background:#0a0a0f;color:#fff;font-family:monospace;padding:40px'><h2>地图不存在: {map_name}</h2><p>找不到 {pcd_path}</p></body></html>"

        # Parse PCD binary
        with open(pcd_path, "rb") as f:
            n_points = 0
            point_step = 16
            while True:
                line = f.readline().decode("ascii", errors="ignore").strip()
                if "POINTS" in line:
                    n_points = int(line.split()[-1])
                if "SIZE" in line:
                    point_step = sum(int(s) for s in line.split()[1:])
                if line.startswith("DATA"):
                    break
            data = f.read(n_points * point_step)

        pts = np.frombuffer(data[:n_points * point_step], dtype=np.float32).reshape(n_points, point_step // 4)[:, :3]
        valid = np.isfinite(pts).all(axis=1)
        pts = pts[valid]
        if len(pts) > 0:
            med = np.median(pts, axis=0)
            dist = np.abs(pts - med).max(axis=1)
            pts = pts[dist < 100]

        # Generate viewer with loaded data directly (no shared state modification)
        return self._generate_viewer_html(override_pts=pts)

    def _generate_viewer_html(self, override_pts=None, robot_pos=None) -> str:
        if override_pts is not None:
            pts = override_pts
        else:
            with self._map_cloud_lock:
                pts = self._map_points
        if pts is None or len(pts) == 0:
            return ("<html><body style='background:#05060d;color:#4488ff;"
                    "font-family:monospace;padding:40px'>"
                    "<h2>暂无地图数据</h2><p>开始建图并移动机器人后刷新。</p></body></html>")

        # Subsample for browser performance
        max_pts = 80000
        if len(pts) > max_pts:
            idx = np.random.choice(len(pts), max_pts, replace=False)
            pts = pts[idx]

        z = pts[:, 2]
        zmin, zmax = float(z.min()), float(z.max())
        n = len(pts)
        coords = ",".join(f"{pts[i,0]:.3f},{pts[i,1]:.3f},{pts[i,2]:.3f}" for i in range(n))
        cx = float(pts[:, 0].mean())
        cy = float(pts[:, 1].mean())

        if robot_pos is not None:
            rx = float(robot_pos.get("x", cx))
            ry = float(robot_pos.get("y", cy))
            ryaw = float(robot_pos.get("yaw", 0.0))
            robot_visible = "true"
        else:
            rx, ry, ryaw = cx, cy, 0.0
            robot_visible = "false"

        return f'''<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<title>灵途 · LingTu</title>
<style>
*{{box-sizing:border-box;margin:0;padding:0;}}
body{{background:#05060d;overflow:hidden;font-family:-apple-system,BlinkMacSystemFont,"SF Pro Display","Helvetica Neue",sans-serif;}}
.glass{{background:rgba(4,12,35,0.78);border:1px solid rgba(40,90,200,0.30);backdrop-filter:blur(22px);-webkit-backdrop-filter:blur(22px);border-radius:14px;}}
#hud{{position:fixed;top:16px;left:16px;padding:14px 18px;min-width:210px;z-index:50;}}
.lbl{{font-size:9px;letter-spacing:2.5px;color:rgba(80,140,255,0.55);text-transform:uppercase;margin-bottom:8px;}}
.row{{font-size:12.5px;color:#8ab4ff;letter-spacing:.4px;line-height:2.0;font-variant-numeric:tabular-nums;}}
.row b{{color:#5599ff;font-weight:600;}}
#ctrlBar{{position:fixed;bottom:22px;left:50%;transform:translateX(-50%);display:flex;align-items:center;gap:10px;padding:9px 16px;z-index:50;white-space:nowrap;}}
#missionTxt{{font-size:11.5px;color:rgba(100,155,255,0.8);letter-spacing:.5px;min-width:160px;}}
.sep{{width:1px;height:22px;background:rgba(50,100,200,0.22);}}
.btn{{background:rgba(20,50,130,0.55);border:1px solid rgba(55,110,240,0.45);color:#7aacff;font-size:10.5px;letter-spacing:.8px;padding:6px 15px;border-radius:8px;cursor:pointer;font-family:inherit;transition:all .16s;outline:none;}}
.btn:hover{{background:rgba(40,85,190,0.70);color:#c0d4ff;border-color:rgba(90,150,255,0.65);}}
.btn.stop{{background:rgba(110,20,20,0.55);border-color:rgba(220,50,50,0.45);color:#ff9999;}}
.btn.stop:hover{{background:rgba(160,30,30,0.70);color:#ffcccc;}}
.btn.active{{background:rgba(120,20,20,0.55);border-color:rgba(220,60,60,0.50);color:#ffaaaa;}}
.btn.active:hover{{background:rgba(170,30,30,0.70);color:#ffdddd;}}
#hint{{position:fixed;top:16px;left:50%;transform:translateX(-50%);font-size:10px;color:rgba(50,100,200,0.42);letter-spacing:1.8px;pointer-events:none;z-index:40;}}
#toast{{position:fixed;bottom:78px;left:50%;transform:translateX(-50%);padding:9px 22px;font-size:11.5px;letter-spacing:.5px;display:none;pointer-events:none;z-index:200;color:#7ab8ff;}}
.dot{{width:6px;height:6px;border-radius:50%;background:#ff4444;display:inline-block;margin-right:8px;flex-shrink:0;transition:background .4s,box-shadow .4s;}}
.dot.live{{background:#44cc88;box-shadow:0 0 7px #44cc8899;}}
#camPanel{{position:fixed;bottom:22px;right:16px;padding:12px 14px;z-index:50;}}
#camPanel img{{width:216px;height:122px;object-fit:cover;border-radius:8px;background:#060c20;display:block;}}
#camStatus{{font-size:9px;color:rgba(80,140,255,0.50);margin-top:5px;letter-spacing:1px;text-align:center;}}
#locPanel{{position:fixed;top:16px;right:16px;padding:12px 14px;z-index:50;min-width:138px;display:none;}}
#locBtns{{display:flex;flex-direction:column;gap:5px;margin-top:2px;}}
#kbHud{{position:fixed;bottom:80px;right:16px;display:grid;grid-template-columns:repeat(3,22px);grid-template-rows:repeat(2,22px);gap:3px;opacity:0;transition:opacity .3s;z-index:60;}}
.kk{{width:22px;height:22px;border-radius:4px;background:rgba(15,40,110,0.45);border:1px solid rgba(50,100,220,0.28);font-size:9px;color:rgba(80,140,220,0.55);display:flex;align-items:center;justify-content:center;}}
.kk.on{{background:rgba(40,90,200,0.75);border-color:rgba(100,160,255,0.75);color:#c8dcff;box-shadow:0 0 6px rgba(80,140,255,0.4);}}
</style>
</head>
<body>
<script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>
<script src="https://cdn.jsdelivr.net/npm/three@0.128.0/examples/js/controls/OrbitControls.js"></script>
<script src="https://cdn.jsdelivr.net/npm/three@0.128.0/examples/js/loaders/STLLoader.js"></script>

<div id="hud" class="glass">
  <div class="lbl">灵途 · 导航状态</div>
  <div class="row">X&nbsp;<b id="hx">—</b>&nbsp;&nbsp;Y&nbsp;<b id="hy">—</b></div>
  <div class="row">模式&nbsp;<b id="hmode">—</b></div>
  <div class="row" id="hmission" style="font-size:11px;color:rgba(80,130,220,0.60);">—</div>
</div>

<div id="ctrlBar" class="glass">
  <span class="dot" id="dot"></span>
  <span id="missionTxt">待机</span>
  <div class="sep"></div>
  <button class="btn stop" onclick="doStop()">■&nbsp;停止</button>
  <button class="btn" id="expBtn" onclick="toggleExplore()">▶&nbsp;探索</button>
</div>

<div id="hint">单击地图发送导航目标 · 拖拽旋转 · 滚轮缩放 · WASD手动驾驶</div>
<div id="toast" class="glass"></div>

<!-- Camera live feed panel -->
<div id="camPanel" class="glass">
  <div class="lbl">摄像头</div>
  <img id="camImg" alt="">
  <div id="camStatus">连接中...</div>
</div>

<!-- Tagged location shortcuts -->
<div id="locPanel" class="glass">
  <div class="lbl">快速导航</div>
  <div id="locBtns"></div>
</div>

<!-- WASD keyboard indicator -->
<div id="kbHud">
  <div></div><div class="kk" id="kW">W</div><div></div>
  <div class="kk" id="kA">A</div><div class="kk" id="kS">S</div><div class="kk" id="kD">D</div>
</div>

<script>
// ── Scene setup ────────────────────────────────────────────────────────────
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x05060d);
scene.fog = new THREE.FogExp2(0x05060d, 0.0055);

const camera = new THREE.PerspectiveCamera(50, innerWidth/innerHeight, 0.1, 600);
camera.position.set({cx+5:.1f}, {cy-28:.1f}, 28);
camera.up.set(0, 0, 1);

const renderer = new THREE.WebGLRenderer({{antialias:true}});
renderer.setSize(innerWidth, innerHeight);
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.toneMapping = THREE.ACESFilmicToneMapping;
renderer.toneMappingExposure = 1.1;
document.body.appendChild(renderer.domElement);

const controls = new THREE.OrbitControls(camera, renderer.domElement);
controls.target.set({cx:.1f}, {cy:.1f}, 1.5);
controls.enableDamping = true;
controls.dampingFactor = 0.06;
controls.minDistance = 2;
controls.maxDistance = 250;
controls.update();

// ── Lighting (for STL meshes) ──────────────────────────────────────────────
scene.add(new THREE.AmbientLight(0x1a2a60, 2.5));
const dLight = new THREE.DirectionalLight(0x5588ff, 2.0);
dLight.position.set(4, -6, 10);
scene.add(dLight);
const dLight2 = new THREE.DirectionalLight(0x2244aa, 0.8);
dLight2.position.set(-4, 4, 3);
scene.add(dLight2);

// ── Point cloud — cold blue height gradient ────────────────────────────────
(function(){{
  const geo = new THREE.BufferGeometry();
  const pos = new Float32Array({n*3});
  const col = new Float32Array({n*3});
  const pts = [{coords}];
  const zmin={zmin:.3f}, zmax={zmax:.3f}, zr = zmax-zmin || 1;
  for(let i=0; i<{n}; i++){{
    pos[i*3]   = pts[i*3];
    pos[i*3+1] = pts[i*3+1];
    pos[i*3+2] = pts[i*3+2];
    const t = (pts[i*3+2] - zmin) / zr;
    // dark navy → ocean blue → bright cyan
    if(t < 0.5){{
      const s = t * 2;
      col[i*3]   = 0.04*(1-s);
      col[i*3+1] = 0.10*(1-s) + 0.33*s;
      col[i*3+2] = 0.24*(1-s) + 0.78*s;
    }} else {{
      const s = (t-0.5)*2;
      col[i*3]   = 0;
      col[i*3+1] = 0.33*(1-s) + 0.90*s;
      col[i*3+2] = 0.78*(1-s) + 1.0*s;
    }}
  }}
  geo.setAttribute('position', new THREE.BufferAttribute(pos, 3));
  geo.setAttribute('color',    new THREE.BufferAttribute(col, 3));
  scene.add(new THREE.Points(geo, new THREE.PointsMaterial({{
    size:0.035, vertexColors:true, sizeAttenuation:true, transparent:true, opacity:0.88
  }})));
}})();

// ── Ground grid ────────────────────────────────────────────────────────────
const grid = new THREE.GridHelper(100, 100, 0x0c1830, 0x070e1d);
grid.rotation.x = Math.PI/2;
grid.position.set({cx:.1f}, {cy:.1f}, {zmin:.2f}-0.01);
scene.add(grid);

// ── Robot group ────────────────────────────────────────────────────────────
const robotGroup = new THREE.Group();
robotGroup.position.set({rx:.3f}, {ry:.3f}, 0.0);
robotGroup.rotation.z = {ryaw:.4f};
robotGroup.visible = {robot_visible};
scene.add(robotGroup);

// Wireframe fallback shown while STLs load
const _fbMesh = new THREE.Mesh(
  new THREE.BoxGeometry(0.62, 0.36, 0.18),
  new THREE.MeshBasicMaterial({{color:0x2255cc, transparent:true, opacity:0.55, wireframe:true}})
);
_fbMesh.position.set(0, 0, 0.40);
robotGroup.add(_fbMesh);

// STL mesh material
const _matBody = new THREE.MeshPhongMaterial({{
  color:0xb8cdff, specular:0x4466bb, shininess:55, transparent:true, opacity:0.92
}});
const _matLeg = new THREE.MeshPhongMaterial({{
  color:0x8aaae8, specular:0x223388, shininess:38, transparent:true, opacity:0.88
}});

(function(){{
  if(typeof THREE.STLLoader === 'undefined') return;
  const loader = new THREE.STLLoader();
  let loaded = 0;
  function load(url, mat, px, py, pz){{
    loader.load(url, function(geo){{
      geo.computeVertexNormals();
      const m = new THREE.Mesh(geo, mat.clone());
      m.position.set(px, py, pz);
      robotGroup.add(m);
      loaded++;
      if(loaded === 1) _fbMesh.visible = false;
    }}, undefined, function(){{/* 404 — skip */}});
  }}
  // base_link sits ~0.40m above ground; URDF hip offsets add to that
  const bz = 0.40;
  load('/robot/meshes/base_link.STL',    _matBody,  0,       0,        bz);
  load('/robot/meshes/fr_hip_Link.STL',  _matLeg,   0.2395, -0.0646,  bz+0.0807);
  load('/robot/meshes/fl_hip_Link.STL',  _matLeg,   0.2395,  0.0654,  bz+0.0807);
  load('/robot/meshes/rr_hip_Link.STL',  _matLeg,  -0.2395, -0.0662,  bz+0.0807);
  load('/robot/meshes/rl_hip_Link.STL',  _matLeg,  -0.2395,  0.0638,  bz+0.0807);
  // Thigh links: hip_pos + thigh_joint_offset (all joints at 0)
  load('/robot/meshes/fr_thigh_Link.STL',_matLeg,   0.3175, -0.1441,  bz+0.0807);
  load('/robot/meshes/fl_thigh_Link.STL',_matLeg,   0.3175,  0.1449,  bz+0.0807);
  load('/robot/meshes/rr_thigh_Link.STL',_matLeg,  -0.1615, -0.1457,  bz+0.0807);
  load('/robot/meshes/rl_thigh_Link.STL',_matLeg,  -0.1615,  0.1433,  bz+0.0807);
}})();

// ── Pulsing position ring (stays at ground level, outside robot group) ─────
const _ringMat = new THREE.MeshBasicMaterial({{
  color:0x3377ff, transparent:true, opacity:0.55, side:THREE.DoubleSide
}});
const _ringMesh = new THREE.Mesh(new THREE.RingGeometry(0.38, 0.52, 56), _ringMat);
_ringMesh.rotation.x = Math.PI/2;
_ringMesh.position.set({rx:.3f}, {ry:.3f}, 0.015);
_ringMesh.visible = {robot_visible};
scene.add(_ringMesh);

// ── Goal marker ────────────────────────────────────────────────────────────
const goalGroup = new THREE.Group();
goalGroup.visible = false;
goalGroup.add(Object.assign(
  new THREE.Mesh(new THREE.SphereGeometry(0.13,20,20),
    new THREE.MeshBasicMaterial({{color:0x00e5ff,transparent:true,opacity:0.88}})),
  {{position: new THREE.Vector3(0,0,0.13)}}
));
const _gRing1 = new THREE.Mesh(
  new THREE.RingGeometry(0.18,0.30,48),
  new THREE.MeshBasicMaterial({{color:0x00e5ff,transparent:true,opacity:0.55,side:THREE.DoubleSide}})
);
_gRing1.rotation.x = Math.PI/2;
goalGroup.add(_gRing1);
const _gRing2 = new THREE.Mesh(
  new THREE.RingGeometry(0.34,0.46,48),
  new THREE.MeshBasicMaterial({{color:0x00e5ff,transparent:true,opacity:0.22,side:THREE.DoubleSide}})
);
_gRing2.rotation.x = Math.PI/2;
goalGroup.add(_gRing2);
scene.add(goalGroup);

// ── Trajectory — rolling ring-buffer with fade gradient ────────────────────
const _trajMax = 2000;
const _trajPos = new Float32Array(_trajMax*3);
const _trajCol = new Float32Array(_trajMax*3);
const _trajGeo = new THREE.BufferGeometry();
_trajGeo.setAttribute('position', new THREE.BufferAttribute(_trajPos, 3));
_trajGeo.setAttribute('color',    new THREE.BufferAttribute(_trajCol, 3));
_trajGeo.setDrawRange(0, 0);
scene.add(new THREE.Line(_trajGeo, new THREE.LineBasicMaterial({{
  vertexColors:true, transparent:true, opacity:0.9
}})));
let _trajN = 0;

// ── Planned path ───────────────────────────────────────────────────────────
const _pathGeo = new THREE.BufferGeometry();
scene.add(new THREE.Line(_pathGeo, new THREE.LineBasicMaterial({{
  color:0x00ffaa, transparent:true, opacity:0.72
}})));

// ── Costmap overlay ─────────────────────────────────────────────────────────
const _cmCanvas = document.createElement('canvas');
const _cmCtx = _cmCanvas.getContext('2d');
const _cmTex = new THREE.CanvasTexture(_cmCanvas);
let _cmMesh = new THREE.Mesh(
  new THREE.PlaneGeometry(1,1),
  new THREE.MeshBasicMaterial({{map:_cmTex,transparent:true,opacity:0.50,side:THREE.DoubleSide,depthWrite:false}})
);
_cmMesh.visible = false;
scene.add(_cmMesh);

// ── Scene-graph object markers ─────────────────────────────────────────────
let _sgMarkers = [];
const _sgColors = {{
  person:0xff4455, chair:0x44aaff, door:0xffcc00, table:0x88ddff,
  charging_station:0x00ff88, dog:0xff8844, cat:0xffaa44, box:0xaaaaff
}};

// ── UI refs & helpers ──────────────────────────────────────────────────────
const $dot = document.getElementById('dot');
const $mission = document.getElementById('missionTxt');
const $hx = document.getElementById('hx');
const $hy = document.getElementById('hy');
const $hmode = document.getElementById('hmode');
const $hmission = document.getElementById('hmission');
const $toast = document.getElementById('toast');
const $expBtn = document.getElementById('expBtn');
let _exploring = false;

function showToast(msg, ms){{
  $toast.textContent=msg; $toast.style.display='block';
  clearTimeout($toast._t);
  $toast._t=setTimeout(()=>{{$toast.style.display='none';}}, ms||3000);
}}
function doStop(){{
  fetch('/api/v1/stop',{{method:'POST'}})
    .then(()=>showToast('■ 紧急停止'))
    .catch(e=>showToast('停止失败: '+e,4000));
}}
function toggleExplore(){{
  const url = _exploring ? '/api/v1/explore/stop' : '/api/v1/explore/start';
  fetch(url,{{method:'POST'}}).then(r=>r.json()).then(()=>{{
    showToast(_exploring ? '探索已停止' : '自主探索启动');
  }}).catch(e=>showToast('探索失败: '+e,4000));
}}
function _setExploring(v){{
  _exploring=v;
  $expBtn.textContent = v ? '■ 停止探索' : '▶ 探索';
  $expBtn.className = v ? 'btn active' : 'btn';
  $mission.textContent = v ? '自主探索中...' : '待机';
}}

// ── Click-to-navigate ──────────────────────────────────────────────────────
const _ray = new THREE.Raycaster();
const _mv  = new THREE.Vector2();
const _gp  = new THREE.Plane(new THREE.Vector3(0,0,1), 0);
let _md    = null;
renderer.domElement.addEventListener('mousedown', e=>{{_md={{x:e.clientX,y:e.clientY}};}});
renderer.domElement.addEventListener('mouseup',   e=>{{
  if(!_md) return;
  const dx=e.clientX-_md.x, dy=e.clientY-_md.y; _md=null;
  if(dx*dx+dy*dy>25) return;
  _mv.x=(e.clientX/innerWidth)*2-1;
  _mv.y=-(e.clientY/innerHeight)*2+1;
  _ray.setFromCamera(_mv, camera);
  const tgt=new THREE.Vector3();
  if(!_ray.ray.intersectPlane(_gp, tgt)) return;
  goalGroup.position.set(tgt.x, tgt.y, 0);
  goalGroup.visible=true;
  fetch('/api/v1/navigate/click',{{
    method:'POST', headers:{{'Content-Type':'application/json'}},
    body:JSON.stringify({{x:tgt.x,y:tgt.y,z:0.0}})
  }}).then(r=>r.json()).then(()=>{{
    showToast('导航目标 → ('+tgt.x.toFixed(2)+', '+tgt.y.toFixed(2)+')');
    $mission.textContent='导航中 → ('+tgt.x.toFixed(1)+', '+tgt.y.toFixed(1)+')';
  }}).catch(e=>{{
    showToast('发送失败: '+e, 4000);
    goalGroup.visible=false;
  }});
}});

// ── SSE live updates ───────────────────────────────────────────────────────
(function(){{
  function _odom(o){{
    if(!o) return;
    const ox=o.x||0, oy=o.y||0;
    robotGroup.position.set(ox, oy, 0);
    robotGroup.rotation.z = o.yaw||0;
    robotGroup.visible = true;
    _ringMesh.position.set(ox, oy, 0.015);
    _ringMesh.visible = true;
    $hx.textContent = ox.toFixed(2);
    $hy.textContent = oy.toFixed(2);
    // Append to trajectory ring-buffer
    if(_trajN < _trajMax){{
      _trajPos[_trajN*3]=ox; _trajPos[_trajN*3+1]=oy; _trajPos[_trajN*3+2]=0.08;
      _trajN++;
    }} else {{
      _trajPos.copyWithin(0,3);
      _trajPos[(_trajMax-1)*3]=ox; _trajPos[(_trajMax-1)*3+1]=oy; _trajPos[(_trajMax-1)*3+2]=0.08;
    }}
    // Rebuild vertex color fade (old=dim, new=bright blue)
    const n=Math.min(_trajN,_trajMax);
    for(let i=0;i<n;i++){{
      const f=i/n;
      _trajCol[i*3]  =0.04+0.22*f;
      _trajCol[i*3+1]=0.20+0.47*f;
      _trajCol[i*3+2]=0.80+0.20*f;
    }}
    _trajGeo.attributes.position.needsUpdate=true;
    _trajGeo.attributes.color.needsUpdate=true;
    _trajGeo.setDrawRange(0,_trajN);
  }}
  function _path(pts){{
    if(!pts||!pts.length) return;
    const a=new Float32Array(pts.length*3);
    for(let i=0;i<pts.length;i++){{a[i*3]=pts[i].x;a[i*3+1]=pts[i].y;a[i*3+2]=0.05;}}
    _pathGeo.setAttribute('position',new THREE.BufferAttribute(a,3));
    _pathGeo.attributes.position.needsUpdate=true;
  }}
  function _mission(m){{
    if(!m) return;
    const s=m.state||m.status||'';
    if(s){{
      $mission.textContent=s; $hmission.textContent=s;
      if(s==='idle'||s==='arrived'||s==='success') goalGroup.visible=false;
    }}
  }}
  function _mode(md){{
    if(md) $hmode.textContent=typeof md==='string'?md:(md.mode||md);
  }}
  function _costmap(ev){{
    const cols=ev.cols, res=ev.resolution, ox=ev.origin[0], oy=ev.origin[1], sz=cols*res;
    const bin=atob(ev.grid_b64);
    const arr=new Uint8Array(bin.length);
    for(let i=0;i<bin.length;i++) arr[i]=bin.charCodeAt(i);
    if(_cmCanvas.width!==cols){{_cmCanvas.width=cols;_cmCanvas.height=cols;}}
    const img=_cmCtx.createImageData(cols,cols);
    for(let i=0;i<cols*cols;i++){{
      const v=arr[i];
      if(v<=0){{img.data[i*4+3]=0;continue;}}
      img.data[i*4]  =v>50?215:175;
      img.data[i*4+1]=v>50? 42: 85;
      img.data[i*4+2]=v>50? 28: 28;
      img.data[i*4+3]=Math.min(205,v*2+55);
    }}
    _cmCtx.putImageData(img,0,0);
    _cmTex.needsUpdate=true;
    _cmMesh.geometry.dispose();
    _cmMesh.geometry=new THREE.PlaneGeometry(sz,sz);
    _cmMesh.position.set(ox+sz/2, oy+sz/2, 0.01);
    _cmMesh.visible=true;
  }}
  function _connect(){{
    const es=new EventSource('/api/v1/events');
    es.onopen=()=>$dot.classList.add('live');
    es.onmessage=function(e){{
      try{{
        const ev=JSON.parse(e.data);
        if(ev.type==='snapshot'){{
          if(ev.data){{ _odom(ev.data.odometry); _mission(ev.data.mission); _mode(ev.data.mode); }}
        }}
        else if(ev.type==='odometry')    _odom(ev.data);
        else if(ev.type==='global_path') _path(ev.points);
        else if(ev.type==='mission')     _mission(ev.data);
        else if(ev.type==='mode')        _mode(ev.data);
        else if(ev.type==='costmap')     _costmap(ev);
        else if(ev.type==='exploring')   _setExploring(ev.active);
        else if(ev.type==='scene_graph') _sgUpdate(ev.objects||[]);
      }}catch(_){{}}
    }};
    es.onerror=function(){{$dot.classList.remove('live');es.close();setTimeout(_connect,3000);}};
  }}
  function _sgUpdate(objects){{
    _sgMarkers.forEach(m=>scene.remove(m)); _sgMarkers=[];
    objects.forEach(function(obj){{
      const color=_sgColors[obj.label]||0xaaaaff;
      const mat=new THREE.MeshBasicMaterial({{color,transparent:true,opacity:0.45+0.45*obj.conf}});
      const sphere=new THREE.Mesh(new THREE.SphereGeometry(0.10,10,10),mat);
      sphere.position.set(obj.x,obj.y,0.55); scene.add(sphere); _sgMarkers.push(sphere);
      const lg=new THREE.BufferGeometry().setFromPoints([
        new THREE.Vector3(obj.x,obj.y,0.05),new THREE.Vector3(obj.x,obj.y,0.45)]);
      const line=new THREE.Line(lg,new THREE.LineBasicMaterial({{color,transparent:true,opacity:0.35}}));
      scene.add(line); _sgMarkers.push(line);
    }});
  }}
  _connect();
}})();

// ── Camera snapshot panel ──────────────────────────────────────────────────
(function(){{
  const img=document.getElementById('camImg');
  const status=document.getElementById('camStatus');
  let _ok=false;
  function refresh(){{
    fetch('/api/v1/camera/snapshot?t='+Date.now())
      .then(r=>{{if(!r.ok)throw new Error(r.status);return r.blob();}})
      .then(blob=>{{
        const prev=img.src; img.src=URL.createObjectURL(blob);
        if(prev&&prev.startsWith('blob:'))URL.revokeObjectURL(prev);
        status.textContent=new Date().toLocaleTimeString('zh-CN',{{hour12:false}});
        _ok=true;
      }})
      .catch(()=>{{status.textContent='摄像头离线';_ok=false;}})
      .finally(()=>setTimeout(refresh,_ok?900:4000));
  }}
  refresh();
}})();

// ── Tagged location shortcuts ──────────────────────────────────────────────
(function(){{
  fetch('/api/v1/locations').then(r=>r.json()).then(function(data){{
    const locs=data.locations||[];
    if(!locs.length)return;
    const panel=document.getElementById('locPanel');
    const btns=document.getElementById('locBtns');
    panel.style.display='block';
    locs.forEach(function(loc){{
      const b=document.createElement('button');
      b.className='btn';
      b.style.cssText='text-align:left;width:100%;font-size:10px;padding:5px 10px;';
      b.textContent='→ '+loc.name;
      b.onclick=function(){{
        fetch('/api/v1/instruction',{{method:'POST',
          headers:{{'Content-Type':'application/json'}},
          body:JSON.stringify({{text:'去'+loc.name}})}})
        .then(r=>r.json())
        .then(()=>{{showToast('导航 → '+loc.name);$mission.textContent='导航中 → '+loc.name;}})
        .catch(e=>showToast('导航失败: '+e,4000));
      }};
      btns.appendChild(b);
      // Pin on 3D map
      const pin=new THREE.Mesh(new THREE.ConeGeometry(0.12,0.35,6),
        new THREE.MeshBasicMaterial({{color:0x00ffaa,transparent:true,opacity:0.78}}));
      pin.rotation.x=Math.PI; pin.position.set(loc.x,loc.y,0.35); scene.add(pin);
      const pRing=new THREE.Mesh(new THREE.RingGeometry(0.14,0.22,32),
        new THREE.MeshBasicMaterial({{color:0x00ffaa,transparent:true,opacity:0.32,side:THREE.DoubleSide}}));
      pRing.rotation.x=Math.PI/2; pRing.position.set(loc.x,loc.y,0.015); scene.add(pRing);
    }});
  }}).catch(function(){{}});
}})();

// ── WASD keyboard control ──────────────────────────────────────────────────
(function(){{
  const kbHud=document.getElementById('kbHud');
  const kW=document.getElementById('kW'),kA=document.getElementById('kA');
  const kS=document.getElementById('kS'),kD=document.getElementById('kD');
  const _k={{}};
  let _wasMoving=false;
  document.addEventListener('keydown',function(e){{
    const k=e.key.toLowerCase();
    if(['w','a','s','d'].includes(k)){{ e.preventDefault(); _k[k]=true; kbHud.style.opacity='1'; }}
  }});
  document.addEventListener('keyup',function(e){{
    _k[e.key.toLowerCase()]=false;
  }});
  setInterval(function(){{
    kW.classList.toggle('on',!!_k['w']); kA.classList.toggle('on',!!_k['a']);
    kS.classList.toggle('on',!!_k['s']); kD.classList.toggle('on',!!_k['d']);
    const vx=(_k['w']?0.40:0)+(_k['s']?-0.40:0);
    const wz=(_k['a']?0.70:0)+(_k['d']?-0.70:0);
    if(vx!==0||wz!==0){{
      fetch('/api/v1/cmd_vel',{{method:'POST',
        headers:{{'Content-Type':'application/json'}},
        body:JSON.stringify({{vx:vx,vy:0,wz:wz}})}});
      _wasMoving=true;
    }} else if(_wasMoving){{
      fetch('/api/v1/stop',{{method:'POST'}});
      _wasMoving=false;
      setTimeout(function(){{kbHud.style.opacity='0';}},1800);
    }}
  }},100);
}})();

// ── Animation loop ─────────────────────────────────────────────────────────
let _t = 0;
(function _loop(){{
  requestAnimationFrame(_loop);
  _t += 0.016;
  // Pulse robot ring
  const rp = 0.5 + 0.5*Math.sin(_t*2.8);
  _ringMesh.material.opacity = 0.35 + 0.25*rp;
  _ringMesh.scale.setScalar(1.0 + 0.07*Math.sin(_t*2.8));
  // Pulse goal rings
  if(goalGroup.visible){{
    const gp = 0.5 + 0.5*Math.sin(_t*4.5);
    _gRing1.material.opacity = 0.28 + 0.27*gp;
    _gRing2.scale.setScalar(1.0 + 0.18*gp);
    _gRing2.material.opacity = 0.18*(1-gp);
  }}
  controls.update();
  renderer.render(scene, camera);
}})();

addEventListener('resize', ()=>{{
  camera.aspect=innerWidth/innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(innerWidth,innerHeight);
}});
</script>
</body>
</html>'''
