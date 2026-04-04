"""GatewayModule — FastAPI HTTP / WebSocket / SSE gateway.

Replaces the legacy C++ gRPC gateway.  No protobuf, no C++ compilation —
API changes are a Python edit + restart.

Endpoints
---------
POST /api/v1/goal        — navigation goal {x, y, z, instruction?}
POST /api/v1/cmd_vel     — velocity command {vx, vy, wz}
POST /api/v1/stop        — emergency stop
POST /api/v1/instruction — natural language instruction {text}
POST /api/v1/mode        — mode switch {mode: manual|autonomous|estop}
POST /api/v1/lease       — control-lease acquire/release/renew
GET  /api/v1/status      — SSE telemetry stream (odometry, safety, mission)
GET  /api/v1/state       — full robot state snapshot (JSON)
GET  /api/v1/scene_graph — current scene graph JSON
GET  /api/v1/health      — gateway health
WS   /ws/teleop          — bidirectional teleop (receive cmd_vel, echo odom)

Blueprint usage::

    bp.add(GatewayModule, port=5050)
    # auto_wire connects odometry, scene_graph, safety_state from other modules
"""

from __future__ import annotations

import asyncio
import json
import logging
import queue
import threading
import time
from typing import Any, Dict, List, Optional

from core.module import Module
from core.stream import In, Out
from core.msgs.geometry import Pose, PoseStamped, Quaternion, Twist, Vector3
from core.msgs.nav import Odometry
from core.msgs.semantic import ExecutionEval, SafetyState, SceneGraph
from core.registry import register

logger = logging.getLogger(__name__)


@register("gateway", "fastapi", description="Python FastAPI gateway (HTTP+WS+SSE)")
class GatewayModule(Module, layer=6):
    """HTTP/WebSocket gateway — external control and telemetry.

    In:  odometry, scene_graph, safety_state, mission_status,
         execution_eval, dialogue_state
    Out: goal_pose, cmd_vel, stop_cmd, instruction, mode_cmd
    """

    # Keep in main process: binds HTTP/WS server ports.
    _run_in_main: bool = True

    # -- Inputs (module → gateway cache → SSE/REST clients) -----------------
    odometry:       In[Odometry]
    scene_graph:    In[SceneGraph]
    safety_state:   In[SafetyState]
    mission_status: In[dict]
    execution_eval: In[ExecutionEval]
    dialogue_state: In[dict]

    # -- Outputs (REST/WS clients → modules) --------------------------------
    goal_pose:   Out[PoseStamped]
    cmd_vel:     Out[Twist]
    stop_cmd:    Out[int]    # 0=clear, 1=soft stop, 2=hard stop → SafetyModule
    instruction: Out[str]    # natural language → SemanticPlannerModule
    mode_cmd:    Out[str]    # "manual" | "autonomous" | "estop"

    def __init__(self, port: int = 5050, host: str = "0.0.0.0", **kw):
        super().__init__(**kw)
        self._port = port
        self._host = host
        self._app = None
        self._server_thread: Optional[threading.Thread] = None

        # Cached state (written by subscriptions, read by HTTP handlers)
        self._latest_odom:    Optional[Dict] = None
        self._latest_sg:      Optional[str]  = None
        self._latest_safety:  Optional[Dict] = None
        self._latest_mission: Optional[Dict] = None
        self._latest_eval:    Optional[Dict] = None
        self._latest_dialogue: Optional[Dict] = None
        self._mode: str = "manual"
        self._lease_holder: Optional[str] = None
        self._lease_expiry: float = 0.0

        self._sse_queues: List[queue.Queue] = []

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
            target=self._run_server, daemon=True, name="gateway-fastapi"
        )
        self._server_thread.start()
        logger.info("GatewayModule started on %s:%d", self._host, self._port)

    def stop(self) -> None:
        self._server_thread = None  # daemon thread exits with process
        super().stop()

    # -- subscription callbacks --------------------------------------------

    def _on_odometry(self, odom: Odometry) -> None:
        self._latest_odom = {
            "x": odom.x, "y": odom.y, "z": getattr(odom, "z", 0.0),
            "vx": odom.twist.linear.x if odom.twist else 0.0,
            "wz": odom.twist.angular.z if odom.twist else 0.0,
            "ts": odom.ts,
        }
        self._push_sse({"type": "odometry", "data": self._latest_odom})

    def _on_scene_graph(self, sg: SceneGraph) -> None:
        self._latest_sg = sg.to_json() if hasattr(sg, "to_json") else str(sg)

    def _on_safety(self, state: SafetyState) -> None:
        self._latest_safety = {
            "level": state.level if hasattr(state, "level") else 0,
            "ts": time.time(),
        }
        self._push_sse({"type": "safety", "data": self._latest_safety})

    def _on_mission(self, status: dict) -> None:
        self._latest_mission = status if isinstance(status, dict) else {"raw": str(status)}
        self._push_sse({"type": "mission", "data": self._latest_mission})

    def _on_eval(self, ev: ExecutionEval) -> None:
        self._latest_eval = ev.to_dict() if hasattr(ev, "to_dict") else {"raw": str(ev)}
        self._push_sse({"type": "eval", "data": self._latest_eval})

    def _on_dialogue(self, state: dict) -> None:
        self._latest_dialogue = state if isinstance(state, dict) else {"raw": str(state)}
        self._push_sse({"type": "dialogue", "data": self._latest_dialogue})

    # -- internal helpers ---------------------------------------------------

    def _check_lease(self, client_id: str) -> bool:
        if self._lease_holder is None:
            return True  # open access when no lease is held
        return self._lease_holder == client_id and time.time() < self._lease_expiry

    def _push_sse(self, event: dict) -> None:
        dead = []
        for q in self._sse_queues:
            try:
                q.put_nowait(event)
            except Exception:
                dead.append(q)
        for q in dead:
            try:
                self._sse_queues.remove(q)
            except ValueError:
                pass

    # -- FastAPI app --------------------------------------------------------

    def _build_app(self):
        try:
            from fastapi import FastAPI, WebSocket, WebSocketDisconnect
            from fastapi.middleware.cors import CORSMiddleware
            from fastapi.responses import JSONResponse, StreamingResponse
        except ImportError:
            logger.error("FastAPI not installed — run: pip install fastapi uvicorn")
            return None

        app = FastAPI(title="LingTu Gateway", version="2.0")
        app.add_middleware(
            CORSMiddleware, allow_origins=["*"],
            allow_methods=["*"], allow_headers=["*"],
        )
        gw = self  # closure reference

        # ── Navigation ─────────────────────────────────────────────────────

        @app.post("/api/v1/goal")
        async def post_goal(body: dict):
            x, y, z = body.get("x", 0.0), body.get("y", 0.0), body.get("z", 0.0)
            gw.goal_pose.publish(PoseStamped(
                pose=Pose(position=Vector3(x, y, z), orientation=Quaternion(0, 0, 0, 1)),
                frame_id="map", ts=time.time(),
            ))
            if inst := body.get("instruction", ""):
                gw.instruction.publish(inst)
            return {"status": "ok", "goal": [x, y, z]}

        @app.post("/api/v1/cmd_vel")
        async def post_cmd_vel(body: dict):
            gw.cmd_vel.publish(Twist(
                linear=Vector3(body.get("vx", 0), body.get("vy", 0), 0),
                angular=Vector3(0, 0, body.get("wz", 0)),
            ))
            return {"status": "ok"}

        @app.post("/api/v1/stop")
        async def post_stop():
            gw.stop_cmd.publish(2)
            gw.cmd_vel.publish(Twist())
            return {"status": "stopped"}

        @app.post("/api/v1/instruction")
        async def post_instruction(body: dict):
            text = body.get("text", "")
            gw.instruction.publish(text)
            return {"status": "ok", "instruction": text}

        # ── Mode ───────────────────────────────────────────────────────────

        @app.post("/api/v1/mode")
        async def post_mode(body: dict):
            mode = body.get("mode", "manual")
            if mode not in ("manual", "autonomous", "estop"):
                return JSONResponse({"error": f"invalid mode: {mode}"}, status_code=400)
            gw._mode = mode
            gw.mode_cmd.publish(mode)
            if mode == "estop":
                gw.stop_cmd.publish(2)
                gw.cmd_vel.publish(Twist())
            return {"status": "ok", "mode": mode}

        # ── Lease ──────────────────────────────────────────────────────────

        @app.post("/api/v1/lease")
        async def post_lease(body: dict):
            action    = body.get("action", "acquire")
            client_id = body.get("client_id", "unknown")
            ttl       = float(body.get("ttl", 30.0))

            if action == "acquire":
                if (gw._lease_holder and gw._lease_holder != client_id
                        and time.time() < gw._lease_expiry):
                    return JSONResponse(
                        {"error": f"lease held by {gw._lease_holder}"}, status_code=409
                    )
                gw._lease_holder = client_id
                gw._lease_expiry = time.time() + ttl
                return {"status": "acquired", "client_id": client_id, "ttl": ttl}

            if action == "release":
                if gw._lease_holder == client_id:
                    gw._lease_holder = None
                    gw._lease_expiry = 0.0
                return {"status": "released"}

            if action == "renew":
                if gw._lease_holder == client_id:
                    gw._lease_expiry = time.time() + ttl
                    return {"status": "renewed", "ttl": ttl}
                return JSONResponse({"error": "not lease holder"}, status_code=403)

            return JSONResponse({"error": f"unknown action: {action}"}, status_code=400)

        # ── Telemetry ──────────────────────────────────────────────────────

        @app.get("/api/v1/status")
        async def sse_status():
            """Server-Sent Events stream: odometry, safety, mission updates."""
            q: queue.Queue = queue.Queue(maxsize=50)
            gw._sse_queues.append(q)

            async def _generate():
                try:
                    while True:
                        try:
                            event = q.get_nowait()
                            yield f"data: {json.dumps(event)}\n\n"
                        except queue.Empty:
                            yield f"data: {json.dumps({'type': 'ping'})}\n\n"
                        await asyncio.sleep(0.1)
                finally:
                    try:
                        gw._sse_queues.remove(q)
                    except ValueError:
                        pass

            return StreamingResponse(_generate(), media_type="text/event-stream")

        @app.get("/api/v1/scene_graph")
        async def get_scene_graph():
            return JSONResponse({"scene_graph": gw._latest_sg or "{}"})

        @app.get("/api/v1/state")
        async def get_full_state():
            return JSONResponse({
                "odometry":  gw._latest_odom,
                "safety":    gw._latest_safety,
                "mission":   gw._latest_mission,
                "eval":      gw._latest_eval,
                "dialogue":  gw._latest_dialogue,
                "mode":      gw._mode,
                "lease": {
                    "holder":  gw._lease_holder,
                    "expires": gw._lease_expiry,
                    "active":  (gw._lease_holder is not None
                                and time.time() < gw._lease_expiry),
                },
            })

        @app.get("/api/v1/health")
        async def get_health():
            return JSONResponse({
                "gateway":     "running",
                "mode":        gw._mode,
                "odometry":    gw._latest_odom is not None,
                "scene_graph": gw._latest_sg is not None,
                "safety":      gw._latest_safety,
                "mission":     gw._latest_mission is not None,
                "port":        gw._port,
                "sse_clients": len(gw._sse_queues),
            })

        # ── WebSocket teleop ───────────────────────────────────────────────

        @app.websocket("/ws/teleop")
        async def ws_teleop(websocket: WebSocket):
            await websocket.accept()
            try:
                while True:
                    data = await websocket.receive_json()
                    if data.get("type") == "cmd_vel":
                        gw.cmd_vel.publish(Twist(
                            linear=Vector3(data.get("vx", 0), data.get("vy", 0), 0),
                            angular=Vector3(0, 0, data.get("wz", 0)),
                        ))
                    elif data.get("type") == "stop":
                        gw.stop_cmd.publish(2)
                    if gw._latest_odom:
                        await websocket.send_json(
                            {"type": "odometry", "data": gw._latest_odom}
                        )
            except WebSocketDisconnect:
                pass

        return app

    def _run_server(self) -> None:
        if self._app is None:
            return
        try:
            import uvicorn
            uvicorn.run(self._app, host=self._host, port=self._port, log_level="warning")
        except ImportError:
            logger.error("uvicorn not installed — run: pip install uvicorn")

    # -- health -------------------------------------------------------------

    def health(self) -> Dict[str, Any]:
        info = super().port_summary()
        info["gateway"] = {
            "port":        self._port,
            "sse_clients": len(self._sse_queues),
            "has_odom":    self._latest_odom is not None,
            "has_sg":      self._latest_sg is not None,
        }
        return info
