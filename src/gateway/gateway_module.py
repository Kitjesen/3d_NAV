"""GatewayModule — Python FastAPI gateway replacing C++ gRPC gateway.

dimos-style: HTTP + WebSocket + SSE for remote control and telemetry.
No protobuf, no C++ compilation. Changes to API = change Python, restart.

Endpoints:
  POST /api/v1/goal          — send navigation goal {x, y, z, instruction}
  POST /api/v1/cmd_vel       — velocity command {vx, vy, wz}
  POST /api/v1/stop          — emergency stop
  POST /api/v1/instruction   — natural language instruction
  GET  /api/v1/status        — SSE telemetry stream (odometry, safety, status)
  GET  /api/v1/scene_graph   — current scene graph JSON
  GET  /api/v1/health        — system health
  WS   /ws/teleop            — bidirectional teleop (binary velocity commands)

Usage in Blueprint::

    bp.add(GatewayModule, port=5050)
    # auto_wire connects odometry, scene_graph, safety_state from other modules
"""

from __future__ import annotations

import asyncio
import json
import logging
import threading
import time
from typing import Any, Dict, Optional

from core.module import Module
from core.stream import In, Out
from core.msgs.geometry import Twist, Vector3, PoseStamped, Pose, Quaternion
from core.msgs.semantic import SceneGraph, SafetyState
from core.msgs.nav import Odometry
from core.registry import register

logger = logging.getLogger(__name__)


@register("gateway", "fastapi", description="Python FastAPI gateway (HTTP+WS+SSE)")
class GatewayModule(Module, layer=6):
    """HTTP/WebSocket gateway — replaces C++ gRPC gateway.

    In:  odometry, scene_graph, safety_state (from other modules)
    Out: goal_pose, cmd_vel, stop_cmd, instruction (to other modules)
    """

    # Receive from modules → stream to clients
    odometry: In[Odometry]
    scene_graph: In[SceneGraph]
    safety_state: In[SafetyState]
    mission_status: In[dict]       # from MissionArcModule
    execution_eval: In[dict]       # from EvaluatorModule
    dialogue_state: In[dict]       # from DialogueModule

    # Receive from clients → publish to modules
    goal_pose: Out[PoseStamped]
    cmd_vel: Out[Twist]
    stop_cmd: Out[int]             # 0=clear, 1=soft, 2=hard (→ SafetyModule)
    instruction: Out[str]          # natural language → GoalResolverModule
    mode_cmd: Out[str]             # "manual"/"autonomous"/"estop" → ModeManager

    def __init__(self, port: int = 5050, host: str = "0.0.0.0", **kw):
        super().__init__(**kw)
        self._port = port
        self._host = host
        self._app = None
        self._server_thread: Optional[threading.Thread] = None
        self._latest_odom: Optional[Dict] = None
        self._latest_sg: Optional[str] = None
        self._latest_safety: Optional[Dict] = None
        self._latest_mission: Optional[Dict] = None
        self._latest_eval: Optional[Dict] = None
        self._latest_dialogue: Optional[Dict] = None
        self._mode: str = "manual"
        self._lease_holder: Optional[str] = None
        self._lease_expiry: float = 0.0
        self._sse_queues: list = []  # active SSE client queues

    def setup(self):
        self.odometry.subscribe(self._on_odometry)
        self.scene_graph.subscribe(self._on_scene_graph)
        self.safety_state.subscribe(self._on_safety)
        self.mission_status.subscribe(self._on_mission)
        self.execution_eval.subscribe(self._on_eval)
        self.dialogue_state.subscribe(self._on_dialogue)
        self._app = self._create_app()

    def start(self):
        super().start()
        self._server_thread = threading.Thread(
            target=self._run_server, daemon=True,
            name="gateway-fastapi")
        self._server_thread.start()
        logger.info("GatewayModule started on %s:%d", self._host, self._port)

    def stop(self):
        # uvicorn doesn't have clean shutdown from thread — daemon thread dies
        self._server_thread = None
        super().stop()

    # -- Data callbacks (modules → cache for clients) -------------------------

    def _on_odometry(self, odom: Odometry):
        self._latest_odom = {
            "x": odom.x, "y": odom.y, "z": getattr(odom, 'z', 0.0),
            "vx": odom.twist.linear.x if odom.twist else 0.0,
            "wz": odom.twist.angular.z if odom.twist else 0.0,
            "ts": odom.ts,
        }
        # Push to SSE clients
        self._push_sse({"type": "odometry", "data": self._latest_odom})

    def _on_scene_graph(self, sg: SceneGraph):
        self._latest_sg = sg.to_json() if hasattr(sg, 'to_json') else str(sg)

    def _on_safety(self, state: SafetyState):
        self._latest_safety = {
            "level": state.level if hasattr(state, 'level') else 0,
            "ts": time.time(),
        }
        self._push_sse({"type": "safety", "data": self._latest_safety})

    def _on_mission(self, status: dict):
        self._latest_mission = status if isinstance(status, dict) else {"raw": str(status)}
        self._push_sse({"type": "mission", "data": self._latest_mission})

    def _on_eval(self, ev: dict):
        self._latest_eval = ev if isinstance(ev, dict) else {"raw": str(ev)}
        self._push_sse({"type": "eval", "data": self._latest_eval})

    def _on_dialogue(self, state: dict):
        self._latest_dialogue = state if isinstance(state, dict) else {"raw": str(state)}
        self._push_sse({"type": "dialogue", "data": self._latest_dialogue})

    def _check_lease(self, client_id: str) -> bool:
        """Check if client holds the control lease."""
        if self._lease_holder is None:
            return True  # no lease = open access
        if self._lease_holder == client_id and time.time() < self._lease_expiry:
            return True
        return False

    def _push_sse(self, event: dict):
        dead = []
        for q in self._sse_queues:
            try:
                q.put_nowait(event)
            except Exception:
                dead.append(q)
        for q in dead:
            self._sse_queues.remove(q)

    # -- FastAPI app ----------------------------------------------------------

    def _create_app(self):
        try:
            from fastapi import FastAPI, WebSocket, WebSocketDisconnect
            from fastapi.responses import JSONResponse, StreamingResponse
            from fastapi.middleware.cors import CORSMiddleware
        except ImportError:
            logger.error("FastAPI not installed. Run: pip install fastapi uvicorn")
            return None

        import queue

        app = FastAPI(title="LingTu Gateway", version="1.0")
        app.add_middleware(CORSMiddleware, allow_origins=["*"],
                          allow_methods=["*"], allow_headers=["*"])
        gw = self  # closure ref

        @app.post("/api/v1/goal")
        async def post_goal(body: dict):
            """Send navigation goal."""
            x = body.get("x", 0.0)
            y = body.get("y", 0.0)
            z = body.get("z", 0.0)
            pose = PoseStamped(
                pose=Pose(position=Vector3(x, y, z),
                          orientation=Quaternion(0, 0, 0, 1)),
                frame_id="map", ts=time.time())
            gw.goal_pose.publish(pose)
            # Also publish instruction if provided
            inst = body.get("instruction", "")
            if inst:
                gw.instruction.publish(inst)
            return {"status": "ok", "goal": [x, y, z]}

        @app.post("/api/v1/cmd_vel")
        async def post_cmd_vel(body: dict):
            """Velocity command for teleop."""
            twist = Twist(
                linear=Vector3(body.get("vx", 0), body.get("vy", 0), 0),
                angular=Vector3(0, 0, body.get("wz", 0)))
            gw.cmd_vel.publish(twist)
            return {"status": "ok"}

        @app.post("/api/v1/stop")
        async def post_stop():
            """Emergency stop."""
            gw.stop_cmd.publish(2)
            gw.cmd_vel.publish(Twist())  # zero velocity
            return {"status": "stopped"}

        @app.post("/api/v1/instruction")
        async def post_instruction(body: dict):
            """Natural language instruction."""
            text = body.get("text", "")
            gw.instruction.publish(text)
            return {"status": "ok", "instruction": text}

        @app.get("/api/v1/status")
        async def sse_status():
            """Server-Sent Events stream for real-time telemetry."""
            q = queue.Queue(maxsize=50)
            gw._sse_queues.append(q)

            async def event_generator():
                try:
                    while True:
                        try:
                            event = q.get_nowait()
                            yield f"data: {json.dumps(event)}\n\n"
                        except queue.Empty:
                            yield f"data: {json.dumps({'type': 'ping'})}\n\n"
                        await asyncio.sleep(0.1)
                finally:
                    if q in gw._sse_queues:
                        gw._sse_queues.remove(q)

            return StreamingResponse(event_generator(),
                                     media_type="text/event-stream")

        @app.get("/api/v1/scene_graph")
        async def get_scene_graph():
            """Current scene graph."""
            return JSONResponse({"scene_graph": gw._latest_sg or "{}"})

        @app.post("/api/v1/mode")
        async def post_mode(body: dict):
            """Switch mode: manual / autonomous / estop."""
            mode = body.get("mode", "manual")
            if mode not in ("manual", "autonomous", "estop"):
                return JSONResponse({"error": f"invalid mode: {mode}"}, 400)
            gw._mode = mode
            gw.mode_cmd.publish(mode)
            if mode == "estop":
                gw.stop_cmd.publish(2)
                gw.cmd_vel.publish(Twist())
            return {"status": "ok", "mode": mode}

        @app.post("/api/v1/lease")
        async def post_lease(body: dict):
            """Acquire/release control lease (prevents multi-client conflicts)."""
            action = body.get("action", "acquire")
            client_id = body.get("client_id", "unknown")
            ttl = body.get("ttl", 30.0)  # seconds
            if action == "acquire":
                if gw._lease_holder and gw._lease_holder != client_id and time.time() < gw._lease_expiry:
                    return JSONResponse({"error": f"lease held by {gw._lease_holder}"}, 409)
                gw._lease_holder = client_id
                gw._lease_expiry = time.time() + ttl
                return {"status": "acquired", "client_id": client_id, "ttl": ttl}
            elif action == "release":
                if gw._lease_holder == client_id:
                    gw._lease_holder = None
                    gw._lease_expiry = 0.0
                return {"status": "released"}
            elif action == "renew":
                if gw._lease_holder == client_id:
                    gw._lease_expiry = time.time() + ttl
                    return {"status": "renewed", "ttl": ttl}
                return JSONResponse({"error": "not lease holder"}, 403)
            return JSONResponse({"error": f"unknown action: {action}"}, 400)

        @app.get("/api/v1/state")
        async def get_full_state():
            """Full robot state snapshot."""
            return JSONResponse({
                "odometry": gw._latest_odom,
                "safety": gw._latest_safety,
                "mission": gw._latest_mission,
                "eval": gw._latest_eval,
                "dialogue": gw._latest_dialogue,
                "mode": gw._mode,
                "lease": {
                    "holder": gw._lease_holder,
                    "expires": gw._lease_expiry,
                    "active": gw._lease_holder is not None and time.time() < gw._lease_expiry,
                },
            })

        @app.get("/api/v1/health")
        async def get_health():
            """System health."""
            return JSONResponse({
                "gateway": "running",
                "mode": gw._mode,
                "odometry": gw._latest_odom is not None,
                "scene_graph": gw._latest_sg is not None,
                "safety": gw._latest_safety,
                "mission": gw._latest_mission is not None,
                "port": gw._port,
                "sse_clients": len(gw._sse_queues),
            })

        @app.websocket("/ws/teleop")
        async def ws_teleop(websocket: WebSocket):
            """Bidirectional WebSocket for teleop."""
            await websocket.accept()
            try:
                while True:
                    data = await websocket.receive_json()
                    if data.get("type") == "cmd_vel":
                        twist = Twist(
                            linear=Vector3(data.get("vx", 0), data.get("vy", 0), 0),
                            angular=Vector3(0, 0, data.get("wz", 0)))
                        gw.cmd_vel.publish(twist)
                    elif data.get("type") == "stop":
                        gw.stop_cmd.publish(2)
                    # Send back latest odom
                    if gw._latest_odom:
                        await websocket.send_json({
                            "type": "odometry", "data": gw._latest_odom})
            except WebSocketDisconnect:
                pass

        return app

    def _run_server(self):
        if self._app is None:
            return
        try:
            import uvicorn
            uvicorn.run(self._app, host=self._host, port=self._port,
                        log_level="warning")
        except ImportError:
            logger.error("uvicorn not installed. Run: pip install uvicorn")

    # -- Health ---------------------------------------------------------------

    def health(self) -> Dict[str, Any]:
        info = super().port_summary()
        info["gateway"] = {
            "port": self._port,
            "sse_clients": len(self._sse_queues),
            "has_odom": self._latest_odom is not None,
            "has_sg": self._latest_sg is not None,
        }
        return info
