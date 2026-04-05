"""LingTu MCP Server — Model Context Protocol for AI agent control.

Exposes robot capabilities as MCP tools so Claude/GPT can control the robot.
Standard MCP JSON-RPC 2.0 over HTTP POST /mcp.

Tools are auto-discovered from @skill methods on all system modules at
build() time via on_system_modules().  No hardcoded tool list — every
@skill automatically becomes an MCP tool with a JSON Schema derived from
the method signature and docstring.

Gateway usage::

    claude mcp add --transport http lingtu http://192.168.66.190:8090/mcp

Module blueprint usage::

    bp.add(MCPServerModule, port=8090)
"""

from __future__ import annotations

import json
import logging
import threading
from typing import Any, Dict, List, Optional

from core.module import Module, skill
from core.stream import In, Out
from core.msgs.geometry import PoseStamped, Pose, Vector3, Quaternion, Twist
from core.msgs.nav import Odometry
from core.msgs.semantic import SceneGraph, SafetyState
from core.registry import register

logger = logging.getLogger(__name__)

MCP_PROTOCOL_VERSION = "2025-11-25"


# ---------------------------------------------------------------------------
# JSON-RPC 2.0 helpers
# ---------------------------------------------------------------------------

def _ok(req_id: Any, result: Any) -> dict:
    return {"jsonrpc": "2.0", "id": req_id, "result": result}

def _text(req_id: Any, text: str) -> dict:
    return _ok(req_id, {"content": [{"type": "text", "text": str(text)}]})

def _error(req_id: Any, code: int, msg: str) -> dict:
    return {"jsonrpc": "2.0", "id": req_id, "error": {"code": code, "message": msg}}


# ---------------------------------------------------------------------------
# MCPServerModule
# ---------------------------------------------------------------------------

@register("mcp", "server", description="MCP server for AI agent robot control")
class MCPServerModule(Module, layer=6):
    """MCP Server — exposes every robot @skill as an AI-callable tool.

    Tools are discovered at build() time via on_system_modules().
    No static list — modules register their own capabilities through @skill.

    Built-in system tools (get_health, list_modules, get_config) are
    implemented as @skill methods on this module so they appear automatically.
    """

    _run_in_main: bool = True

    # -- receive telemetry for read-only queries ----------------------------
    odometry:       In[Odometry]
    scene_graph:    In[SceneGraph]
    safety_state:   In[SafetyState]
    mission_status: In[dict]

    # -- outgoing commands --------------------------------------------------
    goal_pose:   Out[PoseStamped]
    cmd_vel:     Out[Twist]
    stop_cmd:    Out[int]
    instruction: Out[str]
    mode_cmd:    Out[str]

    def __init__(self, port: int = 8090, host: str = "0.0.0.0", **kw):
        super().__init__(**kw)
        self._port = port
        self._host = host
        self._server_thread: Optional[threading.Thread] = None

        # Cached telemetry (written by subscriptions)
        self._odom:    Optional[Dict] = None
        self._sg_json: str = "{}"
        self._safety:  Optional[Dict] = None
        self._mission: Optional[Dict] = None

        # Injected after system.start() by cli/main.py
        self._system_handle = None

        # Populated by on_system_modules() — all @skill across all modules
        self._tool_registry: Dict[str, Any] = {}   # func_name → bound method
        self._tool_list:     List[Dict] = []        # MCP tool descriptors

        # Memory / perception module references (for built-in query tools)
        self._tagged_locations_mod = None
        self._vector_memory_mod    = None
        self._episodic_mod         = None

    # -- lifecycle ----------------------------------------------------------

    def set_system_handle(self, handle: Any) -> None:
        """Inject SystemHandle so get_health / list_modules work."""
        self._system_handle = handle

    def on_system_modules(self, modules: Dict[str, Any]) -> None:
        """Auto-discover every @skill method from every running module.

        Called by Blueprint.build() after all modules are instantiated.
        Builds _tool_list (MCP descriptors) and _tool_registry (call table).
        """
        self._tool_registry = {}
        self._tool_list = []

        # Grab module references for built-in tools
        self._tagged_locations_mod = modules.get("TaggedLocationsModule")
        self._vector_memory_mod    = modules.get("VectorMemoryModule")
        self._episodic_mod         = modules.get("EpisodicMemoryModule")

        # Discover @skill from every module (including self)
        for mod_name, mod in modules.items():
            if not hasattr(mod, "get_skill_infos"):
                continue
            try:
                infos = mod.get_skill_infos()
            except Exception:
                continue
            for info in infos:
                method = getattr(mod, info.func_name, None)
                if method is None:
                    continue
                self._tool_registry[info.func_name] = method
                schema = json.loads(info.args_schema)
                desc = schema.pop("description", "")
                self._tool_list.append({
                    "name": info.func_name,
                    "description": f"[{info.class_name}] {desc}".strip(),
                    "inputSchema": schema,
                })

        # Deduplicate: if a module @skill has the same name as a built-in,
        # the module-native version takes priority (last write in _tool_registry wins).
        # Rebuild _tool_list to reflect the deduplicated registry.
        seen: dict = {}
        for tool in self._tool_list:
            seen[tool["name"]] = tool  # last one wins (matches _tool_registry)
        self._tool_list = list(seen.values())

        logger.info(
            "MCP: %d tools from %d modules",
            len(self._tool_list), len(modules),
        )

    def setup(self) -> None:
        self.odometry.subscribe(self._on_odom)
        self.scene_graph.subscribe(self._on_sg)
        self.safety_state.subscribe(self._on_safety)
        self.mission_status.subscribe(self._on_mission)

    def start(self) -> None:
        super().start()
        self._server_thread = threading.Thread(
            target=self._run_server, daemon=True, name="mcp-server"
        )
        self._server_thread.start()
        logger.info("MCP Server at http://%s:%d/mcp", self._host, self._port)

    def stop(self) -> None:
        self._server_thread = None
        super().stop()

    # -- subscription callbacks --------------------------------------------

    def _on_odom(self, odom: Odometry) -> None:
        self._odom = {
            "x": odom.x, "y": odom.y, "z": getattr(odom, "z", 0.0),
            "yaw": odom.yaw, "vx": odom.vx, "vy": odom.vy, "ts": odom.ts,
        }

    def _on_sg(self, sg: SceneGraph) -> None:
        self._sg_json = sg.to_json() if hasattr(sg, "to_json") else str(sg)

    def _on_safety(self, state: SafetyState) -> None:
        import time
        self._safety = {"level": getattr(state, "level", 0), "ts": time.time()}

    def _on_mission(self, status: dict) -> None:
        self._mission = status

    # -- built-in @skill tools (system + perception read) ------------------

    @skill
    def get_health(self) -> str:
        """Return full system health: modules, connections, message counts."""
        if self._system_handle is None:
            return json.dumps({"error": "system handle not yet injected"})
        return json.dumps(self._system_handle.health(), default=str)

    @skill
    def list_modules(self) -> str:
        """List all deployed modules with layer, port counts, and running state."""
        if self._system_handle is None:
            return json.dumps({"error": "system handle not yet injected"})
        modules = {}
        for n, m in self._system_handle.modules.items():
            modules[n] = {
                "layer":     m.layer,
                "running":   m.running,
                "ports_in":  list(m.ports_in.keys()),
                "ports_out": list(m.ports_out.keys()),
            }
        return json.dumps({"modules": modules})

    @skill
    def get_config(self) -> str:
        """Return robot configuration: speed limits, geometry, safety thresholds."""
        try:
            from core.config import get_config
            cfg = get_config()
            return json.dumps({
                "speed":    {"max_linear": cfg.speed.max_linear,
                             "max_angular": cfg.speed.max_angular,
                             "max_speed": cfg.speed.max_speed},
                "geometry": {"height": cfg.geometry.vehicle_height,
                             "width":  cfg.geometry.vehicle_width,
                             "length": cfg.geometry.vehicle_length},
                "safety":   {"stop_distance":  cfg.safety.stop_distance,
                             "tilt_limit_deg": cfg.safety.tilt_limit_deg,
                             "obstacle_height_thre": cfg.safety.obstacle_height_thre},
            })
        except Exception as exc:
            return json.dumps({"error": str(exc)})

    @skill
    def get_robot_position(self) -> str:
        """Return the robot's current position (x, y, z) and yaw in map frame."""
        return json.dumps(self._odom or {"error": "no odometry yet"})

    @skill
    def get_scene_graph(self) -> str:
        """Return the current scene graph — detected objects with positions and labels."""
        return self._sg_json

    @skill
    def detect_objects(self, query: str) -> str:
        """Search the current scene for objects matching *query* (case-insensitive)."""
        q = query.lower()
        try:
            sg = json.loads(self._sg_json)
            matches = [o for o in sg.get("objects", [])
                       if q in o.get("label", "").lower()]
            return json.dumps({"query": q, "matches": matches, "count": len(matches)})
        except Exception:
            return json.dumps({"query": q, "matches": [], "count": 0})

    @skill
    def query_memory(self, query: str) -> str:
        """Search episodic, spatial, and vector memory for past observations."""
        results = []

        vm = self._vector_memory_mod
        if vm and hasattr(vm, "query_location"):
            try:
                r = vm.query_location(query)
                for hit in r.get("results", [])[:3]:
                    results.append({
                        "source":   "vector",
                        "position": [hit.get("x", 0), hit.get("y", 0)],
                        "score":    hit.get("score", 0),
                        "labels":   hit.get("labels", ""),
                    })
            except Exception:
                pass

        em = self._episodic_mod
        if em and hasattr(em, "memory"):
            try:
                for r in em.memory.query_by_text(query, top_k=3):
                    results.append({
                        "source":   "episodic",
                        "label":    getattr(r, "label", ""),
                        "position": list(getattr(r, "position", [0, 0, 0])),
                        "ts":       getattr(r, "timestamp", 0),
                    })
            except Exception:
                pass

        tl = self._tagged_locations_mod
        if tl and hasattr(tl, "store"):
            try:
                match = tl.store.query_fuzzy(query)
                if match:
                    results.append({
                        "source":   "tagged",
                        "label":    match["name"],
                        "position": match["position"],
                    })
            except Exception:
                pass

        return json.dumps({"query": query, "results": results, "count": len(results)})

    @skill
    def list_tagged_locations(self) -> str:
        """List all named locations tagged by the robot or user."""
        tl = self._tagged_locations_mod
        locs = tl.store.list_all() if (tl and hasattr(tl, "store")) else []
        return json.dumps({"locations": locs})

    @skill
    def tag_location(self, name: str) -> str:
        """Save the robot's current position under *name* for future navigation."""
        if not self._odom:
            return json.dumps({"error": "no odometry — cannot tag location"})
        tl = self._tagged_locations_mod
        if not (tl and hasattr(tl, "store")):
            return json.dumps({"error": "TaggedLocationsModule not running"})
        tl.store.tag(name, x=self._odom["x"], y=self._odom["y"],
                     z=self._odom.get("z", 0))
        entry = tl.store.query(name)
        return json.dumps({"tagged": name, "position": entry})

    @skill
    def navigate_to_object(self, instruction: str) -> str:
        """Navigate to a described object or place using semantic understanding."""
        self.instruction.publish(instruction)
        return json.dumps({"status": "processing", "instruction": instruction})

    @skill
    def send_instruction(self, text: str) -> str:
        """Send a natural language instruction to the semantic planner."""
        self.instruction.publish(text)
        return json.dumps({"status": "sent", "instruction": text})

    @skill
    def stop(self) -> str:
        """Emergency stop — immediately halts all robot motion."""
        self.stop_cmd.publish(2)
        self.cmd_vel.publish(Twist())
        return json.dumps({"status": "stopped"})

    @skill
    def set_mode(self, mode: str) -> str:
        """Set robot operating mode: manual | autonomous | estop."""
        if mode not in ("manual", "autonomous", "estop"):
            return json.dumps({"error": f"invalid mode: {mode!r}"})
        self.mode_cmd.publish(mode)
        if mode == "estop":
            self.stop_cmd.publish(2)
        return json.dumps({"mode": mode})

    # -- FastAPI + MCP JSON-RPC endpoint -----------------------------------

    def _run_server(self) -> None:
        try:
            from fastapi import FastAPI
            from fastapi.middleware.cors import CORSMiddleware
            from fastapi.responses import JSONResponse
            import uvicorn
        except ImportError:
            logger.error("FastAPI not installed — run: pip install fastapi uvicorn")
            return

        app = FastAPI(title="LingTu MCP Server")
        app.add_middleware(CORSMiddleware, allow_origins=["*"],
                           allow_methods=["*"], allow_headers=["*"])
        mcp = self

        @app.post("/mcp")
        async def mcp_endpoint(request: dict):
            method  = request.get("method", "")
            params  = request.get("params") or {}
            req_id  = request.get("id")

            if req_id is None:
                return JSONResponse(status_code=204, content=None)

            if method == "initialize":
                return JSONResponse(_ok(req_id, {
                    "protocolVersion": MCP_PROTOCOL_VERSION,
                    "capabilities":    {"tools": {}},
                    "serverInfo":      {"name": "lingtu", "version": "2.0.0"},
                }))

            if method == "notifications/initialized":
                return JSONResponse(status_code=204, content=None)

            if method == "tools/list":
                return JSONResponse(_ok(req_id, {"tools": mcp._tool_list}))

            if method == "tools/call":
                name   = params.get("name", "")
                args   = params.get("arguments") or {}
                fn     = mcp._tool_registry.get(name)
                if fn is None:
                    return JSONResponse(_text(req_id, f"Unknown tool: {name!r}"))
                try:
                    result = fn(**args)
                    return JSONResponse(_text(req_id, result if result is not None else "Done."))
                except Exception as exc:
                    logger.exception("MCP tool error: %s", name)
                    return JSONResponse(_text(req_id, f"Error in '{name}': {exc}"))

            return JSONResponse(_error(req_id, -32601, f"Unknown method: {method}"))

        @app.get("/health")
        async def health():
            return {
                "status":  "ok",
                "tools":   len(mcp._tool_list),
                "port":    mcp._port,
                "has_handle": mcp._system_handle is not None,
            }

        uvicorn.run(app, host=self._host, port=self._port, log_level="warning")

    # -- Module health summary ---------------------------------------------

    def health(self) -> Dict[str, Any]:
        info = super().port_summary()
        info["mcp"] = {
            "port":       self._port,
            "tools":      len(self._tool_list),
            "has_handle": self._system_handle is not None,
        }
        return info
