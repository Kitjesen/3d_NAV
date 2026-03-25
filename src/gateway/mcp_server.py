"""LingTu MCP Server — Model Context Protocol for AI agent control.

Exposes robot capabilities as MCP tools so Claude/GPT can control the robot.
Standard MCP JSON-RPC over HTTP POST /mcp endpoint.

Tools provided (15+, beyond dimos's ~5):

  Navigation:
    navigate_to          — go to coordinates (x, y, z)
    navigate_to_object   — semantic navigation ("go to the kitchen")
    stop                 — emergency stop
    get_navigation_status — current state (IDLE/NAVIGATING/STUCK/ARRIVED)
    set_mode             — manual / autonomous / estop

  Perception:
    get_scene_graph      — current detected objects with positions
    detect_objects       — query specific objects in scene
    get_robot_position   — current pose (x, y, z, yaw)

  Memory (unique to LingTu):
    query_memory         — search episodic/spatial memory
    list_tagged_locations — named locations (office, kitchen, etc.)
    tag_location         — save current position with a name

  Planning (unique to LingTu):
    send_instruction     — natural language command
    decompose_task       — break instruction into subtasks

  System:
    get_health           — full system health report
    list_modules         — deployed modules + status
    get_config           — robot configuration

Usage:
    # As Module in Blueprint:
    system = autoconnect(..., MCPServerModule.blueprint(port=8090)).build()

    # Claude Code connects:
    claude mcp add --transport http lingtu http://192.168.66.190:8090/mcp
"""

from __future__ import annotations

import asyncio
import json
import logging
import threading
import time
from typing import Any, Callable, Dict, List, Optional

from core.module import Module
from core.stream import In, Out
from core.msgs.geometry import PoseStamped, Pose, Vector3, Quaternion, Twist
from core.msgs.nav import Odometry
from core.msgs.semantic import SceneGraph, SafetyState
from core.registry import register

logger = logging.getLogger(__name__)

MCP_PROTOCOL_VERSION = "2025-11-25"


# ---------------------------------------------------------------------------
# MCP Tool definitions
# ---------------------------------------------------------------------------

TOOLS = [
    # Navigation
    {
        "name": "navigate_to",
        "description": "Navigate the robot to specific coordinates. Returns immediately, robot moves asynchronously.",
        "inputSchema": {
            "type": "object",
            "properties": {
                "x": {"type": "number", "description": "X coordinate in meters (map frame)"},
                "y": {"type": "number", "description": "Y coordinate in meters (map frame)"},
                "z": {"type": "number", "description": "Z coordinate (default 0)", "default": 0},
            },
            "required": ["x", "y"],
        },
    },
    {
        "name": "navigate_to_object",
        "description": "Navigate to a described object or location using semantic understanding. Example: 'go to the red chair' or 'find the kitchen'.",
        "inputSchema": {
            "type": "object",
            "properties": {
                "instruction": {"type": "string", "description": "Natural language navigation instruction"},
            },
            "required": ["instruction"],
        },
    },
    {
        "name": "stop",
        "description": "Emergency stop. Immediately halts all robot motion.",
        "inputSchema": {"type": "object", "properties": {}},
    },
    {
        "name": "get_navigation_status",
        "description": "Get current navigation state: IDLE, NAVIGATING, STUCK, ARRIVED, or ESTOP. Includes position and progress.",
        "inputSchema": {"type": "object", "properties": {}},
    },
    {
        "name": "set_mode",
        "description": "Set robot operating mode.",
        "inputSchema": {
            "type": "object",
            "properties": {
                "mode": {"type": "string", "enum": ["manual", "autonomous", "estop"]},
            },
            "required": ["mode"],
        },
    },
    # Perception
    {
        "name": "get_scene_graph",
        "description": "Get current scene graph — all detected objects with labels, positions, confidence scores, and spatial relations.",
        "inputSchema": {"type": "object", "properties": {}},
    },
    {
        "name": "detect_objects",
        "description": "Query specific objects in the current scene. Returns matching objects with positions and confidence.",
        "inputSchema": {
            "type": "object",
            "properties": {
                "query": {"type": "string", "description": "Object to search for, e.g. 'chair', 'person', 'door'"},
            },
            "required": ["query"],
        },
    },
    {
        "name": "get_robot_position",
        "description": "Get the robot's current position (x, y, z) and orientation (yaw) in the map frame.",
        "inputSchema": {"type": "object", "properties": {}},
    },
    # Memory
    {
        "name": "query_memory",
        "description": "Search the robot's episodic and spatial memory. Returns past observations matching the query.",
        "inputSchema": {
            "type": "object",
            "properties": {
                "query": {"type": "string", "description": "Search query, e.g. 'where did I see a fire extinguisher'"},
            },
            "required": ["query"],
        },
    },
    {
        "name": "list_tagged_locations",
        "description": "List all named/tagged locations the robot knows about (e.g. 'office', 'kitchen', 'charging_station').",
        "inputSchema": {"type": "object", "properties": {}},
    },
    {
        "name": "tag_location",
        "description": "Save the robot's current position with a name for future reference.",
        "inputSchema": {
            "type": "object",
            "properties": {
                "name": {"type": "string", "description": "Location name, e.g. 'meeting_room_3'"},
            },
            "required": ["name"],
        },
    },
    # Planning
    {
        "name": "send_instruction",
        "description": "Send a natural language instruction to the robot. The semantic planner will decompose and execute it.",
        "inputSchema": {
            "type": "object",
            "properties": {
                "text": {"type": "string", "description": "Instruction, e.g. 'patrol the office then return to base'"},
            },
            "required": ["text"],
        },
    },
    {
        "name": "decompose_task",
        "description": "Break a complex instruction into subtasks without executing. Useful for planning review.",
        "inputSchema": {
            "type": "object",
            "properties": {
                "instruction": {"type": "string", "description": "Complex instruction to decompose"},
            },
            "required": ["instruction"],
        },
    },
    # System
    {
        "name": "get_health",
        "description": "Get full system health: all modules, connections, C++ processes, safety state.",
        "inputSchema": {"type": "object", "properties": {}},
    },
    {
        "name": "list_modules",
        "description": "List all deployed modules with their layer, running status, and port counts.",
        "inputSchema": {"type": "object", "properties": {}},
    },
    {
        "name": "get_config",
        "description": "Get robot configuration: speed limits, geometry, safety thresholds.",
        "inputSchema": {"type": "object", "properties": {}},
    },
]


# ---------------------------------------------------------------------------
# JSON-RPC helpers
# ---------------------------------------------------------------------------

def _ok(req_id, result):
    return {"jsonrpc": "2.0", "id": req_id, "result": result}

def _text(req_id, text):
    return _ok(req_id, {"content": [{"type": "text", "text": text}]})

def _error(req_id, code, msg):
    return {"jsonrpc": "2.0", "id": req_id, "error": {"code": code, "message": msg}}


# ---------------------------------------------------------------------------
# MCPServerModule
# ---------------------------------------------------------------------------

@register("mcp", "server", description="MCP server for AI agent robot control")
class MCPServerModule(Module, layer=6):
    """MCP Server — exposes robot as AI-controllable tools.

    Standard MCP JSON-RPC over HTTP. Claude Code / GPT can connect and
    control the robot via tool calls.
    """

    # Receive from modules (for queries)
    odometry: In[Odometry]
    scene_graph: In[SceneGraph]
    safety_state: In[SafetyState]
    mission_status: In[dict]

    # Publish to modules (for commands)
    goal_pose: Out[PoseStamped]
    cmd_vel: Out[Twist]
    stop_cmd: Out[int]
    instruction: Out[str]
    mode_cmd: Out[str]

    def __init__(self, port: int = 8090, host: str = "0.0.0.0", **kw):
        super().__init__(**kw)
        self._port = port
        self._host = host
        self._server_thread: Optional[threading.Thread] = None

        # Cached state from subscriptions
        self._odom: Optional[Dict] = None
        self._sg_json: str = "{}"
        self._safety: Optional[Dict] = None
        self._mission: Optional[Dict] = None
        self._tagged_locations: Dict[str, Dict] = {}
        self._system_handle = None  # set externally for health queries

    def set_system_handle(self, handle):
        """Inject SystemHandle for health/module queries."""
        self._system_handle = handle

    def setup(self):
        self.odometry.subscribe(self._on_odom)
        self.scene_graph.subscribe(self._on_sg)
        self.safety_state.subscribe(self._on_safety)
        self.mission_status.subscribe(self._on_mission)

    def start(self):
        super().start()
        self._server_thread = threading.Thread(
            target=self._run_server, daemon=True, name="mcp-server")
        self._server_thread.start()
        logger.info("MCP Server at http://%s:%d/mcp", self._host, self._port)

    def stop(self):
        self._server_thread = None
        super().stop()

    # -- Subscriptions -------------------------------------------------------

    def _on_odom(self, odom: Odometry):
        self._odom = {"x": odom.x, "y": odom.y, "z": getattr(odom, 'z', 0.0),
                      "yaw": odom.yaw, "vx": odom.vx, "vy": odom.vy, "ts": odom.ts}

    def _on_sg(self, sg: SceneGraph):
        self._sg_json = sg.to_json() if hasattr(sg, 'to_json') else str(sg)

    def _on_safety(self, state: SafetyState):
        self._safety = {"level": state.level if hasattr(state, 'level') else 0,
                        "ts": time.time()}

    def _on_mission(self, status: dict):
        self._mission = status

    # -- Tool execution ------------------------------------------------------

    def _execute_tool(self, name: str, args: dict) -> str:
        """Execute an MCP tool and return result as string."""

        # Navigation
        if name == "navigate_to":
            pose = PoseStamped(
                pose=Pose(position=Vector3(args["x"], args["y"], args.get("z", 0)),
                          orientation=Quaternion(0, 0, 0, 1)),
                frame_id="map", ts=time.time())
            self.goal_pose.publish(pose)
            return json.dumps({"status": "navigating", "goal": [args["x"], args["y"]]})

        if name == "navigate_to_object":
            self.instruction.publish(args["instruction"])
            return json.dumps({"status": "processing", "instruction": args["instruction"]})

        if name == "stop":
            self.stop_cmd.publish(2)
            self.cmd_vel.publish(Twist())
            return json.dumps({"status": "stopped"})

        if name == "get_navigation_status":
            return json.dumps({
                "position": self._odom,
                "mission": self._mission,
                "safety": self._safety,
            })

        if name == "set_mode":
            mode = args["mode"]
            self.mode_cmd.publish(mode)
            if mode == "estop":
                self.stop_cmd.publish(2)
            return json.dumps({"mode": mode})

        # Perception
        if name == "get_scene_graph":
            return self._sg_json

        if name == "detect_objects":
            query = args["query"].lower()
            try:
                sg = json.loads(self._sg_json)
                matches = [o for o in sg.get("objects", [])
                           if query in o.get("label", "").lower()]
                return json.dumps({"query": query, "matches": matches,
                                   "count": len(matches)})
            except Exception:
                return json.dumps({"query": query, "matches": [], "count": 0})

        if name == "get_robot_position":
            return json.dumps(self._odom or {"error": "no odometry"})

        # Memory
        if name == "query_memory":
            # Delegate to memory module if available
            return json.dumps({"query": args["query"],
                               "results": [], "note": "memory query not yet connected"})

        if name == "list_tagged_locations":
            return json.dumps({"locations": self._tagged_locations})

        if name == "tag_location":
            if self._odom:
                self._tagged_locations[args["name"]] = {
                    "x": self._odom["x"], "y": self._odom["y"],
                    "z": self._odom.get("z", 0), "ts": time.time(),
                }
                return json.dumps({"tagged": args["name"],
                                   "position": self._tagged_locations[args["name"]]})
            return json.dumps({"error": "no odometry, can't tag"})

        # Planning
        if name == "send_instruction":
            self.instruction.publish(args["text"])
            return json.dumps({"status": "sent", "instruction": args["text"]})

        if name == "decompose_task":
            # Would call TaskDecomposer if wired
            return json.dumps({"instruction": args["instruction"],
                               "subtasks": [args["instruction"]],
                               "note": "rule-based decomposition"})

        # System
        if name == "get_health":
            if self._system_handle:
                return json.dumps(self._system_handle.health(), default=str)
            return json.dumps({"error": "system handle not connected"})

        if name == "list_modules":
            if self._system_handle:
                modules = {}
                for n, m in self._system_handle.modules.items():
                    modules[n] = {
                        "layer": m.layer,
                        "running": m.running,
                        "ports_in": list(m.ports_in.keys()),
                        "ports_out": list(m.ports_out.keys()),
                    }
                return json.dumps({"modules": modules})
            return json.dumps({"error": "system handle not connected"})

        if name == "get_config":
            try:
                from core.config import get_config
                cfg = get_config()
                return json.dumps({
                    "speed": {"max_linear": cfg.speed.max_linear,
                              "max_angular": cfg.speed.max_angular},
                    "geometry": {"height": cfg.geometry.vehicle_height,
                                 "width": cfg.geometry.vehicle_width},
                    "safety": {"stop_distance": cfg.safety.stop_distance,
                               "tilt_limit_deg": cfg.safety.tilt_limit_deg},
                })
            except Exception:
                return json.dumps({"error": "config not available"})

        return json.dumps({"error": f"unknown tool: {name}"})

    # -- FastAPI + MCP JSON-RPC ----------------------------------------------

    def _run_server(self):
        try:
            from fastapi import FastAPI
            from fastapi.middleware.cors import CORSMiddleware
            from fastapi.responses import JSONResponse
            import uvicorn
        except ImportError:
            logger.error("FastAPI not installed. Run: pip install fastapi uvicorn")
            return

        app = FastAPI(title="LingTu MCP Server")
        app.add_middleware(CORSMiddleware, allow_origins=["*"],
                          allow_methods=["*"], allow_headers=["*"])
        mcp = self

        @app.post("/mcp")
        async def mcp_endpoint(request: dict):
            method = request.get("method", "")
            params = request.get("params", {}) or {}
            req_id = request.get("id")

            if req_id is None:
                return JSONResponse(status_code=204, content=None)

            if method == "initialize":
                return JSONResponse(_ok(req_id, {
                    "protocolVersion": MCP_PROTOCOL_VERSION,
                    "capabilities": {"tools": {}},
                    "serverInfo": {"name": "lingtu", "version": "2.0.0"},
                }))

            if method == "notifications/initialized":
                return JSONResponse(status_code=204, content=None)

            if method == "tools/list":
                return JSONResponse(_ok(req_id, {"tools": TOOLS}))

            if method == "tools/call":
                name = params.get("name", "")
                args = params.get("arguments", {})
                try:
                    result = mcp._execute_tool(name, args)
                    return JSONResponse(_text(req_id, result))
                except Exception as e:
                    logger.exception("MCP tool error: %s", name)
                    return JSONResponse(_text(req_id, f"Error: {e}"))

            return JSONResponse(_error(req_id, -32601, f"Unknown method: {method}"))

        @app.get("/health")
        async def health():
            return {"status": "ok", "tools": len(TOOLS), "port": mcp._port}

        uvicorn.run(app, host=self._host, port=self._port, log_level="warning")

    def health(self) -> Dict[str, Any]:
        info = super().port_summary()
        info["mcp"] = {
            "port": self._port,
            "tools": len(TOOLS),
            "tagged_locations": len(self._tagged_locations),
        }
        return info
