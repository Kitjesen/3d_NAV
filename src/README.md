# src/ — LingTu Navigation System

## Quick Start

```python
from core import autoconnect
from core.registry import get
import drivers.thunder.han_dog_module

system = autoconnect(
    get("driver", "thunder").blueprint(dog_host="192.168.66.190"),
    DetectorModule.blueprint(detector="bpu"),
    EncoderModule.blueprint(encoder="mobileclip"),
    SemanticPlannerModule.blueprint(),
    LLMModule.blueprint(backend="kimi"),
    NavigationModule.blueprint(planner="astar"),
    SafetyRingModule.blueprint(),
    GatewayModule.blueprint(port=5050),
    MCPServerModule.blueprint(port=8090),
    RerunModule.blueprint(web_port=9090),
).build()

system.start()
```

## Directory Map (10 dirs)

```
src/
├── core/              Framework (Module/Blueprint/Transport/NativeModule)
│   ├── msgs/          Message types (geometry, nav, sensor, semantic)
│   ├── transport/     Backends (Local, DDS, SHM, Adapter)
│   ├── blueprints/    System blueprints (full_stack, navigation, stub)
│   ├── spec/          Protocol interfaces (16 specs)
│   └── tests/         599+ tests
│
├── nav/               Navigation core
│   ├── core/          C++ algorithm library (pybind11, zero ROS2 dep)
│   ├── rings/         Safety ring (ROS2 legacy nodes)
│   ├── services/      Map/patrol/geofence/task services
│   ├── navigation_module.py   Unified planner+tracker+FSM
│   └── safety_ring_module.py  Unified safety+eval+dialogue
│
├── semantic/          Semantic intelligence
│   ├── common/        L0 utilities (validation, sanitize)
│   ├── perception/    Perception (YOLO+CLIP+SceneGraph+Service)
│   └── planner/       Planning (GoalResolver+Frontier+LLM+Service)
│
├── drivers/           Hardware
│   ├── thunder/       ThunderDriver (gRPC → brainstem CMS)
│   ├── sim/           Stub + MuJoCo simulation
│   └── livox.../      LiDAR driver (C++)
│
├── gateway/           External interfaces
│   ├── gateway_module.py  HTTP/WS/SSE gateway (FastAPI)
│   └── mcp_server.py     MCP server (16 tools for AI agents)
│
├── base_autonomy/     C++ terrain + local planner
│   └── autonomy_module.py  Unified 4-node manager
│
├── global_planning/   C++ PCT planner + Python adapters
├── slam/              C++ SLAM (Fast-LIO2 + Point-LIO)
├── memory/            Memory layer (spatial/knowledge/storage)
└── reconstruction/    3D reconstruction
```

## Pluggable Modules (10 big modules)

| Module | Layer | Backends / Strategies |
|--------|-------|----------------------|
| ThunderDriver | L1 | thunder / stub / sim_mujoco |
| AutonomyModule | L2 | manages 4 C++ NativeModule nodes |
| DetectorModule | L3 | yoloe / yolo_world / bpu / grounding_dino |
| EncoderModule | L3 | clip / mobileclip |
| SemanticPlannerModule | L4 | resolver + frontier + decomposer + executor |
| LLMModule | L4 | kimi / openai / claude / qwen / mock |
| NavigationModule | L5 | astar / pct (planner) + tracker + FSM |
| SafetyRingModule | L0 | reflex + evaluator + dialogue |
| GatewayModule | L6 | HTTP + WebSocket + SSE |
| MCPServerModule | L6 | 16 MCP tools for AI agent control |
| RerunModule | L6 | 3D browser visualization |

## Backpressure Policies

```python
self.imu.set_policy("all")                        # every message (default)
self.image.set_policy("latest")                    # drop if busy
self.lidar.set_policy("throttle", interval=0.1)   # max 10Hz
self.scan.set_policy("sample", n=5)               # every 5th
self.detections.set_policy("buffer", size=10)      # batch of 10
```

## Transport Decoupling

```python
bp.wire("Safety", "stop_cmd", "Driver", "stop_signal")                    # callback (0 latency)
bp.wire("Perception", "scene_graph", "Planner", "scene_graph", transport="dds")  # decoupled
bp.wire("SLAM", "cloud", "Terrain", "cloud", transport="shm")             # high bandwidth
```

## MCP for AI Agents

```bash
# Connect Claude Code to the robot
claude mcp add --transport http lingtu http://192.168.66.190:8090/mcp

# Then Claude can:
# "Navigate to the kitchen"     → navigate_to_object tool
# "What objects do you see?"    → get_scene_graph tool
# "Stop!"                       → stop tool
# "Tag this as charging_station" → tag_location tool
```
