# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

LingTu (灵途) is an autonomous navigation system for quadruped robots in outdoor/off-road environments.

- **Platform**: S100P (RDK X5, Nash BPU 128 TOPS, aarch64) | ROS2 Humble | Ubuntu 22.04
- **Languages**: Python (framework + semantic modules), C++ (SLAM/terrain/planner)
- **Architecture**: dimos-style modular — 10 big Modules, autoconnect Blueprint, pluggable backends
- **Entry point**: `python main_nav.py` (replaces ros2 launch for the Module stack)

## Quick Start

```bash
# Framework tests (no ROS2 needed, runs on any machine)
python -m pytest src/core/tests/ -q       # 599 tests

# Run navigation stack (stub mode, no hardware)
python main_nav.py --robot stub --no-native --llm mock

# Run on real robot (S100P)
python main_nav.py --robot thunder --dog-host 192.168.66.190 --detector bpu --llm kimi

# ROS2 launch (legacy, still works on S100P)
source /opt/ros/humble/setup.bash
make mapping          # SLAM + sensors, manual drive
make navigation       # loads existing map
```

## Architecture — 10 Big Modules

```python
system = autoconnect(
    ThunderDriver.blueprint(dog_host="192.168.66.190"),   # L1 Robot driver
    AutonomyModule.blueprint(),                            # L2 C++ terrain+planner (4 processes)
    DetectorModule.blueprint(detector="bpu"),              # L3 Object detection (5 backends)
    EncoderModule.blueprint(encoder="mobileclip"),         # L3 Feature encoding (2 backends)
    SemanticPlannerModule.blueprint(),                     # L4 Goal resolution + frontier + decomposer
    LLMModule.blueprint(backend="kimi"),                   # L4 LLM reasoning (5 backends)
    NavigationModule.blueprint(planner="astar"),           # L5 Path planning + tracking + FSM
    SafetyRingModule.blueprint(),                          # L0 Safety + eval + dialogue
    GatewayModule.blueprint(port=5050),                    # L6 HTTP/WS/SSE gateway
    MCPServerModule.blueprint(port=8090),                  # L6 MCP server (16 AI tools)
).build()
system.start()
```

### Pluggable Backends

| Module | Backends |
|--------|----------|
| Driver | `thunder` (gRPC→brainstem), `stub` (testing), `sim_mujoco` |
| Detector | `yoloe`, `yolo_world`, `bpu` (Nash hardware), `grounding_dino` |
| Encoder | `clip` (ViT-B/32), `mobileclip` (edge) |
| LLM | `kimi`, `openai`, `claude`, `qwen`, `mock` |
| Planner | `astar` (pure Python), `pct` (C++ ele_planner.so) |

### Backpressure Policies

```python
self.image.set_policy("latest")                   # drop if busy
self.imu.set_policy("throttle", interval=0.02)    # max 50Hz
self.lidar.set_policy("sample", n=5)              # every 5th
self.detections.set_policy("buffer", size=10)      # batch of 10
```

### Transport Decoupling (per-wire)

```python
bp.wire("Safety", "stop_cmd", "Driver", "stop_signal")                          # callback (0 latency)
bp.wire("Perception", "scene_graph", "Planner", "scene_graph", transport="dds")  # decoupled
bp.wire("SLAM", "cloud", "Terrain", "cloud", transport="shm")                   # high bandwidth
```

## Source Directory (`src/`, 10 dirs)

| Directory | Role |
|-----------|------|
| `core/` | Framework: Module, Blueprint, Transport, NativeModule, msgs, spec, tests (599+) |
| `nav/` | Navigation: `core/` (C++ pybind11 algorithms), `rings/` (safety ROS2 nodes), `services/`, NavigationModule, SafetyRingModule |
| `semantic/` | Semantic: `common/` (L0 utils), `perception/` (YOLO+CLIP+SceneGraph+Service), `planner/` (GoalResolver+LLM+Service) |
| `memory/` | Memory layer: spatial (topological, episodic), knowledge (KG, belief), storage (SQLite, timeseries) |
| `drivers/` | Hardware: `thunder/` (ThunderDriver + han_dog_bridge), `sim/` (stub, MuJoCo), `livox_ros_driver2/` |
| `gateway/` | External: GatewayModule (FastAPI HTTP/WS/SSE), MCPServerModule (16 MCP tools) |
| `base_autonomy/` | C++ terrain_analysis + local_planner + AutonomyModule |
| `global_planning/` | C++ PCT_planner + Python pct_adapters + GlobalPlannerModule |
| `slam/` | C++ SLAM: Fast-LIO2 + Point-LIO |
| `reconstruction/` | 3D reconstruction |

## Key Files

| File | Purpose |
|------|---------|
| `main_nav.py` | Entry point — autoconnect, CLI args, all backends configurable |
| `src/core/module.py` | Module base class (idempotent stop, ref cleanup, layer tags) |
| `src/core/stream.py` | Out[T]/In[T] ports (5 backpressure policies, thread-safe) |
| `src/core/blueprint.py` | Blueprint builder (per-wire transport, topo sort, instance modules) |
| `src/core/native_module.py` | C++ subprocess manager (watchdog, restart, SIGTERM/SIGKILL) |
| `src/core/native_factories.py` | Pre-configured factories for 7 C++ nodes |
| `src/core/rerun_module.py` | Rerun 3D visualization (browser at :9090) |
| `src/nav/navigation_module.py` | Unified planner + tracker + mission FSM |
| `src/nav/safety_ring_module.py` | Unified safety reflex + evaluator + dialogue |
| `src/semantic/planner/.../semantic_planner_module.py` | Unified semantic planner |
| `src/semantic/planner/.../llm_module.py` | Pluggable LLM (5 backends) |
| `src/semantic/perception/.../detector_module.py` | Pluggable detector (5 backends) |
| `src/semantic/perception/.../service.py` | PerceptionService (pure algorithm, no ROS2) |
| `src/semantic/planner/.../service.py` | PlannerService (GoalResolution + Frontier + Action) |
| `src/gateway/gateway_module.py` | FastAPI HTTP/WS/SSE (replaces C++ gRPC gateway) |
| `src/gateway/mcp_server.py` | MCP server — 16 tools for AI agent control |
| `src/base_autonomy/autonomy_module.py` | Manages 4 C++ NativeModule nodes as one unit |
| `src/drivers/thunder/han_dog_module.py` | ThunderDriver (gRPC → brainstem CMS) |
| `config/robot_config.yaml` | Robot physical parameters (single source of truth) |

## Build and Test Commands

```bash
# Framework tests (primary, no ROS2 needed)
python -m pytest src/core/tests/ -q                    # 599 tests, ~40s

# ROS2 build (for C++ nodes on S100P)
source /opt/ros/humble/setup.bash
make build                                              # colcon release build

# Planning pipeline tests (no ROS2)
python tests/planning/test_pct_adapter_logic.py

# System launch (ROS2 legacy)
./mapping.sh          # Mapping mode
./planning.sh         # Navigation mode
```

## Critical Files — Do Not Break

- `src/core/module.py` — Module base class (all 10 modules depend on it)
- `src/core/blueprint.py` — Blueprint + autoconnect (system assembly)
- `src/core/stream.py` — In[T]/Out[T] ports (data flow backbone)
- `src/semantic/perception/.../instance_tracker.py` — Scene graph builder
- `src/semantic/planner/.../goal_resolver.py` — Fast-Slow core logic
- `config/robot_config.yaml` — Robot physical parameters

## Semantic Navigation

### Fast-Slow Dual-Process (`goal_resolver.py`)

**Fast Path** (System 1, ~0.17ms): Direct scene graph matching — keyword + spatial reasoning, confidence fusion (label 35%, CLIP 35%, detector 15%, spatial 15%). Target: >70% hit rate, threshold 0.75.

**Slow Path** (System 2, ~2s): LLM reasoning with ESCA selective grounding — filters 200 objects to ~15 objects (92.5% token reduction), then calls LLM. Returns OmniNav hierarchical room hint.

**AdaNav Entropy Trigger**: Shannon entropy over candidate scores. If `score_entropy > 1.5` and `confidence < 0.85`, forced escalation to Slow Path.

**LERa Failure Recovery**: 3-step Look-Explain-Replan on subgoal failure. After 2nd consecutive failure: LLM decides `retry_different_path | expand_search | requery_goal | abort`.

### LLM Configuration

```bash
export MOONSHOT_API_KEY="..."         # Kimi (default, China-direct)
export OPENAI_API_KEY="sk-..."        # OpenAI
export ANTHROPIC_API_KEY="sk-ant-..." # Claude
export DASHSCOPE_API_KEY="sk-..."     # Qwen (China fallback)
```

Or use LLMModule in Blueprint:
```python
LLMModule.blueprint(backend="kimi")   # auto-reads env var
```

## MCP Server (AI Agent Control)

16 tools exposed via JSON-RPC at `http://<robot>:8090/mcp`:

| Category | Tools |
|----------|-------|
| Navigation | `navigate_to`, `navigate_to_object`, `stop`, `get_navigation_status`, `set_mode` |
| Perception | `get_scene_graph`, `detect_objects`, `get_robot_position` |
| Memory | `query_memory`, `list_tagged_locations`, `tag_location` |
| Planning | `send_instruction`, `decompose_task` |
| System | `get_health`, `list_modules`, `get_config` |

```bash
# Connect Claude Code to the robot
claude mcp add --transport http lingtu http://192.168.66.190:8090/mcp
```

## Configuration Files

| File | Purpose |
|------|---------|
| `config/robot_config.yaml` | Robot geometry, speed limits, safety params, driver config |
| `config/semantic_planner.yaml` | LLM backend, goal resolution, exploration, fusion weights |
| `config/semantic_perception.yaml` | Perception module configuration |
| `config/topic_contract.yaml` | Standard ROS2 topic names (all `/nav/` prefixed) |
| `config/layer_contract.yaml` | 7-layer architecture dependency rules |

## ROS2 Topic Contract

All standard topics use `/nav/` prefix. Defined in `config/topic_contract.yaml`. Key topics:

| Topic | Type | Description |
|---|---|---|
| `/nav/odometry` | Odometry | SLAM odometry |
| `/nav/map_cloud` | PointCloud2 | Map point cloud (world frame) |
| `/nav/terrain_map` | PointCloud2 | Base terrain analysis |
| `/nav/global_path` | Path | Global planned path |
| `/nav/local_path` | Path | Local planned path |
| `/nav/way_point` | PointStamped | Waypoint input to local_planner |
| `/nav/cmd_vel` | TwistStamped | Velocity commands |
| `/nav/goal_pose` | PoseStamped | Navigation goal |
| `/nav/planner_status` | String | IDLE/PLANNING/SUCCESS/FAILED/STUCK |
| `/nav/semantic/scene_graph` | String (JSON) | Scene graph |
| `/nav/semantic/instruction` | String | Natural language instruction |

TF frames: `map` -> `odom` -> `body`

## S100P Deployment

- **SSH**: `ssh sunrise@192.168.66.190`
- **Nav code**: `~/data/SLAM/navigation/`
- **Nav deploy**: `/opt/lingtu/nav/`
- **CycloneDDS**: Built from source at `~/cyclonedds/install/` (Unitree approach)
- **Python**: 3.10.12, cyclonedds==0.10.5

## Code Style

- **C++**: Google style (`.clang-format`, 2-space indent, 100 col)
- **Python**: English comments in new code. Chinese comments exist in legacy code.
- **Framework**: All new Modules use `core.Module` base with In[T]/Out[T] type hints

## Known Limitations

- Fast Path uses rule-based matching (not learned policies)
- S100P has no CUDA — Open3D GPU features unavailable, use C++ terrain_analysis instead
- Kimi API key may expire — Slow Path unavailable without valid LLM key
- Framework tests (599) are mock-based — real hardware integration tests need S100P
