# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

LingTu (灵途) is an autonomous navigation system for quadruped robots in outdoor/off-road environments.

- **Platform**: S100P (RDK X5, Nash BPU 128 TOPS, aarch64) | ROS2 Humble | Ubuntu 22.04
- **Languages**: Python (framework + semantic modules), C++ (SLAM/terrain/planner)
- **Architecture**: Module-First — Module is the only runtime unit, Blueprint is the only orchestration
- **Guideline**: `docs/MODULE_FIRST_GUIDELINE.md` — 8 rules for how code should be structured

## Quick Start

```bash
# Framework tests (no ROS2 needed, runs on any machine)
python -m pytest src/core/tests/ -q       # 580 tests

# CLI with interactive REPL (profile-based, recommended)
python main_nav.py                        # interactive profile selector
python main_nav.py stub                   # no hardware, framework testing
python main_nav.py sim                    # MuJoCo kinematic simulation
python main_nav.py dev                    # semantic pipeline, no C++ nodes
python main_nav.py s100p                  # real S100P robot (BPU + Kimi)
python main_nav.py explore                # exploration, no pre-built map
python main_nav.py --list                 # list all profiles

# Override any profile flag
python main_nav.py s100p --llm mock       # real robot but mock LLM
python main_nav.py s100p --daemon         # background daemon (S100P)
python main_nav.py stop                   # stop running daemon

# Or use Blueprint directly in any script
from core import autoconnect
from drivers.thunder.han_dog_module import ThunderDriver
from nav.navigation_module import NavigationModule
from nav.safety_ring_module import SafetyRingModule
autoconnect(
    ThunderDriver.blueprint(dog_host="192.168.66.190"),
    NavigationModule.blueprint(planner="astar"),
    SafetyRingModule.blueprint(),
).build().start()
```

## Architecture — Module-First

Module is the only runtime unit. No ROS2 Node wrappers. Blueprint composes Modules.

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

### Pluggable Backends (via Registry)

| Module | Backends |
|--------|----------|
| Driver | `thunder` (gRPC→brainstem), `stub` (testing), `sim_mujoco` |
| Detector | `yoloe`, `yolo_world`, `bpu` (Nash hardware), `grounding_dino` |
| Encoder | `clip` (ViT-B/32), `mobileclip` (edge) |
| LLM | `kimi`, `openai`, `claude`, `qwen`, `mock` |
| Planner | `astar` (pure Python), `pct` (C++ ele_planner.so) |

All backends registered via `@register("category", "name")` in `core.registry`. Zero if/else.

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

## Source Directory (`src/`, 9 dirs, 308 .py files)

| Directory | .py | Role |
|-----------|-----|------|
| `core/` | 68 | Framework: Module, Blueprint, Transport, NativeModule, Registry, utils, msgs, tests (580) |
| `nav/` | 10 | NavigationModule, SafetyRingModule, services (map/patrol/geofence/scheduler Modules) |
| `semantic/` | 171 | perception/ (Detector+Encoder+Service), planner/ (SemanticPlanner+LLM+Service), reconstruction/, common/ |
| `memory/` | 27 | Spatial (topological, episodic), knowledge (KG, belief), storage (SQLite, timeseries), Modules |
| `drivers/` | 13 | thunder/ (ThunderDriver + han_dog_bridge), sim/ (stub, MuJoCo) |
| `gateway/` | 5 | GatewayModule (FastAPI HTTP/WS/SSE), MCPServerModule (16 MCP tools) |
| `global_planning/` | 6+C++ | PCT_planner (C++ ele_planner.so) + pct_adapters (GlobalPlanner/PathAdapter/MissionArc Module) |
| `base_autonomy/` | 3 | AutonomyModule — manages 4 C++ NativeModule nodes as one unit |
| `slam/` | 5 | C++ SLAM: Fast-LIO2 + Point-LIO (via NativeModule) |


## Key Files

| File | Purpose |
|------|---------|
| `main_nav.py` | CLI entry point — autoconnect, all backends configurable |
| `docs/MODULE_FIRST_GUIDELINE.md` | 8 rules for Module-First architecture |
| `src/core/module.py` | Module base class (idempotent stop, ref cleanup, layer tags) |
| `src/core/stream.py` | Out[T]/In[T] ports (5 backpressure policies, thread-safe) |
| `src/core/blueprint.py` | Blueprint builder (per-wire transport, topo sort, autoconnect) |
| `src/core/registry.py` | Plugin registry (@register decorator, get/list/auto_select) |
| `src/core/utils/` | Cross-layer utilities: sanitize, robustness, validation |
| `src/core/native_module.py` | C++ subprocess manager (watchdog, restart, SIGTERM/SIGKILL) |
| `src/nav/navigation_module.py` | Unified planner + tracker + mission FSM |
| `src/nav/safety_ring_module.py` | Unified safety reflex + evaluator + dialogue |
| `src/semantic/planner/.../semantic_planner_module.py` | Unified semantic planner |
| `src/semantic/planner/.../llm_module.py` | Pluggable LLM (5 backends) |
| `src/semantic/perception/.../detector_module.py` | Pluggable detector (5 backends) |
| `src/semantic/perception/.../service.py` | PerceptionService (pure algorithm) |
| `src/semantic/planner/.../service.py` | PlannerService (GoalResolution + Frontier + Action) |
| `src/semantic/reconstruction/` | 3D RGB-D reconstruction + semantic labeling + PLY export |
| `src/gateway/gateway_module.py` | FastAPI HTTP/WS/SSE gateway |
| `src/gateway/mcp_server.py` | MCP server — 16 tools for AI agent control |
| `src/drivers/thunder/han_dog_module.py` | ThunderDriver (gRPC → brainstem CMS) |
| `config/robot_config.yaml` | Robot physical parameters (single source of truth) |

## Build and Test Commands

```bash
# Framework tests (primary, no ROS2 needed)
python -m pytest src/core/tests/ -q                    # 580 tests, ~27s

# ROS2 build (for C++ nodes on S100P only)
source /opt/ros/humble/setup.bash
make build                                              # colcon release build
```

## Critical Files — Do Not Break

- `src/core/module.py` — Module base class (all modules depend on it)
- `src/core/blueprint.py` — Blueprint + autoconnect (system assembly)
- `src/core/stream.py` — In[T]/Out[T] ports (data flow backbone)
- `src/core/registry.py` — Plugin registry (all backends depend on it)
- `src/core/utils/` — Cross-layer utilities (18+ files import from here)
- `src/semantic/perception/.../instance_tracker.py` — Scene graph builder
- `src/semantic/planner/.../goal_resolver.py` — Fast-Slow core logic
- `config/robot_config.yaml` — Robot physical parameters

## Module Dependency Rules

```
All Modules ──→ core/ (Module, In/Out, Registry, utils, msgs)
                 ↑ only legal dependency direction

nav/          does NOT import semantic/, drivers/, gateway/
semantic/     does NOT import nav/, drivers/, gateway/
drivers/      does NOT import nav/, semantic/ (lazy import in blueprints only)
gateway/      does NOT import nav/, semantic/, drivers/
```

Planner backends resolved via `core.registry.get("planner_backend", name)`, not direct import.

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

## S100P Deployment

- **SSH**: `ssh sunrise@192.168.66.190`
- **Nav code**: `~/data/SLAM/navigation/`
- **Nav deploy**: `/opt/lingtu/nav/`
- **CycloneDDS**: Built from source at `~/cyclonedds/install/` (Unitree approach)
- **Python**: 3.10.12, cyclonedds==0.10.5

## Code Style

- **C++**: Google style (`.clang-format`, 2-space indent, 100 col)
- **Python**: English comments in new code. Chinese comments exist in legacy code.
- **Framework**: All Modules use `core.Module` base with In[T]/Out[T] type hints
- **No ROS2 in Modules**: rclpy only in legacy/ dirs and C++ NativeModule launchers

## Known Limitations

- Fast Path uses rule-based matching (not learned policies)
- S100P has no CUDA — Open3D GPU features unavailable, use C++ terrain_analysis instead
- Kimi API key may expire — Slow Path unavailable without valid LLM key
- Framework tests (580) are mock-based — real hardware integration tests need S100P
