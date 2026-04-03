# AGENTS.md — LingTu Agent Guide

This file provides guidance to AI coding agents working with the LingTu codebase.

## 1. Project Overview

LingTu (灵途) is an autonomous navigation system for quadruped robots in outdoor/off-road environments.

- **Platform**: S100P (RDK X5, Nash BPU 128 TOPS, aarch64) | ROS2 Humble | Ubuntu 22.04
- **Languages**: Python (framework + semantic modules), C++ (SLAM/terrain/planner)
- **Architecture**: Module-First — Module is the only runtime unit, Blueprint is the only orchestration
- **Guideline**: `docs/MODULE_FIRST_GUIDELINE.md` — 8 rules for how code should be structured

## 2. Quick Start

```bash
# Framework tests (no ROS2 needed, runs on any machine)
python -m pytest src/core/tests/ -q       # 625 tests

# CLI with interactive REPL (profile-based)
python lingtu.py                          # interactive profile selector
python lingtu.py stub                     # no hardware, framework testing
python lingtu.py sim                      # MuJoCo simulation (full stack)
python lingtu.py dev                      # semantic pipeline, no C++ nodes
python lingtu.py s100p                    # real S100P robot (BPU + Kimi)
python lingtu.py explore                  # exploration, no pre-built map
python lingtu.py map                      # mapping mode (SLAM + save)
python lingtu.py --list                   # list all profiles

# Override any profile flag
python lingtu.py s100p --llm mock         # real robot but mock LLM
python lingtu.py s100p --daemon           # background daemon (S100P)
python lingtu.py stop                     # stop running daemon

# Composable factory API (each line = one functional stack)
from core.blueprint import autoconnect
from core.blueprints.stacks import *
system = autoconnect(
    driver("thunder", host="192.168.66.190"),
    slam("localizer"),
    maps(),
    perception("bpu"),
    memory(),
    planner("kimi"),
    navigation("astar"),
    safety(),
    gateway(5050),
).build()
system.start()
```

## 3. Architecture — Dual-Layer System

LingTu is a **hybrid repo** with two layers that run together:

1. **Python Module-First Layer** — High-level orchestration, semantic intelligence, and the entire framework (`core/`, `nav/`, `semantic/`, `memory/`, `gateway/`, `drivers/`). Started via `python lingtu.py`.
2. **C++/ROS2 Layer** — Low-level SLAM, terrain analysis, local planning, and path following (`src/slam/`, `src/base_autonomy/`, `src/global_planning/`). Managed as subprocesses by Python `NativeModule` or run via `ros2 launch`.

### Layer Hierarchy

```
L0  Safety       — SafetyRingModule + GeofenceManagerModule
L1  Hardware     — Driver + CameraBridge + SLAM (managed/bridge/localizer)
L2  Maps         — OccupancyGrid + ESDF + ElevationMap + Terrain + LocalPlanner + PathFollower
L3  Perception   — Detector + Encoder + Reconstruction + SemanticMapper + Episodic + Tagged + VectorMemory
L4  Decision     — SemanticPlanner + LLM + VisualServo (bbox tracking + person following)
L5  Planning     — NavigationModule (A*/PCT + WaypointTracker + mission FSM + goal safety)
L6  Interface    — Gateway + MCP + Teleop
```

High layers depend on low layers only. L5→L2 (waypoint→PathFollower) is command dispatch, not dependency.

### Module-First Principles

1. **Module is the only runtime unit** — no separate ROS2 Node + Module pairs
2. **Blueprint is the only orchestration** — no `ros2 launch` for Python modules
3. **Communication via In/Out + Transport** — not rclpy pub/sub
4. **C++ nodes managed by NativeModule** — started as watchdog-supervised subprocesses
5. **Pluggable backends via Registry** — `@register("category", "name")`, zero if/else
6. **Message types from `core.msgs`** — not `sensor_msgs.msg` or `nav_msgs.msg`
7. **Configuration via constructor parameters** — not ROS2 `declare_parameter`

### Composable Stack Factories (`src/core/blueprints/stacks/`)

| Factory | Returns | Modules |
|---------|---------|---------|
| `driver(robot)` | Blueprint | Driver + CameraBridge (auto-detect) |
| `slam(profile)` | Blueprint | SLAMModule or SlamBridgeModule |
| `maps()` | Blueprint | OccupancyGrid + ESDF + ElevationMap |
| `perception(det, enc)` | Blueprint | Detector + Encoder + Reconstruction |
| `memory(save_dir)` | Blueprint | SemanticMapper + Episodic + Tagged + VectorMemory |
| `planner(llm)` | Blueprint | SemanticPlanner + LLM + VisualServo |
| `navigation(planner)` | Blueprint | NavigationModule + Autonomy chain |
| `safety()` | Blueprint | SafetyRing + Geofence |
| `gateway(port)` | Blueprint | Gateway + MCP + Teleop |

### Pluggable Backends (via Registry)

| Module | Backends |
|--------|----------|
| Driver | `thunder` (gRPC→brainstem), `stub` (testing), `sim_mujoco`, `sim_ros2` |
| SLAM | `fastlio2`, `pointlio`, `localizer` (ICP on pre-built map), `bridge` (external ROS2) |
| Detector | `yoloe`, `yolo_world`, `bpu` (Nash hardware), `grounding_dino` |
| Encoder | `clip` (ViT-B/32), `mobileclip` (edge) |
| LLM | `kimi`, `openai`, `claude`, `qwen`, `mock` |
| Planner | `astar` (pure Python), `pct` (C++ ele_planner.so) |
| PathFollower | `nav_core` (C++ nanobind), `pure_pursuit`, `pid` |

### Profiles

| Profile | Driver | SLAM | Native | Semantic | Use Case |
|---------|--------|------|--------|----------|----------|
| `stub` | stub | none | no | no | Framework testing |
| `dev` | stub | none | no | yes | Semantic pipeline dev |
| `sim` | sim_ros2 | bridge | yes | yes | MuJoCo full simulation |
| `map` | sim_ros2 | fastlio2 | no | no | Build map with SLAM |
| `s100p` | sim_ros2 | localizer | no | yes | Real robot navigation |
| `explore` | thunder | fastlio2 | yes | yes | Exploration (no map) |

## 4. Repository Layout

```
lingtu/
├── lingtu.py              # Primary entry (Module-First CLI + REPL)
├── main_nav.py            # Alias → same as lingtu.py
├── lingtu_cli.py          # pip console script `lingtu`
├── cli/                   # CLI implementation (profiles, REPL, daemon)
├── src/                   # Python packages + ROS2 packages (colcon)
│   ├── core/              # Framework: Module, Blueprint, Transport, Registry, stacks, tests
│   ├── nav/               # NavigationModule, SafetyRing, GlobalPlanner, OccupancyGrid, ESDF
│   ├── semantic/          # perception/ (Detector+Encoder), planner/ (GoalResolver+LLM+VisualServo)
│   ├── memory/            # SemanticMapper, EpisodicMemory, Tagged, VectorMemory, KG
│   ├── drivers/           # thunder/ (gRPC), sim/ (stub, MuJoCo, ROS2), TeleopModule
│   ├── gateway/           # GatewayModule (FastAPI), MCPServerModule
│   ├── base_autonomy/     # C++ terrain + local planner + path follower (nanobind)
│   ├── global_planning/   # C++ PCT planner + Python adapters
│   ├── slam/              # C++ SLAM (Fast-LIO2, Point-LIO, PGO, Localizer)
│   └── reconstruction/    # 3D reconstruction
├── config/                # YAML / DDS / robot params
├── launch/                # ROS2 launch (legacy / bridge stacks)
├── sim/                   # MuJoCo simulation assets + scripts
├── tests/                 # Integration & planning tests
├── tools/                 # Robot-side helpers (dashboard, BPU export)
├── scripts/               # ws/, deploy/, ota/, proto/ helpers
├── docs/                  # Architecture, guides, ADRs
├── Makefile               # colcon build, test, docker targets
├── pyproject.toml         # Root package: lingtu (setuptools)
├── requirements.txt       # S100P runtime deps
└── docker-compose.yml     # Container orchestration
```

## 5. Source Directory Detail (`src/`)

| Directory | Role |
|-----------|------|
| `core/` | Framework: Module, Blueprint, Transport, NativeModule, Registry, stacks/, utils, msgs, tests (625) |
| `core/msgs/` | Message types: geometry, nav, sensor, semantic (replaces ROS2 msg types) |
| `core/transport/` | Backends: Local (callback), DDS (cyclonedds), SHM, Adapter |
| `core/blueprints/` | System blueprints: `full_stack.py` calls 9 stack factories |
| `core/spec/` | Protocol interfaces (16 specs) |
| `core/tests/` | 625 framework tests (pytest, no ROS2 dependency) |
| `nav/` | NavigationModule, SafetyRing, GlobalPlannerService, WaypointTracker, OccupancyGrid, ESDF, ElevationMap |
| `semantic/` | perception/ (Detector+Encoder+SceneGraph), planner/ (SemanticPlanner+LLM+VisualServo+AgentLoop), reconstruction/ |
| `memory/` | SemanticMapper, EpisodicMemory, TaggedLocations, VectorMemory, RoomObjectKG, TopologySemGraph |
| `drivers/` | thunder/ (ThunderDriver + CameraBridge), sim/ (stub, MuJoCo, ROS2), TeleopModule |
| `gateway/` | GatewayModule (FastAPI HTTP/WS/SSE), MCPServerModule (MCP tools) |
| `base_autonomy/` | TerrainModule + LocalPlannerModule + PathFollowerModule (C++ nanobind backends) |
| `slam/` | SLAMModule (Fast-LIO2/Point-LIO/Localizer), SlamBridgeModule, C++ SLAM nodes |
| `global_planning/` | PCT_planner (C++ ele_planner.so) + _AStarBackend / _PCTBackend (via Registry) |

## 6. Key Files

| File | Purpose |
|------|---------|
| `lingtu.py` | CLI entry point — profiles + REPL |
| `src/core/blueprints/full_stack.py` | Top-level blueprint (~190 lines, 9 stack factories + cross-stack wires) |
| `src/core/blueprints/stacks/` | 9 composable factory functions |
| `src/core/module.py` | Module base class (In/Out, @skill, @rpc, layer tags) |
| `src/core/stream.py` | Out[T]/In[T] ports (5 backpressure policies, thread-safe) |
| `src/core/blueprint.py` | Blueprint builder (autoconnect, per-wire transport, auto_wire) |
| `src/core/registry.py` | Plugin registry (@register decorator) |
| `src/nav/navigation_module.py` | Global planner + WaypointTracker + mission FSM |
| `src/nav/global_planner_service.py` | A*/PCT backend + _find_safe_goal BFS |
| `src/nav/safety_ring_module.py` | Safety reflex + evaluator + dialogue |
| `src/semantic/planner/.../semantic_planner_module.py` | 5-level fallback + multi-turn AgentLoop |
| `src/semantic/planner/.../goal_resolver.py` | Fast-Slow dual-process + KG hot-reload |
| `src/semantic/planner/.../visual_servo_module.py` | BBoxNavigator + PersonTracker (dual channel) |
| `src/semantic/planner/.../agent_loop.py` | Multi-turn LLM tool calling (7 tools) |
| `src/memory/modules/semantic_mapper_module.py` | SceneGraph → RoomObjectKG + TopologySemGraph |
| `src/memory/modules/vector_memory_module.py` | CLIP + ChromaDB vector search |
| `src/drivers/teleop_module.py` | WebSocket joystick + camera stream |
| `src/slam/slam_module.py` | SLAM managed mode (fastlio2/pointlio/localizer) |
| `config/robot_config.yaml` | Robot physical parameters (single source of truth) |

## 7. Build and Test Commands

```bash
# Framework tests (primary, no ROS2 needed)
python -m pytest src/core/tests/ -q                    # 625 tests, ~50s

# ROS2 build (for C++ nodes on S100P only)
source /opt/ros/humble/setup.bash
make build                                              # colcon release build
```

## 8. Critical Files — Do Not Break

- `src/core/module.py` — Module base class (all modules depend on it)
- `src/core/blueprint.py` — Blueprint + autoconnect (system assembly)
- `src/core/stream.py` — In[T]/Out[T] ports (data flow backbone)
- `src/core/registry.py` — Plugin registry (all backends depend on it)
- `src/core/utils/` — Cross-layer utilities (18+ files import from here)
- `src/semantic/perception/.../instance_tracker.py` — Scene graph builder
- `src/semantic/planner/.../goal_resolver.py` — 5-level resolution chain
- `config/robot_config.yaml` — Robot physical parameters

## 9. Module Dependency Rules

```
All Modules ──→ core/ (Module, In/Out, Registry, utils, msgs)
                 ↑ only legal dependency direction

nav/          does NOT import semantic/, drivers/, gateway/
semantic/     does NOT import nav/, drivers/, gateway/
drivers/      does NOT import nav/, semantic/ (lazy import in blueprints only)
gateway/      does NOT import nav/, semantic/, drivers/
```

Planner backends resolved via `core.registry.get("planner_backend", name)`, not direct import.

## 10. Backpressure Policies

```python
self.image.set_policy("latest")                   # drop if busy
self.imu.set_policy("throttle", interval=0.02)    # max 50Hz
self.lidar.set_policy("sample", n=5)              # every 5th
self.detections.set_policy("buffer", size=10)      # batch of 10
```

## 11. Transport Decoupling (per-wire)

```python
bp.wire("Safety", "stop_cmd", "Driver", "stop_signal")                          # callback (0 latency)
bp.wire("Perception", "scene_graph", "Planner", "scene_graph", transport="dds")  # decoupled
bp.wire("SLAM", "cloud", "Terrain", "cloud", transport="shm")                   # high bandwidth
```

## 12. Explicit Wires (Cross-Stack, in `full_stack.py`)

```python
# Safety → all actuators
bp.wire("SafetyRingModule", "stop_cmd", driver_name, "stop_signal")
bp.wire("SafetyRingModule", "stop_cmd", "NavigationModule", "stop_signal")

# SLAM odometry priority for NavigationModule
bp.wire(slam_module_name, "odometry", "NavigationModule", "odometry")

# Autonomy chain (when enable_native=True)
bp.wire("NavigationModule", "waypoint", "LocalPlannerModule", "waypoint")
bp.wire("TerrainModule", "terrain_map", "LocalPlannerModule", "terrain_map")
bp.wire("LocalPlannerModule", "local_path", "PathFollowerModule", "local_path")
bp.wire("PathFollowerModule", "cmd_vel", driver_name, "cmd_vel")

# Visual servo dual channel
bp.wire("SemanticPlannerModule", "servo_target", "VisualServoModule", "servo_target")
bp.wire("VisualServoModule", "goal_pose", "NavigationModule", "goal_pose")
bp.wire("VisualServoModule", "cmd_vel", driver_name, "cmd_vel")

# Teleop
bp.wire("TeleopModule", "cmd_vel", driver_name, "cmd_vel")
bp.wire("TeleopModule", "nav_stop", "NavigationModule", "stop_signal")
```

## 13. Semantic Navigation

### 5-Level Goal Resolution Chain

```
Instruction: "go to where I left the backpack"
  ↓
1. Tag Lookup     — exact/fuzzy match in TaggedLocationStore     → goal_pose
2. Fast Path      — scene graph keyword + CLIP matching (<200ms) → goal_pose
3. Vector Memory  — CLIP embedding search in ChromaDB            → goal_pose
4. Frontier       — topology graph information gain exploration   → goal_pose
5. Visual Servo   — VLM bbox detection + PD tracking             → goal_pose/cmd_vel
```

### Fast-Slow Dual-Process (`goal_resolver.py`)

**Fast Path** (System 1, ~0.17ms): Direct scene graph matching — keyword + spatial reasoning, confidence fusion (label 35%, CLIP 35%, detector 15%, spatial 15%). Target: >70% hit rate, threshold 0.75.

**Slow Path** (System 2, ~2s): LLM reasoning with ESCA selective grounding — filters 200 objects to ~15 objects (92.5% token reduction), then calls LLM. Returns OmniNav hierarchical room hint.

**AdaNav Entropy Trigger**: Shannon entropy over candidate scores. If `score_entropy > 1.5` and `confidence < 0.85`, forced escalation to Slow Path.

**LERa Failure Recovery**: 3-step Look-Explain-Replan on subgoal failure. After 2nd consecutive failure: LLM decides `retry_different_path | expand_search | requery_goal | abort`.

### Visual Servo (`visual_servo_module.py`)

Two output channels based on distance:
- Far (> 3m): `goal_pose → NavigationModule → planning stack`
- Near (< 3m): `cmd_vel → Driver directly (PD servo, bypasses planner)`

Components: BBoxNavigator (bbox+depth→3D→PD), PersonTracker (VLM select+CLIP Re-ID), vlm_bbox_query (open-vocab detection).

### Multi-Turn Agent Loop (`agent_loop.py`)

`agent_instruction` port triggers observe→think→act cycle with 7 LLM tools:
`navigate_to`, `navigate_to_object`, `detect_object`, `query_memory`, `tag_location`, `say`, `done`.
Max 10 steps / 120s timeout. Supports OpenAI function-calling + text JSON fallback.

### LLM Configuration

```bash
export MOONSHOT_API_KEY="..."         # Kimi (default, China-direct)
export OPENAI_API_KEY="sk-..."        # OpenAI
export ANTHROPIC_API_KEY="sk-ant-..." # Claude
export DASHSCOPE_API_KEY="sk-..."     # Qwen (China fallback)
```

## 14. SLAM / Localization

| Mode | slam_profile | Backend | Use Case |
|------|-------------|---------|----------|
| Mapping | `fastlio2` | SLAMModule → C++ Fast-LIO2 | First visit, build map |
| Localization | `localizer` | SLAMModule → Fast-LIO2 + ICP Localizer | Navigate with pre-built map |
| Bridge | `bridge` | SlamBridgeModule → ROS2 subscriber | External SLAM (systemd) |
| None | `none` | — | stub/dev mode |

Localizer requires Fast-LIO2 companion (provides `/cloud_registered` + `/Odometry`).
SLAM odometry is explicitly wired to NavigationModule (priority over driver dead-reckoning).

## 15. C++/ROS2 Layer — Coordinate Frames

### Frame Hierarchy

```
map (global map frame) — fixed world reference
 └── odom (odometry frame) — published by PGO/Localizer ← perception + path adaptation
     └── body (robot body frame) — Fast-LIO2 output ← local planning + control
         └── lidar (LiDAR sensor frame) — raw sensor
```

- **Perception** (terrain_analysis) works in **odom** frame
- **Local planning** (local_planner) works in **body** frame
- **Global planning** (pct_planner) works in **map** frame

### TF Transforms

```
map → odom: Published by PGO or Localizer (10-20Hz)
odom → body: Published by Fast-LIO2 (/Odometry, 100Hz)
body → lidar: Static transform (extrinsics from lio.yaml)
```

## 16. C++/ROS2 Layer — Topics and Services

### Core Topics

| Topic | Type | Publisher | Subscriber | Frame |
|-------|------|----------|------------|-------|
| `/livox/lidar` | CustomMsg | livox_ros_driver2 | fastlio2 | lidar |
| `/livox/imu` | Imu | livox_ros_driver2 | fastlio2 | body |
| `/cloud_registered` | PointCloud2 | fastlio2 | PGO, Localizer | body |
| `/cloud_map` | PointCloud2 | fastlio2 | terrain_analysis | odom |
| `/Odometry` | Odometry | fastlio2 | all modules | odom→body |
| `/terrain_map` | PointCloud2 | terrain_analysis | local_planner | odom |
| `/way_point` | PointStamped | TaskManager (sole publisher) | local_planner | odom |
| `/planner_waypoint` | PointStamped | pct_path_adapter | TaskManager | odom |
| `/pct_path` | Path | pct_planner | pct_path_adapter | map |
| `/path` | Path | local_planner | pathFollower | body |
| `/cmd_vel` | Twist | pathFollower | robot_driver | body |
| `/slow_down` | Int8 | local_planner | pathFollower | — |
| `/stop` | Int8 | local_planner, SafetyGate, TaskManager | pathFollower | — |

### Services

| Service | Type | Provider | Purpose |
|---------|------|----------|---------|
| `/relocalize` | Relocalize | Localizer | Load map and relocalize |
| `/relocalize_check` | IsValid | Localizer | Check relocalization status |
| `/pgo/save_maps` | SaveMaps | PGO | Save optimized maps |
| `/save_map` | SaveMaps | fastlio2 | Save current map |

## 17. Task Orchestration (gRPC Remote Monitoring)

The C++ `remote_monitoring` layer provides gRPC services for the Flutter client.

### End-to-End Flow

```
Flutter App ──gRPC──→ ControlService (guard: Lease + AUTONOMOUS)
  → TaskManager (sole /way_point publisher)
    → map→odom tf2 transform
    → sequential waypoint dispatch
    → /way_point (odom) → local_planner → pathFollower → robot_driver
```

### gRPC RPCs (proto: `src/robot_proto/proto/control.proto`)

| RPC | Purpose |
|-----|---------|
| `StartTask` | Start navigation/mapping/patrol (needs Lease + AUTONOMOUS) |
| `CancelTask` | Cancel current task |
| `PauseTask` / `ResumeTask` | Suspend/resume |
| `GetTaskStatus` | Query progress |
| `GetActiveWaypoints` | Query active waypoints (source/list/progress) |
| `ClearWaypoints` | Clear waypoints and stop (needs Lease) |
| `SetMode` | Switch mode (IDLE/MANUAL/TELEOP/AUTONOMOUS/MAPPING) |
| `EmergencyStop` | E-stop (suspends TaskManager) |
| `StreamTeleop` | Bidirectional teleop stream |
| `AcquireLease` | Acquire operation lease |

### Waypoint Coordinate Convention

All waypoints published on `/way_point` are in **odom** frame. App waypoints (map frame) are transformed by TaskManager via tf2 `map→odom`. The `local_planner` reads `/way_point` x,y directly as odom coordinates with no further transformation.

## 18. MCP Server (AI Agent Control)

Auto-discovered @skill methods exposed via JSON-RPC at `http://<robot>:8090/mcp`.

| Category | Tools |
|----------|-------|
| Navigation | `navigate_to`, `navigate_to_object`, `stop`, `get_navigation_status`, `set_mode` |
| Perception | `get_scene_graph`, `detect_objects`, `get_robot_position` |
| Memory | `query_memory`, `query_location` (vector), `list_tagged_locations`, `tag_location` |
| Semantic Map | `get_room_summary`, `query_room_for_object`, `get_exploration_target` |
| Visual Servo | `find_object`, `follow_person`, `stop_servo`, `get_servo_status` |
| Planning | `send_instruction`, `decompose_task` |
| System | `get_health`, `list_modules`, `get_config` |

```bash
claude mcp add --transport http lingtu http://192.168.66.190:8090/mcp
```

## 19. Teleop (Remote Control)

WebSocket joystick at `ws://<robot>:5060/teleop`:
- Phone/browser sends `{"type": "joy", "lx": 0.5, "ly": 0.0, "az": -0.3}`
- Robot streams JPEG camera frames back
- 3s idle → auto-releases to autonomous navigation
- cmd_vel priority: Teleop > VisualServo > PathFollower

## 20. REPL Commands

```
Navigation:  go/navigate <target> | stop | cancel | status
Map:         map list | save <name> | use <name> | build <name> | delete <name>
Semantic:    smap status | rooms | save | load <dir> | query <text>
Vector:      vmem query <text> | vmem stats
Agent:       agent <multi-step instruction>
Teleop:      teleop status | teleop release
Monitor:     health | watch <port> | module <name> | connections | log | config
```

## 21. Configuration Files

| File | Purpose |
|------|---------|
| `config/robot_config.yaml` | Robot physical parameters (single source of truth) |
| `config/semantic_planner.yaml` | Semantic planner configuration |
| `config/semantic_perception.yaml` | Perception pipeline configuration |
| `config/semantic_exploration.yaml` | Exploration parameters |
| `config/topic_contract.yaml` | Port name/type contracts |
| `config/layer_contract.yaml` | Layer dependency contracts |
| `config/qos_profiles.yaml` | QoS/backpressure profiles |
| `config/cyclonedds.xml` | CycloneDDS configuration |
| `src/slam/fastlio2/config/lio.yaml` | Fast-LIO2 parameters |
| `src/slam/pgo/config/pgo.yaml` | PGO parameters |
| `src/slam/localizer/config/localizer.yaml` | Localizer parameters |

## 22. S100P Deployment

- **SSH**: `ssh sunrise@192.168.66.190`
- **Nav code**: `~/data/SLAM/navigation/`
- **Nav deploy**: `/opt/lingtu/nav/`
- **CycloneDDS**: Built from source at `~/cyclonedds/install/`
- **Python**: 3.10.12, cyclonedds==0.10.5

## 23. Code Style

- **C++**: Google style (`.clang-format`, 2-space indent, 100 col)
- **Python**: English comments in new code. Chinese comments exist in legacy code.
- **Framework**: All Modules use `core.Module` base with In[T]/Out[T] type hints
- **No ROS2 in Modules**: rclpy only in Bridge modules and C++ NativeModule launchers

## 24. Known Limitations

- Fast Path uses rule-based matching (not learned policies)
- S100P has no CUDA — Open3D GPU features unavailable, use C++ terrain_analysis instead
- Kimi API key may expire — Slow Path unavailable without valid LLM key
- ChromaDB optional — VectorMemoryModule falls back to numpy brute-force search
- Framework tests (625) are mock-based — real hardware integration tests need S100P

## 25. Related Documentation

| Document | Purpose |
|----------|---------|
| `CLAUDE.md` | Claude Code-specific guidance (mirrors this file) |
| `docs/MODULE_FIRST_GUIDELINE.md` | 8 rules for Module-First architecture |
| `docs/REPO_LAYOUT.md` | Top-level directory map |
| `docs/QUICKSTART.md` | Getting started guide |
| `docs/TUNING.md` | Parameter tuning guide |
| `src/base_autonomy/TERRAIN_ANALYSIS_EXPLAINED.md` | Terrain analysis deep dive |
| `src/remote_monitoring/README.md` | gRPC Gateway documentation |
| `src/robot_proto/proto/control.proto` | Task control protocol definition |
