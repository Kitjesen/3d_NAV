# AGENTS.md 鈥?LingTu Agent Guide

This file provides guidance to AI coding agents working with the LingTu codebase.

## 1. Project Overview

LingTu (鐏甸€? is an autonomous navigation system for quadruped robots in outdoor/off-road environments.

- **Platform**: S100P (RDK X5, Nash BPU 128 TOPS, aarch64) | ROS2 Humble | Ubuntu 22.04
- **Languages**: Python (framework + semantic modules), C++ (SLAM/terrain/planner)
- **Architecture**: Module-First 鈥?Module is the only runtime unit, Blueprint is the only orchestration
- **Guideline**: `docs/MODULE_FIRST_GUIDELINE.md` 鈥?8 rules for how code should be structured

## 2. Quick Start

```bash
# Framework tests (no ROS2 needed, runs on any machine)
python -m pytest src/core/tests/ -q       # ~1000+ framework tests

# CLI with interactive REPL (profile-based)
python lingtu.py                          # interactive profile selector
python lingtu.py stub                     # no hardware, framework testing
python lingtu.py sim                      # MuJoCo simulation (full stack)
python lingtu.py sim_nav                  # pure-Python sim, no ROS2 / C++
python lingtu.py dev                      # semantic pipeline, no C++ nodes
python lingtu.py nav                      # real S100P navigation (BPU + Qwen + PCT)
python lingtu.py explore                  # wavefront frontier exploration
python lingtu.py tare_explore             # CMU TARE hierarchical exploration
python lingtu.py map                      # mapping mode (SLAM + save)
python lingtu.py --list                   # list all profiles

# Override any profile flag
python lingtu.py nav --llm mock           # real robot but mock LLM
python lingtu.py nav --daemon             # background daemon
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

## 3. Architecture 鈥?Dual-Layer System

LingTu is a **hybrid repo** with two layers that run together:

1. **Python Module-First Layer** 鈥?High-level orchestration, semantic intelligence, and the entire framework (`core/`, `nav/`, `semantic/`, `memory/`, `gateway/`, `drivers/`). Started via `python lingtu.py`.
2. **C++/ROS2 Layer** 鈥?Low-level SLAM, terrain analysis, local planning, and path following (`src/slam/`, `src/base_autonomy/`, `src/global_planning/`). Managed as subprocesses by Python `NativeModule` or run via `ros2 launch`.

### Layer Hierarchy

```
L0  Safety       鈥?SafetyRingModule + GeofenceManagerModule
L1  Hardware     鈥?Driver + CameraBridge + SLAM (managed/bridge/localizer)
L2  Maps         鈥?OccupancyGrid + ESDF + ElevationMap + Terrain + LocalPlanner + PathFollower
L3  Perception   鈥?Detector + Encoder + Reconstruction + SemanticMapper + Episodic + Tagged + VectorMemory
L4  Decision     鈥?SemanticPlanner + LLM + VisualServo (bbox tracking + person following)
L5  Planning     鈥?NavigationModule (A*/PCT + WaypointTracker + mission FSM + goal safety)
L6  Interface    鈥?Gateway + MCP + Teleop
```

High layers depend on low layers only. L5鈫扡2 (waypoint鈫扨athFollower) is command dispatch, not dependency.

### Module-First Principles

1. **Module is the only runtime unit** 鈥?no separate ROS2 Node + Module pairs
2. **Blueprint is the only orchestration** 鈥?no `ros2 launch` for Python modules
3. **Communication via In/Out + Transport** 鈥?not rclpy pub/sub
4. **C++ nodes managed by NativeModule** 鈥?started as watchdog-supervised subprocesses
5. **Pluggable backends via Registry** 鈥?`@register("category", "name")`, zero if/else
6. **Message types from `core.msgs`** 鈥?not `sensor_msgs.msg` or `nav_msgs.msg`
7. **Configuration via constructor parameters** 鈥?not ROS2 `declare_parameter`

### Composable Stack Factories (`src/core/blueprints/stacks/`)


| Factory                | Returns   | Modules                                           |
| ---------------------- | --------- | ------------------------------------------------- |
| `driver(robot)`        | Blueprint | Driver + CameraBridge (auto-detect)               |
| `slam(profile)`        | Blueprint | SLAMModule or SlamBridgeModule                    |
| `maps()`               | Blueprint | OccupancyGrid + ESDF + ElevationMap               |
| `perception(det, enc)` | Blueprint | Detector + Encoder + Reconstruction               |
| `memory(save_dir)`     | Blueprint | SemanticMapper + Episodic + Tagged + VectorMemory |
| `planner(llm)`         | Blueprint | SemanticPlanner + LLM + VisualServo               |
| `navigation(planner)`  | Blueprint | NavigationModule + Autonomy chain                 |
| `safety()`             | Blueprint | SafetyRing + Geofence                             |
| `gateway(port)`        | Blueprint | Gateway + MCP + Teleop                            |


### Pluggable Backends (via Registry)


| Module       | Backends                                                                             |
| ------------ | ------------------------------------------------------------------------------------ |
| Driver       | `thunder` (gRPC鈫抌rainstem), `stub` (testing), `sim_mujoco`, `sim_ros2`               |
| SLAM         | `fastlio2`, `pointlio`, `localizer` (ICP on pre-built map), `bridge` (external ROS2) |
| Detector     | `yoloe`, `yolo_world`, `bpu` (Nash hardware), `grounding_dino`                       |
| Encoder      | `clip` (ViT-B/32), `mobileclip` (edge)                                               |
| LLM          | `kimi`, `openai`, `claude`, `qwen`, `mock`                                           |
| Planner      | `astar` (pure Python), `pct` (C++ ele_planner.so)                                    |
| PathFollower | `nav_core` (C++ nanobind), `pure_pursuit`, `pid`                                     |


### Profiles (Complete 鈥?see `cli/profiles_data.py`)


| Profile          | Driver     | SLAM     | LLM  | Planner | Native | Semantic | Gateway | Use Case                       |
| ---------------- | ---------- | -------- | ---- | ------- | ------ | -------- | ------- | ------------------------------ |
| `stub`           | stub       | none     | mock | astar   | no     | no       | yes     | Framework testing              |
| `dev`            | stub       | none     | mock | astar   | no     | yes      | yes     | Semantic pipeline dev          |
| `sim`            | sim_mujoco | bridge   | mock | astar   | yes    | yes      | yes     | MuJoCo full simulation         |
| `sim_nav`        | stub       | none     | mock | astar   | no     | no       | yes     | Pure-Python sim, no ROS2 / C++ |
| `map`            | s100p      | fastlio2 | mock | astar   | no     | no       | yes     | Build map with SLAM            |
| `nav`            | s100p      | bridge   | qwen | pct     | no     | yes      | yes     | Navigate with pre-built map    |
| `explore`        | s100p      | fastlio2 | qwen | pct     | no     | yes      | yes     | Wavefront exploration          |
| `tare_explore`   | s100p      | fastlio2 | qwen | pct     | no     | yes      | yes     | CMU TARE explorer              |


### Robot Presets

Robot presets provide hardware-specific defaults that get merged into profiles via `--robot`:


| Preset    | `robot`    | `slam_profile` | `detector` | `encoder`  | Extra                                       |
| --------- | ---------- | -------------- | ---------- | ---------- | ------------------------------------------- |
| `stub`    | stub       | none           | yoloe      | mobileclip | 鈥?                                          |
| `sim`     | sim_mujoco | bridge         | yoloe      | mobileclip | 鈥?                                          |
| `ros2`    | sim_ros2   | bridge         | yoloe      | mobileclip | 鈥?                                          |
| `s100p`   | sim_ros2   | localizer      | bpu        | mobileclip | 鈥?                                          |
| `thunder` | thunder    | localizer      | bpu        | mobileclip | `dog_host=192.168.66.190`, `dog_port=13145` |


## 4. Repository Layout

```
lingtu/
鈹溾攢鈹€ lingtu.py              # Primary entry (Module-First CLI + REPL)
鈹溾攢鈹€ main_nav.py            # Alias 鈫?same as lingtu.py
鈹溾攢鈹€ lingtu_cli.py          # pip console script `lingtu`
鈹溾攢鈹€ cli/                   # CLI implementation (profiles, REPL, daemon)
鈹溾攢鈹€ src/                   # Python packages + ROS2 packages (colcon)
鈹?  鈹溾攢鈹€ core/              # Framework: Module, Blueprint, Transport, Registry, stacks, tests
鈹?  鈹溾攢鈹€ nav/               # NavigationModule, SafetyRing, GlobalPlanner, OccupancyGrid, ESDF
鈹?  鈹溾攢鈹€ semantic/          # perception/ (Detector+Encoder), planner/ (GoalResolver+LLM+VisualServo)
鈹?  鈹溾攢鈹€ memory/            # SemanticMapper, EpisodicMemory, Tagged, VectorMemory, KG
鈹?  鈹溾攢鈹€ drivers/           # thunder/ (gRPC), sim/ (stub, MuJoCo, ROS2), TeleopModule
鈹?  鈹溾攢鈹€ gateway/           # GatewayModule (FastAPI), MCPServerModule
鈹?  鈹溾攢鈹€ base_autonomy/     # C++ terrain + local planner + path follower (nanobind)
鈹?  鈹溾攢鈹€ global_planning/   # C++ PCT planner + Python adapters
鈹?  鈹溾攢鈹€ slam/              # C++ SLAM (Fast-LIO2, Point-LIO, PGO, Localizer)
鈹?  鈹斺攢鈹€ reconstruction/    # 3D reconstruction
鈹溾攢鈹€ config/                # YAML / DDS / robot params
鈹溾攢鈹€ launch/                # ROS2 launch (legacy / bridge stacks)
鈹溾攢鈹€ sim/                   # MuJoCo simulation assets + scripts
鈹溾攢鈹€ tests/                 # Integration & planning tests
鈹溾攢鈹€ tools/                 # Robot-side helpers (dashboard, BPU export)
鈹溾攢鈹€ scripts/               # ws/, deploy/, ota/, proto/ helpers
鈹溾攢鈹€ docs/                  # Architecture, guides, ADRs
鈹溾攢鈹€ Makefile               # colcon build, test, docker targets
鈹溾攢鈹€ pyproject.toml         # Root package: lingtu (setuptools)
鈹溾攢鈹€ requirements.txt       # S100P runtime deps
鈹斺攢鈹€ docker-compose.yml     # Container orchestration
```

## 5. Source Directory Detail (`src/`)


| Directory          | Role                                                                                                             |
| ------------------ | ---------------------------------------------------------------------------------------------------------------- |
| `core/`            | Framework: Module, Blueprint, Transport, NativeModule, Registry, stacks/, utils, msgs, tests (640)               |
| `core/msgs/`       | Message types: geometry, nav, sensor, semantic (replaces ROS2 msg types)                                         |
| `core/transport/`  | Backends: Local (callback), DDS (cyclonedds), SHM, Adapter                                                       |
| `core/blueprints/` | System blueprints: `full_stack.py` calls 9 stack factories                                                       |
| `core/spec/`       | Protocol interfaces (16 specs)                                                                                   |
| `core/tests/`      | 640 framework tests (pytest, no ROS2 dependency)                                                                 |
| `nav/`             | NavigationModule, SafetyRing, GlobalPlannerService, WaypointTracker, OccupancyGrid, ESDF, ElevationMap           |
| `semantic/`        | perception/ (Detector+Encoder+SceneGraph), planner/ (SemanticPlanner+LLM+VisualServo+AgentLoop), reconstruction/ |
| `memory/`          | SemanticMapper, EpisodicMemory, TaggedLocations, VectorMemory, RoomObjectKG, TopologySemGraph                    |
| `drivers/`         | thunder/ (ThunderDriver + CameraBridge), sim/ (stub, MuJoCo, ROS2), TeleopModule                                 |
| `gateway/`         | GatewayModule (FastAPI HTTP/WS/SSE), MCPServerModule (MCP tools)                                                 |
| `base_autonomy/`   | TerrainModule + LocalPlannerModule + PathFollowerModule (C++ nanobind backends)                                  |
| `slam/`            | SLAMModule (Fast-LIO2/Point-LIO/Localizer), SlamBridgeModule, C++ SLAM nodes                                     |
| `global_planning/` | pct_planner (C++ ele_planner.so) + _AStarBackend / _PCTBackend (via Registry)                                    |


## 6. Key Files


| File                                                  | Purpose                                                                 |
| ----------------------------------------------------- | ----------------------------------------------------------------------- |
| `lingtu.py`                                           | CLI entry point 鈥?profiles + REPL                                       |
| `src/core/blueprints/full_stack.py`                   | Top-level blueprint (~190 lines, 9 stack factories + cross-stack wires) |
| `src/core/blueprints/stacks/`                         | 9 composable factory functions                                          |
| `src/core/module.py`                                  | Module base class (In/Out, @skill, @rpc, layer tags)                    |
| `src/core/stream.py`                                  | Out[T]/In[T] ports (5 backpressure policies, thread-safe)               |
| `src/core/blueprint.py`                               | Blueprint builder (autoconnect, per-wire transport, auto_wire)          |
| `src/core/registry.py`                                | Plugin registry (@register decorator)                                   |
| `src/nav/navigation_module.py`                        | Global planner + WaypointTracker + mission FSM                          |
| `src/nav/global_planner_service.py`                   | A*/PCT backend + _find_safe_goal BFS                                    |
| `src/nav/safety_ring_module.py`                       | Safety reflex + evaluator + dialogue                                    |
| `src/semantic/planner/.../semantic_planner_module.py` | 5-level fallback + multi-turn AgentLoop                                 |
| `src/semantic/planner/.../goal_resolver.py`           | Fast-Slow dual-process + KG hot-reload                                  |
| `src/semantic/planner/.../visual_servo_module.py`     | BBoxNavigator + PersonTracker (dual channel)                            |
| `src/semantic/planner/.../agent_loop.py`              | Multi-turn LLM tool calling (7 tools)                                   |
| `src/memory/modules/semantic_mapper_module.py`        | SceneGraph 鈫?RoomObjectKG + TopologySemGraph                            |
| `src/memory/modules/vector_memory_module.py`          | CLIP + ChromaDB vector search                                           |
| `src/drivers/teleop_module.py`                        | WebSocket joystick + camera stream                                      |
| `src/slam/slam_module.py`                             | SLAM managed mode (fastlio2/pointlio/localizer)                         |
| `config/robot_config.yaml`                            | Robot physical parameters (single source of truth)                      |


## 7. Build and Test Commands

```bash
# Framework tests (primary, no ROS2 needed)
python -m pytest src/core/tests/ -q                    # ~1000+ tests

# Native nav_core only (no ROS2 needed) 鈥?used by Python autonomy chain
make nav_core                                          # builds _nav_core.so

# ROS2 build (for C++ nodes on S100P only)
source /opt/ros/humble/setup.bash
make build                                              # colcon release build
```

## 8. Critical Files 鈥?Do Not Break

- `src/core/module.py` 鈥?Module base class (all modules depend on it)
- `src/core/blueprint.py` 鈥?Blueprint + autoconnect (system assembly)
- `src/core/stream.py` 鈥?In[T]/Out[T] ports (data flow backbone)
- `src/core/registry.py` 鈥?Plugin registry (all backends depend on it)
- `src/core/utils/` 鈥?Cross-layer utilities (18+ files import from here)
- `src/semantic/perception/.../instance_tracker.py` 鈥?Scene graph builder
- `src/semantic/planner/.../goal_resolver.py` 鈥?5-level resolution chain
- `config/robot_config.yaml` 鈥?Robot physical parameters

## 9. Module Dependency Rules

```
All Modules 鈹€鈹€鈫?core/ (Module, In/Out, Registry, utils, msgs)
                 鈫?only legal dependency direction

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
# Safety 鈫?all actuators
bp.wire("SafetyRingModule", "stop_cmd", driver_name, "stop_signal")
bp.wire("SafetyRingModule", "stop_cmd", "NavigationModule", "stop_signal")

# SLAM odometry priority for NavigationModule
bp.wire("SlamBridgeModule", "odometry", "NavigationModule", "odometry")
# SLAM map_cloud 鈫?all consumers (OccupancyGrid, ElevationMap, Terrain, 鈥?
bp.wire("SlamBridgeModule", "map_cloud", "OccupancyGridModule", "map_cloud")
# (and the same for ElevationMapModule, TerrainModule, VoxelGridModule, RerunBridgeModule, GatewayModule)

# Autonomy chain (when enable_native=True; otherwise Python autonomy module replaces it)
bp.wire("NavigationModule", "waypoint",   "LocalPlannerModule", "waypoint")
bp.wire("TerrainModule",   "terrain_map", "LocalPlannerModule", "terrain_map")
bp.wire("LocalPlannerModule", "local_path", "PathFollowerModule", "local_path")
bp.wire("LocalPlannerModule", "local_path", "SafetyRingModule",  "path")

# Visual servo dual channel
bp.wire("SemanticPlannerModule", "servo_target", "VisualServoModule", "servo_target")
bp.wire("VisualServoModule",     "goal_pose",    "NavigationModule",  "goal_pose")
bp.wire("VisualServoModule",     "nav_stop",     "NavigationModule",  "stop_signal")

# CmdVelMux: priority arbitration over all cmd_vel sources
bp.wire("TeleopModule",       "cmd_vel",          "CmdVelMux", "teleop_cmd_vel")
bp.wire("VisualServoModule",  "cmd_vel",          "CmdVelMux", "visual_servo_cmd_vel")
bp.wire("NavigationModule",   "recovery_cmd_vel", "CmdVelMux", "recovery_cmd_vel")
bp.wire("PathFollowerModule", "cmd_vel",          "CmdVelMux", "path_follower_cmd_vel")
bp.wire("CmdVelMux",          "driver_cmd_vel",    driver_name, "cmd_vel")
# SafetyRing observes the mux winner
bp.wire("CmdVelMux",          "driver_cmd_vel",   "SafetyRingModule", "cmd_vel")

# Teleop active signal pauses NavigationModule
bp.wire("TeleopModule", "teleop_active", "NavigationModule", "teleop_active")
```

## 13. Semantic Navigation

### 5-Level Goal Resolution Chain

```
Instruction: "go to where I left the backpack"
  鈫?
1. Tag Lookup     鈥?exact/fuzzy match in TaggedLocationStore     鈫?goal_pose
2. Fast Path      鈥?scene graph keyword + CLIP matching (<200ms) 鈫?goal_pose
3. Vector Memory  鈥?CLIP embedding search in ChromaDB            鈫?goal_pose
4. Frontier       鈥?topology graph information gain exploration   鈫?goal_pose
5. Visual Servo   鈥?VLM bbox detection + PD tracking             鈫?goal_pose/cmd_vel
```

### Fast-Slow Dual-Process (`goal_resolver.py`)

**Fast Path** (System 1, ~0.17ms): Direct scene graph matching 鈥?keyword + spatial reasoning, confidence fusion (label 35%, CLIP 35%, detector 15%, spatial 15%). Target: >70% hit rate, threshold 0.75.

**Slow Path** (System 2, ~2s): LLM reasoning with ESCA selective grounding 鈥?filters 200 objects to ~15 objects (92.5% token reduction), then calls LLM. Returns OmniNav hierarchical room hint.

**AdaNav Entropy Trigger**: Shannon entropy over candidate scores. If `score_entropy > 1.5` and `confidence < 0.85`, forced escalation to Slow Path.

**LERa Failure Recovery**: 3-step Look-Explain-Replan on subgoal failure. After 2nd consecutive failure: LLM decides `retry_different_path | expand_search | requery_goal | abort`.

### Visual Servo (`visual_servo_module.py`)

Two output channels based on distance:

- Far (> 3m): `goal_pose 鈫?NavigationModule 鈫?planning stack`
- Near (< 3m): `cmd_vel 鈫?CmdVelMux 鈫?driver` (PD servo, bypasses planner; mux gives it priority 80, above PathFollower at 40)

Components: BBoxNavigator (bbox+depth鈫?D鈫扨D), PersonTracker (VLM select+CLIP Re-ID), vlm_bbox_query (open-vocab detection).

### Multi-Turn Agent Loop (`agent_loop.py`)

`agent_instruction` port triggers observe鈫抰hink鈫抋ct cycle with 7 LLM tools:
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


| Mode         | slam_profile | Backend                                | Use Case                    |
| ------------ | ------------ | -------------------------------------- | --------------------------- |
| Mapping      | `fastlio2`   | SLAMModule 鈫?C++ Fast-LIO2             | First visit, build map      |
| Localization | `localizer`  | SLAMModule 鈫?Fast-LIO2 + ICP Localizer | Navigate with pre-built map |
| Bridge       | `bridge`     | SlamBridgeModule 鈫?ROS2 subscriber     | External SLAM (systemd)     |
| None         | `none`       | 鈥?                                     | stub/dev mode               |


Localizer requires Fast-LIO2 companion (provides `/cloud_registered` + `/Odometry`).
SLAM odometry is explicitly wired to NavigationModule (priority over driver dead-reckoning).

## 15. C++/ROS2 Layer 鈥?Coordinate Frames

### Frame Hierarchy

```
map (global map frame) 鈥?fixed world reference
 鈹斺攢鈹€ odom (odometry frame) 鈥?published by PGO/Localizer 鈫?perception + path adaptation
     鈹斺攢鈹€ body (robot body frame) 鈥?Fast-LIO2 output 鈫?local planning + control
         鈹斺攢鈹€ lidar (LiDAR sensor frame) 鈥?raw sensor
```

- **Perception** (terrain_analysis) works in **odom** frame
- **Local planning** (local_planner) works in **body** frame
- **Global planning** (pct_planner) works in **map** frame

### TF Transforms

```
map 鈫?odom: Published by PGO or Localizer (10-20Hz)
odom 鈫?body: Published by Fast-LIO2 (/Odometry, 100Hz)
body 鈫?lidar: Static transform (extrinsics from lio.yaml)
```

## 16. C++/ROS2 Layer 鈥?Topics and Services

### Core Topics


| Topic               | Type         | Publisher                              | Subscriber       | Frame     |
| ------------------- | ------------ | -------------------------------------- | ---------------- | --------- |
| `/livox/lidar`      | CustomMsg    | livox_ros_driver2                      | fastlio2         | lidar     |
| `/livox/imu`        | Imu          | livox_ros_driver2                      | fastlio2         | body      |
| `/cloud_registered` | PointCloud2  | fastlio2                               | PGO, Localizer   | body      |
| `/cloud_map`        | PointCloud2  | fastlio2                               | terrain_analysis | odom      |
| `/Odometry`         | Odometry     | fastlio2                               | all modules      | odom鈫抌ody |
| `/terrain_map`      | PointCloud2  | terrain_analysis                       | local_planner    | odom      |
| `/way_point`        | PointStamped | TaskManager (sole publisher)           | local_planner    | odom      |
| `/planner_waypoint` | PointStamped | pct_path_adapter                       | TaskManager      | odom      |
| `/pct_path`         | Path         | pct_planner                            | pct_path_adapter | map       |
| `/path`             | Path         | local_planner                          | pathFollower     | body      |
| `/cmd_vel`          | Twist        | pathFollower                           | robot_driver     | body      |
| `/slow_down`        | Int8         | local_planner                          | pathFollower     | 鈥?        |
| `/stop`             | Int8         | local_planner, SafetyGate, TaskManager | pathFollower     | 鈥?        |


### Services


| Service             | Type       | Provider  | Purpose                     |
| ------------------- | ---------- | --------- | --------------------------- |
| `/relocalize`       | Relocalize | Localizer | Load map and relocalize     |
| `/relocalize_check` | IsValid    | Localizer | Check relocalization status |
| `/pgo/save_maps`    | SaveMaps   | PGO       | Save optimized maps         |
| `/save_map`         | SaveMaps   | fastlio2  | Save current map            |


## 17. Task Orchestration (gRPC Remote Monitoring)

The C++ `remote_monitoring` layer provides gRPC services for the Flutter client.

### End-to-End Flow

```
Flutter App 鈹€鈹€gRPC鈹€鈹€鈫?ControlService (guard: Lease + AUTONOMOUS)
  鈫?TaskManager (sole /way_point publisher)
    鈫?map鈫抩dom tf2 transform
    鈫?sequential waypoint dispatch
    鈫?/way_point (odom) 鈫?local_planner 鈫?pathFollower 鈫?robot_driver
```

### gRPC RPCs (proto: `src/robot_proto/proto/control.proto`)


| RPC                        | Purpose                                                    |
| -------------------------- | ---------------------------------------------------------- |
| `StartTask`                | Start navigation/mapping/patrol (needs Lease + AUTONOMOUS) |
| `CancelTask`               | Cancel current task                                        |
| `PauseTask` / `ResumeTask` | Suspend/resume                                             |
| `GetTaskStatus`            | Query progress                                             |
| `GetActiveWaypoints`       | Query active waypoints (source/list/progress)              |
| `ClearWaypoints`           | Clear waypoints and stop (needs Lease)                     |
| `SetMode`                  | Switch mode (IDLE/MANUAL/TELEOP/AUTONOMOUS/MAPPING)        |
| `EmergencyStop`            | E-stop (suspends TaskManager)                              |
| `StreamTeleop`             | Bidirectional teleop stream                                |
| `AcquireLease`             | Acquire operation lease                                    |


### Waypoint Coordinate Convention

All waypoints published on `/way_point` are in **odom** frame. App waypoints (map frame) are transformed by TaskManager via tf2 `map鈫抩dom`. The `local_planner` reads `/way_point` x,y directly as odom coordinates with no further transformation.

## 18. MCP Server (AI Agent Control)

Auto-discovered @skill methods exposed via JSON-RPC at `http://<robot>:8090/mcp`.


| Category     | Tools                                                                              |
| ------------ | ---------------------------------------------------------------------------------- |
| Navigation   | `navigate_to`, `navigate_to_object`, `stop`, `get_navigation_status`, `set_mode`   |
| Perception   | `get_scene_graph`, `detect_objects`, `get_robot_position`                          |
| Memory       | `query_memory`, `query_location` (vector), `list_tagged_locations`, `tag_location` |
| Semantic Map | `get_room_summary`, `query_room_for_object`, `get_exploration_target`              |
| Visual Servo | `find_object`, `follow_person`, `stop_servo`, `get_servo_status`                   |
| Planning     | `send_instruction`, `decompose_task`                                               |
| System       | `get_health`, `list_modules`, `get_config`                                         |


```bash
claude mcp add --transport http lingtu http://192.168.66.190:8090/mcp
```

## 19. Teleop (Remote Control)

WebSocket joystick at `ws://<robot>:5050/ws/teleop`:

- Phone/browser sends `{"type": "joy", "lx": 0.5, "ly": 0.0, "az": -0.3}`
- Robot streams JPEG camera frames back (or H.264 via `WebRTCStreamModule` when `aiortc` is installed)
- 3 s idle 鈫?auto-releases to autonomous navigation
- cmd_vel priority (`CmdVelMux`): Teleop 100 > VisualServo 80 > Recovery 60 > PathFollower 40, each with 0.5 s freshness

## 20. CLI Reference (`cli/`)

### Entry Points


| Entry           | Role                                                                                |
| --------------- | ----------------------------------------------------------------------------------- |
| `lingtu.py`     | Primary entry 鈥?sets project root, calls `cli.bootstrap.init`, then `cli.main.main` |
| `main_nav.py`   | Backward-compatible alias (same as `lingtu.py`)                                     |
| `lingtu_cli.py` | Target of pip console script (`pyproject.toml` 鈫?`lingtu = "lingtu_cli:main"`)      |


### CLI Arguments

```bash
lingtu [profile] [options]
```


| Argument          | Type       | Effect                                                                     |
| ----------------- | ---------- | -------------------------------------------------------------------------- |
| `profile`         | positional | Profile name (`stub`, `dev`, `sim`, `map`, `nav`, `explore`)               |
| `stop`            | positional | Stop running daemon (SIGTERM to PID from `.lingtu/run.json`)               |
| `status`          | positional | Show current external run status from `.lingtu/run.json`                   |
| `show-config`     | positional | Print the resolved config for a profile without starting the system         |
| `log`             | positional | Print the current run log; use `-f` to follow                              |
| `doctor`          | positional | Run `scripts/doctor.py` diagnostics                                        |
| `rerun`           | positional | Run `scripts/rerun_live.py` visualization                                  |
| `--list`          | flag       | List all profiles and exit                                                 |
| `--daemon` / `-d` | flag       | Fork as Unix daemon (implies no REPL)                                      |
| `--follow` / `-f` | flag       | Follow output for `lingtu log`                                             |
| `--lines`         | int        | Number of lines to show for `lingtu log` (default 80)                      |
| `--force`         | flag       | Force action for commands such as `lingtu stop`                            |
| `--robot`         | str        | Override robot preset (`stub`, `sim`, `ros2`, `s100p`, `thunder`)          |
| `--dog-host`      | str        | Override Thunder robot host IP                                             |
| `--dog-port`      | int        | Override Thunder robot gRPC port                                           |
| `--detector`      | str        | Override detector backend (`yoloe`, `bpu`, `yolo_world`, `grounding_dino`) |
| `--encoder`       | str        | Override encoder backend (`clip`, `mobileclip`)                            |
| `--llm`           | str        | Override LLM backend (`kimi`, `openai`, `claude`, `qwen`, `mock`)          |
| `--planner`       | str        | Override planner backend (`astar`, `pct`)                                  |
| `--tomogram`      | str        | Override tomogram pickle path                                              |
| `--gateway-port`  | int        | Override gateway HTTP port (default 5050)                                  |
| `--no-semantic`   | flag       | Disable semantic pipeline                                                  |
| `--no-gateway`    | flag       | Disable HTTP/WS/MCP gateway                                                |
| `--no-native`     | flag       | Disable C++ autonomy stack (use Python simple/pid)                         |
| `--rerun`         | flag       | Enable Rerun visualization bridge                                          |
| `--no-repl`       | flag       | No interactive REPL; block on signal                                       |
| `--log-level`     | str        | Root log level (default `INFO`)                                            |


### Profile Resolution Order

1. Profile dict from `PROFILES[name]` (or interactive picker if no positional arg)
2. Robot preset from `ROBOT_PRESETS[robot_key]` merged in (profile fields take priority)
3. CLI overrides (`--llm`, `--detector`, etc.) override everything
4. Resulting dict passed to `full_stack_blueprint(**cfg)`

### CLI Source Files


| File                   | Role                                                               |
| ---------------------- | ------------------------------------------------------------------ |
| `cli/__init__.py`      | Package entry                                                      |
| `cli/bootstrap.py`     | `init()`: prepend `src/`, semantic subdirs to `sys.path`           |
| `cli/main.py`          | Argparse, profile resolution, blueprint build, REPL/daemon loop    |
| `cli/profiles_data.py` | `ROBOT_PRESETS`, `PROFILES`, `_ACTIVE_TOMOGRAM`                    |
| `cli/repl.py`          | `LingTuREPL` (cmd.Cmd subclass) 鈥?interactive commands             |
| `cli/ui.py`            | Banner, interactive profile picker, `list_profiles`, `cmd_stop`    |
| `cli/run_state.py`     | PID file + `.lingtu/run.json` for daemon lifecycle                 |
| `cli/runtime_extra.py` | `preflight`, `kill_residual_ports`, `health_check`, `daemonize`    |
| `cli/logging_util.py`  | `setup_logging` 鈥?stderr + `logs/<timestamp>_<profile>/lingtu.log` |
| `cli/paths.py`         | Project root, `.lingtu/` dir, `logs/` base                         |
| `cli/term.py`          | TTY detection, ANSI color helpers                                  |


### Environment Variables


| Variable            | Default            | Used By                                                           |
| ------------------- | ------------------ | ----------------------------------------------------------------- |
| `NAV_MAP_DIR`       | `~/data/nova/maps` | `profiles_data.py` (`_ACTIVE_TOMOGRAM`), `repl.py` (map commands) |
| `MOONSHOT_API_KEY`  | 鈥?                 | Kimi LLM backend                                                  |
| `OPENAI_API_KEY`    | 鈥?                 | OpenAI LLM backend                                                |
| `ANTHROPIC_API_KEY` | 鈥?                 | Claude LLM backend                                                |
| `DASHSCOPE_API_KEY` | 鈥?                 | Qwen LLM backend                                                  |


### REPL Commands


| Command       | Alias       | Usage                                                                      |
| ------------- | ----------- | -------------------------------------------------------------------------- |
| `navigate`    | `nav`       | `navigate x y [z]` 鈥?pose goal to NavigationModule                         |
| `go`          | 鈥?          | `go <natural language>` 鈥?semantic instruction                             |
| `stop`        | 鈥?          | Emergency stop (value 2 to all stop_signal ports)                          |
| `cancel`      | 鈥?          | Cancel current navigation mission                                          |
| `status`      | `s`         | Module list + mission state                                                |
| `health`      | `h`         | System health report                                                       |
| `map`         | 鈥?          | `list` / `save <name>` / `use <name>` / `build <name>` / `delete <name>`   |
| `smap`        | 鈥?          | `status` / `rooms` / `save` / `load <dir>` / `query <text>` (semantic map) |
| `agent`       | 鈥?          | `agent <multi-step instruction>` (AgentLoop)                               |
| `vmem`        | 鈥?          | `query <text>` / `stats` (vector memory)                                   |
| `teleop`      | 鈥?          | `status` / `release`                                                       |
| `rerun`       | 鈥?          | `on` / `off` / `status` (Rerun visualization bridge)                       |
| `watch`       | `w`         | `watch [interval]` 鈥?auto-refresh status (default 2s)                      |
| `live`        | 鈥?          | Full-screen dashboard with hotkeys (`s/g/x/n/q`)                           |
| `module`      | `m`         | `module <name>` 鈥?inspect one module (tab completion)                      |
| `connections` | `c`         | List all wires                                                             |
| `log`         | 鈥?          | `log debug|info|warning|error` 鈥?set log level                             |
| `config`      | 鈥?          | Print current profile configuration                                        |
| `quit`        | `q`, `exit` | Exit REPL                                                                  |


### Daemon Mode

```bash
python lingtu.py s100p --daemon    # fork to background, log to logs/
python lingtu.py status            # inspect current run
python lingtu.py log -f            # follow the current run log
python lingtu.py show-config nav   # print resolved nav config
python lingtu.py stop              # SIGTERM to running daemon
```

Daemon uses Unix double-fork (`setsid`), writes PID to `.lingtu/run.pid`, state to `.lingtu/run.json`. On startup, `kill_residual_ports` cleans ports 5050, 8090 via `fuser -k`.

## 21. Configuration Files


| File                                       | Purpose                                            |
| ------------------------------------------ | -------------------------------------------------- |
| `config/robot_config.yaml`                 | Robot physical parameters (single source of truth) |
| `config/semantic_planner.yaml`             | Semantic planner configuration                     |
| `config/semantic_perception.yaml`          | Perception pipeline configuration                  |
| `config/semantic_exploration.yaml`         | Exploration parameters                             |
| `config/topic_contract.yaml`               | Port name/type contracts                           |
| `config/layer_contract.yaml`               | Layer dependency contracts                         |
| `config/qos_profiles.yaml`                 | QoS/backpressure profiles                          |
| `config/cyclonedds.xml`                    | CycloneDDS configuration                           |
| `src/slam/fastlio2/config/lio.yaml`        | Fast-LIO2 parameters                               |
| `src/slam/pgo/config/pgo.yaml`             | PGO parameters                                     |
| `src/slam/localizer/config/localizer.yaml` | Localizer parameters                               |


## 22. S100P Deployment

- **SSH**: `ssh sunrise@192.168.66.190`
- **Nav code**: `~/data/inovxio/lingtu/`
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
- S100P has no CUDA 鈥?Open3D GPU features unavailable, use C++ terrain_analysis instead
- Kimi API key may expire 鈥?Slow Path unavailable without valid LLM key
- ChromaDB optional 鈥?VectorMemoryModule falls back to numpy brute-force search
- Framework tests (640) are mock-based 鈥?real hardware integration tests need S100P

## 25. Related Documentation


| Document                                          | Purpose                                           |
| ------------------------------------------------- | ------------------------------------------------- |
| `CLAUDE.md`                                       | Claude Code-specific guidance (mirrors this file) |
| `docs/MODULE_FIRST_GUIDELINE.md`                  | 8 rules for Module-First architecture             |
| `docs/REPO_LAYOUT.md`                             | Top-level directory map                           |
| `docs/QUICKSTART.md`                              | Getting started guide                             |
| `docs/TUNING.md`                                  | Parameter tuning guide                            |
| `src/base_autonomy/TERRAIN_ANALYSIS_EXPLAINED.md` | Terrain analysis deep dive                        |
| `src/remote_monitoring/README.md`                 | gRPC Gateway documentation                        |
| `src/robot_proto/proto/control.proto`             | Task control protocol definition                  |


---

## 26. Feature-by-Feature Breakdown and Quick Verification

### 26.1 Localization (瀹氫綅)

**What it does**: Matches live LiDAR scans against a pre-built PCD map using ICP to produce a corrected `map鈫抩dom` TF transform. Requires Fast-LIO2 running as the odometry source.

**Key source files**:


| File                                            | Role                                                                                |
| ----------------------------------------------- | ----------------------------------------------------------------------------------- |
| `src/slam/localizer/`                           | C++ ICP localizer node                                                              |
| `src/slam/localizer/config/localizer.yaml`      | `static_map_path`, ICP resolutions, `update_hz`                                     |
| `src/slam/launch/localizer_launch.py` | Launches Fast-LIO2 + localizer + RViz                                               |
| `src/slam/slam_module.py`                       | Python `SLAMModule` 鈥?`_setup_localizer()` starts LIO + localizer via NativeModule  |
| `src/slam/slam_bridge_module.py`                | `SlamBridgeModule` 鈥?subscribes to `/nav/odometry` + `/nav/map_cloud` via DDS/rclpy |
| `src/core/blueprints/stacks/slam.py`            | `slam("localizer")` 鈥?ensures systemd services, adds `SlamBridgeModule`             |


**Data flow**:

```
Livox LiDAR 鈫?Fast-LIO2 (/cloud_registered + /Odometry)
                 鈫?
           Localizer (ICP match against PCD)
                 鈫?
           TF: map鈫抩dom + /localization_quality
                 鈫?
           SlamBridgeModule 鈫?Python odometry/map_cloud ports
```

**Quick verification**:


| Method                     | Command                                                                                    | Needs             |
| -------------------------- | ------------------------------------------------------------------------------------------ | ----------------- |
| Framework ports/registry   | `python -m pytest src/core/tests/test_new_modules.py::TestSLAMModule -q`                   | Nothing           |
| Bridge module smoke        | `python -m pytest src/core/tests/test_integration_36.py -q`                                | Nothing           |
| Profile build check        | `python -c "from core.blueprints.stacks import slam; bp = slam('localizer'); print('OK')"` | Nothing           |
| Full ROS2 localizer        | `ros2 launch localizer localizer_launch.py` + `ros2 service call /relocalize ...`          | ROS2 + LiDAR      |
| Check localization quality | `ros2 topic echo /localization_quality`                                                    | Running localizer |


**Key parameters** (`localizer.yaml`):

- `static_map_path` 鈥?PCD map file (empty = no auto-load, must call `/relocalize`)
- `update_hz` 鈥?ICP update rate
- Rough/refine ICP resolutions and score thresholds

---

### 26.2 Mapping (寤哄浘)

**What it does**: Builds a 3D point cloud map using Fast-LIO2 (LiDAR-Inertial odometry) with optional PGO loop closure. The map is saved as PCD and then processed offline into a Tomogram for planning.

**Key source files**:


| File                                          | Role                                                      |
| --------------------------------------------- | --------------------------------------------------------- |
| `src/slam/fastlio2/`                          | C++ Fast-LIO2 SLAM node                                   |
| `src/slam/pgo/`                               | C++ Pose Graph Optimization (loop closure)                |
| `src/slam/slam_module.py`                     | `_setup_fastlio2()` 鈥?starts Livox + LIO + PGO            |
| `src/core/blueprints/stacks/slam.py`          | `slam("fastlio2")` 鈥?ensures `slam` + `slam_pgo` services |
| `src/global_planning/pct_planner/tomography/` | Offline PCD 鈫?Tomogram conversion                         |


**Data flow**:

```
Livox LiDAR 鈫?Fast-LIO2 (/cloud_map + /Odometry)
                 鈫?
              PGO (loop closure, optional)
                 鈫?
           /pgo/save_maps 鈫?map.pcd
                 鈫?(offline)
           tomography.py 鈫?tomogram.pickle
```

**Quick verification**:


| Method              | Command                                                                                                                                                           | Needs        |
| ------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------ |
| Framework smoke     | `python -m pytest src/core/tests/test_new_modules.py::TestSLAMModule -q`                                                                                          | Nothing      |
| Profile `map` check | `python -c "from core.blueprints.full_stack import full_stack_blueprint; bp = full_stack_blueprint(slam_profile='fastlio2', enable_semantic=False); print('OK')"` | Nothing      |
| Map REPL            | `python lingtu.py map` then `map save test_map`                                                                                                                   | ROS2 + LiDAR |
| ROS2 direct         | `ros2 launch fastlio2_launch.py` + `ros2 service call /save_map ...`                                                                                          | ROS2 + LiDAR |
| Tomogram generation | `cd src/global_planning/pct_planner && python3 tomography/scripts/tomography.py --scene Common`                                                                   | Saved PCD    |


**Key parameters**:

- `lio.yaml`: `lidar_type`, `scan_line`, `map_resolution`, `max_map_points`, extrinsics (`t_il`, `r_il`)
- `pgo.yaml`: keyframe deltas, loop search radius, score thresholds
- Map storage: `~/data/nova/maps/<name>/map.pcd` (default `NAV_MAP_DIR`)

---

### 26.3 Global Navigation (鍏ㄥ眬瀵艰埅)

**What it does**: Given a `goal_pose`, plans a global path using A* (or PCT) on a 2D costmap or tomogram, then dispatches waypoints to the local planner. Handles mission FSM (IDLE鈫扨LANNING鈫扙XECUTING鈫扴UCCESS/STUCK/FAILED), stuck detection, and replanning.

**Key source files**:


| File                                                            | Role                                                                            |
| --------------------------------------------------------------- | ------------------------------------------------------------------------------- |
| `src/nav/navigation_module.py`                                  | `NavigationModule` 鈥?mission FSM, goal handling, replan logic                   |
| `src/nav/global_planner_service.py`                             | `GlobalPlannerService` 鈥?A*/PCT backend, `_find_safe_goal` BFS, path downsample |
| `src/nav/waypoint_tracker.py`                                   | `WaypointTracker` 鈥?2D arrival detection, stuck timeout                         |
| `src/nav/occupancy_grid_module.py`                              | `OccupancyGridModule` 鈥?LiDAR 鈫?2D costmap (feeds live A* replanning)           |
| `src/nav/esdf_module.py`                                        | `ESDFModule` 鈥?signed distance field from occupancy                             |
| `src/nav/elevation_map_module.py`                               | `ElevationMapModule` 鈥?per-cell height                                          |
| `src/global_planning/pct_adapters/src/global_planner_module.py` | Registers `_AStarBackend` and `_PCTBackend`                                     |
| `src/core/blueprints/stacks/navigation.py`                      | `navigation()` stack factory                                                    |


**Data flow**:

```
goal_pose (from Gateway/MCP/SemanticPlanner)
    鈫?
NavigationModule._on_goal 鈫?_plan()
    鈫?
GlobalPlannerService.plan(start, goal)
    鈫?_find_safe_goal (BFS snap to free cell)
    鈫?_AStarBackend.plan (2D A* on costmap/tomogram)
    鈫?_downsample (sparse waypoints, 2m apart)
    鈫?
WaypointTracker.reset(path) 鈫?first waypoint
    鈫?
NavigationModule publishes waypoint (PoseStamped)
    鈫?
LocalPlannerModule 鈫?PathFollowerModule 鈫?cmd_vel 鈫?Driver

Parallel: OccupancyGridModule 鈫?costmap 鈫?GlobalPlannerService.update_map
          (live replanning when new obstacles detected, 鈮?s cooldown)
```

**Mission states**: `IDLE` 鈫?`PLANNING` 鈫?`EXECUTING` 鈫?`SUCCESS` | `STUCK` (replan up to 3x) | `FAILED` | `CANCELLED`

**Quick verification**:


| Method                | Command                                                                        | Needs       |
| --------------------- | ------------------------------------------------------------------------------ | ----------- |
| NavigationModule FSM  | `python -m pytest src/core/tests/test_new_modules.py::TestNavigationModule -q` | Nothing     |
| Safe goal + planner   | `python -m pytest src/core/tests/test_integration_36.py -q`                    | Nothing     |
| WaypointTracker stuck | `python -m pytest src/core/tests/test_new_features.py -q`                      | Nothing     |
| Full wiring check     | `python -m pytest src/core/tests/test_cross_module_integration.py -q`          | Nothing     |
| Non-native blueprint  | `python -m pytest src/core/tests/test_non_native_navigation_blueprint.py -q`   | Nothing     |
| PCT adapter logic     | `python3 tests/planning/test_pct_adapter_logic.py`                             | Nothing     |
| Stub profile (live)   | `python lingtu.py stub` then REPL: `go 10 5`                                   | Nothing     |
| Sim profile (live)    | `python lingtu.py sim` then REPL: `go 10 5`                                    | MuJoCo deps |


**Key parameters** (`NavigationModule`):

- `planner`: `"astar"` or `"pct"` (registry backend)
- `tomogram`: path to `.pickle` (PCT/A* map source)
- `waypoint_threshold`: 1.5m (arrival distance)
- `stuck_timeout`: 10s / `stuck_dist_thre`: 0.15m
- `max_replan_count`: 3
- `downsample_dist`: 2.0m (waypoint spacing)
- `allow_direct_goal_fallback`: True (single-waypoint if no map)

---

### 26.4 Local Planning (灞€閮ㄨ鍒?

**What it does**: Receives a global waypoint + terrain map, selects a collision-free local path from a pre-computed path library (CMU-style) or simple straight-line, then follows it with Pure Pursuit to produce `cmd_vel`.

**Key source files**:


| File                                                | Role                                                         |
| --------------------------------------------------- | ------------------------------------------------------------ |
| `src/base_autonomy/modules/terrain_module.py`       | `TerrainModule` 鈥?point cloud 鈫?terrain map + traversability |
| `src/base_autonomy/modules/local_planner_module.py` | `LocalPlannerModule` 鈥?waypoint + terrain 鈫?local path       |
| `src/base_autonomy/modules/path_follower_module.py` | `PathFollowerModule` 鈥?local path 鈫?cmd_vel                  |
| `src/base_autonomy/modules/autonomy_module.py`      | `add_autonomy_stack()` 鈥?adds all three modules              |
| `src/base_autonomy/terrain_analysis/`               | C++ terrain analysis node                                    |
| `src/base_autonomy/local_planner/`                  | C++ local planner + pathFollower                             |


**Data flow**:

```
NavigationModule 鈫?waypoint (PoseStamped)
    鈫?
LocalPlannerModule (In: waypoint + terrain_map + odometry)
    鈫?score candidate paths against obstacle grid
    鈫?select best group, lowest penalty
    鈫?
local_path (Path) 鈫?PathFollowerModule
    鈫?Pure Pursuit: lookahead + yaw rate limiting
    鈫?
cmd_vel (Twist) 鈫?Driver
```

**Backends**:

- Terrain: `nanobind` (C++ `_nav_core.TerrainAnalysisCore`), `native` (subprocess), `simple`
- Local planner: `cmu` (CMU-style 12,348 candidate paths), `simple` (straight line)
- Path follower: `nav_core` (C++ nanobind), `pure_pursuit` (native), `pid`

**Quick verification**:


| Method                         | Command                                                                          | Needs               |
| ------------------------------ | -------------------------------------------------------------------------------- | ------------------- |
| Module smoke (simple/pid)      | `python -m pytest src/core/tests/test_new_modules.py::TestLocalPlannerModule -q` | Nothing             |
| PathFollower smoke             | `python -m pytest src/core/tests/test_new_modules.py::TestPathFollowerModule -q` | Nothing             |
| Wiring Nav鈫扡ocal鈫扚ollow鈫扗river | `python -m pytest src/core/tests/test_cross_module_integration.py -q`            | Nothing             |
| Non-native stack               | `python -m pytest src/core/tests/test_non_native_navigation_blueprint.py -q`     | Nothing             |
| C++ nodes (ROS2)               | `ros2 launch local_planner local_planner.launch`                                 | ROS2 + built C++    |
| Stub planning pipeline         | `bash tests/integration/test_planning_stub.sh`                                   | ROS2 + `make build` |


**Key parameters**:

- `LocalPlannerModule`: `_PATH_RANGE=3.5m`, `_OBSTACLE_HEIGHT_THRE=0.5m`, `_DIR_THRE=90deg`
- `PathFollowerModule`: `max_speed`, `lookahead`, `yaw_rate_limit`
- C++ `localPlanner`: `vehicleHeight`, `vehicleWidth`, `adjacentRange`
- C++ `pathFollower`: `baseLookAheadDis_`, `lookAheadRatio_`, `slowRate1/2/3`

---

### 26.5 Semantic Navigation (璇箟瀵艰埅)

**What it does**: Resolves natural language instructions ("go to the red chair") into `goal_pose` through a 5-level fallback chain: tag lookup 鈫?fast scene-graph matching 鈫?vector memory search 鈫?frontier exploration 鈫?visual servo. Includes LERa failure recovery.

**Key source files**:


| File                                                               | Role                                                      |
| ------------------------------------------------------------------ | --------------------------------------------------------- |
| `src/semantic/planner/semantic_planner/semantic_planner_module.py` | Unified planner: instruction 鈫?`_try_resolve` 鈫?goal_pose |
| `src/semantic/planner/semantic_planner/goal_resolver.py`           | `GoalResolver` 鈥?fusion weights, KG reload                |
| `src/semantic/planner/semantic_planner/fast_path.py`               | `fast_resolve()` 鈥?System 1 matching (~0.17ms)            |
| `src/semantic/planner/semantic_planner/slow_path.py`               | `async resolve()` 鈥?tag + AdaCoT + LLM (~2s)              |
| `src/semantic/planner/semantic_planner/agent_loop.py`              | Multi-turn LLM tool calling (7 tools, 10 steps max)       |
| `src/semantic/planner/semantic_planner/task_decomposer.py`         | Rules-based instruction decomposition                     |
| `src/semantic/planner/semantic_planner/action_executor.py`         | LERa failure recovery strategies                          |
| `src/semantic/perception/semantic_perception/perception_module.py` | RGB-D + odom 鈫?scene graph                                |
| `src/semantic/perception/semantic_perception/instance_tracker.py`  | Multi-object tracking, scene graph builder                |
| `src/memory/modules/semantic_mapper_module.py`                     | Scene graph 鈫?RoomObjectKG + TopologySemGraph             |
| `src/memory/modules/vector_memory_module.py`                       | CLIP + ChromaDB vector search                             |
| `src/memory/modules/tagged_locations_module.py`                    | Named location store                                      |
| `src/memory/modules/episodic_module.py`                            | Episodic memory for LLM context                           |


**Data flow**:

```
instruction (str) from Gateway/MCP
    鈫?
SemanticPlannerModule._on_instruction
    鈫?optional TaskDecomposer
    鈫?
_try_resolve(instruction, scene_graph_json):
    鈹溾攢 Level 2: GoalResolver.fast_resolve() 鈥?keyword + CLIP fusion
    鈹?  weights: label 35%, CLIP 35%, detector 15%, spatial 15%
    鈹?  if confidence 鈮?0.75 鈫?goal_pose 鉁?
    鈹?
    鈹溾攢 Level 3: VectorMemoryModule.query_location(instruction)
    鈹?  CLIP embedding search 鈫?if score 鈮?0.3 鈫?goal_pose 鉁?
    鈹?
    鈹溾攢 Level 4: FrontierScorer.get_best_frontier()
    鈹?  topology graph information gain 鈫?goal_pose (EXPLORING)
    鈹?
    鈹斺攢 Level 5: servo_target.publish("find:<instruction>")
        鈫?VisualServoModule (see Tracking below)

LERa Recovery (on mission_status STUCK/FAILED):
    ActionExecutor.lera_recover 鈫?retry_different_path | expand_search | requery_goal | abort
```

**Agent loop** (triggered by `agent_instruction` port):

```
agent_instruction 鈫?AgentLoop(llm_client, 7 tools, max_steps=10, timeout=120s)
    鈫?observe鈫抰hink鈫抋ct cycle
    tools: navigate_to, navigate_to_object, detect_object,
           query_memory, tag_location, say, done
```

**Quick verification**:


| Method                   | Command                                                                        | Needs              |
| ------------------------ | ------------------------------------------------------------------------------ | ------------------ |
| Planner module wiring    | `python -m pytest src/core/tests/test_semantic_planner_modules.py -q`          | Nothing            |
| Visual servo integration | `python -m pytest src/core/tests/test_visual_servo_semantic_integration.py -q` | Nothing            |
| Full semantic pipeline   | `python -m pytest src/core/tests/test_sim_semantic_pipeline_blueprint.py -q`   | Nothing            |
| GoalResolver unit tests  | `python -m pytest src/semantic/planner/tests/test_goal_resolver.py -q`          | `semantic_common`  |
| Fast resolve tests       | `python -m pytest src/semantic/planner/tests/test_fast_resolve.py -q`           | `semantic_common`  |
| BBoxNavigator tests      | `python -m pytest src/semantic/planner/tests/test_bbox_navigator.py -q`         | `semantic_common`  |
| PersonTracker tests      | `python -m pytest src/semantic/planner/tests/test_person_tracker.py -q`         | `semantic_common`  |
| Instance tracker tests   | `python -m pytest src/semantic/perception/tests/test_instance_tracker.py -q`   | `semantic_common`  |
| Dev profile (live)       | `python lingtu.py dev` then REPL: `go red chair`                               | Nothing (mock LLM) |
| Dev with real LLM        | `python lingtu.py dev --llm kimi` then REPL: `go red chair`                    | `MOONSHOT_API_KEY` |


**Key parameters**:

- `SemanticPlannerModule`: `fast_path_threshold=0.75`, `lera_cooldown=15s`, `save_dir`
- `GoalResolver`: confidence weights (label/CLIP/detector/spatial), KG reload interval (120s)
- `PerceptionModule`: `detector_type`, `confidence_threshold`, `skip_frames`
- `VectorMemoryModule`: `store_interval`, `max_results`, `persist_dir`

---

### 26.6 Tracking and Following (璺熻釜)

**What it does**: Tracks and follows objects or people using visual servo. Two modes: **Find** (detect and approach a named object) and **Follow** (track a specific person). Dual output channels: far-range publishes `goal_pose` to NavigationModule, close-range publishes `cmd_vel` directly to driver with PD control.

**Key source files**:


| File                                                           | Role                                                |
| -------------------------------------------------------------- | --------------------------------------------------- |
| `src/semantic/planner/semantic_planner/visual_servo_module.py` | `VisualServoModule` 鈥?dual-channel servo controller |
| `src/semantic/planner/semantic_planner/bbox_navigator.py`      | `BBoxNavigator` 鈥?bbox + depth 鈫?3D 鈫?PD control    |
| `src/semantic/planner/semantic_planner/person_tracker.py`      | `PersonTracker` 鈥?VLM select + CLIP Re-ID           |


**Data flow 鈥?Find mode** (object tracking):

```
servo_target = "find:red chair"
    鈫?
VisualServoModule._tick_find
    鈫?match label in scene_graph.objects
    鈫?BBoxNavigator.update(bbox, depth)
    鈫?compute 3D position + distance
    鈹?
    鈹溾攢 distance > 3m (far): goal_pose 鈫?NavigationModule 鈫?planning stack
    鈹?
    鈹斺攢 distance 鈮?3m (near): cmd_vel 鈫?Driver directly (PD servo)
                              + nav_stop 鈫?NavigationModule (pause planner)
```

**Data flow 鈥?Follow mode** (person following):

```
servo_target = "follow:person_description"
    鈫?
VisualServoModule._tick_follow
    鈫?PersonTracker.update(scene_objects, rgb_frame)
    鈫?get_follow_waypoint 鈫?goal_pose
    鈫?
NavigationModule 鈫?planning stack (always far-range for follow)
```

**Mutual exclusion**: When close-range servo is active, `nav_stop=1` is published to pause NavigationModule so PathFollower stops producing conflicting `cmd_vel`.

**Wiring in `full_stack.py`**:

```python
bp.wire("SemanticPlannerModule", "servo_target", "VisualServoModule", "servo_target")
bp.wire("VisualServoModule", "goal_pose", "NavigationModule", "goal_pose")
bp.wire("VisualServoModule", "nav_stop", "NavigationModule", "stop_signal")
bp.wire("VisualServoModule", "cmd_vel", driver_name, "cmd_vel")
```

**Quick verification**:


| Method                 | Command                                                                        | Needs             |
| ---------------------- | ------------------------------------------------------------------------------ | ----------------- |
| Servo integration test | `python -m pytest src/core/tests/test_visual_servo_semantic_integration.py -q` | Nothing           |
| BBoxNavigator unit     | `python -m pytest src/semantic/planner/tests/test_bbox_navigator.py -q`         | `semantic_common` |
| PersonTracker unit     | `python -m pytest src/semantic/planner/tests/test_person_tracker.py -q`         | `semantic_common` |
| Full wiring check      | `python -m pytest src/core/tests/test_cross_module_integration.py -q`          | Nothing           |
| MCP tools              | `python -m pytest src/semantic/planner/tests/test_mcp_server.py -q`             | `semantic_common` |


**Key parameters**:

- `VisualServoModule`: `servo_takeover_distance=3.0m`, `follow_distance`, `lost_timeout`
- `BBoxNavigator` (`BBoxNavConfig`): `target_distance`, `max_linear_speed`, `max_angular_speed`, `lost_timeout`
- cmd_vel priority: Teleop > VisualServo > PathFollower

---

## 27. Testing Infrastructure Summary

### Test Categories


| Category         | Location                         | Count       | ROS2? | Hardware? | Command                                              |
| ---------------- | -------------------------------- | ----------- | ----- | --------- | ---------------------------------------------------- |
| Core framework   | `src/core/tests/`                | ~640        | No    | No        | `python -m pytest src/core/tests/ -q`                |
| Semantic planner | `src/semantic/planner/tests/`     | ~30 files   | No    | No        | `python -m pytest src/semantic/planner/tests/ -q`     |
| Perception       | `src/semantic/perception/tests/` | ~15 files   | No    | No        | `python -m pytest src/semantic/perception/tests/ -q` |
| Planning logic   | `tests/planning/`                | ~5 files    | No    | No        | `python3 tests/planning/test_pct_adapter_logic.py`   |
| Integration      | `tests/integration/`             | ~10 scripts | Yes   | Some      | `make test-integration`                              |
| Benchmark        | `tests/benchmark/`               | 3 scripts   | Yes   | No        | `make benchmark`                                     |
| ROS2 colcon      | Various `package.xml`            | 鈥?          | Yes   | No        | `make test`                                          |


### Quick Verification Ladder (fastest to most complete)

```bash
# 1. Framework only (no deps, ~50s) 鈥?validates Module/Blueprint/Registry/Transport
python -m pytest src/core/tests/ -q

# 2. Semantic planner (may skip without semantic_common)
python -m pytest src/semantic/planner/tests/ -q

# 3. Perception (may skip without semantic_common/cv2)
python -m pytest src/semantic/perception/tests/ -q

# 4. Stub profile (live REPL, no hardware)
python lingtu.py stub

# 5. Dev profile (semantic pipeline, no hardware, mock LLM)
python lingtu.py dev

# 6. Sim profile (MuJoCo + full stack)
python lingtu.py sim

# 7. ROS2 stub pipeline (needs built workspace)
bash tests/integration/test_planning_stub.sh

# 8. Full integration (needs ROS2 + built workspace)
make test-integration
```

### Profile-Based Verification


| Profile   | What it exercises                                                                | How to start               |
| --------- | -------------------------------------------------------------------------------- | -------------------------- |
| `stub`    | Framework, Blueprint wiring, Gateway, A* planner (no semantic, no hardware)      | `python lingtu.py stub`    |
| `dev`     | All of `stub` + perception + memory + semantic planner + visual servo (mock LLM) | `python lingtu.py dev`     |
| `sim`     | Full algorithm stack with MuJoCo physics simulation + native C++ autonomy        | `python lingtu.py sim`     |
| `map`     | SLAM mapping mode (Fast-LIO2 + PGO)                                              | `python lingtu.py map`     |
| `s100p`   | Real robot with localizer + semantic + BPU detector                              | `python lingtu.py s100p`   |
| `explore` | Real robot with SLAM + full native + semantic (no pre-built map)                 | `python lingtu.py explore` |


