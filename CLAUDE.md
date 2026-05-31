# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

LingTu (鐏甸€? is an autonomous navigation system for quadruped robots in outdoor/off-road environments.

- **Platform**: S100P (RDK X5, Nash BPU 128 TOPS, aarch64) | ROS2 Humble | Ubuntu 22.04
- **Languages**: Python (framework + semantic modules), C++ (SLAM/terrain/planner)
- **Architecture**: Module-First 鈥?Module is the only runtime unit, Blueprint is the only orchestration
- **Guideline**: `docs/MODULE_FIRST_GUIDELINE.md` 鈥?8 rules for how code should be structured

## Quick Start

```bash
# Framework tests (no ROS2 needed, runs on any machine)
python -m pytest src/core/tests/ -q       # 1226 tests

# CLI with interactive REPL (profile-based, recommended)
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
    driver("thunder", host="192.168.66.190"),   # L1 Robot + camera bridge
    slam("localizer"),                           # L1 SLAM / localization
    maps(),                                      # L2 OccupancyGrid + ESDF + ElevationMap
    perception("bpu"),                           # L3 Detector + Encoder + Reconstruction
    memory(),                                    # L3 SemanticMapper + Episodic + Tagged + VectorMemory
    planner("kimi"),                             # L4 SemanticPlanner + LLM + VisualServo
    navigation("astar"),                         # L5 Navigation + Autonomy chain
    safety(),                                    # L0 SafetyRing + Geofence
    gateway(5050),                               # L6 HTTP/WS/SSE + MCP + Teleop
).build()
system.start()
```

## Architecture 鈥?Module-First with Composable Stacks

Module is the only runtime unit. Blueprint composes Modules. Factory functions bundle related Modules into reusable stacks.

### Layer Hierarchy

```
L0  Safety       鈥?SafetyRingModule + GeofenceManagerModule + CmdVelMux
L1  Hardware     鈥?Driver + CameraBridge + SLAM (managed/bridge/localizer)
L2  Maps         鈥?OccupancyGrid + ESDF + ElevationMap + Terrain + LocalPlanner + PathFollower
L3  Perception   鈥?Detector + Encoder + Reconstruction + SemanticMapper + Episodic + Tagged + VectorMemory
L4  Decision     鈥?SemanticPlanner + LLM + VisualServo (bbox tracking + person following)
L5  Planning     鈥?NavigationModule (A*/PCT + WaypointTracker + mission FSM + goal safety)
L6  Interface    鈥?Gateway + MCP + Teleop
```

High layers 鈫?low layers only. L5鈫扡2 (waypoint鈫扨athFollower) is command dispatch, not dependency.

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
| `exploration(backend)` | Blueprint | TARE exploration (wavefront removed 2026-04) |
| `lidar(ip, enabled)` | Blueprint | Livox MID-360 hardware driver (decoupled from SLAM) |
| `sim_lidar(scene_xml)` | Blueprint | Simulated PointCloud2 from MuJoCo scene geometry |

### Pluggable Backends (via Registry)

| Module | Backends |
|--------|----------|
| Driver | `thunder` (gRPC鈫抌rainstem), `stub` (testing), `sim_mujoco`, `sim_ros2` |
| SLAM | `fastlio2`, `pointlio`, `localizer` (ICP on pre-built map), `bridge` (external ROS2) |
| Detector | `yoloe`, `yolo_world`, `bpu` (Nash hardware), `grounding_dino` |
| Encoder | `clip` (ViT-B/32), `mobileclip` (edge) |
| LLM | `kimi`, `openai`, `claude`, `qwen`, `mock` |
| Planner | `astar` (pure Python), `pct` (C++ ele_planner.so) |
| PathFollower | `nav_core` (C++ nanobind), `pure_pursuit`, `pid` |

All backends registered via `@register("category", "name")` in `core.registry`. Zero if/else.

### Profiles

Full definitions in `cli/profiles_data.py` (14 named profiles + 7 robot presets = 21 total combinations).

| Profile | Robot Preset | SLAM | Semantic | Use Case |
|---------|-------------|------|----------|----------|
| `stub` | stub | none | no | Framework testing |
| `dev` | stub | none | yes | Semantic pipeline dev |
| `map` | s100p | fastlio2 | no | Build new map via SLAM |
| `nav` | s100p | bridge | yes | Navigate using saved map (PCT) |
| `explore` | s100p | fastlio2 | yes | Wavefront frontier exploration |
| `tare_explore` | s100p | fastlio2 | yes | CMU TARE hierarchical exploration |
| `sim` | sim | bridge | yes | MuJoCo full simulation |
| `sim_mujoco_live` | sim_gazebo | none | no | MuJoCo raw MID-360 + Fast-LIO live sim |
| `sim_gazebo` | sim_gazebo | none | yes | Gazebo/GZ ROS-native simulation |
| `sim_industrial` | sim_gazebo | none | yes | Engineering industrial-yard simulation |
| `sim_cmu_tare` | sim_gazebo | none | no | CMU Unity + external TARE simulation |
| `sim_nav` | stub | none | no | Pure-Python nav sim (no ROS2/C++) |
| `super_lio` | s100p | super_lio | yes | Evaluate Super-LIO backend |
| `super_lio_relocation` | s100p | super_lio_relocation | yes | Super-LIO relocation against saved map |
| `s100p` | thunder | localizer | yes | Real robot navigation (alias) |
| `thunder` | thunder | localizer | yes | Direct thunder robot control |
| `navigate` | thunder | localizer | yes | Alias for s100p |
| `ros2` | sim_ros2 | bridge | yes | ROS2 simulation |
| `sim_gazebo` | sim_gazebo | none | yes | Gazebo sim (repeated as standalone) |
| `explore` | thunder | fastlio2 | yes | yes | Exploration (no map) |

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

## Source Directory (`src/`)

| Directory | Role |
|-----------|------|
| `core/` | Framework: Module, Blueprint, Transport, NativeModule, Registry, stacks/, utils, msgs, tests (948) |
| `nav/` | NavigationModule, SafetyRing, CmdVelMux, GlobalPlannerService, WaypointTracker, OccupancyGrid, ESDF, ElevationMap |
| `semantic/` | perception/ (Detector+Encoder), planner/ (SemanticPlanner+LLM+VisualServo+AgentLoop), reconstruction/ |
| `memory/` | SemanticMapper, EpisodicMemory, TaggedLocations, VectorMemory, RoomObjectKG, TopologySemGraph |
| `drivers/` | thunder/ (ThunderDriver + CameraBridge), sim/ (stub, MuJoCo, ROS2), TeleopModule |
| `gateway/` | GatewayModule (FastAPI HTTP/WS/SSE), MCPServerModule (MCP tools) |
| `base_autonomy/` | TerrainModule + LocalPlannerModule + PathFollowerModule (C++ nanobind backends) |
| `slam/` | SLAMModule (Fast-LIO2/Point-LIO/Localizer), SlamBridgeModule, C++ SLAM nodes |
| `global_planning/` | pct_planner (C++ ele_planner.so) + _AStarBackend / _PCTBackend (via Registry) |

Note: `calibration/` and `sim/` live at repo root (not under `src/`). See [Sensor Calibration](#sensor-calibration-calibration) section. For `sim/`, see the `sim/engine/` documentation below.

## Key Files

| File | Purpose |
|------|---------|
| `docs/REPO_LAYOUT.md` | Top-level directory map (where `src/`, `scripts/`, `tools/`, 鈥?live) |
| `lingtu.py` | CLI entry point 鈥?profiles + REPL (`main_nav.py` kept as alias) |
| `src/core/blueprints/full_stack.py` | Top-level blueprint (~60 lines, calls 9 stack factories) |
| `src/core/blueprints/stacks/` | 9 composable factory functions |
| `src/core/module.py` | Module base class (In/Out, @skill, @rpc, layer tags) |
| `src/core/stream.py` | Out[T]/In[T] ports (5 backpressure policies, thread-safe) |
| `src/core/blueprint.py` | Blueprint builder (autoconnect, per-wire transport, auto_wire) |
| `src/core/registry.py` | Plugin registry (@register decorator) |
| `src/nav/navigation_module.py` | Global planner + WaypointTracker + mission FSM |
| `src/nav/global_planner_service.py` | A*/PCT backend + _find_safe_goal BFS |
| `src/nav/waypoint_tracker.py` | Arrival + stuck detection |
| `src/nav/safety_ring_module.py` | Safety reflex + evaluator + dialogue |
| `src/nav/cmd_vel_mux_module.py` | Priority-based cmd_vel arbitration (L0) |
| `src/semantic/planner/.../semantic_planner_module.py` | 5-level fallback + multi-turn AgentLoop |
| `src/semantic/planner/.../goal_resolver.py` | Fast-Slow dual-process + KG hot-reload |
| `src/semantic/planner/.../visual_servo_module.py` | BBoxNavigator + PersonTracker (dual channel) |
| `src/semantic/planner/.../agent_loop.py` | Multi-turn LLM tool calling (7 tools) |
| `src/memory/modules/semantic_mapper_module.py` | SceneGraph 鈫?RoomObjectKG + TopologySemGraph |
| `src/memory/modules/vector_memory_module.py` | CLIP + ChromaDB vector search |
| `src/drivers/teleop_module.py` | WebSocket joystick + camera stream |
| `src/slam/slam_module.py` | SLAM managed mode (fastlio2/pointlio/localizer) |
| `src/slam/slam_bridge_module.py` | ROS2 SLAM bridge mode (map鈫抩dom TF transform point) |
| `src/gateway/gateway_module.py` | FastAPI HTTP/WS/SSE + drift watchdog + save hooks |
| `src/nav/services/nav_services/dynamic_filter.py` | DUFOMap wrapper (subprocess repack/run/backup) |
| `src/nav/services/nav_services/map_manager_module.py` | Save pipeline: PGO 鈫?DUFOMap 鈫?tomogram 鈫?occupancy |
| `scripts/lingtu` | **Unified Operations CLI** (status/watch/map/nav/svc/log/health) |
| `scripts/build_dufomap.sh` | Idempotent aarch64 build of DUFOMap (apt + patch + cmake) |
| `scripts/dufomap_offline_test.py` | Standalone validator: run DUFOMap on existing map, print stats |
| `config/robot_config.yaml` | Robot physical parameters (single source of truth) |
| `config/dufomap.toml` | Lingtu-tuned DUFOMap config (Livox Mid-360 thresholds) |
| `docs/05-specialized/dynamic_obstacle_removal.md` | DUFOMap Phase 2 design + roadmap |
| `sim/engine/README.md` | Simulation platform core 鈥?SimEngine, MuJoCo, bridge, scenarios, worlds |

## Build and Test Commands

```bash
# Framework tests (primary, no ROS2 needed)
python -m pytest src/core/tests/ -q                    # 1226 tests, ~5s

# Simulation tests (MuJoCo/Gazebo contract validation, no robot needed)
python -m pytest sim/tests/ -q                         # ~20 scenario-level tests

# C++ nav_core tests (standalone, no ROS2)
cd src/nav/core && mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release && make -j$(nproc)
./test_benchmark                                       # 12 benchmarks
./test_local_planner_core                              # 30 tests
./test_path_follower_core                              # 15 tests
# ... 7 test suites, 96 tests total

# ROS2 build (for C++ nodes on S100P only)
source /opt/ros/humble/setup.bash
make build                                              # colcon release build
```

## C++ Performance (nav_core)

`src/nav/core/` 鏄?header-only C++ 绠楁硶搴擄紝閫氳繃 nanobind 鏆撮湶缁?Python銆俛arch64 閮ㄧ讲鏃舵€ц兘鍏抽敭銆?

### 鍔犻€熷簱

| Library | Version | Purpose |
|---------|---------|---------|
| xsimd | 13.0.0 | 渚挎惡 SIMD (ARM NEON / x86 AVX 鑷姩鍒囨崲) |
| taskflow | 3.8.0 | 浠诲姟骞惰 (澶囩敤锛屽綋鍓嶇敤 OpenMP) |
| OpenMP | 鈥?| 骞惰 for (terrain + scoring) |

### 鍏抽敭浼樺寲

| 浼樺寲 | 鏂囦欢 | 鏁堟灉 |
|------|------|------|
| SoA 鍐呭瓨甯冨眬 | local_planner_full.hpp | SIMD 鍙嬪ソ锛屾秷闄?stride-4 璁垮瓨 |
| CSR 绋€鐤忔牸寮?| local_planner_full.hpp | cache 杩炵画锛屾秷闄ゆ寚閽堣拷韪?|
| scorePathFast LUT | local_planner_core.hpp | **2.08x** (pow025 鏌ヨ〃鏇夸唬 sqrt(sqrt)) |
| OpenMP 骞惰璇勫垎 | local_planner_full.hpp | 36 鏃嬭浆鏂瑰悜骞惰 |
| terrain 骞惰鍖?| terrain_core.hpp | 2601 voxel nth_element 骞惰 |
| SIMD 鎵归噺鏃嬭浆 | simd_accel.hpp | rotateCloud + distSqBatch |
| LTO + fast-math | CMakeLists.txt | 璺ㄥ嚱鏁板唴鑱?+ 鏀炬澗娴偣 |

CMakeLists.txt 纭繚 ROS2 ament_cmake 鍜?standalone 涓や釜璺緞閮藉惎鐢?xsimd + OpenMP + LTO銆?

## Critical Files 鈥?Do Not Break

- `src/core/module.py` 鈥?Module base class (all modules depend on it)
- `src/core/blueprint.py` 鈥?Blueprint + autoconnect (system assembly)
- `src/core/stream.py` 鈥?In[T]/Out[T] ports (data flow backbone)
- `src/core/registry.py` 鈥?Plugin registry (all backends depend on it)
- `src/core/utils/` 鈥?Cross-layer utilities (18+ files import from here)
- `src/semantic/perception/semantic_perception/.../instance_tracker.py` 鈥?Scene graph builder
- `src/semantic/planner/.../goal_resolver.py` 鈥?5-level resolution chain
- `config/robot_config.yaml` 鈥?Robot physical parameters

## Module Dependency Rules

```
All Modules 鈹€鈹€鈫?core/ (Module, In/Out, Registry, utils, msgs)
                 鈫?only legal dependency direction

nav/          does NOT import semantic/, drivers/, gateway/
semantic/     does NOT import nav/, drivers/, gateway/
drivers/      does NOT import nav/, semantic/ (lazy import in blueprints only)
gateway/      does NOT import nav/, semantic/, drivers/
```

Planner backends resolved via `core.registry.get("planner_backend", name)`, not direct import.

## Semantic Navigation

### 5-Level Goal Resolution Chain

```
Instruction: "鍘讳笂娆℃斁鑳屽寘鐨勫湴鏂?
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
- Near (< 3m): `cmd_vel 鈫?CmdVelMux 鈫?Driver (PD servo, bypasses planner)`

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

## SLAM / Localization

| Mode | slam_profile | Backend | Use Case |
|------|-------------|---------|----------|
| Mapping | `fastlio2` | SLAMModule 鈫?C++ Fast-LIO2 | First visit, build map |
| Localization | `localizer` | SLAMModule 鈫?Fast-LIO2 + ICP Localizer | Navigate with pre-built map |
| Bridge | `bridge` | SlamBridgeModule 鈫?ROS2 subscriber | External SLAM (systemd) |
| None | `none` | 鈥?| stub/dev mode |

Localizer requires Fast-LIO2 companion (provides `/cloud_registered` + `/Odometry`).
SLAM odometry is explicitly wired to NavigationModule (priority over driver dead-reckoning).

## REPL Commands

```
Navigation:  go/navigate <target> | stop | cancel | status
Map:         map list | save <name> | use <name> | build <name> | delete <name>
Semantic:    smap status | rooms | save | load <dir> | query <text>
Vector:      vmem query <text> | vmem stats
Agent:       agent <multi-step instruction>
Teleop:      teleop status | teleop release
Monitor:     health | watch <port> | module <name> | connections | log | config
```

## MCP Server (AI Agent Control)

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
# Connect Claude Code to the robot
claude mcp add --transport http lingtu http://192.168.66.190:8090/mcp
```

## Teleop (Remote Control)

WebSocket joystick at `ws://<robot>:5050/ws/teleop`:
- Phone/browser sends `{"type": "joy", "lx": 0.5, "ly": 0.0, "az": -0.3}`
- GatewayModule forwards raw WS joy messages to TeleopModule via `joy_input` port
- TeleopModule manages all teleop state (active/idle/release) and joy scaling
- TeleopModule publishes `teleop_active: Out[bool]` for NavigationModule pause/resume
- cmd_vel goes through CmdVelMux (priority-based arbitration), not directly to driver
- 3s idle auto-release runs in TeleopModule's background thread
- Robot streams JPEG camera frames back

## cmd_vel Priority Arbitration (CmdVelMux)

All cmd_vel sources are routed through CmdVelMux (L0) for priority-based arbitration:

| Source | Priority | Timeout |
|--------|----------|---------|
| Teleop (joystick) | 100 | 0.5s |
| VisualServo (PD tracking) | 80 | 0.5s |
| Recovery (stuck backup) | 60 | 0.5s |
| PathFollower (autonomy) | 40 | 0.5s |

Highest-priority active source wins. A source is "active" if it published within 0.5s.
When a source times out, the mux falls through to the next lower-priority source.

## Explicit Wires (Cross-Stack, in `full_stack.py`)

```python
# Safety 鈫?all actuators
bp.wire("SafetyRingModule", "stop_cmd", driver_name, "stop_signal")
bp.wire("SafetyRingModule", "stop_cmd", "NavigationModule", "stop_signal")

# SLAM odometry priority
bp.wire(slam_module_name, "odometry", "NavigationModule", "odometry")

# Autonomy chain (when enable_native=True)
bp.wire("NavigationModule", "waypoint", "LocalPlannerModule", "waypoint")
bp.wire("TerrainModule", "terrain_map", "LocalPlannerModule", "terrain_map")
bp.wire("LocalPlannerModule", "local_path", "PathFollowerModule", "local_path")

# Visual servo dual channel
bp.wire("SemanticPlannerModule", "servo_target", "VisualServoModule", "servo_target")
bp.wire("VisualServoModule", "goal_pose", "NavigationModule", "goal_pose")

# CmdVelMux 鈥?priority-based velocity arbitration
bp.wire("TeleopModule",      "cmd_vel",          "CmdVelMux", "teleop_cmd_vel")
bp.wire("VisualServoModule", "cmd_vel",          "CmdVelMux", "visual_servo_cmd_vel")
bp.wire("NavigationModule",  "recovery_cmd_vel", "CmdVelMux", "recovery_cmd_vel")
bp.wire("PathFollowerModule", "cmd_vel",         "CmdVelMux", "path_follower_cmd_vel")
bp.wire("CmdVelMux", "driver_cmd_vel", driver_name, "cmd_vel")

# Teleop active 鈫?Navigation pause/resume
bp.wire("TeleopModule", "teleop_active", "NavigationModule", "teleop_active")
```

## S100P Deployment

- **SSH**: `ssh sunrise@192.168.66.190`
- **Nav code**: `~/data/SLAM/navigation/` (symlink 鈫?`~/data/inovxio/lingtu/`)
- **Nav deploy**: `/opt/lingtu/nav/`
- **CycloneDDS**: Built from source at `~/cyclonedds/install/` (Unitree approach)
- **Python**: 3.10.12, cyclonedds==0.10.5
- **DUFOMap binary**: `~/src/dufomap/build/dufomap_run` (see `scripts/build_dufomap.sh`)
- **Default map dir**: `~/data/nova/maps/` (was `~/data/inovxio/data/maps/` 鈥?migrated)

## Operations CLI (`scripts/lingtu`)

鍗曚竴鍏ュ彛 CLI 鍙栦唬澶氫釜闆舵暎鑴氭湰鍜?curl / systemctl銆傚缓璁湰鏈?`alias`:

```bash
alias lingtu='ssh sunrise@192.168.66.190 "bash ~/data/SLAM/navigation/scripts/lingtu"'
alias lingwatch='ssh -t sunrise@192.168.66.190 "bash ~/data/SLAM/navigation/scripts/lingtu watch"'
```

| 瀛愬懡浠?| 鐢ㄩ€?|
|---|---|
| `lingtu status` | 涓€灞?8 鍖虹姸鎬?(session / SLAM / robot / mission / path / ctrl / map / log) |
| `lingtu watch` | `watch -c -n 1` 鎸佺画鍒锋柊 鈥?寤哄浘/瀵艰埅鏃跺壇灞忓紑杩欎釜 |
| `lingtu map start\|save <name>\|end\|list` | 寤哄浘 session 鐢熷懡鍛ㄦ湡 |
| `lingtu nav start <map>\|stop\|goal X Y [YAW]` | 瀵艰埅 session + 鍙戠洰鏍?|
| `lingtu svc status\|restart [slam\|lingtu\|all]` | systemctl wrapper |
| `lingtu log drift\|dufomap\|error\|tail\|all` | journalctl 杩囨护鍣?|
| `lingtu health` | REST `/api/v1/health` 鍘熸牱 dump |

## Dynamic Obstacle Removal (Phase 1 + 2)

寤哄浘杩囩▼ + 淇濆瓨鏃跺弻閲嶈繃婊?娑堥櫎浜?鐗╄蛋杩囩暀涓嬬殑鎷栧熬銆?

- **Phase 1** `voxel hit-count voting` (`gateway_module.py:_on_map_cloud` mapping 鍒嗘敮)
  - 姣忓抚 map_cloud 鏉? 姣忎釜 voxel hit_count +1
  - 鍙?SSE 鍓嶈繃婊?`hit < LINGTU_MAP_MIN_HITS` (榛樿 3) 鐨?voxel
  - **鍙奖鍝?Web 瀹炴椂瑙嗗浘**
- **Phase 2** `DUFOMap (ray-casting + void detection)`
  - 淇濆瓨鍦板浘鏃跺湪 PGO 涔嬪悗 tomogram 涔嬪墠璺戜竴娆?offline filter
  - 璇?`<map>/patches/*.pcd` + `poses.txt`, 鍐欏洖骞插噣 `map.pcd`, 澶囦唤涓?`map.pcd.predufo`
  - **褰卞搷纾佺洏 PCD** 鈫?瀵艰埅鏃跺姞杞藉氨鏄共鍑€搴曞浘
  - 璺戠殑鏄?C++ binary `~/src/dufomap/build/dufomap_run` + `config/dufomap.toml` (Lingtu 璋冨弬)
  - env `LINGTU_SAVE_DYNAMIC_FILTER=0` 鍏抽棴

璇﹁ `docs/05-specialized/dynamic_obstacle_removal.md`銆?

## SLAM Drift Watchdog

Fast-LIO2 IEKF 闀挎椂闂撮潤缃細鍗忔柟宸彂鏁?xy 椋樺埌 10鹿虏 绫炽€侴ateway 鍚庡彴绾跨▼
(`_drift_watchdog_loop`) 姣?60s 妫€鏌?odom,瓒呴槇鍊艰嚜鍔?

1. `svc.stop("slam","slam_pgo","localizer")` 鈥?缁堢粨椋炴帀鐨?IEKF
2. 娓?`self._odom` 缂撳瓨
3. SSE 鎺?`slam_drift` 浜嬩欢
4. 鎸夊綋鍓?session mode `svc.ensure(...)` 閲嶆媺鏈嶅姟
5. 300s 鍐峰嵈闃叉姈

Env: `LINGTU_DRIFT_WATCHDOG=0` (鍏? / `_INTERVAL` / `_XY_LIMIT` / `_V_LIMIT` / `_COOLDOWN`銆?

## Sensor Calibration (`calibration/`)

鍑哄巶鏍囧畾宸ュ叿绠憋紝瑕嗙洊 S100P 鍏ㄩ儴浼犳劅鍣ㄣ€傛爣瀹氱粨鏋滅粺涓€鍐欏叆 `config/robot_config.yaml`銆?

### 鏍囧畾娴佺▼ (SOP)

| Step | 鍐呭 | 宸ュ叿 | 鏃堕棿 |
|------|------|------|------|
| 1 | 鐩告満鍐呭弬 (妫嬬洏鏍?9脳6) | `calibration/camera/calibrate_intrinsic.py` (OpenCV) | ~5 min |
| 2 | IMU 鍣０ (Allan Variance) | `calibration/imu/allan_variance_ros2/` (Autoliv) | ~2-3 hr |
| 3 | LiDAR-IMU 澶栧弬 (8 瀛楄繍鍔? | `calibration/lidar_imu/LiDAR_IMU_Init/` (HKU-MARS) | ~2 min |
| 4 | 鐩告満-LiDAR 澶栧弬 (target-less) | `calibration/camera_lidar/direct_visual_lidar_calibration/` (koide3) | ~10 min |
| 5 | 涓€閿簲鐢?| `calibration/apply_calibration.py` 鈫?robot_config.yaml + SLAM configs | 绉掔骇 |
| 6 | 涓€閿獙璇?| `calibration/verify.py` (鐒﹁窛/鐣稿彉/鏃嬭浆/鎶曞奖閾?sanity check) | 绉掔骇 |

### 鏍囧畾鍙傛暟杈撳嚭

```yaml
# config/robot_config.yaml
camera:
  fx, fy, cx, cy              # Step 1 鐩告満鍐呭弬
  dist_k1..k3, dist_p1..p2    # Step 1 鐣稿彉绯绘暟
  position_x/y/z              # Step 4 鐩告満-LiDAR 澶栧弬
  roll, pitch, yaw            # Step 4 鏃嬭浆

lidar:
  offset_x/y/z                # Step 3 LiDAR-IMU 澶栧弬 (t_il)
  roll, pitch, yaw            # Step 3 鏃嬭浆 (r_il)
```

`apply_calibration.py` 鍚屾椂鍚屾鍒?`src/slam/fastlio2/config/lio.yaml` 鍜?`config/pointlio.yaml` (na, ng, nba, nbg, r_il, t_il)銆?

### 杩愯鏃舵牎楠?

`src/core/utils/calibration_check.py` 鍦?`full_stack_blueprint()` 鍚姩鏃舵牎楠屾爣瀹氬弬鏁帮細
- FAIL 绾?(濡傜劍璺濅负 0銆佹棆杞煩闃甸潪姝ｄ氦) 鈫?闃绘鍚姩
- WARN 绾?(濡傜暩鍙樼郴鏁板叏闆? 鈫?鏃ュ織璀﹀憡锛屼笉闃绘柇

### 鍏抽敭鏂囦欢

| File | Purpose |
|------|---------|
| `calibration/README.md` | 瀹屾暣 SOP 鏂囨。 (鍚懡浠よ绀轰緥) |
| `calibration/apply_calibration.py` | 灏?4 绫绘爣瀹氱粨鏋滃啓鍏?robot_config + SLAM 閰嶇疆 |
| `calibration/verify.py` | 涓€閿獙璇? 鍙傛暟鑼冨洿 + 鎶曞奖閾?+ 璺ㄩ厤缃竴鑷存€?|
| `calibration/camera/calibrate_intrinsic.py` | 鐩告満鍐呭弬 (capture/calibrate/verify 涓夊悎涓€) |
| `calibration/lidar_imu/ros2_adapter/` | ROS2鈫扲OS1 bridge 閫傞厤灞?(rosbag 鍥炴斁) |
| `src/core/utils/calibration_check.py` | 杩愯鏃舵爣瀹氬弬鏁版牎楠?(鍚姩鏃惰皟鐢? |
| `config/robot_config.yaml` | 鏍囧畾鍙傛暟鏈€缁堝綊瀹?(single source of truth) |

## Sensor Calibration (`calibration/`)

鍑哄巶鏍囧畾宸ュ叿绠憋紝瑕嗙洊 S100P 鍏ㄩ儴浼犳劅鍣ㄣ€傛爣瀹氱粨鏋滅粺涓€鍐欏叆 `config/robot_config.yaml`銆?

### 鏍囧畾娴佺▼ (SOP)

| Step | 鍐呭 | 宸ュ叿 | 鏃堕棿 |
|------|------|------|------|
| 1 | 鐩告満鍐呭弬 (妫嬬洏鏍?9脳6) | `calibration/camera/calibrate_intrinsic.py` (OpenCV) | ~5 min |
| 2 | IMU 鍣０ (Allan Variance) | `calibration/imu/allan_variance_ros2/` (Autoliv) | ~2-3 hr |
| 3 | LiDAR-IMU 澶栧弬 (8 瀛楄繍鍔? | `calibration/lidar_imu/LiDAR_IMU_Init/` (HKU-MARS) | ~2 min |
| 4 | 鐩告満-LiDAR 澶栧弬 (target-less) | `calibration/camera_lidar/direct_visual_lidar_calibration/` (koide3) | ~10 min |
| 5 | 涓€閿簲鐢?| `calibration/apply_calibration.py` 鈫?robot_config.yaml + SLAM configs | 绉掔骇 |
| 6 | 涓€閿獙璇?| `calibration/verify.py` (鐒﹁窛/鐣稿彉/鏃嬭浆/鎶曞奖閾?sanity check) | 绉掔骇 |

### 鏍囧畾鍙傛暟杈撳嚭

```yaml
# config/robot_config.yaml
camera:
  fx, fy, cx, cy              # Step 1 鐩告満鍐呭弬
  dist_k1..k3, dist_p1..p2    # Step 1 鐣稿彉绯绘暟
  position_x/y/z              # Step 4 鐩告満-LiDAR 澶栧弬
  roll, pitch, yaw            # Step 4 鏃嬭浆

lidar:
  offset_x/y/z                # Step 3 LiDAR-IMU 澶栧弬 (t_il)
  roll, pitch, yaw            # Step 3 鏃嬭浆 (r_il)
```

`apply_calibration.py` 鍚屾椂鍚屾鍒?`src/slam/fastlio2/config/lio.yaml` 鍜?`config/pointlio.yaml` (na, ng, nba, nbg, r_il, t_il)銆?

### 杩愯鏃舵牎楠?

`src/core/utils/calibration_check.py` 鍦?`full_stack_blueprint()` 鍚姩鏃舵牎楠屾爣瀹氬弬鏁帮細
- FAIL 绾?(濡傜劍璺濅负 0銆佹棆杞煩闃甸潪姝ｄ氦) 鈫?闃绘鍚姩
- WARN 绾?(濡傜暩鍙樼郴鏁板叏闆? 鈫?鏃ュ織璀﹀憡锛屼笉闃绘柇

### 鍏抽敭鏂囦欢

| File | Purpose |
|------|---------|
| `calibration/README.md` | 瀹屾暣 SOP 鏂囨。 (鍚懡浠よ绀轰緥) |
| `calibration/apply_calibration.py` | 灏?4 绫绘爣瀹氱粨鏋滃啓鍏?robot_config + SLAM 閰嶇疆 |
| `calibration/verify.py` | 涓€閿獙璇? 鍙傛暟鑼冨洿 + 鎶曞奖閾?+ 璺ㄩ厤缃竴鑷存€?|
| `calibration/camera/calibrate_intrinsic.py` | 鐩告満鍐呭弬 (capture/calibrate/verify 涓夊悎涓€) |
| `calibration/lidar_imu/ros2_adapter/` | ROS2鈫扲OS1 bridge 閫傞厤灞?(rosbag 鍥炴斁) |
| `src/core/utils/calibration_check.py` | 杩愯鏃舵爣瀹氬弬鏁版牎楠?(鍚姩鏃惰皟鐢? |
| `config/robot_config.yaml` | 鏍囧畾鍙傛暟鏈€缁堝綊瀹?(single source of truth) |

## Code Style

- **C++**: Google style (`.clang-format`, 2-space indent, 100 col)
- **Python**: English comments in new code. Chinese comments exist in legacy code.
- **Framework**: All Modules use `core.Module` base with In[T]/Out[T] type hints
- **No ROS2 in Modules**: rclpy only in Bridge modules and C++ NativeModule launchers

## Known Limitations

- Fast Path uses rule-based matching (not learned policies)
- S100P has no CUDA 鈥?Open3D GPU features unavailable, use C++ terrain_analysis instead
- Kimi API key may expire 鈥?Slow Path unavailable without valid LLM key
- ChromaDB optional 鈥?VectorMemoryModule falls back to numpy brute-force search
- Framework tests (1226) are mock-based 鈥?real hardware integration tests need S100P
- C++ test_validation 6 tests fail under `-ffast-math` (NaN/Inf IEEE compliance) 鈥?expected
