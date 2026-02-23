# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

MapPilot (灵途) is an autonomous navigation system for quadruped robots in outdoor/off-road environments.

- **Platform**: Jetson Orin NX 16GB | ROS2 Humble | Ubuntu 22.04
- **Languages**: Python (semantic modules), C++ (SLAM/planning/gRPC), Dart (Flutter client)
- **Version**: See `VERSION` file (current: 1.5.0)
- **Dual-Board**: Nav Board (navigation) + Dog Board (motion control, RL policy)

## Build and Test Commands

```bash
# Source ROS2 first (required for all commands)
source /opt/ros/humble/setup.bash

# Build
make build            # Release build (colcon)
make build-debug      # Debug build
make clean            # Remove build/ install/ log/

# Build a specific package
colcon build --packages-select semantic_planner

# Full build (PCT core + ROS2 + OTA daemon)
./build_all.sh              # All three components
./build_all.sh --pct-only   # PCT Planner C++ core only
./build_all.sh --ros-only   # ROS2 packages only
./build_all.sh --ota-only   # OTA daemon only

# Test
make test             # All colcon tests
make test-integration # bash tests/integration/run_all.sh
make benchmark        # Performance benchmarks (tests/benchmark/run_all.sh)

# Run a single test file directly
cd src/semantic_planner && python -m pytest test/test_goal_resolver.py -v

# System launch (scripts in repo root)
./mapping.sh          # Mapping mode (SLAM + sensors, manual drive)
./save_map.sh         # Save current map after mapping
./planning.sh         # Navigation mode (loads existing map)

# Or via make
make mapping          # ros2 launch launch/navigation_bringup.launch.py
make navigation       # ros2 launch launch/navigation_run.launch.py

# Code quality
make format           # clang-format on src/**/*.cpp/hpp (Google style, 100 col)
make lint             # clang-tidy (bugprone, performance, modernize checks)
make health           # 12-point system health check (~30s)
make check            # Composite: build + test + health

# Other
make install          # Install systemd services
make sync-version     # Sync VERSION across packages
make docs             # Generate Doxygen documentation

# Docker
make docker-build     # Build Docker image
make docker-run       # docker-compose up -d
make docker-stop      # docker-compose down
```

## Architecture

```
Livox LiDAR → SLAM (Fast-LIO2 + PGO) → Terrain Analysis → Planning → Dog Board
                        ↓                                       ↓
                  Localizer (ICP)                    gRPC Gateway ← Flutter App
                        ↓
           Semantic Layer (optional):
           semantic_perception → scene graph → semantic_planner
```

### Source Packages (`src/`)

| Package | Sub-packages | Role |
|---|---|---|
| `slam/` | fastlio2, pgo, localizer, hba, interface | Fast-LIO2 frontend, PGO loop closure, ICP relocalization (38 C++/Py files) |
| `base_autonomy/` | local_planner, terrain_analysis, terrain_analysis_ext, sensor_scan_generation, visualization_tools | Local planner + terrain analysis (ground estimation, traversability) |
| `global_planning/` | PCT_planner, pct_adapters, NeuPAN | PCT Planner (tomography-based global path planning, pybind11 wrapper) |
| `remote_monitoring/` | — | gRPC gateway (port 50051) — telemetry, control, OTA, WebRTC bridge (C++, 44 files) |
| `drivers/` | livox_ros_driver2, robot_driver | Livox LiDAR driver + quadruped robot serial interface |
| `semantic_perception/` | — | YOLO-World + CLIP + ConceptGraphs scene graph (22,345 LOC, 75 Py files) |
| `semantic_planner/` | — | VLN planner with Fast-Slow dual-process (9,645 LOC, 29 Py files) |
| `vla_nav/` | model, training, ros2, deploy | VLA navigation experiments (RL trainer, Habitat collector, SFT) |
| `VLM/` | yolov5 | Vision-language model integrations (YOLOv5 fork) |
| `ota_daemon/` | — | OTA update daemon (C++, independent CMake build) |
| `utils/` | OrbbecSDK_ROS2 (submodule), serial | Orbbec RGB-D camera driver + serial utilities |
| `robot_proto/` | — | Protobuf definitions (git submodule, shared with Flutter client) |

### Client App

`client/flutter_monitor/` — Flutter cross-platform app (Android, Windows, iOS) for remote robot control and monitoring. CI builds APK, Windows zip, and iOS (unsigned). Communicates via gRPC on port 50051.

## Semantic Navigation

### Fast-Slow Dual-Process (`goal_resolver.py`)

**Fast Path** (System 1, ~0.17ms): Direct scene graph matching — keyword + spatial reasoning, confidence fusion (label 35%, CLIP 35%, detector 15%, spatial 15%). Target: >70% hit rate, threshold 0.75.

**Slow Path** (System 2, ~2s): LLM reasoning with ESCA selective grounding — filters 200 objects → ~15 objects (92.5% token reduction), then calls LLM.

### Key Files in `src/semantic_planner/semantic_planner/`

- `goal_resolver.py` — Fast-Slow core logic (1,783 LOC)
- `planner_node.py` — ROS2 node (2,415 LOC)
- `frontier_scorer.py` — MTU3D frontier grounding potential
- `topological_memory.py` — Spatial memory graph
- `task_decomposer.py` — SayCan-style task decomposition
- `sgnav_reasoner.py` — SGNav scene graph reasoner
- `implicit_fsm_policy.py` — Implicit FSM navigation policy (LOVON-style)
- `exploration_strategy.py` — Frontier exploration strategy
- `voi_scheduler.py` — Value of information scheduling
- `action_executor.py` — Action primitive execution
- `llm_client.py` — Multi-backend LLM client (575 LOC)
- `chinese_tokenizer.py` — jieba integration (335 LOC)
- `prompt_templates.py` — LLM prompt templates
- `semantic_prior.py` — Semantic priors for navigation

### Key Files in `src/semantic_perception/semantic_perception/`

- `perception_node.py` — ROS2 perception node
- `instance_tracker.py` — Scene graph builder (critical)
- `yolo_world_detector.py` — YOLO-World object detection
- `clip_encoder.py` — CLIP feature encoder
- `scg_builder.py` / `scg_builder_optimized.py` — Scene graph construction
- `topology_graph.py` — Topological graph from scene data
- `laplacian_filter.py` — Point cloud filtering
- `belief_network.py` — Bayesian belief network
- `knowledge_graph.py` — Knowledge graph
- `hybrid_planner.py` — Hybrid semantic+geometric planner
- `scg_path_planner.py` — Scene graph-based path planning

### Scene Graph Format (ROS2 topic `/nav/semantic/scene_graph`)

```json
{
  "objects": [{"id": "obj_123", "label": "chair", "clip_feature": [/* 512-dim */],
               "position": [x, y, z], "confidence": 0.85}],
  "relations": [{"subject_id": "obj_123", "predicate": "near", "object_id": "obj_456"}],
  "regions": [{"name": "living_room", "object_ids": ["obj_123"]}]
}
```

### LLM Configuration

```bash
export MOONSHOT_API_KEY="..."         # Kimi (default, China-direct)
export OPENAI_API_KEY="sk-..."        # OpenAI
export ANTHROPIC_API_KEY="sk-ant-..." # Claude
export DASHSCOPE_API_KEY="sk-..."     # Qwen (China fallback)
```

`config/semantic_planner.yaml` controls:
- `llm.backend` (kimi|openai|claude|qwen) — default: kimi
- `llm.model` — default: kimi-k2.5
- `llm.timeout_sec`, `llm.temperature`
- `llm_fallback.*` — automatic fallback to secondary LLM (default: qwen-turbo)

## Configuration Files

| File | Purpose |
|---|---|
| `config/semantic_planner.yaml` | LLM backend, goal resolution, exploration, fusion weights, SG-Nav params |
| `config/semantic_perception.yaml` | Perception module configuration |
| `config/robot_config.yaml` | Robot geometry, speed limits, safety params, driver config (single source of truth) |
| `config/topic_contract.yaml` | Standard ROS2 topic names (all `/nav/` prefixed) |
| `config/qos_profiles.yaml` | ROS2 QoS profiles |
| `config/cyclonedds.xml` | CycloneDDS configuration |

## Launch System

```
launch/
├── navigation_bringup.launch.py   # Mapping mode (SLAM + sensors, manual)
├── navigation_run.launch.py       # Navigation mode (loads existing map)
├── _robot_config.py               # Reads config/robot_config.yaml
├── subsystems/                    # Individual subsystem launch files
│   ├── lidar.launch.py
│   ├── slam.launch.py
│   ├── autonomy.launch.py
│   ├── planning.launch.py
│   ├── grpc.launch.py
│   ├── semantic.launch.py
│   └── driver.launch.py
└── profiles/                      # Algorithm-specific profiles (topic remapping)
    ├── slam_fastlio2.launch.py
    ├── slam_stub.launch.py        # For testing without hardware
    ├── localizer_icp.launch.py
    ├── planner_pct.launch.py
    └── planner_stub.launch.py     # For testing without hardware
```

## Critical Files — Do Not Break

- `src/semantic_planner/semantic_planner/goal_resolver.py` — Core Fast-Slow logic
- `src/semantic_perception/semantic_perception/instance_tracker.py` — Scene graph builder
- `launch/navigation_run.launch.py` — Main navigation launch
- `config/semantic_planner.yaml` — LLM + planner config
- `config/robot_config.yaml` — Robot physical parameters (single source of truth)
- `config/topic_contract.yaml` — ROS2 topic interface contract

## Test Structure

### Unit Tests

| Location | Tests | Coverage |
|---|---|---|
| `src/semantic_planner/test/` | 12 test files (goal_resolver, fast_slow_benchmark, action_executor, frontier_scorer, implicit_fsm, sgnav_reasoner, slow_path_llm, task_decomposer, topological_memory, exploration_strategy, fast_resolve, slow_path_real_llm) | ~90% |
| `src/semantic_perception/test/` | 6 test files (clip_encoder, incremental_update, laplacian_filter, parallel_comparison, scg_ros_integration, yolo_world_detector) | ~40% |

### Integration & Benchmark Tests

| Location | Files |
|---|---|
| `tests/integration/` | run_all.sh, test_full_stack.sh, test_grpc_endpoints.py, test_network_failure.py, test_topic_hz.py |
| `tests/benchmark/` | run_all.sh, benchmark_grpc.sh, benchmark_planner.sh, benchmark_slam.sh |
| `tests/` (root) | test_belief_system.py, test_chinese_tokenizer.py, test_goal_resolver.py, test_laplacian_filter.py, test_offline_pipeline.py, test_topology_graph.py |

### Root-Level Test Scripts

- `test_all_modules.py` / `test_all_modules_v2.py` — Comprehensive module tests
- `full_functional_test.py` — Full functional test suite
- `test_full_pipeline.py` — End-to-end pipeline test
- `test_slow_path.py` — Slow path LLM integration test
- `test_habitat_llm.py` — Habitat + LLM integration test

### Key Test Rule

After modifying Fast-Slow logic, run `src/semantic_planner/test/test_fast_slow_benchmark.py` to verify Fast Path hit rate stays >70%.

## Performance Targets

| Component | Target |
|---|---|
| Fast Path response | <200ms, >70% hit rate |
| YOLO-World | 10–15 FPS on Jetson |
| CLIP cache hit | 60–80% |
| Scene graph update | 1–2 Hz |

## ROS2 Topic Contract

All standard topics use `/nav/` prefix. Defined in `config/topic_contract.yaml`. Key topics:

| Topic | Type | Description |
|---|---|---|
| `/nav/odometry` | nav_msgs/Odometry | SLAM odometry |
| `/nav/registered_cloud` | PointCloud2 | Registered point cloud (body frame) |
| `/nav/map_cloud` | PointCloud2 | Map point cloud (world frame) |
| `/nav/terrain_map` | PointCloud2 | Base terrain analysis |
| `/nav/terrain_map_ext` | PointCloud2 | Extended terrain analysis |
| `/nav/global_path` | nav_msgs/Path | Global planned path |
| `/nav/cmd_vel` | TwistStamped | Velocity commands to robot |
| `/nav/goal_pose` | PoseStamped | Navigation goal |
| `/nav/semantic/scene_graph` | String (JSON) | ConceptGraphs scene graph |
| `/nav/semantic/detections_3d` | Detection3DArray | 3D object detections |
| `/nav/semantic/instruction` | String | Natural language navigation instruction |
| `/nav/semantic/resolved_goal` | PoseStamped | Semantically resolved goal |
| `/nav/semantic/status` | String (JSON) | Semantic planner status |

TF frames: `map` → `odom` → `body`

If changing topic names, update `config/topic_contract.yaml` and `docs/02-architecture/TOPIC_CONTRACT.md`.

## Deployment

### Docker

- **Production**: `docker-compose.yml` — `nav-stack` service with host networking, device passthrough (/dev/ttyUSB0, /dev/ttyACM0), 8GB memory limit
- **Development**: `docker-compose.dev.yml` — Adds X11 forwarding (rviz2), ccache, gdb/valgrind, 16GB memory, source volume mount
- **Environment**: `docker/.env.example` — Subsystem toggles (ENABLE_LIDAR, ENABLE_SLAM, etc.), LiDAR network config
- **Process manager**: Supervisord manages nav-lidar → nav-slam → nav-autonomy → nav-planning chain + independent nav-grpc and ota-daemon
- **RMW**: CycloneDDS (rmw_cyclonedds_cpp) in Docker; DDS shared memory disabled (UDP only)
- **Ports**: 50051, 50052 (gRPC)

### Systemd (bare-metal)

7 service files in `systemd/`: nav-lidar, nav-slam, nav-planning, nav-autonomy, nav-grpc, nav-semantic, ota-daemon. User: `sunrise`, restart on failure.

### OTA Updates

- Build: `scripts/ota/build_nav_package.sh`
- Manifest: `scripts/ota/generate_manifest.py` (Ed25519 signed)
- Release via `release-navigation.yml` CI workflow on version tag push

## CI/CD Workflows (`.github/workflows/`)

| Workflow | Trigger | Purpose |
|---|---|---|
| `build-apk.yml` | Push/PR to main (client paths) | Flutter app: analyze, test, build APK/Windows/iOS, auto-release |
| `release-navigation.yml` | Tag `v[0-9]*` or manual | Build ROS2 workspace, test, package OTA, sign manifest, GitHub release |
| `release-models.yml` | Tag `models-*`/`firmware-*` or manual | Release model files (.pt, .onnx, .engine) with LFS support |

## Code Style

- **C++**: Google style via `.clang-format` (2-space indent, 100 col, K&R braces). Bug checks via `.clang-tidy` (bugprone, performance, modernize). `bugprone-use-after-move` and `bugprone-dangling-handle` are warnings-as-errors.
- **Python**: Standard ROS2 Python conventions. Chinese comments are common (bilingual codebase).
- **Protobuf**: Shared definitions in `src/robot_proto/` submodule. Regenerate with `scripts/proto_gen.sh`.

## Git Submodules

| Path | Repository | Purpose |
|---|---|---|
| `src/utils/OrbbecSDK_ROS2` | gitee.com/orbbecdeveloper/OrbbecSDK_ROS2 | Orbbec RGB-D camera ROS2 driver |
| `src/robot_proto` | github.com/Kitjesen/Robot_Proto | Protobuf definitions (shared with Flutter) |

## Utility Scripts (`scripts/`)

| Script | Purpose |
|---|---|
| `install_deps.sh` | Install system dependencies |
| `install_semantic_deps.sh` | Install semantic navigation Python packages |
| `setup_semantic.sh` | Configure semantic system |
| `proto_gen.sh` | Regenerate Protocol Buffer definitions |
| `health_check.sh` | 12-point system health verification |
| `install_services.sh` | Install systemd service files |
| `sync_versions.sh` | Synchronize VERSION across packages |
| `test_services.sh` | Verify systemd services |
| `test_semantic_nav.sh` | Test semantic navigation pipeline |
| `setup_network.sh` | Configure networking |

## Known Limitations

- Fast Path uses rule-based matching (not learned policies)
- ESCA filtering uses keyword matching (not trained SGClip)
- System is inspired by VLingNav/ESCA/MTU3D papers but uses simplified engineering implementations
- Real-world Jetson testing still needed (validated in simulation)

## Documentation

### Top-level

- `docs/README.md` — Documentation index
- `AGENTS.md` — Detailed ROS2 topic/node map and startup sequence for AI agents (bilingual)
- `TEST_PLAN.md` — Test plan overview

### By Topic (`docs/`)

| Directory | Contents |
|---|---|
| `01-getting-started/` | Quick start, build guide, deployment |
| `02-architecture/` | System architecture, algorithm reference, topic contract |
| `03-development/` | API reference, parameter tuning, troubleshooting, refactoring |
| `04-deployment/` | Docker guide, OTA guide |
| `05-specialized/` | WebRTC, gRPC gateway, proto regeneration, communication optimization |
| `06-semantic-nav/` | Semantic nav reports, Fast-Slow implementation, LOVON, task decomposition (28 files) |
| `07-testing/` | Test reports, regression checklists, performance benchmarks, research papers |
| `08-project-management/` | TODO, changelog, gap analysis, delivery |
| `09-paper/` | IEEE paper draft, literature review, experimental results |

### Experiments

`experiments/` — Evaluation runner, Jetson benchmarks, ablation configs, LLM instruction sets, slow path tests.
