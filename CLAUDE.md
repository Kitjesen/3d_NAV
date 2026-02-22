# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

MapPilot (灵途) is an autonomous navigation system for quadruped robots in outdoor/off-road environments.

- **Platform**: Jetson Orin NX 16GB | ROS2 Humble | Ubuntu 22.04
- **Languages**: Python (semantic modules), C++ (SLAM/planning), Dart (Flutter client)
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

# Test
make test             # All colcon tests
make test-integration # bash tests/integration/run_all.sh
make benchmark        # Performance benchmarks

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
make format           # clang-format on src/**/*.cpp/hpp
make lint             # clang-tidy
make health           # 12-point system health check (~30s)

# Docker
make docker-build     # Build Docker image
make docker-run       # docker-compose up -d
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

| Package | Role |
|---|---|
| `slam/` | Fast-LIO2 frontend, PGO loop closure, Localizer (ICP relocalization) |
| `base_autonomy/` | Local planner + terrain analysis (ground estimation, traversability) |
| `global_planning/` | PCT Planner (tomography-based global path planning) |
| `remote_monitoring/` | gRPC gateway (port 50051) — telemetry, control, OTA |
| `drivers/` | Robot hardware interface |
| `semantic_perception/` | YOLO-World + CLIP + ConceptGraphs scene graph (3,305 LOC) |
| `semantic_planner/` | VLN planner with Fast-Slow dual-process (5,169 LOC) |
| `semantic_common/` | Shared types/utilities for semantic modules |
| `vla_nav/` | VLA navigation experiments |
| `VLM/` | Vision-language model integrations (includes yolov5) |
| `ota_daemon/` | OTA update daemon |
| `utils/` | Shared utilities |

## Semantic Navigation

### Fast-Slow Dual-Process (`goal_resolver.py`)

**Fast Path** (System 1, ~0.17ms): Direct scene graph matching — keyword + spatial reasoning, confidence fusion (label 35%, CLIP 35%, detector 15%, spatial 15%). Target: >70% hit rate.

**Slow Path** (System 2, ~2s): LLM reasoning with ESCA selective grounding — filters 200 objects → ~15 objects (92.5% token reduction), then calls GPT-4o-mini/Claude/Qwen.

### Key Files in `src/semantic_planner/semantic_planner/`

- `goal_resolver.py` — Fast-Slow core logic (720 LOC)
- `planner_node.py` — ROS2 node (940 LOC)
- `frontier_scorer.py` — MTU3D frontier grounding potential
- `topological_memory.py` — Spatial memory graph
- `task_decomposer.py` — SayCan-style task decomposition
- `sgnav_reasoner.py` — SGNav scene graph reasoner
- `implicit_fsm_policy.py` — Implicit FSM navigation policy
- `exploration_strategy.py` — Frontier exploration strategy
- `voi_scheduler.py` — Value of information scheduling
- `action_executor.py` — Action primitive execution
- `llm_client.py` — OpenAI/Claude/Qwen API client (353 LOC)
- `chinese_tokenizer.py` — jieba integration (294 LOC)
- `prompt_templates.py` — LLM prompt templates
- `semantic_prior.py` — Semantic priors for navigation
- `api/` — API interfaces

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
export OPENAI_API_KEY="sk-..."       # default
export ANTHROPIC_API_KEY="sk-ant-..." # alternative
export DASHSCOPE_API_KEY="sk-..."    # Qwen, China fallback
```

`config/semantic_planner.yaml` controls `llm.backend` (openai|claude|qwen), `llm.model`, `llm.timeout_sec`, `llm.temperature`.

## Critical Files — Do Not Break

- `src/semantic_planner/semantic_planner/goal_resolver.py` — Core Fast-Slow logic
- `src/semantic_perception/semantic_perception/instance_tracker.py` — Scene graph builder
- `launch/navigation_run.launch.py` — Main navigation launch
- `config/semantic_planner.yaml` — LLM + planner config

## Test Coverage

- `semantic_planner`: 90% (89 tests, `src/semantic_planner/test/`)
- `semantic_perception`: 40% (13 tests, `src/semantic_perception/test/`)
- Root-level test scripts: `test_all_modules.py`, `full_functional_test.py`, `test_full_pipeline.py`, `test_slow_path.py`
- Integration: `tests/integration/`, benchmarks: `tests/benchmark/`

After modifying Fast-Slow logic, run `src/semantic_planner/test/test_fast_slow_benchmark.py` to verify Fast Path hit rate stays >70%.

## Performance Targets

| Component | Target |
|---|---|
| Fast Path response | <200ms, >70% hit rate |
| YOLO-World | 10–15 FPS on Jetson |
| CLIP cache hit | 60–80% |
| Scene graph update | 1–2 Hz |

## ROS2 Topic Contract

If changing topic names, update `docs/02-architecture/TOPIC_CONTRACT.md`. Key topics:
- `/Odometry` — Fast-LIO2 odometry
- `/cloud_reg` — registered point cloud
- `/terrain_map`, `/terrain_map_ext` — terrain analysis outputs
- `/nav/semantic/scene_graph` — ConceptGraphs scene graph
- `/nav/semantic/detections_3d` — 3D object detections

## Known Limitations

- Fast Path uses rule-based matching (not learned policies)
- ESCA filtering uses keyword matching (not trained SGClip)
- System is inspired by VLingNav/ESCA/MTU3D papers but uses simplified engineering implementations
- Real-world Jetson testing still needed (validated in simulation)

## Documentation

- `docs/README.md` — Documentation index
- `docs/02-architecture/ARCHITECTURE.md` — Full system architecture
- `docs/06-semantic-nav/COMPREHENSIVE_STATUS.md` — Semantic nav status
- `docs/ARCHITECTURE.md`, `docs/BUILD_GUIDE.md`, `docs/PARAMETER_TUNING.md`, `docs/TROUBLESHOOTING.md` — top-level guides
- `AGENTS.md` — Detailed ROS2 topic/node map for AI agents
