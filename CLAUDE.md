# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

MapPilot 3D NAV is a complete autonomous navigation system for quadruped robots in outdoor/off-road environments. The system integrates:

- **Geometric Navigation**: SLAM (FAST-LIO2 + PGO), terrain analysis, path planning (PCT Planner), local planning (base_autonomy)
- **Semantic Navigation**: Vision-language navigation with Fast-Slow dual-process architecture, scene graph reasoning, and LLM-based goal resolution
- **Remote Monitoring**: gRPC gateway with Flutter mobile client for telemetry, control, and OTA updates
- **Dual-Board Architecture**: Nav Board (navigation) + Dog Board (motion control)

**Current Version**: 1.3.2
**Platform**: Jetson Orin NX 16GB | ROS2 Humble | Ubuntu 22.04
**Primary Language**: Python (semantic modules), C++ (SLAM/planning), Dart (Flutter client)

## Build and Test Commands

### Building the Project

```bash
# Full workspace build (Release mode)
make build

# Debug build
make build-debug

# Clean and rebuild
make clean && make build

# Build specific package
colcon build --packages-select semantic_planner semantic_perception
```

### Running Tests

```bash
# Run all tests
make test

# Run integration tests
make test-integration

# Run semantic planner tests specifically
cd src/semantic_planner && python -m pytest test/

# Run semantic perception tests
cd src/semantic_perception && python -m pytest test/

# Run performance benchmarks
make benchmark
```

### System Health and Diagnostics

```bash
# System health check (12 checks, ~30 seconds)
make health

# Check ROS2 topics
ros2 topic list
ros2 topic hz /nav/odometry
ros2 topic echo /nav/semantic/scene_graph --once

# Check running nodes
ros2 node list
ros2 node info /semantic_planner_node
```

### Launching the System

```bash
# Mapping mode (SLAM + sensors)
make mapping
# or: ros2 launch launch/navigation_bringup.launch.py

# Navigation mode (localization + planning + semantic)
make navigation
# or: ros2 launch launch/navigation_run.launch.py

# Launch semantic subsystem only
ros2 launch launch/subsystems/semantic.launch.py
```

## Architecture Overview

### System Layers

```
┌─────────────────────────────────────────────┐
│  Semantic Navigation Layer                  │
│  - semantic_perception: YOLO-World + CLIP  │
│    + ConceptGraphs scene graph              │
│  - semantic_planner: Fast-Slow goal         │
│    resolution + LLM reasoning               │
├─────────────────────────────────────────────┤
│  Geometric Navigation Layer                 │
│  - SLAM: FAST-LIO2 + PGO + Localizer        │
│  - Planning: PCT Planner (global) +         │
│    base_autonomy (local)                    │
│  - Terrain Analysis: ground estimation +    │
│    traversability                           │
├─────────────────────────────────────────────┤
│  Remote Monitoring Layer                    │
│  - gRPC Gateway (port 50051)                │
│  - Flutter Client (Android/iOS/Linux)       │
└─────────────────────────────────────────────┘
```

### Key Directories

```
3d_NAV/
├── src/
│   ├── semantic_perception/     # Vision-language perception (3,305 LOC)
│   │   ├── yolo_world_detector.py    # TensorRT INT8 object detection
│   │   ├── clip_encoder.py           # Multi-scale CLIP features
│   │   ├── instance_tracker.py       # ConceptGraphs scene graph
│   │   └── perception_node.py        # ROS2 node
│   ├── semantic_planner/        # VLN planning (5,169 LOC)
│   │   ├── goal_resolver.py          # Fast-Slow dual-process (720 LOC)
│   │   ├── planner_node.py           # ROS2 node (940 LOC)
│   │   ├── frontier_scorer.py        # MTU3D frontier scoring
│   │   ├── topological_memory.py     # Spatial memory
│   │   ├── task_decomposer.py        # SayCan-style decomposition
│   │   ├── llm_client.py             # OpenAI/Claude/Qwen client
│   │   └── chinese_tokenizer.py      # jieba integration
│   ├── slam/                    # FAST-LIO2, PGO, Localizer
│   ├── base_autonomy/           # Local planner + terrain analysis
│   ├── global_planning/         # PCT Planner (tomography-based)
│   ├── remote_monitoring/       # gRPC Gateway
│   └── drivers/                 # Robot drivers
├── launch/
│   ├── navigation_bringup.launch.py  # Mapping mode
│   ├── navigation_run.launch.py      # Navigation mode
│   └── subsystems/
│       ├── semantic.launch.py        # Semantic subsystem
│       ├── slam.launch.py
│       └── planning.launch.py
├── config/
│   ├── semantic_perception.yaml      # Perception config
│   ├── semantic_planner.yaml         # Planner config (LLM settings)
│   ├── robot_config.yaml
│   └── qos_profiles.yaml
├── tests/
│   ├── benchmark/               # Performance benchmarks
│   ├── integration/             # Integration tests
│   └── e2e/                     # End-to-end tests
├── docs/                        # Documentation (9 categories)
│   ├── 01-getting-started/
│   ├── 02-architecture/
│   ├── 03-development/
│   ├── 06-semantic-nav/         # Semantic navigation docs ⭐
│   └── 07-testing/
├── client/flutter_monitor/      # Flutter mobile app
├── scripts/
│   ├── health_check.sh          # System diagnostics
│   └── ota/                     # OTA deployment tools
└── Makefile                     # Quick commands
```

## Semantic Navigation Implementation

### Fast-Slow Dual-Process Architecture

The semantic planner implements a Fast-Slow dual-process system inspired by VLingNav (2026):

**Fast Path (System 1)**: Direct scene graph matching without LLM calls
- Keyword matching + spatial reasoning
- Multi-source confidence fusion (label 35%, CLIP 35%, detector 15%, spatial 15%)
- Response time: ~0.17ms
- Hit rate: 90% (target: 70%)
- Handles simple instructions like "go to the red chair"

**Slow Path (System 2)**: LLM-based reasoning with ESCA selective grounding
- Filters 200 objects → 15 objects (92.5% token reduction)
- Uses GPT-4o-mini/Claude/Qwen for complex reasoning
- Response time: ~2s
- Handles complex instructions like "go to the room where people usually eat"

**Key Files**:
- `src/semantic_planner/semantic_planner/goal_resolver.py` (720 LOC) - Core Fast-Slow logic
- `src/semantic_planner/semantic_planner/llm_client.py` (353 LOC) - LLM API client
- `src/semantic_planner/semantic_planner/chinese_tokenizer.py` (294 LOC) - jieba integration

### Scene Graph Structure

ConceptGraphs-based scene graph with spatial relationships:

```json
{
  "objects": [
    {
      "id": "obj_123",
      "label": "chair",
      "clip_feature": [512-dim vector],
      "position": [x, y, z],
      "bbox": {...},
      "confidence": 0.85
    }
  ],
  "relations": [
    {"subject_id": "obj_123", "predicate": "near", "object_id": "obj_456"}
  ],
  "regions": [
    {"name": "living_room", "object_ids": ["obj_123", "obj_456"]}
  ]
}
```

### Testing

**Test Coverage**:
- semantic_planner: 90% (89 tests)
- semantic_perception: 40% (13 tests)
- Total: 102 tests, all passing, <2s execution

**Key Test Files**:
- `src/semantic_planner/test/test_fast_slow_benchmark.py` - Performance validation
- `src/semantic_planner/test/test_goal_resolver.py` - Fast-Slow logic
- `src/semantic_planner/test/test_frontier_scorer.py` - MTU3D scoring
- `tests/integration/test_full_stack.sh` - Full system integration

## Configuration

### LLM API Setup

The semantic planner requires LLM API keys for Slow Path reasoning:

```bash
# OpenAI (default)
export OPENAI_API_KEY="sk-..."

# Claude (alternative)
export ANTHROPIC_API_KEY="sk-ant-..."

# Qwen (fallback for China)
export DASHSCOPE_API_KEY="sk-..."
```

Configure in `config/semantic_planner.yaml`:
```yaml
semantic_planner_node:
  ros__parameters:
    llm:
      backend: "openai"  # openai | claude | qwen
      model: "gpt-4o-mini"
      timeout_sec: 10.0
      temperature: 0.2
```

### Python Dependencies

Semantic modules require:
```bash
pip install numpy scipy openai anthropic dashscope jieba
```

For perception (TensorRT):
```bash
# Install on Jetson
pip install tensorrt pycuda
```

## Development Workflow

### Adding New Features

1. **Read existing code first** - Understand the architecture before modifying
2. **Follow existing patterns** - Match the style of surrounding code
3. **Test incrementally** - Run tests after each change
4. **Update documentation** - Keep docs in sync with code changes

### Code Style

- **Python**: Follow PEP 8, use type hints where helpful
- **C++**: Follow ROS2 style guide, use clang-format (`.clang-format` provided)
- **ROS2 Nodes**: Use composition when possible, follow lifecycle patterns
- **Configuration**: Use YAML for parameters, avoid hardcoding

### Common Tasks

**Adding a new semantic reasoning strategy**:
1. Create new file in `src/semantic_planner/semantic_planner/`
2. Implement the strategy class
3. Add tests in `src/semantic_planner/test/`
4. Update `planner_node.py` to integrate
5. Add configuration to `config/semantic_planner.yaml`

**Modifying Fast Path logic**:
1. Edit `goal_resolver.py` `fast_resolve()` method
2. Update confidence fusion weights if needed
3. Run `test_fast_slow_benchmark.py` to verify performance
4. Check that Fast Path hit rate remains >70%

**Adding new object detection model**:
1. Create detector class inheriting from `detector_base.py`
2. Implement `detect()` method returning standardized format
3. Add TensorRT optimization if deploying on Jetson
4. Update `perception_node.py` to support new detector
5. Add configuration to `config/semantic_perception.yaml`

## Important Notes

### Performance Targets

- **Fast Path**: <200ms response, >70% hit rate
- **YOLO-World**: 10-15 FPS on Jetson Orin NX
- **CLIP Encoding**: 60-80% cache hit rate
- **Scene Graph**: Update at 1-2 Hz

### Known Limitations

1. **Semantic modules are engineering implementations**, not end-to-end trained models
2. Fast Path uses rule-based matching, not learned policies
3. ESCA filtering uses keyword matching, not trained SGClip model
4. System is inspired by papers (VLingNav, ESCA, MTU3D) but uses simplified approaches
5. Performance has been validated in simulation but needs real-world Jetson testing

### Critical Files - Do Not Break

- `src/semantic_planner/semantic_planner/goal_resolver.py` - Core Fast-Slow logic
- `src/semantic_perception/semantic_perception/instance_tracker.py` - Scene graph
- `launch/navigation_run.launch.py` - Main launch file
- `config/semantic_planner.yaml` - LLM configuration

### When Making Changes

- Always run `make test` before committing
- Check `make health` to verify system state
- Update relevant documentation in `docs/06-semantic-nav/`
- If modifying Fast-Slow logic, run performance benchmarks
- If changing ROS2 topics, update `docs/02-architecture/TOPIC_CONTRACT.md`

## Documentation

**Primary Documentation**: `docs/README.md` - Central navigation hub

**Key Documents**:
- `docs/01-getting-started/QUICK_START.md` - Quick start guide
- `docs/02-architecture/ARCHITECTURE.md` - System architecture
- `docs/06-semantic-nav/COMPREHENSIVE_STATUS.md` - Semantic nav status
- `docs/06-semantic-nav/IMPLEMENTATION_VERIFICATION.md` - Performance validation
- `docs/07-testing/TEST_REPORT_2026-02-15.md` - Latest test results
- `AGENTS.md` - Detailed system architecture for AI agents

**Paper References** (inspiration, not exact implementations):
- VLingNav (arXiv 2601.08665, 2026) - Fast-Slow dual-process
- ESCA/SGCLIP (NeurIPS 2025) - Selective grounding
- MTU3D (ICCV 2025) - Frontier grounding potential
- ConceptGraphs (ICRA 2024) - Incremental scene graphs
- LOVON (2024) - Quadruped VLN action primitives

## Troubleshooting

**Semantic planner not starting**:
- Check LLM API key: `echo $OPENAI_API_KEY`
- Verify config: `cat config/semantic_planner.yaml`
- Check logs: `ros2 node info /semantic_planner_node`

**Scene graph not publishing**:
- Verify camera topics: `ros2 topic list | grep camera`
- Check perception node: `ros2 node info /semantic_perception_node`
- Test detection: `ros2 topic echo /nav/semantic/detections_3d --once`

**Fast Path hit rate too low**:
- Check scene graph quality: `ros2 topic echo /nav/semantic/scene_graph --once`
- Verify CLIP features are being computed
- Review confidence fusion weights in `goal_resolver.py`

**Build failures**:
- Clean workspace: `make clean`
- Check ROS2 environment: `source /opt/ros/humble/setup.bash`
- Verify dependencies: `rosdep install --from-paths src --ignore-src -r -y`

For more troubleshooting: `docs/03-development/TROUBLESHOOTING.md`
