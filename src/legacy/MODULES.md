# Legacy — Archived Module Index

> Files live under `src/legacy/`
> Refer to `README.md` for when to use or avoid legacy code.

---

## Thunder — `thunder/` (2)

| File | Responsibility |
|------|---------------|
| `__init__.py` | Package init — package marker |
| `han_dog_bridge.py` | Legacy ROS2 bridge for HanDog gRPC communication |
| `han_dog_bridge.py` | Original Thunder gRPC→ROS2 bridge (superseded by ThunderDriver) |

## Gateway — `gateway/` (2)

| File | Responsibility |
|------|---------------|
| `__init__.py` | Package init — package marker |
| `test_grpc_ros2_bridge.py` | Integration test for gRPC-ROS2 bridge compatibility |
| `test_integration.py` | Gateway integration test scripts |

## Semantic — `semantic/` (2)

| File | Responsibility |
|------|---------------|
| `__init__.py` | Package init — package marker |
| `skill_registry.py` | Old skill registry (superseded by planner/semantic_planner_module.py) |

## PCT Planner — `pct_planner/` (5)

| File | Responsibility |
|------|---------------|
| `__init__.py` | Package init — package marker |
| `global_planner.py` | Legacy global planner implementation |
| `pct_planner_astar.py` | Legacy A* planner with PCT integration |
| `tomography.py` | Legacy tomography processing |
| `visualize_tomogram.py` | Tomogram visualization utilities |
| `fake_localization.py` | Fake localization for testing |

## Scripts — `scripts/`

Shell operations scripts migrated from top-level `scripts/legacy/`.
