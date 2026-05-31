# Global Planning — A* and PCT Path Planning Index

> Files live under `src/global_planning/`
> Lingtu-owned files listed below (3rdparty libraries like GTSAM, GeographicLib omitted)

---

## Top-Level (1)

| File | Responsibility |
|------|---------------|
| `__init__.py` | Package init |

---

## PCT Adapters — `pct_adapters/` (2)

| File | Responsibility |
|------|---------------|
| `__init__.py` | Package init |
| `global_planner_module.py` | **GlobalPlannerModule** — orchestrates A* (pure Python) and PCT (C++ ele_planner.so) backends via Registry |

---

## PCT Planner — `pct_planner/` (Lingtu-owned files)

| File | Responsibility |
|------|---------------|
| `launch/system_launch.py` | PCT system launch — ROS2 launch file for full multi-session SLAM + planning pipeline |
| `launch/mapping_launch.py` | PCT mapping launch — RTK-GPS initialization, fastlio2, PGO, dynamic filtering |
| `launch/test/planner_only_launch.py` | Planner-only test launch |
| `launch/test/test_planning_launch.py` | Planning system test launch |
| `planner/config/__init__.py` | Planner config package |
| `planner/config/param.py` | PCT planner parameters — cost weights, search radii, optimization tuning |
