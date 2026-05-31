# Base Autonomy — C++ Local Planner & Path Follower Index

> Files live under `src/base_autonomy/`
> 8 .py files including subdirectories

---

## Top-Level (2)

| File | Responsibility |
|------|---------------|
| `__init__.py` | Package init — exports autonomy stack entry points |
| `native_factories.py` | C++ native binary factory — builds Launcher for nav_core binaries |

---

## Modules — `modules/` (6)

| File | Responsibility |
|------|---------------|
| `__init__.py` | Package init |
| `autonomy_module.py` | **AutonomyModule** — convenience wrapper: adds Terrain + LocalPlanner + PathFollower at once via `add_autonomy_stack()` |
| `terrain_module.py` | **TerrainModule** (L2) — traversability analysis: In(odometry, map_cloud) → Out(terrain_map) |
| `local_planner_module.py` | **LocalPlannerModule** (L2) — obstacle avoidance: In(terrain_map, waypoint) → Out(local_path), supports `cmu` and `simple` backends |
| `path_follower_module.py` | **PathFollowerModule** (L5) — path tracking control: In(local_path) → Out(cmd_vel), supports `pure_pursuit` and `pid` backends via C++ nanobind |
| `_nav_core_loader.py` | C++ nav_core loader — lazy-loads nanobind extension module for aarch64/S100P |
