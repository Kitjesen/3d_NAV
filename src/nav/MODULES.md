# Nav — Navigation Module Index

> Files live under `src/nav/`
> 20+ .py files including subdirectories (excluding tests)

---

## Top-Level (14)

| File | Responsibility |
|------|---------------|
| `__init__.py` | Package init — package marker and description |
| `navigation_module.py` | **NavigationModule** (L5) — global planner dispatch, WaypointTracker, mission FSM, goal safety |
| `global_planner_service.py` | Global planner backend — A*/PCT resolution + `_find_safe_goal` BFS |
| `waypoint_tracker.py` | WaypointTracker — arrival detection, stuck detection, timeout |
| `safety_ring_module.py` | **SafetyRingModule** (L0) — safety reflex, evaluator, dialogue |
| `cmd_vel_mux_module.py` | **CmdVelMux** (L0) — priority-based cmd_vel arbitration (teleop > visual servo > recovery > path follower) |
| `occupancy_grid_module.py` | **OccupancyGridModule** (L2) — 2D occupancy grid from map or live SLAM |
| `esdf_module.py` | **ESDFModule** (L2) — Euclidean Signed Distance Field from occupancy |
| `elevation_map_module.py` | **ElevationMapModule** (L2) — terrain elevation from point cloud |
| `frontier_explorer_module.py` | Frontier-based exploration — unknown region detection |
| `voxel_grid_module.py` | Voxel grid — 3D spatial index for collision checking |
| `traversability_cost_module.py` | Traversability cost — terrain slope/roughness → cost map |
| `traversable_frontier_module.py` | Traversable frontier — reachable frontier detection |
| `plan_safety.py` | Plan safety checks — validate path against current obstacles |

---

## ROS2 Bridges — `ros2_*_bridge_module.py` (3)

| File | Responsibility |
|------|---------------|
| `ros2_grid_bridge_module.py` | ROS2 OccupancyGrid bridge — subscribe/publish nav_msgs/OccupancyGrid |
| `ros2_path_bridge_module.py` | ROS2 Path bridge — publish planned path as nav_msgs/Path |
| `ros2_waypoint_bridge_module.py` | ROS2 waypoint bridge — subscribe to goal poses from external nodes |

---

## Services — `services/` (6)

| File | Responsibility |
|------|---------------|
| `__init__.py` | Package init — package marker |
| `geofence_manager_module.py` | **GeofenceManagerModule** (L0) — virtual boundary enforcement |
| `map_manager_module.py` | **MapManagerModule** — save pipeline: PGO → DUFOMap → tomogram → occupancy |
| `patrol_manager_module.py` | **PatrolManagerModule** — multi-waypoint patrol mission |
| `task_scheduler_module.py` | **TaskSchedulerModule** — timed task execution |
| `frame_contract.py` | Frame contract — coordinate frame conventions and validation |
