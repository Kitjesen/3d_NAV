# Navigation — L5 全局规划与 L0 安全仲裁模块

## 概述

Navigation 模块负责全局路径规划、局部避障规划、路径跟踪、安全监控与速度仲裁，构成 LingTu 自主导航的核心执行链路。

| 层级 | 模块 | 职责 |
|------|------|------|
| L5 | `navigation_module.py` | 全局规划（A*/PCT）+ WaypointTracker + mission FSM |
| L5 | `global_planner_service.py` | A*/PCT 后端调度 + BFS 安全目标搜索 |
| L5 | `waypoint_tracker.py` | 到达检测 + 卡死检测 |
| L5 | `traversability_cost_module.py` | 可通行性代价计算 |
| L2 | `occupancy_grid_module.py` | 占据栅格地图 |
| L2 | `esdf_module.py` | 欧氏符号距离场 |
| L2 | `elevation_map_module.py` | 高程地图 |
| L2 | `voxel_grid_module.py` | 体素网格 |
| L0 | `safety_ring_module.py` | 安全反射 + 评估器 + 对话 |
| L0 | `cmd_vel_mux_module.py` | 优先级速度仲裁（Teleop > VisualServo > Recovery > PathFollower） |
| L0 | `plan_safety.py` | 规划安全性校验 |
| L2 | `frontier_explorer_module.py` | 前沿探索 |
| L2 | `traversable_frontier_module.py` | 可通行前沿筛选 |

## services/

- `map_manager_module.py` — 地图保存管线：PGO -> DUFOMap -> tomogram -> occupancy
- `geofence_manager_module.py` — 电子围栏管理
- `patrol_manager_module.py` — 巡逻任务管理
- `task_scheduler_module.py` — 任务调度
- `frame_contract.py` — 帧协议契约

## core/

C++ `nav_core` 算法库（nanobind 绑定），包含 header-only 的 local_planner、path_follower、terrain 实现，通过 xsimd + OpenMP 加速。

## ROS2 Bridges

- `ros2_path_bridge_module.py` — ROS2 路径话题桥接
- `ros2_grid_bridge_module.py` — ROS2 栅格话题桥接
- `ros2_waypoint_bridge_module.py` — ROS2 航点话题桥接

## 依赖规则

`nav/` 不导入 `semantic/`、`drivers/`、`gateway/`。
