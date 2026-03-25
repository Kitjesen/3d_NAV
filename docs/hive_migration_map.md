# Hive 迁移映射 — src/ 完整扫描

> 自动生成, 共 **558** 个文件, **135,559** 行代码

## 统计总览

### 按语言

| 语言 | 文件数 |
|------|--------|
| Python | 262 |
| C++ | 236 |
| Config | 52 |
| Rust | 5 |
| Launch | 3 |

### 按迁移类型

| 迁移类型 | 文件数 |
|----------|--------|
| ProcessModule (C++子进程) | 235 |
| 直接迁移 (纯算法) | 212 |
| 配置迁移 | 51 |
| 需解耦 (含ROS2依赖) | 33 |
| 不迁移 (工具/示例) | 9 |
| 不迁移 (Launch) | 7 |
| 不迁移 (ROS2 壳) | 6 |
| ProcessModule (Rust子进程) | 5 |

---

## 按模块详情

### `common` (6 文件, 684 行)

| 文件 | 语言 | 行数 | ROS2 | 迁移类型 | 关键依赖 |
|------|------|------|------|----------|----------|
| `package.xml` | Config | 20 |  | 配置迁移 |  |
| `__init__.py` | Python | 51 |  | 直接迁移 (纯算法) |  |
| `robustness.py` | Python | 218 |  | 直接迁移 (纯算法) | semantic_common, torch |
| `sanitize.py` | Python | 156 |  | 直接迁移 (纯算法) | semantic_common |
| `validation.py` | Python | 217 |  | 直接迁移 (纯算法) | cv2, numpy, semantic_common |
| `setup.py` | Python | 22 |  | 直接迁移 (纯算法) | setuptools |

### `global_plan/adapters` (6 文件, 1,172 行)

| 文件 | 语言 | 行数 | ROS2 | 迁移类型 | 关键依赖 |
|------|------|------|------|----------|----------|
| `pct_path_adapter.yaml` | Config | 10 |  | 配置迁移 |  |
| `package.xml` | Config | 28 |  | 配置迁移 |  |
| `__init__.py` | Python | 0 |  | 直接迁移 (纯算法) |  |
| `mission_arc.py` | Python | 500 | ✓ | 需解耦 (含ROS2依赖) | geometry_msgs, nav_msgs, rclpy, semantic_common, std_msgs |
| `pct_path_adapter.cpp` | C++ | 359 | ✓ | ProcessModule (C++子进程) |  |
| `pct_path_adapter.py` | Python | 275 | ✓ | 需解耦 (含ROS2依赖) | geometry_msgs, nav_msgs, numpy, rclpy, semantic_common |

### `global_plan/boundary` (6 文件, 1,218 行)

| 文件 | 语言 | 行数 | ROS2 | 迁移类型 | 关键依赖 |
|------|------|------|------|----------|----------|
| `default.yaml` | Config | 4 |  | 配置迁移 |  |
| `graph_extractor.h` | C++ | 307 | ✓ | ProcessModule (C++子进程) |  |
| `intersection.h` | C++ | 74 |  | ProcessModule (C++子进程) |  |
| `point_struct.h` | C++ | 177 |  | ProcessModule (C++子进程) |  |
| `package.xml` | Config | 27 |  | 配置迁移 |  |
| `graph_extractor.cpp` | C++ | 629 | ✓ | ProcessModule (C++子进程) |  |

### `global_plan/config` (2 文件, 33 行)

| 文件 | 语言 | 行数 | ROS2 | 迁移类型 | 关键依赖 |
|------|------|------|------|----------|----------|
| `__init__.py` | Python | 1 |  | 直接迁移 (纯算法) |  |
| `param.py` | Python | 32 |  | 直接迁移 (纯算法) |  |

### `global_plan/far_planner` (31 文件, 9,452 行)

| 文件 | 语言 | 行数 | ROS2 | 迁移类型 | 关键依赖 |
|------|------|------|------|----------|----------|
| `default.yaml` | Config | 58 |  | 配置迁移 |  |
| `matterport.yaml` | Config | 58 |  | 配置迁移 |  |
| `contour_detector.h` | C++ | 250 | ✓ | ProcessModule (C++子进程) |  |
| `contour_graph.h` | C++ | 275 | ✓ | ProcessModule (C++子进程) |  |
| `dynamic_graph.h` | C++ | 699 | ✓ | ProcessModule (C++子进程) |  |
| `far_planner.h` | C++ | 240 | ✓ | ProcessModule (C++子进程) |  |
| `graph_msger.h` | C++ | 84 | ✓ | ProcessModule (C++子进程) |  |
| `graph_planner.h` | C++ | 250 | ✓ | ProcessModule (C++子进程) |  |
| `grid.h` | C++ | 335 |  | ProcessModule (C++子进程) |  |
| `intersection.h` | C++ | 74 |  | ProcessModule (C++子进程) |  |
| `map_handler.h` | C++ | 203 |  | ProcessModule (C++子进程) |  |
| `node_struct.h` | C++ | 161 |  | ProcessModule (C++子进程) |  |
| `planner_visualizer.h` | C++ | 106 | ✓ | ProcessModule (C++子进程) |  |
| `point_struct.h` | C++ | 175 |  | ProcessModule (C++子进程) |  |
| `scan_handler.h` | C++ | 65 |  | ProcessModule (C++子进程) |  |
| `terrain_planner.h` | C++ | 129 | ✓ | ProcessModule (C++子进程) |  |
| `time_measure.h` | C++ | 57 |  | ProcessModule (C++子进程) |  |
| `utility.h` | C++ | 452 | ✓ | ProcessModule (C++子进程) |  |
| `package.xml` | Config | 39 |  | 配置迁移 |  |
| `goal_pose_to_point.py` | Python | 41 | ✓ | 需解耦 (含ROS2依赖) | geometry_msgs, rclpy |
| `contour_detector.cpp` | C++ | 189 |  | ProcessModule (C++子进程) |  |
| `contour_graph.cpp` | C++ | 732 | ✓ | ProcessModule (C++子进程) |  |
| `dynamic_graph.cpp` | C++ | 853 | ✓ | ProcessModule (C++子进程) |  |
| `far_planner.cpp` | C++ | 913 | ✓ | ProcessModule (C++子进程) |  |
| `graph_msger.cpp` | C++ | 214 | ✓ | ProcessModule (C++子进程) |  |
| `graph_planner.cpp` | C++ | 488 | ✓ | ProcessModule (C++子进程) |  |
| `map_handler.cpp` | C++ | 542 |  | ProcessModule (C++子进程) |  |
| `planner_visualizer.cpp` | C++ | 486 | ✓ | ProcessModule (C++子进程) |  |
| `scan_handler.cpp` | C++ | 182 |  | ProcessModule (C++子进程) |  |
| `terrain_planner.cpp` | C++ | 189 | ✓ | ProcessModule (C++子进程) |  |
| `utility.cpp` | C++ | 913 | ✓ | ProcessModule (C++子进程) |  |

### `global_plan/graph_decoder` (5 文件, 899 行)

| 文件 | 语言 | 行数 | ROS2 | 迁移类型 | 关键依赖 |
|------|------|------|------|----------|----------|
| `default.yaml` | Config | 3 |  | 配置迁移 |  |
| `decoder_node.h` | C++ | 175 | ✓ | ProcessModule (C++子进程) |  |
| `point_struct.h` | C++ | 175 |  | ProcessModule (C++子进程) |  |
| `package.xml` | Config | 26 |  | 配置迁移 |  |
| `decoder_node.cpp` | C++ | 520 | ✓ | ProcessModule (C++子进程) |  |

### `global_plan/launch` (4 文件, 964 行)

| 文件 | 语言 | 行数 | ROS2 | 迁移类型 | 关键依赖 |
|------|------|------|------|----------|----------|
| `mapping_launch.py` | Python | 70 |  | 不迁移 (Launch) | launch, launch_ros |
| `system_launch.py` | Python | 653 |  | 不迁移 (Launch) | ament_index_python, launch, launch_ros |
| `planner_only_launch.py` | Python | 81 |  | 不迁移 (Launch) | ament_index_python, launch, launch_ros |
| `test_planning_launch.py` | Python | 160 |  | 不迁移 (Launch) | ament_index_python, launch, launch_ros |

### `global_plan/planner` (8 文件, 1,522 行)

| 文件 | 语言 | 行数 | ROS2 | 迁移类型 | 关键依赖 |
|------|------|------|------|----------|----------|
| `global_planner.py` | Python | 392 | ✓ | 需解耦 (含ROS2依赖) | ament_index_python, config, geometry_msgs, importlib, nav_msgs |
| `pct_planner_astar.py` | Python | 670 | ✓ | 需解耦 (含ROS2依赖) | geometry_msgs, nav_msgs, numpy, pickle, rclpy |
| `planner_wrapper.py` | Python | 276 |  | 直接迁移 (纯算法) | build_tomogram, gc, lib, numpy, pickle |
| `check_map.py` | Python | 87 |  | 直接迁移 (纯算法) | numpy, pickle |
| `fake_localization.py` | Python | 57 | ✓ | 需解耦 (含ROS2依赖) | geometry_msgs, rclpy, tf2_ros |
| `__init__.py` | Python | 2 |  | 直接迁移 (纯算法) |  |
| `convertion.py` | Python | 14 |  | 直接迁移 (纯算法) | numpy |
| `vis_ros.py` | Python | 24 |  | 直接迁移 (纯算法) | geometry_msgs, nav_msgs, std_msgs |

### `global_plan/resources` (1 文件, 23 行)

| 文件 | 语言 | 行数 | ROS2 | 迁移类型 | 关键依赖 |
|------|------|------|------|----------|----------|
| `filter.py` | Python | 23 |  | 直接迁移 (纯算法) | open3d |

### `global_plan/tomography` (16 文件, 1,110 行)

| 文件 | 语言 | 行数 | ROS2 | 迁移类型 | 关键依赖 |
|------|------|------|------|----------|----------|
| `__init__.py` | Python | 11 |  | 直接迁移 (纯算法) |  |
| `param.py` | Python | 16 |  | 直接迁移 (纯算法) |  |
| `prototype.py` | Python | 28 |  | 直接迁移 (纯算法) | numpy, sensor_msgs |
| `scene.py` | Python | 21 |  | 直接迁移 (纯算法) |  |
| `scene_building.py` | Python | 23 |  | 直接迁移 (纯算法) |  |
| `scene_common.py` | Python | 22 |  | 直接迁移 (纯算法) |  |
| `scene_floor.py` | Python | 22 |  | 直接迁移 (纯算法) |  |
| `scene_plaza.py` | Python | 23 |  | 直接迁移 (纯算法) |  |
| `scene_room.py` | Python | 22 |  | 直接迁移 (纯算法) |  |
| `scene_spiral.py` | Python | 23 |  | 直接迁移 (纯算法) |  |
| `scene_stairs.py` | Python | 22 |  | 直接迁移 (纯算法) |  |
| `build_tomogram.py` | Python | 105 |  | 直接迁移 (纯算法) | numpy, open3d, pickle, tomogram |
| `kernels.py` | Python | 175 |  | 直接迁移 (纯算法) | numba, numpy |
| `tomogram.py` | Python | 185 |  | 直接迁移 (纯算法) | kernels, numpy |
| `tomography.py` | Python | 236 | ✓ | 需解耦 (含ROS2依赖) | config, numpy, open3d, pickle, rclpy |
| `visualize_tomogram.py` | Python | 176 | ✓ | 需解耦 (含ROS2依赖) | config, numpy, pickle, rclpy, sensor_msgs |

### `global_plan/visibility` (1 文件, 27 行)

| 文件 | 语言 | 行数 | ROS2 | 迁移类型 | 关键依赖 |
|------|------|------|------|----------|----------|
| `package.xml` | Config | 27 |  | 配置迁移 |  |

### `hardware/bridge_rust` (5 文件, 610 行)

| 文件 | 语言 | 行数 | ROS2 | 迁移类型 | 关键依赖 |
|------|------|------|------|----------|----------|
| `build.rs` | Rust | 9 |  | ProcessModule (Rust子进程) |  |
| `cms_client.rs` | Rust | 270 |  | ProcessModule (Rust子进程) |  |
| `main.rs` | Rust | 156 |  | ProcessModule (Rust子进程) |  |
| `quaternion.rs` | Rust | 82 |  | ProcessModule (Rust子进程) |  |
| `watchdog.rs` | Rust | 93 |  | ProcessModule (Rust子进程) |  |

### `hardware/driver` (4 文件, 825 行)

| 文件 | 语言 | 行数 | ROS2 | 迁移类型 | 关键依赖 |
|------|------|------|------|----------|----------|
| `driver_node.py` | Python | 222 | ✓ | 需解耦 (含ROS2依赖) | geometry_msgs, nav_msgs, rclpy, serial, std_msgs |
| `han_dog_bridge.py` | Python | 572 | ✓ | 需解耦 (含ROS2依赖) | geometry_msgs, grpc, han_dog_message, interface, nav_msgs |
| `package.xml` | Config | 30 |  | 配置迁移 |  |
| `__init__.py` | Python | 1 |  | 直接迁移 (纯算法) |  |

### `hardware/lidar` (44 文件, 5,060 行)

| 文件 | 语言 | 行数 | ROS2 | 迁移类型 | 关键依赖 |
|------|------|------|------|----------|----------|
| `HAP_config.json` | Config | 42 |  | 配置迁移 |  |
| `MID360_config.json` | Config | 42 |  | 配置迁移 |  |
| `mixed_HAP_MID360_config.json` | Config | 76 |  | 配置迁移 |  |
| `msg_HAP_launch.py` | Python | 55 |  | 直接迁移 (纯算法) | ament_index_python, launch, launch_ros |
| `msg_MID360_launch.py` | Python | 58 |  | 直接迁移 (纯算法) | ament_index_python, launch, launch_ros |
| `rviz_HAP_launch.py` | Python | 63 |  | 直接迁移 (纯算法) | ament_index_python, launch, launch_ros |
| `rviz_MID360_launch.py` | Python | 63 |  | 直接迁移 (纯算法) | ament_index_python, launch, launch_ros |
| `rviz_mixed.py` | Python | 63 |  | 直接迁移 (纯算法) | ament_index_python, launch, launch_ros |
| `package.xml` | Config | 35 |  | 配置迁移 |  |
| `package_ROS1.xml` | Config | 82 |  | 配置迁移 |  |
| `package_ROS2.xml` | Config | 35 |  | 配置迁移 |  |
| `lidar_common_callback.cpp` | C++ | 73 |  | ProcessModule (C++子进程) |  |
| `lidar_common_callback.h` | C++ | 40 |  | ProcessModule (C++子进程) |  |
| `livox_lidar_callback.cpp` | C++ | 333 |  | ProcessModule (C++子进程) |  |
| `livox_lidar_callback.h` | C++ | 71 |  | ProcessModule (C++子进程) |  |
| `cache_index.cpp` | C++ | 121 |  | ProcessModule (C++子进程) |  |
| `cache_index.h` | C++ | 54 |  | ProcessModule (C++子进程) |  |
| `comm.cpp` | C++ | 70 |  | ProcessModule (C++子进程) |  |
| `comm.h` | C++ | 304 |  | ProcessModule (C++子进程) |  |
| `ldq.cpp` | C++ | 150 |  | ProcessModule (C++子进程) |  |
| `ldq.h` | C++ | 66 |  | ProcessModule (C++子进程) |  |
| `lidar_imu_data_queue.cpp` | C++ | 70 |  | ProcessModule (C++子进程) |  |
| `lidar_imu_data_queue.h` | C++ | 77 |  | ProcessModule (C++子进程) |  |
| `pub_handler.cpp` | C++ | 457 |  | ProcessModule (C++子进程) |  |
| `pub_handler.h` | C++ | 138 |  | ProcessModule (C++子进程) |  |
| `semaphore.cpp` | C++ | 41 |  | ProcessModule (C++子进程) |  |
| `semaphore.h` | C++ | 51 |  | ProcessModule (C++子进程) |  |
| `driver_node.cpp` | C++ | 46 |  | ProcessModule (C++子进程) |  |
| `driver_node.h` | C++ | 78 | ✓ | ProcessModule (C++子进程) |  |
| `livox_ros_driver2.h` | C++ | 40 |  | ProcessModule (C++子进程) |  |
| `ros1_headers.h` | C++ | 54 |  | ProcessModule (C++子进程) |  |
| `ros2_headers.h` | C++ | 52 | ✓ | ProcessModule (C++子进程) |  |
| `ros_headers.h` | C++ | 34 |  | ProcessModule (C++子进程) |  |
| `lddc.cpp` | C++ | 703 | ✓ | ProcessModule (C++子进程) |  |
| `lddc.h` | C++ | 163 | ✓ | ProcessModule (C++子进程) |  |
| `lds.cpp` | C++ | 200 |  | ProcessModule (C++子进程) |  |
| `lds.h` | C++ | 83 |  | ProcessModule (C++子进程) |  |
| `lds_lidar.cpp` | C++ | 214 |  | ProcessModule (C++子进程) |  |
| `lds_lidar.h` | C++ | 95 |  | ProcessModule (C++子进程) |  |
| `livox_ros_driver2.cpp` | C++ | 235 | ✓ | ProcessModule (C++子进程) |  |
| `parse_cfg_file.cpp` | C++ | 67 |  | ProcessModule (C++子进程) |  |
| `parse_cfg_file.h` | C++ | 52 |  | ProcessModule (C++子进程) |  |
| `parse_livox_lidar_cfg.cpp` | C++ | 156 |  | ProcessModule (C++子进程) |  |
| `parse_livox_lidar_cfg.h` | C++ | 58 |  | ProcessModule (C++子进程) |  |

### `interaction/grpc_gateway` (46 文件, 13,903 行)

| 文件 | 语言 | 行数 | ROS2 | 迁移类型 | 关键依赖 |
|------|------|------|------|----------|----------|
| `grpc_gateway.yaml` | Config | 165 |  | 配置迁移 |  |
| `event_buffer.hpp` | C++ | 72 |  | ProcessModule (C++子进程) |  |
| `flight_recorder.hpp` | C++ | 156 |  | ProcessModule (C++子进程) |  |
| `geofence_monitor.hpp` | C++ | 110 | ✓ | ProcessModule (C++子进程) |  |
| `health_monitor.hpp` | C++ | 167 | ✓ | ProcessModule (C++子进程) |  |
| `idempotency_cache.hpp` | C++ | 125 |  | ProcessModule (C++子进程) |  |
| `lease_manager.hpp` | C++ | 50 |  | ProcessModule (C++子进程) |  |
| `localization_scorer.hpp` | C++ | 143 | ✓ | ProcessModule (C++子进程) |  |
| `mode_manager.hpp` | C++ | 154 | ✓ | ProcessModule (C++子进程) |  |
| `safety_gate.hpp` | C++ | 121 | ✓ | ProcessModule (C++子进程) |  |
| `service_orchestrator.hpp` | C++ | 149 | ✓ | ProcessModule (C++子进程) |  |
| `task_manager.hpp` | C++ | 281 | ✓ | ProcessModule (C++子进程) |  |
| `grpc_gateway.hpp` | C++ | 95 | ✓ | ProcessModule (C++子进程) |  |
| `control_service.hpp` | C++ | 109 |  | ProcessModule (C++子进程) |  |
| `data_service.hpp` | C++ | 140 | ✓ | ProcessModule (C++子进程) |  |
| `system_service.hpp` | C++ | 151 | ✓ | ProcessModule (C++子进程) |  |
| `telemetry_service.hpp` | C++ | 45 |  | ProcessModule (C++子进程) |  |
| `status_aggregator.hpp` | C++ | 221 | ✓ | ProcessModule (C++子进程) |  |
| `webrtc_bridge.hpp` | C++ | 191 | ✓ | ProcessModule (C++子进程) |  |
| `grpc_gateway.launch.py` | Launch | 33 | ✓ | 不迁移 (Launch) |  |
| `package.xml` | Config | 28 |  | 配置迁移 |  |
| `ble_peripheral.py` | Python | 398 |  | 直接迁移 (纯算法) | bluezero, dbus |
| `test_grpc_ros2_bridge.py` | Python | 399 |  | 直接迁移 (纯算法) |  |
| `test_integration.py` | Python | 667 |  | 直接迁移 (纯算法) |  |
| `event_buffer.cpp` | C++ | 266 |  | ProcessModule (C++子进程) |  |
| `flight_recorder.cpp` | C++ | 295 |  | ProcessModule (C++子进程) |  |
| `geofence_monitor.cpp` | C++ | 231 | ✓ | ProcessModule (C++子进程) |  |
| `health_monitor.cpp` | C++ | 296 | ✓ | ProcessModule (C++子进程) |  |
| `lease_manager.cpp` | C++ | 115 |  | ProcessModule (C++子进程) |  |
| `localization_scorer.cpp` | C++ | 292 | ✓ | ProcessModule (C++子进程) |  |
| `mode_manager.cpp` | C++ | 354 | ✓ | ProcessModule (C++子进程) |  |
| `safety_gate.cpp` | C++ | 435 | ✓ | ProcessModule (C++子进程) |  |
| `service_orchestrator.cpp` | C++ | 409 | ✓ | ProcessModule (C++子进程) |  |
| `task_manager.cpp` | C++ | 1153 | ✓ | ProcessModule (C++子进程) |  |
| `grpc_gateway.cpp` | C++ | 486 | ✓ | ProcessModule (C++子进程) |  |
| `main.cpp` | C++ | 16 | ✓ | ProcessModule (C++子进程) |  |
| `control_service.cpp` | C++ | 821 |  | ProcessModule (C++子进程) |  |
| `data_service.cpp` | C++ | 1352 | ✓ | ProcessModule (C++子进程) |  |
| `system_service.cpp` | C++ | 1065 | ✓ | ProcessModule (C++子进程) |  |
| `telemetry_service.cpp` | C++ | 164 |  | ProcessModule (C++子进程) |  |
| `status_aggregator.cpp` | C++ | 751 | ✓ | ProcessModule (C++子进程) |  |
| `webrtc_bridge.cpp` | C++ | 750 | ✓ | ProcessModule (C++子进程) |  |
| `test_event_buffer.cpp` | C++ | 111 |  | ProcessModule (C++子进程) |  |
| `test_flight_recorder.cpp` | C++ | 148 |  | ProcessModule (C++子进程) |  |
| `test_idempotency_cache.cpp` | C++ | 122 |  | ProcessModule (C++子进程) |  |
| `test_lease_manager.cpp` | C++ | 101 |  | ProcessModule (C++子进程) |  |

### `interaction/rings` (6 文件, 1,017 行)

| 文件 | 语言 | 行数 | ROS2 | 迁移类型 | 关键依赖 |
|------|------|------|------|----------|----------|
| `__init__.py` | Python | 6 |  | 直接迁移 (纯算法) |  |
| `dialogue_manager.py` | Python | 279 | ✓ | 需解耦 (含ROS2依赖) | nav_msgs, rclpy, semantic_common, std_msgs |
| `evaluator.py` | Python | 338 | ✓ | 需解耦 (含ROS2依赖) | geometry_msgs, nav_msgs, numpy, rclpy, semantic_common |
| `safety_monitor.py` | Python | 343 | ✓ | 需解耦 (含ROS2依赖) | geometry_msgs, interface, nav_msgs, rclpy, semantic_common |
| `package.xml` | Config | 25 |  | 配置迁移 |  |
| `setup.py` | Python | 26 |  | 直接迁移 (纯算法) | setuptools |

### `llm/agent` (1 文件, 714 行)

| 文件 | 语言 | 行数 | ROS2 | 迁移类型 | 关键依赖 |
|------|------|------|------|----------|----------|
| `agent_node.py` | Python | 714 | ✓ | 需解耦 (含ROS2依赖) | , geometry_msgs, langchain, langchain_core, langchain_openai |

### `llm/client` (1 文件, 741 行)

| 文件 | 语言 | 行数 | ROS2 | 迁移类型 | 关键依赖 |
|------|------|------|------|----------|----------|
| `llm_client.py` | Python | 741 |  | 直接迁移 (纯算法) | anthropic, dashscope, openai |

### `llm/mcp` (1 文件, 284 行)

| 文件 | 语言 | 行数 | ROS2 | 迁移类型 | 关键依赖 |
|------|------|------|------|----------|----------|
| `mcp_server.py` | Python | 284 |  | 直接迁移 (纯算法) | semantic_planner |

### `llm/prompts` (1 文件, 715 行)

| 文件 | 语言 | 行数 | ROS2 | 迁移类型 | 关键依赖 |
|------|------|------|------|----------|----------|
| `prompt_templates.py` | Python | 715 |  | 直接迁移 (纯算法) |  |

### `llm/reasoning` (2 文件, 1,009 行)

| 文件 | 语言 | 行数 | ROS2 | 迁移类型 | 关键依赖 |
|------|------|------|------|----------|----------|
| `adacot.py` | Python | 208 |  | 直接迁移 (纯算法) |  |
| `sgnav_reasoner.py` | Python | 801 |  | 直接迁移 (纯算法) | , numpy |

### `llm/tokenizer` (1 文件, 508 行)

| 文件 | 语言 | 行数 | ROS2 | 迁移类型 | 关键依赖 |
|------|------|------|------|----------|----------|
| `chinese_tokenizer.py` | Python | 508 |  | 直接迁移 (纯算法) | jieba |

### `local_plan/local_planner` (7 文件, 3,379 行)

| 文件 | 语言 | 行数 | ROS2 | 迁移类型 | 关键依赖 |
|------|------|------|------|----------|----------|
| `omniDir.yaml` | Config | 15 |  | 配置迁移 |  |
| `standard.yaml` | Config | 15 |  | 配置迁移 |  |
| `package.xml` | Config | 28 |  | 配置迁移 |  |
| `localPlanner.cpp` | C++ | 1449 | ✓ | ProcessModule (C++子进程) |  |
| `pathFollower.cpp` | C++ | 680 | ✓ | ProcessModule (C++子进程) |  |
| `test_algorithms.cpp` | C++ | 583 |  | ProcessModule (C++子进程) |  |
| `test_planner_perf.cpp` | C++ | 609 |  | ProcessModule (C++子进程) |  |

### `local_plan/scan_gen` (2 文件, 170 行)

| 文件 | 语言 | 行数 | ROS2 | 迁移类型 | 关键依赖 |
|------|------|------|------|----------|----------|
| `package.xml` | Config | 29 |  | 配置迁移 |  |
| `sensorScanGeneration.cpp` | C++ | 141 | ✓ | ProcessModule (C++子进程) |  |

### `local_plan/terrain` (2 文件, 847 行)

| 文件 | 语言 | 行数 | ROS2 | 迁移类型 | 关键依赖 |
|------|------|------|------|----------|----------|
| `package.xml` | Config | 28 |  | 配置迁移 |  |
| `terrainAnalysis.cpp` | C++ | 819 | ✓ | ProcessModule (C++子进程) |  |

### `local_plan/terrain_ext` (2 文件, 625 行)

| 文件 | 语言 | 行数 | ROS2 | 迁移类型 | 关键依赖 |
|------|------|------|------|----------|----------|
| `package.xml` | Config | 28 |  | 配置迁移 |  |
| `terrainAnalysisExt.cpp` | C++ | 597 | ✓ | ProcessModule (C++子进程) |  |

### `local_plan/viz` (4 文件, 627 行)

| 文件 | 语言 | 行数 | ROS2 | 迁移类型 | 关键依赖 |
|------|------|------|------|----------|----------|
| `package.xml` | Config | 30 |  | 不迁移 (工具/示例) |  |
| `realTimePlot.py` | Python | 156 | ✓ | 不迁移 (工具/示例) | matplotlib, numpy, rclpy, std_msgs |
| `visualizationTools.cpp` | C++ | 441 | ✓ | 不迁移 (工具/示例) |  |
| `__init__.py` | Python | 0 |  | 不迁移 (工具/示例) |  |

### `memory/episodic` (1 文件, 205 行)

| 文件 | 语言 | 行数 | ROS2 | 迁移类型 | 关键依赖 |
|------|------|------|------|----------|----------|
| `episodic_memory.py` | Python | 205 |  | 直接迁移 (纯算法) | numpy |

### `memory/tagged` (1 文件, 138 行)

| 文件 | 语言 | 行数 | ROS2 | 迁移类型 | 关键依赖 |
|------|------|------|------|----------|----------|
| `tagged_locations.py` | Python | 138 |  | 直接迁移 (纯算法) |  |

### `memory/topological` (1 文件, 762 行)

| 文件 | 语言 | 行数 | ROS2 | 迁移类型 | 关键依赖 |
|------|------|------|------|----------|----------|
| `topological_memory.py` | Python | 762 |  | 直接迁移 (纯算法) | numpy, scipy, semantic_common |

### `nav_core` (14 文件, 2,527 行)

| 文件 | 语言 | 行数 | ROS2 | 迁移类型 | 关键依赖 |
|------|------|------|------|----------|----------|
| `py_nav_core.cpp` | C++ | 154 |  | ProcessModule (C++子进程) |  |
| `local_planner_core.hpp` | C++ | 176 |  | ProcessModule (C++子进程) |  |
| `path_follower_core.hpp` | C++ | 219 |  | ProcessModule (C++子进程) |  |
| `pct_adapter_core.hpp` | C++ | 200 |  | ProcessModule (C++子进程) |  |
| `types.hpp` | C++ | 54 |  | ProcessModule (C++子进程) |  |
| `validation.hpp` | C++ | 54 |  | ProcessModule (C++子进程) |  |
| `package.xml` | Config | 17 |  | 配置迁移 |  |
| `test_benchmark.cpp` | C++ | 169 |  | ProcessModule (C++子进程) |  |
| `test_local_planner_core.cpp` | C++ | 222 |  | ProcessModule (C++子进程) |  |
| `test_param_sensitivity.cpp` | C++ | 298 |  | ProcessModule (C++子进程) |  |
| `test_path_edge_cases.cpp` | C++ | 395 |  | ProcessModule (C++子进程) |  |
| `test_path_follower_core.cpp` | C++ | 229 |  | ProcessModule (C++子进程) |  |
| `test_pct_adapter_core.cpp` | C++ | 213 |  | ProcessModule (C++子进程) |  |
| `test_validation.cpp` | C++ | 127 |  | ProcessModule (C++子进程) |  |

### `reconstruction` (7 文件, 680 行)

| 文件 | 语言 | 行数 | ROS2 | 迁移类型 | 关键依赖 |
|------|------|------|------|----------|----------|
| `package.xml` | Config | 30 |  | 配置迁移 |  |
| `__init__.py` | Python | 0 |  | 直接迁移 (纯算法) |  |
| `color_projector.py` | Python | 128 |  | 直接迁移 (纯算法) | numpy |
| `ply_writer.py` | Python | 104 |  | 直接迁移 (纯算法) | numpy |
| `reconstruction_node.py` | Python | 293 | ✓ | 需解耦 (含ROS2依赖) | , cv2, cv_bridge, message_filters, numpy |
| `semantic_labeler.py` | Python | 101 |  | 直接迁移 (纯算法) | numpy |
| `setup.py` | Python | 24 |  | 直接迁移 (纯算法) | setuptools |

### `semantic/api` (8 文件, 1,193 行)

| 文件 | 语言 | 行数 | ROS2 | 迁移类型 | 关键依赖 |
|------|------|------|------|----------|----------|
| `__init__.py` | Python | 75 |  | 直接迁移 (纯算法) |  |
| `detector_api.py` | Python | 111 |  | 直接迁移 (纯算法) | , numpy |
| `encoder_api.py` | Python | 168 |  | 直接迁移 (纯算法) | , numpy |
| `exceptions.py` | Python | 70 |  | 直接迁移 (纯算法) |  |
| `factory.py` | Python | 233 |  | 直接迁移 (纯算法) |  |
| `perception_api.py` | Python | 193 |  | 直接迁移 (纯算法) | , numpy |
| `tracker_api.py` | Python | 124 |  | 直接迁移 (纯算法) | , numpy |
| `types.py` | Python | 219 |  | 直接迁移 (纯算法) | numpy |

### `semantic/belief` (2 文件, 1,253 行)

| 文件 | 语言 | 行数 | ROS2 | 迁移类型 | 关键依赖 |
|------|------|------|------|----------|----------|
| `belief_network.py` | Python | 652 |  | 直接迁移 (纯算法) | numpy, random, torch |
| `belief_propagation.py` | Python | 601 |  | 直接迁移 (纯算法) | , numpy, scipy |

### `semantic/core` (20 文件, 9,064 行)

| 文件 | 语言 | 行数 | ROS2 | 迁移类型 | 关键依赖 |
|------|------|------|------|----------|----------|
| `__init__.py` | Python | 64 |  | 直接迁移 (纯算法) |  |
| `baseline_wrappers.py` | Python | 429 |  | 直接迁移 (纯算法) | numpy, semantic_perception |
| `dataset_loader.py` | Python | 479 |  | 直接迁移 (纯算法) | cv2, numpy |
| `detector_base.py` | Python | 58 |  | 直接迁移 (纯算法) | numpy |
| `end_to_end_evaluation.py` | Python | 405 |  | 直接迁移 (纯算法) | numpy, semantic_perception |
| `evaluation_framework.py` | Python | 464 |  | 直接迁移 (纯算法) | numpy, psutil |
| `geometry_extractor.py` | Python | 400 |  | 直接迁移 (纯算法) | numpy, scipy |
| `global_coverage_mask.py` | Python | 354 |  | 直接迁移 (纯算法) | numpy, pickle |
| `hybrid_planner.py` | Python | 523 |  | 直接迁移 (纯算法) | numpy |
| `instance_tracker.py` | Python | 1571 |  | 直接迁移 (纯算法) | , numpy, scipy, semantic_common, semantic_perception |
| `keyframe_selector.py` | Python | 219 |  | 直接迁移 (纯算法) | cv2, numpy |
| `laplacian_filter.py` | Python | 43 |  | 直接迁移 (纯算法) | cv2, numpy |
| `leiden_segmentation.py` | Python | 366 |  | 直接迁移 (纯算法) | igraph, leidenalg, networkx, numpy |
| `local_rolling_grid.py` | Python | 340 |  | 直接迁移 (纯算法) | numpy |
| `polyhedron_expansion.py` | Python | 515 |  | 直接迁移 (纯算法) | numpy, scipy |
| `projection.py` | Python | 241 |  | 直接迁移 (纯算法) | numpy |
| `room_manager.py` | Python | 959 |  | 直接迁移 (纯算法) | , numpy, scipy, sklearn |
| `tracked_objects.py` | Python | 566 | ✓ | 需解耦 (含ROS2依赖) | , numpy |
| `uncertainty_model.py` | Python | 464 |  | 直接迁移 (纯算法) | numpy |
| `visualization_tools.py` | Python | 604 |  | 直接迁移 (纯算法) | matplotlib, mpl_toolkits, numpy |

### `semantic/detectors` (3 文件, 735 行)

| 文件 | 语言 | 行数 | ROS2 | 迁移类型 | 关键依赖 |
|------|------|------|------|----------|----------|
| `grounding_dino_detector.py` | Python | 148 |  | 直接迁移 (纯算法) | , PIL, groundingdino, numpy, torch |
| `yolo_world_detector.py` | Python | 364 |  | 直接迁移 (纯算法) | , numpy, semantic_common, torch, ultralytics |
| `yoloe_detector.py` | Python | 223 |  | 直接迁移 (纯算法) | , cv2, numpy, ultralytics |

### `semantic/detectors_bpu` (3 文件, 1,262 行)

| 文件 | 语言 | 行数 | ROS2 | 迁移类型 | 关键依赖 |
|------|------|------|------|----------|----------|
| `bpu_detector.py` | Python | 672 |  | 直接迁移 (纯算法) | , cv2, glob, hbm_runtime, numpy |
| `bpu_qp_bridge.py` | Python | 314 |  | 直接迁移 (纯算法) | , numpy, qp_perception, semantic_perception |
| `bpu_tracker.py` | Python | 276 |  | 直接迁移 (纯算法) | , numpy, types, ultralytics |

### `semantic/encoders` (2 文件, 896 行)

| 文件 | 语言 | 行数 | ROS2 | 迁移类型 | 关键依赖 |
|------|------|------|------|----------|----------|
| `clip_encoder.py` | Python | 623 |  | 直接迁移 (纯算法) | PIL, numpy, open_clip, semantic_common, torch |
| `mobileclip_encoder.py` | Python | 273 |  | 直接迁移 (纯算法) | mobileclip, numpy, open_clip, torch |

### `semantic/examples` (4 文件, 1,327 行)

| 文件 | 语言 | 行数 | ROS2 | 迁移类型 | 关键依赖 |
|------|------|------|------|----------|----------|
| `api_usage_examples.py` | Python | 220 |  | 不迁移 (工具/示例) | numpy, semantic_perception |
| `performance_analysis.py` | Python | 356 |  | 不迁移 (工具/示例) | cProfile, numpy, pstats, scipy, semantic_perception |
| `run_quantitative_experiments.py` | Python | 472 |  | 不迁移 (工具/示例) | matplotlib, numpy, scipy, semantic_perception |
| `uss_nav_integration_demo.py` | Python | 279 |  | 不迁移 (工具/示例) | numpy, semantic_perception |

### `semantic/impl` (5 文件, 1,603 行)

| 文件 | 语言 | 行数 | ROS2 | 迁移类型 | 关键依赖 |
|------|------|------|------|----------|----------|
| `__init__.py` | Python | 17 |  | 直接迁移 (纯算法) |  |
| `clip_encoder.py` | Python | 410 |  | 直接迁移 (纯算法) | , PIL, numpy, open_clip, torch |
| `instance_tracker.py` | Python | 315 |  | 直接迁移 (纯算法) | , numpy |
| `perception_impl.py` | Python | 533 |  | 直接迁移 (纯算法) | , numpy |
| `yolo_world_detector.py` | Python | 328 |  | 直接迁移 (纯算法) | , numpy, ultralytics |

### `semantic/knowledge` (2 文件, 2,512 行)

| 文件 | 语言 | 行数 | ROS2 | 迁移类型 | 关键依赖 |
|------|------|------|------|----------|----------|
| `knowledge_data.py` | Python | 1944 |  | 直接迁移 (纯算法) |  |
| `knowledge_graph.py` | Python | 568 |  | 直接迁移 (纯算法) | , numpy |

### `semantic/pipeline` (1 文件, 483 行)

| 文件 | 语言 | 行数 | ROS2 | 迁移类型 | 关键依赖 |
|------|------|------|------|----------|----------|
| `perception_pipeline.py` | Python | 483 | ✓ | 需解耦 (含ROS2依赖) | , numpy, rclpy, semantic_common, std_msgs |

### `semantic/ros2_shell` (2 文件, 809 行)

| 文件 | 语言 | 行数 | ROS2 | 迁移类型 | 关键依赖 |
|------|------|------|------|----------|----------|
| `perception_node.py` | Python | 567 | ✓ | 不迁移 (ROS2 壳) | , cv2, cv_bridge, message_filters, nav_msgs |
| `perception_publishers.py` | Python | 242 |  | 不迁移 (ROS2 壳) | aiohttp, httpx, std_msgs, visualization_msgs |

### `semantic/scene_graph` (2 文件, 1,088 行)

| 文件 | 语言 | 行数 | ROS2 | 迁移类型 | 关键依赖 |
|------|------|------|------|----------|----------|
| `scg_builder.py` | Python | 516 |  | 直接迁移 (纯算法) | numpy, scipy |
| `scg_path_planner.py` | Python | 572 |  | 直接迁移 (纯算法) | numpy |

### `semantic/storage` (3 文件, 601 行)

| 文件 | 语言 | 行数 | ROS2 | 迁移类型 | 关键依赖 |
|------|------|------|------|----------|----------|
| `__init__.py` | Python | 18 |  | 直接迁移 (纯算法) | semantic_perception |
| `sqlite_store.py` | Python | 265 |  | 直接迁移 (纯算法) | pickle, semantic_perception, sqlite3 |
| `timeseries_store.py` | Python | 318 |  | 直接迁移 (纯算法) |  |

### `semantic/topology` (2 文件, 996 行)

| 文件 | 语言 | 行数 | ROS2 | 迁移类型 | 关键依赖 |
|------|------|------|------|----------|----------|
| `topology_graph.py` | Python | 536 |  | 直接迁移 (纯算法) | , numpy |
| `topology_types.py` | Python | 460 | ✓ | 需解耦 (含ROS2依赖) | numpy |

### `semantic_planner/action_executor` (1 文件, 382 行)

| 文件 | 语言 | 行数 | ROS2 | 迁移类型 | 关键依赖 |
|------|------|------|------|----------|----------|
| `action_executor.py` | Python | 382 |  | 直接迁移 (纯算法) | numpy, semantic_common |

### `semantic_planner/bbox_nav` (2 文件, 602 行)

| 文件 | 语言 | 行数 | ROS2 | 迁移类型 | 关键依赖 |
|------|------|------|------|----------|----------|
| `bbox_nav_mixin.py` | Python | 223 |  | 直接迁移 (纯算法) | , geometry_msgs, numpy, std_msgs |
| `bbox_navigator.py` | Python | 379 |  | 直接迁移 (纯算法) | numpy |

### `semantic_planner/core` (6 文件, 1,708 行)

| 文件 | 语言 | 行数 | ROS2 | 迁移类型 | 关键依赖 |
|------|------|------|------|----------|----------|
| `__init__.py` | Python | 62 |  | 直接迁移 (纯算法) |  |
| `goal_mixin.py` | Python | 382 |  | 直接迁移 (纯算法) | , geometry_msgs, semantic_common, std_msgs |
| `operational_mixin.py` | Python | 266 |  | 直接迁移 (纯算法) | , geometry_msgs, std_msgs |
| `planner_state.py` | Python | 20 |  | 直接迁移 (纯算法) |  |
| `state_mixin.py` | Python | 373 |  | 直接迁移 (纯算法) | , geometry_msgs, numpy, semantic_common, std_msgs |
| `subgoal_mixin.py` | Python | 605 |  | 直接迁移 (纯算法) | , numpy, semantic_common |

### `semantic_planner/exploration` (1 文件, 125 行)

| 文件 | 语言 | 行数 | ROS2 | 迁移类型 | 关键依赖 |
|------|------|------|------|----------|----------|
| `exploration_strategy.py` | Python | 125 |  | 直接迁移 (纯算法) | , numpy |

### `semantic_planner/fast_path` (1 文件, 939 行)

| 文件 | 语言 | 行数 | ROS2 | 迁移类型 | 关键依赖 |
|------|------|------|------|----------|----------|
| `fast_path.py` | Python | 939 |  | 直接迁移 (纯算法) | , numpy, semantic_common |

### `semantic_planner/frontier` (2 文件, 998 行)

| 文件 | 语言 | 行数 | ROS2 | 迁移类型 | 关键依赖 |
|------|------|------|------|----------|----------|
| `frontier_scorer.py` | Python | 596 |  | 直接迁移 (纯算法) | , numpy, semantic_common |
| `frontier_types.py` | Python | 402 |  | 直接迁移 (纯算法) | , numpy, semantic_common |

### `semantic_planner/fsm_policy` (1 文件, 298 行)

| 文件 | 语言 | 行数 | ROS2 | 迁移类型 | 关键依赖 |
|------|------|------|------|----------|----------|
| `implicit_fsm_policy.py` | Python | 298 |  | 直接迁移 (纯算法) | numpy |

### `semantic_planner/goal_resolver` (1 文件, 376 行)

| 文件 | 语言 | 行数 | ROS2 | 迁移类型 | 关键依赖 |
|------|------|------|------|----------|----------|
| `goal_resolver.py` | Python | 376 |  | 直接迁移 (纯算法) | , numpy, semantic_common, semantic_perception |

### `semantic_planner/person_tracker` (1 文件, 480 行)

| 文件 | 语言 | 行数 | ROS2 | 迁移类型 | 关键依赖 |
|------|------|------|------|----------|----------|
| `person_tracker.py` | Python | 480 |  | 直接迁移 (纯算法) | numpy |

### `semantic_planner/room_kg` (1 文件, 457 行)

| 文件 | 语言 | 行数 | ROS2 | 迁移类型 | 关键依赖 |
|------|------|------|------|----------|----------|
| `room_object_kg.py` | Python | 457 |  | 直接迁移 (纯算法) | , semantic_perception |

### `semantic_planner/ros2_shell` (4 文件, 2,303 行)

| 文件 | 语言 | 行数 | ROS2 | 迁移类型 | 关键依赖 |
|------|------|------|------|----------|----------|
| `callbacks_mixin.py` | Python | 302 |  | 不迁移 (ROS2 壳) | , base64, cv2, cv_bridge, nav_msgs |
| `init_mixin.py` | Python | 632 | ✓ | 不迁移 (ROS2 壳) | , geometry_msgs, nav2_msgs, nav_msgs, rclpy |
| `nav2_mixin.py` | Python | 872 | ✓ | 不迁移 (ROS2 壳) | , geometry_msgs, nav2_msgs, numpy, semantic_common |
| `planner_node.py` | Python | 497 | ✓ | 不迁移 (ROS2 壳) | , geometry_msgs, nav2_msgs, nav_msgs, numpy |

### `semantic_planner/semantic_prior` (1 文件, 615 行)

| 文件 | 语言 | 行数 | ROS2 | 迁移类型 | 关键依赖 |
|------|------|------|------|----------|----------|
| `semantic_prior.py` | Python | 615 |  | 直接迁移 (纯算法) |  |

### `semantic_planner/skill_registry` (1 文件, 547 行)

| 文件 | 语言 | 行数 | ROS2 | 迁移类型 | 关键依赖 |
|------|------|------|------|----------|----------|
| `skill_registry.py` | Python | 547 |  | 直接迁移 (纯算法) | langchain_core, pydantic |

### `semantic_planner/slow_path` (1 文件, 945 行)

| 文件 | 语言 | 行数 | ROS2 | 迁移类型 | 关键依赖 |
|------|------|------|------|----------|----------|
| `slow_path.py` | Python | 945 |  | 直接迁移 (纯算法) | , numpy, random, semantic_common, semantic_perception |

### `semantic_planner/task_decomposer` (1 文件, 337 行)

| 文件 | 语言 | 行数 | ROS2 | 迁移类型 | 关键依赖 |
|------|------|------|------|----------|----------|
| `task_decomposer.py` | Python | 337 |  | 直接迁移 (纯算法) |  |

### `semantic_planner/task_rules` (1 文件, 871 行)

| 文件 | 语言 | 行数 | ROS2 | 迁移类型 | 关键依赖 |
|------|------|------|------|----------|----------|
| `task_rules.py` | Python | 871 |  | 直接迁移 (纯算法) |  |

### `semantic_planner/viz` (1 文件, 443 行)

| 文件 | 语言 | 行数 | ROS2 | 迁移类型 | 关键依赖 |
|------|------|------|------|----------|----------|
| `rerun_viewer.py` | Python | 443 | ✓ | 不迁移 (工具/示例) | geometry_msgs, nav_msgs, numpy, rclpy, rerun |

### `semantic_planner/vlm_nav` (1 文件, 365 行)

| 文件 | 语言 | 行数 | ROS2 | 迁移类型 | 关键依赖 |
|------|------|------|------|----------|----------|
| `vlm_bbox_query.py` | Python | 365 |  | 直接迁移 (纯算法) |  |

### `semantic_planner/voi_scheduler` (1 文件, 315 行)

| 文件 | 语言 | 行数 | ROS2 | 迁移类型 | 关键依赖 |
|------|------|------|------|----------|----------|
| `voi_scheduler.py` | Python | 315 |  | 直接迁移 (纯算法) |  |

### `services` (8 文件, 1,382 行)

| 文件 | 语言 | 行数 | ROS2 | 迁移类型 | 关键依赖 |
|------|------|------|------|----------|----------|
| `__init__.py` | Python | 0 |  | 直接迁移 (纯算法) |  |
| `geofence_manager.py` | Python | 262 | ✓ | 需解耦 (含ROS2依赖) | geometry_msgs, nav_msgs, rclpy, semantic_common, std_msgs |
| `map_manager.py` | Python | 333 | ✓ | 需解耦 (含ROS2依赖) | geometry_msgs, interface, nav_msgs, rclpy, semantic_common |
| `mission_logger.py` | Python | 270 | ✓ | 需解耦 (含ROS2依赖) | nav_msgs, rclpy, semantic_common, std_msgs |
| `patrol_manager.py` | Python | 229 | ✓ | 需解耦 (含ROS2依赖) | rclpy, semantic_common, std_msgs, yaml |
| `task_scheduler.py` | Python | 237 | ✓ | 需解耦 (含ROS2依赖) | rclpy, semantic_common, std_msgs, yaml |
| `package.xml` | Config | 22 |  | 配置迁移 |  |
| `setup.py` | Python | 29 |  | 直接迁移 (纯算法) | setuptools |

### `slam/fastlio2` (19 文件, 3,738 行)

| 文件 | 语言 | 行数 | ROS2 | 迁移类型 | 关键依赖 |
|------|------|------|------|----------|----------|
| `lio.yaml` | Config | 49 |  | 配置迁移 |  |
| `lio_velodyne.yaml` | Config | 41 |  | 配置迁移 |  |
| `lio_launch.py` | Python | 37 |  | 直接迁移 (纯算法) | launch, launch_ros |
| `package.xml` | Config | 30 |  | 配置迁移 |  |
| `lio_node.cpp` | C++ | 388 | ✓ | ProcessModule (C++子进程) |  |
| `commons.cpp` | C++ | 37 |  | ProcessModule (C++子进程) |  |
| `commons.h` | C++ | 88 |  | ProcessModule (C++子进程) |  |
| `ieskf.cpp` | C++ | 122 |  | ProcessModule (C++子进程) |  |
| `ieskf.h` | C++ | 84 |  | ProcessModule (C++子进程) |  |
| `ikd_Tree.cpp` | C++ | 1732 |  | ProcessModule (C++子进程) |  |
| `ikd_Tree.h` | C++ | 345 |  | ProcessModule (C++子进程) |  |
| `imu_processor.cpp` | C++ | 132 |  | ProcessModule (C++子进程) |  |
| `imu_processor.h` | C++ | 24 |  | ProcessModule (C++子进程) |  |
| `lidar_processor.cpp` | C++ | 348 |  | ProcessModule (C++子进程) |  |
| `lidar_processor.h` | C++ | 51 |  | ProcessModule (C++子进程) |  |
| `map_builder.cpp` | C++ | 35 |  | ProcessModule (C++子进程) |  |
| `map_builder.h` | C++ | 28 |  | ProcessModule (C++子进程) |  |
| `utils.cpp` | C++ | 119 |  | ProcessModule (C++子进程) |  |
| `utils.h` | C++ | 48 | ✓ | ProcessModule (C++子进程) |  |

### `slam/hba` (11 文件, 1,284 行)

| 文件 | 语言 | 行数 | ROS2 | 迁移类型 | 关键依赖 |
|------|------|------|------|----------|----------|
| `hba.yaml` | Config | 11 |  | 配置迁移 |  |
| `hba_launch.py` | Python | 32 |  | 直接迁移 (纯算法) | launch, launch_ros |
| `package.xml` | Config | 25 |  | 配置迁移 |  |
| `blam.cpp` | C++ | 458 |  | ProcessModule (C++子进程) |  |
| `blam.h` | C++ | 140 |  | ProcessModule (C++子进程) |  |
| `commons.cpp` | C++ | 1 |  | ProcessModule (C++子进程) |  |
| `commons.h` | C++ | 38 |  | ProcessModule (C++子进程) |  |
| `hba.cpp` | C++ | 174 |  | ProcessModule (C++子进程) |  |
| `hba.h` | C++ | 57 |  | ProcessModule (C++子进程) |  |
| `hba_node copy.cpp` | C++ | 126 | ✓ | ProcessModule (C++子进程) |  |
| `hba_node.cpp` | C++ | 222 | ✓ | ProcessModule (C++子进程) |  |

### `slam/interface` (2 文件, 179 行)

| 文件 | 语言 | 行数 | ROS2 | 迁移类型 | 关键依赖 |
|------|------|------|------|----------|----------|
| `package.xml` | Config | 27 |  | 配置迁移 |  |
| `robot_state_publisher.cpp` | C++ | 152 | ✓ | ProcessModule (C++子进程) |  |

### `slam/localizer` (8 文件, 610 行)

| 文件 | 语言 | 行数 | ROS2 | 迁移类型 | 关键依赖 |
|------|------|------|------|----------|----------|
| `localizer.yaml` | Config | 16 |  | 配置迁移 |  |
| `localizer_launch.py` | Python | 53 |  | 直接迁移 (纯算法) | launch, launch_ros |
| `package.xml` | Config | 27 |  | 配置迁移 |  |
| `localizer_node.cpp` | C++ | 356 | ✓ | ProcessModule (C++子进程) |  |
| `commons.cpp` | C++ | 1 |  | ProcessModule (C++子进程) |  |
| `commons.h` | C++ | 15 |  | ProcessModule (C++子进程) |  |
| `icp_localizer.cpp` | C++ | 93 |  | ProcessModule (C++子进程) |  |
| `icp_localizer.h` | C++ | 49 |  | ProcessModule (C++子进程) |  |

### `slam/pgo` (8 文件, 723 行)

| 文件 | 语言 | 行数 | ROS2 | 迁移类型 | 关键依赖 |
|------|------|------|------|----------|----------|
| `pgo.yaml` | Config | 13 |  | 配置迁移 |  |
| `pgo_launch.py` | Python | 46 |  | 直接迁移 (纯算法) | launch, launch_ros |
| `package.xml` | Config | 30 |  | 配置迁移 |  |
| `pgo_node.cpp` | C++ | 307 | ✓ | ProcessModule (C++子进程) |  |
| `commons.cpp` | C++ | 8 |  | ProcessModule (C++子进程) |  |
| `commons.h` | C++ | 30 |  | ProcessModule (C++子进程) |  |
| `simple_pgo.cpp` | C++ | 211 |  | ProcessModule (C++子进程) |  |
| `simple_pgo.h` | C++ | 78 |  | ProcessModule (C++子进程) |  |

### `slam/pointlio` (37 文件, 11,713 行)

| 文件 | 语言 | 行数 | ROS2 | 迁移类型 | 关键依赖 |
|------|------|------|------|----------|----------|
| `avia.yaml` | Config | 73 |  | 配置迁移 |  |
| `horizon.yaml` | Config | 73 |  | 配置迁移 |  |
| `mid360.yaml` | Config | 73 |  | 配置迁移 |  |
| `ouster64.yaml` | Config | 73 |  | 配置迁移 |  |
| `velody16.yaml` | Config | 73 |  | 配置迁移 |  |
| `esekfom.hpp` | C++ | 343 |  | ProcessModule (C++子进程) |  |
| `util.hpp` | C++ | 82 |  | ProcessModule (C++子进程) |  |
| `build_manifold.hpp` | C++ | 248 |  | ProcessModule (C++子进程) |  |
| `SubManifold.hpp` | C++ | 123 |  | ProcessModule (C++子进程) |  |
| `mtkmath.hpp` | C++ | 294 |  | ProcessModule (C++子进程) |  |
| `vectview.hpp` | C++ | 168 |  | ProcessModule (C++子进程) |  |
| `startIdx.hpp` | C++ | 328 |  | ProcessModule (C++子进程) |  |
| `S2.hpp` | C++ | 293 |  | ProcessModule (C++子进程) |  |
| `SEn.hpp` | C++ | 342 |  | ProcessModule (C++子进程) |  |
| `SOn.hpp` | C++ | 357 |  | ProcessModule (C++子进程) |  |
| `vect.hpp` | C++ | 513 |  | ProcessModule (C++子进程) |  |
| `wrapped_cv_mat.hpp` | C++ | 113 |  | ProcessModule (C++子进程) |  |
| `common_lib.h` | C++ | 223 | ✓ | ProcessModule (C++子进程) |  |
| `eigen_types.h` | C++ | 95 |  | ProcessModule (C++子进程) |  |
| `hilbert.hpp` | C++ | 647 |  | ProcessModule (C++子进程) |  |
| `ivox3d.h` | C++ | 341 |  | ProcessModule (C++子进程) |  |
| `ivox3d_node.hpp` | C++ | 452 |  | ProcessModule (C++子进程) |  |
| `matplotlibcpp.h` | C++ | 2659 |  | ProcessModule (C++子进程) |  |
| `so3_math.h` | C++ | 128 |  | ProcessModule (C++子进程) |  |
| `point_lio.launch.py` | Launch | 73 | ✓ | 不迁移 (Launch) |  |
| `package.xml` | Config | 39 |  | 配置迁移 |  |
| `Estimator.cpp` | C++ | 392 |  | ProcessModule (C++子进程) |  |
| `Estimator.h` | C++ | 61 |  | ProcessModule (C++子进程) |  |
| `IMU_Processing.cpp` | C++ | 115 | ✓ | ProcessModule (C++子进程) |  |
| `IMU_Processing.h` | C++ | 61 | ✓ | ProcessModule (C++子进程) |  |
| `laserMapping.cpp` | C++ | 1061 | ✓ | ProcessModule (C++子进程) |  |
| `li_initialization.cpp` | C++ | 298 | ✓ | ProcessModule (C++子进程) |  |
| `li_initialization.h` | C++ | 38 |  | ProcessModule (C++子进程) |  |
| `parameters.cpp` | C++ | 293 | ✓ | ProcessModule (C++子进程) |  |
| `parameters.h` | C++ | 89 | ✓ | ProcessModule (C++子进程) |  |
| `preprocess.cpp` | C++ | 935 | ✓ | ProcessModule (C++子进程) |  |
| `preprocess.h` | C++ | 144 | ✓ | ProcessModule (C++子进程) |  |

### `transport` (7 文件, 737 行)

| 文件 | 语言 | 行数 | ROS2 | 迁移类型 | 关键依赖 |
|------|------|------|------|----------|----------|
| `__init__.py` | Python | 32 | ✓ | 需解耦 (含ROS2依赖) |  |
| `core.py` | Python | 93 | ✓ | 需解耦 (含ROS2依赖) |  |
| `dds_transport.py` | Python | 80 | ✓ | 需解耦 (含ROS2依赖) | , rclpy |
| `dual_transport.py` | Python | 79 | ✓ | 需解耦 (含ROS2依赖) |  |
| `factory.py` | Python | 100 | ✓ | 需解耦 (含ROS2依赖) |  |
| `ros2_mixin.py` | Python | 171 | ✓ | 需解耦 (含ROS2依赖) |  |
| `shm_transport.py` | Python | 182 | ✓ | 需解耦 (含ROS2依赖) | , multiprocessing, numpy, pickle |

### `uncategorized` (131 文件, 25,815 行)

| 文件 | 语言 | 行数 | ROS2 | 迁移类型 | 关键依赖 |
|------|------|------|------|----------|----------|
| `package.xml` | Config | 26 |  | 配置迁移 |  |
| `a_star_search.h` | C++ | 124 |  | ProcessModule (C++子进程) |  |
| `type.h` | C++ | 17 |  | ProcessModule (C++子进程) |  |
| `data_types.h` | C++ | 37 |  | ProcessModule (C++子进程) |  |
| `affine_constraint.h` | C++ | 58 |  | ProcessModule (C++子进程) |  |
| `discrete_points_math.h` | C++ | 34 |  | ProcessModule (C++子进程) |  |
| `osqp_spline1d_solver.h` | C++ | 45 |  | ProcessModule (C++子进程) |  |
| `osqp_spline2d_solver.h` | C++ | 44 |  | ProcessModule (C++子进程) |  |
| `polynomialxd.h` | C++ | 52 |  | ProcessModule (C++子进程) |  |
| `osqp_interface.h` | C++ | 42 |  | ProcessModule (C++子进程) |  |
| `osqp_sparse_matrix.h` | C++ | 89 |  | ProcessModule (C++子进程) |  |
| `spline1d.h` | C++ | 52 |  | ProcessModule (C++子进程) |  |
| `spline1d_constraint.h` | C++ | 83 |  | ProcessModule (C++子进程) |  |
| `spline1d_kernel.h` | C++ | 69 |  | ProcessModule (C++子进程) |  |
| `spline1d_kernel_helper.h` | C++ | 66 |  | ProcessModule (C++子进程) |  |
| `spline1d_seg.h` | C++ | 51 |  | ProcessModule (C++子进程) |  |
| `spline1d_solver.h` | C++ | 56 |  | ProcessModule (C++子进程) |  |
| `spline2d.h` | C++ | 93 |  | ProcessModule (C++子进程) |  |
| `spline2d_constraint.h` | C++ | 123 |  | ProcessModule (C++子进程) |  |
| `spline2d_kernel.h` | C++ | 73 |  | ProcessModule (C++子进程) |  |
| `spline2d_seg.h` | C++ | 66 |  | ProcessModule (C++子进程) |  |
| `spline2d_solver.h` | C++ | 57 |  | ProcessModule (C++子进程) |  |
| `math.h` | C++ | 26 |  | ProcessModule (C++子进程) |  |
| `offline_ele_planner.h` | C++ | 65 |  | ProcessModule (C++子进程) |  |
| `dense_elevation_map.h` | C++ | 83 |  | ProcessModule (C++子进程) |  |
| `gp_heading_rate_factor.h` | C++ | 22 |  | ProcessModule (C++子进程) |  |
| `gp_interpolate_heading_rate_factor.h` | C++ | 36 |  | ProcessModule (C++子进程) |  |
| `gp_interpolate_obstacle_factor.h` | C++ | 47 |  | ProcessModule (C++子进程) |  |
| `gp_obstacle_factor.h` | C++ | 36 |  | ProcessModule (C++子进程) |  |
| `gp_prior_factor.h` | C++ | 28 |  | ProcessModule (C++子进程) |  |
| `gp_prior_factor.h` | C++ | 28 |  | ProcessModule (C++子进程) |  |
| `gp_interpolate_obstacle_factor.h` | C++ | 46 |  | ProcessModule (C++子进程) |  |
| `gp_obstacle_factor.h` | C++ | 36 |  | ProcessModule (C++子进程) |  |
| `gp_prior_factor.h` | C++ | 28 |  | ProcessModule (C++子进程) |  |
| `gpmp_optimizer.h` | C++ | 69 |  | ProcessModule (C++子进程) |  |
| `gpmp_optimizer_wnoa.h` | C++ | 63 |  | ProcessModule (C++子进程) |  |
| `wnoa_interpolator.hpp` | C++ | 64 |  | ProcessModule (C++子进程) |  |
| `wnoa_trajectory_interpolator.h` | C++ | 22 |  | ProcessModule (C++子进程) |  |
| `wnoj_interpolator.hpp` | C++ | 48 |  | ProcessModule (C++子进程) |  |
| `wnoj_trajectory_interpolator.h` | C++ | 22 |  | ProcessModule (C++子进程) |  |
| `wnoa.hpp` | C++ | 90 |  | ProcessModule (C++子进程) |  |
| `wnoa_origin.hpp` | C++ | 53 |  | ProcessModule (C++子进程) |  |
| `wnoj.hpp` | C++ | 148 |  | ProcessModule (C++子进程) |  |
| `height_smoother.h` | C++ | 12 |  | ProcessModule (C++子进程) |  |
| `package.xml` | Config | 36 |  | 配置迁移 |  |
| `setup.py` | Python | 31 |  | 直接迁移 (纯算法) | setuptools |
| `__init__.py` | Python | 0 |  | 直接迁移 (纯算法) |  |
| `conftest.py` | Python | 128 |  | 直接迁移 (纯算法) | numpy, semantic_perception |
| `test_clip_encoder.py` | Python | 244 |  | 直接迁移 (纯算法) | numpy, semantic_perception, torch |
| `test_global_coverage_mask.py` | Python | 222 |  | 直接迁移 (纯算法) | numpy, semantic_perception |
| `test_incremental_update.py` | Python | 470 |  | 直接迁移 (纯算法) | numpy, semantic_perception |
| `test_instance_tracker.py` | Python | 272 |  | 直接迁移 (纯算法) | numpy, semantic_perception |
| `test_knowledge_graph.py` | Python | 349 |  | 直接迁移 (纯算法) | semantic_perception |
| `test_laplacian_filter.py` | Python | 269 |  | 直接迁移 (纯算法) | numpy, semantic_perception |
| `test_parallel_comparison.py` | Python | 1008 | ✓ | 需解耦 (含ROS2依赖) | numpy, semantic_perception |
| `test_room_clip.py` | Python | 194 |  | 直接迁移 (纯算法) | numpy, semantic_perception |
| `test_scg_ros_integration.py` | Python | 566 |  | 直接迁移 (纯算法) | numpy, semantic_perception |
| `test_topology_graph.py` | Python | 490 |  | 直接迁移 (纯算法) | numpy, semantic_perception |
| `test_yolo_world_detector.py` | Python | 284 |  | 直接迁移 (纯算法) | numpy, semantic_perception, torch |
| `test_baseline_wrappers.py` | Python | 300 |  | 直接迁移 (纯算法) | numpy, semantic_perception |
| `test_dataset_loader.py` | Python | 351 |  | 直接迁移 (纯算法) | cv2, numpy, semantic_perception |
| `test_end_to_end_evaluation.py` | Python | 240 |  | 直接迁移 (纯算法) | cv2, numpy, semantic_perception |
| `test_evaluation_framework.py` | Python | 335 |  | 直接迁移 (纯算法) | numpy, semantic_perception |
| `test_gcm.py` | Python | 287 |  | 直接迁移 (纯算法) | numpy, semantic_perception |
| `test_geometry_enhanced_topology.py` | Python | 227 |  | 直接迁移 (纯算法) | numpy, semantic_perception |
| `test_hybrid_planner.py` | Python | 253 |  | 直接迁移 (纯算法) | numpy, semantic_perception |
| `test_local_rolling_grid.py` | Python | 268 |  | 直接迁移 (纯算法) | numpy, semantic_perception |
| `test_polyhedron_debug.py` | Python | 189 |  | 直接迁移 (纯算法) | numpy, semantic_perception |
| `test_polyhedron_expansion.py` | Python | 203 |  | 直接迁移 (纯算法) | numpy, semantic_perception |
| `test_quantitative_experiments.py` | Python | 268 |  | 直接迁移 (纯算法) | examples, matplotlib, numpy, semantic_perception |
| `test_scg_builder.py` | Python | 310 |  | 直接迁移 (纯算法) | numpy, semantic_perception |
| `test_scg_path_planner.py` | Python | 310 |  | 直接迁移 (纯算法) | numpy, semantic_perception |
| `test_uncertainty_model.py` | Python | 339 |  | 直接迁移 (纯算法) | numpy, semantic_perception |
| `test_visualization_tools.py` | Python | 344 |  | 直接迁移 (纯算法) | matplotlib, numpy, scipy, semantic_perception |
| `test_viz_simple.py` | Python | 132 |  | 直接迁移 (纯算法) | matplotlib, numpy, scipy, semantic_perception |
| `package.xml` | Config | 29 |  | 配置迁移 |  |
| `setup.py` | Python | 42 |  | 直接迁移 (纯算法) | setuptools |
| `__init__.py` | Python | 0 |  | 直接迁移 (纯算法) |  |
| `conftest.py` | Python | 144 |  | 直接迁移 (纯算法) | numpy, semantic_planner |
| `test_action_executor.py` | Python | 264 |  | 直接迁移 (纯算法) | numpy, semantic_planner |
| `test_bahsg_ablation.py` | Python | 547 |  | 直接迁移 (纯算法) | numpy, random |
| `test_bbox_navigator.py` | Python | 240 |  | 直接迁移 (纯算法) | numpy, semantic_planner |
| `test_episodic_memory.py` | Python | 68 |  | 直接迁移 (纯算法) | numpy, semantic_planner |
| `test_exploration_strategy.py` | Python | 138 |  | 直接迁移 (纯算法) | numpy, semantic_planner |
| `test_fast_resolve.py` | Python | 351 |  | 直接迁移 (纯算法) | semantic_planner |
| `test_fast_slow_benchmark.py` | Python | 598 |  | 直接迁移 (纯算法) | numpy, random, semantic_planner |
| `test_fast_slow_efficiency.py` | Python | 630 |  | 直接迁移 (纯算法) | numpy, random, semantic_planner |
| `test_frontier_scorer.py` | Python | 265 |  | 直接迁移 (纯算法) | numpy, semantic_planner |
| `test_goal_resolver.py` | Python | 226 |  | 直接迁移 (纯算法) | semantic_planner |
| `test_implicit_fsm_policy.py` | Python | 87 |  | 直接迁移 (纯算法) | numpy, semantic_planner |
| `test_intent_parsing.py` | Python | 1028 |  | 直接迁移 (纯算法) | semantic_planner |
| `test_llm_client_async.py` | Python | 203 |  | 直接迁移 (纯算法) | concurrent, semantic_planner |
| `test_mcp_server.py` | Python | 216 |  | 直接迁移 (纯算法) | semantic_planner |
| `test_person_tracker.py` | Python | 170 |  | 直接迁移 (纯算法) | semantic_planner |
| `test_planner_node_init.py` | Python | 202 | ✓ | 需解耦 (含ROS2依赖) | semantic_planner, types |
| `test_room_object_kg.py` | Python | 346 |  | 直接迁移 (纯算法) | numpy, semantic_planner |
| `test_semantic_prior.py` | Python | 370 |  | 直接迁移 (纯算法) | semantic_planner |
| `test_sgnav_reasoner.py` | Python | 226 |  | 直接迁移 (纯算法) | numpy, semantic_planner |
| `test_skill_registry.py` | Python | 317 |  | 直接迁移 (纯算法) | langchain_core, semantic_planner |
| `test_slow_path_llm.py` | Python | 318 |  | 直接迁移 (纯算法) | semantic_planner |
| `test_slow_path_real_llm.py` | Python | 321 |  | 直接迁移 (纯算法) | semantic_planner |
| `test_tagged_locations.py` | Python | 238 |  | 直接迁移 (纯算法) | semantic_planner |
| `test_task_decomposer.py` | Python | 203 |  | 直接迁移 (纯算法) | semantic_planner |
| `test_topological_memory.py` | Python | 173 |  | 直接迁移 (纯算法) | numpy, semantic_planner |
| `test_uncertainty_scoring.py` | Python | 127 |  | 直接迁移 (纯算法) | numpy, semantic_planner |
| `test_vlm_bbox_query.py` | Python | 174 |  | 直接迁移 (纯算法) | semantic_planner |
| `test_voi_comparison.py` | Python | 307 |  | 直接迁移 (纯算法) | semantic_planner |
| `test_voi_scheduler.py` | Python | 533 |  | 直接迁移 (纯算法) | semantic_planner |
| `vla_nav.yaml` | Config | 69 |  | 配置迁移 |  |
| `package.xml` | Config | 34 |  | 配置迁移 |  |
| `setup.py` | Python | 37 |  | 直接迁移 (纯算法) | setuptools |
| `test_model_architecture.py` | Python | 335 |  | 直接迁移 (纯算法) | numpy, torch, vla_nav |
| `__init__.py` | Python | 20 |  | 直接迁移 (纯算法) |  |
| `__init__.py` | Python | 1 |  | 直接迁移 (纯算法) |  |
| `quantize.py` | Python | 267 |  | 直接迁移 (纯算法) | auto_gptq, datasets, optimum, peft, torch |
| `tensorrt_export.py` | Python | 304 |  | 直接迁移 (纯算法) | onnxruntime, tensorrt, torch, transformers, vla_nav |
| `__init__.py` | Python | 17 |  | 直接迁移 (纯算法) | vla_nav |
| `action_head.py` | Python | 195 |  | 直接迁移 (纯算法) | torch |
| `adacot.py` | Python | 277 |  | 直接迁移 (纯算法) | torch |
| `backbone.py` | Python | 383 |  | 直接迁移 (纯算法) | PIL, numpy, qwen_vl_utils, torch, transformers |
| `vla_model.py` | Python | 445 |  | 直接迁移 (纯算法) | numpy, torch, vla_nav |
| `vlingmem.py` | Python | 373 |  | 直接迁移 (纯算法) | numpy, torch |
| `__init__.py` | Python | 1 |  | 直接迁移 (纯算法) |  |
| `vla_nav.launch.py` | Launch | 65 | ✓ | 不迁移 (Launch) |  |
| `vla_nav_node.py` | Python | 464 | ✓ | 需解耦 (含ROS2依赖) | cv_bridge, geometry_msgs, nav_msgs, numpy, rclpy |
| `__init__.py` | Python | 1 |  | 直接迁移 (纯算法) |  |
| `adacot_annotator.py` | Python | 409 |  | 直接迁移 (纯算法) | dashscope, numpy, openai |
| `dataset.py` | Python | 202 |  | 直接迁移 (纯算法) | cv2, numpy, torch |
| `habitat_collector.py` | Python | 452 |  | 直接迁移 (纯算法) | cv2, habitat, habitat_sim, numpy |
| `rl_trainer.py` | Python | 550 |  | 直接迁移 (纯算法) | numpy, torch, vla_nav |
| `sft_trainer.py` | Python | 465 |  | 直接迁移 (纯算法) | numpy, peft, torch, transformers, vla_nav |

---

## sdk/modules/ 目标结构建议

```
sdk/modules/
├── common/
│   ├── (root)/ (6 files)
├── global_plan/
│   ├── adapters/ (6 files)
│   ├── boundary/ (6 files)
│   ├── config/ (2 files)
│   ├── far_planner/ (31 files)
│   ├── graph_decoder/ (5 files)
│   ├── launch/ (4 files)
│   ├── planner/ (8 files)
│   ├── resources/ (1 files)
│   ├── tomography/ (16 files)
│   ├── visibility/ (1 files)
├── hardware/
│   ├── bridge_rust/ (5 files)
│   ├── driver/ (4 files)
│   ├── lidar/ (44 files)
├── interaction/
│   ├── grpc_gateway/ (46 files)
│   ├── rings/ (6 files)
├── llm/
│   ├── agent/ (1 files)
│   ├── client/ (1 files)
│   ├── mcp/ (1 files)
│   ├── prompts/ (1 files)
│   ├── reasoning/ (2 files)
│   ├── tokenizer/ (1 files)
├── local_plan/
│   ├── local_planner/ (7 files)
│   ├── scan_gen/ (2 files)
│   ├── terrain/ (2 files)
│   ├── terrain_ext/ (2 files)
│   ├── viz/ (4 files)
├── memory/
│   ├── episodic/ (1 files)
│   ├── tagged/ (1 files)
│   ├── topological/ (1 files)
├── nav_core/
│   ├── (root)/ (14 files)
├── reconstruction/
│   ├── (root)/ (7 files)
├── semantic/
│   ├── api/ (8 files)
│   ├── belief/ (2 files)
│   ├── core/ (20 files)
│   ├── detectors/ (3 files)
│   ├── detectors_bpu/ (3 files)
│   ├── encoders/ (2 files)
│   ├── examples/ (4 files)
│   ├── impl/ (5 files)
│   ├── knowledge/ (2 files)
│   ├── pipeline/ (1 files)
│   ├── ros2_shell/ (2 files)
│   ├── scene_graph/ (2 files)
│   ├── storage/ (3 files)
│   ├── topology/ (2 files)
├── semantic_planner/
│   ├── action_executor/ (1 files)
│   ├── bbox_nav/ (2 files)
│   ├── core/ (6 files)
│   ├── exploration/ (1 files)
│   ├── fast_path/ (1 files)
│   ├── frontier/ (2 files)
│   ├── fsm_policy/ (1 files)
│   ├── goal_resolver/ (1 files)
│   ├── person_tracker/ (1 files)
│   ├── room_kg/ (1 files)
│   ├── ros2_shell/ (4 files)
│   ├── semantic_prior/ (1 files)
│   ├── skill_registry/ (1 files)
│   ├── slow_path/ (1 files)
│   ├── task_decomposer/ (1 files)
│   ├── task_rules/ (1 files)
│   ├── viz/ (1 files)
│   ├── vlm_nav/ (1 files)
│   ├── voi_scheduler/ (1 files)
├── services/
│   ├── (root)/ (8 files)
├── slam/
│   ├── fastlio2/ (19 files)
│   ├── hba/ (11 files)
│   ├── interface/ (2 files)
│   ├── localizer/ (8 files)
│   ├── pgo/ (8 files)
│   ├── pointlio/ (37 files)
├── transport/
│   ├── (root)/ (7 files)
├── uncategorized/
│   ├── (root)/ (131 files)
```
