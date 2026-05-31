# Drivers — Hardware & Simulation Driver Index

> Files live under `src/drivers/`
> 30+ .py files including subdirectories (excluding tests)

---

## Top-Level (5)

| File | Responsibility |
|------|---------------|
| `__init__.py` | Package init — exports public driver APIs |
| `teleop_module.py` | **TeleopModule** — WebSocket joystick, idle auto-release (3s), teleop_active signal |
| `robot_profile.py` | **RobotProfile** — physical robot parameters (wheelbase, max speed, turning radius) |
| `sensor_suite.py` | **SensorSuite** — extrinsics management, sensor frame tree, transform composition |
| `spec.py` | Driver spec — `implements(DriverSpec)` contract for all backends |

---

## Real Hardware — `real/` (7)

| File | Responsibility |
|------|---------------|
| `__init__.py` | Package init |
| `thunder/__init__.py` | Thunder driver package init |
| `thunder/blueprints.py` | Thunder blueprint builder — composes ThunderDriver + CameraBridge |
| `thunder/camera_bridge_module.py` | **CameraBridgeModule** — HAP camera stream → ImageMsg over DDS |
| `thunder/connection.py` | Thunder gRPC connection manager — reconnection, health check |
| `thunder/han_dog_module.py` | **ThunderDriver** (HanDog variant) — gRPC→brainstem cmd_vel/status bridge |
| `lidar/__init__.py` | LiDAR package init |
| `lidar/lidar.py` | Livox LiDAR driver — MID-360 UDP stream, point cloud assembly |
| `lidar/lidar_module.py` | **LidarModule** — Module wrapper around Livox driver |
| `lidar/_dds.py` | LiDAR DDS bridge — publish assembled PointCloud2 over CycloneDDS |

---

## Simulation — `sim/` (11)

| File | Responsibility |
|------|---------------|
| `__init__.py` | Package init |
| `stub.py` | **StubDriver** — mock driver for framework testing, teleports on cmd_vel |
| `mujoco_driver_module.py` | **MuJoCoDriverModule** — MuJoCo physics step → cmd_vel, odometry feedback |
| `mujoco_lingtu_stack.py` | MuJoCo full stack — compiles all modules for sim use |
| `mujoco_live_runtime.py` | MuJoCo live runtime — Fast-LIO2 + MuJoCo live sim |
| `mujoco_ros2_bridge.py` | MuJoCo → ROS2 bridge — joint states, TF, odometry |
| `mujoco_scene_metadata.py` | MuJoCo scene metadata — XML scene introspection |
| `mujoco_sensor_bridge.py` | MuJoCo sensor data → ROS2 topics — IMU, lidar, camera |
| `mujoco_viz_bridge.py` | MuJoCo 3D visualization — point cloud, trajectory, goal rendering |
| `nova_nav_bridge.py` | NOVA navigation bridge — sim-side nav stack integration |
| `ros2_sim_driver.py` | ROS2 simulation driver — subscribes to /cmd_vel, publishes /odom |
| `sim_pointcloud_provider.py` | Simulated point cloud — samples from MuJoCo scene geometry |
| `test_full_pipeline_s100p.py` | S100P full pipeline test — end-to-end verification script |

---

## Livox ROS2 Launch — `livox_ros_driver2/launch/` (5)

| File | Responsibility |
|------|---------------|
| `msg_HAP_launch.py` | Livox HAP LiDAR ROS2 launch config |
| `msg_MID360_launch.py` | Livox MID-360 LiDAR ROS2 launch config |
| `rviz_HAP_launch.py` | RViz visualization config for HAP |
| `rviz_MID360_launch.py` | RViz visualization config for MID-360 |
| `rviz_mixed.py` | Mixed HAP + MID-360 RViz config |
