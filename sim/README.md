# LingTu Simulation Environment

> A hardware-free, full-stack simulation framework for quadruped robot navigation. Built on MuJoCo physics with ray-cast LiDAR, it enables end-to-end testing of SLAM, terrain-aware planning, semantic navigation, and person-following — all without a physical robot.

## 1. Overview

The simulation environment mirrors the real-world LingTu deployment stack while replacing hardware sensors and actuators with physics-accurate MuJoCo models. This enables rapid iteration on navigation algorithms, regression testing, and data collection for learning-based methods.

**Supported capabilities:**

- Full 6-DOF rigid body dynamics with contact
- Ray-cast LiDAR simulation (Livox Mid-360 pattern, 360° FoV)
- RGB-D camera rendering
- Person-following behavioral simulation with FSM control
- Semantic search and exploration
- Direct integration with both ROS2 and pure-Python LingTu stacks

## 2. System Architecture

The system has three layers:

**Physics** — MuJoCo loads a world XML + robot MJCF, runs the physics step each tick, and reads ray-cast LiDAR data from the sensor plugin.

**Bridge** — Three bridge options connect physics to navigation:

| Bridge | Path | Notes |
|--------|------|-------|
| `mujoco_ros2_bridge.py` | MuJoCo → ROS2 topics → C++ autonomy stack | Full ROS2, same as real robot |
| `nova_nav_bridge.py` | MuJoCo → Python LingTu modules directly | No ROS2 dependency, fastest iteration |
| `mujoco_viz_bridge.py` | MuJoCo → visualization only | Rendering, no navigation |

**Navigation** — Either the ROS2 C++ autonomy stack (terrain + local planner + path follower) or the pure-Python LingTu module stack (`python lingtu.py sim`).

Worlds, robots, and bridges are independent — any world can host any robot, and either bridge connects to navigation.

## 3. Scenes

Four pre-built environments are provided, each targeting different navigation challenges:

| Scene | File | Description | Terrain Generation |
|-------|------|-------------|-------------------|
| Open Field | `worlds/open_field.xml` | Flat ground for basic validation | None |
| Spiral Terrain | `worlds/spiral_terrain.xml` | 4-layer spiral ramp with elevation changes | `gen_terrain_mesh.py` |
| Building | `worlds/building_scene.xml` | Multi-room indoor building with corridors | None |
| Factory | `worlds/factory_scene.xml` | Warehouse layout with shelving and obstacles | None |

Additional composite scenes in `scenes/` combine robots with specific environments (e.g., `go2_room_nova.xml` places a Go2 in a furnished room).

## 4. LiDAR Simulation

Two approaches are supported for point cloud generation:

### 4.1 Ray-Cast Plugin (Default)

Uses the [mujoco_ray_caster](https://github.com/Albusgive/mujoco_ray_caster) C++ sensor plugin, which calls `mj_ray()` natively within the physics step. No GPU required. Outputs a dense point cloud directly from sensor data.

**Build instructions:**

```bash
git clone https://github.com/google-deepmind/mujoco.git
cd mujoco/plugin
git clone https://github.com/Albusgive/mujoco_ray_caster.git
cd .. && mkdir build && cd build
cmake .. && cmake --build . -j8
export MUJOCO_PLUGIN_PATH=$(pwd)/bin/mujoco_plugin
```

### 4.2 OmniPerception (GPU, Large-Scale)

Based on [OmniPerception](https://github.com/aCodeDog/OmniPerception) (CoRL 2025). Uses Warp/CUDA GPU ray tracing with the exact Livox Mid-360 scanning pattern. Recommended for RL training and batch simulation where throughput matters.

```bash
pip install warp-lang[extras]
cd LidarSensor && pip install -e .
```

## 5. Person Following

The `following/` module implements a complete person-following pipeline for behavioral evaluation.

### 5.1 Architecture

The following system is structured as five independent layers:

| Layer | Module | Responsibility |
|-------|--------|---------------|
| Scene | `engine/` | Physics, robot spawning, person spawning |
| Person | `person/` | Simulated human movement (RoomAwareWalk, waypoint paths) |
| Perception | `perception/` | Simulated detection and tracking with configurable noise |
| Controller | `controller/` | Motion commands from perception input |
| Metrics | `metrics/` | Distance error, tracking loss rate, response latency |

### 5.2 Behavioral FSM

The `FollowingBehavior` state machine manages five states:

| State | Trigger In | Trigger Out |
|-------|-----------|-------------|
| **FOLLOW** | target re-acquired / target moves | target stopped → WAIT, target lost → SEARCH |
| **WAIT** | target stopped | target moves → FOLLOW |
| **SEARCH** | target lost | target found → FOLLOW, timeout → EXPLORE |
| **EXPLORE** | search timeout | target found → FOLLOW, timeout → RECOVER |
| **RECOVER** | explore timeout | (terminal) |

### 5.3 Controller Comparison

Four controllers were benchmarked on stop-walk-stop scenarios:

| Controller | Mean Distance Error | Tracking Loss Rate |
|-----------|--------------------|--------------------|
| PurePursuit | 0.23 m | 4.2% |
| **PurePursuit + Velocity Prediction** | **0.14 m** | **1.8%** |
| PID with Lookahead | 0.19 m | 3.1% |
| Predictive MPC | 0.16 m | 2.4% |

PurePursuit with velocity prediction achieves the best overall performance and is the default.

## 6. Quick Start

### 6.1 Prerequisites

```bash
pip install mujoco numpy scipy
bash sim/scripts/install_deps.sh        # optional: installs all deps
```

### 6.2 Basic Simulation (No ROS2)

```bash
python sim/scripts/go1_indoor_nav.py    # Go1 indoor navigation demo
python lingtu.py sim                     # Full LingTu stack in simulation
```

### 6.2.1 Headless Navigation Smoke Mode

`MujocoDriverModule` supports two drive modes:

| Mode | Purpose |
|------|---------|
| `policy` | Default. Uses the ONNX gait policy and MuJoCo contacts. |
| `kinematic` | Applies `cmd_vel` directly to the floating base for deterministic headless navigation tests. |

Use kinematic mode when validating that LingTu planning, path following,
`CmdVelMux`, odometry, and exploration are wired correctly without requiring a
working gait checkpoint:

```bash
LINGTU_SIM_DRIVE_MODE=kinematic python lingtu.py sim
```

Kinematic mode proves the navigation stack can command route-level motion in
simulation. It is not evidence that the Thunder v3 RL gait policy is healthy;
policy-mode smoke tests should still be run before claiming gait-level fidelity.
Run the policy-mode smoke tests on a machine that has MuJoCo, ONNX Runtime, and
`policy_251119.onnx` available:

```bash
PYTHONPATH=src:. python -m pytest src/core/tests/test_sim_runtime_compat.py -q \
  -k "policy_cmd_vel or full_stack_policy_mode"

PYTHONPATH=src:. python sim/scripts/policy_nav_smoke.py \
  --world open_field \
  --direct-duration 6 \
  --nav-duration 18 \
  --goal-distance 1.0
```

For a longer server-side soak, increase `--nav-duration` to 60 seconds and save
the JSON output with `--json-out artifacts/policy_nav_smoke_open_field.json`.

#### Policy-Mode Soak On Sunrise

Use `sunrise` only as a simulation compute host for this check. Do not start
robot services, publish real `/nav/cmd_vel`, send Gateway goals, or run
`scripts/lingtu nav`, `scripts/lingtu map`, `ros2 topic pub`, or
`systemctl restart robot-*` during the soak.

```bash
ssh sunrise@192.168.66.190
cd ~/data/SLAM/navigation
mkdir -p artifacts
export PYTHONPATH=src:.
export LINGTU_SIM_DRIVE_MODE=policy
export PYTHONIOENCODING=utf-8

python sim/scripts/policy_nav_smoke.py \
  --world open_field \
  --direct-duration 6 \
  --nav-duration 60 \
  --goal-distance 1.0 \
  --json-out artifacts/policy_nav_smoke_open_field.json
```

Treat skipped policy tests as `BLOCKED`, not `PASS`. The pass gate is: policy
loads, all poses stay finite, direct policy motion exceeds `0.20 m`, body height
stays inside `0.35 m < z < 0.55 m`, roll/pitch stay below the script gate,
full-stack nav emits costmap, waypoint, local path, path-follower command and
mux command, `direct_fallback == 0`, and full-stack policy motion exceeds
`0.20 m`.

Common failures:

| Symptom | Likely Cause | First Checks |
| --- | --- | --- |
| policy test skipped | MuJoCo, `onnxruntime`, or checkpoint missing | Check `_POLICY_CANDIDATES` and the sunrise brainstem checkout. |
| `UnsupportedPolicyInputError` | Wrong checkpoint contract | Do not pad or truncate; wire the original observation builder first. |
| little forward motion | Direction/action scale, standing pose, or joint order mismatch | Check `PolicyRunner.build_obs`, `ACTION_SCALE`, `MJ_TO_DART`, and `DART_TO_MJ`. |
| low body height or high roll/pitch | Bad warm-up, contact, actuator mapping, or incompatible policy | Check MJCF actuator-to-joint resolution and policy warm-up. |
| no costmap | LiDAR scene/body/group problem | Run the MuJoCo LiDAR smoke and inspect `get_lidar_points()`. |
| no waypoint/local path/mux command | Navigation wiring, not gait | Compare the kinematic full-stack test first. |
| `direct_fallback > 0` | Planner or adapter path generation failed | Treat as a full-stack failure, not a gait pass. |

Policy mode currently follows the brainstem `StandardObservationBuilder`
contract: 57 values per frame, optionally stacked as `57 * N` history frames.
The preferred navigation gait checkpoint is brainstem's default
`model/policy_251119.onnx`; place it at
`sim/robots/nova_dog/model/policy_251119.onnx`, repo-root
`model/policy_251119.onnx`, or a sibling `brainstem` checkout using the same
relative `model/policy_251119.onnx` path. The checked-in legacy
`robots/nova_dog/policy.onnx` remains only as a development fallback and uses
5 frames (`285` input values), while brainstem's public
`onnx_runtime/example/policy.onnx` is a single-frame (`57` input values)
example. Newer Thunder checkpoints with non-57-multiple inputs, such as 76-D
RobotLab-style exports, need their original training observation field order,
normalization, and action semantics wired into a dedicated observation builder
before running. The runner intentionally refuses to pad or truncate those inputs
because that produces misleading "policy runs but robot will not walk" failures.

### 6.3 Full Stack with ROS2

```bash
source /opt/ros/humble/setup.bash
ros2 launch sim/launch/sim.launch.py world:=building_scene
```

### 6.4 Send Navigation Goals

```bash
# Via REPL
python lingtu.py sim
> go 5 3
> go 找到餐桌

# Via ROS2
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 5.0, y: 3.0, z: 0.0}}}"
```

### 6.5 Person Following Benchmark

```bash
python sim/scripts/benchmark_following.py
# Results saved to sim/output/benchmark/
```

## 7. Robots

| Robot | Directory | Control | Notes |
|-------|-----------|---------|-------|
| Unitree Go2 | `robots/go2/` | RL policy (48D obs, action_scale=0.5) | MuJoCo Playground compatible |
| Thunder v3 | `assets/{urdf,xml,mjcf}/` | LingTu MuJoCo adapter | Current upstream CAD asset baseline |
| Thunder v3 compatibility | `robots/nova_dog/robot_with_camera.xml` | LingTu MuJoCo adapter | Legacy path now mirrors the current Thunder v3 runtime XML |
| Thunder v3 URDF compatibility | `robot/thunder.urdf` | URDF | Legacy path with mesh references adjusted to `assets/meshes/` |

## 8. Datasets

Offline LiDAR/IMU datasets for algorithm development and testing:

| Dataset | Source | Use Case |
|---------|--------|----------|
| `datasets/Avia/` | Livox Avia | LiDAR-inertial odometry testing |
| `datasets/legkilo*/` | Legged robot | Kinematic-inertial-LiDAR fusion |

## 9. ROS2 Interface

| Topic | Message Type | Direction | Rate |
|-------|-------------|-----------|------|
| `/mujoco/pos_w_pointcloud` | `sensor_msgs/PointCloud2` | Sim → Nav | 10 Hz |
| `/nav/odometry` | `nav_msgs/Odometry` | Sim → Nav | 50 Hz |
| `/nav/cmd_vel` | `geometry_msgs/TwistStamped` | Nav → Sim | 50 Hz |
| TF (`map→odom→body`) | `tf2_msgs/TFMessage` | Sim → Nav | 50 Hz |

## 10. Directory Reference

| Directory | Contents |
|-----------|----------|
| `engine/` | Simulation engine framework (physics loop, MuJoCo wrappers, scenario management) |
| `worlds/` | MuJoCo XML scene definitions (4 environments) |
| `scenes/` | Composite robot + environment configs |
| `robots/` | Robot model definitions (Go2, NOVA Dog) |
| `robot/` | Legacy Thunder URDF + meshes |
| `sensors/` | LiDAR simulation (Livox Mid-360 Python fallback) |
| `bridge/` | Physics ↔ navigation bridges (ROS2, direct Python, visualization) |
| `following/` | Person-following simulation (FSM, controllers, perception, metrics) |
| `semantic/` | Semantic navigation simulation tests |
| `datasets/` | Offline LiDAR/IMU datasets |
| `scripts/` | Demo scripts, benchmarks, terrain generation, dependency installer |
| `launch/` | ROS2 launch files |
| `assets/` | Generated terrain meshes |
| `maps/` | Saved simulation maps |
| `output/` | Demo videos and benchmark results |
| `configs/` | Simulation configuration files |

## 11. Dependencies

| Package | Version | Required |
|---------|---------|----------|
| MuJoCo | >= 3.0 | Yes |
| Python | >= 3.10 | Yes |
| NumPy | >= 1.24 | Yes |
| SciPy | >= 1.10 | Yes |
| ROS2 Humble | latest | Optional (for ROS2 bridge) |
| warp-lang | >= 0.10 | Optional (GPU LiDAR) |
