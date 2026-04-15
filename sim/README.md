# sim/ — LingTu MuJoCo Simulation

Hardware-free full-stack simulation: MuJoCo physics + ray-cast LiDAR + ROS2 navigation + person following + semantic search.

## Quick Start

```bash
# Install dependencies
pip install mujoco numpy scipy

# Simple test (no ROS2 needed)
python sim/scripts/go1_indoor_nav.py

# Full stack with ROS2
source /opt/ros/humble/setup.bash
ros2 launch sim/launch/sim.launch.py world:=open_field
```

## Architecture

```
┌──────────────────────────────────────────────────────────────────────┐
│                        MuJoCo Physics                                │
│                                                                      │
│  worlds/*.xml (4 scenes)    robots/ (Go2, NOVA Dog)                  │
│  ├── terrain meshes         ├── URDF + collision meshes              │
│  ├── obstacles/walls        └── actuator configs                     │
│  └── sensor sites                                                    │
│       └── ray_caster_lidar (mujoco_ray_caster plugin)                │
└──────────────────┬───────────────────────────────────────────────────┘
                   │
         bridge/ (3 bridges)
          ├── mujoco_ros2_bridge.py     MuJoCo ↔ ROS2 (odom/TF/cloud/cmd_vel)
          ├── mujoco_viz_bridge.py      MuJoCo ↔ visualization
          └── nova_nav_bridge.py        MuJoCo ↔ LingTu nav stack (no ROS2)
                   │
         ┌─────────┴──────────┐
         ▼                    ▼
   ROS2 nav stack        LingTu Module stack
   (C++ autonomy)        (Python, sim profile)
```

## Directory Structure

```
sim/
├── engine/                  Simulation engine framework
│   ├── core/                Physics loop, step control
│   ├── mujoco/              MuJoCo-specific wrappers
│   ├── bridge/              Sensor/actuator bridge interfaces
│   ├── worlds/              World loading & management
│   ├── scenarios/           Pre-defined test scenarios
│   └── cli.py               Engine CLI entry point
│
├── worlds/                  MuJoCo XML scene files
│   ├── open_field.xml       Flat ground (quick validation)
│   ├── spiral_terrain.xml   4-layer spiral ramp (requires gen_terrain_mesh.py)
│   ├── building_scene.xml   Indoor multi-room building
│   └── factory_scene.xml    Factory/warehouse layout
│
├── scenes/                  Composite scene configs
│   ├── go2_room_nova.xml    Go2 robot in room environment
│   └── indoor_office.xml    Office with furniture + obstacles
│
├── robots/                  Robot model definitions
│   ├── go2/                 Unitree Go2 (RL policy)
│   └── nova_dog/            NOVA Dog / Thunder (Brainstem policy)
│
├── robot/                   Legacy robot MJCF
│   ├── thunder.urdf         Thunder quadruped URDF
│   └── thunder_meshes/      Collision/visual meshes
│
├── sensors/                 Sensor simulation
│   └── livox_mid360.py      Livox Mid-360 LiDAR (pure Python mj_multiRay fallback)
│
├── bridge/                  Physics ↔ Nav stack bridges
│   ├── mujoco_ros2_bridge.py    Full ROS2 bridge (odom, TF, PointCloud2, cmd_vel)
│   ├── mujoco_viz_bridge.py     Visualization bridge
│   └── nova_nav_bridge.py       Direct Python bridge (no ROS2 dependency)
│
├── following/               Person-following simulation
│   ├── behavior.py          FollowingBehavior FSM (FOLLOW/WAIT/SEARCH/EXPLORE/RECOVER)
│   ├── person/              Simulated person movement (RoomAwareWalk, waypoints)
│   ├── perception/          Simulated detection + tracking
│   ├── controller/          Following controllers (PurePursuit, PID, predictive)
│   ├── metrics/             Benchmark metrics (distance, tracking loss, latency)
│   └── interfaces.py        Abstract interfaces
│
├── semantic/                Semantic navigation simulation
│   └── factory_stub_test.py Factory floor semantic test
│
├── datasets/                LiDAR/IMU datasets for offline testing
│   ├── Avia/                Livox Avia dataset
│   └── legkilo*/            Legged-robot kinematic-inertial-LiDAR datasets
│
├── scripts/                 Utilities & demos
│   ├── go1_indoor_nav.py    Go1 indoor navigation demo
│   ├── go1_nav_full.py      Go1 full nav stack demo
│   ├── demo_search.py       Semantic search demo
│   ├── benchmark_following.py  Person-following benchmark
│   ├── gen_terrain_mesh.py  Generate spiral terrain .stl mesh
│   ├── install_deps.sh      Install simulation dependencies
│   └── factory_demo/        Factory floor demo scripts
│
├── launch/                  ROS2 launch files
│   ├── sim.launch.py        Full simulation launch
│   └── sim_full.sh          Shell wrapper for full stack
│
├── assets/                  Generated assets
│   └── meshes/              Terrain meshes (.stl)
│
├── maps/                    Saved simulation maps
│
├── output/                  Demo videos & benchmark results
│   ├── demo_*.mp4           Navigation demos
│   └── benchmark/           Following benchmark data
│
└── configs/                 Simulation configs
```

## LiDAR Simulation

Two approaches:

| Approach | Project | How it works | GPU | Use case |
|----------|---------|-------------|-----|----------|
| **A (default)** | [mujoco_ray_caster](https://github.com/Albusgive/mujoco_ray_caster) | MuJoCo C++ sensor plugin, native `mj_ray()` | No | General simulation |
| **B (large-scale)** | [OmniPerception](https://github.com/aCodeDog/OmniPerception) | Warp/CUDA GPU ray tracing, Livox pattern | CUDA 11+ | RL training, batch sim |

### Building mujoco_ray_caster

```bash
git clone https://github.com/google-deepmind/mujoco.git
cd mujoco/plugin
git clone https://github.com/Albusgive/mujoco_ray_caster.git

# Add to CMakeLists.txt: add_subdirectory(plugin/mujoco_ray_caster)
cd .. && mkdir build && cd build
cmake .. && cmake --build . -j8

# Install plugin
mkdir -p bin/mujoco_plugin && cp ../lib/libray_caster*.so ./mujoco_plugin/
export MUJOCO_PLUGIN_PATH=$(pwd)/mujoco_plugin
```

## Worlds

| World | Description | Requires mesh gen? |
|-------|-------------|-------------------|
| `open_field.xml` | Flat ground, quick validation | No |
| `spiral_terrain.xml` | 4-layer spiral ramp, elevation changes | Yes (`gen_terrain_mesh.py`) |
| `building_scene.xml` | Multi-room indoor building | No |
| `factory_scene.xml` | Factory warehouse with obstacles | No |

## Person Following

The `following/` module provides a complete person-following simulation pipeline:

```bash
# Run following benchmark
python sim/scripts/benchmark_following.py

# Results: PurePursuit + velocity prediction is best (0.14m stop-walk-stop error)
```

**FSM states**: FOLLOW → WAIT → SEARCH → EXPLORE → RECOVER

**Controllers tested**:
- PurePursuit (baseline)
- PurePursuit + velocity prediction (best)
- PID with lookahead
- Predictive MPC

## ROS2 Topics

| Topic | Type | Direction |
|-------|------|-----------|
| `/mujoco/pos_w_pointcloud` | PointCloud2 | MuJoCo → nav stack |
| `/nav/odometry` | Odometry | MuJoCo → nav stack |
| `/nav/cmd_vel` | TwistStamped | nav stack → MuJoCo |
| `tf` (map→odom→body) | TF2 | MuJoCo → nav stack |

## Running with LingTu

```bash
# Option 1: Full ROS2 bridge
source /opt/ros/humble/setup.bash
ros2 launch sim/launch/sim.launch.py world:=building_scene

# Option 2: Direct Python (no ROS2)
python lingtu.py sim    # Uses nova_nav_bridge.py internally

# Send navigation goal
> go 5 3                # In REPL
> go 找到餐桌            # Semantic goal
```

## Dependencies

- MuJoCo >= 3.0 (with ray_caster plugin for LiDAR)
- Python: `mujoco numpy scipy`
- ROS2 Humble (optional): `sensor_msgs nav_msgs tf2_ros`
- GPU (optional): `warp-lang` for OmniPerception LiDAR
