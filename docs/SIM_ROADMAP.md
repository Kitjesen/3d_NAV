# MapPilot 仿真闭环构建指南

## 目标

```
MuJoCo 物理 → LiDAR 射线追踪 → terrain_analysis → local_planner → cmd_vel → MuJoCo
     ↑                                                                          |
     └──────────────────────────────────────────────────────────────────────────┘
```

一个完整的闭环仿真，能测真实障碍碰撞、坡度地形、slopeWeight 参数、stuck 检测。

---

## 现状盘点

### 已有代码（sim/ 目录）

| 文件 | 功能 | 状态 |
|------|------|------|
| `bridge/mujoco_ros2_bridge.py` | MuJoCo↔ROS2 桥接（odom/TF/点云/cmd_vel） | ✅ 完整，但 `_apply_cmd()` 是空壳 |
| `sensors/livox_mid360.py` | LiDAR：C++ plugin 读取 + Python `mj_multiRay` fallback | ✅ 完整 |
| `scripts/run_sim.py` | 主入口（headless/GUI/no-ros） | ✅ 完整 |
| `launch/sim.launch.py` | ROS2 全栈 launch（7 节点） | ✅ 完整 |
| `worlds/open_field.xml` | 平地 + 5 障碍物 + LiDAR 配置 | ✅ 可用 |
| `worlds/building_scene.xml` | 2 层楼 + 14 级楼梯 | ✅ 可用 |
| `worlds/factory_scene.xml` | 4 层工厂 + 坡道 + 楼梯 | ✅ 可用 |

### 断点（阻止闭环的 3 件事）

| # | 问题 | 说明 |
|---|------|------|
| **B1** | `_apply_cmd()` 为空 | bridge 收到 `/nav/cmd_vel` 但没注入 MuJoCo，机器人不动 |
| **B2** | 点云坐标系不对 | bridge 发 `map` frame，terrain_analysis 实际能接受（仿真中 map=odom），但需确认 |
| **B3** | LiDAR C++ plugin 未编译 | Python fallback 可用但慢（~50ms/scan），不阻塞但影响性能 |

**结论**: 只要修 B1（~15 行代码）+ 确认 B2（可能零改动），就能闭环。B3 是性能优化，不阻塞。

---

## mujoco_ray_caster 是什么

[github.com/Albusgive/mujoco_ray_caster](https://github.com/Albusgive/mujoco_ray_caster) — MuJoCo 原生射线追踪传感器插件。

| 特性 | 说明 |
|------|------|
| 原理 | MuJoCo C++ `mj_ray()` 封装成 plugin，在 XML 里一行配置 |
| 传感器类型 | ray_caster（基础）、ray_caster_camera（深度相机）、ray_caster_lidar（LiDAR） |
| 数据输出 | 距离、世界坐标点云（pos_w）、体坐标点云（pos_b）、加噪版本 |
| 多线程 | `num_thread` 配置，4 线程 ~5ms/scan |
| ROS2 Demo | `demo/ROS2/colcon/` 目录，colcon build 直接用 |
| MuJoCo 版本 | 需匹配源码版本，高级特性需 ≥3.5.0 |
| DDS 注意 | 文档说用 cyclonedds-cpp，不要用 fastdds |

### 编译方式

```bash
# 1. 克隆 MuJoCo 源码
git clone https://github.com/google-deepmind/mujoco.git
cd mujoco

# 2. 在 plugin 目录下克隆 ray_caster
cd plugin
git clone https://github.com/Albusgive/mujoco_ray_caster.git

# 3. 修改 MuJoCo 顶层 CMakeLists.txt，在 plugin 列表末尾加：
#    add_subdirectory(plugin/mujoco_ray_caster)

# 4. 编译
cd .. && mkdir build && cd build
cmake .. && cmake --build . -j$(nproc)

# 5. 部署 plugin .so
cd bin && mkdir -p mujoco_plugin
cp ../lib/libray_caster*.so ./mujoco_plugin/
export MUJOCO_PLUGIN_PATH=$(pwd)/mujoco_plugin
```

### 但是！Python fallback 已经能用

我们的 `sensors/livox_mid360.py` 里 `LivoxMid360Fallback` 类用纯 Python `mj_multiRay` 实现了同样的功能：
- 6400 rays/frame，黄金角螺旋模式模拟 Livox 非重复扫描
- 高斯噪声 σ=0.02m
- 不需要编译任何 C++ 代码

**所以第一步不需要编译 ray_caster plugin**，直接用 Python fallback 跑通闭环，之后再换 C++ 提性能。

---

## 构建步骤

### Step 0 — 选择验证平台

| 平台 | MuJoCo | ROS2 | terrain_analysis (C++) | 推荐用途 |
|------|--------|------|----------------------|---------|
| **S100P (aarch64)** | `pip install mujoco` | Humble ✅ | 已编译 | **首选：全栈闭环** |
| **Docker x86** | ✅ | Humble ✅ | 需 colcon build | CI headless |
| **Windows 开发机** | ✅ | ❌ | ❌ | 仅 viewer 看场景 |

推荐在 S100P 上验证，因为 terrain_analysis / local_planner / pathFollower 已经编译好了。

### Step 1 — 修 `_apply_cmd()` 让机器人能动

```python
# sim/bridge/mujoco_ros2_bridge.py
# 把空壳 _apply_cmd() 改为 qvel 注入

def _apply_cmd(self):
    """将 cmd_vel 注入 MuJoCo freejoint 速度。"""
    if time.time() - self._cmd_ts > self.CMD_TIMEOUT:
        self._cmd_vx = self._cmd_vy = self._cmd_wz = 0.0

    import mujoco
    bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, self.robot_body)

    # 从四元数提取 yaw
    q = self.data.xquat[bid]  # [w, x, y, z]
    yaw = np.arctan2(2.0 * (q[0]*q[3] + q[1]*q[2]),
                     1.0 - 2.0 * (q[2]**2 + q[3]**2))

    # body frame → world frame
    c, s = np.cos(yaw), np.sin(yaw)
    vx_w = self._cmd_vx * c - self._cmd_vy * s
    vy_w = self._cmd_vx * s + self._cmd_vy * c

    # freejoint: qvel[0:3] = 线速度 (world), qvel[3:6] = 角速度 (world)
    jnt_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, 'root')
    adr = self.model.jnt_dofadr[jnt_id]
    self.data.qvel[adr + 0] = vx_w
    self.data.qvel[adr + 1] = vy_w
    self.data.qvel[adr + 2] = 0.0      # 保持 Z 速度为 0（不飞）
    self.data.qvel[adr + 3] = 0.0
    self.data.qvel[adr + 4] = 0.0
    self.data.qvel[adr + 5] = self._cmd_wz
```

**原理**: MuJoCo freejoint 有 6 个自由度（3 平移 + 3 旋转），直接写 `qvel` 就相当于给机器人施加速度。不需要腿部关节模型。

### Step 2 — 确认点云坐标系

当前 bridge 发布点云 frame_id = `map`（世界坐标系）。terrain_analysis 订阅 `/cloud_map`，期望 odom 坐标系。

仿真中 map = odom（static TF identity），所以 **frame_id 改为 `odom` 即可**：

```python
# sim/bridge/mujoco_ros2_bridge.py  _pub_cloud() 中：
msg = _pack_pointcloud2(pts, 'odom', stamp)   # 原来是 'map'
```

### Step 3 — 确认 launch remap

`sim/launch/sim.launch.py` 已配好：
```python
# terrain_analysis remap:
('/cloud_map', '/livox/lidar'),    # bridge 发到 /livox/lidar ✅
```

但需要修一个问题：`run_sim.py` 里 `MuJoCoROS2Bridge` 构造函数参数名不匹配。

```python
# run_sim.py:90  当前：
bridge = MuJoCoROS2Bridge(model, data, lidar=lidar, ...)
# bridge __init__ 参数名是 lidar_sensor，不是 lidar
# 需要确认 bridge 实际用的参数名，可能需要改为：
bridge = MuJoCoROS2Bridge(model, data, lidar_sensor=lidar, ...)
```

另外 `run_sim.py` 用 `LivoxMid360SimVectorized`（不存在），实际类名是 `LidarSensor`：

```python
# run_sim.py:76  改为：
from sensors.livox_mid360 import LidarSensor
lidar = LidarSensor(model, data, body_name='lidar_link')
```

### Step 4 — 在 S100P 上验证

```bash
# SSH 到 S100P
ssh sunrise@192.168.66.190

# 安装 mujoco（如果还没装）
pip install mujoco

# 同步代码
cd ~/lingtu

# 确保 ROS2 nav 栈已编译
source /opt/ros/humble/setup.bash
source install/setup.bash

# 方式 A：只启动 MuJoCo + bridge（手动启动 nav 栈其他节点）
python sim/scripts/run_sim.py --world open_field --headless

# 方式 B：全栈 launch（terrain_analysis + local_planner + pathFollower 一起启动）
ros2 launch sim/launch/sim.launch.py world:=open_field
```

### Step 5 — 验证闭环

```bash
# Terminal 2: 检查数据流
ros2 topic hz /livox/lidar          # 期望 ~10 Hz, 6400 pts/frame
ros2 topic hz /nav/odometry         # 期望 ~50 Hz
ros2 topic hz /nav/terrain_map      # 期望 ~2 Hz (terrain_analysis 输出)
ros2 topic hz /nav/cmd_vel          # pathFollower 输出

# Terminal 3: 键盘遥控测试（先不发 goal，手动控制看机器人动不动）
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args --remap cmd_vel:=/nav/cmd_vel

# Terminal 4: 发导航目标
ros2 topic pub --once /nav/goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 6.0, y: 0.0, z: 0.5}}}"
```

**成功标志**: 机器人从 (0,0) 出发，绕过 box1 (6,0,0.8)，到达 (6,0) 附近。`/nav/planner_status` 发布 `GOAL_REACHED`。

---

## 场景升级路径

### Level 1: open_field（平地 + 障碍物）

验证基础避障。terrain_analysis 看到 box/cylinder/wall 并标记为障碍物。local_planner 绕行。

### Level 2: building_scene（两层楼 + 楼梯）

```
起点: (2, 3, 0.5) → 1F 大厅
目标: (18, 11, 4.0) → 2F Room B
挑战: 14 级楼梯 (25cm/级)
```

需要：
1. 从 MuJoCo 采集建筑点云 → 保存 PCD
2. `build_tomogram_from_pcd()` 生成 tomogram
3. global_planner 加载 → 跨楼层 3D 规划

### Level 3: factory_scene（四层工厂）

```
起点: (5, 3, -2.0) → B1 卸货区
目标: (72, 52, 14.0) → RF 屋顶
路径: B1 → 坡道 R1 → G0 → 楼梯 S1 → M1 → S3 → M2 → S4 → RF
垂直: 16.5m  水平: ~100m
```

终极验证。跑通这个 = 3D 导航栈就绪。

---

## 对标 FSGP_BGK

[ZJU-FAST-Lab/FSGP_BGK](https://github.com/ZJU-FAST-Lab/FSGP_BGK)（IROS 2025 Oral）的仿真环境：

| 项目 | FSGP_BGK | 我们 |
|------|----------|------|
| 物理引擎 | 未明确（回放 PCD） | MuJoCo |
| LiDAR | 预录/程序生成点云 | mj_multiRay 实时射线追踪 |
| 地形生成 | EPFL terrain generator | MuJoCo XML 手动建模 |
| 场景复杂度 | 丘陵/森林（单层） | 多层建筑/工厂（3D） |
| 可通行性 | SGP + BGK（论文核心） | 体素栅格 + 分位数（terrain_analysis.cpp） |
| ROS2 | Humble colcon | Humble colcon |

**他们的仿真环境比我们简单** — 主要是回放 PCD 测试 BGK 算法，不是物理闭环仿真。

**他们的 BGK 算法比我们强** — 稀疏高斯过程 + 贝叶斯推断 vs 我们的体素栅格。可以作为 terrain_analysis 的升级方案：

```
# 并行运行，local_planner 通过参数选择数据源
terrain_analysis (C++, 现有)  → /nav/terrain_map
FSGP_BGK (Python+GPU, 可选)  → /nav/terrain_map_bgk
```

集成方式：
```bash
cd ~/lingtu
git clone https://github.com/ZJU-FAST-Lab/FSGP_BGK.git third_party/FSGP_BGK
touch third_party/FSGP_BGK/src/simulation_env_ros1/COLCON_IGNORE
colcon build --packages-select fsgp_bgk

# 单独运行 BGK 节点（订阅 /livox/lidar，发布可通行性地图）
python third_party/FSGP_BGK/src/fsgp_bgk/python/node_ros2.py
```

---

## LiDAR 性能升级路径

| 方案 | 实现 | 性能 | 适用 |
|------|------|------|------|
| **Python fallback（当前）** | `LivoxMid360Fallback` — `mj_multiRay` | ~50ms/scan, 6400 pts | Phase 1 验证 |
| **mujoco_ray_caster C++** | 编译 plugin → XML 自动加载 | ~5ms/scan, 可开到 40k pts | Phase 2 长期 |
| **OmniPerception GPU** | Warp/CUDA batch ray tracing | <1ms/scan, 1000+ 环境并行 | Phase 4 RL 训练 |

Python fallback 够用，不急着编译 C++ plugin。

如果要编译 mujoco_ray_caster（在 S100P aarch64 上）：
```bash
# 确认 cmake 版本 >= 3.16
cmake --version

# 克隆 MuJoCo 源码（需匹配 pip install mujoco 的版本）
pip show mujoco  # 看版本号，比如 3.2.0
git clone --branch 3.2.0 --depth 1 https://github.com/google-deepmind/mujoco.git /tmp/mujoco_src
cd /tmp/mujoco_src/plugin
git clone https://github.com/Albusgive/mujoco_ray_caster.git

# 修改 CMakeLists.txt
echo 'add_subdirectory(plugin/mujoco_ray_caster)' >> /tmp/mujoco_src/CMakeLists.txt

# 编译（aarch64 上可能较慢，约 10-20 分钟）
cd /tmp/mujoco_src && mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release && cmake --build . -j4

# 部署
mkdir -p ~/mujoco_plugin
cp lib/libray_caster*.so ~/mujoco_plugin/
echo 'export MUJOCO_PLUGIN_PATH=~/mujoco_plugin' >> ~/.bashrc
```

编译成功后，`sim/worlds/open_field.xml` 里的 `<sensor plugin="mujoco.sensor.ray_caster_lidar">` 会自动生效，`sensors/livox_mid360.py` 的 `LidarSensor` 类会自动检测到 plugin 并切换到 C++ 模式。

---

## 待修改文件清单

| 文件 | 改动 | 行数 |
|------|------|------|
| `sim/bridge/mujoco_ros2_bridge.py` | `_apply_cmd()` qvel 注入 + `_pub_cloud` frame_id→`odom` | ~20 行 |
| `sim/scripts/run_sim.py` | 修 LiDAR 类名 `LivoxMid360SimVectorized` → `LidarSensor` | ~3 行 |
| `sim/launch/sim.launch.py` | （可能不改，remap 已配好）| 0 行 |

**总改动量: ~25 行代码。** 框架已经搭好了，只差把断点连上。
