# PCT Planner

## Overview



---

## 1. 快速开始

### 1.1 编译安装

```bash
cd planner/
./build_thirdparty.sh
./build.sh
```

### 1.2 准备地图（通过 PGO 保存）
**推荐使用 PGO 的 `/pgo/save_maps` 服务保存地图**，这样可以获得回环优化后的高质量地图。
#### 使用一体化建图 Launch（推荐）
```bash
# 1. 启动完整的建图流程（LiDAR + Fast-LIO2 + PGO）
ros2 launch PCT_planner mapping_launch.py
# 2. 在环境中移动机器人，等待 PGO 检测到回环
#    观察 /pgo/loop_markers 话题确认回环检测
# 3. 保存优化后的地图
ros2 service call /pgo/save_maps interface/srv/SaveMaps \
  "{file_path: '/path/to/save_dir', save_patches: true}"

# 4. 复制地图到 PCT_planner
#    PGO 会保存：map.pcd, poses.txt, patches/目录
cp /path/to/save_dir/map.pcd rsc/pcd/my_map.pcd
```

#### 方式 B: 手动分别启动

```bash
# 1. 分别启动各个节点
ros2 launch livox_ros_driver2 msg_MID360_launch.py
ros2 launch fastlio2 lio_launch.py
ros2 launch pgo pgo_launch.py

# 2. 保存地图（同上）
ros2 service call /pgo/save_maps interface/srv/SaveMaps \
  "{file_path: '/path/to/save_dir', save_patches: true}"
```

**为什么不使用 `/save_map` (Fast-LIO2)？**
- `/save_map` 保存的是**未优化**的原始点云
- `/pgo/save_maps` 保存的是**回环优化后**的地图，漂移更小，更适合全局规划
- PGO 保存的地图包含完整的位姿图信息，质量更高

### 1.3 选择启动方式

#### 方式 A: 离线预处理（推荐，启动最快）

```bash
# 1. 离线构建 Tomogram
python3 tomography/scripts/tomography.py --scene Common
# 生成: rsc/tomogram/Common.pickle

# 2. 在线加载运行（使用 .pickle）
ros2 run pct_planner global_planner --ros-args -p map_file:="Common.pickle"
```

#### 方式 B: 在线实时构建（方便，首次启动较慢）

```bash
# 直接从 PCD 构建并运行（无需预处理）
ros2 run pct_planner global_planner --ros-args -p map_file:="my_map.pcd"

# 首次运行会自动构建并缓存 .pickle，后续启动加速
```

---

## 2. 系统架构

```
┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐
│   Tomography    │────▶│  Global Planner │────▶│  Path Adapter   │
│  (离线/在线)     │     │  (ROS 2 Node)   │     │ (pct_adapters)  │
└─────────────────┘     └─────────────────┘     └─────────────────┘
         │                       │                       │
         │ .pickle/PCD          │ /pct_path            │ /way_point
         │                       │                       │
         ▼                       ▼                       ▼
   rsc/tomogram/           RViz显示目标点          Local Planner
```

### 模块说明

| 模块 | 功能 | 启动方式 |
|------|------|---------|
| **tomography** | 将 PCD → Tomogram（多层地图） | `python3 tomography.py --scene XXX` |
| **global_planner** | 加载地图，接收目标点，规划全局路径 | `ros2 run pct_planner global_planner` |
| **pct_adapters** | 将全局路径转换为局部航点序列 | `ros2 run pct_adapters pct_path_adapter` |

---

## 3. 详细使用指南

### 3.1 场景配置 (--scene)

`--scene` 参数选择一套预设的配置（PCD文件、分辨率、可穿越性参数等）。

#### 使用内置场景

```bash
# 通用平地场景
python3 tomography/scripts/tomography.py --scene Common

# 楼梯场景（更陡的坡度容忍）
python3 tomography/scripts/tomography.py --scene Stairs

# 室内地板
python3 tomography/scripts/tomography.py --scene Floor
```

**内置场景列表**:

| 场景名 | 适用环境 | 关键参数特点 |
|--------|---------|-------------|
| `Common` | 通用平地 | 默认配置 |
| `Stairs` | 楼梯场景 | `slope_max=0.60`, `step_max=0.2` |
| `Floor` | 室内地板 | `slope_max=0.40`, `step_max=0.17` |
| `Room` | 小房间 | 安全边界较小 |
| `Plaza` | 广场 | `slope_max=0.36` |
| `Building` | 建筑物 | 默认配置 |
| `Spiral` | 螺旋路径 | `step_max=0.30`, 前瞻距离大 |

#### 创建自定义场景

1. **准备PCD文件**
   ```bash
   cp your_map.pcd rsc/pcd/my_scene.pcd
   ```

2. **创建配置文件** (`tomography/config/scene_myscene.py`)
   ```python
   from .scene import ScenePCD, SceneMap, SceneTrav

   class SceneMyscene():
       pcd = ScenePCD()
       pcd.file_name = 'my_scene.pcd'    # PCD文件名

       map = SceneMap()
       map.resolution = 0.10              # 地图分辨率(m)
       map.ground_h = 0.0                 # 地面高度(m)
       map.slice_dh = 0.5                 # 层高间隔(m)

       trav = SceneTrav()
       trav.kernel_size = 7               # 可穿越性分析核大小
       trav.slope_max = 0.40              # 最大坡度(tan值)
       trav.step_max = 0.20               # 最大台阶高度(m)
       trav.standable_ratio = 0.20        # 可站立比例阈值
       trav.cost_barrier = 50.0           # 障碍代价值
       trav.safe_margin = 0.4             # 安全边界(m)
       trav.inflation = 0.2               # 膨胀系数(m)
   ```

3. **注册场景** (`tomography/config/__init__.py`)
   ```python
   from .scene_myscene import SceneMyscene
   ```

4. **运行**
   ```bash
   python3 tomography/scripts/tomography.py --scene Myscene
   ```

### 3.2 ROS 2 参数配置

`global_planner` 支持通过 ROS 参数动态配置：

```bash
ros2 run pct_planner global_planner --ros-args \
  -p map_file:="Common.pickle" \
  -p map_frame:="map" \
  -p robot_frame:="body" \
  -p min_plan_interval:=2.0 \
  -p default_goal_height:=0.0 \
  -p tomogram_resolution:=0.1 \
  -p tomogram_slice_dh:=0.5 \
  -p publish_map_pointcloud:=true \
  -p publish_tomogram:=true
```

**参数说明**:

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `map_file` | string | - | Tomogram文件(.pickle)或PCD文件(.pcd) |
| `map_frame` | string | "map" | 地图坐标系 |
| `robot_frame` | string | "body" | 机器人坐标系 |
| `min_plan_interval` | float | 2.0 | 最小规划间隔(秒)，防止连续触发 |
| `default_goal_height` | float | 0.0 | 默认目标高度(m) |
| `tomogram_resolution` | float | 0.2 | Tomogram分辨率(仅PCD构建时生效) |
| `tomogram_slice_dh` | float | 0.5 | 层高间隔(仅PCD构建时生效) |
| `tomogram_ground_h` | float | 0.0 | 地面高度(仅PCD构建时生效) |
| `publish_map_pointcloud` | bool | false | 是否发布原始地图点云 |
| `publish_tomogram` | bool | false | 是否发布Tomogram可视化 |

### 3.3 交互式规划

启动后，在 RViz 中：

1. **设置目标点**: 点击 "2D Goal Pose" 工具，在地图上选择目标位置
2. **查看路径**: 规划的 `/pct_path` 会自动发布
3. **查看状态**: 订阅 `/pct_planner/status` 查看规划状态

**话题接口**:

| 话题名 | 类型 | 方向 | 说明 |
|--------|------|------|------|
| `/goal_pose` | PoseStamped | Sub | RViz 2D Goal Pose |
| `/clicked_point` | PointStamped | Sub | RViz Clicked Point |
| `/pct_path` | Path | Pub | 规划的全局路径 |
| `/pct_planner/status` | String | Pub | 规划状态 (IDLE/PLANNING/SUCCESS/FAILED) |
| `/map_pointcloud` | PointCloud2 | Pub | 原始地图点云(可选) |
| `/tomogram` | PointCloud2 | Pub | Tomogram可视化(可选) |

---

## 4. 系统集成

### 4.1 与局部规划器集成

PCT Planner 作为全局规划器，需要通过 `pct_adapters` 与 `local_planner` 联动：

```bash
# 终端1: 启动全局规划
ros2 run pct_planner global_planner --ros-args -p map_file:="Common.pickle"

# 终端2: 启动路径适配器
ros2 run pct_adapters pct_path_adapter

# 终端3: 启动局部规划器
ros2 launch local_planner local_planner.launch
```

**数据流**:
```
RViz 2D Goal ──▶ global_planner ──▶ /pct_path ──▶ pct_adapters ──▶ /way_point ──▶ local_planner
```

### 4.2 完整导航启动流程

```bash
# === 阶段1: SLAM和定位 ===
# 方式1: 使用一体化建图launch（如果是建图阶段）
ros2 launch PCT_planner mapping_launch.py

# 方式2: 或分别启动（如果是运行阶段）
ros2 launch livox_ros_driver2 msg_MID360_launch.py
ros2 launch fastlio2 lio_launch.py
ros2 launch localizer localizer_launch.py

# 重定位到预建地图（使用 PGO 保存的地图）
ros2 service call /relocalize interface/srv/Relocalize \
  "{pcd_path: '/path/to/pgo_save_dir/map.pcd', x: 0.0, y: 0.0, z: 0.0, yaw: 0.0, pitch: 0.0, roll: 0.0}"

# === 阶段2: 规划和控制 ===
ros2 launch terrain_analysis terrain_analysis.launch
ros2 launch local_planner local_planner.launch

# === 阶段3: 全局规划 ===
ros2 run pct_planner global_planner --ros-args -p map_file:="Common.pickle"
ros2 run pct_adapters pct_path_adapter

# === 阶段4: 在RViz中点击目标点 ===
```

---

## 5. 故障排查

### 问题1: `ModuleNotFoundError: No module named 'config'`

**解决**: 确保在 `tomography/scripts/` 目录下运行，或设置 PYTHONPATH
```bash
export PYTHONPATH=$PYTHONPATH:$(pwd)/tomography/config
```

### 问题2: `No .pickle found; building tomogram from PCD`

**说明**: 这是正常行为，首次从 PCD 构建需要几秒到几十秒。会自动缓存 `.pickle` 文件。

**加速**: 建议先离线运行 `tomography.py` 生成 `.pickle`，再在线加载。

### 问题3: 地图质量差，规划路径不准确

**可能原因**: 使用了 Fast-LIO2 的 `/save_map` 保存的未优化地图。

**解决**: 使用 PGO 的 `/pgo/save_maps` 保存回环优化后的地图：
```bash
# 错误：保存的是未优化地图
ros2 service call /save_map interface/srv/SaveMaps "{file_path: '/path/map.pcd'}"

# 正确：保存的是优化后的地图
ros2 service call /pgo/save_maps interface/srv/SaveMaps \
  "{file_path: '/path/save_dir', save_patches: true}"
```

### 问题3: TF 变换失败

**检查**:
```bash
ros2 run tf2_tools view_frames
# 确保 map → body 的变换链完整
```

### 问题4: 规划失败 (status: FAILED)

**可能原因**:
- 起点或终点在障碍物中
- 起点和终点之间无可连通路径
- Tomogram 参数不合适（如 `step_max` 太小）

**调试**:
```bash
# 查看Tomogram可视化
ros2 param set /pct_global_planner publish_tomogram true
# 在RViz中检查 /tomogram 话题
```

---

## 6. 文件组织

```
PCT_planner/
├── planner/
│   ├── scripts/
│   │   ├── global_planner.py      # ROS 2 规划节点
│   │   └── planner_wrapper.py     # TomogramPlanner 封装
│   ├── lib/                        # C++ 库 (A*, 轨迹优化)
│   └── config/param.py            # ROS 参数配置
│
├── tomography/
│   ├── scripts/
│   │   ├── tomography.py          # 离线/在线 Tomogram 构建
│   │   └── build_tomogram.py      # PCD → Tomogram 核心逻辑
│   └── config/
│       ├── scene.py               # 基础配置类
│       ├── scene_common.py        # 通用场景
│       ├── scene_stairs.py        # 楼梯场景
│       └── ...                    # 其他场景
│
├── rsc/
│   ├── pcd/                        # PCD地图文件
│   └── tomogram/                   # 生成的 .pickle 文件
│
└── pct_adapters/                   # 路径适配器（单独包）
    └── pct_path_adapter.py        # 全局→局部桥梁
```

---

## 7. Citing

If you use PCT Planner, please cite:

```bibtex
@ARTICLE{yang2024efficient,
  author={Yang, Bowen and Cheng, Jie and Xue, Bohuan and Jiao, Jianhao and Liu, Ming},
  journal={IEEE/ASME Transactions on Mechatronics}, 
  title={Efficient Global Navigational Planning in 3-D Structures Based on Point Cloud Tomography}, 
  year={2024},
  volume={},
  number={},
  pages={1-12}
}
```

---

## 8. License

The source code is released under [GPLv2](http://www.gnu.org/licenses/) license.

For commercial use, please contact Bowen Yang [byangar@connect.ust.hk](mailto:byangar@connect.ust.hk).
