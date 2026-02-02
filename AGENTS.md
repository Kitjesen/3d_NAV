# Navigation System - Agent Guide

## 1. 系统概述

本项目是一个完整的**野外/越野自主导航系统**，包含从感知到控制的完整链路：

- **SLAM**: Fast-LIO2 实时建图 + PGO 回环优化
- **定位**: Localizer 重定位模块（支持预加载地图）
- **感知**: 地形分析（地面估计、障碍物检测、可穿越性分析）
- **规划**: 全局规划（PCT_planner）+ 局部规划（base_autonomy）
- **驱动**: 机器人底盘控制接口

---

## 2. 系统架构图

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              感知层 (Perception)                              │
├─────────────────────────────────────────────────────────────────────────────┤
│  Livox LiDAR  ──▶  livox_ros_driver2  ──▶  /livox/lidar, /livox/imu        │
└─────────────────────────────────────────────────────────────────────────────┘
                                       │
                                       ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                              SLAM层 (SLAM & Localization)                     │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐       │
│  │   Fast-LIO2     │────▶│  sensor_scan    │────▶│  local_planner  │       │
│  │   (实时里程计)   │     │  _generation    │     │  (局部规划输入)  │       │
│  │                 │     │  (坐标转换)      │     │                 │       │
│  │ 输出:           │     │                 │     │                 │       │
│  │ /cloud_reg      │     │ 输出:            │     │                 │       │
│  │ /cloud_map      │     │ /sensor_scan    │     │                 │       │
│  │ /Odometry       │     │                 │     │                 │       │
│  └────────┬────────┘     └─────────────────┘     └─────────────────┘       │
│           │                                                                 │
│           │  /cloud_map                                                     │
│           ▼                                                                 │
│  ┌─────────────────┐     优化后位姿    ┌─────────────────┐                 │
│  │      PGO        │◄────────────────▶│    Localizer    │                 │
│  │  (回环检测优化)  │                  │   (重定位模块)   │                 │
│  │                 │                  │                 │                 │
│  │ 服务:           │                  │ 服务:           │                 │
│  │ /pgo/save_maps  │                  │ /relocalize     │                 │
│  └─────────────────┘                  │ /relocalize_check│                │
│                                       └─────────────────┘                 │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
                                       │
                                       │ TF: map → odom
                                       │
                                       ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                           地形分析层 (Terrain Analysis)                       │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐       │
│  │ terrain_analysis│────▶│terrain_analysis_│────▶│  local_planner  │       │
│  │  (地面估计)      │     │     ext         │     │  (局部规划)      │       │
│  │                 │     │  (连通性检查)    │     │                 │       │
│  │ 输入:           │     │                 │     │                 │       │
│  │ /cloud_reg      │     │ 输入:           │     │                 │       │
│  │ /Odometry       │     │ /terrain_map    │     │                 │       │
│  │                 │     │                 │     │                 │       │
│  │ 输出:           │     │ 输出:           │     │                 │       │
│  │ /terrain_map    │     │ /terrain_map_ext│     │                 │       │
│  └─────────────────┘     └─────────────────┘     └─────────────────┘       │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
                                       │
                                       ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                           规划层 (Planning)                                   │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │                        全局规划 (PCT_planner)                        │    │
│  │  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐             │    │
│  │  │  tomography │───▶│  Tomogram   │───▶│   Global    │             │    │
│  │  │  (层析成像)  │    │  (多层地图)  │    │  Planner    │             │    │
│  │  │             │    │             │    │  (A* + G)   │             │    │
│  │  └─────────────┘    └─────────────┘    └──────┬──────┘             │    │
│  │                                               │                     │    │
│  │         SceneTrav 可穿越性配置                 │                     │    │
│  │                                               ▼                     │    │
│  │                                      ┌─────────────┐                │    │
│  │                                      │ pct_adapters│                │    │
│  │                                      │ (ROS2接口)   │                │    │
│  │                                      └─────────────┘                │    │
│  └─────────────────────────────────────────────────────────────────────┘    │
│                                                                             │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │                        局部规划 (base_autonomy)                      │    │
│  │                                                                     │    │
│  │  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐             │    │
│  │  │local_planner│───▶│pathFollower │───▶│robot_driver │             │    │
│  │  │ (路径选择)   │    │ (路径跟踪)   │    │ (底盘控制)   │             │    │
│  │  │             │    │             │    │             │             │    │
│  │  │ 输入:        │    │             │    │             │             │    │
│  │  │ /terrain_map│    │             │    │             │             │    │
│  │  │ /way_point  │    │             │    │             │             │    │
│  │  │ /joy        │    │             │    │             │             │    │
│  │  └─────────────┘    └─────────────┘    └─────────────┘             │    │
│  └─────────────────────────────────────────────────────────────────────┘    │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 3. 启动顺序和依赖关系

### 3.1 启动顺序

```
阶段1: 基础驱动 (必须最先启动)
├── livox_ros_driver2          # LiDAR驱动
└── robot_driver               # 底盘驱动

阶段2: SLAM核心 (依赖驱动)
├── fastlio2                   # 实时里程计
└── sensor_scan_generation     # 点云坐标转换

阶段3: 高级SLAM (依赖SLAM核心)
├── pgo                        # 回环检测与优化 (可选，用于建图)
└── localizer                  # 重定位模块 (运行时必需)

阶段4: 感知与规划 (依赖SLAM)
├── terrain_analysis           # 地形分析
├── terrain_analysis_ext       # 扩展地形分析
└── local_planner              # 局部规划器

阶段5: 全局规划 (独立运行)
└── pct_planner                # 全局规划 (需预建地图)
```

### 3.2 依赖关系图

```
livox_ros_driver2
    │
    ├──▶ fastlio2
    │       │
    │       ├──▶ sensor_scan_generation
    │       │       └──▶ local_planner ◀── pct_adapters ◀── pct_planner
    │       │                              (闭环跟踪)
    │       ├──▶ pgo (可选)
    │       │       └──▶ TF: map → odom
    │       │
    │       └──▶ localizer
    │               └──▶ TF: map → odom
    │
    └──▶ terrain_analysis
            └──▶ terrain_analysis_ext
                    └──▶ local_planner

离线处理 (建图后):
PCD地图文件 ──▶ tomography ──▶ Tomogram (.pickle) ──▶ pct_planner (运行时加载)
```

### 3.3 启动命令示例

#### 建图模式 (首次探索)
```bash
# 终端1: 驱动 + SLAM
ros2 launch livox_ros_driver2 msg_MID360_launch.py
ros2 launch fastlio2 lio_launch.py

# 终端2: 回环优化 (可选但推荐)
ros2 launch pgo pgo_launch.py

# 保存地图
ros2 service call /save_map interface/srv/SaveMaps "{file_path: '/path/to/map.pcd'}"
```

#### 离线处理 (生成Tomogram)
```bash
# 将保存的 map.pcd 复制到 PCT_planner/rsc/pcd/
cd src/global_planning/PCT_planner

# 运行层析成像 (生成 .pickle 文件)
python3 tomography/scripts/tomography.py --scene Common
# 输出: rsc/tomogram/Common.pickle
```

#### 运行模式 (定位+规划)
```bash
# 终端1: 驱动 + SLAM
ros2 launch livox_ros_driver2 msg_MID360_launch.py
ros2 launch fastlio2 lio_launch.py

# 终端2: 重定位 (加载预建地图)
ros2 launch localizer localizer_launch.py
ros2 service call /relocalize interface/srv/Relocalize \
  "{pcd_path: '/path/to/map.pcd', x: 0.0, y: 0.0, z: 0.0, yaw: 0.0, pitch: 0.0, roll: 0.0}"

# 终端3: 地形分析和局部规划
ros2 launch terrain_analysis terrain_analysis.launch
ros2 launch terrain_analysis_ext terrain_analysis_ext.launch
ros2 launch local_planner local_planner.launch

# 终端4: 全局规划 + 路径适配
ros2 launch PCT_planner planner_only_launch.py  # 或 python3 planner/scripts/global_planner.py
ros2 run pct_adapters pct_path_adapter

# 设置目标点 (通过RViz或命令行)
ros2 topic pub /way_point geometry_msgs/PointStamped \
  "{header: {frame_id: 'map'}, point: {x: 10.0, y: 0.0, z: 0.0}}"
```

---

## 4. 坐标系定义

### 4.1 坐标系层级

```
map (全局地图坐标系) - 固定不变的世界参考系
    └── odom (里程计坐标系) - 由PGO或Localizer发布
        └── body (机器人本体坐标系) - Fast-LIO2输出
            └── lidar (激光雷达坐标系) - 传感器原始坐标系
```

### 4.2 坐标系详细说明

| 坐标系 | 名称 | 定义 | 发布者 | 备注 |
|--------|------|------|--------|------|
| **map** | 地图坐标系 | 全局固定坐标系 | PGO/Localizer | 经过回环优化或重定位的统一坐标系 |
| **odom** | 里程计坐标系 | 局部平滑坐标系 | PGO/Localizer | 与map的偏移量实时更新 |
| **body** | 本体坐标系 | IMU中心 | Fast-LIO2 | 机器人本体参考点 |
| **lidar** | 雷达坐标系 | LiDAR传感器中心 | - | 通过外参与body关联 |
| **sensor_at_scan** | 扫描时刻坐标系 | 历史扫描位置 | sensor_scan_generation | 用于点云对齐 |

### 4.3 TF变换关系

```
map → odom: 由 PGO 或 Localizer 发布
  - PGO: 基于回环检测的偏移量 (m_r_offset, m_t_offset)
  - Localizer: 基于ICP配准的偏移量 (last_offset_r, last_offset_t)

odom → body: 由 Fast-LIO2 发布 (/Odometry)
  - 实时高频更新 (100Hz)
  - 包含位置和姿态

body → lidar: 静态变换 (外参)
  - 通过配置文件 lio.yaml 中的 r_il, t_il 定义
```

### 4.4 关键配置参数

```yaml
# src/slam/fastlio2/config/lio.yaml
body_frame: "body"      # 机器人本体坐标系
world_frame: "odom"     # 里程计坐标系

# src/slam/pgo/config/pgo.yaml
map_frame: "map"        # 全局地图坐标系
local_frame: "odom"     # 局部坐标系

# src/slam/localizer/config/localizer.yaml
map_frame: "map"
local_frame: "odom"
```

---

## 5. 话题和服务列表

### 5.1 核心话题 (Topics)

#### 输入话题

| 话题名 | 类型 | 发布者 | 订阅者 | 说明 |
|--------|------|--------|--------|------|
| `/livox/lidar` | CustomMsg | livox_ros_driver2 | fastlio2 | LiDAR原始点云 |
| `/livox/imu` | Imu | livox_ros_driver2 | fastlio2 | IMU数据 |
| `/joy` | Joy | 手柄驱动 | local_planner | 手动控制输入 |
| `/way_point` | PointStamped | pct_adapters/RViz | local_planner | 目标航点 |
| `/speed` | Float32 | 速度控制 | local_planner | 速度指令 |
| `/pct_path` | Path | pct_planner | pct_adapters | 全局路径 |

#### 输出话题

| 话题名 | 类型 | 发布者 | 订阅者 | 说明 |
|--------|------|--------|--------|------|
| `/cloud_registered` | PointCloud2 | fastlio2 | terrain_analysis, localizer | Body系点云 |
| `/cloud_map` | PointCloud2 | fastlio2 | - | 世界地图点云 |
| `/Odometry` | Odometry | fastlio2 | 所有模块 | 里程计信息 |
| `/terrain_map` | PointCloud2 | terrain_analysis | terrain_analysis_ext, local_planner | 地形地图 |
| `/terrain_map_ext` | PointCloud2 | terrain_analysis_ext | local_planner | 扩展地形地图 |
| `/path` | Path | local_planner | pathFollower | 规划路径 |
| `/free_paths` | PointCloud2 | local_planner | RViz | 可视化可用路径 |
| `/pct_path` | Path | pct_planner | pct_adapters | 全局规划路径 |
| `/way_point` | PointStamped | pct_adapters | local_planner | 当前目标航点 |
| `/slow_down` | Int8 | local_planner | pathFollower | 减速指令 (0-3级) |
| `/stop` | Int8 | 安全模块 | pathFollower | 紧急停止 |

#### TF变换

| 变换 | 发布者 | 频率 | 说明 |
|------|--------|------|------|
| `map → odom` | PGO/Localizer | 10-20Hz | 全局到局部坐标系 |
| `odom → body` | fastlio2 | 100Hz | 实时位姿 |
| `map → sensor_at_scan` | sensor_scan_generation | 与点云同步 | 扫描时刻坐标系 |

### 5.2 服务 (Services)

| 服务名 | 类型 | 提供者 | 说明 |
|--------|------|--------|------|
| `/relocalize` | Relocalize | Localizer | 加载地图并重定位 |
| `/relocalize_check` | IsValid | Localizer | 检查重定位状态 |
| `/pgo/save_maps` | SaveMaps | PGO | 保存优化后的地图和位姿 |
| `/save_map` | SaveMaps | fastlio2 | 保存当前地图 |

### 5.3 局部规划模块详解 (base_autonomy)

#### local_planner (路径选择器)

**功能**: 基于地形分析结果，从预定义路径库中选择最优路径，并发布减速指令

**路径库结构**:
```
36方向 × 343条 = 12,348条候选路径
    │
    ├── 36个方向 (每10°一个，覆盖360°)
    └── 每个方向7组不同曲率路径 (pathList_ 0-6)
```

**碰撞检测机制**:

使用预计算的 `correspondences_` 映射表实现 O(1) 查询：

```cpp
// 预计算：启动时加载
std::vector<int> correspondences_[gridVoxelNum_];
// 含义：网格ind被哪些路径穿过

// 运行时：遍历障碍物
for (each obstacle in terrain_map) {
    int ind = point_to_grid(obstacle);  // 转网格索引
    for (int path_id : correspondences_[ind]) {  // 查表！
        if (h > obstacleHeightThre_) {
            clearPathList_[path_id]++;  // 标记阻塞
        } else {
            pathPenaltyList_[path_id] = max(h);  // 记录惩罚
        }
    }
}
```

**减速分级逻辑**:

| 减速级别 | 触发条件 | 含义 | 速度比例 |
|---------|---------|------|---------|
| 1 | `penaltyScore > 0.15` | 地形代价高(大障碍物/陡坡) | 25% |
| 2 | `penaltyScore > 0.10` | 地形代价中等 | 50% |
| 3 | 可选路径<5且偏离中心 | 路径选择受限 | 75% |
| 0 | 其他 | 地形良好 | 100% |

**penaltyScore计算**:
```cpp
penaltyScore = pathPenaltyPerGroupScore_[group] / clearPathPerGroupNum_[group]
             = 平均障碍物高度 (米)
```

#### pathFollower (路径跟踪器)

**功能**: Pure Pursuit 路径跟踪，接收减速指令调整速度

**订阅话题**:
| 话题 | 类型 | 说明 |
|------|------|------|
| `/path` | Path | 选定的局部路径 |
| `/Odometry` | Odometry | 当前位姿 |
| `/joy` | Joy | 手动控制输入 |
| `/slow_down` | Int8 | 减速指令 (1=25%, 2=50%, 3=75%) |
| `/stop` | Int8 | 紧急停止 |

**核心参数**:
```cpp
lookAheadDis_ = baseLookAheadDis_ + lookAheadRatio_ * currentSpeed;  // 自适应前瞻
slowRate1_ = 0.25;  // 一级减速比例
slowRate2_ = 0.50;  // 二级减速比例
slowRate3_ = 0.75;  // 三级减速比例
```

---

### 5.4 全局规划模块详解

#### tomography (离线地图处理)

**功能**: 将PCD点云地图转换为多层可穿越性地图(Tomogram)

**工作流程**:
```
输入: PCD文件 (如 scans.pcd)
   ↓
处理: 层析成像 + 可穿越性分析 (SceneTrav参数)
   ↓
输出: 
   1. Tomogram文件 (.pickle) - 供pct_planner加载
   2. 可视化话题 (用于RViz查看)
```

**启动命令**:
```bash
# 离线处理 (在PCT_planner目录下)
python3 tomography/scripts/tomography.py --scene Common
# 或
python3 tomography/scripts/tomography.py --scene Stairs

# 可视化 (ROS2节点)
ros2 run pct_planner tomography --scene Common
```

**输出文件**:
- `rsc/tomogram/{scene_name}.pickle` - 多层地图数据

**可视化话题**:
| 话题 | 说明 |
|------|------|
| `/global_points` | 原始点云 (用于对齐) |
| `/layer_G_0` ~ `/layer_G_N` | 各层地面高度 |
| `/layer_C_0` ~ `/layer_C_N` | 各层可穿越性代价 |
| `/tomogram` | 合并的可视化点云 |

**配置参数** (`tomography/config/scene_*.py`):
```python
class SceneTrav():
    kernel_size = 7          # 可穿越性分析核大小
    interval_min = 0.50      # 最小层高间隔
    interval_free = 0.65     # 自由空间层高间隔
    slope_max = 0.36         # 最大坡度
    step_max = 0.20          # 最大台阶高度
    standable_ratio = 0.20   # 可站立比例阈值
    cost_barrier = 50.0      # 障碍代价值
    safe_margin = 0.4        # 安全边界
    inflation = 0.2          # 膨胀系数
```

---

#### pct_planner (全局规划器)

**功能**: 基于Tomogram进行全局路径规划

**输入**: 
- Tomogram文件 (.pickle)
- 起点/终点坐标

**输出**: `/pct_path` (Path消息，map坐标系)

**规划算法**: A* + G-value引导

---

#### pct_adapters (路径适配器)

**功能**: 桥接全局规划和局部规划，实现闭环跟踪

**核心逻辑**:
```
订阅 /pct_path (全局路径, map坐标系)
订阅 /Odometry (当前位姿, odom坐标系)
   ↓
TF变换: map → odom
   ↓
路径下采样 → 航点序列
   ↓
闭环控制: 到达当前航点才发布下一航点
   ↓
发布 /way_point (当前目标航点, odom坐标系)
```

**参数**:
| 参数 | 默认值 | 说明 |
|------|--------|------|
| `waypoint_distance` | 0.5 | 航点间距 (米) |
| `arrival_threshold` | 0.5 | 到达判定距离 (米) |
| `lookahead_dist` | 1.0 | 前瞻距离 (米) |

**启动命令**:
```bash
ros2 run pct_adapters pct_path_adapter
# 或
ros2 launch pct_adapters pct_adapter.launch.py
```

**为什么需要这个模块**:
- 全局规划器发布的是完整路径，局部规划器只需要当前目标点
- 实现"胡萝卜"引导机制，确保机器人沿全局路径 corridor 行驶
- 允许局部规划器在航点间自主避障，同时保持全局方向

---

### 5.5 服务详细定义

#### `/relocalize` (interface/srv/Relocalize)

**Request:**
```
string pcd_path    # 地图文件路径
float32 x          # 初始位置 X (m)
float32 y          # 初始位置 Y (m)
float32 z          # 初始位置 Z (m)
float32 yaw        # 偏航角 (rad)
float32 pitch      # 俯仰角 (rad)
float32 roll       # 横滚角 (rad)
```

**Response:**
```
bool success       # 是否成功
string message     # 返回信息
```

**调用示例:**
```bash
ros2 service call /relocalize interface/srv/Relocalize \
  "{pcd_path: '/path/to/map.pcd', x: 0.0, y: 0.0, z: 0.0, yaw: 0.0, pitch: 0.0, roll: 0.0}"
```

---

## 6. 工程化改进建议

### 6.1 高优先级

#### 1. 系统监控节点 (System Health Monitor)

```cpp
class SystemHealthMonitor : public rclcpp::Node {
    // 监控各模块心跳
    // /lio/status: OK/WARNING/ERROR
    // /terrain_analysis/status: OK
    // 检测到故障时触发恢复或紧急停止
};
```

**功能:**
- 监控各模块发布频率
- 检测TF变换是否超时
- 检测服务是否可用
- 发布 `/system_health` 话题

#### 2. 统一参数管理

```yaml
# 创建 navigation_params/config/system_params.yaml
system:
  coordinate_frames:
    map_frame: "map"
    odom_frame: "odom"
    body_frame: "body"
    
  vehicle:
    height: 1.5
    length: 0.6
    width: 0.6
    
  thresholds:
    obstacle_height: 0.2
    slope_max: 0.36
```

#### 3. 紧急停止机制 (E-Stop)

```cpp
class EmergencyStopNode : public rclcpp::Node {
    // 订阅安全相关话题
    // - 障碍物距离
    // - 定位方差
    // - 系统健康状态
    // 触发条件时发布 /emergency_stop
};
```

### 6.2 中优先级

#### 4. 状态机管理 (Behavior Tree)

```
[Idle] → [Initializing] → [Localization] → [Planning] → [Executing]
    ↑                                              ↓
    └──────────────── [EmergencyStop] ←───────────┘
```

建议框架: `BehaviorTree.CPP`

#### 5. 代码规范化

- 添加 `.clang-format` 统一代码风格
- 添加 `.clang-tidy` 静态检查
- 配置 GitHub Actions CI/CD

#### 6. 测试框架

```
test/
├── test_fastlio2.cpp        # LIO单元测试
├── test_terrain_analysis.py # 地形分析测试
├── test_local_planner.cpp   # 规划器测试
└── integration_test/        # 集成测试
```

### 6.3 低优先级

#### 7. 诊断工具

- 使用 `ros2 doctor` 扩展
- 添加 `/diagnostics` 话题
- RViz插件开发

#### 8. Docker 开发环境

```dockerfile
# Dockerfile.dev
FROM ros:foxy
# 安装依赖
# 挂载源码
# 配置开发环境
```

---

## 7. 常见问题排查

### 7.1 TF变换问题

**问题**: `map → odom` 未发布
- 检查 PGO 或 Localizer 是否启动
- 检查是否有回环检测成功 (PGO)
- 检查 `/relocalize` 是否成功调用 (Localizer)

### 7.2 规划失败

**问题**: local_planner 不发布路径
- 检查 `/terrain_map` 是否有数据
- 检查 `/way_point` 是否设置
- 检查手柄 `/joy` 是否发送速度指令

### 7.3 定位丢失

**问题**: 机器人位置跳变
- 检查 Fast-LIO2 是否正常运行
- 检查 Localizer 的 ICP 是否收敛
- 检查地图文件是否正确加载

---

## 8. 附录

### 8.1 参数速查

| 模块 | 配置文件 | 关键参数 |
|------|---------|---------|
| fastlio2 | `lio.yaml` | `body_frame`, `world_frame`, `r_il`, `t_il` |
| pgo | `pgo.yaml` | `map_frame`, `local_frame`, `loop_score_tresh` |
| localizer | `localizer.yaml` | `static_map_path`, `update_hz` |
| terrain_analysis | launch参数 | `obstacleHeightThre`, `slope_max` |
| local_planner | launch参数 | `vehicleHeight`, `vehicleWidth`, `adjacentRange` |

### 8.2 相关文档

- `docs/plan.md` - 开发计划
- `src/slam/interface/README.md` - 接口定义
- `src/base_autonomy/TERRAIN_ANALYSIS_EXPLAINED.md` - 地形分析详解

---

*最后更新: 2026-02-02*
