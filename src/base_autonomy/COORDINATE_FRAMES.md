# 导航系统坐标系设计文档

## 📐 坐标系层次结构

```
map (全局地图坐标系) - 固定世界参考系
 │
 ├─ TF: PGO/Localizer 发布 ───────────────────┐
 │                                             │
 ▼                                             ▼
odom (里程计坐标系) ◄─ 感知和全局路径适配    map系规划数据
 │                      在这里工作！           │
 ├─ TF: Fast-LIO2 发布                         │
 │                                             │
 ▼                                             │
body (机器人本体坐标系) ◄─ 局部规划和控制在这里！ │
 │                                             │
 ├─ 静态外参                                   │
 │                                             │
 ▼                                             │
lidar (激光雷达坐标系)                         │
                                               │
                    全局规划 ──────────────────┘
```

---

## 🎯 各层坐标系的作用

### map 坐标系
- **定义**: 固定的全局世界坐标系，经过回环优化或重定位的统一参考系
- **使用场景**: 
  - 全局路径规划（PCT_planner）
  - 地图保存和加载
  - 回环检测和优化（PGO）
- **发布者**: 无（静态参考系）
- **变换发布**: PGO 或 Localizer 发布 `map → odom` 变换

### odom 坐标系
- **定义**: 里程计坐标系，局部连续且平滑，与 map 通过动态变换关联
- **使用场景**: 
  - **地形分析**（terrain_analysis, terrain_analysis_ext）
  - **全局路径适配**（pct_adapters）
  - **机器人位姿**（/Odometry 的位置部分）
- **特点**: 
  - 适合维护滚动地图（需要世界坐标参考）
  - 与 map 系通过小的偏移量关联（PGO/Localizer 修正）
  - 短期内保持一致性，长期可能漂移

### body 坐标系
- **定义**: 机器人本体坐标系，原点在传感器（IMU）中心
- **使用场景**: 
  - **局部路径规划**（local_planner 的内部计算）
  - **路径跟踪控制**（pathFollower）
  - **底盘控制指令**（/cmd_vel）
- **特点**: 
  - 机器人永远在原点 (0,0,0)
  - 适合以机器人为中心的局部操作
  - 简化碰撞检测和路径评估
- **注意**: 
  - 代码中的 `vehicleX_` 已考虑传感器偏移（sensorOffsetX/Y）
  - 表示机器人底盘中心在 odom 系下的位置
  - 不需要单独的 "vehicle" 坐标系

### lidar 坐标系
- **定义**: 激光雷达传感器坐标系
- **使用场景**: 原始传感器数据
- **变换**: 通过静态外参 `r_il`, `t_il` 与 body 系关联

---

## 📊 完整数据流表

### Fast-LIO2 输出

| 话题 | 类型 | 坐标系 | 订阅者 | 说明 |
|------|------|--------|--------|------|
| `/cloud_registered` | PointCloud2 | **body** | sensor_scan_generation, PGO, Localizer | 本体坐标系点云 |
| `/cloud_map` | PointCloud2 | **odom** | terrain_analysis, terrain_analysis_ext, local_planner | 世界坐标系点云 ✅ |
| `/Odometry` | Odometry | **odom** | 所有模块 | 机器人在 odom 系的位姿 |
| TF: `odom→body` | - | - | - | 实时位姿变换（100Hz） |

### 地形分析输出

| 话题 | 类型 | 坐标系 | 订阅者 | 说明 |
|------|------|--------|--------|------|
| `/terrain_map` | PointCloud2 | **odom** | terrain_analysis_ext, local_planner | 地形障碍物地图 ✅ |
| `/terrain_map_ext` | PointCloud2 | **odom** | local_planner | 扩展地形地图 ✅ |

### 规划和控制输出

| 话题 | 类型 | 坐标系 | 订阅者 | 说明 |
|------|------|--------|--------|------|
| `/pct_path` | Path | **map** | pct_adapters | 全局规划路径 |
| `/way_point` | PointStamped | **odom** | local_planner | 当前目标航点 ✅ |
| `/path` | Path | **body** | pathFollower | 局部规划路径 ✅ |
| `/cmd_vel` | TwistStamped | **body** | robot_driver | 底盘控制指令 ✅ |
| `/slow_down` | Int8 | - | pathFollower | 减速指令（无坐标系） |

---

## 🔄 坐标系转换流程

### 1. 全局规划到局部执行

```
PCT_planner
  │
  ├─ 输出: /pct_path (map 坐标系)
  ▼
pct_adapters
  │
  ├─ TF变换: map → odom
  ├─ 路径下采样 → 航点序列
  ├─ 闭环控制: 到达当前航点才发布下一个
  │
  ├─ 输出: /way_point (odom 坐标系) ✅
  ▼
local_planner
  │
  ├─ 输入: /way_point (odom), /terrain_map (odom), /Odometry (odom)
  ├─ 转换到 body 系: (第758-815行)
  │    pointX_body = (pointX_odom - vehicleX_odom) * cos(yaw) + ...
  │    注: vehicleX_odom 已考虑传感器偏移，指向底盘中心
  │
  ├─ 在 body 系进行路径选择和碰撞检测
  │
  ├─ 输出: /path (body 坐标系) ✅
  ▼
pathFollower
  │
  ├─ 输入: /path (body), /Odometry
  ├─ Pure Pursuit 跟踪
  │
  ├─ 输出: /cmd_vel (body 坐标系) ✅
  ▼
robot_driver (底盘执行)
```

### 2. 感知到规划

```
Fast-LIO2
  │
  ├─ /cloud_map (odom 坐标系)
  ├─ /Odometry (odom 坐标系)
  ▼
terrain_analysis
  │
  ├─ 输入: /cloud_map (odom), /Odometry (odom) ✅
  ├─ 滚动地图维护 (基于 vehicleX_odom 触发)
  ├─ 地面估计、障碍物检测
  │
  ├─ 输出: /terrain_map (odom 坐标系) ✅
  ▼
terrain_analysis_ext
  │
  ├─ 输入: /cloud_map (odom), /terrain_map (odom) ✅
  ├─ 连通性检查、扩展地图
  │
  ├─ 输出: /terrain_map_ext (odom 坐标系) ✅
  ▼
local_planner
  │
  ├─ 输入: /terrain_map (odom) ✅
  ├─ 转换到 vehicle 系规划
  │
  ├─ 输出: /path (vehicle) ✅
```

---

## ⚠️ 常见错误示例

### ❌ 错误 1: 不同坐标系直接相减

```cpp
// 错误示例（修改前的 terrain_analysis）
point.x;  // 来自 /cloud_registered (body 系)
vehicleX_; // 来自 /Odometry (odom 系)
float dis = sqrt((point.x - vehicleX_) * ...);  // ❌ 不同坐标系！
```

**正确做法**：
```cpp
// 方法1: 统一使用 odom 系
point.x;  // 来自 /cloud_map (odom 系) ✅
vehicleX_; // 来自 /Odometry (odom 系) ✅
float dis = sqrt((point.x - vehicleX_) * ...);  // ✅ 同一坐标系

// 方法2: 进行坐标变换
point_odom = transform(point_body, TF_body_to_odom);  // ✅
float dis = sqrt((point_odom.x - vehicleX_) * ...);
```

### ❌ 错误 2: frame_id 与实际数据不符

```cpp
// 错误示例
terrainCloud2.header.frame_id = "map";  // 声称 map 系
// 但实际数据是在 odom 系或混合坐标系 ❌
```

**正确做法**：
```cpp
terrainCloud2.header.frame_id = "odom";  // 与实际数据坐标系一致 ✅
```

---

## 🛠️ 已修复的问题

### 修改前的问题

1. ❌ `terrain_analysis` 订阅 `/cloud_registered` (body系)，但用 `vehicleX_` (odom系) 计算
2. ❌ `terrain_analysis_ext` 同样的问题
3. ❌ `local_planner` 在 `useTerrainAnalysis_=false` 时同样问题
4. ❌ 输出 frame_id 声称 `map`，但实际是错误的混合坐标系

### 修改后的架构 ✅

| 模块 | 输入点云 | 输入位姿 | 计算坐标系 | 输出坐标系 |
|------|---------|---------|-----------|-----------|
| terrain_analysis | `/cloud_map` (odom) | `/Odometry` (odom) | odom | `/terrain_map` (odom) ✅ |
| terrain_analysis_ext | `/cloud_map` (odom) | `/Odometry` (odom) | odom | `/terrain_map_ext` (odom) ✅ |
| local_planner | `/terrain_map` (odom) | `/Odometry` (odom) | odom→body | `/path` (body) ✅ |
| pathFollower | `/path` (body) | `/Odometry` | body | `/cmd_vel` (body) ✅ |

---

## 🔍 验证检查清单

### 启动后检查

```bash
# 1. 检查 TF 树
ros2 run tf2_tools view_frames
# 应该看到: map → odom → body → lidar

# 2. 检查话题坐标系
ros2 topic echo /cloud_map --field header.frame_id     # 应显示: odom
ros2 topic echo /terrain_map --field header.frame_id   # 应显示: odom ✅
ros2 topic echo /path --field header.frame_id          # 应显示: body

# 3. 检查 TF 变换
ros2 run tf2_ros tf2_echo map odom     # 查看 map → odom 偏移
ros2 run tf2_ros tf2_echo odom body    # 查看 odom → body 位姿
```

### RViz 可视化验证

```yaml
# RViz Fixed Frame: map 或 odom
Displays:
  - /cloud_map (odom 系，应与机器人对齐)
  - /terrain_map (odom 系，应与 /cloud_map 对齐) ✅
  - /path (body 系，应以机器人为中心)
  - TF 树（检查连续性）
```

---

## 📌 设计原则总结

### 原则 1: 分层设计
- **map 系**: 全局规划和地图管理
- **odom 系**: 感知、局部地图和路径适配
- **body 系**: 局部规划和底盘控制

### 原则 2: 数据一致性
- 同一模块内的计算应在同一坐标系
- 跨坐标系必须通过 TF 变换
- frame_id 必须与实际数据坐标系一致

### 原则 3: 性能优化
- 高频数据（点云、控制指令）使用局部坐标系（odom/body）
- 低频数据（全局路径）使用全局坐标系（map）
- 坐标转换尽量集中在边界处（pct_adapters, local_planner）

---

## 🔧 代码示例

### 正确的 odom 系处理

```cpp
// terrain_analysis.cpp (修改后)
subLaserCloud_ = subscribe("/cloud_map", ...);  // odom 系点云 ✅
vehicleX_ = odom->pose.pose.position.x;         // odom 系位置 ✅

// 计算（同一坐标系）
float dis = sqrt((point.x - vehicleX_) * (point.x - vehicleX_) + 
                 (point.y - vehicleY_) * (point.y - vehicleY_));  // ✅

// 输出
terrainCloud2.header.frame_id = "odom";  // ✅
```

### 正确的 odom → body 转换

```cpp
// local_planner.cpp (第758-772行)
// 输入在 odom 系
float pointX_odom = plannerCloud_->points[i].x;  // odom 系
float vehicleX_odom = vehicleX_;  // odom 系（已考虑传感器偏移，指向底盘中心）

// 转换到 body 系
float pointX1 = pointX_odom - vehicleX_odom;     // 平移到机器人中心
float pointY1 = pointY_odom - vehicleY_odom;

// 旋转到机器人朝向
point.x = pointX1 * cosVehicleYaw + pointY1 * sinVehicleYaw;  // ✅
point.y = -pointX1 * sinVehicleYaw + pointY1 * cosVehicleYaw;
```

**关于传感器偏移**:
```cpp
// odometryHandler (第365-368行)
// /Odometry 给出的是传感器(IMU)在 odom 系的位置
// vehicleX_ 减去传感器偏移，得到底盘中心在 odom 系的位置
vehicleX_ = odom->pose.pose.position.x - cos(yaw) * sensorOffsetX_ 
                                        + sin(yaw) * sensorOffsetY_;  // ✅
```

### 正确的 map → odom 转换

```cpp
// pct_adapters.cpp (第120-152行)
geometry_msgs::msg::PointStamped input;
input.header.frame_id = "map";          // 输入在 map 系
input.point = point_in_map;

// TF 变换
auto transform = tf_buffer_->lookupTransform("odom", "map", ...);
tf2::doTransform(input, output, transform);  // ✅

waypoint_msg.header.frame_id = odom_frame_;  // 输出在 odom 系 ✅
```

---

## 📝 修改记录

### 2026-02-03: 坐标系统一修复

**修改文件**:
1. `src/base_autonomy/terrain_analysis/src/terrainAnalysis.cpp`
   - 第166行: `/cloud_registered` → `/cloud_map`
   - 第669行: `frame_id = "map"` → `frame_id = "odom"`

2. `src/base_autonomy/terrain_analysis_ext/src/terrainAnalysisExt.cpp`
   - 第96行: `/cloud_registered` → `/cloud_map`
   - 第583行: `frame_id = "map"` → `frame_id = "odom"`

3. `src/base_autonomy/local_planner/src/localPlanner.cpp`
   - 第150行: `/cloud_registered` → `/cloud_map` (当 useTerrainAnalysis=false 时使用)

**影响**:
- ✅ 所有感知模块现在在统一的 odom 坐标系下工作
- ✅ 消除了 body-odom 坐标系混用的 bug
- ✅ frame_id 与实际数据坐标系一致

---

*最后更新: 2026-02-03*
