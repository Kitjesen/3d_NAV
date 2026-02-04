# 坐标系统一修复总结报告

**日期**: 2026-02-03  
**修复人**: AI Assistant  
**问题**: 导航系统坐标系混乱，导致数据不一致

---

## 🚨 发现的问题

### 问题 1: terrain_analysis 坐标系混用

**原始代码**:
```cpp
// 订阅 body 坐标系点云
subLaserCloud_ = subscribe("/cloud_registered", ...);  // body 系

// 但使用 odom 坐标系位置
vehicleX_ = odom->pose.pose.position.x;  // odom 系

// 直接相减（错误！不同坐标系）
float dis = sqrt((point.x - vehicleX_) * ...);  // ❌

// 输出声称是 map 系（实际是混乱的）
terrainCloud2.header.frame_id = "map";  // ❌
```

**影响**: 地形分析结果完全错误，障碍物位置偏移

---

### 问题 2: terrain_analysis_ext 同样问题

**原始代码**: 与 terrain_analysis 相同的坐标系混用问题

---

### 问题 3: local_planner 坐标系不一致

**原始代码**:
```cpp
// 当 useTerrainAnalysis=false 时
subLaserCloud_ = subscribe("/cloud_registered", ...);  // body 系 ❌
vehicleX_ = odom->pose.pose.position.x - offset;       // odom 系 ❌
```

---

### 问题 4: vehicle vs body 坐标系混乱

**原始设计**:
- Fast-LIO2 发布 `odom → body` TF
- local_planner 输出 `frame_id = "vehicle"`
- Launch 文件发布 `sensor → vehicle` TF
- **TF 树不完整**: `body` 和 `sensor` 没有连接 ❌

---

## ✅ 修复方案

### 核心设计原则

```
map 系 ──▶ 全局规划（pct_planner）
  │
  ├─ TF ─▶ odom 系 ──▶ 感知层（terrain_analysis）
  │                  ─▶ 路径适配（pct_adapters）
  │
  └─ TF ─▶ body 系 ──▶ 局部规划（local_planner）
                      ─▶ 底盘控制（pathFollower）
```

### 修复详情

#### 1. terrain_analysis.cpp

**修改位置**: 第166行, 第669行

```cpp
// 修改前
subLaserCloud_ = subscribe("/cloud_registered", ...);  // ❌ body 系
terrainCloud2.header.frame_id = "map";                 // ❌ 错误声明

// 修改后
subLaserCloud_ = subscribe("/cloud_map", ...);         // ✅ odom 系
terrainCloud2.header.frame_id = "odom";                // ✅ 正确声明
```

**逻辑**:
- 输入: `/cloud_map` (odom 系) ✅
- 位姿: `vehicleX_` (odom 系) ✅
- 计算: 同一坐标系相减 ✅
- 输出: `/terrain_map` (odom 系) ✅

---

#### 2. terrain_analysis_ext.cpp

**修改位置**: 第96行, 第583行

```cpp
// 修改前
subLaserCloud_ = subscribe("/cloud_registered", ...);  // ❌
terrainCloud2.header.frame_id = "map";                 // ❌

// 修改后
subLaserCloud_ = subscribe("/cloud_map", ...);         // ✅
terrainCloud2.header.frame_id = "odom";                // ✅
```

---

#### 3. localPlanner.cpp

**修改位置**: 第150行, 第1022/1068/1095/1103行

```cpp
// 修改前
subLaserCloud_ = subscribe("/cloud_registered", ...);  // ❌ body 系
path.header.frame_id = "vehicle";                      // ❌ TF树中无此坐标系

// 修改后
subLaserCloud_ = subscribe("/cloud_map", ...);         // ✅ odom 系
path.header.frame_id = "body";                         // ✅ 与 Fast-LIO2 TF 一致
```

**逻辑**:
- 输入: `/cloud_map` (odom), `/terrain_map` (odom) ✅
- 转换: odom → body (第758-772行) ✅
- 输出: `/path` (body 系) ✅

---

#### 4. pathFollower.cpp

**修改位置**: 第461行

```cpp
// 修改前
cmd_vel.header.frame_id = "vehicle";  // ❌ TF树中无此坐标系

// 修改后
cmd_vel.header.frame_id = "body";     // ✅ 与 Fast-LIO2 TF 一致
```

---

#### 5. local_planner.launch

**修改位置**: 第96-102行

```xml
<!-- 修改前 -->
<node pkg="tf2_ros" exec="static_transform_publisher" 
      args="-$(var sensorOffsetX) -$(var sensorOffsetY) 0 0 0 0 /sensor /vehicle"/>
<node pkg="tf2_ros" exec="static_transform_publisher" 
      args="0 0 $(var cameraOffsetZ) -1.5707963 0 -1.5707963 /sensor /camera"/>

<!-- 修改后 -->
<node pkg="tf2_ros" exec="static_transform_publisher" 
      args="$(var lidarOffsetX) $(var lidarOffsetY) $(var lidarOffsetZ) $(var lidarRoll) $(var lidarPitch) $(var lidarYaw) body lidar"/>
<node pkg="tf2_ros" exec="static_transform_publisher" 
      args="0 0 $(var cameraOffsetZ) -1.5707963 0 -1.5707963 lidar camera"/>
<!-- 注释: sensorOffset 在代码中处理（vehicleX_ 计算），不在 TF 树中 -->
```

---

## 📊 修复后的完整架构

### TF 树结构

```
map
 └─ odom (PGO/Localizer 发布)
     └─ body (Fast-LIO2 发布)
         ├─ lidar (静态外参)
         └─ camera (launch 文件发布)
```

### 数据流

```
┌────────────────────────────────────────────────────────┐
│ Fast-LIO2                                               │
├────────────────────────────────────────────────────────┤
│ /cloud_map (odom) ──┐                                  │
│ /Odometry (odom) ───┼─────────┐                        │
│ TF: odom→body ──────┘         │                        │
└───────────────────────────────┼────────────────────────┘
                                ▼
                    ┌────────────────────────┐
                    │ terrain_analysis       │
                    │ (odom 系工作) ✅       │
                    └────────┬───────────────┘
                             │
                    /terrain_map (odom) ✅
                             │
                             ▼
                    ┌────────────────────────┐
                    │ local_planner          │
                    │ odom → body 转换 ✅    │
                    └────────┬───────────────┘
                             │
                      /path (body) ✅
                             │
                             ▼
                    ┌────────────────────────┐
                    │ pathFollower           │
                    │ (body 系控制) ✅       │
                    └────────┬───────────────┘
                             │
                     /cmd_vel (body) ✅
```

---

## 📋 修改文件清单

| 文件 | 修改行数 | 修改内容 |
|------|---------|---------|
| `terrain_analysis/src/terrainAnalysis.cpp` | 2处 | 订阅话题 + frame_id |
| `terrain_analysis_ext/src/terrainAnalysisExt.cpp` | 2处 | 订阅话题 + frame_id |
| `local_planner/src/localPlanner.cpp` | 5处 | 订阅话题 + 4个 frame_id |
| `local_planner/src/pathFollower.cpp` | 1处 | frame_id |
| `local_planner/launch/local_planner.launch` | 1处 | TF 发布器 + lidar 外参参数 |
| `AGENTS.md` | 3处 | 文档更新 |
| **新增** `COORDINATE_FRAMES.md` | - | 完整坐标系设计文档 |

---

## 🔍 验证方法

### 1. 编译检查
```bash
cd /home/sunrise/data/SLAM/navigation
colcon build --packages-select local_planner terrain_analysis terrain_analysis_ext
```

### 2. 运行时检查

```bash
# 启动系统后
ros2 topic echo /terrain_map --field header.frame_id --once
# 应输出: odom ✅

ros2 topic echo /path --field header.frame_id --once
# 应输出: body ✅

ros2 run tf2_ros tf2_echo odom body
# 应显示正常的 TF 变换 ✅
```

### 3. RViz 可视化验证

```yaml
Fixed Frame: odom

Displays:
  - /cloud_map (odom 系，应与机器人对齐) ✅
  - /terrain_map (odom 系，应与 /cloud_map 对齐) ✅
  - /path (body 系，通过 TF: odom→body 正确显示) ✅
  - TF 树（应完整连续）✅
```

---

## 🎯 关键修复点总结

### 修复前

| 模块 | 输入点云坐标系 | vehicleX_坐标系 | 计算正确性 | 输出frame_id | TF树完整性 |
|------|---------------|----------------|-----------|-------------|-----------|
| terrain_analysis | body | odom | ❌ 不同系相减 | "map" ❌ | - |
| terrain_analysis_ext | body | odom | ❌ 不同系相减 | "map" ❌ | - |
| local_planner | body | odom | ❌ 不同系相减 | "vehicle" ❌ | ❌ 缺 TF |
| pathFollower | - | odom | - | "vehicle" ❌ | ❌ 缺 TF |

### 修复后

| 模块 | 输入点云坐标系 | vehicleX_坐标系 | 计算正确性 | 输出frame_id | TF树完整性 |
|------|---------------|----------------|-----------|-------------|-----------|
| terrain_analysis | **odom** ✅ | **odom** ✅ | ✅ 同系相减 | **"odom"** ✅ | ✅ |
| terrain_analysis_ext | **odom** ✅ | **odom** ✅ | ✅ 同系相减 | **"odom"** ✅ | ✅ |
| local_planner | **odom** ✅ | **odom** ✅ | ✅ 转换正确 | **"body"** ✅ | ✅ 完整 |
| pathFollower | - | **odom** ✅ | ✅ | **"body"** ✅ | ✅ 完整 |

---

## 💡 设计亮点

### 1. 分层清晰

- **map 系**: 全局规划和地图管理（长期一致性）
- **odom 系**: 感知和局部地图（中期一致性）
- **body 系**: 局部规划和控制（瞬时操作）

### 2. 性能优化

- 高频数据（100Hz）使用局部坐标系（odom/body）
- 低频数据（1-10Hz）使用全局坐标系（map）
- 坐标转换集中在模块边界（pct_adapters, local_planner）

### 3. 易于调试

- frame_id 与实际数据坐标系一致
- TF 树完整且连续
- RViz 可以正确显示所有数据

---

## ⚙️ 传感器偏移处理

### vehicleX_ 的含义

```cpp
// odometryHandler (local_planner 第365-368行)
vehicleX_ = odom->pose.pose.position.x  // 传感器(body)在 odom 系的位置
            - cos(yaw) * sensorOffsetX_  // 减去传感器到底盘中心的偏移
            + sin(yaw) * sensorOffsetY_;
// 结果: 底盘中心在 odom 系的位置 ✅
```

### 为什么不需要单独的 vehicle 坐标系？

1. **代码中已处理**: `vehicleX_` 已经是底盘中心位置（考虑了传感器偏移）
2. **局部规划逻辑**: 转换到 body 系后，机器人在原点，偏移已在 odom 系处理
3. **简化 TF 树**: 避免冗余的坐标系和 TF 发布

---

## 📐 完整 TF 树

### 修复后的 TF 树

```
map (全局地图)
 │
 └─ odom (里程计) [PGO/Localizer 发布]
     │
     └─ body (机器人本体/传感器中心) [Fast-LIO2 发布]
         │
         ├─ lidar [静态外参: r_il, t_il]
         │
         └─ camera [launch 文件发布]
```

### TF 发布者

| 变换 | 发布者 | 类型 | 频率 |
|------|--------|------|------|
| `map → odom` | PGO/Localizer | 动态 | 10-20Hz |
| `odom → body` | Fast-LIO2 | 动态 | 100Hz |
| `body → lidar` | Fast-LIO2 | 静态 | - |
| `body → camera` | launch 文件 | 静态 | - |

---

## 🧪 测试检查项

### 启动前检查

- [ ] 确认所有修改的文件已编译
- [ ] 检查 launch 文件语法正确
- [ ] 确认配置参数未改变

### 启动后检查

```bash
# 1. 检查话题坐标系
ros2 topic echo /cloud_map --field header.frame_id --once
# 期望: odom ✅

ros2 topic echo /terrain_map --field header.frame_id --once
# 期望: odom ✅

ros2 topic echo /terrain_map_ext --field header.frame_id --once
# 期望: odom ✅

ros2 topic echo /path --field header.frame_id --once
# 期望: body ✅

# 2. 检查 TF 树
ros2 run tf2_tools view_frames
# 期望: map → odom → body → [lidar, camera] ✅

# 3. 检查数据对齐
# 在 RViz 中同时显示:
# - /cloud_map (odom 系)
# - /terrain_map (odom 系)
# - /path (body 系，通过 TF 自动转换)
# 期望: 所有数据在空间上对齐 ✅
```

### 功能测试

- [ ] 建图模式：地形分析正确识别障碍物
- [ ] 定位模式：重定位后坐标系正确
- [ ] 全局规划：pct_path 正确转换到 way_point
- [ ] 局部规划：避障路径合理
- [ ] 路径跟踪：机器人正确跟踪路径

---

## 📝 未来改进建议

### 1. 添加坐标系验证节点

```cpp
class CoordinateValidator : public rclcpp::Node {
  // 订阅所有关键话题
  // 检查 frame_id 是否符合规范
  // 检查 TF 树是否完整
  // 发布 /system/coord_status
};
```

### 2. 统一参数管理

```yaml
# navigation_params/coordinate_frames.yaml
coordinate_frames:
  map_frame: "map"
  odom_frame: "odom"
  body_frame: "body"
  lidar_frame: "lidar"
```

### 3. 添加运行时检查

```cpp
// 在每个模块初始化时
void checkCoordinateConsistency() {
  if (input_cloud.frame_id != expected_frame) {
    RCLCPP_ERROR("Frame mismatch!");
  }
}
```

---

## ⚠️ 注意事项

### 传感器偏移

- `sensorOffsetX_`, `sensorOffsetY_` 参数仍然保留
- 在代码中通过 `vehicleX_ = odom_x - offset` 处理
- **不在 TF 树中表示**（避免混乱）

### 兼容性

- 修改后可能需要更新 RViz 配置文件（Fixed Frame）
- 旧的录制数据（rosbag）如果使用了错误的 frame_id，可能需要重新录制

### 性能影响

- ✅ 减少了不必要的 TF 查询
- ✅ 消除了坐标系转换错误
- ✅ 可能略微增加点云数据量（`/cloud_map` 比 `/cloud_registered` 稍大）

---

## 📚 相关文档

- `src/base_autonomy/COORDINATE_FRAMES.md` - 完整坐标系设计文档
- `AGENTS.md` - 系统架构文档（已更新）
- `src/slam/fastlio2/config/lio.yaml` - Fast-LIO2 坐标系配置

---

*修复完成时间: 2026-02-03*  
*验证状态: 编译中...*
