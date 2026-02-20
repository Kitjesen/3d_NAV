# 故障排除指南

> 涵盖编译错误和运行时常见故障的排查步骤。

---

## 目录

- [编译错误](#编译错误)
- [运行时故障](#运行时故障)
- [定位问题](#定位问题)
- [规划问题](#规划问题)
- [通信问题](#通信问题)
- [OTA 问题](#ota-问题)

---

## 编译错误

### tf2_ros/buffer_interface.hpp 找不到

**错误**:
```
fatal error: tf2_ros/buffer_interface.hpp: No such file or directory
```

**原因**: `tf2_ros` 包未正确安装。

**解决**:
```bash
sudo apt update
sudo apt install ros-humble-tf2-ros ros-humble-tf2 ros-humble-tf2-geometry-msgs
```

### tf2_geometry_msgs/tf2_geometry_msgs.hpp 找不到

**错误**:
```
fatal error: tf2_geometry_msgs/tf2_geometry_msgs.hpp: No such file or directory
```

**原因**: `visualization_tools` 的 `CMakeLists.txt` 缺少依赖。

**解决**:
在 `visualization_tools/CMakeLists.txt` 中添加:
```cmake
find_package(tf2_geometry_msgs REQUIRED)
```
并在 `ament_target_dependencies` 中加入 `tf2_geometry_msgs`。

### GTSAM not found

**错误**: CMake 找不到 GTSAM 库。

**解决**:
```bash
# 确认 GTSAM 已编译
ls src/global_planning/PCT_planner/planner/lib/3rdparty/gtsam-4.1.1/install/lib/libgtsam.so.4

# 未编译则重新编译
cd src/global_planning/PCT_planner/planner/lib/3rdparty/gtsam-4.1.1
mkdir -p build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=../install -DGTSAM_BUILD_TESTS=OFF -DGTSAM_WITH_TBB=OFF
make -j$(nproc) && make install
```

### libgtsam.so.4: cannot open (运行时)

**原因**: `LD_LIBRARY_PATH` 未配置。

**解决**:
```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$(pwd)/src/global_planning/PCT_planner/planner/lib/3rdparty/gtsam-4.1.1/install/lib
# 加入 ~/.bashrc 永久生效
```

### CMake CMP0074 策略警告

**解决**: 构建时忽略开发者警告:
```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -Wno-dev
```

### 通用依赖安装

```bash
sudo apt install ros-humble-desktop-full libusb-dev python3-colcon-common-extensions python-is-python3 python3-pip
pip install transforms3d pyyaml numpy scipy scikit-learn
rosdep install --from-paths src --ignore-src -r -y
```

---

## 运行时故障

### TF 变换 `map → odom` 未发布

**症状**: RViz 中数据不显示，模块报 TF lookup 失败。

**排查**:
```bash
# 检查 TF 树是否完整
ros2 run tf2_tools view_frames

# 检查各变换是否存在
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom body
```

**可能原因**:
| 原因 | 解决 |
|------|------|
| PGO / Localizer 未启动 | 启动对应节点 |
| PGO 未检测到回环 | 正常现象，首次回环前 `map→odom` 为单位变换 |
| Localizer 未调用 `/relocalize` | 调用重定位服务并确认返回 success |
| Fast-LIO2 未运行 | 检查 LiDAR 驱动和 fastlio2 节点 |

### 坐标系 frame_id 不正确

**验证各关键话题的 frame_id**:
```bash
ros2 topic echo /cloud_map --field header.frame_id --once       # 期望: odom
ros2 topic echo /terrain_map --field header.frame_id --once     # 期望: odom
ros2 topic echo /terrain_map_ext --field header.frame_id --once # 期望: odom
ros2 topic echo /path --field header.frame_id --once            # 期望: body
```

如果不正确，参考 [CHANGELOG.md](CHANGELOG.md) 中的坐标系修复记录。

### 话题频率异常

```bash
# 检查关键话题频率
ros2 topic hz /Odometry           # 期望: ~100Hz
ros2 topic hz /cloud_map          # 期望: ~10Hz
ros2 topic hz /terrain_map        # 期望: ~5Hz
ros2 topic hz /cmd_vel            # 期望: ~20Hz (运动时)
```

**频率过低的常见原因**:
- CPU 过载: `htop` 检查 CPU 占用
- 点云数据量过大: 增大 `laserVoxelSize` 参数
- 网络带宽不足 (分布式部署时)

### HealthMonitor 报 FAULT

**排查**:
```bash
ros2 topic echo /robot_health
```

| 告警 | 含义 | 解决 |
|------|------|------|
| SLAM < 20Hz | 里程计频率不足 | 检查 LiDAR 数据 + Fast-LIO2 |
| TF 断裂 | map→odom→body 链不完整 | 见上方 TF 排查 |
| 地形分析 < 1Hz | terrain_analysis 节点异常 | 重启节点 |
| 定位质量 ≥ 0.3 | ICP 配准不佳 | 换更好的初始位姿重新 relocalize |

---

## 定位问题

### 机器人位置跳变

**可能原因**:
1. **Fast-LIO2 退化**: 特征点不足 (如长走廊/空旷场地)
2. **Localizer ICP 不收敛**: 初始位姿偏差过大
3. **地图文件不正确**: 加载了错误的 PCD 文件

**排查**:
```bash
# 检查定位质量
ros2 topic echo /localization_quality   # < 0.1 为优, ≥ 0.3 为差

# 检查里程计协方差
ros2 topic echo /Odometry --field pose.covariance

# 重新定位
ros2 service call /relocalize interface/srv/Relocalize \
  "{pcd_path: '/path/to/map.pcd', x: 0.0, y: 0.0, z: 0.0, yaw: 0.0, pitch: 0.0, roll: 0.0}"
```

### 重定位失败

```bash
# 检查服务是否可用
ros2 service list | grep relocalize

# 检查 Localizer 节点状态
ros2 node info /localizer

# 检查 PCD 文件是否存在且可读
ls -la /path/to/map.pcd
```

---

## 规划问题

### local_planner 不发布路径

**排查清单**:
1. `/terrain_map` 是否有数据: `ros2 topic hz /terrain_map`
2. `/way_point` 是否已设置: `ros2 topic echo /way_point`
3. 手柄 `/joy` 是否发送速度指令 (手动模式): `ros2 topic echo /joy`
4. 是否所有路径都被障碍物阻塞: 在 RViz 中查看 `/free_paths`

### 机器人持续减速 (`/slow_down` 不断触发)

```bash
ros2 topic echo /slow_down
```

| slow_down 值 | 含义 | 调参建议 |
|-------------|------|---------|
| 1 | 大障碍物/陡坡 (penaltyScore > 0.15) | 增大 `obstacleHeightThre` |
| 2 | 中等地形代价 (penaltyScore > 0.10) | 增大 `laserVoxelSize` 降低密度 |
| 3 | 可选路径 < 5 | 增大 `adjacentRange` 扩大搜索范围 |

详细调参参见 [PARAMETER_TUNING.md](PARAMETER_TUNING.md)

### 全局规划器无路径

```bash
# 检查 Tomogram 是否加载
ros2 topic echo /tomogram   # 应有点云数据

# 检查目标点是否在地图范围内
# 检查 traversability 权重是否过高
```

---

## 通信问题

### Flutter App 无法连接 Nav Board

**排查**:
```bash
# 检查 gRPC Gateway 是否运行
ros2 node info /grpc_gateway

# 检查端口监听
ss -tlnp | grep 50051

# 检查 WiFi 连通性 (从手机 ping 导航板)
ping 192.168.4.1
```

### gRPC 断联后机器人不减速

**检查断联降级是否生效**:
```bash
# 心跳是否在发送
ros2 topic echo /slow_down

# 检查 GrpcGateway 日志
ros2 topic echo /rosout --filter "grpc_gateway"
```

断联降级仅在 AUTONOMOUS 模式下生效，其他模式不触发。

### Dog Board 连接失败

```bash
# 检查 Dog Board 网络可达
ping 192.168.4.100   # Dog Board IP

# 检查 CMS 端口
nc -zv 192.168.4.100 13145

# 检查 han_dog_bridge 状态
ros2 node info /han_dog_bridge
```

### 遥控时 SafetyGate 避障不生效

**检查 `/terrain_map` 是否有数据**:
```bash
ros2 topic hz /terrain_map   # 期望: ~5Hz
ros2 topic echo /terrain_map --once | head -5
```

如果无数据，检查 `terrain_analysis` 是否启动：
```bash
ros2 node list | grep terrain
```

**检查参数是否正确加载**:
```bash
ros2 param get /grpc_gateway obstacle_height_thre  # 期望: 0.2
ros2 param get /grpc_gateway stop_distance          # 期望: 0.8
ros2 param get /grpc_gateway slow_distance          # 期望: 2.0
```

**检查 SafetyGate 是否在 TELEOP 模式**:
- SafetyGate 仅在 TELEOP 模式下发布 `/cmd_vel`
- 非 TELEOP 模式下 `limit_reasons` 会包含 `not_teleop_mode`
- 通过 App 的 `TeleopFeedback.limit_reasons` 确认当前限幅原因

### 遥控/自主模式切换后机器人不受控

**`/cmd_vel` 仲裁问题**:
- SafetyGate (TELEOP) 和 pathFollower (AUTONOMOUS) 都发布到 `/cmd_vel`
- ModeManager 在模式切换时自动处理：
  - 进入 TELEOP → StopPathFollower (发 `/stop=1`, `/speed=0`)
  - 退出 TELEOP → SafetyGate 发零速度清除残余
  - 进入 AUTONOMOUS → StartPathFollower (发 `/stop=0`)
- 如果仍有问题，手动确认：
```bash
ros2 topic echo /stop          # 检查 stop 信号
ros2 topic echo /speed         # 检查 speed 信号
ros2 topic hz /cmd_vel         # 检查发布频率
```

---

## OTA 问题

### DownloadFromUrl 失败

**排查**:
```bash
# 检查机器人是否能访问外网
curl -I https://github.com

# 检查 DNS
nslookup github.com

# 检查磁盘空间
df -h /tmp/ota_staging/
```

### ApplyUpdate SHA256 校验失败

**原因**: 文件传输损坏或 manifest 中的 SHA256 不匹配。

**解决**:
```bash
# 手动验证
sha256sum /tmp/ota_staging/policy_walk_v2.3.onnx
# 对比 manifest.json 中的值
```

### 回滚失败

```bash
# 检查备份目录
ls -la /opt/robot/ota/backup/

# 检查已安装 manifest
cat /opt/robot/ota/installed_manifest.json | python3 -m json.tool
```

### 依赖检查失败 (v2)

**错误**: "Missing dependency: nav_firmware >= 1.2.0"

**原因**: 制品声明了 `dependencies`，但前置制品未安装或版本过旧。

**解决**:
```bash
# 查看已安装版本
cat /opt/robot/ota/installed_manifest.json | python3 -c "
import json,sys; d=json.load(sys.stdin)
for k,v in d.items(): print(f'  {k}: v{v.get(\"version\",\"?\")}')"

# 先安装依赖制品，再安装目标制品
# 或使用 force=true 跳过检查 (不推荐)
```

### 事务日志残留 (安装中断恢复)

**症状**: CheckUpdateReadiness 返回 `stale_transaction` 警告。

**原因**: 上次安装被中断（断电/崩溃），留下未完成的事务日志。

**排查**:
```bash
# 查看残留事务
ls /opt/robot/ota/backup/txn_*.json
cat /opt/robot/ota/backup/txn_policy_walk.json

# 手动清理（如果确认不需要恢复）
rm /opt/robot/ota/backup/txn_*.json
```

### COLD 更新安全检查

**错误**: "COLD 更新: 安装前需确保机器人坐下并禁用电机"

**解决**: 在 App 中先执行 `SetMode(IDLE)` → `SitDown` → `Disable`，确认机器人完全静止后再安装。

---

## 快速诊断命令集

```bash
# === 系统状态总览 ===
ros2 node list                              # 所有运行节点
ros2 topic list                             # 所有话题
ros2 service list                           # 所有服务
ros2 run tf2_tools view_frames              # TF 树可视化

# === 关键频率检查 ===
ros2 topic hz /Odometry &
ros2 topic hz /cloud_map &
ros2 topic hz /terrain_map &
ros2 topic hz /cmd_vel &
wait

# === 坐标系验证 ===
for topic in /cloud_map /terrain_map /terrain_map_ext /path; do
  echo -n "$topic: "
  ros2 topic echo $topic --field header.frame_id --once 2>/dev/null
done

# === 安全状态 ===
ros2 topic echo /robot_health --once
ros2 topic echo /slow_down --once
ros2 topic echo /stop --once
```

---

## 相关文档

- [BUILD_GUIDE.md](BUILD_GUIDE.md) — 编译步骤和依赖安装
- [PARAMETER_TUNING.md](PARAMETER_TUNING.md) — 参数调优
- [ARCHITECTURE.md](ARCHITECTURE.md) — 系统架构
- [CHANGELOG.md](CHANGELOG.md) — 变更记录

---

*最后更新: 2026-02-08*
