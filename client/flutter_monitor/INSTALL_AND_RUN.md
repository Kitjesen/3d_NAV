# 安装与运行指南

## 📋 检查清单

在开始前，确认以下条件：

- [ ] 机器人 gRPC Gateway 正在运行
- [ ] 机器人 IP: 192.168.66.190（或你的实际IP）
- [ ] 端口 50051 可访问
- [ ] Flutter SDK 已安装
- [ ] Dart 已安装

---

## 🚀 一步步执行

### 步骤 1: 验证机器人服务

在机器人上执行：

```bash
# 检查服务运行
ps aux | grep grpc_gateway

# 检查端口监听
ss -tlnp | grep 50051

# 如果未运行，启动服务
cd /home/sunrise/data/SLAM/navigation
source install/setup.bash
ros2 run remote_monitoring grpc_gateway
```

**预期输出**：

```
[INFO] [grpc_gateway]: gRPC Gateway listening on :50051
```

---

### 步骤 2: 安装 Flutter（选择一种方式）

#### 方案 A: Snap（最快，推荐）

```bash
sudo snap install flutter --classic
flutter doctor
```

#### 方案 B: 手动安装

```bash
cd ~
wget https://storage.googleapis.com/flutter_infra_release/releases/stable/linux/flutter_linux_3.16.0-stable.tar.xz
tar xf flutter_linux_3.16.0-stable.tar.xz
echo 'export PATH="$PATH:$HOME/flutter/bin"' >> ~/.bashrc
source ~/.bashrc
flutter doctor
```

#### 验证安装

```bash
flutter --version
dart --version
```

---

### 步骤 3: 安装 protoc-gen-dart

```bash
dart pub global activate protoc_plugin
export PATH="$PATH:$HOME/.pub-cache/bin"

# 验证
which protoc-gen-dart
protoc-gen-dart --version
```

如果 `which protoc-gen-dart` 找不到，添加到环境变量：

```bash
echo 'export PATH="$PATH:$HOME/.pub-cache/bin"' >> ~/.bashrc
source ~/.bashrc
```

---

### 步骤 4: 生成 Dart gRPC 代码

```bash
cd /home/sunrise/data/SLAM/navigation/client/flutter_monitor
./generate_proto.sh
```

**预期输出**：

```
✓ Code generation completed!
Generated files in lib/generated/
  common.pb.dart
  common.pbenum.dart
  common.pbjson.dart
  system.pb.dart
  system.pbgrpc.dart
  telemetry.pb.dart
  telemetry.pbgrpc.dart
  control.pb.dart
  control.pbgrpc.dart
  data.pb.dart
  data.pbgrpc.dart
```

---

### 步骤 5: 安装 Flutter 依赖

```bash
cd /home/sunrise/data/SLAM/navigation/client/flutter_monitor
flutter pub get
```

**预期输出**：

```
Resolving dependencies...
Got dependencies!
```

---

### 步骤 6: 运行 Flutter 应用

#### 选项 1: Linux 桌面端（推荐）

```bash
flutter run -d linux
```

#### 选项 2: Android 设备

```bash
# 连接手机，开启 USB 调试
flutter devices

# 运行
flutter run -d <device-id>
```

#### 选项 3: Web 浏览器

```bash
flutter run -d chrome
```

---

## 📱 使用 Flutter 应用

1. **连接界面**
   - IP: `192.168.66.190`（默认已填）
   - Port: `50051`（默认已填）
   - 点击 "Connect"

2. **状态监控界面**（自动跳转）
   - 实时位姿（X, Y, Z）
   - 速度（线速度、角速度）
   - 姿态（Roll, Pitch, Yaw）
   - 话题频率（Odom, Terrain, Path, LiDAR）
   - 系统资源（CPU, Memory, Temp）
   - TF 状态

3. **数据更新**
   - 快速状态: ~10Hz
   - 慢速状态: ~1Hz

---

## 🐛 故障排查

### 问题 1: `protoc-gen-dart: command not found`

```bash
# 重新安装
dart pub global activate protoc_plugin

# 添加到 PATH
export PATH="$PATH:$HOME/.pub-cache/bin"

# 验证
ls ~/.pub-cache/bin/protoc-gen-dart
```

### 问题 2: `flutter: command not found`

```bash
# 检查安装
which flutter

# 如果用snap安装
sudo snap install flutter --classic

# 重新加载环境变量
source ~/.bashrc
```

### 问题 3: gRPC 连接失败

```bash
# 测试网络连通性
ping 192.168.66.190

# 测试端口（需要 nc）
nc -zv 192.168.66.190 50051

# 查看机器人日志
ps aux | grep grpc_gateway
```

### 问题 4: `lib/generated/*.dart` 文件不存在

```bash
# 确保在正确目录
cd /home/sunrise/data/SLAM/navigation/client/flutter_monitor

# 检查 proto 文件
ls proto/*.proto

# 重新生成
./generate_proto.sh
```

---

## 📊 预期效果

成功运行后，你将看到：

```
┌─────────────────────────────────┐
│  Robot Status Monitor           │
│  ● Connected (42 updates)       │
├─────────────────────────────────┤
│  Position (odom frame)          │
│  X: 1.234 m  Y: -0.567 m        │
│  Z: 0.123 m                     │
├─────────────────────────────────┤
│  Velocity (body frame)          │
│  Linear: 0.500 m/s              │
│  Angular: 0.200 rad/s           │
├─────────────────────────────────┤
│  Orientation (RPY)              │
│  Roll: 2.5°  Pitch: -1.2°       │
│  Yaw: 45.6°   [TF OK ✓]        │
├─────────────────────────────────┤
│  Topic Rates                    │
│  Odometry:     50.2 Hz          │
│  Terrain Map:  10.5 Hz          │
│  Path:          5.1 Hz          │
│  LiDAR:        20.0 Hz          │
├─────────────────────────────────┤
│  System Resources               │
│  CPU:    [████░░░░░░] 45%      │
│  Memory: [██████░░░░] 62%      │
│  Temp:   48.5°C                 │
│  Battery: 87%                   │
└─────────────────────────────────┘
```

---

## ⏭️ 下一步

验证成功后：

1. **打包 Android APK**
   ```bash
   flutter build apk --release
   # APK 位置: build/app/outputs/flutter-apk/app-release.apk
   ```

2. **安装到手机**
   ```bash
   flutter install
   ```

3. **配置机器人 Wi-Fi AP**（Phase 0）
   - 让手机直接连接机器人热点
   - IP 固定为 192.168.4.1

---

## 当前命令总结（按顺序执行）

```bash
# 1. 安装 Flutter
sudo snap install flutter --classic

# 2. 安装 protoc-gen-dart
dart pub global activate protoc_plugin
export PATH="$PATH:$HOME/.pub-cache/bin"

# 3. 生成 Dart 代码
cd /home/sunrise/data/SLAM/navigation/client/flutter_monitor
./generate_proto.sh

# 4. 安装依赖
flutter pub get

# 5. 运行（桌面端）
flutter run -d linux
```

**就这5条命令，完成后就能看到监控界面！**
