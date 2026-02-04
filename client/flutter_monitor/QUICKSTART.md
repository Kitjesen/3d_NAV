# Flutter Monitor - 快速启动

## 当前状态

✅ 项目结构已创建  
✅ Proto 文件已复制  
✅ Flutter 代码已编写  
⏳ 等待生成 Dart gRPC 代码  

**机器人 IP**: 192.168.66.190  
**gRPC 端口**: 50051

---

## 立即执行的命令

### 1. 安装 Flutter（如果还没有）

```bash
# 方案 A: 使用 snap（推荐，最快）
sudo snap install flutter --classic

# 方案 B: 手动下载
cd ~
wget https://storage.googleapis.com/flutter_infra_release/releases/stable/linux/flutter_linux_3.16.0-stable.tar.xz
tar xf flutter_linux_3.16.0-stable.tar.xz
echo 'export PATH="$PATH:$HOME/flutter/bin"' >> ~/.bashrc
source ~/.bashrc
```

验证安装：

```bash
flutter doctor
```

### 2. 安装 protoc-gen-dart

```bash
dart pub global activate protoc_plugin
echo 'export PATH="$PATH:$HOME/.pub-cache/bin"' >> ~/.bashrc
source ~/.bashrc

# 验证
which protoc-gen-dart
```

### 3. 生成 Dart gRPC 代码

```bash
cd /home/sunrise/data/SLAM/navigation/client/flutter_monitor
./generate_proto.sh
```

预期输出：

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
  ...
```

### 4. 安装 Flutter 依赖

```bash
cd /home/sunrise/data/SLAM/navigation/client/flutter_monitor
flutter pub get
```

### 5. 运行应用

#### 桌面端（Linux，推荐）

```bash
flutter run -d linux
```

#### Android 设备

```bash
# 连接手机，开启 USB 调试
flutter devices  # 查看设备
flutter run -d android
```

#### Web 浏览器（测试）

```bash
flutter run -d chrome --web-hostname=0.0.0.0 --web-port=8080
```

---

## 验证步骤

### 终端 1：启动机器人服务

```bash
cd /home/sunrise/data/SLAM/navigation
source install/setup.bash
ros2 run remote_monitoring grpc_gateway
```

看到：

```
[INFO] [grpc_gateway]: gRPC Gateway listening on :50051
```

### 终端 2：验证端口

```bash
ss -tlnp | grep 50051
# 应该看到：LISTEN ... *:50051
```

### 终端 3：测试连接（可选）

```bash
# 安装 grpcurl
sudo snap install grpcurl

# 列出服务
grpcurl -plaintext 192.168.66.190:50051 list

# 调用 GetRobotInfo
grpcurl -plaintext 192.168.66.190:50051 robot.v1.SystemService/GetRobotInfo
```

### 终端 4：运行 Flutter 客户端

```bash
cd /home/sunrise/data/SLAM/navigation/client/flutter_monitor
flutter run -d linux
```

---

## 预期效果

Flutter 应用启动后：

1. **连接界面**
   - 输入 IP: 192.168.66.190
   - 端口: 50051
   - 点击 "Connect"

2. **状态界面**（自动切换）
   - 实时位姿（X, Y, Z, Yaw）
   - 速度（线速度、角速度）
   - 姿态角（Roll, Pitch, Yaw）
   - 话题频率（Odom, Terrain, Path）
   - TF 状态指示
   - 系统资源（CPU, Memory, Temperature）

3. **数据更新**
   - FastState: 每 100ms 更新一次
   - SlowState: 每 1s 更新一次
   - 连接状态实时显示

---

## 故障排查

### 问题 1: protoc-gen-dart 找不到

```bash
dart pub global activate protoc_plugin
export PATH="$PATH:$HOME/.pub-cache/bin"
which protoc-gen-dart
```

### 问题 2: Flutter 依赖下载慢

```bash
# 使用国内镜像
export PUB_HOSTED_URL=https://pub.flutter-io.cn
export FLUTTER_STORAGE_BASE_URL=https://storage.flutter-io.cn
flutter pub get
```

### 问题 3: gRPC 连接失败

```bash
# 检查网络连通性
ping 192.168.66.190

# 检查端口
nc -zv 192.168.66.190 50051

# 查看机器人端日志
cat ~/.cursor/projects/home-sunrise-data-SLAM-navigation/terminals/869460.txt | tail -50
```

### 问题 4: TF 警告（正常）

如果看到 TF 警告，是因为完整导航堆栈未启动。要消除警告：

```bash
# 启动完整导航系统
ros2 launch <your_nav_launch_file>
```

---

## 下一步

完成验证后，可以：

1. 部署到 Android 手机（APK）
2. 添加控制功能（遥操作）
3. 添加路径可视化
4. 集成事件告警

**当前优先级**：验证 gRPC 通信正常，看到实时数据流。
