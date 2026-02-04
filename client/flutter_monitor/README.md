# Robot Monitor Flutter Client

Flutter 客户端，通过 gRPC 实时监控机器人导航系统状态。

## 功能

- 实时位姿显示（X, Y, Z, Roll, Pitch, Yaw）
- 速度监控（线速度、角速度）
- 话题频率统计（/Odometry, /terrain_map, /path, LiDAR）
- 系统资源监控（CPU, Memory, Temperature）
- TF 状态检查

## 前置要求

- Flutter SDK >= 3.0.0
- Dart SDK
- protoc（Protocol Buffers编译器）
- protoc-gen-dart

## 安装步骤

### 1. 安装 Flutter

```bash
# 下载 Flutter（Linux）
cd ~
wget https://storage.googleapis.com/flutter_infra_release/releases/stable/linux/flutter_linux_3.16.0-stable.tar.xz
tar xf flutter_linux_3.16.0-stable.tar.xz
export PATH="$PATH:`pwd`/flutter/bin"

# 验证安装
flutter doctor
```

### 2. 安装 protoc-gen-dart

```bash
dart pub global activate protoc_plugin
export PATH="$PATH:$HOME/.pub-cache/bin"
```

### 3. 生成 Dart gRPC 代码

在项目根目录执行：

```bash
cd /home/sunrise/data/SLAM/navigation/client/flutter_monitor

# 生成 Dart proto 文件
protoc --dart_out=grpc:lib/generated \
  -I proto \
  proto/common.proto \
  proto/system.proto \
  proto/control.proto \
  proto/telemetry.proto \
  proto/data.proto
```

### 4. 安装依赖

```bash
flutter pub get
```

## 运行

### 桌面端（Linux）

```bash
flutter run -d linux
```

### Android（连接设备或模拟器）

```bash
flutter run -d android
```

### Web（测试用）

```bash
flutter run -d chrome
```

## 配置

默认连接地址：`192.168.66.190:50051`

可以在连接界面修改 IP 和端口。

## 项目结构

```
flutter_monitor/
├── pubspec.yaml           # 依赖配置
├── proto/                 # Proto 源文件（从 remote_monitoring 复制）
│   ├── common.proto
│   ├── system.proto
│   ├── control.proto
│   ├── telemetry.proto
│   └── data.proto
├── lib/
│   ├── main.dart          # 应用入口，连接界面
│   ├── generated/         # protoc 生成的 Dart 代码
│   │   ├── common.pb.dart
│   │   ├── system.pbgrpc.dart
│   │   ├── telemetry.pbgrpc.dart
│   │   └── ...
│   ├── services/
│   │   └── robot_client.dart   # gRPC 客户端封装
│   └── screens/
│       └── status_screen.dart  # 状态展示页面
└── README.md
```

## 使用说明

1. 启动机器人端 gRPC 服务：
   ```bash
   ros2 run remote_monitoring grpc_gateway
   ```

2. 确认机器人 IP（例如：192.168.66.190）

3. 启动 Flutter 应用

4. 输入机器人 IP 和端口（50051）

5. 点击 "Connect" 连接

6. 查看实时数据流

## 故障排查

### 连接失败

- 检查机器人和客户端是否在同一网络
- 确认 gRPC Gateway 正在运行：`ss -tlnp | grep 50051`
- 尝试 ping 机器人 IP
- 检查防火墙设置

### 生成代码错误

```bash
# 确认 protoc 版本
protoc --version  # 应该 >= 3.12

# 确认 protoc-gen-dart 已安装
which protoc-gen-dart
```

### Flutter 依赖问题

```bash
flutter clean
flutter pub get
```

## API 说明

### RobotClient

```dart
// 创建客户端
final client = RobotClient(host: '192.168.66.190', port: 50051);

// 连接
await client.connect();

// 订阅快速状态（10Hz）
client.streamFastState(desiredHz: 10.0).listen((state) {
  print('Position: ${state.pose.position}');
  print('Velocity: ${state.velocity.linear}');
});

// 订阅慢速状态（1Hz）
client.streamSlowState().listen((state) {
  print('CPU: ${state.resources.cpuPercent}%');
});

// 断开连接
await client.disconnect();
```

## 后续扩展

- [ ] 添加路径可视化（2D 轨迹图）
- [ ] 事件流展示（告警、错误）
- [ ] 遥操作控制（需要租约）
- [ ] 任务管理界面
- [ ] 历史数据图表
- [ ] 多机器人切换

## 技术栈

- **Flutter**: UI 框架
- **gRPC**: 通信协议
- **Protobuf**: 数据序列化
- **flutter_bloc**: 状态管理（可选）

## License

MIT
