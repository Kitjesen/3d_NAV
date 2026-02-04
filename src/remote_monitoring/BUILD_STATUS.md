# 构建状态与依赖

## 当前状态

所有代码已实现，但**需要安装 gRPC 依赖才能构建**。

## 依赖要求

### 必需依赖
- protobuf-compiler (protoc)
- libprotobuf-dev
- libgrpc++-dev
- protobuf-compiler-grpc (grpc_cpp_plugin)

## 安装步骤

### 选项 1：从包管理器安装（推荐，快速）

```bash
sudo apt-get update
sudo apt-get install -y libgrpc++-dev protobuf-compiler-grpc libprotobuf-dev
```

如果包管理器中没有 gRPC（旧版 Ubuntu），使用选项 2。

### 选项 2：运行自动安装脚本

```bash
cd src/remote_monitoring/scripts
./install_grpc.sh
```

**注意**：从源码编译 gRPC 需要 30+ 分钟。

### 选项 3：手动从源码编译

参考 `scripts/install_grpc.sh` 中的步骤。

## 构建命令

安装依赖后：

```bash
cd /home/sunrise/data/SLAM/navigation
colcon build --packages-select remote_monitoring
source install/setup.bash
```

## 运行

```bash
ros2 launch remote_monitoring grpc_gateway.launch.py
```

或直接运行：

```bash
ros2 run remote_monitoring grpc_gateway
```

## 验证

检查 gRPC 服务是否监听：

```bash
# 检查端口
ss -tlnp | grep 50051

# 使用 grpcurl 测试（需要安装 grpcurl）
grpcurl -plaintext localhost:50051 list
```

## 客户端接入（Flutter）

1. 复制 proto 文件到 Flutter 项目
2. 生成 Dart 代码：

```bash
protoc --dart_out=grpc:lib/generated \
  -I src/remote_monitoring/proto \
  common.proto system.proto control.proto telemetry.proto data.proto
```

3. 连接示例：

```dart
final channel = ClientChannel(
  '192.168.4.1',  // 机器人 IP
  port: 50051,
  options: ChannelOptions(credentials: ChannelCredentials.insecure()),
);

final telemetry = TelemetryServiceClient(channel);
await for (var state in telemetry.streamFastState(FastStateRequest())) {
  print('Position: ${state.pose.position}');
  print('Velocity: ${state.velocity.linear}');
}
```

## 已完成的功能

- ✅ Proto 契约定义（5 个文件）
- ✅ TelemetryService（fast/slow/event 三流）
- ✅ SystemService（Login/Heartbeat/Capabilities）
- ✅ ControlService（Lease/Mode/EmergencyStop/StreamTeleop）
- ✅ Safety Gate（deadman + 限幅）
- ✅ 统一 gRPC Gateway
- ✅ Launch 和配置文件

## 待完成（Phase 2-4）

- DataService 点云订阅（带压缩）
- WebRTC 视频推流
- 事件持久化与回放优化
- 任务管理器实现
