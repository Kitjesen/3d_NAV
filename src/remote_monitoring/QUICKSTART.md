# 快速开始指南

## 1. 构建完成状态

✅ 所有功能已实现并构建成功：
- gRPC Gateway（4 个服务）
- Proto 契约（5 个文件）
- Safety Gate（deadman + 限幅）
- 状态聚合（fast/slow 双流）
- 事件系统（带回放）

## 2. 启动服务

```bash
cd /home/sunrise/data/SLAM/navigation
source install/setup.bash
ros2 run remote_monitoring grpc_gateway
```

或使用 launch 文件：

```bash
ros2 launch remote_monitoring grpc_gateway.launch.py
```

## 3. 验证服务运行

### 检查端口

```bash
ss -tlnp | grep 50051
```

预期输出：

```
LISTEN 0 4096  *:50051  *:*  users:(("grpc_gateway",pid=XXX,fd=16))
```

### 使用 grpcurl 测试（可选）

安装 grpcurl：

```bash
# 方式 1: 从包管理器
sudo apt-get install grpcurl

# 方式 2: 从 GitHub releases
wget https://github.com/fullstorydev/grpcurl/releases/download/v1.8.9/grpcurl_1.8.9_linux_arm64.tar.gz
tar -xzf grpcurl_1.8.9_linux_arm64.tar.gz
sudo mv grpcurl /usr/local/bin/
```

测试服务：

```bash
# 列出可用服务
grpcurl -plaintext localhost:50051 list

# 调用 GetRobotInfo
grpcurl -plaintext localhost:50051 robot.v1.SystemService/GetRobotInfo

# 流式获取状态（按 Ctrl+C 停止）
grpcurl -plaintext localhost:50051 robot.v1.TelemetryService/StreamFastState
```

## 4. Flutter 客户端接入

### 生成 Dart 代码

```bash
# 安装 protoc-gen-dart
dart pub global activate protoc_plugin

# 生成代码
protoc --dart_out=grpc:lib/generated \
  -I src/remote_monitoring/proto \
  common.proto system.proto control.proto telemetry.proto data.proto
```

### 连接示例

```dart
import 'package:grpc/grpc.dart';
import 'generated/telemetry.pbgrpc.dart';
import 'generated/system.pbgrpc.dart';

void main() async {
  // 连接到机器人（替换为实际 IP）
  final channel = ClientChannel(
    '192.168.4.1',  // 机器人 AP IP
    port: 50051,
    options: const ChannelOptions(
      credentials: ChannelCredentials.insecure(),
    ),
  );

  // 创建客户端
  final telemetry = TelemetryServiceClient(channel);
  final system = SystemServiceClient(channel);

  // 获取机器人信息
  final info = await system.getRobotInfo(Empty());
  print('Robot ID: ${info.robotId}');
  print('Version: ${info.firmwareVersion}');

  // 订阅快速状态流
  final fastStateStream = telemetry.streamFastState(
    FastStateRequest(desiredHz: 10.0)
  );

  await for (final state in fastStateStream) {
    print('Position: ${state.pose.position}');
    print('Velocity: ${state.velocity.linear}');
    print('RPY: ${state.rpyDeg}');
  }

  await channel.shutdown();
}
```

## 5. 当前状态与注意事项

### 已完成（P0）
- ✅ Proto 接口契约
- ✅ SystemService（Login/Heartbeat/Capabilities）
- ✅ ControlService（Lease/EmergencyStop/StreamTeleop）
- ✅ TelemetryService（FastState/SlowState/Events）
- ✅ Safety Gate（deadman + 限幅）
- ✅ 构建系统与配置

### 已实现但未完全集成
- 事件系统（EventBuffer 已实现，但需要与 ROS2 `/diagnostics` 集成）
- 任务管理（接口已定义，但需要实际任务调度器）

### Phase 2-4 待完成
- DataService 点云订阅（带压缩）
- WebRTC 视频推流
- 点云 ROI 裁剪与压缩
- 文件下载分块

### TF 警告处理

当前服务会报 TF 警告，因为：
- 系统未运行完整 SLAM 堆栈
- `map` 坐标系不存在

**解决方法**：启动完整导航系统后，TF 自动可用。

或临时忽略 TF 检查：

```yaml
# grpc_gateway.yaml
tf_map_frame: "odom"  # 临时用 odom
```

## 6. 远程监控工作流

### 完整启动流程

```bash
# 终端1: 启动导航系统
ros2 launch <your_nav_stack>

# 终端2: 启动 gRPC Gateway
ros2 launch remote_monitoring grpc_gateway.launch.py

# 远程设备：连接 Wi-Fi AP，使用 Flutter App 订阅
```

### 网络配置（下一步）

**当前**：gRPC 服务已就绪，但需要配置 Wi-Fi AP。

**下一步**：
1. 配置机器人 AP（见 `PLAN.md` Phase 0）
2. 设置 SSID / 密码 / IP
3. 远程设备接入测试

## 7. 快速测试命令

```bash
# 检查服务是否运行
ps aux | grep grpc_gateway

# 检查端口
ss -tlnp | grep 50051

# 查看日志
ros2 run remote_monitoring grpc_gateway  # 前台运行查看输出

# 停止服务
pkill -f grpc_gateway
```

## 8. 故障排查

### gRPC 服务不启动
- 检查端口是否被占用：`ss -tlnp | grep 50051`
- 检查日志：前台运行查看输出

### TF 警告过多
- 启动完整 SLAM 系统
- 或临时禁用 TF 检查（修改配置）

### 编译失败
- 确认 gRPC 依赖已安装：`dpkg -l | grep libgrpc`
- 重新安装：`./scripts/install_grpc.sh`
