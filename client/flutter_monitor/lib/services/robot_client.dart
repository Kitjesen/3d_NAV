import 'dart:async';
import 'package:grpc/grpc.dart';
import '../generated/telemetry.pbgrpc.dart';
import '../generated/system.pbgrpc.dart';
import '../generated/common.pb.dart';
import '../generated/google/protobuf/empty.pb.dart';

class RobotClient {
  final String host;
  final int port;
  
  late ClientChannel _channel;
  late TelemetryServiceClient _telemetryClient;
  late SystemServiceClient _systemClient;
  
  bool _isConnected = false;
  StreamSubscription? _fastStateSubscription;
  StreamSubscription? _slowStateSubscription;

  RobotClient({
    required this.host,
    this.port = 50051,
  });

  /// 连接到机器人
  Future<bool> connect() async {
    try {
      _channel = ClientChannel(
        host,
        port: port,
        options: const ChannelOptions(
          credentials: ChannelCredentials.insecure(),
        ),
      );

      _telemetryClient = TelemetryServiceClient(_channel);
      _systemClient = SystemServiceClient(_channel);

      // 测试连接
      final info = await _systemClient.getRobotInfo(
        Empty(),
        options: CallOptions(timeout: const Duration(seconds: 3)),
      );

      print('Connected to robot: ${info.robotId}');
      _isConnected = true;
      return true;
    } catch (e) {
      print('Connection failed: $e');
      _isConnected = false;
      return false;
    }
  }

  /// 订阅快速状态流（位姿、速度、IMU）
  Stream<FastState> streamFastState({double desiredHz = 10.0}) {
    if (!_isConnected) {
      throw Exception('Not connected to robot');
    }

    final request = FastStateRequest()..desiredHz = desiredHz;
    
    return _telemetryClient.streamFastState(request).map((state) {
      return state;
    }).handleError((error) {
      print('FastState stream error: $error');
      _isConnected = false;
    });
  }

  /// 订阅慢速状态流（系统资源、模式）
  Stream<SlowState> streamSlowState() {
    if (!_isConnected) {
      throw Exception('Not connected to robot');
    }

    final request = SlowStateRequest();
    
    return _telemetryClient.streamSlowState(request).map((state) {
      return state;
    }).handleError((error) {
      print('SlowState stream error: $error');
      _isConnected = false;
    });
  }

  /// 获取机器人信息
  Future<RobotInfoResponse> getRobotInfo() async {
    return await _systemClient.getRobotInfo(Empty());
  }

  /// 获取机器人能力
  Future<CapabilitiesResponse> getCapabilities() async {
    return await _systemClient.getCapabilities(Empty());
  }

  /// 断开连接
  Future<void> disconnect() async {
    await _fastStateSubscription?.cancel();
    await _slowStateSubscription?.cancel();
    await _channel.shutdown();
    _isConnected = false;
  }

  bool get isConnected => _isConnected;
}
