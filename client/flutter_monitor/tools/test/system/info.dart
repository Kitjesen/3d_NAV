/// 测试 SystemService: GetRobotInfo + Heartbeat + GetCapabilities
/// dart run tools/test/system/info.dart [host] [port]
import 'dart:io';
import 'package:grpc/grpc.dart';
import 'package:robot_proto/src/system.pbgrpc.dart';
import 'package:robot_proto/src/system.pb.dart';
import 'package:protobuf/well_known_types/google/protobuf/empty.pb.dart';

void main(List<String> args) async {
  final host = args.isNotEmpty ? args[0] : '192.168.66.190';
  final port = args.length > 1 ? int.parse(args[1]) : 50051;
  print('=== SystemService Test | $host:$port ===\n');

  final ch = ClientChannel(host, port: port,
      options: const ChannelOptions(credentials: ChannelCredentials.insecure()));
  final c = SystemServiceClient(ch);
  final opt = CallOptions(timeout: const Duration(seconds: 5));

  try {
    // GetRobotInfo
    final info = await c.getRobotInfo(Empty(), options: opt);
    print('[PASS] GetRobotInfo');
    print('  ID: ${info.robotId}  Name: ${info.displayName}');
    print('  FW: ${info.firmwareVersion}  SW: ${info.softwareVersion}');

    // Heartbeat
    final hb = await c.heartbeat(HeartbeatRequest(), options: opt);
    print('[PASS] Heartbeat  ts=${hb.serverTimestamp}');

    // GetCapabilities
    final cap = await c.getCapabilities(Empty(), options: opt);
    print('[PASS] Capabilities');
    print('  Resources: ${cap.supportedResources}');
    print('  Tasks: ${cap.supportedTasks}');
  } catch (e) {
    print('[FAIL] $e');
  }

  await ch.shutdown();
  exit(0);
}
