/// 测试 OtaService.GetDeviceInfo
/// dart run tools/test/ota/device_info.dart [host] [port]
import 'dart:io';
import 'package:grpc/grpc.dart';
import 'package:robot_proto/src/data.pbgrpc.dart';
import 'package:protobuf/well_known_types/google/protobuf/empty.pb.dart';

void main(List<String> args) async {
  final host = args.isNotEmpty ? args[0] : '192.168.66.190';
  final port = args.length > 1 ? int.parse(args[1]) : 50052;
  print('=== OTA DeviceInfo Test | $host:$port ===\n');

  final ch = ClientChannel(host, port: port,
      options: const ChannelOptions(credentials: ChannelCredentials.insecure()));
  final c = OtaServiceClient(ch);

  try {
    final d = await c.getDeviceInfo(Empty(),
        options: CallOptions(timeout: const Duration(seconds: 5)));
    print('[PASS] OTA Daemon connected');
    print('  Hostname: ${d.hostname}');
    print('  Robot ID: ${d.robotId}');
    print('  IP: ${d.ipAddresses.join(", ")}');
    print('  Disk: ${(d.diskFreeBytes.toInt() / 1e9).toStringAsFixed(1)}G free / ${(d.diskTotalBytes.toInt() / 1e9).toStringAsFixed(1)}G total');
    print('  Battery: ${d.batteryPercent}%');
    print('  Uptime: ${(d.uptimeSeconds.toInt() / 3600).toStringAsFixed(1)}h');
    print('  OTA ver: ${d.otaDaemonVersion}');
    print('  Services:');
    for (final s in d.services) print('    ${s.name}: ${s.state} (${s.subState})');
  } catch (e) {
    print('[FAIL] $e');
  }

  await ch.shutdown();
  exit(0);
}
