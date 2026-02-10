/// 测试 TelemetryService.StreamSlowState (模式/电池/CPU/健康)
/// dart run tools/test/telemetry/slow_state.dart [host] [port] [seconds]
import 'dart:async';
import 'dart:io';
import 'package:grpc/grpc.dart';
import 'package:robot_proto/src/telemetry.pbgrpc.dart';

void main(List<String> args) async {
  final host = args.isNotEmpty ? args[0] : '192.168.66.190';
  final port = args.length > 1 ? int.parse(args[1]) : 50051;
  final dur = args.length > 2 ? int.parse(args[2]) : 3;
  print('=== SlowState Test | $host:$port | ${dur}s ===\n');

  final ch = ClientChannel(host, port: port,
      options: const ChannelOptions(credentials: ChannelCredentials.insecure()));
  final c = TelemetryServiceClient(ch);
  int count = 0;

  try {
    await for (final s in c.streamSlowState(SlowStateRequest())
        .timeout(Duration(seconds: dur))) {
      count++;
      if (count == 1) {
        print('Frame 1:');
        print('  Mode: ${s.currentMode}');
        print('  Battery: ${s.resources.batteryPercent.toStringAsFixed(1)}% ${s.resources.batteryVoltage.toStringAsFixed(1)}V');
        print('  CPU: ${s.resources.cpuPercent.toStringAsFixed(1)}% ${s.resources.cpuTemp.toStringAsFixed(1)}C');
        print('  Mem: ${s.resources.memPercent.toStringAsFixed(1)}%');
        print('  Health: ${s.health.overallLevel}');
      }
    }
  } on TimeoutException { /* normal */ }

  print('\n$count frames');
  await ch.shutdown();
  exit(0);
}
