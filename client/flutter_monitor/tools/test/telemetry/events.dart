/// 测试 TelemetryService.StreamEvents
/// dart run tools/test/telemetry/events.dart [host] [port] [seconds]
import 'dart:async';
import 'dart:io';
import 'package:grpc/grpc.dart';
import 'package:robot_proto/src/telemetry.pbgrpc.dart';

void main(List<String> args) async {
  final host = args.isNotEmpty ? args[0] : '192.168.66.190';
  final port = args.length > 1 ? int.parse(args[1]) : 50051;
  final dur = args.length > 2 ? int.parse(args[2]) : 5;
  print('=== Events Test | $host:$port | ${dur}s ===\n');

  final ch = ClientChannel(host, port: port,
      options: const ChannelOptions(credentials: ChannelCredentials.insecure()));
  final c = TelemetryServiceClient(ch);
  int count = 0;

  try {
    await for (final e in c.streamEvents(EventStreamRequest())
        .timeout(Duration(seconds: dur))) {
      count++;
      print('  [${e.severity}] ${e.title}: ${e.description}');
    }
  } on TimeoutException { /* normal */ }

  print('\n$count events (${count == 0 ? "none in ${dur}s, may be normal" : "OK"})');
  await ch.shutdown();
  exit(0);
}
