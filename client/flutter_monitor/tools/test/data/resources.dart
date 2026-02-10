/// 测试 DataService.ListResources
/// dart run tools/test/data/resources.dart [host] [port]
import 'dart:io';
import 'package:grpc/grpc.dart';
import 'package:robot_proto/src/data.pbgrpc.dart';
import 'package:protobuf/well_known_types/google/protobuf/empty.pb.dart';

void main(List<String> args) async {
  final host = args.isNotEmpty ? args[0] : '192.168.66.190';
  final port = args.length > 1 ? int.parse(args[1]) : 50051;
  print('=== ListResources Test | $host:$port ===\n');

  final ch = ClientChannel(host, port: port,
      options: const ChannelOptions(credentials: ChannelCredentials.insecure()));
  final c = DataServiceClient(ch);

  try {
    final resp = await c.listResources(Empty(),
        options: CallOptions(timeout: const Duration(seconds: 5)));
    print('[PASS] ${resp.resources.length} resources:');
    for (final r in resp.resources) {
      print('  ${r.id.name} (${r.id.type}) avail=${r.available} | ${r.description}');
    }
  } catch (e) {
    print('[FAIL] $e');
  }

  await ch.shutdown();
  exit(0);
}
