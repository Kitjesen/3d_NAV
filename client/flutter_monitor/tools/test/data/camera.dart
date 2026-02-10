/// 测试 DataService.Subscribe 摄像头 JPEG 数据
/// dart run tools/test/data/camera.dart [host] [port] [seconds]
import 'dart:io';
import 'dart:math';
import 'package:grpc/grpc.dart';
import 'package:robot_proto/src/data.pbgrpc.dart';
import 'package:robot_proto/src/common.pb.dart';

void main(List<String> args) async {
  final host = args.isNotEmpty ? args[0] : '192.168.66.190';
  final port = args.length > 1 ? int.parse(args[1]) : 50051;
  final dur = args.length > 2 ? int.parse(args[2]) : 5;
  print('=== Camera Test | $host:$port | ${dur}s ===\n');

  final ch = ClientChannel(host, port: port,
      options: const ChannelOptions(credentials: ChannelCredentials.insecure()));
  final c = DataServiceClient(ch);

  final rid = Random().nextInt(999999).toString().padLeft(6, '0');
  final req = SubscribeRequest()
    ..base = (RequestBase()..requestId = 'cam-$rid')
    ..resourceId = (ResourceId()
      ..name = 'front'
      ..type = ResourceType.RESOURCE_TYPE_CAMERA)
    ..profile = (SubscribeProfile()..frequency = 30.0);

  int count = 0, totalBytes = 0;
  final sw = Stopwatch()..start();

  try {
    await for (final chunk in c.subscribe(req)) {
      count++;
      totalBytes += chunk.data.length;
      if (count == 1) {
        final isJpeg = chunk.data.length > 2 && chunk.data[0] == 0xFF && chunk.data[1] == 0xD8;
        print('Frame 1: ${chunk.data.length} bytes  JPEG: ${isJpeg ? "YES" : "NO"}');
      }
      if (sw.elapsedMilliseconds > dur * 1000) break;
    }
  } catch (e) {
    print('[ERROR] $e');
  }

  sw.stop();
  final elapsed = sw.elapsedMilliseconds / 1000.0;
  if (count > 0) {
    print('\n[PASS] $count frames | ${(count / elapsed).toStringAsFixed(1)} fps | avg ${(totalBytes / count / 1024).toStringAsFixed(1)} KB');
  } else {
    print('\n[FAIL] 0 frames in ${dur}s');
    print('  Check: ros2 topic hz /camera/color/image_raw/compressed');
  }
  await ch.shutdown();
  exit(count > 0 ? 0 : 1);
}
