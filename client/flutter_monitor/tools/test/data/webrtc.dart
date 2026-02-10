/// 测试 DataService.WebRTCSignaling 信令握手
/// dart run tools/test/data/webrtc.dart [host] [port]
import 'dart:async';
import 'dart:io';
import 'package:grpc/grpc.dart';
import 'package:robot_proto/src/data.pbgrpc.dart';
import 'package:robot_proto/src/data.pb.dart';

void main(List<String> args) async {
  final host = args.isNotEmpty ? args[0] : '192.168.66.190';
  final port = args.length > 1 ? int.parse(args[1]) : 50051;
  print('=== WebRTC Signaling Test | $host:$port ===\n');

  final ch = ClientChannel(host, port: port,
      options: const ChannelOptions(credentials: ChannelCredentials.insecure()));
  final c = DataServiceClient(ch);
  final ctrl = StreamController<WebRTCSignal>();

  try {
    final stream = c.webRTCSignaling(ctrl.stream);
    ctrl.add(WebRTCSignal()
      ..sessionId = 'test-001'
      ..type = WebRTCSignalType.WEBRTC_SIGNAL_TYPE_OFFER
      ..sdp = 'v=0\r\no=- 0 0 IN IP4 127.0.0.1\r\ns=-\r\nt=0 0\r\n'
      ..config = (WebRTCSessionConfig()..videoEnabled = true..cameraId = 'front'));

    await for (final r in stream.timeout(const Duration(seconds: 5))) {
      print('[RECV] type=${r.type}  sdp=${r.sdp.length} bytes');
      if (r.type == WebRTCSignalType.WEBRTC_SIGNAL_TYPE_ANSWER ||
          r.type == WebRTCSignalType.WEBRTC_SIGNAL_TYPE_ICE_CANDIDATE) {
        print('[PASS] Signaling OK');
        break;
      }
    }
  } on TimeoutException {
    print('[FAIL] Timeout - no response in 5s');
  } catch (e) {
    print('[FAIL] $e');
  }

  await ctrl.close();
  await ch.shutdown();
  exit(0);
}
