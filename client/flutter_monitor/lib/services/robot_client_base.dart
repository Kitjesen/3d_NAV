import '../generated/common.pb.dart';
import '../generated/control.pb.dart';
import '../generated/telemetry.pb.dart';
import '../generated/data.pb.dart';
import '../generated/data.pbgrpc.dart';

abstract class RobotClientBase {
  Future<bool> connect();

  Stream<FastState> streamFastState({double desiredHz = 10.0});
  Stream<SlowState> streamSlowState();
  Stream<Event> streamEvents({String lastEventId = ''});
  Stream<DataChunk> subscribeToResource(ResourceId resourceId);

  Future<bool> acquireLease();
  Future<void> releaseLease();
  Stream<TeleopFeedback> streamTeleop(Stream<Twist> velocityStream);
  Future<bool> setMode(RobotMode mode);
  Future<bool> emergencyStop({bool hardStop = false});
  Future<void> ackEvent(String eventId);

  Future<void> disconnect();
  bool get isConnected;
  
  /// DataServiceClient for WebRTC signaling (null if mock/unsupported)
  DataServiceClient? get dataServiceClient => null;
}
