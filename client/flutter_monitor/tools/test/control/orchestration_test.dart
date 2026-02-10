/// 测试服务编排: SetMode 切换后验证 systemd 服务实际启停
/// dart run tools/test/control/orchestration_test.dart [host] [port]
///
/// 测试流程:
///   1. 获取初始服务状态 (通过 systemctl 查询)
///   2. SetMode MAPPING → 验证 nav-lidar + nav-slam active
///   3. SetMode IDLE → 验证服务保持 (IDLE 保留 lidar+slam)
///   4. AcquireLease + SetMode TELEOP → 验证 nav-autonomy 也启动
///   5. SetMode IDLE → 验证 nav-autonomy 停止, lidar+slam 保留
///   6. 清理: ReleaseLease
///
/// 需要: grpc_gateway (50051) 运行; 本地执行 (通过 systemctl 查服务状态)
import 'dart:io';
import 'package:grpc/grpc.dart';
import 'package:robot_proto/src/control.pbgrpc.dart';
import 'package:robot_proto/src/common.pb.dart';

RequestBase _base(String tag) =>
    RequestBase()..requestId = 'orch-$tag-${DateTime.now().millisecondsSinceEpoch}';

// ── 服务状态查询 (通过 systemctl) ──

Future<Map<String, String>> getServiceStates(_) async {
  final services = [
    'nav-lidar.service',
    'nav-slam.service',
    'nav-autonomy.service',
    'nav-planning.service',
  ];
  final map = <String, String>{};
  for (final svc in services) {
    try {
      final result = await Process.run('systemctl', ['is-active', svc]);
      map[svc] = (result.stdout as String).trim();
    } catch (_) {
      map[svc] = 'unknown';
    }
  }
  return map;
}

void printServiceStates(Map<String, String> states) {
  if (states.isEmpty) {
    print('  (no service info available)');
    return;
  }
  for (final entry in states.entries) {
    final indicator = entry.value == 'active' ? '●' : '○';
    print('  $indicator ${entry.key}: ${entry.value}');
  }
}

// ── 测试结果 ──

class TestResult {
  final String name;
  final bool pass;
  final String detail;
  TestResult(this.name, this.pass, this.detail);
  @override
  String toString() =>
      '  ${pass ? "[PASS]" : "[FAIL]"} $name${detail.isNotEmpty ? "  $detail" : ""}';
}

// ── 验证服务状态 ──

TestResult verifyServices(
  String testName,
  Map<String, String> states,
  Map<String, String> expected, // service_name → expected_state
) {
  if (states.isEmpty) {
    return TestResult(testName, false, 'no service info');
  }
  final failures = <String>[];
  for (final entry in expected.entries) {
    final actual = states[entry.key] ?? 'unknown';
    if (actual != entry.value) {
      failures.add('${entry.key}: expected ${entry.value}, got $actual');
    }
  }
  if (failures.isEmpty) {
    return TestResult(testName, true, '');
  }
  return TestResult(testName, false, failures.join('; '));
}

// ── main ──

void main(List<String> args) async {
  final host = args.isNotEmpty ? args[0] : '127.0.0.1';
  final grpcPort = args.length > 1 ? int.parse(args[1]) : 50051;
  print('╔══════════════════════════════════════════════════════════╗');
  print('║  Service Orchestration Test');
  print('║  gRPC: $host:$grpcPort  (systemctl for service status)');
  print('╚══════════════════════════════════════════════════════════╝\n');

  final grpcCh = ClientChannel(host, port: grpcPort,
      options: const ChannelOptions(credentials: ChannelCredentials.insecure()));
  final ctrl = ControlServiceClient(grpcCh);
  const ota = null; // 不需要 OTA client, 使用 systemctl 查状态

  final results = <TestResult>[];

  // 服务编排是异步的, 等待时间
  const waitSec = 8;

  // ──────────────────────────────────────────
  // T0: 初始状态
  // ──────────────────────────────────────────
  print('── T0: 初始服务状态 ──');
  var states = await getServiceStates(ota);
  printServiceStates(states);

  // 确保从 IDLE 开始
  await ctrl.setMode(
    SetModeRequest()
      ..base = _base('init')
      ..mode = RobotMode.ROBOT_MODE_IDLE,
    options: CallOptions(timeout: const Duration(seconds: 10)),
  );
  await Future.delayed(const Duration(seconds: 2));

  // ──────────────────────────────────────────
  // T1: SetMode MAPPING → lidar+slam should start
  // ──────────────────────────────────────────
  print('\n── T1: IDLE → MAPPING ──');
  var resp = await ctrl.setMode(
    SetModeRequest()
      ..base = _base('mapping')
      ..mode = RobotMode.ROBOT_MODE_MAPPING,
    options: CallOptions(timeout: const Duration(seconds: 10)),
  );
  print('  SetMode result: ${resp.base.errorCode.name} mode=${resp.currentMode.name}');
  print('  Waiting ${waitSec}s for services to start...');
  await Future.delayed(const Duration(seconds: waitSec));

  states = await getServiceStates(ota);
  printServiceStates(states);
  results.add(verifyServices('T1 MAPPING services', states, {
    'nav-lidar.service': 'active',
    'nav-slam.service': 'active',
  }));

  // ──────────────────────────────────────────
  // T2: SetMode IDLE → services should remain (IDLE keeps lidar+slam)
  // ──────────────────────────────────────────
  print('\n── T2: MAPPING → IDLE ──');
  resp = await ctrl.setMode(
    SetModeRequest()
      ..base = _base('idle1')
      ..mode = RobotMode.ROBOT_MODE_IDLE,
    options: CallOptions(timeout: const Duration(seconds: 10)),
  );
  print('  SetMode result: ${resp.base.errorCode.name} mode=${resp.currentMode.name}');
  await Future.delayed(const Duration(seconds: 4));

  states = await getServiceStates(ota);
  printServiceStates(states);
  results.add(verifyServices('T2 IDLE services (lidar+slam kept)', states, {
    'nav-lidar.service': 'active',
    'nav-slam.service': 'active',
  }));

  // ──────────────────────────────────────────
  // T3: AcquireLease + SetMode TELEOP → autonomy should start
  // ──────────────────────────────────────────
  print('\n── T3: AcquireLease + IDLE → TELEOP ──');
  String leaseToken = '';
  try {
    final leaseResp = await ctrl.acquireLease(
      AcquireLeaseRequest()..base = _base('lease'),
      options: CallOptions(timeout: const Duration(seconds: 5)),
    );
    leaseToken = leaseResp.lease.leaseToken;
    print('  Lease acquired: ${leaseToken.substring(0, 8)}...');
  } catch (e) {
    print('  [WARN] AcquireLease failed: $e');
  }

  resp = await ctrl.setMode(
    SetModeRequest()
      ..base = _base('teleop')
      ..mode = RobotMode.ROBOT_MODE_TELEOP,
    options: CallOptions(timeout: const Duration(seconds: 10)),
  );
  print('  SetMode result: ${resp.base.errorCode.name} mode=${resp.currentMode.name}');
  print('  Waiting ${waitSec}s for autonomy to start...');
  await Future.delayed(const Duration(seconds: waitSec));

  states = await getServiceStates(ota);
  printServiceStates(states);
  results.add(verifyServices('T3 TELEOP services', states, {
    'nav-lidar.service': 'active',
    'nav-slam.service': 'active',
    'nav-autonomy.service': 'active',
  }));

  // ──────────────────────────────────────────
  // T4: TELEOP → IDLE → autonomy should stop
  // ──────────────────────────────────────────
  print('\n── T4: TELEOP → IDLE ──');
  resp = await ctrl.setMode(
    SetModeRequest()
      ..base = _base('idle2')
      ..mode = RobotMode.ROBOT_MODE_IDLE,
    options: CallOptions(timeout: const Duration(seconds: 10)),
  );
  print('  SetMode result: ${resp.base.errorCode.name} mode=${resp.currentMode.name}');
  print('  Waiting ${waitSec}s for autonomy to stop...');
  await Future.delayed(const Duration(seconds: waitSec));

  states = await getServiceStates(ota);
  printServiceStates(states);
  results.add(verifyServices('T4 IDLE services (autonomy stopped)', states, {
    'nav-lidar.service': 'active',
    'nav-slam.service': 'active',
    'nav-autonomy.service': 'inactive',
  }));

  // ──────────────────────────────────────────
  // T5: 清理 — ReleaseLease
  // ──────────────────────────────────────────
  print('\n── T5: 清理 ──');
  if (leaseToken.isNotEmpty) {
    try {
      await ctrl.releaseLease(
        ReleaseLeaseRequest()
          ..base = _base('release')
          ..leaseToken = leaseToken,
        options: CallOptions(timeout: const Duration(seconds: 5)),
      );
      print('  Lease released');
      results.add(TestResult('T5 ReleaseLease', true, ''));
    } catch (e) {
      results.add(TestResult('T5 ReleaseLease', false, '$e'));
    }
  }

  // ──────────────────────────────────────────
  // 报告
  // ──────────────────────────────────────────
  print('\n╔══════════════════════════════════════════════════════════╗');
  print('║  Test Results');
  print('╚══════════════════════════════════════════════════════════╝');
  for (final r in results) {
    print(r);
  }
  final passed = results.where((r) => r.pass).length;
  final total = results.length;
  print('\n$passed / $total passed');

  await grpcCh.shutdown();
  exit(passed == total ? 0 : 1);
}
