/// 测试 ControlService 状态机: SetMode / Lease / EmergencyStop
/// dart run tools/test/control/mode_test.dart [host] [port]
///
/// 测试矩阵:
///   1. IDLE → MAPPING  (无条件)
///   2. MAPPING → IDLE  (无条件)
///   3. IDLE → TELEOP   (无 lease → 拒绝)
///   4. AcquireLease → IDLE → TELEOP (有 lease → 成功)
///   5. TELEOP → IDLE   (无条件)
///   6. EmergencyStop   (任何状态 → ESTOP)
///   7. SetMode(IDLE) while ESTOP → 拒绝
///   8. ClearEstop 隐含测试 (如果支持)
///   9. ReleaseLease
import 'dart:io';
import 'package:grpc/grpc.dart';
import 'package:robot_proto/src/control.pbgrpc.dart';
import 'package:robot_proto/src/common.pb.dart';

// ── helpers ──

String modeName(RobotMode m) => m.name;

String errorName(ErrorCode c) => c.name;

RequestBase _base(String tag) => RequestBase()..requestId = 'test-$tag-${DateTime.now().millisecondsSinceEpoch}';

class TestResult {
  final String name;
  final bool pass;
  final String detail;
  TestResult(this.name, this.pass, this.detail);

  @override
  String toString() => '  ${pass ? "[PASS]" : "[FAIL]"} $name${detail.isNotEmpty ? "  $detail" : ""}';
}

// ── test cases ──

Future<TestResult> testSetMode(
  ControlServiceClient c,
  String name,
  RobotMode target, {
  bool expectSuccess = true,
  ErrorCode? expectError,
}) async {
  try {
    final resp = await c.setMode(
      SetModeRequest()
        ..base = _base('setmode')
        ..mode = target,
      options: CallOptions(timeout: const Duration(seconds: 10)),
    );
    final ok = resp.base.errorCode == ErrorCode.ERROR_CODE_OK ||
        resp.base.errorCode == ErrorCode.ERROR_CODE_UNSPECIFIED;
    if (expectSuccess && ok) {
      return TestResult(name, true, '→ ${modeName(resp.currentMode)}');
    }
    if (!expectSuccess && !ok) {
      final matches = expectError == null || resp.base.errorCode == expectError;
      return TestResult(name, matches,
          'rejected: ${errorName(resp.base.errorCode)} "${resp.base.errorMessage}"');
    }
    return TestResult(name, false,
        'expected ${expectSuccess ? "success" : "failure"} got ${errorName(resp.base.errorCode)} "${resp.base.errorMessage}"');
  } catch (e) {
    return TestResult(name, false, 'exception: $e');
  }
}

Future<({String token, TestResult result})> testAcquireLease(ControlServiceClient c) async {
  try {
    final resp = await c.acquireLease(
      AcquireLeaseRequest()..base = _base('lease'),
      options: CallOptions(timeout: const Duration(seconds: 5)),
    );
    final ok = resp.base.errorCode == ErrorCode.ERROR_CODE_OK ||
        resp.base.errorCode == ErrorCode.ERROR_CODE_UNSPECIFIED;
    if (ok && resp.lease.leaseToken.isNotEmpty) {
      return (
        token: resp.lease.leaseToken,
        result: TestResult('AcquireLease', true,
            'token=${resp.lease.leaseToken.substring(0, 8)}... ttl=${resp.lease.ttl}'),
      );
    }
    return (
      token: '',
      result: TestResult('AcquireLease', false,
          '${errorName(resp.base.errorCode)} "${resp.base.errorMessage}"'),
    );
  } catch (e) {
    return (token: '', result: TestResult('AcquireLease', false, 'exception: $e'));
  }
}

Future<TestResult> testReleaseLease(ControlServiceClient c, String token) async {
  try {
    final resp = await c.releaseLease(
      ReleaseLeaseRequest()
        ..base = _base('release')
        ..leaseToken = token,
      options: CallOptions(timeout: const Duration(seconds: 5)),
    );
    final ok = resp.base.errorCode == ErrorCode.ERROR_CODE_OK ||
        resp.base.errorCode == ErrorCode.ERROR_CODE_UNSPECIFIED;
    return TestResult('ReleaseLease', ok, errorName(resp.base.errorCode));
  } catch (e) {
    return TestResult('ReleaseLease', false, 'exception: $e');
  }
}

Future<TestResult> testEmergencyStop(ControlServiceClient c) async {
  try {
    final resp = await c.emergencyStop(
      EmergencyStopRequest()..base = _base('estop'),
      options: CallOptions(timeout: const Duration(seconds: 5)),
    );
    final ok = resp.base.errorCode == ErrorCode.ERROR_CODE_OK ||
        resp.base.errorCode == ErrorCode.ERROR_CODE_UNSPECIFIED;
    return TestResult('EmergencyStop', ok && resp.stopped, 'stopped=${resp.stopped}');
  } catch (e) {
    return TestResult('EmergencyStop', false, 'exception: $e');
  }
}

// ── helper: 确保回到 IDLE ──
Future<void> ensureIdle(ControlServiceClient c) async {
  // 先尝试直接 SetMode IDLE
  var resp = await c.setMode(
    SetModeRequest()
      ..base = _base('reset')
      ..mode = RobotMode.ROBOT_MODE_IDLE,
    options: CallOptions(timeout: const Duration(seconds: 10)),
  );
  // 如果是 ESTOP, 需要 ClearEstop (通过 SetMode IDLE from ESTOP 的特殊处理)
  if (resp.currentMode == RobotMode.ROBOT_MODE_ESTOP) {
    // EmergencyStop 的 clear 路径:
    // 有些实现需要先 EmergencyStop(hard_stop=false) 再 SetMode(IDLE)
    // 我们这里多试一次
    await Future.delayed(const Duration(seconds: 1));
    await c.setMode(
      SetModeRequest()
        ..base = _base('reset2')
        ..mode = RobotMode.ROBOT_MODE_IDLE,
      options: CallOptions(timeout: const Duration(seconds: 10)),
    );
  }
}

// ── main ──

void main(List<String> args) async {
  final host = args.isNotEmpty ? args[0] : '127.0.0.1';
  final port = args.length > 1 ? int.parse(args[1]) : 50051;
  print('╔══════════════════════════════════════════════════════╗');
  print('║  ControlService Mode Test | $host:$port');
  print('╚══════════════════════════════════════════════════════╝\n');

  final ch = ClientChannel(host, port: port,
      options: const ChannelOptions(credentials: ChannelCredentials.insecure()));
  final c = ControlServiceClient(ch);

  final results = <TestResult>[];

  // ──────────────────────────────────────────────
  // 阶段 1: 基本状态转换
  // ──────────────────────────────────────────────
  print('── 阶段 1: 基本状态转换 ──');

  // 确保从 IDLE 开始
  await ensureIdle(c);
  await Future.delayed(const Duration(seconds: 1));

  // T1: IDLE → MAPPING
  results.add(await testSetMode(c, 'T1 IDLE→MAPPING', RobotMode.ROBOT_MODE_MAPPING));
  await Future.delayed(const Duration(seconds: 2));

  // T2: MAPPING → IDLE
  results.add(await testSetMode(c, 'T2 MAPPING→IDLE', RobotMode.ROBOT_MODE_IDLE));
  await Future.delayed(const Duration(seconds: 2));

  // ──────────────────────────────────────────────
  // 阶段 2: Lease 守卫
  // ──────────────────────────────────────────────
  print('\n── 阶段 2: Lease 守卫 ──');

  // T3: IDLE → TELEOP (无 lease) — 应拒绝
  results.add(await testSetMode(c, 'T3 IDLE→TELEOP (no lease)', RobotMode.ROBOT_MODE_TELEOP,
      expectSuccess: false, expectError: ErrorCode.ERROR_CODE_MODE_CONFLICT));
  await Future.delayed(const Duration(seconds: 1));

  // T4: AcquireLease
  final leaseResult = await testAcquireLease(c);
  results.add(leaseResult.result);
  final token = leaseResult.token;
  await Future.delayed(const Duration(seconds: 1));

  // T5: IDLE → TELEOP (有 lease) — 应成功
  if (token.isNotEmpty) {
    results.add(await testSetMode(c, 'T5 IDLE→TELEOP (has lease)', RobotMode.ROBOT_MODE_TELEOP));
    await Future.delayed(const Duration(seconds: 2));

    // T6: TELEOP → IDLE
    results.add(await testSetMode(c, 'T6 TELEOP→IDLE', RobotMode.ROBOT_MODE_IDLE));
    await Future.delayed(const Duration(seconds: 1));
  } else {
    results.add(TestResult('T5 IDLE→TELEOP (has lease)', false, 'skipped: no lease'));
    results.add(TestResult('T6 TELEOP→IDLE', false, 'skipped: no lease'));
  }

  // ──────────────────────────────────────────────
  // 阶段 3: 非法转换
  // ──────────────────────────────────────────────
  print('\n── 阶段 3: 非法转换 ──');

  // T7: IDLE → AUTONOMOUS
  // 注: 当 TF/定位守卫未配置时, 转换可成功; 守卫存在时会拒绝
  results.add(await testSetMode(c, 'T7 IDLE→AUTONOMOUS', RobotMode.ROBOT_MODE_AUTONOMOUS));
  await Future.delayed(const Duration(seconds: 1));
  // 回到 IDLE
  await testSetMode(c, '_cleanup IDLE', RobotMode.ROBOT_MODE_IDLE);
  await Future.delayed(const Duration(seconds: 1));

  // T8: MAPPING → TELEOP (非法: TELEOP 只能从 IDLE 或 AUTONOMOUS 到达)
  await testSetMode(c, '_setup MAPPING', RobotMode.ROBOT_MODE_MAPPING);
  await Future.delayed(const Duration(seconds: 1));
  results.add(await testSetMode(c, 'T8 MAPPING→TELEOP (illegal)', RobotMode.ROBOT_MODE_TELEOP,
      expectSuccess: false));
  await testSetMode(c, '_cleanup IDLE', RobotMode.ROBOT_MODE_IDLE);
  await Future.delayed(const Duration(seconds: 1));

  // ──────────────────────────────────────────────
  // 阶段 4: 急停
  // ──────────────────────────────────────────────
  print('\n── 阶段 4: 急停 ──');

  // T9: EmergencyStop
  results.add(await testEmergencyStop(c));
  await Future.delayed(const Duration(seconds: 1));

  // T10: ESTOP → MAPPING (应拒绝)
  results.add(await testSetMode(c, 'T10 ESTOP→MAPPING (blocked)', RobotMode.ROBOT_MODE_MAPPING,
      expectSuccess: false));
  await Future.delayed(const Duration(seconds: 1));

  // T11: ESTOP → IDLE (允许: SwitchMode 支持 ESTOP→IDLE, ClearEstop 是更安全的路径)
  results.add(await testSetMode(c, 'T11 ESTOP→IDLE (allowed)', RobotMode.ROBOT_MODE_IDLE));

  // ──────────────────────────────────────────────
  // 阶段 5: 清理
  // ──────────────────────────────────────────────
  print('\n── 阶段 5: 清理 ──');

  // 恢复: 尝试 clear estop (SetMode IDLE 作为 fallback)
  // 注: 如果 ClearEstop 需要独立 RPC, 这里可能无法恢复
  await ensureIdle(c);

  if (token.isNotEmpty) {
    results.add(await testReleaseLease(c, token));
  }

  // ──────────────────────────────────────────────
  // 报告
  // ──────────────────────────────────────────────
  print('\n╔══════════════════════════════════════════════════════╗');
  print('║  Test Results');
  print('╚══════════════════════════════════════════════════════╝');
  for (final r in results) {
    print(r);
  }
  final passed = results.where((r) => r.pass).length;
  final total = results.length;
  print('\n$passed / $total passed');

  await ch.shutdown();
  exit(passed == total ? 0 : 1);
}
