#!/usr/bin/env python3
"""
gRPC ↔ ROS2 话题桥接验证测试

测试 gRPC gateway 与 ROS2 话题之间的通信是否正常。
不依赖 Fast-LIO2 等特定节点是否运行，仅验证桥接本身。

Usage:
    python3 test_grpc_ros2_bridge.py [--host HOST] [--port PORT]
"""

import argparse
import json
import os
import subprocess
import sys
import threading
import time
import uuid


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

class Colors:
    GREEN = "\033[92m"
    RED = "\033[91m"
    YELLOW = "\033[93m"
    CYAN = "\033[96m"
    BOLD = "\033[1m"
    RESET = "\033[0m"


def _c(color, text):
    return f"{color}{text}{Colors.RESET}"


def _rid():
    return str(uuid.uuid4())[:8]


def grpcurl(host, port, method, data=None, timeout=10):
    """Call grpcurl and return parsed JSON (or empty dict on failure)."""
    cmd = ["grpcurl", "-plaintext"]
    if data is not None:
        cmd += ["-d", json.dumps(data) if isinstance(data, dict) else data]
    cmd += [f"{host}:{port}", method]
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        if result.stdout.strip():
            return json.loads(result.stdout)
        return {}
    except (subprocess.TimeoutExpired, json.JSONDecodeError):
        return {}


def grpcurl_stream(host, port, method, data=None, timeout=4):
    """Call a streaming grpcurl, return raw stdout."""
    cmd = ["grpcurl", "-plaintext"]
    if data is not None:
        cmd += ["-d", json.dumps(data) if isinstance(data, dict) else data]
    cmd += [f"{host}:{port}", method]
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return result.stdout
    except subprocess.TimeoutExpired as e:
        stdout = e.stdout.decode("utf-8", errors="replace") if e.stdout else ""
        return stdout


def ros2_echo_background(topic, duration=6):
    """Run ros2 topic echo in background, return collected output."""
    output_file = f"/tmp/grpc_test_{topic.replace('/', '_')}.txt"
    cmd = f"timeout {duration} ros2 topic echo {topic} > {output_file} 2>&1"
    proc = subprocess.Popen(cmd, shell=True)
    return proc, output_file


def read_and_cleanup(output_file):
    """Read output file and clean up."""
    try:
        with open(output_file, "r") as f:
            content = f.read()
        os.remove(output_file)
        return content
    except FileNotFoundError:
        return ""


# ---------------------------------------------------------------------------
# Test class
# ---------------------------------------------------------------------------

class BridgeTestRunner:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.results = []

    def record(self, name, passed, detail=""):
        self.results.append((name, passed, detail))
        status = _c(Colors.GREEN, "PASS") if passed else _c(Colors.RED, "FAIL")
        detail_str = f"  ({detail})" if detail else ""
        print(f"  [{status}] {name}{detail_str}")

    def call(self, method, data=None, timeout=10):
        return grpcurl(self.host, self.port, method, data, timeout)

    def stream(self, method, data=None, timeout=4):
        return grpcurl_stream(self.host, self.port, method, data, timeout)

    def _reset_state(self):
        """Reset to IDLE and acquire lease."""
        self.call("robot.v1.ControlService/SetMode",
                  {"base": {"request_id": _rid()}, "mode": 1})
        time.sleep(0.3)
        resp = self.call("robot.v1.ControlService/AcquireLease",
                         {"base": {"request_id": _rid()}})
        lease = resp.get("lease", {}).get("lease_token",
                resp.get("lease", {}).get("leaseToken", ""))
        return lease

    def summary(self):
        total = len(self.results)
        passed = sum(1 for _, p, _ in self.results if p)
        failed = total - passed

        print()
        print(_c(Colors.BOLD, "=" * 60))
        if failed == 0:
            print(_c(Colors.GREEN + Colors.BOLD,
                      f"  ALL TESTS PASSED: {passed}/{total}"))
        else:
            print(_c(Colors.YELLOW + Colors.BOLD,
                      f"  RESULT: {passed}/{total} passed, {failed} failed"))
            for name, p, detail in self.results:
                if not p:
                    print(f"    - {name}: {detail}")
        print(_c(Colors.BOLD, "=" * 60))
        return 0 if failed == 0 else 1


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def test_01_grpc_connection(t: BridgeTestRunner):
    """gRPC 服务可连接且所有服务已注册"""
    cmd = ["grpcurl", "-plaintext", f"{t.host}:{t.port}", "list"]
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
        services = result.stdout.strip().split("\n")
        expected = ["robot.v1.ControlService", "robot.v1.DataService",
                    "robot.v1.SystemService", "robot.v1.TelemetryService"]
        found = [s for s in expected if s in services]
        t.record("gRPC连接+服务注册", len(found) == 4,
                 f"找到 {len(found)}/4 个服务")
    except Exception as e:
        t.record("gRPC连接+服务注册", False, str(e))


def test_02_get_robot_info(t: BridgeTestRunner):
    """GetRobotInfo 返回有效数据"""
    resp = t.call("robot.v1.SystemService/GetRobotInfo")
    robot_id = resp.get("robot_id", resp.get("robotId", ""))
    t.record("GetRobotInfo", robot_id != "",
             f"robot_id={robot_id}")


def test_03_get_capabilities(t: BridgeTestRunner):
    """GetCapabilities 返回资源和任务列表"""
    resp = t.call("robot.v1.SystemService/GetCapabilities")
    resources = resp.get("supported_resources",
                         resp.get("supportedResources", []))
    tasks = resp.get("supported_tasks",
                     resp.get("supportedTasks", []))
    ok = len(resources) > 0 and len(tasks) > 0
    t.record("GetCapabilities", ok,
             f"resources={resources}, tasks={tasks}")


def test_04_lease_acquire_release(t: BridgeTestRunner):
    """Lease 获取和释放"""
    resp = t.call("robot.v1.ControlService/AcquireLease",
                  {"base": {"request_id": _rid()}})
    lease = resp.get("lease", {})
    token = lease.get("lease_token", lease.get("leaseToken", ""))
    has_token = len(token) > 0

    # Release
    resp2 = t.call("robot.v1.ControlService/ReleaseLease",
                   {"base": {"request_id": _rid()}, "lease_token": token})
    err_code = resp2.get("base", {}).get("error_code",
               resp2.get("base", {}).get("errorCode", ""))
    released = err_code == "ERROR_CODE_OK"
    t.record("Lease获取+释放", has_token and released,
             f"token={token[:20]}..., released={released}")


def test_05_mode_switch_teleop(t: BridgeTestRunner):
    """SetMode(TELEOP) 并验证 /speed 话题发布"""
    t._reset_state()

    # Start listening to /speed
    proc, outfile = ros2_echo_background("/speed", duration=8)
    time.sleep(3)  # let subscriber fully settle

    # Switch to TELEOP
    resp = t.call("robot.v1.ControlService/SetMode",
                  {"base": {"request_id": _rid()}, "mode": 3})
    mode = resp.get("current_mode", resp.get("currentMode", ""))

    time.sleep(4)
    proc.wait()
    output = read_and_cleanup(outfile)

    mode_ok = mode == "ROBOT_MODE_TELEOP"
    has_speed = "data:" in output
    t.record("SetMode(TELEOP)→gRPC返回", mode_ok, f"mode={mode}")
    t.record("SetMode(TELEOP)→/speed话题", has_speed,
             f"收到 /speed 数据: {'是' if has_speed else '否'}")

    # Reset
    t.call("robot.v1.ControlService/SetMode",
           {"base": {"request_id": _rid()}, "mode": 1})


def test_06_emergency_stop(t: BridgeTestRunner):
    """EmergencyStop 并验证 /stop 话题发布"""
    t._reset_state()
    time.sleep(0.3)

    # Start listening to /stop
    proc, outfile = ros2_echo_background("/stop", duration=10)
    time.sleep(4)  # let subscriber fully settle (DDS discovery can be slow)

    # Trigger ESTOP
    resp = t.call("robot.v1.ControlService/EmergencyStop",
                  {"base": {"request_id": _rid()}})
    stopped = resp.get("stopped", False)

    time.sleep(4)
    proc.wait()
    output = read_and_cleanup(outfile)

    has_stop = "data: 1" in output
    t.record("EmergencyStop→gRPC返回", stopped, f"stopped={stopped}")
    t.record("EmergencyStop→/stop话题", has_stop,
             f"收到 /stop=1: {'是' if has_stop else '否'}")

    # Reset to idle
    t.call("robot.v1.ControlService/SetMode",
           {"base": {"request_id": _rid()}, "mode": 1})


def test_07_fast_state_stream(t: BridgeTestRunner):
    """StreamFastState 遥测流"""
    out = t.stream("robot.v1.TelemetryService/StreamFastState", "{}", timeout=4)
    has_data = "header" in out or "pose" in out or "velocity" in out
    lines = len(out.split("\n"))
    t.record("StreamFastState遥测", has_data,
             f"收到数据: {'是' if has_data else '否'} ({lines} 行)")


def test_08_slow_state_stream(t: BridgeTestRunner):
    """StreamSlowState 含 health/topic_rates"""
    out = t.stream("robot.v1.TelemetryService/StreamSlowState", "{}", timeout=4)
    has_health = "health" in out
    has_topic_rates = "topic_rates" in out or "topicRates" in out
    has_resources = "cpu_percent" in out or "cpuPercent" in out
    has_mode = "current_mode" in out or "currentMode" in out
    t.record("SlowState-health字段", has_health,
             f"health={'存在' if has_health else '缺失'}")
    t.record("SlowState-topicRates字段", has_topic_rates,
             f"topicRates={'存在' if has_topic_rates else '缺失'}")
    t.record("SlowState-系统资源", has_resources,
             f"cpu/mem={'存在' if has_resources else '缺失'}")
    t.record("SlowState-模式信息", has_mode,
             f"mode={'存在' if has_mode else '缺失'}")


def test_09_list_resources(t: BridgeTestRunner):
    """ListResources 返回可订阅的数据资源"""
    resp = t.call("robot.v1.DataService/ListResources")
    resources = resp.get("resources", [])
    names = [r.get("id", {}).get("name", "") for r in resources]
    t.record("ListResources", len(names) > 0,
             f"资源列表: {names}")


def test_10_relocalize_service_bridge(t: BridgeTestRunner):
    """Relocalize gRPC → ROS2 service 桥接"""
    resp = t.call("robot.v1.SystemService/Relocalize", {
        "base": {"request_id": _rid()},
        "pcd_path": "/tmp/nonexistent.pcd",
        "x": 0, "y": 0, "z": 0,
    }, timeout=15)
    err_code = resp.get("base", {}).get("error_code",
               resp.get("base", {}).get("errorCode", ""))
    # Localizer not running → should get SERVICE_UNAVAILABLE or TIMEOUT
    ok = err_code in ("ERROR_CODE_SERVICE_UNAVAILABLE", "ERROR_CODE_TIMEOUT")
    t.record("Relocalize→ROS2服务桥接", ok,
             f"error_code={err_code} (符合预期: localizer未运行)")


def test_11_save_map_service_bridge(t: BridgeTestRunner):
    """SaveMap gRPC → ROS2 service 桥接"""
    resp = t.call("robot.v1.SystemService/SaveMap", {
        "base": {"request_id": _rid()},
        "file_path": "/tmp/test_map.pcd",
    }, timeout=15)
    err_code = resp.get("base", {}).get("error_code",
               resp.get("base", {}).get("errorCode", ""))
    # FastLIO2 运行时返回 OK，未运行时返回 SERVICE_UNAVAILABLE
    ok = err_code in ("ERROR_CODE_OK", "ERROR_CODE_SERVICE_UNAVAILABLE", "ERROR_CODE_TIMEOUT")
    status_msg = ("保存成功" if err_code == "ERROR_CODE_OK"
                  else "服务不可用(fastlio未运行)" if "UNAVAILABLE" in err_code
                  else err_code)
    t.record("SaveMap→ROS2服务桥接", ok,
             f"error_code={err_code} ({status_msg})")


def test_12_topic_health_detection(t: BridgeTestRunner):
    """HealthMonitor 正确检测话题状态"""
    out = t.stream("robot.v1.TelemetryService/StreamSlowState", "{}", timeout=4)
    
    # Parse health subsystems
    slam_detected = "slam" in out.lower()
    terrain_detected = "terrain" in out.lower()
    tf_detected = "tf" in out.lower()
    
    t.record("HealthMonitor-SLAM检测", slam_detected,
             f"SLAM状态{'已报告' if slam_detected else '未报告'}")
    t.record("HealthMonitor-TF链检测", tf_detected,
             f"TF状态{'已报告' if tf_detected else '未报告'}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="gRPC ↔ ROS2 话题桥接验证测试")
    parser.add_argument("--host", default="localhost",
                        help="gRPC server host (default: localhost)")
    parser.add_argument("--port", default="50051",
                        help="gRPC server port (default: 50051)")
    args = parser.parse_args()

    t = BridgeTestRunner(args.host, args.port)

    # Pre-flight
    print(_c(Colors.BOLD, "\n=== 预检查 ===\n"))
    cmd = ["grpcurl", "-plaintext", f"{args.host}:{args.port}", "list"]
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
        if result.returncode != 0:
            raise RuntimeError(result.stderr)
        print(_c(Colors.GREEN, f"  已连接 gRPC 服务: {args.host}:{args.port}"))
    except Exception as e:
        print(_c(Colors.RED, f"  无法连接 gRPC 服务: {args.host}:{args.port}"))
        print(f"  错误: {e}")
        print(f"\n  请确保 gateway 已启动:")
        print(f"    ros2 launch remote_monitoring grpc_gateway.launch.py")
        sys.exit(1)

    # Run tests
    print(_c(Colors.BOLD, "\n=== 1. gRPC 服务基本功能 ===\n"))
    test_01_grpc_connection(t)
    test_02_get_robot_info(t)
    test_03_get_capabilities(t)
    test_04_lease_acquire_release(t)

    print(_c(Colors.BOLD, "\n=== 2. gRPC → ROS2 话题桥接 ===\n"))
    test_05_mode_switch_teleop(t)
    test_06_emergency_stop(t)

    print(_c(Colors.BOLD, "\n=== 3. ROS2 → gRPC 遥测桥接 ===\n"))
    test_07_fast_state_stream(t)
    test_08_slow_state_stream(t)

    print(_c(Colors.BOLD, "\n=== 4. DataService 资源发现 ===\n"))
    test_09_list_resources(t)

    print(_c(Colors.BOLD, "\n=== 5. ROS2 Service 桥接 ===\n"))
    test_10_relocalize_service_bridge(t)
    test_11_save_map_service_bridge(t)

    print(_c(Colors.BOLD, "\n=== 6. HealthMonitor 系统监控 ===\n"))
    test_12_topic_health_detection(t)

    # Summary
    sys.exit(t.summary())


if __name__ == "__main__":
    main()
