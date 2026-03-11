#!/usr/bin/env python3
"""
Mission Arc 集成测试 — 验证任务生命周期状态机

在 S100P (192.168.66.190) 上执行:
1. 启动 mission_arc 节点
2. 模拟 planner_status / adapter_status / goal_pose 事件流
3. 验证 /nav/mission_status 状态转换正确

测试项:
  [M1] mission_arc 节点正常启动
  [M2] goal_pose → PLANNING 转换
  [M3] planner SUCCESS → EXECUTING 转换
  [M4] adapter waypoint_reached → 进度更新
  [M5] adapter goal_reached → COMPLETE 转换
  [M6] planner STUCK → RECOVERING → REPLANNING 恢复链
  [M7] planner FAILED → FAILED 状态
"""

import paramiko
import time
import json
import sys

ROBOT = "192.168.66.190"
USER = "sunrise"
PASS = "sunrise"
NAV_WS = "/home/sunrise/data/SLAM/navigation"
SETUP = f"source /opt/ros/humble/setup.bash && source {NAV_WS}/install/setup.bash"


def ssh_connect():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(ROBOT, username=USER, password=PASS, timeout=10)
    return ssh


def run_cmd(ssh, cmd, timeout=15):
    _, out, err = ssh.exec_command(f"{SETUP} && {cmd}", timeout=timeout)
    return out.read().decode(), err.read().decode()


def run_bg(ssh, cmd):
    full = f"{SETUP} && {cmd}"
    ssh.exec_command(f"nohup bash -c '{full}' > /tmp/mission_arc_bg.log 2>&1 &")
    time.sleep(0.5)


def kill_procs(ssh, names):
    for n in names:
        ssh.exec_command(f"pkill -f '{n}' 2>/dev/null")
    time.sleep(1)


def main():
    ssh = ssh_connect()
    print("SSH connected to", ROBOT)

    results = []

    def check(name, passed, detail=""):
        results.append({"name": name, "pass": passed})
        mark = "\033[32mPASS\033[0m" if passed else "\033[31mFAIL\033[0m"
        print(f"  [{mark}] {name}  {detail}")

    kill_procs(ssh, ["mission_arc", "mission_arc_test_harness"])

    try:
        # ================================================================
        # 写测试 harness
        # ================================================================
        harness = r'''#!/usr/bin/env python3
"""Mission Arc test: 模拟事件流 + 采集状态"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Int8
import json, time, sys

QOS_TL = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)

class Harness(Node):
    def __init__(self):
        super().__init__('mission_arc_test_harness')

        # Publishers — 模拟事件源
        self.goal_pub = self.create_publisher(PoseStamped, '/nav/goal_pose', QOS_TL)
        self.planner_pub = self.create_publisher(String, '/nav/planner_status', 10)
        self.adapter_pub = self.create_publisher(String, '/nav/adapter_status', 10)
        self.stop_pub = self.create_publisher(Int8, '/nav/stop', 10)

        # Subscriber — 采集 mission_status
        self.statuses = []
        self.create_subscription(String, '/nav/mission_status', self.status_cb, 10)

        self.step = 0
        self.create_timer(1.0, self.run_scenario)
        self.get_logger().info("Harness started")

    def status_cb(self, msg):
        try:
            data = json.loads(msg.data)
            self.statuses.append(data)
            state = data.get('state', '?')
            if len(self.statuses) <= 30:
                self.get_logger().info(f'  status: {state}  replan={data.get("replan_count",0)}')
        except:
            pass

    def pub_planner(self, status):
        m = String(); m.data = status
        self.planner_pub.publish(m)
        self.get_logger().info(f'  >> planner_status: {status}')

    def pub_adapter(self, event, **kw):
        data = {"event": event}; data.update(kw)
        m = String(); m.data = json.dumps(data)
        self.adapter_pub.publish(m)
        self.get_logger().info(f'  >> adapter_status: {event}')

    def pub_goal(self, x, y):
        m = PoseStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'map'
        m.pose.position.x = float(x)
        m.pose.position.y = float(y)
        m.pose.orientation.w = 1.0
        self.goal_pub.publish(m)
        self.get_logger().info(f'  >> goal_pose: ({x}, {y})')

    def run_scenario(self):
        self.step += 1

        # === Scenario A: 正常流程 goal → PLANNING → EXECUTING → COMPLETE ===
        if self.step == 2:
            self.get_logger().info("=== Scenario A: normal flow ===")
            self.pub_goal(10.0, 5.0)

        elif self.step == 3:
            # 验证进入 PLANNING
            pass

        elif self.step == 4:
            self.pub_planner("SUCCESS")

        elif self.step == 5:
            self.pub_adapter("path_received", total=5)

        elif self.step == 6:
            self.pub_adapter("waypoint_reached", index=0, total=5)

        elif self.step == 7:
            self.pub_adapter("waypoint_reached", index=1, total=5)

        elif self.step == 8:
            self.pub_adapter("waypoint_reached", index=2, total=5)

        elif self.step == 9:
            self.pub_adapter("goal_reached")

        # === Scenario B: STUCK → RECOVERING → REPLANNING ===
        elif self.step == 11:
            self.get_logger().info("=== Scenario B: stuck recovery ===")
            self.pub_goal(20.0, 10.0)

        elif self.step == 12:
            self.pub_planner("SUCCESS")

        elif self.step == 13:
            self.pub_adapter("path_received", total=8)
            self.pub_adapter("waypoint_reached", index=0, total=8)

        elif self.step == 14:
            self.pub_planner("STUCK")

        # 等 recovery_timeout (8s) → 自动 REPLANNING
        # mission_arc 被动等待 adapter 处理 replan

        elif self.step == 23:
            # recovery_timeout=8s 已过, 应已进入 REPLANNING
            # 模拟 adapter 完成 replan → planner 返回 SUCCESS
            self.pub_planner("SUCCESS")

        elif self.step == 24:
            self.pub_adapter("path_received", total=6)
            self.pub_adapter("goal_reached")

        # === Scenario C: FAILED ===
        elif self.step == 26:
            self.get_logger().info("=== Scenario C: planning failure ===")
            self.pub_goal(30.0, 15.0)

        elif self.step == 28:
            self.pub_planner("FAILED")

        # === 输出结果 ===
        elif self.step == 31:
            # 提取状态转换序列
            state_seq = []
            for s in self.statuses:
                st = s.get('state', '?')
                if not state_seq or state_seq[-1] != st:
                    state_seq.append(st)

            result = {
                "total_statuses": len(self.statuses),
                "state_sequence": state_seq,
                "final_states": self.statuses[-3:] if len(self.statuses) >= 3 else self.statuses,
            }
            print("RESULT:" + json.dumps(result))
            sys.exit(0)

def main():
    rclpy.init()
    node = Harness()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
'''
        sftp = ssh.open_sftp()
        with sftp.open('/tmp/mission_arc_test_harness.py', 'w') as f:
            f.write(harness)
        sftp.chmod('/tmp/mission_arc_test_harness.py', 0o755)
        sftp.close()
        print("Harness uploaded")

        # ================================================================
        # Step 1: 启动 mission_arc
        # ================================================================
        print("\n[Step 1] Starting mission_arc node...")
        arc_cmd = (
            f"ros2 run pct_adapters mission_arc.py "
            f"--ros-args "
            f"-p max_replan_count:=3 "
            f"-p recovery_timeout_sec:=8.0 "
            f"-p planning_timeout_sec:=30.0 "
            f"-p publish_hz:=2.0 "
        )
        run_bg(ssh, arc_cmd)
        time.sleep(2)

        out, _ = run_cmd(ssh, "pgrep -f mission_arc")
        arc_running = len(out.strip()) > 0
        check("M1 mission_arc 节点启动", arc_running, f"PID: {out.strip()}")

        if not arc_running:
            out, _ = run_cmd(ssh, "cat /tmp/mission_arc_bg.log | tail -20")
            print("  Log:", out)
            return 1

        # ================================================================
        # Step 2: 运行测试 harness (30 步, 每步 1s)
        # ================================================================
        print("\n[Step 2] Running scenario harness (30s)...")
        cmd = f"timeout 35 python3 /tmp/mission_arc_test_harness.py 2>&1"
        _, out, err = ssh.exec_command(f"{SETUP} && {cmd}", timeout=45)
        output = out.read().decode()

        print("  Harness output (last 1500 chars):")
        print("  " + output[-1500:].replace("\n", "\n  "))

        # 解析结果
        result_data = None
        for line in output.split('\n'):
            if line.startswith('RESULT:'):
                result_data = json.loads(line[7:])
                break

        # 检查 mission_arc 日志
        print("\n  === mission_arc 日志 ===")
        out3, _ = run_cmd(ssh, "cat /tmp/mission_arc_bg.log 2>/dev/null | tail -25")
        print("  " + out3.strip().replace("\n", "\n  "))

        if result_data:
            seq = result_data['state_sequence']
            print(f"\n  State sequence: {' → '.join(seq)}")
            print(f"  Total statuses received: {result_data['total_statuses']}")

            # M2: goal → PLANNING
            check("M2 goal → PLANNING", 'PLANNING' in seq)

            # M3: SUCCESS → EXECUTING
            check("M3 SUCCESS → EXECUTING", 'EXECUTING' in seq)

            # M4: 进度更新 (检查 final_states 中有 progress)
            has_progress = any(
                'progress' in s for s in result_data.get('final_states', [])
            )
            # 更好的检查: 看所有状态里有没有 progress
            all_statuses_str = json.dumps(result_data)
            check("M4 waypoint 进度更新", 'waypoints_completed' in all_statuses_str or 'progress' in all_statuses_str,
                  "progress fields in status")

            # M5: goal_reached → COMPLETE
            check("M5 goal_reached → COMPLETE", 'COMPLETE' in seq)

            # M6: STUCK → RECOVERING → REPLANNING 链
            has_recovering = 'RECOVERING' in seq
            has_replanning = 'REPLANNING' in seq
            check("M6 STUCK → RECOVERING → REPLANNING", has_recovering and has_replanning,
                  f"RECOVERING={has_recovering}, REPLANNING={has_replanning}")

            # M7: planner FAILED → FAILED
            check("M7 planner FAILED → FAILED", 'FAILED' in seq)

        else:
            for name in ["M2", "M3", "M4", "M5", "M6", "M7"]:
                check(f"{name} (no data)", False, "harness produced no RESULT")

            # 看日志
            out2, _ = run_cmd(ssh, "cat /tmp/mission_arc_bg.log | tail -20")
            print("\n  mission_arc log:", out2)

    finally:
        print("\n[Cleanup]")
        kill_procs(ssh, ["mission_arc", "mission_arc_test_harness"])
        ssh.close()

    passed = sum(1 for r in results if r['pass'])
    total = len(results)
    print(f"\n{'='*50}")
    print(f"Mission Arc 集成测试: {passed}/{total} PASS")
    print(f"{'='*50}")

    return 0 if passed == total else 1


if __name__ == '__main__':
    sys.exit(main())
