"""Clean restart + Slow Path timing test."""
import paramiko
import json
import time
import sys

sys.stdout.reconfigure(encoding='utf-8', errors='replace')

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect('192.168.66.190', username='sunrise', password='sunrise', timeout=10)


def ssh_run(cmd, timeout=10):
    stdin, stdout, stderr = ssh.exec_command(cmd, timeout=timeout)
    try:
        return stdout.read().decode('utf-8', errors='replace').strip()
    except Exception:
        return ''


# 1. Kill old publishers
ssh_run('pkill -f demo_scene_graph_publisher 2>/dev/null')
time.sleep(1)

# 2. Restart nav-semantic (non-blocking sudo)
ssh_run('sudo systemctl restart nav-semantic &', timeout=5)
time.sleep(8)
status = ssh_run('systemctl is-active nav-semantic')
print(f'[1] nav-semantic: {status}')

# 3. Start publisher AFTER planner
ssh_run(
    'nohup bash -c "source /opt/ros/humble/setup.bash && '
    'python3 /tmp/demo_scene_graph_publisher.py --scenario factory --rate 1.0" '
    '> /tmp/sg_pub.log 2>&1 &'
)
time.sleep(8)
print('[2] Publisher started, waiting for DDS discovery...')

# 4. Verify scene graph reception
journal = ssh_run('journalctl -u nav-semantic --no-pager -n 5 2>/dev/null')
if 'stale' in journal.lower():
    print('[!] Scene graph still stale, waiting more...')
    time.sleep(10)

# 5. Clear state and start monitor
ssh_run('rm -f /tmp/sem_status.json')
ssh_run(
    'source /opt/ros/humble/setup.bash && source /opt/nav/install/setup.bash 2>/dev/null && '
    'nohup python3 /opt/nav/tools/status_monitor.py 60 > /dev/null 2>&1 &'
)
time.sleep(2)

# 6. Send complex instruction
instr = '去仓储区看看有没有异常'
print(f'[3] Sending: "{instr}"')
t0 = time.time()
ssh_run(
    'source /opt/ros/humble/setup.bash && source /opt/nav/install/setup.bash 2>/dev/null && '
    f'ros2 topic pub --once /nav/semantic/instruction std_msgs/msg/String '
    f'"{{data: \\"{instr}\\"}}" 2>/dev/null',
    timeout=15
)

# 7. Poll
for i in range(40):
    time.sleep(1)
    raw = ssh_run('cat /tmp/sem_status.json 2>/dev/null', timeout=5)
    if raw:
        try:
            s = json.loads(raw)
            elapsed = time.time() - t0
            conf = s.get('confidence', 0)
            target = s.get('target_label', '')
            state = s.get('state', '')
            if target and conf > 0 and state not in ('idle', 'failed', 'completed'):
                path = 'Fast' if conf >= 0.75 else 'Slow'
                print(f'  [{elapsed:.1f}s] {path} Path | target={target} | conf={conf:.2f} | state={state}')
                break
        except Exception:
            pass
else:
    print('  TIMEOUT after 40s')

# 8. Detailed timeline
time.sleep(2)
print()
print('=== Timeline ===')
jout = ssh_run('journalctl -u nav-semantic --no-pager -n 25 2>/dev/null')
for line in jout.split('\n'):
    lo = line.lower()
    if any(k in lo for k in ['instruction', 'goal via', 'state:', 'subgoal', 'slow', 'fast',
                               'esca', 'llm', 'resolve', 'scene graph', 'stale', 'decomp',
                               'started:', 'kimi']):
        print(line.strip())

ssh_run('pkill -f status_monitor.py 2>/dev/null')
ssh.close()
