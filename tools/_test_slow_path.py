"""Test Slow Path timing with warm scene graph."""
import paramiko
import json
import time
import sys

sys.stdout.reconfigure(encoding='utf-8', errors='replace')

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect('192.168.66.190', username='sunrise', password='sunrise', timeout=10)

# Restart scene graph publisher
ssh.exec_command('pkill -f demo_scene_graph_publisher.py 2>/dev/null')
time.sleep(1)
ssh.exec_command(
    'source /opt/ros/humble/setup.bash && source /opt/nav/install/setup.bash 2>/dev/null && '
    'nohup python3 /tmp/demo_scene_graph_publisher.py --scenario factory --rate 1.0 '
    '> /tmp/lingtu_demo/scene_graph.log 2>&1 &'
)
time.sleep(3)
print('[1] Scene graph publisher restarted')

# Wait for planner to receive scene graph
time.sleep(6)

# Clear old state
ssh.exec_command('rm -f /tmp/sem_status.json')
ssh.exec_command(
    'source /opt/ros/humble/setup.bash && source /opt/nav/install/setup.bash 2>/dev/null && '
    'nohup python3 /opt/nav/tools/status_monitor.py 60 > /dev/null 2>&1 &'
)
time.sleep(2)

# Send complex instruction
instr = '去设备间检查一下'
print(f'[2] Sending: "{instr}"')
t0 = time.time()
stdin, stdout, stderr = ssh.exec_command(
    'source /opt/ros/humble/setup.bash && source /opt/nav/install/setup.bash 2>/dev/null && '
    f'ros2 topic pub --once /nav/semantic/instruction std_msgs/msg/String '
    f'"{{data: \\"{instr}\\"}}" 2>/dev/null',
    timeout=15
)
stdout.read()

# Poll
for i in range(30):
    time.sleep(1)
    stdin, stdout, stderr = ssh.exec_command('cat /tmp/sem_status.json 2>/dev/null', timeout=5)
    raw = stdout.read().decode('utf-8', errors='replace').strip()
    if raw:
        try:
            s = json.loads(raw)
            elapsed = time.time() - t0
            conf = s.get('confidence', 0)
            target = s.get('target_label', '')
            state = s.get('state', '')
            if target and conf > 0 and state not in ('idle', 'failed'):
                path = 'Fast' if conf >= 0.75 else 'Slow'
                print(f'  [{elapsed:.1f}s] {path} Path | target={target} | conf={conf:.2f} | state={state}')
                break
        except Exception:
            pass
else:
    print(f'  TIMEOUT after 30s')

# Journal
print()
print('=== Journal ===')
stdin, stdout, stderr = ssh.exec_command(
    'journalctl -u nav-semantic --no-pager -n 20 2>/dev/null', timeout=10
)
journal = stdout.read().decode('utf-8', errors='replace').strip()
for line in journal.split('\n'):
    low = line.lower()
    if any(k in low for k in ['instruction', 'goal via', 'stale', 'state:', 'subgoal',
                                'slow', 'fast', 'esca', 'llm', 'resolve', 'scene graph']):
        print(line.strip())

ssh.exec_command('pkill -f status_monitor.py 2>/dev/null')
ssh.close()
