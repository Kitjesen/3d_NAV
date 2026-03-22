"""Test Slow Path with instruction that has NO scene graph keyword match."""
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


# Kill old monitors, clear state
ssh_run('pkill -f status_monitor.py 2>/dev/null')
ssh_run('rm -f /tmp/sem_status.json')
time.sleep(2)

# Start fresh monitor
ssh_run(
    'source /opt/ros/humble/setup.bash && source /opt/nav/install/setup.bash 2>/dev/null && '
    'nohup python3 /opt/nav/tools/status_monitor.py 120 > /dev/null 2>&1 &'
)
time.sleep(2)

# Verify clean state
raw = ssh_run('cat /tmp/sem_status.json 2>/dev/null')
print('[0] Clean state:', repr(raw[:50]) if raw else 'empty (good)')

# Send instruction with NO keyword match in scene graph
# Scene graph labels: 大门,门,走廊,桌子,椅子,电脑,办公室,灭火器,传送带,工厂车间,
#                     控制台,控制室,楼梯,楼梯间,货架,仓储区,设备间,消防栓
# "医务室" has zero overlap with any label → should force Slow Path
instr = '找到医务室'
print(f'[1] Sending: "{instr}" (no match in scene graph)')
t0 = time.time()
ssh_run(
    'source /opt/ros/humble/setup.bash && source /opt/nav/install/setup.bash 2>/dev/null && '
    f'ros2 topic pub --once /nav/semantic/instruction std_msgs/msg/String '
    f'"{{data: \\\\"{instr}\\\\"}}" 2>/dev/null',
    timeout=15
)

prev_state = None
for i in range(90):
    time.sleep(1)
    raw = ssh_run('cat /tmp/sem_status.json 2>/dev/null', timeout=5)
    if raw:
        try:
            s = json.loads(raw)
            elapsed = time.time() - t0
            conf = s.get('confidence', 0)
            target = s.get('target_label', '')
            state = s.get('state', '')

            if state != prev_state:
                path = 'Fast' if conf >= 0.75 else 'Slow'
                print(f'  [{elapsed:.1f}s] state={state} | target={target} | conf={conf:.2f} | path={path}')
                prev_state = state

            if state in ('navigating', 'exploring', 'failed', 'completed'):
                path = 'Fast' if conf >= 0.75 else 'Slow'
                print(f'  RESOLVED [{elapsed:.1f}s] {path} Path | target={target} | conf={conf:.2f}')
                break
        except Exception:
            pass
    if i % 15 == 14:
        print(f'  ... waiting ({i+1}s)')
else:
    elapsed = time.time() - t0
    print(f'  TIMEOUT after {elapsed:.0f}s')

# Journal
time.sleep(1)
print()
print('=== Journal Timeline ===')
jout = ssh_run('journalctl -u nav-semantic --no-pager -n 40 --since "2 min ago" 2>/dev/null')
for line in jout.split('\n'):
    lo = line.lower()
    if any(k in lo for k in ['instruction', 'goal via', 'state:', 'slow', 'fast',
                               'esca', 'llm', 'resolve', 'entropy', 'kimi',
                               'decomp', 'confidence', 'candidates',
                               'implicit', 'explore', 'timeout', 'fail', 'error']):
        print(line.strip())

ssh_run('pkill -f status_monitor.py 2>/dev/null')
ssh.close()
