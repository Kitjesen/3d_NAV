import paramiko, time, sys

HOST = "fe91fae6a6756695.natapp.cc"
PORT = 12346
USER = "bsrl"
PASS = "abcABC123"

CMD = """
cd /home/bsrl/hongsenpang/habitat && \
BASE=/home/bsrl/hongsenpang/habitat/navimind_eval && \
PY=/home/bsrl/hongsenpang/habitat/conda_env/bin/python && \
export PYTHONPATH=$BASE:$BASE/semantic_common:$BASE/semantic_perception_pkg:$BASE/semantic_planner_pkg && \
$PY $BASE/eval_objectnav.py --max-episodes 5 --verbose 2>&1
"""

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
print(f"Connecting...", flush=True)
ssh.connect(HOST, port=PORT, username=USER, password=PASS, timeout=30)
print("Connected. Starting 5-episode eval...", flush=True)

stdin, stdout, stderr = ssh.exec_command(CMD, timeout=600)

# Stream output line by line
for line in iter(stdout.readline, ""):
    print(line, end="", flush=True)

err = stderr.read().decode(errors="replace")
if err.strip():
    print("\n=== STDERR ===")
    print(err[-3000:])  # last 3000 chars of stderr

exit_code = stdout.channel.recv_exit_status()
print(f"\nExit code: {exit_code}")
ssh.close()
