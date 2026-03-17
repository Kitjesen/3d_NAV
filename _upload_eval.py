import paramiko, os, sys, traceback

HOST = "fe91fae6a6756695.natapp.cc"
PORT = 12346
USER = "bsrl"
PASS = "abcABC123"
REMOTE_DIR = "/home/bsrl/hongsenpang/habitat/navimind_eval"

files = [
    r"D:\inovxio\brain\lingtu\experiments\habitat_eval\habitat_navimind_agent.py",
    r"D:\inovxio\brain\lingtu\experiments\habitat_eval\eval_objectnav.py",
    r"D:\inovxio\brain\lingtu\experiments\habitat_eval\run_ablation.sh",
]

try:
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    print(f"Connecting to {HOST}:{PORT}...", flush=True)
    ssh.connect(HOST, port=PORT, username=USER, password=PASS, timeout=30)
    print("Connected!", flush=True)
    sftp = ssh.open_sftp()
    for local in files:
        if not os.path.exists(local):
            print(f"LOCAL FILE NOT FOUND: {local}", flush=True)
            sys.exit(1)
        remote = f"{REMOTE_DIR}/{os.path.basename(local)}"
        sftp.put(local, remote)
        print(f"Uploaded: {os.path.basename(local)} -> {remote}", flush=True)
    sftp.close()
    ssh.close()
    print("Done.", flush=True)
except Exception:
    traceback.print_exc()
    sys.exit(1)
