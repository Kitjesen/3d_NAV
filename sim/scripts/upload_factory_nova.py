#!/usr/bin/env python3
"""
upload_factory_nova.py — 上传 factory nova 导航测试文件到 S100P (sunrise)

上传内容:
  nova_nav_bridge.py → /tmp/nova_sim/bridge/
  gen_factory_nova_map.py → /tmp/nova_sim/scripts/
  gen_factory_pcd.py → /tmp/nova_sim/scripts/   (dependency)
  test_factory_nova.sh → /tmp/

然后在 sunrise 上运行:
  1. python3 /tmp/nova_sim/scripts/gen_factory_nova_map.py
  2. bash /tmp/test_factory_nova.sh 22 15 0.0 120
"""
import paramiko
from pathlib import Path

HOST = "192.168.66.190"
USER = "sunrise"
PASS = "sunrise"

# 本地仓库根目录
REPO = Path(__file__).resolve().parent.parent.parent

FILES = [
    # (local_path, remote_path)
    (REPO / "sim/bridge/nova_nav_bridge.py",         "/tmp/nova_sim/bridge/nova_nav_bridge.py"),
    (REPO / "sim/scripts/gen_factory_nova_map.py",   "/tmp/nova_sim/scripts/gen_factory_nova_map.py"),
    (REPO / "sim/scripts/gen_factory_pcd.py",        "/tmp/nova_sim/scripts/gen_factory_pcd.py"),
    (REPO / "sim/scripts/test_factory_nova.sh",      "/tmp/test_factory_nova.sh"),
]

def upload():
    print(f"Connecting to {USER}@{HOST} ...")
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(HOST, username=USER, password=PASS, timeout=10)
    sftp = ssh.open_sftp()

    # 创建目录
    for remote_dir in ["/tmp/nova_sim/bridge", "/tmp/nova_sim/scripts", "/tmp/sim_maps"]:
        ssh.exec_command(f"mkdir -p {remote_dir}")

    # 上传文件
    for local, remote in FILES:
        if not local.exists():
            print(f"  [SKIP] {local} (not found)")
            continue
        print(f"  {local.name} → {remote}")
        sftp.put(str(local), remote)

    # 设置 shell 脚本可执行
    ssh.exec_command("chmod +x /tmp/test_factory_nova.sh")

    # 验证上传
    print("\nVerifying uploads:")
    for _, remote in FILES:
        _, stdout, _ = ssh.exec_command(f"ls -lh {remote} 2>/dev/null || echo MISSING")
        line = stdout.read().decode().strip()
        print(f"  {line}")

    sftp.close()
    ssh.close()

    print("\n✓ Upload complete!")
    print("\nNext steps on sunrise:")
    print("  1. 生成地图:")
    print("     python3 /tmp/nova_sim/scripts/gen_factory_nova_map.py")
    print()
    print("  2. 运行导航测试:")
    print("     bash /tmp/test_factory_nova.sh 22 15 0.0 120")
    print()
    print("  目标 (22,15,0): 1F 工厂内部, 穿过内墙门洞到达对角")


if __name__ == "__main__":
    upload()
