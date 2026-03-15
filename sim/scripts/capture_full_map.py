"""
Capture FULL accumulated point cloud map using Point-LIO's built-in PCD saver.
Point-LIO's pcd_save_en=True saves the complete iVox map when node exits.

Usage: python sim/scripts/capture_full_map.py
"""
import paramiko
import sys
import time
import os

ROBOT = "192.168.66.190"
USER = "sunrise"
PASS = "sunrise"
NAV_WS = "/home/sunrise/data/SLAM/navigation"
BAG_PATH = "/home/sunrise/data/slam_datasets/legkilo_corridor"
LOCAL_OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         "..", "..", "docs", "07-testing")


def main():
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(ROBOT, username=USER, password=PASS, timeout=10)

    # Point-LIO with PCD saving enabled
    # The PCD gets saved to CWD when node exits
    pcd_remote = "/home/sunrise/data/SLAM/navigation/src/slam/pointlio/PCD/scans.pcd"

    script = f"""#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
source {NAV_WS}/install/setup.bash

pkill -f pointlio_node 2>/dev/null || true
pkill -f lio_node 2>/dev/null || true
sleep 1

# Create output dir (PCD/ subdir is where Point-LIO saves scans.pcd)
mkdir -p /tmp/pointlio_map/PCD
cd /tmp/pointlio_map

# Generate config with PCD saving and VLP-16 settings
BASE_CONFIG="{NAV_WS}/install/pointlio/share/pointlio/config/velody16.yaml"
sed -e 's/scan_line: 32/scan_line: 16/' \\
    -e 's|lid_topic:.*|lid_topic: "/velodyne_points"|' \\
    -e 's|imu_topic:.*|imu_topic: "/imu/data"|' \\
    -e 's|pcd_save_en:.*|pcd_save_en: True|' \\
    -e 's|scan_bodyframe_pub_en:.*|scan_bodyframe_pub_en: True|' \\
    "$BASE_CONFIG" > /tmp/vlp16_pcd_save.yaml

echo "CONFIG_READY"
cat /tmp/vlp16_pcd_save.yaml | grep -E "pcd_save|scan_line|lid_topic|imu_topic"

echo "STARTING_POINTLIO"
ros2 run pointlio pointlio_node --ros-args \\
  --params-file /tmp/vlp16_pcd_save.yaml \\
  -r aft_mapped_to_init:=/nav/odometry \\
  -r cloud_registered_body:=/nav/registered_cloud \\
  -r cloud_registered:=/nav/map_cloud \\
  > /tmp/pointlio_pcd.log 2>&1 &
SLAM_PID=$!
sleep 3

if ! kill -0 $SLAM_PID 2>/dev/null; then
    echo "SLAM_CRASHED"
    cat /tmp/pointlio_pcd.log | tail -20
    exit 1
fi
echo "SLAM_RUNNING (PID=$SLAM_PID)"

# Play bag at 1x speed
echo "PLAYING_BAG"
ros2 bag play {BAG_PATH} --rate 1.0 \\
  --remap /points_raw:=/velodyne_points /imu_raw:=/imu/data

echo "BAG_DONE"
sleep 3

# Send SIGINT to the actual C++ node (not the ros2 run wrapper)
echo "STOPPING_SLAM"
pkill -2 -f "pointlio_node --ros-args" 2>/dev/null || true
sleep 5
# If still alive, SIGTERM
pkill -f "pointlio_node" 2>/dev/null || true
sleep 10  # Give it time to write PCD

# Check what was saved (Point-LIO saves to CWD/PCD/scans.pcd)
echo "CHECKING_OUTPUT"
ls -lhR /tmp/pointlio_map/ 2>/dev/null

if [ -f {pcd_remote} ]; then
    SIZE=$(du -h {pcd_remote} | cut -f1)
    echo "PCD_FOUND: {pcd_remote} ($SIZE)"
else
    # Try finding any PCD file
    PCD_FILE=$(find /tmp/pointlio_map -name "*.pcd" 2>/dev/null | head -1)
    if [ -n "$PCD_FILE" ]; then
        SIZE=$(du -h "$PCD_FILE" | cut -f1)
        echo "PCD_FOUND_ALT: $PCD_FILE ($SIZE)"
        cp "$PCD_FILE" {pcd_remote}
    else
        echo "NO_PCD_FOUND"
        echo "Log tail:"
        tail -30 /tmp/pointlio_pcd.log
    fi
fi

echo "DONE"
"""

    script_path = "/tmp/capture_full_map.sh"
    sftp = client.open_sftp()
    with sftp.file(script_path, 'w') as f:
        f.write(script)
    sftp.close()

    print("Starting Point-LIO with PCD saving on corridor dataset...")
    print("This will take ~8 minutes (460s bag at 1x speed)")

    channel = client.get_transport().open_session()
    channel.exec_command(f"bash {script_path}")
    channel.settimeout(660)

    buf = ""
    start = time.time()
    while time.time() - start < 620:
        if channel.recv_ready():
            data = channel.recv(4096).decode('utf-8', errors='replace')
            buf += data
            while '\n' in buf:
                line, buf = buf.split('\n', 1)
                line = line.strip()
                if line:
                    print(f"  {line}")
                if line == "DONE":
                    break
            if line == "DONE":
                break
        elif channel.exit_status_ready():
            # Read remaining
            while channel.recv_ready():
                data = channel.recv(4096).decode('utf-8', errors='replace')
                for l in data.strip().split('\n'):
                    if l.strip():
                        print(f"  {l.strip()}")
            break
        else:
            time.sleep(1)

    channel.close()

    # Download PCD
    print(f"\nDownloading PCD...")
    sftp = client.open_sftp()
    try:
        sftp.stat(pcd_remote)
        local_path = os.path.join(LOCAL_OUT, "map_pointlio_corridor.pcd")
        sftp.get(pcd_remote, local_path)
        size_mb = os.path.getsize(local_path) / 1024 / 1024
        print(f"Downloaded: {local_path} ({size_mb:.1f} MB)")
    except FileNotFoundError:
        print("PCD file not found on robot. Checking alternatives...")
        # Try listing what's in the directory
        stdin, stdout, stderr = client.exec_command("ls -lh /tmp/pointlio_map/ 2>&1")
        print(stdout.read().decode('utf-8', errors='replace'))
    finally:
        sftp.close()

    client.close()
    print("\nDone! Run: python docs/07-testing/render_pcd.py")


if __name__ == "__main__":
    main()
