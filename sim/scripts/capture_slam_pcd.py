"""
Run SLAM on corridor dataset and capture the accumulated map point cloud as PCD.
Usage: python sim/scripts/capture_slam_pcd.py [pointlio|fastlio2]

Connects to robot via SSH, runs SLAM + bag, saves /nav/map_cloud to PCD,
then downloads the PCD to local machine.
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
FASTLIO2_YAML = f"{NAV_WS}/install/fastlio2/share/fastlio2/config/lio_velodyne.yaml"
LOCAL_OUT = os.path.join(os.path.dirname(__file__), "..", "..", "docs", "07-testing")

# PCD saver node — periodic save to disk (robust against kill)
PCD_SAVER_SCRIPT = r'''#!/usr/bin/env python3
"""Subscribe to /nav/map_cloud, periodically save to PCD file."""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
import struct, sys, os

class PcdSaver(Node):
    def __init__(self, output_path):
        super().__init__('pcd_saver')
        self.output_path = output_path
        self.cloud_data = None
        self.msg_count = 0
        self.saved_count = 0

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.sub = self.create_subscription(
            PointCloud2, '/nav/map_cloud', self.cb, qos)
        # Save every 10 seconds
        self.timer = self.create_timer(10.0, self.periodic_save)
        self.get_logger().info(f'Waiting for /nav/map_cloud, saving every 10s ...')

    def cb(self, msg):
        self.cloud_data = msg
        self.msg_count += 1
        if self.msg_count % 20 == 0:
            n_pts = msg.width * msg.height
            self.get_logger().info(f'Cloud #{self.msg_count}: {n_pts} pts')

    def periodic_save(self):
        if self.cloud_data is not None:
            self.save_pcd()

    def save_pcd(self):
        msg = self.cloud_data
        if msg is None:
            return
        n_pts = msg.width * msg.height
        if n_pts == 0:
            return

        field_map = {}
        for f in msg.fields:
            field_map[f.name] = (f.offset, f.datatype)
        point_step = msg.point_step
        data = bytes(msg.data)
        x_off = field_map.get('x', (0,7))[0]
        y_off = field_map.get('y', (4,7))[0]
        z_off = field_map.get('z', (8,7))[0]
        has_i = 'intensity' in field_map
        i_off = field_map['intensity'][0] if has_i else 0

        # Write to temp file, then rename (atomic)
        tmp = self.output_path + '.tmp'
        with open(tmp, 'w') as f:
            hdr_fields = "FIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1" if has_i else "FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1"
            f.write(f"# .PCD v0.7\nVERSION 0.7\n{hdr_fields}\nWIDTH {n_pts}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {n_pts}\nDATA ascii\n")
            for i in range(n_pts):
                base = i * point_step
                try:
                    x = struct.unpack_from('<f', data, base + x_off)[0]
                    y = struct.unpack_from('<f', data, base + y_off)[0]
                    z = struct.unpack_from('<f', data, base + z_off)[0]
                    if has_i:
                        iv = struct.unpack_from('<f', data, base + i_off)[0]
                        f.write(f"{x:.4f} {y:.4f} {z:.4f} {iv:.1f}\n")
                    else:
                        f.write(f"{x:.4f} {y:.4f} {z:.4f}\n")
                except:
                    pass
        os.replace(tmp, self.output_path)
        self.saved_count += 1
        self.get_logger().info(f'Saved PCD #{self.saved_count}: {n_pts} pts -> {self.output_path}')

def main():
    output_path = sys.argv[1] if len(sys.argv) > 1 else '/tmp/slam_map.pcd'
    rclpy.init()
    node = PcdSaver(output_path)
    try:
        rclpy.spin(node)
    except:
        pass
    finally:
        if node.cloud_data is not None:
            node.save_pcd()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
'''


def ssh_exec(client, cmd, timeout=600):
    stdin, stdout, stderr = client.exec_command(cmd, timeout=timeout)
    return stdout.read().decode('utf-8', errors='replace'), stderr.read().decode('utf-8', errors='replace')


def run_slam_and_capture(client, algo):
    """Run SLAM algorithm on corridor bag and capture map PCD."""
    pcd_path = f"/tmp/slam_map_{algo}.pcd"
    saver_path = "/tmp/pcd_saver.py"

    print(f"\n{'='*60}")
    print(f"  Capturing {algo} point cloud map")
    print(f"{'='*60}")

    # Upload PCD saver script
    sftp = client.open_sftp()
    with sftp.file(saver_path, 'w') as f:
        f.write(PCD_SAVER_SCRIPT)
    sftp.close()

    # Build the SLAM launch command
    if algo == "pointlio":
        setup_cmd = f"""
BASE_CONFIG="{NAV_WS}/install/pointlio/share/pointlio/config/velody16.yaml"
sed -e 's/scan_line: 32/scan_line: 16/' \\
    -e 's|lid_topic:.*|lid_topic: "/velodyne_points"|' \\
    -e 's|imu_topic:.*|imu_topic: "/imu/data"|' \\
    -e 's|scan_bodyframe_pub_en:.*|scan_bodyframe_pub_en: True|' \\
    -e 's|pcd_save_en:.*|pcd_save_en: False|' \\
    "$BASE_CONFIG" > /tmp/vlp16_capture.yaml
"""
        slam_cmd = f"""
ros2 run pointlio pointlio_node --ros-args \\
  --params-file /tmp/vlp16_capture.yaml \\
  -r aft_mapped_to_init:=/nav/odometry \\
  -r cloud_registered_body:=/nav/registered_cloud \\
  -r cloud_registered:=/nav/map_cloud \\
  > /tmp/slam_capture.log 2>&1 &
"""
    else:  # fastlio2
        setup_cmd = ""
        slam_cmd = f"""
ros2 run fastlio2 lio_node --ros-args \\
  -p config_path:={FASTLIO2_YAML} \\
  -r /Odometry:=/nav/odometry \\
  -r /cloud_registered:=/nav/registered_cloud \\
  -r /cloud_map:=/nav/map_cloud \\
  > /tmp/slam_capture.log 2>&1 &
"""

    script = f"""#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
source {NAV_WS}/install/setup.bash

pkill -f pointlio_node 2>/dev/null || true
pkill -f lio_node 2>/dev/null || true
pkill -f pcd_saver 2>/dev/null || true
sleep 1

{setup_cmd}

echo "STARTING_{algo.upper()}"
{slam_cmd}
SLAM_PID=$!
sleep 3

if ! kill -0 $SLAM_PID 2>/dev/null; then
    echo "SLAM_CRASHED"
    cat /tmp/slam_capture.log 2>/dev/null | tail -20
    exit 1
fi
echo "SLAM_RUNNING"

# Start PCD saver in background
python3 {saver_path} {pcd_path} > /tmp/pcd_saver.log 2>&1 &
SAVER_PID=$!
sleep 2

# Play bag (full speed to save time, no --clock)
echo "PLAYING_BAG"
ros2 bag play {BAG_PATH} --rate 1.0 \\
  --remap /points_raw:=/velodyne_points /imu_raw:=/imu/data
echo "BAG_DONE"

# Wait for periodic save to fire (timer=10s, so wait 15s)
echo "WAITING_FOR_SAVE"
sleep 15

# Kill saver
kill $SAVER_PID 2>/dev/null || true
sleep 2

# Check result
if [ -f {pcd_path} ]; then
    SIZE=$(du -h {pcd_path} | cut -f1)
    LINES=$(wc -l < {pcd_path})
    echo "PCD_SAVED: $SIZE, $LINES lines"
else
    echo "PCD_MISSING"
fi

# Cleanup
kill $SLAM_PID 2>/dev/null || true
pkill -f pointlio_node 2>/dev/null || true
pkill -f lio_node 2>/dev/null || true
sleep 1
echo "DONE_{algo.upper()}"
"""

    script_path = f"/tmp/capture_{algo}.sh"
    sftp = client.open_sftp()
    with sftp.file(script_path, 'w') as f:
        f.write(script)
    sftp.close()

    # Execute
    channel = client.get_transport().open_session()
    channel.exec_command(f"bash {script_path}")
    channel.settimeout(600)

    buf = ""
    start = time.time()
    while time.time() - start < 540:
        if channel.recv_ready():
            data = channel.recv(4096).decode('utf-8', errors='replace')
            buf += data
            while '\n' in buf:
                line, buf = buf.split('\n', 1)
                line = line.strip()
                if line:
                    print(f"  [{algo}] {line}")
                if "DONE_" in line or "SLAM_CRASHED" in line:
                    channel.close()
                    return pcd_path
        elif channel.exit_status_ready():
            break
        else:
            time.sleep(0.5)

    channel.close()
    return pcd_path


def download_pcd(client, remote_path, local_name):
    """Download PCD file from robot."""
    local_path = os.path.join(LOCAL_OUT, local_name)
    sftp = client.open_sftp()
    try:
        sftp.stat(remote_path)
        print(f"Downloading {remote_path} -> {local_path}")
        sftp.get(remote_path, local_path)
        size = os.path.getsize(local_path)
        print(f"  Downloaded: {size / 1024 / 1024:.1f} MB")
        return local_path
    except FileNotFoundError:
        print(f"  Remote file not found: {remote_path}")
        return None
    finally:
        sftp.close()


def main():
    algo = sys.argv[1] if len(sys.argv) > 1 else "both"

    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(ROBOT, username=USER, password=PASS)

    algos = ["pointlio", "fastlio2"] if algo == "both" else [algo]

    for a in algos:
        pcd_path = run_slam_and_capture(client, a)
        local = download_pcd(client, pcd_path, f"map_{a}_corridor.pcd")
        if local:
            print(f"  Saved: {local}")
        if len(algos) > 1 and a != algos[-1]:
            print("\n--- Waiting 5s between runs ---")
            time.sleep(5)

    client.close()
    print("\nDone! Use render_pcd.py to visualize.")


if __name__ == "__main__":
    main()
