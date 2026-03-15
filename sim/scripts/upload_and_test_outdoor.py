"""
Upload outdoor terrain dataset (grass/slope) to robot and run Point-LIO SLAM test.
Usage: python sim/scripts/upload_and_test_outdoor.py [grass|slope|both]
"""
import sys
import os
import time
import paramiko

ROBOT_IP = "192.168.66.190"
ROBOT_USER = "sunrise"
ROBOT_PASS = "sunrise"
LOCAL_BASE = os.path.join(os.path.dirname(__file__), "..", "datasets", "legkilo_outdoor")
REMOTE_BASE = "/home/sunrise/data/slam_datasets"

# Test script template for outdoor dataset
TEST_SCRIPT = r"""#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
source /home/sunrise/data/SLAM/navigation/install/setup.bash
NAV_DIR=/home/sunrise/data/SLAM/navigation
BAG_DIR={bag_dir}
DATASET_NAME={dataset_name}

echo "============================================================"
echo "  SLAM Outdoor Terrain Test - Point-LIO + Leg-KILO $DATASET_NAME"
echo "  Dataset: Unitree Go1 + VLP-16 ($DATASET_NAME terrain)"
echo "  Time: $(date '+%Y-%m-%d %H:%M:%S')"
echo "============================================================"

pkill -f pointlio_node 2>/dev/null || true
pkill -f lio_node 2>/dev/null || true
pkill -f "ros2 bag play" 2>/dev/null || true
pkill -f nova_nav_bridge 2>/dev/null || true
sleep 2

BASE_CONFIG="$NAV_DIR/install/pointlio/share/pointlio/config/velody16.yaml"
CONFIG_FILE="/tmp/vlp16_{dataset_name}.yaml"
sed -e 's/scan_line: 32/scan_line: 16/' \
    -e 's|lid_topic:.*|lid_topic: "/points_raw"|' \
    -e 's|imu_topic:.*|imu_topic: "/imu_raw"|' \
    "$BASE_CONFIG" > "$CONFIG_FILE"

echo "[1/3] Starting Point-LIO..."
ros2 run pointlio pointlio_node --ros-args \
    --params-file "$CONFIG_FILE" \
    -r cloud_registered_body:=/test/registered_cloud \
    -r cloud_registered:=/test/map_cloud \
    -r aft_mapped_to_init:=/test/odometry \
    > /tmp/slam_{dataset_name}_test.log 2>&1 &
SLAM_PID=$!
sleep 5

if ! kill -0 $SLAM_PID 2>/dev/null; then
    echo "[ERROR] Point-LIO failed!"
    cat /tmp/slam_{dataset_name}_test.log
    exit 1
fi
echo "  PID: $SLAM_PID"

echo "[2/3] Playing bag..."
ros2 bag play "$BAG_DIR" --rate 1.0 &
PLAY_PID=$!
sleep 3

echo "Waiting for init..."
for i in $(seq 1 30); do
    ODOM=$(timeout 3 ros2 topic echo /test/odometry --once --no-arr 2>/dev/null | head -3)
    if [ -n "$ODOM" ]; then
        echo "  Init OK (${{i}}s)"
        break
    fi
    sleep 1
done

echo ""
echo "[3/3] Monitoring..."
echo "t,x,y,z" > /tmp/slam_{dataset_name}_trajectory.csv

CRASH=false
INIT_X="" ; INIT_Y="" ; INIT_Z=""
MAX_DRIFT=0
TOTAL=0 ; OK_COUNT=0

for i in $(seq 1 200); do
    sleep 3
    if ! kill -0 $SLAM_PID 2>/dev/null; then
        CRASH=true
        echo "CRASH at $((i*3))s"
        break
    fi
    if ! kill -0 $PLAY_PID 2>/dev/null; then
        echo "BAG_DONE at $((i*3))s"
        break
    fi
    TOTAL=$((TOTAL+1))
    ODOM_MSG=$(timeout 3 ros2 topic echo /test/odometry --once 2>/dev/null || true)
    if [ -z "$ODOM_MSG" ]; then
        printf "%4ds | NO_DATA\n" "$((i*3))"
        continue
    fi
    OK_COUNT=$((OK_COUNT+1))
    PX=$(echo "$ODOM_MSG" | grep -A3 "position:" | grep "x:" | head -1 | awk '{{print $2}}')
    PY=$(echo "$ODOM_MSG" | grep -A3 "position:" | grep "y:" | head -1 | awk '{{print $2}}')
    PZ=$(echo "$ODOM_MSG" | grep -A3 "position:" | grep "z:" | head -1 | awk '{{print $2}}')
    SEC=$((i*3))
    if [ -z "$INIT_X" ]; then INIT_X="${{PX:-0}}"; INIT_Y="${{PY:-0}}"; INIT_Z="${{PZ:-0}}"; fi
    DRIFT=$(echo "$PX $PY $PZ $INIT_X $INIT_Y $INIT_Z" | awk '{{dx=$1-$4;dy=$2-$5;dz=$3-$6;printf "%.3f",sqrt(dx*dx+dy*dy+dz*dz)}}')
    MAX_DRIFT=$(echo "$DRIFT $MAX_DRIFT" | awk '{{if($1>$2) print $1; else print $2}}')
    printf "%4ds | %8.3f | %8.3f | %8.3f | drift=%sm\n" "$SEC" "${{PX:-0}}" "${{PY:-0}}" "${{PZ:-0}}" "$DRIFT"
    echo "$SEC,${{PX:-0}},${{PY:-0}},${{PZ:-0}}" >> /tmp/slam_{dataset_name}_trajectory.csv
done

echo ""
ODOM_HZ=$(timeout 6 ros2 topic hz /test/odometry 2>&1 | grep "average rate:" | tail -1 | awk '{{print $3}}' || echo "?")
echo "Hz: $ODOM_HZ"
echo "MaxDrift: $MAX_DRIFT"
echo "Samples: $OK_COUNT/$TOTAL"
echo "Crash: $CRASH"
echo ""
echo "=== TRAJECTORY ==="
cat /tmp/slam_{dataset_name}_trajectory.csv
echo ""
echo "=== LOG (last 30 lines) ==="
tail -30 /tmp/slam_{dataset_name}_test.log

kill $SLAM_PID $PLAY_PID 2>/dev/null || true
wait $SLAM_PID $PLAY_PID 2>/dev/null || true
echo "DONE"
"""


def upload_bag(sftp, local_dir, remote_dir, dataset_name):
    """Upload converted ROS2 bag directory to robot."""
    ros2_dir = os.path.join(local_dir, f"{dataset_name}_ros2")
    remote_bag = f"{remote_dir}/legkilo_{dataset_name}"

    try:
        sftp.stat(remote_bag)
        print(f"  Remote dir exists: {remote_bag}")
    except FileNotFoundError:
        sftp.mkdir(remote_bag)
        print(f"  Created: {remote_bag}")

    for fname in os.listdir(ros2_dir):
        local_file = os.path.join(ros2_dir, fname)
        remote_file = f"{remote_bag}/{fname}"
        fsize = os.path.getsize(local_file) / (1024 * 1024)
        print(f"  Uploading {fname} ({fsize:.1f} MB)...")
        sftp.put(local_file, remote_file)
        print(f"  Done: {remote_file}")

    return remote_bag


def run_test(ssh, dataset_name, bag_dir):
    """Upload and run test script."""
    script = TEST_SCRIPT.format(bag_dir=bag_dir, dataset_name=dataset_name)
    # Write script to remote
    remote_script = f"/tmp/slam_outdoor_{dataset_name}_test.sh"
    sftp = ssh.open_sftp()
    with sftp.open(remote_script, 'w') as f:
        f.write(script.replace('\r\n', '\n'))
    sftp.close()

    print(f"\n{'='*60}")
    print(f"  Running SLAM test: {dataset_name}")
    print(f"{'='*60}\n")

    chan = ssh.get_transport().open_session()
    chan.exec_command(f"bash {remote_script}")
    chan.settimeout(600)  # 10 min timeout

    output = []
    while True:
        try:
            if chan.recv_ready():
                data = chan.recv(4096).decode('utf-8', errors='replace')
                print(data, end='', flush=True)
                output.append(data)
            if chan.recv_stderr_ready():
                data = chan.recv_stderr(4096).decode('utf-8', errors='replace')
                # Skip stderr noise
            if chan.exit_status_ready():
                break
            time.sleep(0.5)
        except Exception as e:
            print(f"[WARN] {e}")
            break

    # Drain remaining output
    while chan.recv_ready():
        data = chan.recv(4096).decode('utf-8', errors='replace')
        print(data, end='', flush=True)
        output.append(data)

    exit_code = chan.recv_exit_status()
    full_output = ''.join(output)

    print(f"\nExit code: {exit_code}")
    return full_output, exit_code


def main():
    dataset = sys.argv[1] if len(sys.argv) > 1 else "grass"
    datasets_to_test = ["grass", "slope"] if dataset == "both" else [dataset]

    print(f"Connecting to robot {ROBOT_IP}...")
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(ROBOT_IP, username=ROBOT_USER, password=ROBOT_PASS)
    sftp = ssh.open_sftp()
    print("Connected.")

    # Ensure base directory exists
    try:
        sftp.stat(REMOTE_BASE)
    except FileNotFoundError:
        sftp.mkdir(REMOTE_BASE)

    results = {}
    for ds in datasets_to_test:
        local_ros2 = os.path.join(LOCAL_BASE, f"{ds}_ros2")
        if not os.path.exists(local_ros2):
            print(f"[SKIP] {ds}_ros2 not found (not converted yet)")
            continue

        print(f"\n--- Uploading {ds} dataset ---")
        bag_dir = upload_bag(sftp, LOCAL_BASE, REMOTE_BASE, ds)

        print(f"\n--- Testing {ds} ---")
        output, code = run_test(ssh, ds, bag_dir)
        results[ds] = {
            "exit_code": code,
            "crash": "CRASH" in output,
            "output": output,
        }

        # Extract trajectory
        if "=== TRAJECTORY ===" in output:
            traj_section = output.split("=== TRAJECTORY ===")[1].split("===")[0].strip()
            traj_file = os.path.join(LOCAL_BASE, f"{ds}_trajectory.csv")
            with open(traj_file, 'w') as f:
                f.write(traj_section + '\n')
            print(f"Saved trajectory: {traj_file}")

    sftp.close()
    ssh.close()

    # Summary
    print(f"\n{'='*60}")
    print("  OUTDOOR TERRAIN TEST SUMMARY")
    print(f"{'='*60}")
    for ds, r in results.items():
        status = "CRASH" if r["crash"] else ("PASS" if r["exit_code"] == 0 else "FAIL")
        print(f"  {ds:10s}: {status}")
    print()


if __name__ == "__main__":
    main()
