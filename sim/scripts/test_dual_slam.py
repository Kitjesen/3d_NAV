"""
Run both Point-LIO and Fast-LIO2 on the same dataset, collect trajectories for comparison.
Usage: python sim/scripts/test_dual_slam.py [corridor|grass|slope]
"""
import paramiko
import sys
import time
import json

ROBOT = "192.168.66.190"
USER = "sunrise"
PASS = "sunrise"
NAV_WS = "/home/sunrise/data/SLAM/navigation"
DATASET_BASE = "/home/sunrise/data/slam_datasets"

DATASETS = {
    "corridor": {
        "bag": f"{DATASET_BASE}/legkilo_corridor",
        "duration": 460,
        "monitor": 240,
    },
    "grass": {
        "bag": f"{DATASET_BASE}/legkilo_grass",
        "duration": 100,
        "monitor": 35,
    },
    "slope": {
        "bag": f"{DATASET_BASE}/legkilo_slope",
        "duration": 320,
        "monitor": 170,
    },
}

# Fast-LIO2 VLP-16 config
FASTLIO2_YAML = f"{NAV_WS}/install/fastlio2/share/fastlio2/config/lio_velodyne.yaml"


def ssh_exec(client, cmd, timeout=600):
    stdin, stdout, stderr = client.exec_command(cmd, timeout=timeout)
    return stdout.read().decode(), stderr.read().decode()


def run_slam_test(client, algo, dataset_name, dataset_info):
    bag_path = dataset_info["bag"]
    bag_dur = dataset_info["duration"]
    monitor_dur = dataset_info["monitor"]

    print(f"\n{'='*60}")
    print(f"  Running {algo} on {dataset_name}")
    print(f"{'='*60}")

    if algo == "pointlio":
        # Generate temp yaml from installed config (same approach as test_slam_datasets.sh)
        setup_cmd = f"""
BASE_CONFIG="{NAV_WS}/install/pointlio/share/pointlio/config/velody16.yaml"
sed -e 's/scan_line: 32/scan_line: 16/' \\
    -e 's|lid_topic:.*|lid_topic: "/velodyne_points"|' \\
    -e 's|imu_topic:.*|imu_topic: "/imu/data"|' \\
    "$BASE_CONFIG" > /tmp/vlp16_legkilo.yaml
"""
        slam_cmd = f"""
ros2 run pointlio pointlio_node --ros-args \\
  --params-file /tmp/vlp16_legkilo.yaml \\
  -r aft_mapped_to_init:=/nav/odometry \\
  -r cloud_registered_body:=/nav/registered_cloud \\
  -r cloud_registered:=/nav/map_cloud \\
  > /tmp/slam_test.log 2>&1 &
"""
    else:  # fastlio2
        setup_cmd = ""
        slam_cmd = f"""
ros2 run fastlio2 lio_node --ros-args \\
  -p config_path:={FASTLIO2_YAML} \\
  -r /Odometry:=/nav/odometry \\
  -r /cloud_registered:=/nav/registered_cloud \\
  -r /cloud_map:=/nav/map_cloud \\
  > /tmp/slam_test.log 2>&1 &
"""

    script = f"""#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
source {NAV_WS}/install/setup.bash

pkill -f pointlio_node 2>/dev/null || true
pkill -f lio_node 2>/dev/null || true
sleep 1

{setup_cmd}

echo "START_{algo.upper()}"
{slam_cmd}
SLAM_PID=$!
sleep 3

# Verify SLAM is alive
if ! kill -0 $SLAM_PID 2>/dev/null; then
    echo "CRASH_{algo.upper()}"
    cat /tmp/slam_test.log 2>/dev/null | tail -10
    exit 1
fi

# Play bag with topic remaps
ros2 bag play {bag_path} --rate 1.0 \\
  --remap /points_raw:=/velodyne_points /imu_raw:=/imu/data &
BAG_PID=$!
sleep 5

echo "MONITORING"
ELAPSED=0
while [ $ELAPSED -lt {monitor_dur} ]; do
    DATA=$(timeout 4 ros2 topic echo /nav/odometry --once --no-arr 2>/dev/null | head -20)
    if [ -n "$DATA" ]; then
        PX=$(echo "$DATA" | grep -A3 "position:" | grep "x:" | head -1 | awk '{{print $2}}')
        PY=$(echo "$DATA" | grep -A3 "position:" | grep "y:" | head -1 | awk '{{print $2}}')
        PZ=$(echo "$DATA" | grep -A3 "position:" | grep "z:" | head -1 | awk '{{print $2}}')
        if [ -n "$PX" ] && [ -n "$PY" ] && [ -n "$PZ" ]; then
            echo "ODOM,$ELAPSED,$PX,$PY,$PZ"
        fi
    fi
    sleep 3
    ELAPSED=$((ELAPSED + 3))
done

echo "DONE_{algo.upper()}"
kill $BAG_PID 2>/dev/null || true
kill $SLAM_PID 2>/dev/null || true
pkill -f pointlio_node 2>/dev/null || true
pkill -f lio_node 2>/dev/null || true
sleep 1
"""

    script_path = f"/tmp/test_{algo}_{dataset_name}.sh"
    sftp = client.open_sftp()
    with sftp.file(script_path, 'w') as f:
        f.write(script)
    sftp.close()

    trajectory = []
    channel = client.get_transport().open_session()
    channel.exec_command(f"bash {script_path}")
    channel.settimeout(bag_dur + 120)

    buf = ""
    start_time = time.time()
    timeout_s = bag_dur + 120

    while time.time() - start_time < timeout_s:
        if channel.recv_ready():
            data = channel.recv(4096).decode('utf-8', errors='replace')
            buf += data
            while '\n' in buf:
                line, buf = buf.split('\n', 1)
                line = line.strip()
                if line.startswith("ODOM,"):
                    parts = line.split(",")
                    if len(parts) == 5:
                        try:
                            t = float(parts[1])
                            x = float(parts[2])
                            y = float(parts[3])
                            z = float(parts[4])
                            trajectory.append((t, x, y, z))
                            print(f"  [{algo}] t={t:.0f}s  x={x:.3f} y={y:.3f} z={z:.3f}")
                        except ValueError:
                            pass
                elif "DONE_" in line:
                    print(f"  [{algo}] Completed: {len(trajectory)} data points")
                    break
                elif "CRASH_" in line:
                    print(f"  [{algo}] CRASHED!")
                    break
                elif "START_" in line or "MONITORING" in line:
                    print(f"  [{algo}] {line}")
        elif channel.exit_status_ready():
            break
        else:
            time.sleep(0.5)

    channel.close()
    return trajectory


def main():
    dataset_name = sys.argv[1] if len(sys.argv) > 1 else "corridor"
    if dataset_name not in DATASETS:
        print(f"Unknown dataset: {dataset_name}. Choose: {list(DATASETS.keys())}")
        sys.exit(1)

    dataset_info = DATASETS[dataset_name]
    print(f"Dual-SLAM comparison test on: {dataset_name}")

    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(ROBOT, username=USER, password=PASS)

    out, _ = ssh_exec(client, f"ls {dataset_info['bag']}/metadata.yaml 2>/dev/null")
    if not out.strip():
        print(f"ERROR: Dataset not found at {dataset_info['bag']}")
        client.close()
        sys.exit(1)

    # Run both algorithms
    traj_pointlio = run_slam_test(client, "pointlio", dataset_name, dataset_info)
    print("\n--- Waiting 5s between algorithm runs ---")
    time.sleep(5)
    traj_fastlio2 = run_slam_test(client, "fastlio2", dataset_name, dataset_info)
    client.close()

    # Save results
    results = {
        "dataset": dataset_name,
        "pointlio": [{"t": t, "x": x, "y": y, "z": z} for t, x, y, z in traj_pointlio],
        "fastlio2": [{"t": t, "x": x, "y": y, "z": z} for t, x, y, z in traj_fastlio2],
    }
    out_path = f"docs/07-testing/dual_slam_{dataset_name}.json"
    with open(out_path, 'w') as f:
        json.dump(results, f, indent=2)
    print(f"\nResults saved to {out_path}")

    print(f"\n{'='*60}")
    print(f"  SUMMARY: {dataset_name}")
    print(f"{'='*60}")
    for algo, traj in [("Point-LIO", traj_pointlio), ("Fast-LIO2", traj_fastlio2)]:
        print(f"  {algo}: {len(traj)} data points")
        if traj:
            print(f"    last pos: ({traj[-1][1]:.2f}, {traj[-1][2]:.2f}, {traj[-1][3]:.2f})")


if __name__ == "__main__":
    main()
