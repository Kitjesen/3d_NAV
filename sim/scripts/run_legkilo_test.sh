#!/bin/bash
set -e

echo "========================================"
echo " Point-LIO VLP-16 LegKilo Test"
echo "========================================"
echo ""

# Step 3: Kill leftovers
echo "[Step 3] Killing leftover processes..."
pkill -f pointlio_node 2>/dev/null || true
pkill -f lio_node 2>/dev/null || true
pkill -f "ros2 bag" 2>/dev/null || true
sleep 2
echo "Done."

# Source ROS2
source /opt/ros/humble/setup.bash
source /home/sunrise/data/SLAM/navigation/install/setup.bash

echo ""
echo "[Step 2] Config check (scan_line and topics):"
grep -E "scan_line|lid_topic|imu_topic" /tmp/vlp16_legkilo.yaml
echo ""

# Step 4: Start Point-LIO
echo "[Step 4] Starting Point-LIO with VLP-16 config..."
ros2 run pointlio pointlio_node --ros-args \
    --params-file /tmp/vlp16_legkilo.yaml \
    -r cloud_registered_body:=/test/registered_cloud \
    -r cloud_registered:=/test/map_cloud \
    -r aft_mapped_to_init:=/test/odometry \
    > /tmp/slam_legkilo_test.log 2>&1 &
SLAM_PID=$!
echo "SLAM PID: $SLAM_PID"
sleep 5

if kill -0 $SLAM_PID 2>/dev/null; then
    echo "Point-LIO started OK"
else
    echo "Point-LIO FAILED to start"
    cat /tmp/slam_legkilo_test.log
    exit 1
fi

# Step 5: Play bag
echo ""
echo "[Step 5] Playing bag..."
ros2 bag play /home/sunrise/data/slam_datasets/legkilo_corridor --rate 1.0 &
PLAY_PID=$!
echo "Play PID: $PLAY_PID"
sleep 3

# Step 6: Wait for SLAM init
echo ""
echo "[Step 6] Waiting for SLAM init..."
GOT_ODOM=0
for i in $(seq 1 30); do
    ODOM=$(timeout 3 ros2 topic echo /test/odometry --once --no-arr 2>/dev/null | head -5)
    if [ -n "$ODOM" ]; then
        echo "GOT ODOMETRY at ${i}s!"
        echo "$ODOM"
        GOT_ODOM=1
        break
    fi
    echo "  waiting... ${i}/30"
    sleep 1
done

if [ $GOT_ODOM -eq 0 ]; then
    echo "WARNING: No odometry after 30s"
    echo "SLAM log:"
    cat /tmp/slam_legkilo_test.log
fi

# Step 7: Monitor for 60s
echo ""
echo " SEC | POS_X    | POS_Y    | POS_Z    | STATUS"
echo "-----|----------|----------|----------|--------"

for i in $(seq 1 12); do
    sleep 5

    if ! kill -0 $SLAM_PID 2>/dev/null; then
        echo "[CRASH] SLAM crashed at $((i*5))s!"
        break
    fi

    if ! kill -0 $PLAY_PID 2>/dev/null; then
        echo "[DONE] Bag playback finished at $((i*5))s"
        break
    fi

    ODOM_MSG=$(timeout 3 ros2 topic echo /test/odometry --once 2>/dev/null || true)
    if [ -z "$ODOM_MSG" ]; then
        printf "%4ds | ---      | ---      | ---      | NO_DATA\n" "$((i*5))"
        continue
    fi

    POS_X=$(echo "$ODOM_MSG" | grep -A 3 "position:" | grep "x:" | head -1 | awk '{print $2}')
    POS_Y=$(echo "$ODOM_MSG" | grep -A 3 "position:" | grep "y:" | head -1 | awk '{print $2}')
    POS_Z=$(echo "$ODOM_MSG" | grep -A 3 "position:" | grep "z:" | head -1 | awk '{print $2}')

    printf "%4ds | %8.3f | %8.3f | %8.3f | OK\n" "$((i*5))" "${POS_X:-0}" "${POS_Y:-0}" "${POS_Z:-0}"
done

# Step 8: Hz and log
echo ""
echo "[Step 8] Measuring Hz (5s)..."
timeout 5 ros2 topic hz /test/odometry 2>&1 | tail -3 || echo "(no hz data)"

echo ""
echo "SLAM log (last 30 lines):"
tail -30 /tmp/slam_legkilo_test.log

# Step 9: Cleanup
echo ""
echo "[Step 9] Cleanup..."
kill $SLAM_PID $PLAY_PID 2>/dev/null || true
wait $SLAM_PID $PLAY_PID 2>/dev/null || true
echo "Done. Test complete."
