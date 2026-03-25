#!/bin/bash
# 一键启动相机 + 感知 pipeline (Demo 用)
# 用法: bash /opt/nav/scripts/start_perception_demo.sh
set -e

echo '[1/4] Checking camera USB...'
if ! lsusb | grep -qi orbbec; then
    echo 'Camera not found, attempting USB reset...'
    sudo bash /opt/nav/scripts/reset_camera_usb.sh
fi

echo '[2/4] Starting camera driver...'
source /opt/ros/humble/setup.bash
# Kill existing camera
ps aux | grep -E 'component_container.*camera|ros2 launch orbbec' | grep -v grep | awk '{print $2}' | xargs kill -TERM 2>/dev/null || true
sleep 2

nohup ros2 launch orbbec_camera gemini_330_series.launch.py \
  enable_frame_sync:=false \
  enable_colored_point_cloud:=false \
  enable_point_cloud:=false \
  enable_accel:=false \
  enable_gyro:=false \
  enable_ir:=false \
  enable_left_ir:=false \
  enable_right_ir:=false \
  > /tmp/camera_demo.log 2>&1 &

echo '  Waiting for camera to stream...'
sleep 12

# Verify camera
for i in 1 2 3 4 5; do
    if timeout 5 ros2 topic hz /camera/color/image_raw --window 3 2>&1 | grep -q 'average rate'; then
        echo '  Camera streaming OK'
        break
    fi
    if [ $i -eq 5 ]; then
        echo '  WARN: Camera not streaming, trying USB reset...'
        sudo bash /opt/nav/scripts/reset_camera_usb.sh
        nohup ros2 launch orbbec_camera gemini_330_series.launch.py \
          enable_frame_sync:=false enable_colored_point_cloud:=false enable_point_cloud:=false \
          enable_accel:=false enable_gyro:=false enable_ir:=false \
          enable_left_ir:=false enable_right_ir:=false \
          > /tmp/camera_demo.log 2>&1 &
        sleep 12
    fi
done

echo '[3/4] Publishing body -> camera_link TF...'
ps aux | grep 'static_transform_publisher.*camera_link' | grep -v grep | awk '{print $2}' | xargs kill 2>/dev/null || true
sleep 1
ros2 run tf2_ros static_transform_publisher \
  --x 0.10 --y 0.0 --z 0.15 --qx 0.5 --qy -0.5 --qz 0.5 --qw 0.5 \
  --frame-id body --child-frame-id camera_link &
sleep 2

echo '[4/4] Starting perception node...'
ps aux | grep semantic_perception_node | grep -v grep | awk '{print $2}' | xargs kill -TERM 2>/dev/null || true
sleep 2
export PYTHONPATH="/opt/nav/install/semantic_perception/lib/python3.10/site-packages:/opt/nav/install/semantic_common/lib/python3.10/site-packages:${PYTHONPATH}"
export HF_HUB_OFFLINE=1
export TRANSFORMERS_OFFLINE=1

nohup bash /tmp/run_perception.sh > /tmp/perception_demo.log 2>&1 &

echo 'Waiting for YOLO model load...'
sleep 15

# Final status
echo ''
echo '========== PERCEPTION DEMO STATUS =========='
CAM_HZ=$(timeout 4 ros2 topic hz /camera/color/image_raw --window 2 2>&1 | grep 'average' || echo 'NOT STREAMING')
SG_HZ=$(timeout 4 ros2 topic hz /nav/semantic/scene_graph --window 2 2>&1 | grep 'average' || echo 'NOT PUBLISHING')
PERC_PID=$(ps aux | grep semantic_perception_node | grep -v grep | awk '{print $2}')
echo "Camera: $CAM_HZ"
echo "Scene Graph: $SG_HZ"
echo "Perception PID: $PERC_PID"
echo '============================================='
