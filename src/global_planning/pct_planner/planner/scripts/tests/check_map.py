#!/usr/bin/env python3
"""
测试用：从 tomogram 的 .pickle 里找一对可通行起终点，并打印 ros2 命令，便于在终端测规划。
用法: python3 check_map.py [地图名]
  地图名 默认 spiral0.3_2，对应 rsc/tomogram/<地图名>.pickle（相对本包根路径）。
"""
import pickle
import numpy as np
import sys
import os
import argparse

# 本脚本在 planner/scripts/test/，包根在 ../../..
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_PKG_ROOT = os.path.normpath(os.path.join(_SCRIPT_DIR, '..', '..', '..'))


def main():
    parser = argparse.ArgumentParser(description='从 tomogram 中选起终点并打印 ros2 命令（测试用）')
    parser.add_argument('map_name', nargs='?', default='spiral0.3_2', help='地图名（不含扩展名），默认 spiral0.3_2')
    args = parser.parse_args()
    pickle_path = os.path.join(_PKG_ROOT, 'rsc', 'tomogram', args.map_name + '.pickle')

    if not os.path.exists(pickle_path):
        print(f"错误: 未找到 {pickle_path}")
        print("用法: python3 check_map.py [地图名]  地图名对应 rsc/tomogram/<地图名>.pickle")
        sys.exit(1)

    with open(pickle_path, 'rb') as f:
        data = pickle.load(f)

    resolution = float(data['resolution'])
    center = np.asarray(data['center'], dtype=np.double)
    tomogram = np.asarray(data['data'], dtype=np.float32)

    print(f"Resolution: {resolution}, Center: {center}, Shape: {tomogram.shape}")

    map_h, map_w = tomogram.shape[2], tomogram.shape[3]
    offset = np.array([map_h / 2, map_w / 2])

    # tomogram 维度 (5, n_slice, dim_x, dim_y)，第 0 层为可穿越性
    slice_idx = min(2, tomogram.shape[1] - 1)
    trav_map = tomogram[0, slice_idx, :, :]

    valid_indices = np.argwhere(trav_map > 0.5)
    if len(valid_indices) == 0:
        valid_indices = np.argwhere(trav_map != 0)
    if len(valid_indices) == 0:
        print("未找到可通行点。")
        sys.exit(1)

    start_idx_flat = len(valid_indices) // 2
    start_yx = valid_indices[start_idx_flat]
    target_dist_px = 25
    best_end_yx = None
    min_dist_err = 99999
    start_y, start_x = start_yx

    for yx in valid_indices:
        dy, dx = yx[0] - start_y, yx[1] - start_x
        dist = np.sqrt(dy*dy + dx*dx)
        err = abs(dist - target_dist_px)
        if err < min_dist_err:
            min_dist_err = err
            best_end_yx = yx

    def get_world_pos(yx):
        y, x = yx
        wy = (y - offset[0]) * resolution + center[1]
        wx = (x - offset[1]) * resolution + center[0]
        return wx, wy

    sx, sy = get_world_pos(start_yx)
    ex, ey = get_world_pos(best_end_yx)

    print("-" * 40)
    print("起终点（世界坐标）:")
    print(f"  起点  x={sx:.2f}, y={sy:.2f}, z=0.0")
    print(f"  终点  x={ex:.2f}, y={ey:.2f}, z=0.0")
    print("-" * 40)
    print("复制以下命令测试规划（先启动 test launch + RViz）:")
    print(f"  ros2 run tf2_ros static_transform_publisher {sx:.2f} {sy:.2f} 0 0 0 0 map body")
    print(f"  ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \"{{header: {{frame_id: 'map'}}, pose: {{position: {{x: {ex:.2f}, y: {ey:.2f}, z: 0.0}}}}}}\"")


if __name__ == '__main__':
    main()
