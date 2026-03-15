#!/usr/bin/env python3
"""
gen_factory_nova_map.py — 为 nova_nav_bridge.py 生成三层工厂导航地图

输出:
  {SIM_DIR}/robot/factory_nova_scene.xml  — MuJoCo 场景 (工厂几何 + robot include)
  /tmp/sim_maps/factory_nova.pcd          — 工厂点云 (无机器人)
  /tmp/sim_maps/factory_nova.pickle       — PCT tomogram (供 global_planner 使用)

用法:
  python3 gen_factory_nova_map.py
  python3 gen_factory_nova_map.py --no-tomogram   # 只生成 PCD, 跳过 tomogram
"""
import sys
import os
import re
import argparse
import subprocess
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
SIM_DIR = SCRIPT_DIR.parent
ROBOT_DIR = SIM_DIR / "robot"
SCENE_XML_PATH = ROBOT_DIR / "factory_nova_scene.xml"
MAP_DIR = Path("/tmp/sim_maps")
PCD_PATH = MAP_DIR / "factory_nova.pcd"
PICKLE_PATH = MAP_DIR / "factory_nova.pickle"

# ── 导入 gen_factory_pcd 中的工厂场景构建器 ──────────────────────
sys.path.insert(0, str(SCRIPT_DIR))
from gen_factory_pcd import (
    build_factory_xml, scan_at, build_ray_dirs,
    N_RAYS, STAIR_CX, STAIR_CY, STAIR_R
)

import numpy as np
import mujoco


# ── 生成工厂 + 机器人 场景 XML ────────────────────────────────────

def build_factory_nova_scene_xml():
    """
    从 gen_factory_pcd 的工厂几何体生成 nova_nav_bridge.py 专用场景 XML.

    结构:
      <mujoco>
        <include file="robot.xml"/>   ← 机器人 (同目录)
        <worldbody>
          ... factory geometry ...    ← 三层工厂几何体
        </worldbody>
      </mujoco>
    """
    factory_xml = build_factory_xml()

    # 提取 worldbody 内容
    wb_match = re.search(r'<worldbody>(.*?)</worldbody>', factory_xml, re.DOTALL)
    if not wb_match:
        raise RuntimeError("Cannot find <worldbody> in factory XML")
    wb_content = wb_match.group(1)

    # 修正地面 friction (匹配训练环境 friction="1 0.5 0.5")
    wb_content = wb_content.replace(
        'type="plane" size="40 40 0.1" group="1" conaffinity="1"',
        'type="plane" size="40 40 0.1" group="1" conaffinity="1" condim="3" friction="1 0.5 0.5"'
    )

    scene = f"""<mujoco model="factory_nova">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.002"/>
  <visual>
    <global offwidth="1280" offheight="960"/>
    <headlight ambient="0.4 0.4 0.4"/>
    <map znear="0.01" zfar="200"/>
  </visual>

  <!-- 机器人: 与 factory_nova_scene.xml 同目录的 robot.xml -->
  <include file="robot.xml"/>

  <worldbody>
{wb_content}
  </worldbody>
</mujoco>
"""
    return scene


# ── 扫描三层工厂点云 ─────────────────────────────────────────────

def scan_factory_pcd_cloud():
    """扫描三层工厂点云 (无机器人), 返回 (N,3) ndarray."""
    factory_xml = build_factory_xml()
    model = mujoco.MjModel.from_xml_string(factory_xml)
    data = mujoco.MjData(model)
    mujoco.mj_step(model, data)

    ray_dirs = build_ray_dirs(N_RAYS)
    geomgroup = np.zeros(6, dtype=np.uint8)
    geomgroup[1] = 1  # 只扫描环境几何体 (group=1)

    positions = []
    # 1F: z=0.7m, 2m 间隔网格
    for x in np.arange(1, 30, 2.0):
        for y in np.arange(1, 20, 2.0):
            positions.append([x, y, 0.7])
    # 2F: z=3.7m
    for x in np.arange(1, 30, 2.0):
        for y in np.arange(1, 20, 2.0):
            positions.append([x, y, 3.7])
    # 3F: z=6.7m
    for x in np.arange(1, 30, 2.0):
        for y in np.arange(1, 20, 2.0):
            positions.append([x, y, 6.7])
    # 楼梯间加密
    for z in np.arange(0.5, 6.5, 0.5):
        for angle in np.linspace(0, 2 * np.pi, 8, endpoint=False):
            x = STAIR_CX + (STAIR_R + 0.5) * np.cos(angle)
            y = STAIR_CY + (STAIR_R + 0.5) * np.sin(angle)
            positions.append([x, y, z + 0.5])

    all_pts = []
    for pos in positions:
        pts = scan_at(model, data, pos, ray_dirs, geomgroup)
        if len(pts) > 0:
            all_pts.append(pts)

    print(f"  Scanned {len(positions)} positions, {sum(len(p) for p in all_pts):,} raw pts")
    if not all_pts:
        raise RuntimeError("No points scanned!")

    cloud = np.concatenate(all_pts)

    # Bbox 裁剪
    mask = (
        (cloud[:, 0] >= -2) & (cloud[:, 0] <= 32) &
        (cloud[:, 1] >= -2) & (cloud[:, 1] <= 22) &
        (cloud[:, 2] >= -0.5) & (cloud[:, 2] <= 9.0)
    )
    cloud = cloud[mask]
    print(f"  After bbox clip: {len(cloud):,} pts")
    return cloud


def voxel_downsample_numpy(pts, voxel_size=0.1):
    """无需 open3d 的体素降采样."""
    keys = np.floor(pts / voxel_size).astype(np.int64)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return pts[idx]


def save_pcd(pts, path):
    """保存为 PCD ASCII 格式 (open3d 可读)."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)

    try:
        import open3d as o3d
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts.astype(np.float64))
        pcd = pcd.voxel_down_sample(0.1)
        o3d.io.write_point_cloud(str(path), pcd)
        n = len(pcd.points)
        arr = np.asarray(pcd.points)
    except ImportError:
        print("  [WARN] open3d not found, using numpy voxel downsample")
        arr = voxel_downsample_numpy(pts, 0.1)
        n = len(arr)
        # Write PCD ASCII
        header = (
            f"# .PCD v0.7 - Point Cloud Data file format\n"
            f"VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n"
            f"COUNT 1 1 1\nWIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
            f"POINTS {n}\nDATA ascii\n"
        )
        with open(path, 'w') as f:
            f.write(header)
            np.savetxt(f, arr, fmt='%.4f')

    print(f"  Saved: {path} ({n:,} pts)")
    print(f"  Bounds: x=[{arr[:,0].min():.1f},{arr[:,0].max():.1f}] "
          f"y=[{arr[:,1].min():.1f},{arr[:,1].max():.1f}] "
          f"z=[{arr[:,2].min():.1f},{arr[:,2].max():.1f}]")
    return arr


def build_tomogram(pcd_path, pickle_path):
    """调用 pct_planner 生成 tomogram."""
    # 优先用 numpy 1.x venv (ele_planner.so 依赖), 但 build_tomogram 本身不需要
    venv_py = "/tmp/venv_np1/bin/python3"
    python_bin = venv_py if os.path.exists(venv_py) else sys.executable

    script = f"""
import sys
sys.path.insert(0, '/home/sunrise/data/SLAM/navigation/install/pct_planner/lib/python3/dist-packages')
from pct_planner.pct_planner_core import build_tomogram_from_pcd
build_tomogram_from_pcd('{pcd_path}', '{pickle_path}', voxel=0.2, ground_h=0.0)
print('OK: tomogram saved to {pickle_path}')
"""
    result = subprocess.run(
        [python_bin, "-c", script],
        capture_output=True, text=True, timeout=180
    )
    if result.returncode != 0:
        print(f"  [WARN] tomogram build failed:\n{result.stderr[:400]}")
        print(f"\n  手动生成命令:")
        print(f"    python3 -c \"")
        print(f"      import sys; sys.path.insert(0, '/home/sunrise/data/SLAM/navigation/install/pct_planner/lib/python3/dist-packages')")
        print(f"      from pct_planner.pct_planner_core import build_tomogram_from_pcd")
        print(f"      build_tomogram_from_pcd('{pcd_path}', '{pickle_path}')\"")
        return False
    else:
        print(f"  {result.stdout.strip()}")
        return True


def main():
    parser = argparse.ArgumentParser(description='生成三层工厂 nova_nav_bridge 地图')
    parser.add_argument('--no-tomogram', action='store_true',
                        help='跳过 tomogram 生成 (只生成 scene XML + PCD)')
    args = parser.parse_args()

    print("=" * 55)
    print("  Factory Nova Map Generator (3-floor, 30×20m)")
    print("=" * 55)

    # 1. 生成场景 XML
    print("\n[1] Generating factory_nova_scene.xml ...")
    ROBOT_DIR.mkdir(parents=True, exist_ok=True)
    scene_xml = build_factory_nova_scene_xml()
    SCENE_XML_PATH.write_text(scene_xml)
    print(f"  Saved: {SCENE_XML_PATH}")

    # 2. 扫描点云
    print("\n[2] Scanning factory PCD ...")
    cloud = scan_factory_pcd_cloud()
    pts = save_pcd(cloud, PCD_PATH)

    # 3. 生成 tomogram
    if not args.no_tomogram:
        print("\n[3] Building PCT tomogram ...")
        ok = build_tomogram(str(PCD_PATH), str(PICKLE_PATH))
        if ok:
            print(f"  Tomogram: {PICKLE_PATH}")
    else:
        print("\n[3] Skipping tomogram (--no-tomogram)")

    print("\n" + "=" * 55)
    print("  Done!")
    print(f"  Scene XML:  {SCENE_XML_PATH}")
    print(f"  PCD:        {PCD_PATH}")
    if not args.no_tomogram:
        print(f"  Tomogram:   {PICKLE_PATH}")
    print()
    print("  Run navigation:")
    print(f"    python3 nova_nav_bridge.py --headless \\")
    print(f"        --scene {SCENE_XML_PATH} --start 2 2 0.35")
    print()
    print("  Or run full test:")
    print(f"    bash test_factory_nova.sh")
    print("=" * 55)


if __name__ == '__main__':
    main()
