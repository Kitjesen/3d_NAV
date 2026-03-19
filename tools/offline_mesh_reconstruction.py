"""
offline_mesh_reconstruction.py — 离线 Mesh 重建（在开发机运行）

用法:
  python tools/offline_mesh_reconstruction.py maps/reconstruction/reconstruction_*.ply
  python tools/offline_mesh_reconstruction.py maps/reconstruction/reconstruction_*.ply --depth 9

依赖（开发机，非 S100P）:
  pip install open3d

流程:
  1. 读取 PLY 点云
  2. 统计离群值过滤
  3. 法向量估计
  4. Poisson 表面重建
  5. 按密度过滤低质量三角形
  6. 保存 mesh.ply
"""

import argparse
import os
import sys
import glob


def main():
    parser = argparse.ArgumentParser(description="离线 Poisson Mesh 重建")
    parser.add_argument("input", nargs="+", help="输入 PLY 文件（支持通配符）")
    parser.add_argument("--depth", type=int, default=9, help="Poisson 重建深度 (default: 9)")
    parser.add_argument("--output", default="maps/reconstruction/mesh.ply", help="输出 mesh 路径")
    parser.add_argument("--voxel", type=float, default=0.05, help="降采样体素大小 (m)")
    args = parser.parse_args()

    try:
        import open3d as o3d
    except ImportError:
        print("错误: 需要安装 open3d。运行: pip install open3d")
        sys.exit(1)

    # ── 展开输入文件 ──
    input_files = []
    for pattern in args.input:
        input_files.extend(glob.glob(pattern))
    input_files = sorted(set(input_files))

    if not input_files:
        print(f"错误: 未找到输入文件 {args.input}")
        sys.exit(1)

    print(f"输入文件: {len(input_files)} 个")

    # ── 读取并合并点云 ──
    clouds = []
    for f in input_files:
        pcd = o3d.io.read_point_cloud(f)
        print(f"  {f}: {len(pcd.points)} 点")
        clouds.append(pcd)

    if len(clouds) == 1:
        merged = clouds[0]
    else:
        merged = clouds[0]
        for c in clouds[1:]:
            merged += c

    print(f"合并后: {len(merged.points)} 点")

    # ── 体素降采样 ──
    merged = merged.voxel_down_sample(voxel_size=args.voxel)
    print(f"降采样后: {len(merged.points)} 点")

    # ── 统计离群值过滤 ──
    cl, ind = merged.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    merged = merged.select_by_index(ind)
    print(f"过滤离群值后: {len(merged.points)} 点")

    # ── 法向量估计 ──
    print("估计法向量...")
    merged.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30)
    )
    merged.orient_normals_consistent_tangent_plane(k=15)

    # ── Poisson 表面重建 ──
    print(f"Poisson 重建 (depth={args.depth})...")
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        merged, depth=args.depth
    )

    # ── 按密度过滤低质量三角形 ──
    import numpy as np
    density_threshold = np.quantile(np.asarray(densities), 0.05)
    vertices_to_remove = np.asarray(densities) < density_threshold
    mesh.remove_vertices_by_mask(vertices_to_remove)
    print(f"Mesh: {len(mesh.vertices)} 顶点, {len(mesh.triangles)} 三角形")

    # ── 保存 ──
    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    o3d.io.write_triangle_mesh(args.output, mesh)
    print(f"已保存: {args.output}")

    # ── 可视化（可选）──
    print("提示: 用 open3d.visualization.draw_geometries([mesh]) 可视化")


if __name__ == "__main__":
    main()
