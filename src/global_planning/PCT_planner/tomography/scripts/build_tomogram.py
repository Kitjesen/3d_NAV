"""
Standalone PCD -> tomogram builder (no ROS).
Used by planner_wrapper to build .pickle on-the-fly when only .pcd exists.
"""
import os
import pickle
import numpy as np
import open3d as o3d

# Run from tomography/scripts so tomogram and config can be imported
_script_dir = os.path.dirname(os.path.abspath(__file__))
if _script_dir not in __import__('sys').path:
    __import__('sys').path.insert(0, _script_dir)
if os.path.dirname(_script_dir) not in __import__('sys').path:
    __import__('sys').path.insert(0, os.path.dirname(_script_dir))

from tomogram import Tomogram


def _make_scene_cfg(resolution=0.2, slice_dh=0.5, ground_h=0.0, **trav_kw):
    """Minimal scene config for Tomogram (no ROS, no scene_*.py)."""
    class MapCfg:
        pass
    class TravCfg:
        pass
    m = MapCfg()
    m.resolution = resolution
    m.slice_dh = slice_dh
    m.ground_h = ground_h
    t = TravCfg()
    t.kernel_size = trav_kw.get('kernel_size', 7)
    t.interval_min = trav_kw.get('interval_min', 0.50)
    t.interval_free = trav_kw.get('interval_free', 0.65)
    t.slope_max = trav_kw.get('slope_max', 0.40)
    t.step_max = trav_kw.get('step_max', 0.30)
    t.standable_ratio = trav_kw.get('standable_ratio', 0.40)
    t.cost_barrier = trav_kw.get('cost_barrier', 50.0)
    t.safe_margin = trav_kw.get('safe_margin', 1.2)
    t.inflation = trav_kw.get('inflation', 0.2)
    class Cfg:
        map = m
        trav = t
    return Cfg()


def build_tomogram_from_pcd(
    pcd_path,
    output_pickle_path=None,
    resolution=0.2,
    slice_dh=0.5,
    ground_h=0.0,
    **trav_kwargs
):
    """
    Build tomogram from a .pcd file (no ROS).

    Args:
        pcd_path: Full path to .pcd file.
        output_pickle_path: If set, save data_dict to this path (e.g. .../map.pickle).
        resolution: Grid resolution (m).
        slice_dh: Height step between slices (m).
        ground_h: Ground height for bounds (m).
        **trav_kwargs: Optional traversability params (kernel_size, interval_min, ...).

    Returns:
        data_dict: Same format as planner_wrapper expects (data, resolution, center, slice_h0, slice_dh).
    """
    if not os.path.isfile(pcd_path):
        raise FileNotFoundError(f"PCD not found: {pcd_path}")

    pcd = o3d.io.read_point_cloud(pcd_path)
    points = np.asarray(pcd.points).astype(np.float32)
    if points.shape[1] > 3:
        points = points[:, :3]

    points_max = np.max(points, axis=0)
    points_min = np.min(points, axis=0)
    points_min[-1] = ground_h
    map_dim_x = int(np.ceil((points_max[0] - points_min[0]) / resolution)) + 4
    map_dim_y = int(np.ceil((points_max[1] - points_min[1]) / resolution)) + 4
    n_slice_init = int(np.ceil((points_max[2] - points_min[2]) / slice_dh))
    center = (points_max[:2] + points_min[:2]) / 2
    slice_h0 = points_min[-1] + slice_dh

    scene_cfg = _make_scene_cfg(resolution=resolution, slice_dh=slice_dh, ground_h=ground_h, **trav_kwargs)
    tomogram = Tomogram(scene_cfg)
    tomogram.initMappingEnv(center, map_dim_x, map_dim_y, n_slice_init, slice_h0)

    layers_t, trav_grad_x, trav_grad_y, layers_g, layers_c, _ = tomogram.point2map(points)
    tomogram_stack = np.stack((layers_t, trav_grad_x, trav_grad_y, layers_g, layers_c))

    data_dict = {
        'data': tomogram_stack.astype(np.float16),
        'resolution': resolution,
        'center': center,
        'slice_h0': slice_h0,
        'slice_dh': slice_dh,
    }

    if output_pickle_path:
        os.makedirs(os.path.dirname(output_pickle_path) or '.', exist_ok=True)
        with open(output_pickle_path, 'wb') as f:
            pickle.dump(data_dict, f, protocol=pickle.HIGHEST_PROTOCOL)

    return data_dict
