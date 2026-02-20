"""
2D → 3D 投影: 将像素坐标 + 深度图投影到 3D 世界坐标系。
"""

from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np


@dataclass
class CameraIntrinsics:
    """相机内参。"""
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int


@dataclass
class Detection3D:
    """3D 检测结果。"""
    position: np.ndarray    # [x, y, z] in world/odom frame
    label: str
    score: float
    bbox_2d: np.ndarray     # [x1, y1, x2, y2] in pixels
    depth: float            # 检测中心深度 (m)
    features: np.ndarray    # CLIP 特征 (可选, 用于实例匹配)


def bbox_center_depth(
    depth_image: np.ndarray,
    bbox: np.ndarray,
    depth_scale: float = 0.001,
    kernel_size: int = 5,
) -> Optional[float]:
    """
    计算 bbox 中心区域的中值深度。

    Args:
        depth_image: HxW uint16 深度图
        bbox: [x1, y1, x2, y2] 像素坐标
        depth_scale: 深度值 → 米 的缩放因子
        kernel_size: 中心采样区域半径 (像素)

    Returns:
        深度 (米), 无效则返回 None
    """
    x1, y1, x2, y2 = bbox.astype(int)
    cx = (x1 + x2) // 2
    cy = (y1 + y2) // 2
    h, w = depth_image.shape[:2]

    # 在 bbox 中心取一个小 patch
    r = kernel_size
    y_lo = max(0, cy - r)
    y_hi = min(h, cy + r + 1)
    x_lo = max(0, cx - r)
    x_hi = min(w, cx + r + 1)

    patch = depth_image[y_lo:y_hi, x_lo:x_hi].astype(np.float64) * depth_scale
    valid = patch[patch > 0]

    if len(valid) == 0:
        return None

    return float(np.median(valid))


def project_to_3d(
    pixel_u: float,
    pixel_v: float,
    depth_m: float,
    intrinsics: CameraIntrinsics,
) -> np.ndarray:
    """
    单点 2D → 3D 投影 (相机坐标系)。

    Args:
        pixel_u, pixel_v: 像素坐标
        depth_m: 深度 (米)
        intrinsics: 相机内参

    Returns:
        [x, y, z] 相机坐标系
    """
    x = (pixel_u - intrinsics.cx) * depth_m / intrinsics.fx
    y = (pixel_v - intrinsics.cy) * depth_m / intrinsics.fy
    z = depth_m
    return np.array([x, y, z])


def transform_point(
    point_camera: np.ndarray,
    tf_camera_to_world: np.ndarray,
) -> np.ndarray:
    """
    将相机坐标系的点变换到世界坐标系。

    Args:
        point_camera: [x, y, z] 相机坐标系
        tf_camera_to_world: 4x4 变换矩阵 (camera → world)

    Returns:
        [x, y, z] 世界坐标系
    """
    p_homo = np.array([*point_camera, 1.0])
    p_world = tf_camera_to_world @ p_homo
    return p_world[:3]
