"""MuJoCo 相机渲染 — RGB + 深度
# Inspired by DimOS dimos/simulation/engines/mujoco_engine.py

使用 mujoco.Renderer 实现离屏 RGB 和深度渲染。
"""
from typing import Optional, Tuple

import numpy as np

from simulate.core.engine import CameraData
from simulate.core.sensor import CameraConfig


class MuJoCoCamera:
    """MuJoCo 相机渲染器.

    每个相机实例对应一个 MuJoCo camera，内部维护独立的 Renderer。
    使用 mujoco.Renderer（MuJoCo 3.x 标准 API），不依赖 OpenGL 窗口。
    """

    def __init__(self, model, config: CameraConfig) -> None:
        """初始化相机渲染器.

        Args:
            model: mujoco.MjModel 实例
            config: 相机配置
        """
        import mujoco

        self._model = model
        self._config = config
        self._camera_name = config.name

        # 验证相机存在于模型中
        cam_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, config.name)
        if cam_id < 0:
            raise ValueError(
                f"[MuJoCoCamera] Camera '{config.name}' not found in MuJoCo model. "
                f"Check robot.xml camera definitions."
            )
        self._cam_id = cam_id

        # 创建 RGB 渲染器
        self._renderer_rgb = mujoco.Renderer(model, config.height, config.width)

        # 创建深度渲染器（可选）
        self._renderer_depth: Optional[object] = None
        if config.render_depth:
            self._renderer_depth = mujoco.Renderer(model, config.height, config.width)

        print(f'[MuJoCoCamera] Ready: {config.name} ({config.width}x{config.height}, '
              f'fovy={config.fovy}°, depth={config.render_depth})')

    def render(self, data) -> CameraData:
        """渲染当前帧并返回 CameraData.

        Args:
            data: mujoco.MjData 实例（当前仿真状态）

        Returns:
            CameraData with rgb (H,W,3) uint8 and depth (H,W) float32 in meters
        """
        import mujoco

        # 渲染 RGB
        self._renderer_rgb.update_scene(data, camera=self._camera_name)
        rgb = self._renderer_rgb.render().copy()  # (H, W, 3) uint8

        # 渲染深度
        depth_meters: np.ndarray
        if self._renderer_depth is not None:
            self._renderer_depth.update_scene(data, camera=self._camera_name)
            self._renderer_depth.enable_depth_rendering()
            depth_raw = self._renderer_depth.render()  # (H, W) float32, 0..1 normalized
            self._renderer_depth.disable_depth_rendering()

            # MuJoCo 深度 → 真实距离（米）
            # depth_raw = (zfar * znear) / (zfar - depth_raw * (zfar - znear))
            # 这里用 MuJoCo 标准反投影公式
            depth_meters = self._linearize_depth(
                depth_raw,
                self._config.depth_near,
                self._config.depth_far
            )
        else:
            depth_meters = np.zeros(
                (self._config.height, self._config.width), dtype=np.float32
            )

        return CameraData(
            rgb=rgb,
            depth=depth_meters,
            intrinsics=self._config.intrinsics,
            timestamp=float(data.time),
        )

    @staticmethod
    def _linearize_depth(depth_raw: np.ndarray,
                          near: float, far: float) -> np.ndarray:
        """将 MuJoCo 归一化深度缓冲区转换为真实距离（米）.

        MuJoCo 深度缓冲区存储的是 NDC 深度（投影后的 z/w），
        使用标准线性化公式还原为真实深度。

        Args:
            depth_raw: (H, W) float32, 值域 [0, 1]
            near: 近裁剪面距离 m
            far: 远裁剪面距离 m

        Returns:
            (H, W) float32 真实距离 m
        """
        # 线性深度 = near * far / (far - depth * (far - near))
        depth = near * far / (far - depth_raw * (far - near))
        # 裁剪到有效范围
        depth = np.clip(depth, near, far).astype(np.float32)
        return depth

    def get_rgb(self, data) -> np.ndarray:
        """仅渲染 RGB，不计算深度（更快）.

        Returns:
            (H, W, 3) uint8
        """
        self._renderer_rgb.update_scene(data, camera=self._camera_name)
        return self._renderer_rgb.render().copy()

    def close(self) -> None:
        """释放渲染器资源."""
        self._renderer_rgb.close()
        if self._renderer_depth is not None:
            self._renderer_depth.close()

    @property
    def config(self) -> CameraConfig:
        return self._config

    @property
    def cam_id(self) -> int:
        return self._cam_id
