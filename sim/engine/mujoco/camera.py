"""MuJoCo camera rendering — RGB + depth
# Inspired by DimOS dimos/simulation/engines/mujoco_engine.py

Implements offscreen RGB and depth rendering via mujoco.Renderer.
"""
from typing import Optional, Tuple

import numpy as np

from sim.engine.core.engine import CameraData
from sim.engine.core.sensor import CameraConfig


class MuJoCoCamera:
    """MuJoCo camera renderer.

    Each instance corresponds to one MuJoCo camera and maintains its own Renderer.
    Uses mujoco.Renderer (MuJoCo 3.x standard API), no OpenGL window required.
    """

    def __init__(self, model, config: CameraConfig) -> None:
        """Initialize camera renderer.

        Args:
            model: mujoco.MjModel instance
            config: camera configuration
        """
        import mujoco

        self._model = model
        self._config = config
        self._camera_name = config.name

        # Verify camera exists in model
        cam_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, config.name)
        if cam_id < 0:
            raise ValueError(
                f"[MuJoCoCamera] Camera '{config.name}' not found in MuJoCo model. "
                f"Check robot.xml camera definitions."
            )
        self._cam_id = cam_id

        # Create RGB renderer
        self._renderer_rgb = mujoco.Renderer(model, config.height, config.width)

        # Create depth renderer (optional)
        self._renderer_depth: Optional[object] = None
        if config.render_depth:
            self._renderer_depth = mujoco.Renderer(model, config.height, config.width)

        print(f'[MuJoCoCamera] Ready: {config.name} ({config.width}x{config.height}, '
              f'fovy={config.fovy}deg, depth={config.render_depth})')

    def render(self, data) -> CameraData:
        """Render current frame and return CameraData.

        Args:
            data: mujoco.MjData instance (current simulation state)

        Returns:
            CameraData with rgb (H,W,3) uint8 and depth (H,W) float32 in meters
        """
        import mujoco

        # Render RGB
        self._renderer_rgb.update_scene(data, camera=self._camera_name)
        rgb = self._renderer_rgb.render().copy()  # (H, W, 3) uint8

        # Render depth
        depth_meters: np.ndarray
        if self._renderer_depth is not None:
            self._renderer_depth.update_scene(data, camera=self._camera_name)
            self._renderer_depth.enable_depth_rendering()
            depth_raw = self._renderer_depth.render()  # (H, W) float32, 0..1 normalized
            self._renderer_depth.disable_depth_rendering()

            # MuJoCo depth -> real distance (meters)
            # depth_raw = (zfar * znear) / (zfar - depth_raw * (zfar - znear))
            # Standard MuJoCo back-projection formula
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
        """Convert MuJoCo normalized depth buffer to real distance in meters.

        MuJoCo depth buffer stores NDC depth (projected z/w).
        Uses standard linearization formula to recover true depth.

        Args:
            depth_raw: (H, W) float32, range [0, 1]
            near: near clip plane distance m
            far: far clip plane distance m

        Returns:
            (H, W) float32 real distance m
        """
        # linear depth = near * far / (far - depth * (far - near))
        depth = near * far / (far - depth_raw * (far - near))
        # Clip to valid range
        depth = np.clip(depth, near, far).astype(np.float32)
        return depth

    def get_rgb(self, data) -> np.ndarray:
        """Render RGB only, skip depth (faster).

        Returns:
            (H, W, 3) uint8
        """
        self._renderer_rgb.update_scene(data, camera=self._camera_name)
        return self._renderer_rgb.render().copy()

    def close(self) -> None:
        """Release renderer resources."""
        self._renderer_rgb.close()
        if self._renderer_depth is not None:
            self._renderer_depth.close()

    @property
    def config(self) -> CameraConfig:
        return self._config

    @property
    def cam_id(self) -> int:
        return self._cam_id
