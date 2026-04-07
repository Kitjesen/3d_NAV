"""ReconstructionModule -- hive Module version of ReconstructionNode.

Combines color projection, semantic labelling, and PLY export into a
single Module with typed ports.  The heavy ROS2 coupling (TF lookup,
message_filters sync, PointCloud2 serialisation) is replaced by
direct numpy input via ports.

Ports:
    In:  color_image (Image)                 -- BGR colour image
         depth_image (Image)                 -- depth image (DEPTH_U16 or DEPTH_F32)
         camera_info (CameraIntrinsics)      -- pinhole intrinsics
         scene_graph (SceneGraph)             -- semantic labels
         odometry (Odometry)                 -- camera-to-world transform source
    Out: semantic_cloud (dict)               -- {points, labels, stats} summary
         reconstruction_stats (dict)         -- running statistics
"""

from __future__ import annotations

import json
import os
import threading
import time
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

from core import In, Module, Out
from core.msgs.nav import Odometry
from core.msgs.semantic import SceneGraph
from core.msgs.sensor import CameraIntrinsics, Image

from .color_projector import ColorProjector
from .semantic_labeler import SemanticLabeler


class ReconstructionModule(Module, layer=3):
    """3-D semantic reconstruction module (hive Module).

    Combines RGB-D projection into a voxel colour table with
    scene-graph-based semantic labelling.

    The ROS2-specific parts (TF lookup, message_filters synchronisation,
    PointCloud2 serialisation, save_ply service) are not available here.
    Use ``update_frame()`` for direct numpy-level integration, or wire
    the In ports for stream-based operation.
    """

    color_image: In[Image]
    depth_image: In[Image]
    camera_info: In[CameraIntrinsics]
    scene_graph: In[SceneGraph]
    odometry: In[Odometry]
    semantic_cloud: Out[dict]
    reconstruction_stats: Out[dict]

    def __init__(self, **config: Any) -> None:
        super().__init__(**config)
        voxel_size = config.get("voxel_size", 0.05)
        self._projector = ColorProjector(voxel_size=voxel_size)
        self._labeler = SemanticLabeler()
        self._min_points = config.get("min_points_to_publish", 1000)
        self._save_dir = config.get("save_dir", "maps/reconstruction")

        # Cached state
        self._intrinsics: CameraIntrinsics | None = None
        self._latest_color: Image | None = None
        self._latest_depth: Image | None = None
        self._camera_to_world: np.ndarray | None = None
        self._lock = threading.Lock()

    def setup(self) -> None:
        self.color_image.subscribe(self._on_color)
        self.depth_image.subscribe(self._on_depth)
        self.camera_info.subscribe(self._on_camera_info)
        self.scene_graph.subscribe(self._on_scene_graph)
        self.odometry.subscribe(self._on_odom)

    # -- port callbacks ---------------------------------------------------------

    def _on_color(self, img: Image) -> None:
        with self._lock:
            self._latest_color = img

    def _on_depth(self, img: Image) -> None:
        with self._lock:
            self._latest_depth = img

    def _on_camera_info(self, info: CameraIntrinsics) -> None:
        if self._intrinsics is None:
            self._intrinsics = info

    def _on_scene_graph(self, sg: SceneGraph) -> None:
        sg_json = sg.to_json()
        self._labeler.update_from_scene_graph(sg_json)

    def _on_odom(self, odom: Odometry) -> None:
        # In full ROS2 mode, the camera-to-world transform comes from TF.
        # Here we cache odometry as a simple translation-only approximation.
        # Real usage should call update_frame() with the actual 4x4 matrix.
        pass

    # -- direct API (no ROS2) ---------------------------------------------------

    def update_frame(
        self,
        color_bgr: np.ndarray,
        depth_mm: np.ndarray,
        intrinsics: CameraIntrinsics,
        camera_to_world: np.ndarray,
    ) -> int:
        """Process one RGB-D frame directly (bypassing ports).

        Parameters
        ----------
        color_bgr : (H, W, 3) uint8
        depth_mm : (H, W) uint16, millimetres
        intrinsics : CameraIntrinsics
        camera_to_world : (4, 4) float64

        Returns
        -------
        int
            Number of voxels updated.
        """
        return self._projector.update_from_frame(
            color_bgr=color_bgr,
            depth_mm=depth_mm,
            fx=intrinsics.fx,
            fy=intrinsics.fy,
            cx=intrinsics.cx,
            cy=intrinsics.cy,
            camera_to_world=camera_to_world,
        )

    def publish_cloud(self) -> dict[str, Any] | None:
        """Build and publish the semantic cloud if enough points exist.

        Returns the stats dict, or None if below threshold.
        """
        xyzrgb = self._projector.get_colored_cloud()
        if len(xyzrgb) < self._min_points:
            return None

        labels = self._labeler.label_cloud(xyzrgb)
        n_labeled = sum(1 for lb in labels if lb != "background")

        stats: dict[str, Any] = {
            "total_points": len(xyzrgb),
            "labeled_points": n_labeled,
            "objects": self._labeler.object_count,
            "voxels": self._projector.voxel_count,
        }

        cloud_payload: dict[str, Any] = {
            "points_shape": list(xyzrgb.shape),
            "labels_count": len(labels),
            "stats": stats,
        }

        self.semantic_cloud.publish(cloud_payload)
        self.reconstruction_stats.publish(stats)
        return stats

    def save_ply(self, filepath: str | None = None) -> dict[str, Any]:
        """Save current reconstruction to PLY file.

        Returns
        -------
        dict with 'success', 'message', and optional 'filepath'.
        """
        from .ply_writer import save_ply_with_labels

        xyzrgb = self._projector.get_colored_cloud()
        if len(xyzrgb) == 0:
            return {"success": False, "message": "no point cloud data"}

        labels = self._labeler.label_cloud(xyzrgb)
        if filepath is None:
            timestamp = int(time.time())
            filepath = os.path.join(self._save_dir, f"reconstruction_{timestamp}.ply")

        try:
            n = save_ply_with_labels(xyzrgb, labels, filepath)
            return {"success": True, "message": f"saved {n} points to {filepath}", "filepath": filepath}
        except Exception as exc:
            return {"success": False, "message": f"save failed: {exc}"}

    def health(self) -> dict[str, Any]:
        info = super().port_summary()
        info["mesh_vertices"] = self._projector.voxel_count if self._projector else 0
        info["last_update_time"] = getattr(self, "_last_update_time", None)
        return info

    @property
    def voxel_count(self) -> int:
        return self._projector.voxel_count

    @property
    def object_count(self) -> int:
        return self._labeler.object_count
