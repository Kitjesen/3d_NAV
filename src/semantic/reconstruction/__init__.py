"""Semantic 3D reconstruction — RGB-D voxel coloring + semantic labeling + PLY export.

On-robot modules:
    ReconstructionModule         — streaming TSDF voxel map (lightweight, on-robot)
    ReconKeyframeExporterModule  — keyframe collector + HTTP uploader to recon server

Server-side (run separately on a GPU workstation / cloud VM):
    semantic.reconstruction.server.recon_server  — FastAPI server + backend registry
    Backends: tsdf, open3d, nerfstudio, gsfusion
"""

from .color_projector import ColorProjector
from .dataset_recorder_module import DatasetRecorderModule
from .keyframe_exporter_module import ReconKeyframeExporterModule
from .reconstruction_module import ReconstructionModule
from .semantic_labeler import SemanticLabeler

__all__ = [
    "ColorProjector",
    "DatasetRecorderModule",
    "ReconstructionModule",
    "ReconKeyframeExporterModule",
    "SemanticLabeler",
]
