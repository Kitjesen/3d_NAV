"""Perception stack: semantic perception + optional encoder + reconstruction."""

from __future__ import annotations

import logging

from core.blueprint import Blueprint

logger = logging.getLogger(__name__)
_NATIVE_CAMERA_DRIVERS = {"MujocoDriverModule"}  # Only MuJoCo has built-in camera


def perception(detector: str = "yoloe", encoder: str = "mobileclip", **config) -> Blueprint:
    """Visual perception: RGB-D semantic perception + optional encoder + 3D reconstruction."""
    bp = Blueprint()
    try:
        from core.service_manager import get_service_manager
        svc = get_service_manager()
        svc.ensure("camera")
    except Exception:
        pass

    drv_name = config.get("_driver_cls_name", "")
    needs_camera_bridge = bool(config.get("force_camera_bridge")) or (
        drv_name not in _NATIVE_CAMERA_DRIVERS
    )

    try:
        from drivers.thunder.camera_bridge_module import CameraBridgeModule

        if needs_camera_bridge:
            # Read camera rotation from robot_config.yaml
            cam_rotate = config.get("camera_rotate", 0)
            if cam_rotate == 0:
                try:
                    from core.config import get_config
                    cam_rotate = get_config().raw.get("camera", {}).get("rotate", 0)
                except Exception:
                    pass
            bp.add(CameraBridgeModule, rotate=int(cam_rotate))
    except ImportError:
        pass

    try:
        from semantic.perception.semantic_perception.encoder_module import EncoderModule
        from semantic.perception.semantic_perception.perception_module import PerceptionModule

        bp.add(
            PerceptionModule,
            detector_type=detector,
            confidence_threshold=config.get("confidence", 0.3),
            tracking_iou_threshold=config.get(
                "tracking_iou_threshold",
                config.get("iou_threshold", 0.3),
            ),
            detector_iou_threshold=config.get(
                "detector_iou_threshold",
                config.get("iou_threshold", 0.45),
            ),
            detector_max_detections=config.get(
                "detector_max_detections",
                config.get("max_detections", 64),
            ),
            detector_min_box_size_px=config.get(
                "detector_min_box_size_px",
                config.get("min_box_size_px", 12),
            ),
            detector_model_size=config.get("model_size", "l"),
            detector_device=config.get("device", ""),
            detector_model_path=config.get(
                "detector_model_path",
                config.get("model_path", ""),
            ),
            skip_frames=config.get("perception_skip_frames", 1),
            world=config.get("world", ""),
        )
        bp.add(EncoderModule, encoder=encoder)
    except ImportError as e:
        logger.warning("Perception modules not available: %s", e)

    try:
        from semantic.reconstruction.reconstruction_module import ReconstructionModule
        bp.add(ReconstructionModule)
    except ImportError:
        pass

    # Optional: record keyframes to disk for offline reconstruction
    # Enabled when recon_save_dir is provided in config
    recon_save_dir = config.get("recon_save_dir", "")
    if recon_save_dir:
        try:
            from semantic.reconstruction.dataset_recorder_module import (
                DatasetRecorderModule,
            )
            bp.add(
                DatasetRecorderModule,
                save_dir=recon_save_dir,
                keyframe_dist_m=float(config.get("recon_kf_dist_m", 0.15)),
                keyframe_rot_rad=float(config.get("recon_kf_rot_rad", 0.17)),
                keyframe_time_s=float(config.get("recon_kf_time_s", 1.0)),
                max_depth_m=float(config.get("recon_max_depth_m", 6.0)),
                jpeg_quality=int(config.get("recon_jpeg_quality", 90)),
                session_name=str(config.get("recon_session", "")),
            )
        except ImportError:
            pass

    # Optional: stream keyframes to a remote reconstruction server
    # Enabled when recon_server_url is provided in config
    recon_server_url = config.get("recon_server_url", "")
    if recon_server_url:
        try:
            from semantic.reconstruction.keyframe_exporter_module import (
                ReconKeyframeExporterModule,
            )
            bp.add(
                ReconKeyframeExporterModule,
                server_url=recon_server_url,
                keyframe_dist_m=float(config.get("recon_kf_dist_m", 0.3)),
                keyframe_rot_rad=float(config.get("recon_kf_rot_rad", 0.26)),
                keyframe_time_s=float(config.get("recon_kf_time_s", 2.0)),
                jpeg_quality=int(config.get("recon_jpeg_quality", 85)),
            )
        except ImportError:
            pass

    return bp
