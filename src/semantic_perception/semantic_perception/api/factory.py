"""
Semantic Perception API - 工厂类

用于创建各种感知组件的实例
"""

from typing import Optional

from .perception_api import PerceptionAPI
from .detector_api import DetectorAPI
from .encoder_api import EncoderAPI
from .tracker_api import TrackerAPI
from .types import PerceptionConfig
from .exceptions import ConfigurationError


class PerceptionFactory:
    """
    感知系统工厂类

    提供统一的创建接口，隐藏具体实现细节
    """

    @staticmethod
    def create_perception(
        detector_type: str = "yolo_world",
        encoder_type: str = "clip",
        tracker_type: str = "instance",
        config: Optional[PerceptionConfig] = None
    ) -> PerceptionAPI:
        """
        创建完整的感知系统

        Args:
            detector_type: 检测器类型 (yolo_world | grounding_dino)
            encoder_type: 编码器类型 (clip | blip)
            tracker_type: 追踪器类型 (instance)
            config: 配置对象

        Returns:
            PerceptionAPI实例

        Raises:
            ConfigurationError: 配置错误

        Example:
            >>> from semantic_perception.api import PerceptionFactory
            >>> perception = PerceptionFactory.create_perception(
            ...     detector_type="yolo_world",
            ...     encoder_type="clip",
            ...     config=config
            ... )
            >>> detections = perception.process_frame(rgb, depth, camera_info)
        """
        try:
            # 创建各个组件
            detector = PerceptionFactory.create_detector(detector_type, config)
            encoder = PerceptionFactory.create_encoder(encoder_type, config)
            tracker = PerceptionFactory.create_tracker(tracker_type, config)

            # 创建感知系统
            from ..impl.perception_impl import PerceptionImpl
            return PerceptionImpl(detector, encoder, tracker, config)

        except Exception as e:
            raise ConfigurationError(f"Failed to create perception system: {e}")

    @staticmethod
    def create_detector(
        detector_type: str,
        config: Optional[PerceptionConfig] = None
    ) -> DetectorAPI:
        """
        创建检测器

        Args:
            detector_type: 检测器类型
                - "yolo_world": YOLO-World开放词汇检测器
                - "grounding_dino": Grounding DINO检测器
            config: 配置对象

        Returns:
            DetectorAPI实例

        Raises:
            ConfigurationError: 不支持的检测器类型

        Example:
            >>> detector = PerceptionFactory.create_detector("yolo_world")
            >>> detector.set_classes(["chair", "table", "person"])
            >>> detections = detector.detect(image)
        """
        if detector_type == "yolo_world":
            from ..impl.yolo_world_detector import YOLOWorldDetector
            return YOLOWorldDetector(config)
        elif detector_type == "grounding_dino":
            # TODO: 实现GroundingDINO检测器
            raise ConfigurationError(
                "GroundingDINO detector not implemented yet. "
                "Use 'yolo_world' instead."
            )
        else:
            raise ConfigurationError(
                f"Unknown detector type: {detector_type}. "
                f"Supported types: yolo_world, grounding_dino"
            )

    @staticmethod
    def create_encoder(
        encoder_type: str,
        config: Optional[PerceptionConfig] = None
    ) -> EncoderAPI:
        """
        创建编码器

        Args:
            encoder_type: 编码器类型
                - "clip": CLIP视觉-语言编码器
                - "blip": BLIP编码器
            config: 配置对象

        Returns:
            EncoderAPI实例

        Raises:
            ConfigurationError: 不支持的编码器类型

        Example:
            >>> encoder = PerceptionFactory.create_encoder("clip")
            >>> image_feat = encoder.encode_image(image)
            >>> text_feat = encoder.encode_text("a red chair")
            >>> similarity = encoder.compute_similarity(image_feat, text_feat)
        """
        if encoder_type == "clip":
            from ..impl.clip_encoder import CLIPEncoder
            return CLIPEncoder(config)
        elif encoder_type == "blip":
            # TODO: 实现BLIP编码器
            raise ConfigurationError(
                "BLIP encoder not implemented yet. "
                "Use 'clip' instead."
            )
        else:
            raise ConfigurationError(
                f"Unknown encoder type: {encoder_type}. "
                f"Supported types: clip, blip"
            )

    @staticmethod
    def create_tracker(
        tracker_type: str,
        config: Optional[PerceptionConfig] = None
    ) -> TrackerAPI:
        """
        创建追踪器

        Args:
            tracker_type: 追踪器类型
                - "instance": 实例追踪器
            config: 配置对象

        Returns:
            TrackerAPI实例

        Raises:
            ConfigurationError: 不支持的追踪器类型

        Example:
            >>> tracker = PerceptionFactory.create_tracker("instance")
            >>> tracked = tracker.update(detections_3d)
        """
        if tracker_type == "instance":
            from ..impl.instance_tracker import InstanceTracker
            return InstanceTracker(config)
        else:
            raise ConfigurationError(
                f"Unknown tracker type: {tracker_type}. "
                f"Supported types: instance"
            )

    @staticmethod
    def create_from_config(config: PerceptionConfig) -> PerceptionAPI:
        """
        从配置对象创建感知系统

        Args:
            config: 配置对象，包含detector_type和encoder_type

        Returns:
            PerceptionAPI实例

        Example:
            >>> config = PerceptionConfig(
            ...     detector_type="yolo_world",
            ...     encoder_type="clip",
            ...     confidence_threshold=0.3
            ... )
            >>> perception = PerceptionFactory.create_from_config(config)
        """
        return PerceptionFactory.create_perception(
            detector_type=config.detector_type,
            encoder_type=config.encoder_type,
            tracker_type="instance",
            config=config
        )

    @staticmethod
    def get_available_detectors() -> list:
        """
        获取可用的检测器类型

        Returns:
            检测器类型列表
        """
        return ["yolo_world"]  # "grounding_dino" 待实现

    @staticmethod
    def get_available_encoders() -> list:
        """
        获取可用的编码器类型

        Returns:
            编码器类型列表
        """
        return ["clip"]  # "blip" 待实现

    @staticmethod
    def get_available_trackers() -> list:
        """
        获取可用的追踪器类型

        Returns:
            追踪器类型列表
        """
        return ["instance"]
