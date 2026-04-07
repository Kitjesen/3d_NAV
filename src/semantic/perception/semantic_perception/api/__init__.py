"""
Semantic Perception API

统一的语义感知接口层
"""

from .detector_api import DetectorAPI
from .encoder_api import EncoderAPI
from .exceptions import (
    ConfigurationError,
    DetectorError,
    DetectorInferenceError,
    DetectorInitError,
    EncoderError,
    EncoderInferenceError,
    EncoderInitError,
    InvalidCameraInfoError,
    InvalidDepthError,
    InvalidImageError,
    PerceptionAPIError,
    SceneGraphError,
    TrackerError,
)
from .factory import PerceptionFactory
from .perception_api import PerceptionAPI
from .tracker_api import TrackerAPI
from .types import (
    BBox2D,
    CameraInfo,
    Detection2D,
    Detection3D,
    PerceptionConfig,
    Position3D,
    Region,
    Relation,
    SceneGraph,
)

__all__ = [
    # Types
    "BBox2D",
    "Position3D",
    "Detection2D",
    "Detection3D",
    "Relation",
    "Region",
    "SceneGraph",
    "CameraInfo",
    "PerceptionConfig",
    # Exceptions
    "PerceptionAPIError",
    "DetectorError",
    "DetectorInitError",
    "DetectorInferenceError",
    "EncoderError",
    "EncoderInitError",
    "EncoderInferenceError",
    "TrackerError",
    "SceneGraphError",
    "InvalidImageError",
    "InvalidDepthError",
    "InvalidCameraInfoError",
    "ConfigurationError",
    # APIs
    "PerceptionAPI",
    "DetectorAPI",
    "EncoderAPI",
    "TrackerAPI",
    # Factory
    "PerceptionFactory",
]

__version__ = "1.0.0"
