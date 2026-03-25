"""
Semantic Perception API

统一的语义感知接口层
"""

from .types import (
    BBox2D,
    Position3D,
    Detection2D,
    Detection3D,
    Relation,
    Region,
    SceneGraph,
    CameraInfo,
    PerceptionConfig,
)

from .exceptions import (
    PerceptionAPIError,
    DetectorError,
    DetectorInitError,
    DetectorInferenceError,
    EncoderError,
    EncoderInitError,
    EncoderInferenceError,
    TrackerError,
    SceneGraphError,
    InvalidImageError,
    InvalidDepthError,
    InvalidCameraInfoError,
    ConfigurationError,
)

from .perception_api import PerceptionAPI
from .detector_api import DetectorAPI
from .encoder_api import EncoderAPI
from .tracker_api import TrackerAPI
from .factory import PerceptionFactory

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
