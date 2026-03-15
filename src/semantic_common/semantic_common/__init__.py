"""
semantic_common — 语义导航共享防御基础设施

为 semantic_perception 和 semantic_planner 提供:
- robustness: 超时/重试/GPU 安全装饰器
- validation: 图像/深度/内参/四元数校验器
- sanitize: NaN 清理、JSON 安全序列化
"""

from .robustness import (
    async_timeout,
    gpu_safe,
    retry,
    retry_async,
)
from .validation import (
    validate_bgr,
    validate_depth,
    validate_depth_pair,
    validate_intrinsics,
    normalize_quaternion,
)
from .sanitize import (
    sanitize_float,
    sanitize_position,
    safe_json_dumps,
    safe_json_loads,
    sanitize_dict,
)

__version__ = "1.0.0"

__all__ = [
    # robustness
    "async_timeout",
    "gpu_safe",
    "retry",
    "retry_async",
    # validation
    "validate_bgr",
    "validate_depth",
    "validate_depth_pair",
    "validate_intrinsics",
    "normalize_quaternion",
    # sanitize
    "sanitize_float",
    "sanitize_position",
    "safe_json_dumps",
    "safe_json_loads",
    "sanitize_dict",
]
