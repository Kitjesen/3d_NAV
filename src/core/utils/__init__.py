"""core.utils — cross-layer utility functions (sanitize, robustness, validation)."""

from .livox_config import (
    build_mid360_config_dict,
    ensure_mid360_config_file,
)
from .robustness import (
    _try_empty_cuda_cache,
    async_timeout,
    gpu_safe,
    retry,
    retry_async,
)
from .sanitize import (
    safe_json_dump,
    safe_json_dumps,
    safe_json_loads,
    sanitize_dict,
    sanitize_float,
    sanitize_position,
)
from .validation import (
    IntrinsicsResult,
    normalize_quaternion,
    validate_bgr,
    validate_depth,
    validate_depth_pair,
    validate_intrinsics,
)

__all__ = [
    "IntrinsicsResult",
    "_try_empty_cuda_cache",
    "async_timeout",
    "build_mid360_config_dict",
    "ensure_mid360_config_file",
    "gpu_safe",
    "normalize_quaternion",
    "retry",
    "retry_async",
    "safe_json_dump",
    "safe_json_dumps",
    "safe_json_loads",
    "sanitize_dict",
    "sanitize_float",
    "sanitize_position",
    "validate_bgr",
    "validate_depth",
    "validate_depth_pair",
    "validate_intrinsics",
]
