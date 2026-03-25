"""core.utils — cross-layer utility functions (sanitize, robustness, validation)."""

from .sanitize import (
    sanitize_float,
    sanitize_position,
    sanitize_dict,
    safe_json_dumps,
    safe_json_loads,
    safe_json_dump,
)
from .robustness import (
    async_timeout,
    gpu_safe,
    retry,
    retry_async,
    _try_empty_cuda_cache,
)
from .validation import (
    validate_bgr,
    validate_depth,
    validate_depth_pair,
    validate_intrinsics,
    normalize_quaternion,
    IntrinsicsResult,
)

__all__ = [
    "sanitize_float", "sanitize_position", "sanitize_dict",
    "safe_json_dumps", "safe_json_loads", "safe_json_dump",
    "async_timeout", "gpu_safe", "retry", "retry_async", "_try_empty_cuda_cache",
    "validate_bgr", "validate_depth", "validate_depth_pair",
    "validate_intrinsics", "normalize_quaternion", "IntrinsicsResult",
]
