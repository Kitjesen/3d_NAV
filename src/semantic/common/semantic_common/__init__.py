"""semantic_common — re-exports from core.utils (canonical location).

All utilities have been moved to core.utils/. This package re-exports
for backward compatibility only.
"""

from core.utils.robustness import (
    async_timeout,
    gpu_safe,
    retry,
    retry_async,
)
from core.utils.validation import (
    validate_bgr,
    validate_depth,
    validate_depth_pair,
    validate_intrinsics,
    normalize_quaternion,
)
from core.utils.sanitize import (
    sanitize_float,
    sanitize_position,
    safe_json_dumps,
    safe_json_loads,
    safe_json_dump,
    sanitize_dict,
)

__all__ = [
    "async_timeout", "gpu_safe", "retry", "retry_async",
    "validate_bgr", "validate_depth", "validate_depth_pair",
    "validate_intrinsics", "normalize_quaternion",
    "sanitize_float", "sanitize_position", "safe_json_dumps",
    "safe_json_loads", "safe_json_dump", "sanitize_dict",
]
