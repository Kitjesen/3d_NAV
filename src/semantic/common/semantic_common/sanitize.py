"""
sanitize.py — NaN 清理 / JSON 安全序列化

用法:
    from semantic.common.semantic_common import safe_json_dumps, safe_json_loads, sanitize_position

    json_str = safe_json_dumps(data)           # NaN/Inf → 0.0, 永远合法 JSON
    data = safe_json_loads(raw_str, default={}) # 永远不抛异常
    pos = sanitize_position(obj.position)       # [NaN, 1.0, 2.0] → [0.0, 1.0, 2.0]
"""

import json
import logging
import math
from typing import Any, Dict, List, Optional, Sequence, Union

logger = logging.getLogger(__name__)


# ──────────────────────────────────────────────
#  基本值清理
# ──────────────────────────────────────────────

def sanitize_float(v: float, default: float = 0.0) -> float:
    """NaN / Inf → default。"""
    if isinstance(v, float) and (math.isnan(v) or math.isinf(v)):
        return default
    return v


def sanitize_position(
    pos: Optional[Sequence[float]],
    default: tuple = (0.0, 0.0, 0.0),
) -> List[float]:
    """校验 3D 位置，NaN/Inf 分量替换为 default 对应分量。

    Returns:
        始终返回长度 3 的 float 列表。
    """
    if pos is None or len(pos) < 3:
        return list(default)

    result = []
    for i in range(3):
        v = float(pos[i])
        if math.isfinite(v):
            result.append(v)
        else:
            result.append(float(default[i]) if i < len(default) else 0.0)
    return result


# ──────────────────────────────────────────────
#  递归字典清理
# ──────────────────────────────────────────────

def sanitize_dict(obj: Any, default: float = 0.0) -> Any:
    """递归清理字典/列表中的 NaN/Inf float 值。"""
    if isinstance(obj, float):
        return default if (math.isnan(obj) or math.isinf(obj)) else obj
    if isinstance(obj, dict):
        return {k: sanitize_dict(v, default) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [sanitize_dict(v, default) for v in obj]
    return obj


# ──────────────────────────────────────────────
#  JSON 安全序列化 / 反序列化
# ──────────────────────────────────────────────

def safe_json_dumps(
    data: Any,
    *,
    default_value: str = "{}",
    **kwargs,
) -> str:
    """安全 JSON 序列化，永远不产生非法 JSON。

    1. 首先尝试 allow_nan=False (最快路径)
    2. 失败时递归清理 NaN/Inf 后重试
    3. 仍失败返回 default_value

    Args:
        data: 要序列化的数据
        default_value: 全部失败时的返回值
        **kwargs: 传递给 json.dumps 的参数 (如 indent, ensure_ascii)
    """
    kwargs.setdefault("ensure_ascii", False)

    # Fast path: 大多数情况下没有 NaN
    try:
        return json.dumps(data, allow_nan=False, **kwargs)
    except (ValueError, TypeError):
        pass

    # Slow path: 递归清理
    try:
        cleaned = sanitize_dict(data)
        return json.dumps(cleaned, allow_nan=False, **kwargs)
    except (ValueError, TypeError) as e:
        logger.error("JSON serialization failed even after sanitize: %s", e)
        return default_value


def safe_json_loads(
    s: str,
    *,
    default: Any = None,
) -> Any:
    """安全 JSON 反序列化，永远不抛异常。

    Args:
        s: JSON 字符串
        default: 解析失败时的返回值
    """
    if not s:
        return default
    try:
        return json.loads(s)
    except (json.JSONDecodeError, TypeError, ValueError) as e:
        logger.debug("JSON parse failed: %s", e)
        return default


# ──────────────────────────────────────────────
#  JSON 安全 dump 到文件
# ──────────────────────────────────────────────

def safe_json_dump(
    data: Any,
    filepath: str,
    **kwargs,
) -> bool:
    """安全写入 JSON 文件，NaN/Inf 自动清理。

    Returns:
        True 成功, False 失败。
    """
    kwargs.setdefault("ensure_ascii", False)
    kwargs.setdefault("indent", 2)

    try:
        with open(filepath, "w", encoding="utf-8") as f:
            try:
                json.dump(data, f, allow_nan=False, **kwargs)
            except ValueError:
                # NaN detected — sanitize and retry
                f.seek(0)
                f.truncate()
                cleaned = sanitize_dict(data)
                json.dump(cleaned, f, allow_nan=False, **kwargs)
        return True
    except Exception as e:
        logger.error("Failed to write JSON to %s: %s", filepath, e)
        return False
