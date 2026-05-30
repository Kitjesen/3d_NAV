"""Map-name and save-time filtering helpers for gateway map routes."""

from __future__ import annotations

import re

from core.dynamic_filter import apply_dynamic_filter_step1half


def safe_map_name(name: str) -> str | None:
    """Validate a map name from user input. Return an error message or None."""
    if not name or not isinstance(name, str):
        return "empty name"
    if len(name) > 100:
        return "name too long (max 100)"
    if "/" in name or "\\" in name or ".." in name:
        return f"unsafe characters in name: {name!r}"
    if name.startswith(".") or name.startswith("-"):
        return f"name cannot start with . or -: {name!r}"
    if not re.fullmatch(r"[A-Za-z0-9_\-\.]+", name):
        return f"only [A-Za-z0-9_.-] allowed: {name!r}"
    return None
