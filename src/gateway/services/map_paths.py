"""Shared map path helpers for Gateway routes and runtime state."""

from __future__ import annotations

import os
from pathlib import Path


DEFAULT_NAV_MAP_DIR = "~/data/nova/maps"


def nav_map_root() -> Path:
    """Return the configured navigation map root."""
    return Path(os.environ.get("NAV_MAP_DIR", DEFAULT_NAV_MAP_DIR)).expanduser().resolve()


def nav_map_root_str() -> str:
    return str(nav_map_root())


def active_map_link(root: Path | None = None) -> Path:
    return (root or nav_map_root()) / "active"


def active_map_name(root: Path | None = None) -> str | None:
    """Return the active map name if the active symlink targets a direct child."""
    map_root = root or nav_map_root()
    link = active_map_link(map_root)
    if not link.is_symlink():
        return None
    try:
        raw_target = os.readlink(link)
    except OSError:
        return None

    target_path = Path(raw_target)
    resolved = (
        target_path.resolve()
        if target_path.is_absolute()
        else (map_root / target_path).resolve()
    )
    try:
        rel = resolved.relative_to(map_root)
    except ValueError:
        return None
    if len(rel.parts) != 1:
        return None
    return rel.parts[0]


def map_dir_for(name: str, root: Path | None = None) -> Path:
    return (root or nav_map_root()) / name
