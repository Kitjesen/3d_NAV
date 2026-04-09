"""Colcon install-layout path helpers for NativeModule factories.

Knows how to locate compiled binaries and share files inside a standard
colcon install tree.  Nothing here is product-specific; it only encodes
the two standard install layouts (full-colcon and flat-prefix).

Usage::

    from core.native_install import exe, share, DDS_ENV
    from core.config import get_config

    cfg = get_config()
    binary = exe(cfg, "local_planner", "localPlanner")
    config = share(cfg, "fastlio2", "config", "lio.yaml")
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .config import RobotConfig

# Default install prefix: <lingtu_repo>/install
# (the colcon build output lives next to src/ inside the repo root)
# Overridable via cfg.raw["nav_install"] or env var LINGTU_NAV_INSTALL.
_REPO_ROOT = Path(__file__).resolve().parent.parent.parent
_DEFAULT_PREFIX = os.environ.get(
    "LINGTU_NAV_INSTALL",
    str(_REPO_ROOT / "install"),
)

DDS_ENV: dict[str, str] = {
    "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp",
}


def _install_prefix(cfg: RobotConfig) -> str:
    return cfg.raw.get("nav_install", _DEFAULT_PREFIX)


def exe(cfg: RobotConfig, package: str, binary: str) -> str:
    """Return the absolute path to a compiled binary.

    Tries colcon full layout first (prefix/package/lib/package/binary),
    then falls back to flat layout (prefix/lib/package/binary).
    """
    prefix = _install_prefix(cfg)
    colcon = os.path.join(prefix, package, "lib", package, binary)
    if os.path.exists(colcon):
        return colcon
    return os.path.join(prefix, "lib", package, binary)


def share(cfg: RobotConfig, package: str, *sub: str) -> str:
    """Return the absolute path to a share file.

    Tries colcon full layout first (prefix/package/share/package/...),
    then falls back to flat layout (prefix/share/package/...).
    """
    prefix = _install_prefix(cfg)
    colcon = os.path.join(prefix, package, "share", package, *sub)
    if os.path.exists(colcon):
        return colcon
    return os.path.join(prefix, "share", package, *sub)
