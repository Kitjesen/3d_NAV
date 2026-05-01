"""Map-name and save-time filtering helpers for gateway map routes."""

from __future__ import annotations

import logging
import os
import re

logger = logging.getLogger(__name__)


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


def apply_dynamic_filter_step1half(save_dir: str | os.PathLike[str]) -> dict | None:
    """Run the optional DUFOMap filter after PGO and before tomogram build."""
    if os.environ.get("LINGTU_SAVE_DYNAMIC_FILTER", "1") in (
        "0",
        "false",
        "False",
        "FALSE",
        "no",
        "off",
        "",
    ):
        return None
    try:
        from nav.services.nav_services.dynamic_filter import refilter_map

        result = refilter_map(save_dir, timeout_s=300.0)
        if result.get("success"):
            orig = result.get("orig_count", 0)
            clean = result.get("clean_count", 0)
            dropped = result.get("dropped", 0)
            pct = 100 * dropped / max(1, orig)
            logger.info(
                "dynamic_filter: %s %d->%d pts (-%d, %.1f%%) in %.1fs",
                os.path.basename(str(save_dir)),
                orig,
                clean,
                dropped,
                pct,
                result.get("elapsed_s", 0.0),
            )
        else:
            logger.warning("dynamic_filter: skipped: %s", result.get("error"))
        return result
    except Exception as e:
        logger.warning("dynamic_filter: crashed (non-fatal): %s", e)
        return {"success": False, "error": str(e)}
