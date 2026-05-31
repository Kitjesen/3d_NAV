"""NOVA quadruped dog -- gRPC bridge to brainstem CMS.

Package attributes are imported lazily so registry seeding a concrete driver
module does not also import legacy blueprint helpers or mutate ``sys.path``.
"""

from __future__ import annotations

import importlib
from typing import Any

__all__ = [
    "NovaDogConnection",
    "ThunderDriver",
    "nova_dog_basic",
    "nova_dog_nav",
    "nova_dog_semantic",
]


def __getattr__(name: str) -> Any:
    if name == "ThunderDriver":
        return importlib.import_module(f"{__name__}.han_dog_module").ThunderDriver
    if name == "NovaDogConnection":
        return importlib.import_module(f"{__name__}.connection").NovaDogConnection
    if name in {"nova_dog_basic", "nova_dog_nav", "nova_dog_semantic"}:
        return getattr(importlib.import_module(f"{__name__}.blueprints"), name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
