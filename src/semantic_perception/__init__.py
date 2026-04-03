"""Compatibility wrapper for the top-level ``semantic_perception`` package."""

from importlib import import_module
from pathlib import Path

_IMPL_PACKAGE = "semantic.perception.semantic_perception"
_IMPL_DIR = (
    Path(__file__).resolve().parent.parent
    / "semantic"
    / "perception"
    / "semantic_perception"
)

if _IMPL_DIR.is_dir() and str(_IMPL_DIR) not in __path__:
    __path__.append(str(_IMPL_DIR))

_impl = import_module(_IMPL_PACKAGE)

for _name, _value in vars(_impl).items():
    if not _name.startswith("_"):
        globals().setdefault(_name, _value)

__all__ = getattr(_impl, "__all__", [name for name in globals() if not name.startswith("_")])
