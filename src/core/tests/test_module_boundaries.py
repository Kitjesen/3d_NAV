from __future__ import annotations

import ast
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[3]
SRC = ROOT / "src"

BOUNDARY_RULES = {
    "gateway": {"nav", "semantic", "drivers"},
    "nav": {"semantic", "drivers", "gateway"},
    "semantic": {"nav", "drivers", "gateway"},
    "drivers": {"nav", "semantic", "gateway"},
}

ALLOWED_BRIDGE_FILES = {
    "core/blueprints/full_stack.py",
    "core/blueprints/full_stack_wiring.py",
    "drivers/real/thunder/blueprints.py",
    "drivers/sim/stub.py",
}


def _python_files(package: str) -> list[Path]:
    return sorted(
        path
        for path in (SRC / package).rglob("*.py")
        if "__pycache__" not in path.parts
    )


def _imported_modules(tree: ast.AST) -> list[str]:
    modules: list[str] = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            modules.extend(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            modules.append(node.module)
    return modules


def _top_level(module: str) -> str:
    return module.split(".", 1)[0]


def _is_test_file(path: Path) -> bool:
    return (
        "tests" in path.parts
        or "test" in path.parts
        or path.name.startswith("test_")
    )


def _is_example_file(path: Path) -> bool:
    return "examples" in path.parts or "example" in path.parts


def _is_allowed_bridge(rel: str) -> bool:
    src_rel = rel.removeprefix("src/")
    return src_rel in ALLOWED_BRIDGE_FILES or src_rel.startswith("core/blueprints/stacks/")


@pytest.mark.parametrize("package,forbidden", BOUNDARY_RULES.items())
def test_package_does_not_import_forbidden_layers_directly(
    package: str,
    forbidden: set[str],
) -> None:
    violations: list[str] = []

    for path in _python_files(package):
        rel = path.relative_to(ROOT).as_posix()
        if _is_test_file(path) or _is_example_file(path) or _is_allowed_bridge(rel):
            continue
        tree = ast.parse(path.read_text(encoding="utf-8-sig"), filename=str(path))
        for module in _imported_modules(tree):
            if _top_level(module) in forbidden:
                violations.append(f"{rel}: imports {module}")

    assert violations == []
