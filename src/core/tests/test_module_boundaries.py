from __future__ import annotations

import ast
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
SRC = ROOT / "src"


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


def test_gateway_package_does_not_import_domain_layers_directly() -> None:
    forbidden = {"nav", "semantic", "drivers"}
    violations: list[str] = []

    for path in _python_files("gateway"):
        rel = path.relative_to(ROOT).as_posix()
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        for module in _imported_modules(tree):
            if _top_level(module) in forbidden:
                violations.append(f"{rel}: imports {module}")

    assert violations == []
