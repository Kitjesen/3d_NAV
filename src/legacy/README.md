# Legacy

> Files live under `src/legacy/`

This package consolidates code migrated from various parts of the repository during refactoring. Legacy modules are kept for reference and backward compatibility but are **not** part of the active Module-First runtime.

## When to use legacy code

- **Historical reference**: The original implementations document design decisions and past approaches.
- **Transition period**: Active modules that still depend on legacy interfaces during migration.
- **Testing**: Some test suites reference legacy routines for regression coverage.

## When to avoid legacy code

- **New development**: New features should use the current Module-First framework (`src/core/`, registered backends).
- **Production deployment**: Legacy code is not tested against the current Blueprint lifecycle.

## Subdirectories

| Directory | Origin |
|-----------|--------|
| `thunder/` | ROS2-level legacy bridge from `drivers/real/thunder/legacy/` |
| `gateway/` | Integration test scripts from `gateway/scripts/legacy/` |
| `semantic/` | Old skill registry from `semantic/planner/legacy/` |
| `pct_planner/` | Legacy planner/tomography scripts from `pct_planner/.../legacy/` |
| `scripts/` | Shell ops scripts from top-level `scripts/legacy/` |
