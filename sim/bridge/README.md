# sim/bridge/ — Thin Entrypoints

> Please do not re-add new bridge implementations here. This directory holds
> only thin entrypoints for backward compatibility. All real bridge code
> lives in `src/drivers/sim/`. Files in `sim/robot/` are preserved
> separately.

## History

Bridge files were moved from `sim/bridge/` to `src/drivers/sim/` during
the Sprint-6 consolidation. The `__init__.py` here redirects old imports
(`from sim.bridge.xxx import ...`) to the canonical location.

## Moved files

| Old path | New path |
|----------|----------|
| `sim/bridge/mujoco_ros2_bridge.py` | `src/drivers/sim/mujoco_ros2_bridge.py` |
| `sim/bridge/mujoco_viz_bridge.py` | `src/drivers/sim/mujoco_viz_bridge.py` |
| `sim/bridge/nova_nav_bridge.py` | `src/drivers/sim/nova_nav_bridge.py` |

See `src/drivers/sim/README_bridge.md` for full documentation.
