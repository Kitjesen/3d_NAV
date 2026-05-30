# Simulation Bridge Boundary

`sim/bridge/` contains legacy top-level bridge entrypoints that connect a
physics runtime to a navigation runtime.

## Files

| File | Role |
| --- | --- |
| `mujoco_ros2_bridge.py` | MuJoCo to ROS 2 topic bridge for native autonomy stack experiments. |
| `nova_nav_bridge.py` | Direct MuJoCo to Python LingTu module bridge for fast local simulation. |
| `mujoco_viz_bridge.py` | MuJoCo visualization bridge; not a navigation control path. |

## Contract

- Keep these paths stable while existing scripts still launch them directly.
- New reusable bridge logic should move into `sim/engine/` or module adapters;
  this folder should stay as thin entrypoints.
- Simulation bridge commands must not start robot services or connect to a
  physical driver.
- Robot model defaults must use `sim/robots/` or `sim/assets/`; do not re-add
  deleted `sim/robot/` paths.
