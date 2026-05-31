# sim/engine

`sim/engine` is the canonical simulation runtime core. Code here must stay
hardware-free: it can create synthetic worlds, MuJoCo engines, bridges, and
scenario assets, but it must not start robot services, publish to real robot
topics, or depend on field hardware being present.

## Layout

| Path | Responsibility |
| --- | --- |
| `core/` | Engine abstractions such as simulated worlds, robots, sensors, and runtime contracts. |
| `mujoco/` | MuJoCo engine adapters and kinematic/policy runtime helpers. |
| `bridge/` | Simulation bridge code for connecting synthetic runtime outputs to LingTu module ports or ROS-facing shims. |
| `scenarios/` | Deterministic scenario builders and validation assets such as multi-floor and large-terrain map products. |
| `worlds/` | Engine-local world descriptions used by scenario builders and runtime profiles. |

## Entrypoints

Use `python lingtu.py sim`, profile launchers, or the stable scripts under
`sim/scripts/`. Do not add new top-level simulation entrypoints here unless the
profile graph and tests need a new stable contract.

## Evidence Boundary

Generated reports belong under `artifacts/`, not under `sim/engine`. G4 server
closure evidence must remain simulation-only and should preserve:

- `simulation_only=true`
- `real_robot_motion=false`
- `cmd_vel_sent_to_hardware=false`

