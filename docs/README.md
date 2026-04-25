# LingTu Documentation

LingTu (灵途) is an autonomous navigation system for quadruped robots in
outdoor / off-road environments.

- **Platform**: S100P (RDK X5, Nash BPU 128 TOPS, aarch64) + ROS2 Humble + Ubuntu 22.04
- **Languages**: Python (Module-First framework + semantic) and C++ (SLAM, terrain, planner via nanobind)
- **Entry point**: `python lingtu.py [profile]` (see `lingtu.py` and `cli/`)

The full architectural rules and the agent-facing reference live in
[`AGENTS.md`](./AGENTS.md). All other docs in this tree are pointers into
that reference and into the source code.

## Doc map (kept English, in sync with code)

| Path | Purpose |
|------|---------|
| [`AGENTS.md`](./AGENTS.md) | Primary agent / engineer reference: profiles, stacks, registry backends, wires, CLI flags, REPL, MCP. |
| [`MODULE_FIRST_GUIDELINE.md`](./MODULE_FIRST_GUIDELINE.md) | The eight architectural rules. Read this before adding a new module. |
| [`QUICKSTART.md`](./QUICKSTART.md) | Install + run the supported profiles in order. |
| [`REPO_LAYOUT.md`](./REPO_LAYOUT.md) | Top-level directory map. |
| [`01-getting-started/`](./01-getting-started/) | Build dependencies (third-party C++ libs needed by SLAM and PCT). |
| [`02-architecture/`](./02-architecture/) | System overview, planner selection, GNSS fusion, ROS2 topic contract, task orchestration. |
| [`TUNING.md`](./TUNING.md) | Runtime parameter tuning. |
| [`SIM_ROADMAP.md`](./SIM_ROADMAP.md) | MuJoCo simulation roadmap. |
| [`LINGTU_HANDBOOK.md`](./LINGTU_HANDBOOK.md) | Operator handbook (legacy reference, not authoritative). |

## Quick start

```bash
# Framework tests (no ROS2, no hardware)
python -m pytest src/core/tests/ -q

# Interactive profile picker
python lingtu.py

# Specific profile (see cli/profiles_data.py)
python lingtu.py stub        # framework, no hardware
python lingtu.py dev         # semantic pipeline + mock LLM, no hardware
python lingtu.py sim         # MuJoCo full stack
python lingtu.py sim_nav     # pure-Python sim, no ROS2 / C++
python lingtu.py map         # build a map (S100P)
python lingtu.py nav         # navigate with a saved map (S100P)
python lingtu.py explore     # wavefront frontier (S100P)
python lingtu.py tare_explore  # CMU TARE explorer (S100P)
```

`lingtu --list` enumerates everything currently registered.

## Notes for editors

- All docs in this tree must be in English.
- Treat `src/core/`, `cli/profiles_data.py`, `src/core/blueprints/full_stack.py`,
  and `src/core/blueprints/stacks/*` as the source of truth — when those
  diverge from a doc, update the doc, not the code.
- ROS2 launch files for Module-First profiles do **not** exist:
  the only launch files left under `launch/profiles/` are SLAM / planner
  algorithm bridges used by `_NativeModule` subprocesses. There is no
  `navigation_run.launch.py`, `navigation_bringup.launch.py`,
  `launch/subsystems/`, or `scripts/legacy/` — anything referencing them is stale.
