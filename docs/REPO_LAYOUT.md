# LingTu Repository Layout

```
brain/lingtu/
├── lingtu.py              # Primary Python entry — sets project root, runs cli.main
├── main_nav.py            # Backwards-compatible alias (same as lingtu.py)
├── lingtu_cli.py          # Target of the pip console script `lingtu`
├── cli/                   # CLI implementation (argparse, profiles, REPL, daemon)
│   ├── bootstrap.py       # init() — extends sys.path to src/ and semantic subpkgs
│   ├── main.py            # main() — argparse, profile resolution, blueprint build
│   ├── profiles_data.py   # ROBOT_PRESETS, PROFILES, _ACTIVE_TOMOGRAM
│   ├── repl.py            # LingTuREPL (cmd.Cmd subclass)
│   ├── run_state.py       # .lingtu/run.{pid,json} for daemon lifecycle
│   ├── runtime_extra.py   # preflight, kill_residual_ports, daemonize, health_check
│   └── ...                # logging_util, paths, term, ui
│
├── src/                   # Python packages + ROS2 packages (colcon)
│   ├── core/              # Module, Blueprint, Transport, NativeModule, Registry, msgs
│   │   ├── blueprints/
│   │   │   ├── full_stack.py     # full_stack_blueprint(): assembles the stacks below
│   │   │   └── stacks/           # composable factory functions
│   │   │       ├── driver.py
│   │   │       ├── lidar.py / sim_lidar.py
│   │   │       ├── slam.py
│   │   │       ├── maps.py
│   │   │       ├── perception.py
│   │   │       ├── memory.py
│   │   │       ├── planner.py
│   │   │       ├── navigation.py
│   │   │       ├── exploration.py     # tare | none (wavefront removed 2026-04-25)
│   │   │       ├── safety.py
│   │   │       └── gateway.py
│   │   └── tests/                # ~1000+ framework tests, no ROS2 needed
│   ├── nav/               # NavigationModule, SafetyRing, CmdVelMux, OccupancyGrid, ESDF, …
│   ├── semantic/          # perception/ + planner/ + reconstruction/
│   ├── memory/            # SemanticMapper, EpisodicMemory, Tagged, VectorMemory, KG
│   ├── drivers/           # thunder/ (gRPC), sim/ (stub, MuJoCo, ROS2), TeleopModule
│   ├── gateway/           # GatewayModule (FastAPI), MCPServerModule
│   ├── webrtc/            # Optional WebRTC camera stream module (aiortc)
│   ├── base_autonomy/     # C++ terrain + local planner + path follower (nanobind)
│   ├── global_planning/   # PCT planner (C++ ele_planner.so) + Python adapters
│   ├── slam/              # Fast-LIO2, Point-LIO, PGO, Localizer, GNSS bridge
│   ├── exploration/       # tare_planner submodule + supervisor + Python TAREExplorerModule
│   └── reconstruction/    # 3D reconstruction
│
├── config/                # YAML / DDS / robot params (devices.yaml, robot_config.yaml, …)
├── calibration/           # Sensor calibration toolbox (camera, IMU, LiDAR, camera-LiDAR)
├── launch/                # ROS2 launch — ONLY algorithm-bridge launches now
│   ├── _robot_config.py
│   └── profiles/          # SLAM/planner profiles consumed by NativeModule subprocesses
│       ├── slam_fastlio2.launch.py
│       ├── slam_pointlio.launch.py
│       ├── slam_stub.launch.py
│       ├── localizer_icp.launch.py
│       ├── planner_pct.launch.py
│       ├── planner_pct_py.launch.py
│       ├── planner_far.launch.py
│       └── planner_stub.launch.py
├── sim/                   # MuJoCo simulation assets, scenes, scripts
├── tests/                 # Integration + planning tests (some need ROS2)
├── tools/                 # Robot-side helpers (dashboards, BPU export, diagnostics)
├── scripts/               # Build helpers, deploy, ota, proto, lingtu CLI shell wrapper
│   ├── build/             # ROS workspace build helpers, fetch_ortools, build_tare, …
│   ├── deploy/            # Installers, systemd, OTA, monitoring
│   ├── ota/               # Package and push to robot
│   ├── proto/             # protoc helpers
│   ├── lingtu             # Bash CLI for ops on the robot (status / map / nav / svc / log)
│   └── …                  # build_dufomap.sh, build_nav_core.sh, doctor.py, ...
├── web/                   # React + Vite dashboard
├── docs/                  # Architecture, guides, ADRs
├── Makefile               # colcon build / test / format / lint / mapping / navigation
├── pyproject.toml         # Root package: lingtu (setuptools)
├── requirements.txt       # S100P runtime deps
└── docker-compose.yml     # Container orchestration
```

## Entry-point precedence

1. `python lingtu.py [profile]` — primary, CLI + REPL.
2. `lingtu` — pip console script (`pyproject.toml` → `lingtu = lingtu_cli:main`).
3. `python main_nav.py` — alias kept for older scripts.
4. `make mapping` / `make navigation` — thin wrappers around `lingtu.py map` / `lingtu.py s100p`.

There are **no** Module-First ROS2 launch files. The Module is the runtime
unit; SLAM and other C++ subsystems are managed via `NativeModule` (see
`docs/MODULE_FIRST_GUIDELINE.md`).

## Where things actually live

| You want… | Read |
|-----------|------|
| The eight architectural rules | `docs/MODULE_FIRST_GUIDELINE.md` |
| Which modules a profile pulls in | `cli/profiles_data.py` + `src/core/blueprints/full_stack.py` |
| All cross-stack wires | `src/core/blueprints/full_stack.py` |
| Which backends are registered for a category | `src/core/registry.py` plus the `@register(...)` calls in each Module file |
| Robot physical parameters | `config/robot_config.yaml` (single source of truth) |
| Calibration SOP | `calibration/README.md` |
| Operations CLI on the robot | `scripts/lingtu` + `docs/AGENTS.md` |
