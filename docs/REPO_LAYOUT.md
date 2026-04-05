# LingTu repository layout

```
brain/lingtu/
├── lingtu.py              # Primary Python entry (Module-First CLI + REPL)
├── main_nav.py            # Alias → same as lingtu.py
├── lingtu_cli.py          # pip console script `lingtu`
├── cli/                   # CLI implementation (profiles, REPL, daemon)
├── src/                   # Python packages + ROS2 packages (colcon)
├── config/                # YAML / DDS / robot params
├── launch/                # ROS2 launch (legacy / bridge stacks)
├── sim/                   # MuJoCo simulation assets + scripts
├── tests/                 # Integration & planning tests
├── tools/                 # Robot-side helpers (dashboard, BPU export, …)
├── scripts/
│   ├── build/             # ROS workspace build helpers (see scripts/build/README.md)
│   ├── deploy/            # Installers, systemd, OTA, monitoring bots
│   │   └── infra/stack/   # Docker + extra systemd + cron/logrotate (was top-level deploy/)
│   ├── ota/               # Package & push to robot
│   ├── proto/             # protoc helpers
│   └── …
├── assets/
│   ├── media/             # Generated videos (gitignored when large)
│   └── models/            # Optional local weights (gitignored)
├── docs/                  # Architecture, guides, ADRs
├── research/              # Reference repos / papers (optional)
└── …                      # Makefile, docker-compose, VERSION, etc.
```

**Entry points**: Prefer `python lingtu.py` or `lingtu` after `pip install -e .`. ROS-centric flows may still use `./mapping.sh`, `./build_all.sh`, or `ros2 launch launch/...`.
