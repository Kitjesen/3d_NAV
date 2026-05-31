# Exploration — Autonomous Exploration Index

> Files live under `src/exploration/`
> 5 .py files (non-test)

---

| File | Responsibility |
|------|---------------|
| `__init__.py` | Package init |
| `exploration_supervisor_module.py` | **ExplorationSupervisorModule** — high-level exploration orchestration: manages mission lifecycle, frontier selection, coverage completion detection |
| `tare_explorer_module.py` | **TAREExplorerModule** — CMU TARE hierarchical exploration backend: coarse-to-fine multi-resolution frontier planning, DDS-based subprocess communication |
| `tare_ros2_bridge_module.py` | **TARERos2BridgeModule** — ROS2 bridge for TARE: converts standard ROS2 topics to DDS for the TARE subprocess |
| `native_factories.py` | Native binary factory — builds Launcher for TARE C++ exploration binary |
