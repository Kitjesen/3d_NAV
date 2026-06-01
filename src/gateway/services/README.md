# Services — Business Logic Behind the REST API

This package implements the business logic layer that powers the Gateway's REST endpoints. Each service module encapsulates a cohesive set of operations, keeping route handlers thin.

## Files

- **`app_bootstrap.py`** — Application bootstrap: service initialization order, dependency resolution, and startup validation.
- **`commands.py`** — Command dispatch service: routes REST commands to appropriate module ports with validation.
- **`control_commands.py`** — Control commands: speed override, mode switch, and emergency stop execution.
- **`goal_builder.py`** — Goal construction: converts text/fuzzy targets into typed Goal messages via the resolution chain.
- **`map_paths.py`** — Map path resolution: translates map names to filesystem paths and validates map directory structure.
- **`map_safety.py`** — Map safety checks: validates map integrity, occupancy grid bounds, and navigation feasibility.
- **`media_status.py`** — Media status: camera stream health, recording state, and storage capacity reporting.
- **`readiness.py`** — System readiness: pre-flight checks for all subsystems before mission start.
- **`runtime_dataflow.py`** — Runtime data flow: coordinates data movement between modules at runtime.
- **`runtime_status.py`** — Runtime status aggregation: collects module states into a unified status report.
- **`runtime_switch_plan.py`** — Runtime switch plan: coordinates safe mode transitions (mapping↔navigation).
- **`safety_status.py`** — Safety status: aggregates SafetyRing, Geofence, and CmdVelMux states into a single report.
- **`state_snapshot.py`** — Full system snapshot: captures all module states for diagnostics and debugging.
- **`telemetry_normalizers.py`** — Telemetry normalization: converts raw sensor values to standard units and coordinate frames.
- **`traffic.py`** — Traffic management: rate-limiting, request queuing, and concurrent access control.
