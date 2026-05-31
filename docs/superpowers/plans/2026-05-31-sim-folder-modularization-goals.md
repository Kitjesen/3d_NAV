# Simulation Folder And Modularization Goals

Date: 2026-05-31

## Goal

Close the LingTu server-side simulation validation loop without real robot
motion. The target evidence must cover multi-floor scenes, LiDAR/SLAM
localization, global planning, local planning, tracking, and exploration. Every
motion command claim must preserve `simulation_only=true`,
`real_robot_motion=false`, and `cmd_vel_sent_to_hardware=false`.

## Stop Conditions

- Do not run real robot services, publish to hardware topics, or start
  `lingtu.py nav/map/explore` against a physical robot.
- Accept a simulation gate only from fresh JSON evidence, not from stale
  artifacts or README claims.
- Treat Dimos benchmark closure and G4 full simulation closure as separate
  contracts until their required gate sets are explicitly aligned.

## Current Folder Contract

| Path | Role | Decision |
| --- | --- | --- |
| `sim/engine/` | Simulation runtime core, world registry, MuJoCo/Gazebo/CMU adapters. | Keep canonical. |
| `sim/worlds/` | Canonical XML/SDF world and scene definitions used by profiles and contracts. | Keep canonical. |
| `sim/assets/` | Canonical Thunder v3 robot assets, MJCF/URDF/meshes, Livox pattern assets. | Keep canonical. |
| `sim/robots/` | Compatibility robot asset paths and policy fallback paths for older scripts. | Keep as compatibility index. |
| `sim/scripts/` | Stable launcher, gate, validation, demo, and benchmark script contract. | Do not physically split; add indexes/tests instead. |
| `sim/validation/` | Full-system validation package imported by script wrappers and tests. | Keep canonical. |
| `sim/evaluation/` | Offline SLAM evaluation manifests and dataset tooling. | Keep separate from live simulation gates. |
| `sim/following/` | Person-following simulation and behavior metrics. | Keep separate; add gate only when claiming person tracking. |
| `sim/bridge/` | Legacy top-level bridge entrypoints. | Keep as legacy until references are removed. |
| `sim/launch/` | ROS 2/MuJoCo launch wrappers. | Keep; document legacy vs current runtime contract. |
| `sim/sensors/` | Sensor simulators/fallbacks not owned by runtime engine. | Boundary README added; keep contract locked. |
| `sim/datasets/` | Offline datasets and metadata. | Boundary README added; keep dataset/large-file policy locked. |
| `sim/output/` | Generated local outputs. | Prefer `artifacts/` for reproducible evidence. |
| `sim/external_scenes/` | External/license-constrained scene placeholders. | Keep. |
| `sim/meshes/` | Legacy mesh path. | Do not move until references are audited. |
| `sim/maps/`, `sim/configs/` | Reserved placeholders. | Keep only if referenced; otherwise prune later. |
| `sim/semantic/` | Legacy semantic simulation residue. | Move to tests or mark experimental later. |

## Migration Status

Completed:

- Core scene migration: `sim/scenes/*.xml` moved into `sim/worlds/`.
- Core robot entry migration: `sim/robot/thunder.urdf` moved under
  `sim/robots/`.
- Current stable runtime entrypoints are `sim/worlds`, `sim/assets`,
  `sim/robots`, `sim/scripts`, and `lingtu.py --endpoint ...`.

Intentional non-migration:

- `sim/scripts/<name>` remains a stable path contract. The earlier idea to move
  scripts into physical `gates/`, `validation/`, and `demos/` folders is not the
  current plan because profiles, runtime contracts, tests, and CI refer to these
  paths directly.

Known cleanup targets:

- `sim/README.md` stale `scenes/` and `robot/` directory references were
  corrected in the same pass that created this goal file.
- `sim/bridge/nova_nav_bridge.py` has been moved off the deleted `sim/robot`
  defaults and now points at `sim/robots/nova_dog/robot_with_camera.xml` plus
  `sim/robots/nova_dog/policy.onnx`.
- `tests/planning/` is legacy but still referenced by simulation contracts and
  profile graph tests.

## Modularization Targets

P0: safety and command-chain contracts

- MCP `stop_cmd` must reach the driver stop input and Navigation stop input.
- MCP `cmd_vel` must never bypass `CmdVelMux`; it is treated as an external
  manual control source and enters `CmdVelMux.teleop_cmd_vel`.
- MCP `set_mode("estop")` must publish both `stop_cmd` and a zero `cmd_vel` so
  estop mode follows the same safety/mux path as the explicit stop tool.
- Gateway and MCP backend route tables must stay in parity until moved to a
  shared metadata source.

P1: plugin and backend contracts

- Planner backend switching is currently wrapped by `NavigationModule`; decide
  whether global planning becomes a first-class Module or remains an internal
  service with an explicit no-hot-switch contract.
- `local_planner`, `path_follower`, and `terrain` expose status but do not yet
  implement runtime backend switching.
- Registry-backed plugins are still constrained by module-local static
  allowlists in several places.
- Perception has three plugin surfaces: `PerceptionModule`, legacy
  `DetectorModule`/`EncoderModule`, and `PerceptionFactory`; one canonical
  surface should be selected.

Status: `PerceptionModule` constructor now validates detector/encoder backends
against the live registry catalog, so newly registered providers can be used as
startup configuration as well as runtime switch targets.

Motion backends remain fail-closed for runtime reconfiguration:
`NavigationModule` planner, `LocalPlannerModule`, `PathFollowerModule`,
`TerrainModule`, and `SlamBridgeModule` all inherit the default unsupported
response unless a future change adds a specifically tested safe switch path.

P2: observability and evidence

- Diagnostics should preserve plugin fields such as `fallback_backend`,
  `reconfigurable`, `capabilities`, and readiness, not only backend/degraded
  fields.
- Stack factories should report missing required modules as explicit health
  issues instead of silently building partial graphs after `ImportError`.
- ESDF wiring into `LocalPlannerModule` is currently reserved/cached; do not
  claim ESDF affects local planning until a behavior test proves it.
- Closure reports should present missing required gates in canonical execution
  order, not alphabetic order, so the next-action queue matches
  `required_gate_sequence`.

## Current Algorithm Chain

PCT and A* are global planner backends. Current `nav`, `explore`, and
`tare_explore` style profiles keep `enable_native=False`, so local planning and
tracking use the in-process CMU-style chain:
`LocalPlannerModule(backend="nanobind") -> PathFollowerModule(backend="nav_core")
-> CmdVelMux -> driver`.

The external CMU ROS 2 `localPlanner/pathFollower` chain is still available for
dedicated native simulation gates such as `native_pct_mujoco`; it is not the
default Python Module runtime path.

Algorithm handoff contract:

1. SLAM or driver odometry fans into Navigation, maps, local planner, path
   follower, safety, Gateway, and exploration consumers.
2. Map cloud fans into occupancy, voxel, elevation, terrain, Gateway, and
   visualization consumers.
3. `NavigationModule` owns global planning through `GlobalPlannerService`
   (`pct` or `astar`) and publishes `global_path` plus waypoints.
4. `LocalPlannerModule` consumes `global_path`, odometry, costmap, terrain, and
   reserved ESDF input, then publishes `local_path`.
5. `PathFollowerModule` tracks `local_path` into `cmd_vel`.
6. `CmdVelMux` arbitrates teleop, visual servo, Navigation recovery, and
   path-follower commands before the selected driver sees velocity commands.
7. `SafetyRingModule` monitors mux output and publishes stop commands to the
   driver and Navigation.

Known weak points:

- PCT native replanning does not currently consume live fused costmap as a full
  dynamic replanning source; dynamic response is mainly local-planner and
  safety-layer evidence.
- ESDF is wired but not yet behavior-proven in local scoring.
- TARE publishes exploration goals/path strategy into Navigation; it does not
  own direct driver command output.
- Optional planning/observability wires can still degrade quietly when a module
  or port is absent; only L0 stop wiring is currently fail-closed.

## Simulation Evidence Matrix

| Area | Required gates/tests |
| --- | --- |
| Multi-floor | `multifloor_exploration`; route matrix with PCT and A*. |
| LiDAR/SLAM localization | `fastlio2_dynamic_inspection`, `large_loop_closure`. |
| Global planning | `large_terrain`, `native_pct_mujoco`, `pct_saved_map_navigation`. |
| Local planning | `dynamic_obstacle_local_planner`, `moving_obstacle_sweep`, `native_pct_mujoco`. |
| Nav tracking | `multifloor_exploration`, `native_pct_mujoco`. |
| Person/target tracking | Add a dedicated person-tracking simulation gate before claiming this. |
| Exploration | `gazebo_runtime`, `multifloor_exploration`; add `mujoco_tare_exploration` only when claiming TARE. |
| Saved-map relocalization | `saved_map_relocalize`, `pct_saved_map_navigation`; optional `bbs3d_kidnapped_relocalize`. |

## Current G4 Closure Snapshot

Latest local aggregator run:

```bash
PYTHONPATH=src:. python sim/scripts/server_sim_closure.py \
  --preset g4_server_full_sim \
  --required-only \
  --json-out artifacts/server_sim_closure_summary_g4_current.json
```

Result on 2026-05-31:

- `ok=false`
- `simulation_only=true`
- `real_robot_motion=false`
- `cmd_vel_sent_to_hardware=false`
- Fresh verified required gates: `gateway_runtime_acceptance`,
  `routecheck_preflight`, and `dynamic_obstacle_local_planner`.
- Required gates still blocking: `multifloor_exploration`, `large_terrain`,
  `native_pct_mujoco`,
  `fastlio2_dynamic_inspection`, `moving_obstacle_sweep`,
  `large_loop_closure`, `gazebo_runtime`, `saved_map_relocalize`, and
  `pct_saved_map_navigation`.

Fresh gate detail:

| Gate | Current status | Evidence / blocker |
| --- | --- | --- |
| `routecheck_preflight` | Pass | `artifacts/server_sim_closure/routecheck/summary.json`; non-motion routecheck kept published `goal_pose`, `cmd_vel`, and `stop_cmd` counts at zero. |
| `dynamic_obstacle_local_planner` | Pass | `artifacts/server_sim_closure/dynamic_obstacle_local_planner/report.json`; nanobind backend verified dynamic replan, obstacle response, clear-path recovery, and non-hardware command fields. |
| `gateway_runtime_acceptance` | Pass | `artifacts/server_sim_closure/gateway_runtime_acceptance/report.json`; when the default local HTTP Gateway is absent, non-motion acceptance now falls back to an in-process `GatewayModule` stub with `snapshot_source=in_process_stub`, `simulation_only=true`, `real_robot_motion=false`, `cmd_vel_sent_to_hardware=false`, complete product links, ModulePort-primary dataflow, command whitelist, and 8 runtime stages. This is data-plane contract evidence only, not live motion or field readiness. |
| `multifloor_exploration` | Fail | `artifacts/server_sim_closure/multifloor_exploration/report.json`; upper/cross-floor far safe-goal projections are no longer counted as feasible requested-goal plans. The gate now reports `environment_runtime` plus `planning_tracking`, and the highest-priority runtime blocker is `PCT native runtime unavailable` on this Windows/Python 3.13 host. |
| `large_terrain` | Fail | `artifacts/server_sim_closure/large_terrain/report.json`; assets and non-motion flags refresh successfully, but PCT native runtime is unavailable because the checked-in native modules are Linux ELF / Python 3.10-oriented while this host is Windows / Python 3.13. |

Stop condition: do not claim G4 full simulation health until every required gate
above is fresh and passing in `artifacts/server_sim_closure_summary_g4_current.json`.

## Next Execution Goals

G1. Done in this pass: lock MCP command-chain wiring with profile graph tests.

G2. Done in the continuation pass: create `g4_server_full_sim` as the
server-side full simulation preset. It is Dimos plus
`multifloor_exploration`, and README, algorithm gate constants, closure
script preset selection, default freshness checks, and pytest expectations are
aligned.

G3. Done in the current continuation: add the folder boundary goal file, fix
the live `sim/README.md` directory table, and add per-folder boundary indexes
for `sim/sensors/`, `sim/datasets/`, and legacy `sim/bridge/` without moving
stable `sim/scripts` paths.

G4. Partially done: add backend allowlist/catalog parity tests and make
`terrain/cmu` a registered NativeModule alias instead of an invisible
allowlist-only backend. Existing runtime backend switch tests still lock the
fail-closed/no-hot-switch behavior.

G5. Done in the continuation pass: diagnostics active backend status now
preserves extended metadata such as `fallback_backend`, `reconfigurable`,
`capabilities`, and nested readiness fields.

G6. Active now: turn the G4 closure snapshot into a repeatable execution queue.
Local non-motion gates are refreshed first; ROS2/MuJoCo/Linux-native gates stay
blocked until a host with ROS 2 Humble, native PCT libraries, and MuJoCo EGL can
generate fresh artifacts without connecting to robot hardware.

G7. Done in the current continuation: order `missing_or_failed`,
`remaining_gaps`, optional gaps, and generated missing-gate commands by
canonical gate order instead of alphabetic order.

G8. Done in the current continuation: add `core.plugin_seed` as the central
built-in plugin registration seed for safe core surfaces. Only pure registration
triggers are wired through it now (`driver` stack, `GlobalPlannerService`, and
CLI backend validation). Gateway/WebRTC/visualization remain explicit opt-in
seed groups because they include optional runtime surfaces or duplicate
registration names.

G9. Done in the current continuation: make `gateway-runtime-acceptance
--acceptance-mode non_motion` repeatable without a live HTTP Gateway by using a
strict in-process stub fallback only when all required default-local Gateway
fetches fail. The fallback does not call `GatewayModule.start()`, disables the
blackbox recorder through a temporary env override, and asserts command output
ports remain unpublished.

G10. Done in the xhigh continuation: organize the remaining simulation folder
contracts without moving stable entrypoints. `sim/engine/README.md` now defines
the runtime-core boundary, `artifacts/server_sim_closure/README.md` defines the
generated evidence boundary, `sim/scripts/README.md` indexes the stable
validation helpers, and `sim/README.md` points full-closure users at the
canonical G4 summary artifact.

G11. Done in the xhigh continuation: move another set of stack factories toward
module-first plugin resolution. `safety()`, `lidar()`, and `gateway()` now
resolve their built-in modules through the registry seed path with explicit
fallbacks, and tests lock fake registry substitution for those stack factories.
This does not make motion backends hot-swappable; runtime switching remains
fail-closed unless a specifically tested safe path exists.

G12. Next parallel target: split the remaining work into three lanes.
Lane A fixes or documents the PCT native runtime contract for Windows/Python
3.13 versus Linux/Python 3.10 evidence hosts. Lane B turns the remaining
artifact-contract gates (`fastlio2_dynamic_inspection`,
`moving_obstacle_sweep`, `large_loop_closure`, `gazebo_runtime`,
`saved_map_relocalize`, `pct_saved_map_navigation`, `native_pct_mujoco`) into
fresh runnable/non-runnable evidence with exact host requirements. Lane C
continues module-first pluginization for planner, local planner, path follower,
terrain, SLAM, perception, gateway tools, and exploration without introducing
new dependencies.

G13. Done in the current xhigh continuation: `server_sim_closure` now publishes
structured `host_requirements` for each gate, and propagates them into
`missing_required_commands`, `next_actions`, and the per-gate summary entry.
The current G4 summary artifact therefore distinguishes local non-motion Python
gates from Linux/ROS 2/MuJoCo/PCT-native gates before a worker tries to execute
the queue.

## Current Verification Evidence

Fresh checks from this pass:

```bash
python -m pytest src/core/tests/test_plugin_seed.py src/core/tests/test_stack_registry_resolution.py src/core/tests/test_sim_runtime_compat.py::test_sim_boundary_indexes_document_stable_contracts src/core/tests/test_server_sim_closure.py::test_server_sim_closure_multifloor_surfaces_pct_runtime_blocker src/core/tests/test_multifloor_sim_validation.py::test_partial_global_plan_is_kept_for_diagnostics_but_not_feasible src/core/tests/test_multifloor_sim_validation.py::test_far_projected_safe_goal_is_not_counted_as_requested_goal -q
# 10 passed

python -m py_compile src/core/blueprints/stacks/_registry.py src/core/blueprints/stacks/safety.py src/core/blueprints/stacks/lidar.py src/core/blueprints/stacks/gateway.py src/drivers/real/lidar/lidar_module.py src/nav/cmd_vel_mux_module.py src/nav/services/nav_services/geofence_manager_module.py sim/scripts/multifloor_nav_validation.py sim/scripts/server_sim_closure.py
# passed

PYTHONPATH=src:. python sim/scripts/large_terrain_nav_validation.py --output-dir artifacts/server_sim_closure/large_terrain --planners pct,astar --json-out artifacts/server_sim_closure/large_terrain/report.json
# exited 0; report ok=false, simulation_only=true, real_robot_motion=false,
# cmd_vel_sent_to_hardware=false; environment_blockers=["PCT native runtime unavailable"]

PYTHONPATH=src:. python sim/scripts/server_sim_closure.py --preset g4_server_full_sim --required-only --json-out artifacts/server_sim_closure_summary_g4_current.json
# exited 0; summary ok=false, simulation_only=true, real_robot_motion=false,
# cmd_vel_sent_to_hardware=false; verified true: gateway_runtime_acceptance,
# routecheck_preflight, dynamic_obstacle_local_planner; multifloor and
# large_terrain now surface environment_runtime/planning_tracking blockers;
# next_actions and missing_required_commands include host_requirements.

python -m pytest src/core/tests/test_server_sim_closure.py::test_server_sim_closure_summary_lists_missing_required_commands src/core/tests/test_server_sim_closure.py::test_server_sim_closure_next_actions_separate_runtime_blocker_from_missing_report -q
# 2 passed

python -m pytest src/core/tests/test_server_sim_closure.py -q
# 121 passed

python -m pytest src/core/tests/test_plugin_seed.py src/core/tests/test_backend_status.py src/core/tests/test_registry.py -q
# 28 passed

python -m pytest src/core/tests/test_sim_runtime_compat.py::test_sim_boundary_indexes_document_stable_contracts src/core/tests/test_sim_runtime_compat.py::test_legacy_nova_nav_bridge_uses_current_robot_paths src/core/tests/test_sim_runtime_compat.py::test_sim_mujoco_full_stack_routes_autonomy_cmds_through_mux src/core/tests/test_sim_runtime_compat.py::test_full_stack_mux_wiring_tolerates_legacy_nav_without_recovery_cmd -q
# 4 passed

python -m pytest src/core/tests/test_server_sim_closure.py -q -k "g4_summary_required_gate_sequence_preserves_core_order or g4_missing_required_gates_preserve_core_order or g4_required_gates_are_default_freshness_required or readme_current_full_closure_gates_match_g4_server_full_sim_preset or readme_full_closure_command_uses_g4_server_full_sim_preset or server_sim_closure_g4_server_full_sim_preset_selects_closure_gate_set"
# 6 passed, 114 deselected

python -m py_compile src/core/plugin_seed.py cli/main.py src/core/blueprints/stacks/driver.py src/nav/global_planner_service.py sim/scripts/server_sim_closure.py
# passed

PYTHONPATH=src:. python sim/scripts/server_sim_closure.py --preset g4_server_full_sim --json-out artifacts/server_sim_closure_summary_g4_current.json
# exited 0; summary ok=false, simulation_only=true, real_robot_motion=false,
# cmd_vel_sent_to_hardware=false; verified true: gateway_runtime_acceptance,
# routecheck_preflight, dynamic_obstacle_local_planner;
# blocking required gates remain: multifloor_exploration, large_terrain,
# native_pct_mujoco, fastlio2_dynamic_inspection, moving_obstacle_sweep,
# large_loop_closure, gazebo_runtime, saved_map_relocalize,
# pct_saved_map_navigation

python -m pytest src/gateway/tests/test_gateway_runtime_acceptance.py::test_gateway_runtime_acceptance_passes_non_motion_without_ros2_topic src/gateway/tests/test_gateway_runtime_acceptance.py::test_gateway_runtime_acceptance_non_motion_exposes_top_level_sim_safety_flags src/gateway/tests/test_gateway_runtime_acceptance.py::test_gateway_runtime_acceptance_non_motion_uses_local_stub_when_gateway_is_down src/gateway/tests/test_gateway_runtime_acceptance.py::test_gateway_runtime_acceptance_field_does_not_use_local_stub_when_gateway_is_down src/gateway/tests/test_gateway_runtime_acceptance.py::test_gateway_runtime_acceptance_in_process_stub_stays_non_motion src/gateway/tests/test_gateway_runtime_acceptance.py::test_gateway_runtime_acceptance_simulation_passes_without_real_runtime_evidence src/gateway/tests/test_gateway_runtime_acceptance.py::test_gateway_runtime_acceptance_field_passes_with_live_samples -q
# 7 passed

python -m py_compile src/core/gateway_runtime_acceptance.py src/gateway/tests/test_gateway_runtime_acceptance.py
# passed

PYTHONPATH=src:. python lingtu.py gateway-runtime-acceptance --acceptance-mode non_motion --json-out artifacts/server_sim_closure/gateway_runtime_acceptance/report.json
# exited 0; report ok=true, snapshot_source=in_process_stub,
# runtime_contract=in_process_stub, simulation_only=true,
# real_robot_motion=false, cmd_vel_sent_to_hardware=false, blockers=[]

PYTHONPATH=src:. python sim/scripts/server_sim_closure.py --required gateway_runtime_acceptance --required-only --gateway-runtime-acceptance-report artifacts/server_sim_closure/gateway_runtime_acceptance/report.json --json-out artifacts/server_sim_closure_gateway_runtime_acceptance_check.json --strict
# exited 0; gateway_runtime_acceptance passed as a single required gate

python -m pytest src/core/tests/test_server_sim_closure.py -q
# 119 passed

python -m pytest src/core/tests/test_backend_status.py -q
# 13 passed

python -m pytest src/core/tests/test_runtime_backend_switch.py -q
# 7 passed

python -m pytest src/gateway/tests/test_gateway_runtime_status.py -q
# 78 passed

python -m pytest src/core/tests/test_profile_graph_snapshots.py -q
# 33 passed

python -m py_compile src/core/algorithm_gates.py sim/scripts/server_sim_closure.py src/base_autonomy/modules/terrain_module.py src/gateway/routes/diagnostics.py
# passed

python -m pytest src/core/tests/test_profile_graph_snapshots.py::test_profile_graph_snapshot_locks_safety_gateway_and_mux_edges -q
# 1 passed

python -m pytest src/core/tests/test_sim_runtime_compat.py::test_sim_mujoco_full_stack_routes_autonomy_cmds_through_mux src/core/tests/test_sim_runtime_compat.py::test_full_stack_mux_wiring_tolerates_legacy_nav_without_recovery_cmd -q
# 2 passed

python -m pytest src/gateway/tests/test_gateway_runtime_status.py::test_gateway_and_mcp_backend_route_tables_stay_in_parity src/gateway/tests/test_gateway_runtime_status.py::test_mcp_backend_switch_tool_uses_gateway_guard src/gateway/tests/test_gateway_runtime_status.py::test_mcp_backend_switch_tool_guards_motion_without_gateway_module src/gateway/tests/test_gateway_runtime_status.py::test_mcp_backend_switch_reads_nested_navigation_state_without_gateway_module -q
# 4 passed

python -m pytest src/core/tests/test_sim_runtime_compat.py::test_legacy_nova_nav_bridge_uses_current_robot_paths -q
# 1 passed

python -m py_compile sim/bridge/nova_nav_bridge.py
# passed

python -m pytest src/core/tests/test_new_modules.py::TestMCPServerModule::test_set_mode_via_skill src/core/tests/test_new_modules.py::TestMCPServerModule::test_set_mode_estop_publishes_stop_and_zero_twist src/core/tests/test_new_modules.py::TestMCPServerModule::test_set_mode_invalid -q
# 3 passed

python -m pytest src/gateway/tests/test_gateway_runtime_status.py::test_gateway_and_mcp_backend_route_tables_stay_in_parity src/gateway/tests/test_mcp_auth.py -q
# 7 passed

python -m py_compile src/gateway/mcp_server.py
# passed

python -m pytest src/core/tests/test_server_sim_closure.py -q -k "dimos_required_gates_come_from_core_algorithm_gate_constant or g4_server_full_sim_required_gates_come_from_core_algorithm_gate_constant or readme_current_full_closure_gates_match_g4_server_full_sim_preset or readme_full_closure_command_uses_g4_server_full_sim_preset or g4_summary_required_gate_sequence_preserves_core_order or g4_required_gates_are_default_freshness_required or dimos_summary_required_gate_sequence_preserves_core_order or server_sim_closure_g4_server_full_sim_preset_selects_closure_gate_set"
# 8 passed, 111 deselected

python -m pytest src/core/tests/test_backend_status.py::test_autonomy_backend_registry_names_are_visible src/core/tests/test_backend_status.py::test_autonomy_backend_allowlists_match_registry_catalog src/core/tests/test_backend_status.py::test_terrain_cmu_backend_uses_native_setup -q
# 3 passed

python -m pytest src/gateway/tests/test_gateway_runtime_status.py::test_diagnostics_plugin_catalog_route_exposes_active_backend_status -q
# 1 passed

python -m py_compile src/base_autonomy/modules/terrain_module.py src/gateway/routes/diagnostics.py
# passed

python -m pytest src/core/tests/test_perception_module.py::TestDetectorConfiguration::test_constructor_accepts_registered_detector_and_encoder_plugins src/core/tests/test_perception_module.py::TestDetectorConfiguration::test_unknown_perception_backend_fails_fast src/core/tests/test_perception_module.py::TestDetectorConfiguration::test_perception_backend_registry_names_are_visible -q
# 3 passed

python -m pytest src/core/tests/test_runtime_backend_switch.py::test_perception_reconfigure_detector_rejects_unknown_backend src/core/tests/test_runtime_backend_switch.py::test_perception_reconfigure_detector_updates_health_status src/core/tests/test_runtime_backend_switch.py::test_perception_reconfigure_encoder_updates_health_status -q
# 3 passed

python -m py_compile src/semantic/perception/semantic_perception/perception_module.py
# passed

python -m pytest src/core/tests/test_runtime_backend_switch.py::test_motion_modules_reconfigure_backend_fails_closed -q
# 5 passed

python -m pytest src/core/tests/test_sim_runtime_compat.py::test_sim_boundary_indexes_document_stable_contracts -q
# 1 passed

python -m pytest src/core/tests/test_server_sim_closure.py::test_g4_missing_required_gates_preserve_core_order -q
# 1 passed

python -m pytest src/core/tests/test_runtime_backend_switch.py -q
# 12 passed

python -m pytest src/core/tests/test_backend_status.py -q
# 13 passed

python -m pytest src/core/tests/test_perception_module.py::TestDetectorConfiguration -q
# 10 passed

python -m pytest src/core/tests/test_sim_runtime_compat.py::test_legacy_nova_nav_bridge_uses_current_robot_paths src/core/tests/test_sim_runtime_compat.py::test_sim_boundary_indexes_document_stable_contracts src/core/tests/test_sim_runtime_compat.py::test_sim_mujoco_full_stack_routes_autonomy_cmds_through_mux src/core/tests/test_sim_runtime_compat.py::test_full_stack_mux_wiring_tolerates_legacy_nav_without_recovery_cmd -q
# 4 passed

python -m pytest src/gateway/tests/test_gateway_runtime_status.py::test_gateway_and_mcp_backend_route_tables_stay_in_parity src/gateway/tests/test_gateway_runtime_status.py::test_mcp_backend_switch_tool_uses_gateway_guard src/gateway/tests/test_gateway_runtime_status.py::test_mcp_backend_switch_tool_guards_motion_without_gateway_module src/gateway/tests/test_gateway_runtime_status.py::test_mcp_backend_switch_reads_nested_navigation_state_without_gateway_module src/gateway/tests/test_gateway_runtime_status.py::test_diagnostics_plugin_catalog_route_exposes_active_backend_status -q
# 5 passed

PYTHONPATH=src:. python sim/scripts/server_sim_closure.py --preset g4_server_full_sim --required-only --json-out NUL
# exited 0; summary ok=false, simulation_only=true, real_robot_motion=false
# verified true: gateway_runtime_acceptance, routecheck_preflight,
# dynamic_obstacle_local_planner
# still failed/missing/stale: multifloor_exploration, large_terrain,
# native_pct_mujoco, fastlio2_dynamic_inspection, moving_obstacle_sweep,
# large_loop_closure, gazebo_runtime, saved_map_relocalize,
# pct_saved_map_navigation
```

Stale path search still has expected historical references in plan/archive
documents. The live `sim/README.md` and `sim/bridge/nova_nav_bridge.py` checks
no longer match `sim/robot` or `sim/scenes` runtime paths.
