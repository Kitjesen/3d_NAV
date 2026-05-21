# DimOS Benchmark Gap Matrix

Last updated: 2026-05-21.

This matrix converts the DimOS-style validation pattern into LingTu gates. It is
not a product claim that LingTu matches DimOS feature-for-feature. It is a
repeatable acceptance surface for the parts of DimOS that are relevant to this
project: replay/simulation first, fixed waypoint missions, frontier progress,
dynamic obstacle response, saved-map lifecycle, and explicit command safety.

## Reference

- Official repository: <https://github.com/dimensionalOS/dimos>
- Native navigation documentation:
  <https://github.com/dimensionalOS/dimos/blob/main/docs/capabilities/navigation/native/index.md>
- Nav stack documentation:
  <https://github.com/dimensionalOS/dimos/blob/main/docs/capabilities/navigation/nav_stack.md>
- Testing documentation:
  <https://github.com/dimensionalOS/dimos/blob/main/docs/development/testing.md>
- CLI/replay/simulation usage:
  <https://github.com/dimensionalOS/dimos/blob/main/docs/usage/cli.md>

DimOS is useful here mainly as a validation model, not as an algorithm to copy.
Its documented Go2 native stack uses replay and simulation entry points,
voxel/cost mapping, continual replanning, frontier exploration, and visible
CLI/MCP operations. LingTu's stack is different: ROS 2 Humble, Livox
MID-360/Fast-LIO2, PCT, local planner/path follower, saved map/tomogram
lifecycle, and S100P deployment constraints.

The public DimOS repository does not expose a navigation leaderboard that can be
copied one-for-one. The useful pattern is its layered evidence surface:
fast pytest checks, heavier self-hosted/ROS checks, MuJoCo replay/simulation
entry points, rosbag-style output deviation tests, and e2e replanning tests
such as blocking one route and requiring replanning through another route.
LingTu should therefore keep strict JSON gates with concrete artifacts instead
of relying on a single demo video.

Concrete upstream anchors used for this matrix:

| DimOS validation surface | Upstream artifact | LingTu counterpart |
| --- | --- | --- |
| No-hardware replay | `dimos --replay run unitree-go2` in the README; described as quadruped navigation replay with SLAM, costmap, and A* planning. | Saved-map relocalization plus `pct_saved_map_navigation`; add real-log replay before field claims. |
| Simulation run | `dimos --simulation run unitree-go2` and `unitree-g1-sim`. | MuJoCo/Gazebo/CMU Unity runtime gates, all marked simulation-only. |
| Modular nav stack | `create_nav_stack()` composes terrain analysis, local planner, path follower, PGO, FAR/Simple planner, and optional TARE. | Fast-LIO/PCT/local planner/path follower gates, with TARE and wavefront exploration kept as separate surfaces. |
| Rosbag/deviation tests | `test_local_planner_rosbag.py`, `test_path_follower_rosbag.py`, `test_far_planner_rosbag.py`, and `test_pgo_rosbag.py`. | Missing stronger LingTu replay/deviation gates for local path endpoint error, cmd velocity ratio, PGO/loop correction, and path tracking drift. |
| Dynamic replanning e2e | `test_dimsim_path_replaning.py` adds a wall when the robot approaches a door and requires replanning to the target. | `moving_obstacle_sweep` should remain stricter: continuous moving actors, speed/density bins, clearance, live Fast-LIO/PCT/local/cmd proof, and video evidence. |
| Heavy-test separation | DimOS testing docs separate default tests from self-hosted/LFS/ROS/CUDA/MuJoCo/DimSim tests. | LingTu must report which evidence is local pytest, remote simulation, replay, or hardware; local green tests are not enough for algorithm health. |

## Dimos-Style LingTu Preset

Run this preset when the question is "do we have enough DimOS-style evidence to
trust the navigation algorithm surface?"

```bash
PYTHONPATH=src:. python3 sim/scripts/server_sim_closure.py \
  --preset dimos_benchmark \
  --required-only \
  --run-missing \
  --strict \
  --max-report-age-s 86400 \
  --json-out artifacts/server_sim_closure/summary_dimos_benchmark_24h.json
```

`dimos_benchmark` requires:

| Gate | DimOS-style evidence covered |
| --- | --- |
| `routecheck_preflight` | Non-motion route preview and command-safety counters. |
| `large_terrain` | Fixed large-scene route matrix over saved tomogram assets. |
| `native_pct_mujoco` | PCT global path through local planner/path follower into simulated motion. |
| `dynamic_obstacle_local_planner` | Local replanning around changing obstacle phases. |
| `fastlio2_dynamic_inspection` | Raw LiDAR/IMU -> Fast-LIO2 -> LingTu nav plus PCT patrol and moving obstacles with video. |
| `moving_obstacle_sweep` | Aggregates live dynamic-obstacle videos across required speed and point-density bins, with each bin proving the Fast-LIO -> PCT -> local path -> cmd_vel chain. |
| `large_loop_closure` | Large loop route with live Fast-LIO, PCT, local path/cmd_vel closure, video, and return-to-start drift bounds. |
| `gazebo_runtime` | ROS-native smoke, navigation loop, map growth, frontier exploration, and publisher identity. |
| `saved_map_relocalize` | Saved-map/localizer contract, same-source metadata, live Fast-LIO, and localization health. |
| `pct_saved_map_navigation` | Saved map/tomogram -> PCT -> local planning/path following -> motion. |

## Current Gaps To Keep Visible

| Gap | Why it matters | Next gate shape |
| --- | --- | --- |
| Fresh full-stack runtime evidence is expensive and environment-bound | A green stale artifact can hide current regressions. | Keep `--max-report-age-s` on benchmark summaries and rerun stale gates before claims. |
| Dynamic obstacle sweep runtime evidence is newly required | Passing one speed/density, or passing videos without proving live navigation chain wiring, does not prove crowd robustness. | Run `moving_obstacle_sweep` after generating slow/fast x sparse/dense live inspection videos. Each child case must carry `live_nav_chain.ok=true`; the aggregate must carry `required_live_nav_chain=true`. |
| Long-range loop closure runtime evidence is newly required | Short patrol proves local consistency, not large loop drift recovery. | Run `large_loop_closure` and require Fast-LIO/PCT/local path/cmd_vel plus start/end closure error. |
| Saved-map assets must remain same-source | Pairing a relocalization report with a different or merely newer tomogram/map can create false failures or false passes. | Keep saved-map relocalize and PCT navigation gates tied to the same `same_source_map` directory. |
| Real S100P hardware is outside the simulation closure | Simulation-only gates must not be called field readiness. | Add real-log replay first, then hardware dry-run with command boundary and localization health evidence. |
| Frontier/no-gain behavior is spread across runtime gates | Exploration can pass by motion alone if no-gain termination is not checked. | Add a frontier no-gain/stall termination metric to the relevant runtime reports. |
| Video evidence can become a weak proxy | Frame counters alone do not prove the MP4 artifact exists or is decodable. | Strict dynamic-inspection evaluation resolves the video path, requires a non-empty file, and decodes the first frame when OpenCV is available. |

The previous 8-gate `dimos_benchmark` required summary on 2026-05-21 is green:

- Report:
  `artifacts/server_sim_closure/summary_dimos_benchmark_after_saved_map_fix.json`
- `ok=true`, `missing_or_failed=[]`, `remaining_gaps=[]`.
- Verified required gates:
  `routecheck_preflight`, `large_terrain`, `native_pct_mujoco`,
  `dynamic_obstacle_local_planner`, `fastlio2_dynamic_inspection`,
  `gazebo_runtime`, `saved_map_relocalize`, and
  `pct_saved_map_navigation`.

The current preset is stricter than that artifact. It now also requires
`moving_obstacle_sweep` and `large_loop_closure`, so the old green summary is
no longer enough to claim full benchmark closure. Those two reports must be
freshly generated before a new `dimos_benchmark` summary can be green.

Current stricter status after the latest remote physical-rolling reruns and
gate hardening:

- The strict live MuJoCo/Fast-LIO input default is now
  `scan_time_profile=physical_rolling`. This supersedes the earlier
  `synthetic_rolling` default because the accepted model must accumulate
  actual MuJoCo subscans over the MID-360 scan window and publish real
  per-point offsets. Fresh physical-rolling evidence is green for fixed
  forward+yaw Fast-LIO
  (`artifacts/server_sim_closure/fixed_forward_yaw_physical_pass/gate-20260521_154007/report.json`)
  and for a one-goal inspection using live Fast-LIO, saved large-terrain
  tomogram, PCT, local planner/path follower, and nonzero cmd_vel
  (`artifacts/server_sim_closure/inspection_onegoal_physical_fastlio/inspection-20260521_154351/report.json`).
  This is not a DimOS benchmark pass until the large-loop and dynamic-obstacle
  speed/density gates are refreshed under the same physical-rolling default.
- Current strict summary:
  `artifacts/server_sim_closure/summary_dimos_benchmark_physical_rolling_current.json`
  has `ok=false`, `missing_or_failed=["large_loop_closure","moving_obstacle_sweep"]`,
  and `algorithm_validation.claim_allowed=false`.
- `moving_obstacle_sweep` is red under the current strict evaluator:
  `artifacts/server_sim_closure/moving_obstacle_sweep/report_physical_rolling_20260521.json`.
  It covers `slow:sparse`, `slow:dense`, `fast:sparse`, and `fast:dense` with
  `scan_time_profile=physical_rolling`, the
  `physical_subscans_with_actual_sim_time_offsets` contract, real video files,
  moving obstacles, and no trail collision. All four child cases still fail:
  the robot reaches only one of three inspection checkpoints, and the child
  reports show Fast-LIO motion/Z divergence under the dynamic scene. The gate
  summary now surfaces `fastlio2_consistency`, `blocking_subsystems`, and
  `minimal_red_defect` so this failure is visible without manually opening
  every child report.
- `large_loop_closure` is red under the current same-source strict evaluator:
  `artifacts/server_sim_closure/large_loop_physical_rolling_same_source/large_loop_closure_report.json`
  over child runtime report
  `artifacts/server_sim_closure/large_loop_physical_rolling_same_source/inspection-loop-video-20260521_161429/report.json`.
  It proves same-source world/tomogram artifacts, `physical_rolling` scan time,
  a written video (`video_frame_count=1801`), PCT global planning
  (`global_path_count=2`), local planning (`local_path_count=1131`), and
  nonzero navigation commands. It still fails the acceptance gate after the
  900 s wall guard with only one of four checkpoints reached. The gate now
  classifies a timeout with live local paths and cmd_vel as
  `planning_tracking`, not `validation_harness`, while true gate exceptions
  still remain `validation_harness`.
- The large-loop launcher default in `server_sim_closure` now uses explicit
  retest speed controls for this gate:
  `LINGTU_MUJOCO_LIVE_NAV_MAX_LINEAR_SPEED=0.45`,
  `LINGTU_MUJOCO_LIVE_CMD_VEL_LINEAR_LIMIT=0.45`, and
  `LINGTU_MUJOCO_LIVE_CMD_VEL_LINEAR_ACCEL_LIMIT=0.8`, all still overridable by
  environment variables. This does not change acceptance thresholds; it tests
  the hypothesis that the same-source run was progress-limited rather than
  only SLAM-limited.
- The benchmark summary must therefore keep `missing_or_failed` containing
  both `large_loop_closure` and `moving_obstacle_sweep`, and
  `algorithm_validation.claim_allowed` must be `false`.
- The current large-loop control matrix is also red:
  `artifacts/server_sim_closure/diagnosis_matrix/summary.json`.
  It now reports `slam_localization` as the blocking failure:
  - Static large-terrain Fast-LIO control is green
    (`z_delta_error_m=0.0146`), so startup alone is not the blocker.
  - 0.05 m/s fixed-motion Fast-LIO control is green
    (`sim_moved_m=0.9472`, `z_delta_error_m=0.0295`).
  - 0.10, 0.15, and 0.25 m/s controls are red. The 0.25 m/s control fails
    hard with `z_delta_error_m=31.5056`, yaw error `1.8064rad`, and motion
    divergence.
  - The best current 0.25 m/s Fast-LIO tuning control improves the red item
    but does not clear it (`z_delta_error_m=1.1448` with
    `lidar_filter_num=2`, `near_search_num=8`, and `ieskf_max_iter=8`).
  - A conservative full-chain PCT/Fast-LIO/local-planner control is green for
    a 20 s low-speed segment:
    `artifacts/server_sim_closure/diagnosis_matrix/large_loop_variants/fastlio_pct_conservative_v005_20s/inspection-loop-video-20260521_084605/report.json`.
    It proves the realtime chain is wired (`nav_data_source=fastlio2`,
    `global_path_count=1`, `local_path_count=191`, no runtime faults), but it
    uses `min_required_checkpoints=0`, so it is not a loop-closure pass.
  - Latest route-level scan-time A/B narrows the input-model issue:
    `instantaneous` fails almost immediately with motion/Z blow-up
    (`fastlio2_moved_m=144.7474`, `sim_moved_m=0.1669`,
    `z_delta_error_m=78.9128`), while `synthetic_rolling` removes that early
    Z/motion failure (`fastlio2_moved_m=1.4478`, `sim_moved_m=0.8890`,
    `z_delta_error_m=0.3416`) but still fails on yaw drift and patrol timeout.
    The validation launcher now defaults to `physical_rolling`; the older
    `synthetic_rolling` result remains a diagnostic ablation, not an acceptance
    pass, because `large_loop_closure` remains red.
  - The logs still show Fast-LIO degeneracy and IEKF non-convergence warnings,
    so the current blocker remains SLAM/localization robustness plus closed-loop
    yaw/path-following behavior, not PCT global planning or local path
    generation alone.
- The Fast-LIO speed/config diagnostic boundary is now explicit:
  `artifacts/server_sim_closure/diagnosis_matrix/fastlio_speed_boundary/report.json`.
  It is a diagnostic-only gate (`algorithm_pass=false`,
  `claim_allowed=false`), not an acceptance gate. Current fixed-drive evidence
  has `green_speed_mps=0.05`, `first_red_speed_mps=0.10`, and red speeds
  `[0.10, 0.15, 0.25]`. The minimal red defect is classified as
  `slam_localization` with time-aligned motion/Z drift at 0.10 m/s. The best
  known tuning control at 0.25 m/s improves the error but remains red.
  The diagnostic case schema now keeps `linear_y` and planar speed, which is
  required for the next PCT command-shape matrix.
- The refined speed/scan/tuning boundary is also diagnostic-only:
  `artifacts/server_sim_closure/diagnosis_matrix/fastlio_speed_boundary/refined_report.json`.
  It aggregates the older coarse controls plus refined 0.060/0.075/0.090/0.100
  m/s controls, instantaneous-vs-rolling scan timing controls, and the best
  known tuned control. A later route-level A/B made `synthetic_rolling` the
  interim MuJoCo MID-360 default, and that has since been superseded by
  `physical_rolling`.
  It reports `boundary_characterized=true`,
  `claim_allowed=false`, `fixed_control_count=21`, and
  `state="non_monotonic_or_unstable"`. The lowest reproduced red speed is
  0.06 m/s with `z_delta_error_m=1.4906`; because other historical controls
  include green runs up to 0.15 m/s, this is an instability boundary rather
  than a clean monotonic speed limit. This section predates the
  `physical_rolling` default and remains useful as a diagnostic boundary, not
  as current acceptance evidence.
- The yaw-rate diagnostic boundary is now explicit:
  `artifacts/server_sim_closure/diagnosis_matrix/yaw_rate_boundary/summary_vx_wz_report.json`.
  It keeps straight speed and turn-rate cases separate. The current summary has
  four fixed controls. Straight `vx=0.10,wz=0.0` is a `slam_localization` red
  item (`z_delta_error_m=2.2031`). Turning has one SLAM red item,
  `vx=0.05,wz=0.25` (`z_delta_error_m=1.2398`), while spin
  `vx=0.0,wz=0.25` and sharper turn `vx=0.10,wz=0.45` are classified as
  `validation_runtime` because Fast-LIO Z/yaw passed but the run hit the
  wall-time guard. This diagnostic report cannot green-light the benchmark; it
  narrows the current large-loop blocker.
- The time-offset diagnostic boundary is also red. The fixed `vx=0.10,wz=0.0`
  controls show `time_diff_lidar_to_imu=+0.010` improves Z drift relative to
  `0.0`, while `-0.010` and `+0.020` are worse. The best combined
  `+0.010` plus tuned Fast-LIO control is still above threshold
  (`z_delta_error_m=1.7469`), so time offset is not sufficient to close
  `large_loop_closure`.
- The MID-360 point-density diagnostic is the strongest new root-cause clue.
  On the same fixed `vx=0.10,wz=0.0` control, 12000 and 15000 samples per frame
  pass for 20.02 s sim time with an expanded wall-time guard, while 18000 and
  the original 24000-sample profile are red. Summary:
  `artifacts/server_sim_closure/diagnosis_matrix/point_density_boundary/summary.json`.
  This remains diagnostic-only; reducing samples is not an acceptance shortcut
  until the same profile passes the large-loop gate and the validation profile
  decision is documented.
- The first route-level 15k-sample transfer check is red:
  `artifacts/server_sim_closure/large_loop_samples_15000_60s/large_loop_closure_report.json`.
  The route had PCT and local planning active (`global_planner=pct`,
  `local_path_count=207`, `nav_cmd_vel_nonzero=218`) and no wall timeout, but
  Fast-LIO diverged after roughly 20 s (`fastlio2_moved_m=1226.861` versus
  `sim_moved_m=3.0639`, `z_delta_error_m=594.5294`). The fixed-control green
  window therefore does not close the DimOS benchmark gap by itself; the next
  matrix needs command-shape controls that reproduce PCT yaw/lateral behavior
  before changing acceptance thresholds.
- The first 15k-sample command-shape matrix has now reproduced a smaller
  failing boundary:
  `artifacts/server_sim_closure/diagnosis_matrix/command_shape_boundary_summary.json`.
  It aggregates six fixed-control reports. Low-speed yaw/lateral cases pass,
  and `vx=0.25,vy=0.0,wz=0.0` passes as a high-speed straight control. The
  first red turning SLAM case is `vx=0.25,vy=0.0,wz=0.45`
  (`z_delta_error_m=26.0378`, `yaw_delta_error_rad=1.2663`,
  `fastlio2_moved_m=104.0555` versus `sim_moved_m=0.4910`). Adding
  `vy=0.08` at the same `vx/wz` is also red, but only by Z drift
  (`z_delta_error_m=1.2869`) with yaw/motion checks still green. This keeps the
  DimOS benchmark red while narrowing the next optimization to Fast-LIO
  robustness under high forward speed plus high yaw rate.
- The first tuned command-shape matrix is still diagnostic-only:
  `artifacts/server_sim_closure/diagnosis_matrix/command_shape_tuned_summary.json`.
  With 15k MID-360 samples, `time_diff_lidar_to_imu=+0.010`, and the best
  current Fast-LIO tuning, the lateral turn
  `vx=0.25,vy=0.08,wz=0.45` becomes green (`z_delta_error_m=0.1027`,
  `yaw_delta_error_rad=0.1629`). The no-lateral turn at the same speed remains
  red by Z drift (`vx=0.25,vy=0.0,wz=0.45`,
  `z_delta_error_m=2.5252`). Lower yaw rates are also not clean:
  `wz=0.35` is yaw-red and `wz=0.25` is Z-red. This improves the failure mode
  but does not close `large_loop_closure`.
- Live MuJoCo/Fast-LIO reports now carry
  `fastlio_large_loop_diagnostic_report` as diagnostic evidence. This report is
  not an acceptance signal; it preserves the evidence needed to debug a red
  large-loop result: segment consistency, IMU statistics, scan timing
  statistics, and command trajectory statistics. The current instrumentation
  smoke artifact is
  `artifacts/server_sim_closure/diagnosis_matrix/diagnostic_instrumentation_smoke/report.json`.

Without `--run-missing`, `server_sim_closure.py` only evaluates existing
artifacts and returns the ordered missing gate commands in
`missing_required_commands`. With `--run-missing`, it executes missing or stale
required gates in preset order, records `gate_runs`, and then summarizes again.

The strict summary now includes `algorithm_validation.validation_flow`. That
flow is the review surface for the whole algorithm: map asset, static global
planning, local dynamic avoidance, realtime Fast-LIO mapping/localization,
long-range loop closure, saved-map lifecycle, command safety, and ROS
integration. A green child gate cannot override a red stage. The claim boundary
also states that PCT global planning uses static saved map/tomogram artifacts,
while live costmap and moving obstacles are local-planning and safety inputs.

Dynamic sweep command shape:

```bash
PYTHONPATH=src:. python3 sim/scripts/moving_obstacle_sweep_gate.py \
  --run-matrix \
  --child-run-root artifacts/server_sim_closure/moving_obstacle_sweep/children \
  --inspection-tomogram artifacts/server_sim_closure/large_terrain_odom/tomogram.pickle \
  --report-glob 'artifacts/server_sim_closure/mujoco_fastlio2_live*/inspection*/*/report.json' \
  --required-speed-bins slow,fast \
  --required-density-bins sparse,dense \
  --required-scan-time-profile physical_rolling \
  --require-video-file \
  --json-out artifacts/server_sim_closure/moving_obstacle_sweep/report.json \
  --strict
```

Large loop command shape:

```bash
bash sim/scripts/launch_mujoco_fastlio2_live.sh inspection-loop-video

PYTHONPATH=src:. python3 sim/scripts/large_loop_closure_gate.py \
  --report artifacts/server_sim_closure/mujoco_fastlio2_live/<inspection-loop-run>/report.json \
  --required-scan-time-profile physical_rolling \
  --require-video-file \
  --json-out artifacts/server_sim_closure/large_loop_closure/report.json \
  --strict
```

The saved-map relocalize gate now keeps the runtime map selection on
TARE/exploration same-source artifacts before generic recent MuJoCo inspection
maps. The refreshed report uses live MuJoCo MID-360/IMU through Fast-LIO,
`/nav/relocalize`, and localizer health:

- `artifacts/server_sim_closure/saved_map_relocalize_runtime/report.json`
- `localizer.latest_health_state=LOCKED`, `tracking_health_samples=485`,
  `lost_health_samples=0`.
- Live Fast-LIO consistency passed:
  `sim_moved_m=3.294`, `fastlio2_moved_m=3.317`,
  `z_delta_error_m=0.0189`.

The saved-map PCT gate now uses the tomogram next to the relocalized
`map.pcd`, passes that directory as `--map-root`, and uses a bounded saved-map
goal. Refreshed evidence:

- `artifacts/server_sim_closure/pct_saved_map_navigation/report.json`
- PCT selected with no fallback; native local planner/path follower reached
  the goal.
- `moved_m=3.797`, `final_distance_m=0.494`,
  `trajectory_quality.final_progress_ratio=0.902`.

The saved-map runtime gate now reads `same_source_map/metadata.json` for the
MuJoCo world and scan-time profile instead of defaulting to an unrelated
scene. New same-source map artifacts also record `scan_time_profile`,
`nav_data_source`, and `fastlio_lidar_input`.

## Acceptance Boundary

Passing the current `dimos_benchmark` means LingTu has current
simulation/replay-style evidence across realtime mapping/localization, PCT
global planning, local avoidance, dynamic-obstacle speed/density coverage,
large-loop drift bounds, frontier smoke, saved-map navigation, and command
safety. It still does not prove physical gait robustness, real MID-360 timing
on the robot, or field SLAM quality.
