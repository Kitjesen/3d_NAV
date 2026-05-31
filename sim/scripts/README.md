# sim/scripts Index

This directory is a stable script contract, not a package boundary. Many
profiles, tests, and evidence gates refer to `sim/scripts/<name>` directly, so
scripts stay in place and are grouped by naming convention.

## Safety Classes

Use the safety class before running a script:

| Safety class | Meaning | Examples |
| --- | --- | --- |
| summary-only unless --run-missing | Reads existing reports and writes an aggregate summary. It must not launch missing gates unless `--run-missing` is passed. Use `--host-preflight` to check host suitability without gate execution; use `--json-out -` for stdout-only reporting. | `server_sim_closure.py` |
| local non-motion | Runs local Python checks, asset generation, or in-memory module dataflow. It must report `real_robot_motion=false` and `cmd_vel_sent_to_hardware=false`. | `multifloor_nav_validation.py --skip-mujoco`, `large_terrain_nav_validation.py`, `routecheck_preflight_gate.py` |
| simulated motion only | May move a MuJoCo/Gazebo/Unity simulated robot. It must stay disconnected from physical robot drivers and hardware command subscribers. | `policy_nav_smoke.py`, `native_pct_mujoco_gate.py`, `mujoco_fastlio2_live_gate.py` |
| ROS2 isolated simulation | May source ROS 2, launch sim nodes, or publish sim topics. Use an isolated `ROS_DOMAIN_ID`; never run on a robot ROS domain. | `gazebo_runtime_gate.py`, `launch_mujoco_fastlio2_live.sh`, `launch_lingtu_gazebo_industrial_demo.sh` |
| legacy manual | Historical helpers or dataset scripts. They can source install spaces, start subprocesses, or assume local assets; they are not part of the G4 closure unless another gate explicitly consumes their report. | `_run_legkilo_test.sh`, `run_legkilo_test.sh`, `test_*.sh`, legacy Go1 demos |

## Gate Scripts

- `server_sim_closure.py` - G4 evidence aggregator; summary-only unless --run-missing. `--host-preflight` reports whether the current host can safely run the selected gates without launching them.
- `routecheck_preflight_gate.py` - Gateway route preflight with no goal or cmd_vel publication.
- `gateway_goal_dry_run_gate.py` - Gateway dry-run goal contract.
- `dynamic_obstacle_local_planner_gate.py` - Dynamic-obstacle local planner gate.
- `large_loop_closure_gate.py` - Large-loop closure report validator.
- `moving_obstacle_sweep_gate.py` - Moving-obstacle sweep report validator.
- `fastlio2_rosbag_replay_gate.py` - Fast-LIO2 rosbag replay gate.
- `fastlio_speed_boundary_gate.py` - Fast-LIO speed-boundary gate.
- `mujoco_fastlio2_live_gate.py` - MuJoCo live LiDAR/IMU plus Fast-LIO2 simulation gate.
- `native_pct_mujoco_gate.py` - Native PCT plus ROS2 local planner/path follower into MuJoCo simulation.
- `gazebo_runtime_gate.py` - Gazebo runtime simulation gate; requires ROS2 isolated simulation.
- `pct_saved_map_navigation_gate.py` - Saved-map PCT navigation gate.
- `saved_map_relocalize_contract_gate.py` / `saved_map_relocalize_runtime_gate.py` - Saved-map relocalization gates.
- `cmu_unity_runtime_gate.py` / `cmu_unity_sim_gate.py` - CMU Unity runtime and contract gates.

## Validation And Diagnosis

- `multifloor_nav_validation.py` - Multi-floor navigation validation. Default safe mode is local non-motion; `--bridge-loop` changes it to simulated motion only.
- `large_terrain_nav_validation.py` - Large-terrain global-planning asset validation. It does not exercise local planner or path follower backends and reports those algorithm surfaces as `not_exercised`.
- `full_sim_validation.py` - Compatibility wrapper for the full simulation validation gate; canonical G4 closure aggregation is `server_sim_closure.py`.
- `large_loop_diagnosis_matrix.py` - Large-loop diagnosis matrix.
- `render_slam_validation_screenshots.py` - SLAM validation screenshot renderer.
- `run_slam_dataset_test_v2.py` - SLAM dataset test runner.
- `cmu_unity_tomogram_capture.py` - CMU Unity tomogram capture helper.
- `run_global_planner.py` - Legacy ROS launch compatibility wrapper for `sim/launch/sim.launch.py`. It requires an isolated nonzero `ROS_DOMAIN_ID` and must not be used as the current G4 planner evidence source.

## Demo And Runtime Entrypoints

- `run_sim.py` - Generic simulation launcher.
- `run_person_following.py` - Person-following simulation launcher.
- `run_semantic_full_stack.py` - Semantic full-stack simulation launcher.
- `demo_search.py` - Search demo.
- `policy_nav_smoke.py` - Policy-mode navigation smoke test; simulated motion only.
- `cmu_unity_lingtu_stack.py` - CMU Unity LingTu stack helper for controlled simulation experiments; not the default product runtime.
- `nav_overlay.py` - Navigation overlay visualization.
- `view_scene.py` - Scene viewer.
- `benchmark_following.py` - Person-following benchmark.
- `record_policy_nav_video.py` / `render_gazebo_frontier_video.py` - Video recording/rendering helpers.
- `go1_indoor_nav.py` / `go1_nav_full.py` - legacy Go1 demos; require optional `sim/robots/go1_playground/` assets and are not part of current G4 closure.

## Shell Launchers And Legacy Helpers

- `launch_mujoco_fastlio2_live.sh` - MuJoCo + Fast-LIO2 live simulation launcher for the `sim_mujoco_live` contract. Use ROS2 isolated simulation.
- `launch_cmu_unity_lingtu_runtime.sh` / `launch_cmu_unity_baseline.sh` - CMU Unity runtime and baseline launchers.
- `launch_lingtu_gazebo_industrial_demo.sh` - Gazebo industrial simulation demo launcher.
- `_run_legkilo_test.sh` / `run_legkilo_test.sh` - legacy/manual dataset helper scripts. They may source ROS/install spaces, start SLAM dataset processes, or clean up external processes; do not include them in the G4 server closure.
- `test_*.sh` - integration smoke helpers. Treat as legacy manual unless a gate documents a stricter contract.
- `install_deps.sh` - optional dependency installer.
- `fastlio_speed_scan_plan.sh` - speed scan helper.

## Tooling

- `algorithm_dataflow_summary.py` - Algorithm dataflow summary used by tests.
- `rosbag_slam_bridge_replay.py` - Raw rosbag to `SlamBridgeModule` replay.
