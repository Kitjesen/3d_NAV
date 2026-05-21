# Gazebo Deliverable Validation

This page records the Gazebo validation boundary for product delivery claims.
It is intentionally stricter than a ROS topic smoke test: the gate must prove a
closed navigation loop in Gazebo before navigation is described as Gazebo-ready.

## Product Claim

Passed claim:

- ROS-native Gazebo runtime starts headlessly on the server.
- Odometry publishes `odom -> body`.
- Simulated registered and map point clouds publish in the expected frames.
- A navigation goal is published.
- The Gazebo validation planner publishes `/nav/global_path` from the
  LiDAR-derived occupancy grid. This Gazebo gate currently uses the grid/A*
  line planner path, not PCT.
- Local planning publishes `/nav/local_path`.
- The stack publishes non-zero `/nav/cmd_vel`.
- Gazebo odometry changes by at least `0.05 m`.
- Wavefront frontier exploration starts in Gazebo validation.
- Frontier goals are produced and forwarded to `/nav/goal_pose`.
- Frontier-driven navigation publishes global path, local path, and non-zero
  `cmd_vel`.
- Frontier-driven Gazebo odometry moves by at least `0.05 m`.
- Exploration coverage grows during the Gazebo run from a Gazebo LiDAR-derived
  2D occupancy grid, not from a synthetic test-only coverage grid.
- Frontier goals are selected from the real unknown/free boundary of that
  LiDAR-derived occupancy grid.
- The Gazebo runtime publishes `/nav/cumulative_map_cloud` and the gate verifies
  cumulative point/voxel growth, retention against prior samples, negative
  control against `/nav/registered_cloud`, and static obstacle centroid
  stability. The cumulative map filters Gazebo max-range/no-hit returns with
  local range and height bounds before accumulation, so RViz diagnostics do not
  present invalid sky/far-field points as explored map.
- A Gazebo + RViz diagnostic recording shows the enclosed room/obstacle scene,
  the LiDAR-derived occupancy grid, paths, frontier-driven motion evidence, and
  RViz global status `Ok`. RViz keeps cumulative debug clouds disabled by
  default so invalid/far-field accumulation cannot be mistaken for the map.
- The run is simulation-only: no real robot motion and no hardware `cmd_vel`.

Not yet claimed:

- TARE Gazebo runtime planning. The current gate checks the TARE source,
  launch, bridge, and supervisor contract; it does not require a built
  `tare_planner_node` binary unless `--require-tare-runtime` is set.
- SLAM-quality mapping. `/nav/map_cloud` remains the filtered live
  odom-projected Gazebo LiDAR cloud used by terrain and local planning. The
  frontier gate raytraces that cloud into a 2D occupancy grid for Gazebo
  validation. `/nav/cumulative_map_cloud` is a Gazebo-only accumulated
  diagnostic cloud, not a replacement for Fast-LIO2/Super-LIO field SLAM.
- PCT planning inside the Gazebo frontier gate. PCT is covered by the separate
  native PCT MuJoCo no-fallback gate, not by this Gazebo report.
- S100P field validation from this Gazebo run.

Wavefront frontier exploration is now Gazebo-validated through the runtime gate.
TARE remains a separate backend and should be described as integrated but not
Gazebo-runtime-validated until a server with the TARE binary passes a TARE
runtime gate.

## Gate Command

Run from a ROS 2 Humble workspace where the Gazebo simulation packages are built:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
DOMAIN=${ROS_DOMAIN_ID:-30}
PART=lingtu_grid_astar_odomfoot_$(date +%s)
PYTHONPATH=src:.:/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages:$PYTHONPATH \
  python3 sim/scripts/gazebo_runtime_gate.py \
    --check-nav-loop \
    --check-frontier-exploration \
    --check-cumulative-map \
    --check-tare-contract \
    --warmup-sec 20 \
    --smoke-timeout-sec 20 \
    --nav-timeout-sec 35 \
    --nav-gazebo-warmup-sec 10 \
    --nav-warmup-sec 0.0 \
    --nav-goal-delay-sec 0.0 \
    --nav-goal-republish-sec 0.5 \
    --frontier-timeout-sec 60 \
    --frontier-continue-after-pass-sec 5 \
    --spawn-x 0.0 \
    --spawn-y 0.0 \
    --nav-goal-x 3.0 \
    --nav-goal-y 1.0 \
    --nav-goal-z 0.0 \
    --ros-domain-id "$DOMAIN" \
    --gz-partition "$PART" \
    --frontier-trace-out artifacts/server_sim_closure/gazebo_runtime_explore/frontier_trace_grid_astar_odomfoot.json \
    --json-out artifacts/server_sim_closure/gazebo_runtime_explore/report_grid_astar_odomfoot.json \
    --launch-log artifacts/server_sim_closure/gazebo_runtime_explore/launch_grid_astar_odomfoot.log
```

The same command is embedded in `sim/scripts/server_sim_closure.py` as the
`gazebo_runtime` product gate.

## Server Evidence

Latest product gate evidence (2026-05-14):

- Host: `gpu8x3090`
- Workspace: `/home/bsrl/hongsenpang/lingtu_closure_22f127b2`
- Gate report:
  `artifacts/server_sim_closure/gazebo_runtime_explore/report_grid_astar_odomfoot.json`
- Frontier trace:
  `artifacts/server_sim_closure/gazebo_runtime_explore/frontier_trace_grid_astar_odomfoot.json`
- Closure summary:
  `artifacts/server_sim_closure/summary_default_latest_gazebo.json`
- Launch log:
  `artifacts/server_sim_closure/gazebo_runtime_explore/launch_grid_astar_odomfoot.log`
- Result: `ok=true`, `simulation_only=true`, `real_robot_motion=false`,
  `cmd_vel_sent_to_hardware=false`.

Observed evidence from the latest passing report:

- Navigation loop: `global_path_seen=true`, `local_path_seen=true`,
  `cmd_vel_nonzero=true`, `odom_delta_m=0.0965`, and
  `odom_delta_x_m=0.0923`.
- Frontier exploration: `frontier_goal_count=2`, `frontier_goal_in_room=true`,
  `frontier_count_max=7`, `known_cells_delta=939`, and
  `explored_area_delta_m2=9.39`.
- Frontier-driven motion: `odom_delta_m=1.9403`,
  `odom_delta_x_m=1.7418`, and `odom_path_length_m=1.966`.
- Topic and frame sync: `/nav/registered_cloud` in `body`,
  `/nav/map_cloud` in `odom`, `/nav/terrain_map` in `odom`,
  `/nav/terrain_map_ext` in `odom`, with
  `max_cloud_odom_skew_ms=0.0`.
- Cumulative diagnostic cloud: samples `47`, point growth
  `1447 -> 17277`, unique voxel growth `339 -> 2910`, retention minimum
  `0.9941`, and `/nav/cumulative_map_cloud` versus
  `/nav/registered_cloud` voxel ratio `7.4807`.
- Trajectory quality: `room_violation_count=0`,
  `local_path_occupied_overlap_count=0`,
  `local_path_unknown_ratio_max=0.1881`, and
  `min_obstacle_clearance_m=0.3206`.
- Static obstacle stability: block, column, left wall, and right wall were all
  observed as stable ROIs.
- TARE contract: source/launch/bridge/supervisor present; runtime Gazebo
  execution still not claimed.

Previous GUI/video evidence:

- Host: `gpu8x3090`
- Workspace: `/home/bsrl/hongsenpang/lingtu_closure_22f127b2`
- Gate report:
  `artifacts/server_sim_closure/gazebo_runtime_room_full_v16/report.json`
- Frontier trace:
  `artifacts/server_sim_closure/gazebo_runtime_room_full_v16/frontier_trace.json`
- Evidence video:
  `artifacts/server_sim_closure/gazebo_runtime_room_full_v16/gazebo_frontier_lidar_occupancy_v16.mp4`
- Gazebo + RViz recording:
  `artifacts/server_sim_closure/gazebo_rviz_product_scene_v17/gazebo_rviz_frontier_v17.mp4`
- Gazebo + RViz report:
  `artifacts/server_sim_closure/gazebo_rviz_product_scene_v17/report.json`
- Result: `ok=true`, `simulation_only=true`, `real_robot_motion=false`,
  `cmd_vel_sent_to_hardware=false`

Observed evidence from the latest passing report:

- Source of exploration costmap: `gazebo_lidar_derived`
- Source of map growth: `gazebo_lidar_raytrace`
- LiDAR map updates: `78`; raytrace updates: `2043`
- Known cells: `460 -> 1464`, delta `1004`
- Explored area: `4.60 m^2 -> 14.64 m^2`, delta `10.04 m^2`
- Final free cells: `98`; final occupied cells: `1366`
- Frontier goals published: `7`; selected frontier goal stayed inside the room
- Frontier-driven motion: `odom_delta_m=4.0338`
- Path height contract: `global_path_z_range=[0.0, 0.0]`,
  `local_path_z_range=[0.0, 0.0]`
- Point-cloud frames: `/nav/registered_cloud` in `body`,
  `/nav/map_cloud` in `odom`
- Cumulative diagnostic cloud: samples `79`, point growth
  `15003 -> 61197`, unique voxel growth `2482 -> 8499`, retention minimum
  `0.9978`
- Static obstacle drift tail-window maximum: block `0.0059 m`, column
  `0.0 m`, left wall `0.0504 m`, right wall `0.233 m`
- TARE contract: source/launch/bridge/supervisor present; runtime Gazebo
  execution still not claimed.

The v17 Gazebo + RViz recording repeats the visible GUI validation with the
same real LiDAR-derived mapping path. Its frontier report passed with
`lidar_map_updates=101`, `raytrace_updates=3532`, `known_cells_delta=1118`,
`explored_area_delta_m2=11.18`, `odom_delta_m=3.9581`, and zero hardware
output.

Historical server run:

- Host: `gpu8x3090`
- Workspace: `/home/bsrl/hongsenpang/lingtu_closure_22f127b2`
- Report: `artifacts/server_sim_closure/gazebo_runtime_explore/report.json`
- Summary: `artifacts/server_sim_closure/gazebo_runtime_explore/summary.json`
- Launch log: `artifacts/server_sim_closure/gazebo_runtime_explore/launch.log`
- Result: `ok=true`

Observed evidence from the passing report:

- `/nav/map_cloud`: frame `odom`, `2269` points
- `/nav/registered_cloud`: frame `body`, `23040` points
- Navigation loop: `goal_published=true`, `global_path_seen=true`,
  `local_path_seen=true`, `cmd_vel_seen=true`, `cmd_vel_nonzero=true`
- Navigation topic samples: `/nav/global_path=1`, `/nav/local_path=32`,
  `/nav/cmd_vel=108`, `/nav/odometry=29`
- Navigation odometry moved `odom_delta_m=1.4535`
- Frontier exploration: `frontier_started=true`,
  `frontier_goal_published=true`, `global_path_seen=true`,
  `local_path_seen=true`, `cmd_vel_nonzero=true`
- Frontier topic samples: `/nav/goal_pose=10`, `/nav/global_path=3`,
  `/nav/local_path=64`, `/nav/cmd_vel=197`, `/nav/odometry=61`
- Frontier odometry moved `odom_delta_m=0.1763`
- Exploration coverage grew by `23` known cells / `0.23 m^2`
- TARE contract: `source_contract_ok=true`, `runtime_available=false`,
  `gazebo_runtime_verified=false`
- Safety boundary: `simulation_only=true`, `real_robot_motion=false`,
  `cmd_vel_sent_to_hardware=false`

Additional recorded demo evidence:

- Video: `artifacts/server_sim_closure/gazebo_frontier_video_long/gazebo_frontier_exploration_long.mp4`
- Video summary: `frames=44`, `duration_sec=5.5`, `frontier_ok=true`
- Long frontier run: `odom_delta_m=5.1354`, `known_cells_delta=1420`,
  `explored_area_delta_m2=14.2`
- Gazebo + RViz video:
  `artifacts/server_sim_closure/gazebo_rviz_obstacle_recording5/gazebo_rviz_frontier.mp4`
- Gazebo + RViz report:
  `artifacts/server_sim_closure/gazebo_rviz_obstacle_recording5/report.json`
- Gazebo + RViz frame evidence:
  `artifacts/server_sim_closure/gazebo_rviz_obstacle_recording5/frame_100.png`
  shows visible Gazebo obstacles and RViz `Global Status: Ok` with
  `/nav/map_cloud`; `frame_260.png` shows path, goal/odometry markers, and the
  live odom-projected cloud during the frontier run.
- Gazebo + RViz run: `/nav/map_cloud=12798` points,
  `/nav/registered_cloud=23040` points, frontier goal
  `[0.7366, -0.3315, 0.0]`, `odom_delta_m=2.9651`,
  `known_cells_delta=1547`, `explored_area_delta_m2=15.47`.
- Product-scene cumulative-map gate report:
  `artifacts/server_sim_closure/gazebo_runtime_product_scene/report.json`.
  It passed with `ok=true`, `/nav/cumulative_map_cloud` samples `85`, frame
  `odom`, point growth `8034 -> 24334`, unique voxel growth
  `1669 -> 6575`, retention minimum `0.9976`, and
  `/nav/cumulative_map_cloud` versus `/nav/registered_cloud` voxel ratio
  `4.0662`. During the same run, frontier exploration moved
  `odom_delta_m=2.7032`, grew exploration by `1566` known cells /
  `15.66 m^2`, and kept static obstacle centroid drift at or below
  `0.0323 m` for the original wall/block/column ROIs. The nav-loop subgate
  also checks forward-direction consistency for a positive-x goal:
  `cmd_vel_linear_x_max=0.4284` and `odom_delta_x_m=0.0803`.
- Product-scene Gazebo + RViz recording:
  `artifacts/server_sim_closure/gazebo_rviz_product_scene/gazebo_rviz_product_scene_frontier.mp4`.
  It contains `46` frames at `2 FPS`; frame
  `artifacts/server_sim_closure/gazebo_rviz_product_scene/frame_035.png`
  shows the enriched Gazebo corridor/obstacle scene plus RViz filtered
  `CumulativeMapCloud`, live `RegisteredCloud`, global/local paths, goal, and
  robot marker without the previous ground-plane-heavy point-cloud display or
  odometry-arrow clutter.

Additional topic synchronization evidence:

- Report: `artifacts/server_sim_closure/gazebo_topic_sync/report.json`
- `/nav/odometry`: frame `odom`, child frame `body`, monotonic stamps
- `/nav/map_cloud`: frame `odom`, monotonic stamps, non-zero point cloud
- `/nav/registered_cloud`: frame `body`, monotonic stamps, non-zero point cloud
- Nearest point-cloud-to-odometry timestamp skew: `p95=0.03 s`,
  `max=0.16 s`

## Delivery Decision

Gazebo navigation and wavefront frontier exploration are now deliverable
validation gates, not just launch smoke tests. A release can cite these Gazebo
capabilities only when this gate passes and the server simulation closure
summary marks `gazebo_runtime` as verified.

TARE remains the next exploration deliverability gap. The expected MVP for that
gap is a TARE runtime gate that launches the built `tare_planner_node`, observes
TARE waypoints/path/runtime diagnostics, forwards at least one TARE waypoint into
the navigation stack, and confirms safe non-zero motion in Gazebo simulation
only.
