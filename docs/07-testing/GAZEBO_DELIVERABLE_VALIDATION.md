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
- PCT global planning publishes `/nav/global_path`.
- Local planning publishes `/nav/local_path`.
- The stack publishes non-zero `/nav/cmd_vel`.
- Gazebo odometry changes by at least `0.05 m`.
- Wavefront frontier exploration starts in Gazebo validation.
- Frontier goals are produced and forwarded to `/nav/goal_pose`.
- Frontier-driven navigation publishes global path, local path, and non-zero
  `cmd_vel`.
- Frontier-driven Gazebo odometry moves by at least `0.05 m`.
- Exploration coverage grows during the Gazebo run.
- The Gazebo runtime publishes `/nav/cumulative_map_cloud` and the gate verifies
  cumulative point/voxel growth, retention against prior samples, negative
  control against `/nav/registered_cloud`, and static obstacle centroid
  stability.
- A Gazebo + RViz diagnostic recording shows visible obstacles, live
  odom-projected LiDAR cloud, paths, frontier-driven motion evidence, and RViz
  global status `Ok`.
- The run is simulation-only: no real robot motion and no hardware `cmd_vel`.

Not yet claimed:

- TARE Gazebo runtime planning. The current gate checks the TARE source,
  launch, bridge, and supervisor contract; it does not require a built
  `tare_planner_node` binary unless `--require-tare-runtime` is set.
- SLAM-quality mapping. `/nav/map_cloud` remains the live odom-projected LiDAR
  cloud used by terrain and local planning. `/nav/cumulative_map_cloud` is a
  Gazebo-only accumulated map used for validation and RViz diagnostics, not a
  replacement for Fast-LIO2/Super-LIO field SLAM.
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
PYTHONPATH=src:.:/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages:$PYTHONPATH \
  python3 sim/scripts/gazebo_runtime_gate.py \
    --check-nav-loop \
    --check-frontier-exploration \
    --check-cumulative-map \
    --check-tare-contract \
    --json-out artifacts/server_sim_closure/gazebo_runtime_nav/report.json \
    --launch-log artifacts/server_sim_closure/gazebo_runtime_nav/launch.log
```

The same command is embedded in `sim/scripts/server_sim_closure.py` as the
`gazebo_runtime` product gate.

## Server Evidence

Authoritative server run:

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
- Cumulative-map gate report:
  `artifacts/server_sim_closure/gazebo_runtime_cumulative3/report.json`.
  It passed with `ok=true`, `/nav/cumulative_map_cloud` samples `77`, frame
  `odom`, point growth `26842 -> 80000`, unique voxel growth
  `9008 -> 29380`, retention minimum `0.9782`, and
  `/nav/cumulative_map_cloud` versus `/nav/registered_cloud` voxel ratio
  `18.374`.
- Cumulative-map RViz recording:
  `artifacts/server_sim_closure/gazebo_rviz_cumulative_recording/gazebo_rviz_cumulative_frontier.mp4`.
  It contains `36` frames at `2 FPS`; frame
  `artifacts/server_sim_closure/gazebo_rviz_cumulative_recording/frame_035.png`
  shows Gazebo obstacles plus RViz `LiveOdomCloud` and
  `CumulativeMapCloud` enabled.

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
