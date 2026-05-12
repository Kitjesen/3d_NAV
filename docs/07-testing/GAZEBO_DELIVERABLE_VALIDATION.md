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
- The run is simulation-only: no real robot motion and no hardware `cmd_vel`.

Not yet claimed:

- A customer-facing Gazebo demo video or scripted exploratory coverage report.
- TARE Gazebo runtime planning. The current gate checks the TARE source,
  launch, bridge, and supervisor contract; it does not require a built
  `tare_planner_node` binary unless `--require-tare-runtime` is set.
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
