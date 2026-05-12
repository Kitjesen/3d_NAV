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
- The run is simulation-only: no real robot motion and no hardware `cmd_vel`.

Not yet claimed:

- Gazebo frontier exploration mapping as a product gate.
- A customer-facing Gazebo demo video or scripted exploratory coverage report.
- S100P field validation from this Gazebo run.

Exploration has separate module-level and kinematic simulation coverage in the
server simulation closure suite. Until a Gazebo exploration gate exists, do not
describe exploration mapping as Gazebo-validated.

## Gate Command

Run from a ROS 2 Humble workspace where the Gazebo simulation packages are built:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
PYTHONPATH=src:.:/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages:$PYTHONPATH \
  python3 sim/scripts/gazebo_runtime_gate.py \
    --check-nav-loop \
    --json-out artifacts/server_sim_closure/gazebo_runtime_nav/report.json \
    --launch-log artifacts/server_sim_closure/gazebo_runtime_nav/launch.log
```

The same command is embedded in `sim/scripts/server_sim_closure.py` as the
`gazebo_runtime` product gate.

## Server Evidence

Authoritative server run:

- Host: `gpu8x3090`
- Workspace: `/home/bsrl/hongsenpang/lingtu_closure_22f127b2`
- Report: `artifacts/server_sim_closure/gazebo_runtime_nav/report.json`
- Launch log: `artifacts/server_sim_closure/gazebo_runtime_nav/launch.log`
- Result: `ok=true`

Observed evidence from the passing report:

- `/nav/map_cloud`: frame `odom`, `11266` points
- `/nav/registered_cloud`: frame `body`, `23040` points
- Navigation loop: `goal_published=true`, `global_path_seen=true`,
  `local_path_seen=true`, `cmd_vel_seen=true`, `cmd_vel_nonzero=true`
- Topic samples: `/nav/global_path=4`, `/nav/local_path=62`,
  `/nav/cmd_vel=312`, `/nav/odometry=57`
- Odometry moved from approximately `[0.0, 0.0]` to
  `[0.0579, 0.0539]`, `odom_delta_m=0.0791`
- Safety boundary: `simulation_only=true`, `real_robot_motion=false`,
  `cmd_vel_sent_to_hardware=false`

## Delivery Decision

Gazebo navigation is now a deliverable validation gate, not just a launch smoke
test. A release can cite Gazebo navigation only when this gate passes and the
server simulation closure summary marks `gazebo_runtime` as verified.

Gazebo exploration remains the next deliverability gap. The expected MVP for
that gap is a Gazebo gate that launches frontier exploration, observes frontier
goal publication, verifies map growth or explored-area increase, and confirms
safe non-zero motion in simulation only.
