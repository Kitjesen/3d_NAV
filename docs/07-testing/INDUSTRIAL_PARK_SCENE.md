# LingTu Industrial Park Gazebo Scene

## Decision

LingTu owns the product demo scene. CMU Unity remains an external exploration
benchmark, not the default customer-visible scene.

The first-class Gazebo industrial-park scene is:

```text
sim/worlds/lingtu_gazebo_industrial_park.sdf
```

It models a small outdoor factory park: perimeter fencing, west gate, main
asphalt road, cross road, warehouse, factory hall, loading dock, containers,
pipe racks, pallets, tanks, and guard posts. The robot spawns on the main road
and should explore inside the fenced site rather than driving away from the
environment.

## Server View Command

Use a ROS domain under 100. High domain IDs can make Fast DDS fail with
`Calculated port number is too high`.

```bash
cd /home/bsrl/hongsenpang/lingtu_closure_22f127b2
source /opt/ros/humble/setup.bash
[ -f install/setup.bash ] && source install/setup.bash

export PYTHONPATH=src:.:/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages:$PYTHONPATH
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
export ROS_DOMAIN_ID=42
export GZ_PARTITION=lingtu_industrial_watch_$(date +%s)

/usr/bin/python3 sim/scripts/gazebo_runtime_gate.py \
  --no-headless \
  --world sim/worlds/lingtu_gazebo_industrial_park.sdf \
  --spawn-x 0.0 \
  --spawn-y 0.0 \
  --check-nav-loop \
  --frontier-coverage-size-m 34 \
  --frontier-room-min-x -7.8 \
  --frontier-room-max-x 23.8 \
  --frontier-room-min-y -8.8 \
  --frontier-room-max-y 8.8 \
  --frontier-max-trajectory-abs-y-m 8.4
```

Use `/usr/bin/python3` for ROS gates on the server. The conda/base `python3`
may be Python 3.13 and cannot load Humble's `rclpy` extension.

## Deliverable Gate

This gate proves the product chain that is safe to claim today:

```text
Gazebo industrial park -> simulated LiDAR -> LiDAR-derived occupancy/frontier
-> cleaned occupancy-grid PCD -> tomogram -> offline PCT plan preview
```

```bash
cd /home/bsrl/hongsenpang/lingtu_closure_22f127b2
source /opt/ros/humble/setup.bash
[ -f install/setup.bash ] && source install/setup.bash || true

export PYTHONPATH=src:.:/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages:$PYTHONPATH
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
export ROS_DOMAIN_ID=42
export GZ_PARTITION=lingtu_industrial_pct_$(date +%s)

/usr/bin/python3 sim/scripts/gazebo_runtime_gate.py \
  --world sim/worlds/lingtu_gazebo_industrial_park.sdf \
  --check-frontier-exploration \
  --check-cumulative-map \
  --check-explored-map-pct \
  --warmup-sec 45 \
  --smoke-timeout-sec 35 \
  --frontier-timeout-sec 90 \
  --nav-gazebo-warmup-sec 6 \
  --frontier-gazebo-warmup-sec 6 \
  --frontier-coverage-size-m 34 \
  --frontier-room-min-x -7.8 \
  --frontier-room-max-x 23.8 \
  --frontier-room-min-y -8.8 \
  --frontier-room-max-y 8.8 \
  --frontier-max-trajectory-abs-y-m 8.4 \
  --frontier-static-roi-preset industrial_park \
  --frontier-pcd-out artifacts/server_sim_closure/gazebo_runtime/industrial_clean_map.pcd \
  --frontier-tomogram-out artifacts/server_sim_closure/gazebo_runtime/industrial_clean_tomogram.pickle \
  --json-out artifacts/server_sim_closure/gazebo_runtime/industrial_clean_pct_report.json \
  --launch-log artifacts/server_sim_closure/gazebo_runtime/industrial_clean_pct_launch.log
```

## Map Artifact Policy

Do not use `/nav/cumulative_map_cloud` as a product map artifact. It is a raw
debug cloud and keeps MID-360 scan lines, vertical return bands, and transient
returns. Product PCD/tomogram artifacts must come from `/nav/exploration_grid`
through `LidarOccupancyGrid.navigation_artifact_points()`.

Latest corrected server evidence:

- Report: `artifacts/server_sim_closure/gazebo_runtime/industrial_clean_pct_report.json`
- Frontier exploration: passed; cleaned artifact source is `occupancy_grid`
  with source topic `/nav/exploration_grid`.
- Clean map artifact: PCD point count `5188`, tomogram shape `[5, 2, 64, 56]`.
- Occupancy grid at export: known cells `4644`, free cells `4372`, occupied
  cells `272`, resolution `0.1 m`.
- PCT strict preview: failed, because path safety rejected the route with
  `internal_free_route:path_safety_failed:30`.
- Hardware safety: simulation only; no real robot motion and no hardware
  `/cmd_vel` output.

Current status: mapping/frontier artifact generation is corrected, but the
industrial-park chain is not yet a complete autonomy claim. The remaining gate
is online PCT global path -> local planner -> path follower -> `/nav/cmd_vel`
in the same industrial world, with strict path safety passing on the generated
tomogram.

## Product Boundary

- Product demo and deterministic CI: LingTu Gazebo scenes.
- Realistic TARE benchmark: CMU Unity adapter and assets.
- Stress diversity: selected external worlds after license review.
