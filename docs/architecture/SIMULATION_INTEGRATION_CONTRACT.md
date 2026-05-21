# Simulation Integration Contract

LingTu is the product system. Simulation environments are external inputs and
validation targets. They must not become product profiles, disable safety, or
replace the navigation chain with demo-specific shortcuts.

## Product Boundary

Product profiles are the field-facing LingTu stacks:

- `map`
- `nav`
- `explore`
- `tare_explore`
- `super_lio`
- `super_lio_relocation`

Simulation and development profiles are separate:

- `stub`
- `dev`
- `sim`
- `sim_mujoco_live`
- `sim_gazebo`
- `sim_industrial`
- `sim_cmu_tare`
- `sim_nav`

`sim_industrial` is the first customer-visible Gazebo delivery scene profile.
It uses the same Module graph boundary as `sim_gazebo`, but pins the industrial
yard world, filtered RViz view, frontier/map growth checks, and simulation-only
command relay contract.

CMU Unity is not a LingTu product profile. It is split into two explicit
contracts:

- `cmu_unity_baseline`: CMU's original Unity + TARE/FAR + terrain/local planner
  + path follower loop. This is reference evidence only.
- `cmu_unity_external`: LingTu's adapter/execution gate. CMU Unity owns the
  world and sensors, external TARE owns exploration waypoints, and LingTu owns
  the `/nav/*` execution chain and simulation-only command relay.

## How Simulation Enters LingTu

Simulation may enter only through explicit boundary adapters:

1. The simulator publishes sensor, odometry, map, terrain, or waypoint topics.
2. A simulation adapter converts those topics into LingTu's canonical contracts:
   `/nav/*` for navigation data and `/exploration/*` for exploration data.
3. LingTu modules run the same global planning, local planning, path following,
   safety, and gateway contracts that product profiles use.
4. Any command relay back to a simulator must be isolated to a non-default ROS
   domain and marked `simulation_only=true`.

## Unified Runtime Interface

LingTu core must not care whether the endpoint is a real S100P, MuJoCo, Gazebo,
CMU Unity, or a future simulator. Each endpoint must satisfy the same runtime
port contract. The only endpoint-specific code is the adapter that converts
native topics, frames, and commands into this contract.

The canonical Python interface is defined in `src/core/runtime_interface.py`.
`config/topic_contract.yaml` mirrors it for operators and diagnostics. New
simulators or robot variants must add/choose a data-source contract there
instead of creating new `/nav/*` meanings.

`src/core/blueprints/runtime_endpoint.py` defines the Dimos-style split between
task and connection layer. The product task remains `map`, `nav`, `explore`, or
`tare_explore`; `--endpoint real_s100p|mujoco_live|gazebo|cmu_unity` selects the
runtime source/sink. Compatibility profiles such as `sim_mujoco_live`,
`sim_industrial`, and `sim_cmu_tare` are launcher aliases for gates and demos,
not independent product architectures.

Examples:

```bash
python lingtu.py explore --endpoint mujoco_live
python lingtu.py explore --endpoint gazebo --record
python lingtu.py tare_explore --endpoint cmu_unity --record
```

### Canonical Port Groups

| Group | Canonical topics/artifacts | Producer | Consumer | Required for |
| --- | --- | --- | --- | --- |
| Canonical SLAM input | `/nav/lidar_scan`, `/nav/imu` | Real LiDAR/IMU adapter | Fast-LIO or equivalent SLAM backend | Production mapping/localization |
| Raw SLAM validation input | `/points_raw`, `/imu_raw` | Raw-sensor simulator adapter | Fast-LIO or equivalent SLAM backend | LingTu-owned mapping/localization validation |
| Localization/map output | `/nav/odometry`, `/nav/registered_cloud`, `/nav/map_cloud`, `/nav/localization_health` | LingTu SLAM or external-source adapter | Maps, exploration, navigation, gateway, safety | Live exploration and navigation |
| Exploration | `/exploration/start`, `/exploration/way_point`, `/nav/exploration_grid`, `/exploration/status` | TARE, frontier, or external exploration adapter | Exploration bridge, Navigation, Gateway | No-map exploration |
| Planning/execution | `/nav/goal_pose`, `/nav/patrol_goals`, `/nav/global_path`, `/nav/local_path`, `/nav/mission_status` | LingTu navigation stack | Path follower, safety, gateway, recorder | Goal navigation and route execution |
| Command | `/nav/cmd_vel` | LingTu `CmdVelMux` | Endpoint command adapter only | Simulated or real actuation |
| Artifacts | `map.pcd`, `tomogram.pickle`, `occupancy.npz`, `metadata.json` | Map manager/save pipeline | Relocalization, PCT, diagnostics | Saved-map navigation |

### Endpoint Adapter Matrix

| Endpoint | Native input | Adapter obligation | Canonical LingTu entry | Command sink | Product claim allowed |
| --- | --- | --- | --- | --- | --- |
| Real S100P | MID-360 LiDAR, IMU, robot driver | Publish canonical sensor topics, run Fast-LIO/localizer, relay only muxed safe commands | `/nav/lidar_scan + /nav/imu -> Fast-LIO -> /nav/*` | Real driver through safety/mux | Field mapping, relocalization, navigation after hardware validation |
| MuJoCo raw MID-360 | Simulated MID-360 pattern cloud and IMU | Generate credible raw sensor topics and feed Fast-LIO | `/points_raw + /imu_raw -> Fast-LIO -> /nav/*` | MuJoCo command adapter | LingTu SLAM/map/exploration smoke when raw-source evidence is present |
| Gazebo industrial | Gazebo world, physics, rendered LiDAR, sim odom | Convert Gazebo sensor/odom to `/nav/*`, isolate command relay | `/nav/odometry`, `/nav/registered_cloud`, `/nav/map_cloud`, `/nav/exploration_grid` | `/lingtu/gazebo/cmd_vel` | ROS/GZ integration, obstacle/map-growth, navigation smoke; not Fast-LIO quality |
| CMU Unity external | CMU `/state_estimation`, `/registered_scan`, `/terrain_map_ext`, `/way_point` | Remap external state/map/TARE waypoints into LingTu execution contract | `/nav/odometry`, `/nav/registered_cloud`, `/nav/terrain_map_ext`, `/exploration/way_point` | `/cmd_vel` in isolated simulation domain | LingTu can consume external CMU/TARE waypoints and execute; not LingTu SLAM |
| Future Unity raw-sensor mode | Unity raw LiDAR/IMU export | Publish raw LiDAR/IMU to Fast-LIO before `/nav/*` | `/points_raw + /imu_raw -> Fast-LIO -> /nav/*` | Unity command adapter | LingTu SLAM claim only after raw-source evidence passes |

### Adapter Acceptance Rules

Every endpoint adapter must declare:

- source ownership: who owns world, sensors, localization, map, exploration,
  planning, path following, and command relay;
- frame mapping: native frames to `map`, `odom`, and `body`;
- topic mapping: native topics to the canonical contract;
- command boundary: `/nav/cmd_vel` must never reach hardware from a simulation
  adapter;
- artifact provenance: saved `map.pcd`, tomogram, and occupancy products must
  record the source run and checksums;
- forbidden fallbacks: if PCT falls back to A*, or Fast-LIO is bypassed, the
  report must say so and strict gates must fail.

Simulation must not:

- add new simulator-specific product architectures when a runtime endpoint or
  compatibility gate alias is sufficient;
- disable `SafetyRingModule` stop wiring;
- enable direct-goal or direct-track planning bypasses in product profiles;
- relay `/nav/cmd_vel` to hardware or a default ROS domain;
- use cumulative debug clouds as proof of product SLAM quality.

## SLAM Source Boundary

Every simulation contract must declare where localization and map data come
from. A run may not claim LingTu SLAM, localization, mapping, or exploration
closure unless the declared SLAM source is present in the runtime evidence.

Current boundaries:

- `gazebo_industrial` uses Gazebo odometry and a Gazebo-LiDAR-derived
  occupancy grid. It can validate command plumbing, obstacle checks, frontier
  regressions, and map growth, but it is not a Fast-LIO mapping validation.
- `cmu_unity_baseline` uses CMU Unity `/state_estimation`,
  `/registered_scan`, and `/terrain_map_ext`. It is reference behavior only.
- `cmu_unity_external` relays CMU Unity state, scan, and terrain topics into
  `/nav/*`. It validates LingTu execution against external TARE waypoints; it
  does not validate LingTu Fast-LIO.
- `mujoco_fastlio2_live` is the official MuJoCo raw-sensor Fast-LIO profile
  behind `python lingtu.py sim_mujoco_live ...`. Its native source path is
  `/points_raw + /imu_raw -> fastlio2 -> /Odometry + /cloud_registered +
  /cloud_map`, and the gate must relay those outputs into `/nav/odometry`,
  `/nav/registered_cloud`, and `/nav/map_cloud` before LingTu modules consume
  them.

If a report uses simulator-provided `/state_estimation`, `/registered_scan`, or
`/terrain_map_ext`, it must be described as an external simulation data source,
not as LingTu-built SLAM output.

## Gazebo Role

Gazebo is a ROS-native smoke, regression, and delivery-demo gate. It is useful
for:

- ROS/GZ bridge health;
- frame and timestamp contracts;
- LiDAR-derived occupancy growth;
- wavefront frontier regression checks;
- closed-loop command plumbing.

The supported customer-visible Gazebo entry is `sim_industrial`, launched by
`sim/scripts/launch_lingtu_gazebo_industrial_demo.sh`. Gazebo still owns only
world geometry, physics, sensor rendering, and simulated actuation; LingTu owns
occupancy mapping, exploration goal selection, global planning, local planning,
path following, `CmdVelMux`, and safety.

## CMU Unity Role

CMU Unity is a benchmark adapter for large simulation scenes and TARE/FAR-style
exploration evidence. LingTu adopts the topic contract, not the upstream stack
as a product dependency.

The adapter boundary is:

- CMU -> LingTu: `/state_estimation`, `/registered_scan`, `/terrain_map`,
  `/terrain_map_ext`, `/way_point`;
- LingTu -> CMU: `/exploration/start`;
- simulation-only command relay: `/nav/cmd_vel -> /cmd_vel`, disabled by
  default and allowed only in an isolated ROS domain.

## Responsibility Matrix

| Contract | World/Sensors | Exploration | Global/Strategy | Local Planning | Path Following | Command Owner | Product Claim |
| --- | --- | --- | --- | --- | --- | --- | --- |
| `gazebo_industrial` | Gazebo | LingTu frontier/TARE contract | LingTu navigation | LingTu navigation | LingTu navigation | LingTu `CmdVelMux` to Gazebo adapter | LingTu closed-loop smoke/demo evidence |
| `cmu_unity_baseline` | CMU Unity | CMU TARE/FAR | CMU exploration stack | CMU `localPlanner` | CMU `pathFollower` | CMU path follower to Unity simulator | Reference effect only; not LingTu validation |
| `cmu_unity_external` | CMU Unity | External CMU TARE waypoint source | LingTu navigation, optionally PCT when the gate requires it | LingTu navigation | LingTu path follower | LingTu adapter relay to CMU simulator | LingTu can ingest CMU/TARE waypoints and execute in simulation |
| `mujoco_fastlio2_live` | MuJoCo raw LiDAR/IMU | LingTu frontier when `explore/video` is used | LingTu navigation when `explore/video` is used | LingTu navigation when `explore/video` is used | LingTu path follower when `explore/video` is used | MuJoCo velocity adapter or fixed gate motion | Fast-LIO raw sensor to canonical `/nav/*`; optional live exploration/navigation gate |

Forbidden claims are part of the runtime contract:

- `cmu_unity_baseline` must not be reported as LingTu planning, LingTu command
  ownership, a LingTu product profile, or real-robot readiness.
- `cmu_unity_external` must not be reported as CMU-baseline equivalence, pure
  LingTu exploration, "PCT executes TARE", LingTu Fast-LIO mapping, or
  real-robot readiness.
- PCT is a LingTu planner backend. TARE is an exploration strategy. A gate may
  prove that LingTu/PCT can plan on a same-source CMU map, but that is a
  separate planning claim from TARE exploration quality.

## Runtime Evidence Required

A simulation run can support a product claim only when the report proves:

- `simulation_only=true`;
- `real_robot_motion=false`;
- `cmd_vel_sent_to_hardware=false`;
- odometry, map, terrain, global path, local path, and command topics are seen;
- publisher identity matches the expected LingTu/simulation boundary nodes;
- map or explored area grows from live sensor input;
- Gateway planner diagnostics are available for strict CMU Unity runtime gates.

If topic samples exist but publisher identity is missing or fake, the run is not
a valid LingTu closure.
