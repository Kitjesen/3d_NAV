# Simulation Delivery Backlog

This file records known simulation gaps that must not be hidden behind passing
unit or contract gates.

## Closed In This Pass

- Official Livox MID-360 scan pattern is now a repo asset at
  `sim/assets/livox/mid360.npy`.
- MuJoCo Fast-LIO live mapping and native PCT MuJoCo gates default to the
  official MID-360 pattern and report the asset path, SHA256, and
  `forced_pattern=true`.
- The legacy golden-spiral LiDAR fan is available only through the explicit
  `--allow-golden-spiral-lidar` escape hatch.

## Still Open

- CMU Unity assets are still external benchmark assets, not first-class LingTu
  scenes. The product path needs a checked server asset layout plus a single
  launch wrapper that starts Unity, LingTu adapter, TARE, mapping, navigation,
  and evidence recording.
- Gazebo remains a smaller visual/runtime smoke scene. It should be replaced or
  complemented by an explicit large demo room/factory scene with enough free
  space for turning, exploration, and obstacle clearance checks.
- Gazebo LiDAR is still Gazebo sensor driven. The MID-360 pattern enforcement in
  this pass applies to MuJoCo gates; Gazebo needs either the official Livox
  plugin path or a matching point-cloud replay/adapter before claiming MID-360
  pattern parity.
- TARE is not yet proven as a native Gazebo runtime exploration loop. Current
  reliable runtime evidence should come from the CMU Unity adapter line until
  Gazebo TARE is launched and verified end to end.
- `mujoco_ros2_control` is not integrated. It may be useful later for
  joint-level controller validation, but the current product gates still use
  LingTu velocity/control adapters and native planner nodes.
- Full-stack proof still needs a single report showing global plan, local plan,
  path follower, `/nav/cmd_vel`, odometry, map growth, and exploration frontier
  evidence from the same simulation run.
- Server sync remains an operational step: the server checkout must contain
  this asset and code change before server gates can be treated as current.
