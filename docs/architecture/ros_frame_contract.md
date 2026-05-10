# LingTu ROS Frame Contract

This document records the frame and topic contract used by the navigation
stack. It describes the boundary that modules, ROS 2 launch files, simulators,
and App/Web telemetry must preserve.

## Canonical Frames

| Frame | Meaning | Owner |
| --- | --- | --- |
| `map` | Saved-map and global planning frame. Goals and global paths are expressed here. | localizer / map manager |
| `odom` | Continuous local odometry frame. LIO publishes odometry here before global relocalization correction. | Fast-LIO2 / Point-LIO / simulation bridge |
| `body` | LingTu control body frame. Local planner paths and velocity commands are body-relative. | runtime contract |
| `base_link` | Robot model root link used by URDF/MJCF/Gazebo assets. | robot model |
| `world` | Simulator world frame only. It must be aliased into `map` or `odom` before entering LingTu runtime topics. | simulator |

`body` and `base_link` are not interchangeable inside the codebase. The bridge
boundary must make the alias explicit:

```text
robot model:      base_link
LingTu runtime:   body
bridge contract:  base_link == body, published as odom -> body
```

## Required TF Shape

```text
map -> odom -> body
                 -> lidar_link
                 -> camera_link
```

For simulation, `world` may be identical to `map` at startup, but it should not
appear as the frame on `/nav/*` topics. Publish `map -> odom` as identity when
there is no relocalization correction.

## Navigation Topic Frames

| Topic | Type | Required frame |
| --- | --- | --- |
| `/nav/goal_pose` | `geometry_msgs/PoseStamped` | `map` |
| `/nav/global_path` | `nav_msgs/Path` | `map` |
| `/nav/way_point` | `geometry_msgs/PointStamped` | `odom` when consumed by ROS local planner, `map` when consumed by `NavigationModule` only |
| `/nav/odometry` | `nav_msgs/Odometry` | `map` for `NavigationModule`; `odom` is allowed only when an adapter transforms it before module entry |
| `/nav/map_cloud` | `sensor_msgs/PointCloud2` | `map` or `odom` world/local-map coordinates, never body-relative points |
| `/nav/registered_cloud` | `sensor_msgs/PointCloud2` | `body` |
| `/nav/terrain_map` | `sensor_msgs/PointCloud2` | `body` or `odom`, matching the local planner profile |
| `/nav/local_path` | `nav_msgs/Path` | `body` |
| `/nav/cmd_vel` | `geometry_msgs/TwistStamped` | `body` |

The Python `NavigationModule` deliberately rejects odometry, goals, and
costmaps that do not match its configured `planning_frame_id`. Real robot
`nav` and `explore` profiles currently set `planning_frame_id="odom"` because
their external SLAM services publish the live navigation contract in that
frame. Saved-map global navigation should move toward `map` once the
`map -> odom` transform is consistently available and tested.

## Gazebo/GZ Integration Boundary

Gazebo should be treated as another simulator backend, not as a special
navigation stack. The first supported Gazebo shape is:

```text
Gazebo world/model sensors
  -> ros_gz_bridge or a small Python bridge
  -> /nav/odometry, /nav/registered_cloud, /camera/*
  -> ROS2SimDriverModule / native ROS planners
  -> /nav/cmd_vel
  -> Gazebo velocity controller
```

The bridge must not publish `base_link` or `world` directly into LingTu module
ports. It should translate model frames into the canonical contract above.

## Current Gaps

1. Gazebo/GZ now has a first-class `sim_gazebo` profile and launch entry, but
   the default robot is a navigation proxy model for ROS-native validation, not
   a full Thunder/S100P dynamics model.
2. `body`/`base_link` aliasing is still bridge-level convention, not enforced by
   a real TF validation gate.
3. `/nav/map_cloud` semantics are mixed across legacy simulation code. New code
   must keep map/world clouds separate from body-frame registered clouds.
4. The ROS local planner consumes `odom` and `body` frames, while the Python
   navigation module can be configured for `map` or `odom`. Tests must state
   which path is being validated.
5. Gazebo sensor/controller plugins still need runtime verification on a ROS 2
   host. Static tests verify the model/topic contract; they do not prove Gazebo
   is publishing every bridged topic at rate.
