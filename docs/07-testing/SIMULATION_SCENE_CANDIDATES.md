# Simulation Scene Candidates

This note tracks external open-source or publicly available simulation scenes
that can be adapted into LingTu validation. It is intentionally product-facing:
the goal is to pick environments that make navigation, mapping, obstacle
avoidance, and exploration evidence credible.

## Recommendation

Use three tiers instead of one overloaded scene:

1. **LingTu Gazebo product demo**: default local smoke and customer-visible
   Gazebo demo. Keep this in-repo and deterministic. The preferred product
   scene is now `sim/worlds/lingtu_gazebo_industrial_park.sdf`; the smaller
   `lingtu_gazebo_demo_room.sdf` remains a compact CI/control gate.
2. **CMU/TARE Unity environment**: primary realistic exploration benchmark.
   Use the Unity adapter and require `/way_point`, `/nav/cmd_vel`, odometry
   displacement, and map/exploration growth.
3. **External Gazebo warehouse/office worlds**: regression and stress scenes
   for navigation, SLAM/map growth, and obstacle layout diversity.

## Candidates

| Candidate | Fit | Why it matters | Integration risk |
| --- | --- | --- | --- |
| CMU autonomous exploration development environment | High | Designed for ground robot navigation/exploration, includes multiple environment types and point clouds. | ROS1/Noetic roots; use assets and adapter path rather than importing runtime wholesale. |
| CMU mecanum autonomy stack Unity model | High | Closest to TARE/FAR planner demos and uses Unity scene assets with map/traversable files. | ROS2 Jazzy-oriented; assets often downloaded separately. |
| Clearpath simulator worlds | High | Gazebo Harmonic worlds include warehouse, office, construction, orchard, pipeline, and solar farm. | ROS2 Jazzy/Harmonic; import selected worlds/assets, do not adopt robot stack. |
| SIMLAN warehouse | Medium | ROS2 Humble/Jazzy, Ignition Gazebo warehouse, multi-sensor support. | Camera-centric; adapt scene assets and sensor layout selectively. |
| Gazebo worlds/models/maps dataset | Medium | Many classic Gazebo office/house/hospital/factory/dynamic worlds, useful for fast regression diversity. | Gazebo Classic 9/11 oriented; convert or isolate as legacy import. |
| bcr_bot small warehouse | Low/Medium | Good ROS2 mapping/Nav2 reference with Gazebo and Isaac examples. | Robot package first, scene second; useful as reference rather than primary LingTu scene. |
| Husarion Gazebo worlds | Medium | Apache-2.0 sample Gazebo worlds including an office SDF with maps/models layout. | Small asset set; useful as a clean import template. |
| PAL PMB2 simulation | Medium | ROS2 Humble Gazebo setup with PAL office world, Nav2, and SLAM launch path. | Robot-specific stack; import the office world or use as launch/reference only. |
| rosnav | Medium | ROS2 Humble/Jazzy Gazebo Harmonic stack with maze, warehouse, house, corridor, obstacles, SLAM, Nav2, and frontier exploration. | Young project; inspect license/assets before depending on it. |
| Pathfinder | Low/Medium | ROS2 Humble Gazebo Classic worlds, SLAM Toolbox, Nav2, m-explore, and multi-robot simulation. | Classic Gazebo; good exploration reference, not primary scene format. |
| RDSim | Medium | Delivery-robot Gazebo simulator with SLAM, localization, planning, Nav2, MPPI, and dynamic actors. | More system than scene; assess build cost before import. |
| Autonomous Explorer and Mapper ROS2 Nav2 | Low | Simple frontier exploration workflow using TurtleBot3, Nav2, and SLAM Toolbox. | Algorithm reference only; relies on TurtleBot3 worlds. |

## Import Plan

1. Keep LingTu-owned Gazebo scenes as the product demo path. Use
   `lingtu_gazebo_industrial_park.sdf` for customer-visible industrial-park
   validation and `lingtu_gazebo_demo_room.sdf` for fast CI regression.
2. Add `sim/external_scenes/README.md` with download locations and license
   checks before copying assets into the repo.
3. For CMU Unity, standardize the server asset layout:

   ```text
   sim/external_scenes/cmu_unity/
     environment/
       Model.x86_64
       Model_Data/
       Dimensions.csv
       Categories.csv
       AssetList.csv
     map.ply
     traversable_area.ply
     object_list.txt
   ```

4. Add a LingTu scene selector that maps `scene:=cmu_unity`, `scene:=gazebo_demo`,
   `scene:=industrial_park`, and `scene:=warehouse` to the correct
   adapter/launch path.
5. Make each scene pass a narrow gate before it is called product-ready:

   - odometry moves and stays inside bounds
   - `/nav/cmd_vel` is non-zero and simulation-only
   - sensor topics are non-empty and frame-aligned
   - map or occupancy area grows
   - planned path does not cross occupied cells
   - hardware command output remains disabled

## Do Not Do

- Do not merge an external robot stack into LingTu runtime just to get a scene.
- Do not make CMU Unity the only demo path; it has heavier GPU/runtime
  requirements than Gazebo.
- Do not claim imported scene assets are redistributable until the license and
  download terms are checked.

## Current Product Decision

Build our own LingTu industrial-park scene first. It gives us deterministic
SDF assets, clear ownership, stable CI behavior, and a customer-visible setting
that matches outdoor factory-park work: main road, cross road, perimeter fences,
warehouse/factory buildings, loading dock, containers, pallets, pipe racks,
tanks, and guard posts.

Keep CMU Unity connected as an external benchmark for TARE-style exploration
and larger-scale evidence. It is valuable, but it should not be the default
deliverable scene because it brings third-party asset layout, GPU/display
requirements, and licensing/download checks into the core product path.
