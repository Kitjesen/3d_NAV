# External Simulation Scenes

External scenes are optional assets used for server-side validation. Keep large
or license-restricted assets out of git unless their redistribution terms are
confirmed.

Expected server layout for CMU Unity assets:

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

Expected server layout for imported Gazebo worlds:

```text
sim/external_scenes/gazebo/
  <scene_name>/
    worlds/
    models/
    maps/
```

Use a scene only after its gate proves sensor topics, frame alignment, map
growth, non-zero simulated motion, obstacle clearance, and no hardware command
output.
