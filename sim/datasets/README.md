# Simulation Dataset Boundary

`sim/datasets/` stores offline LiDAR/IMU datasets and small metadata fixtures
used by replay and SLAM evaluation scripts.

## Current Dataset Families

| Directory | Role |
| --- | --- |
| `Avia/` | Livox Avia replay data or placeholders for LiDAR-inertial tests. |
| `legkilo_outdoor/` | Outdoor LEG-KILO trajectory and ROS 2 metadata fixtures. |
| `legkilo_full/` | Expanded LEG-KILO corridor fixtures and metadata. |
| `legkilo_all/` | Aggregated LEG-KILO dataset material when present locally. |
| `legkilo-dataset/` | Compatibility path for older scripts. |

## Contract

- This folder is for offline replay inputs, not generated validation evidence.
- Reproducible gate outputs belong under `artifacts/`, not `sim/datasets/`.
- Large raw bags should stay out of git unless there is an explicit small
  fixture contract.
- Dataset scripts must report whether evidence is raw replay, SLAM algorithm
  replay, or saved-map relocalization; do not treat those levels as equivalent.
