# LingTu — Import-Ready API Wrapper

> Files live under `src/lingtu/`

This package provides a high-level, import-ready Python API for the LingTu navigation system. It wraps the underlying Module-First framework into simple classes so that users can drive the robot without touching Blueprints, Modules, or ROS2 internals.

## Quick Start

```python
from lingtu import LiDAR, SLAM, Navigator, Camera, Detector, Robot

# Full robot (all-in-one, builds a real blueprint system)
robot = Robot("sim").start()
robot.go("体育馆")               # semantic navigation
robot.follow("person in red")   # follow a described person
robot.stop_follow()
robot.save_map("building_a")
robot.shutdown()
```

## API Classes

| Class | File | Purpose |
|-------|------|---------|
| `LiDAR` | `lidar.py` | Livox MID-360 hardware driver wrapper |
| `SLAM` | `slam.py` | SLAM lifecycle (start/stop/save_map) |
| `Navigator` | `navigator.py` | High-level navigation commands |
| `Camera` | `camera.py` | HAP camera stream access |
| `Detector` | `detector.py` | Object detection wrapper |
| `Robot` | `robot.py` | All-in-one robot controller |

## Usage Profiles

The `Robot` class supports the same profiles as the CLI entry point:

- `"sim"` — MuJoCo simulation (full stack)
- `"nav"` — Navigate using saved map
- `"explore"` — Exploration mode
- `"map"` — Mapping mode (SLAM + save)
- `"dev"` — Semantic pipeline, no C++ nodes

## Testing

```bash
python -m pytest src/lingtu/tests/
```
