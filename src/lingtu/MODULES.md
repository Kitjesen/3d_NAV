# LingTu — Import-Ready API Wrapper Index

> Files live under `src/lingtu/`

---

## Module Index

| File | Class | Purpose |
|------|-------|---------|
| `__init__.py` | — | Package init — exports LiDAR, SLAM, Navigator, Camera, Detector, Robot |
| `robot.py` | `Robot` | All-in-one robot controller — builds blueprint system from profile |
| `slam.py` | `SLAM` | SLAM lifecycle — start, stop, save/load maps |
| `navigator.py` | `Navigator` | High-level navigation — go(target), go_to(x, y), stop(), follow() |
| `camera.py` | `Camera` | Camera stream access — capture frame, stream URL |
| `detector.py` | `Detector` | Object detection — detect(), detect_objects_in_bbox() |
| `lidar.py` | `LiDAR` | LiDAR hardware driver wrapper — start/stop/point cloud access |

## Tests — `tests/`

| File | Purpose |
|------|---------|
| `test_lingtu_api.py` | Integration tests for the import-ready API |
