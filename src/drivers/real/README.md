# Real — Hardware Drivers for Physical Robots

This package provides drivers for real robot hardware. Each sub-package encapsulates a complete hardware interface stack, from low-level communication to high-level Module ports.

## `thunder/` — Thunder Robot (gRPC via Brainstem)

- **`__init__.py`** — Package init; exports `ThunderDriverModule` and `CameraBridgeModule`.
- **`blueprints.py`** — Thunder stack blueprint: wires ThunderDriver + CameraBridge into a composable driver stack.
- **`camera_bridge_module.py`** — Camera bridge: converts compressed camera frames from Thunder to standard Image messages.
- **`connection.py`** — Connection management: gRPC channel lifecycle, retry, and reconnection to brainstem control service.
- **`han_dog_module.py`** — HanDog hardware module: low-level joint command interface for the Han robot variant.

## `lidar/` — Livox MID-360 LiDAR Driver

- **`__init__.py`** — Package init; exports `LidarModule`.
- **`lidar.py`** — LiDAR configuration and utilities: IP, port, scan pattern parameters for Livox MID-360.
- **`lidar_module.py`** — LidarModule: hardware driver for Livox MID-360; publishes PointCloud2 via UDP packet capture.
- **`_dds.py`** — DDS integration: publishes lidar data over CycloneDDS for cross-process consumption.
