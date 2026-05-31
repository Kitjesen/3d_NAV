# SLAM — Localization & Mapping Index

> Files live under `src/slam/`
> 16 .py files including subdirectories

---

## Top-Level (8)

| File | Responsibility |
|------|---------------|
| `__init__.py` | Package init — exports SLAM module classes |
| `slam_module.py` | **SLAMModule** — managed mode: wraps Fast-LIO2/Point-LIO/Localizer C++ binaries, handles lifecycle + health checks |
| `slam_bridge_module.py` | **SlamBridgeModule** — bridge mode: subscribes to external ROS2 SLAM topics (/odom, /cloud_registered), handles map→odom TF transform |
| `fastlio2_live_bridge.py` | Fast-LIO2 live bridge — real-time odometry passthrough for sim profiles |
| `fastlio2_nav_bridge.py` | Fast-LIO2 navigation bridge — registered cloud + odometry for nav stack |
| `gnss_module.py` | **GnssModule** — GNSS localization: RTK GPS → geodetic pose, UTM conversion |
| `gnss_bridge.py` | GNSS ROS2 bridge — subscribes to /fix, publishes GeodeticPose |
| `gnss_serial_driver.py` | GNSS serial driver — NMEA-0183 over UART for ZED-F9P module |
| `ntrip_client_module.py` | **NtripClientModule** — NTRIP RTCM correction client for RTK GNSS |
| `native_factories.py` | Native binary factory — builds Launcher for SLAM C++ binaries (fastlio2, pointlio, localizer, pgo, hba) |
| `depth_visual_odom_module.py` | **DepthVisualOdomModule** — visual odometry from depth camera (VIO fallback when LiDAR unavailable) |

---

## Launch — `launch/` (5)

| File | Responsibility |
|------|---------------|
| `fastlio2_launch.py` | Fast-LIO2 SLAM launch — LiDAR-inertial odometry with IEKF |
| `genz_icp_launch.py` | GenZ ICP launch — LiDAR scan-to-map registration |
| `hba_launch.py` | HBA (Hierarchical Bundle Adjustment) launch — global map refinement |
| `localizer_launch.py` | ICP localizer launch — localization against pre-built map |
| `pgo_launch.py` | PGO (Pose Graph Optimization) launch — loop closure + map correction |
| `pointlio_launch.py` | Point-LIO launch — point-based LiDAR-inertial odometry |
