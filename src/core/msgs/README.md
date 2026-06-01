# Messages — Typed Data Structures for Module Communication

This package defines the message types exchanged between Modules through In[T]/Out[T] ports. Each module specifies its port types explicitly; messages provide structured, validated data containers.

## Files

- **`geometry.py`** — Pose, Transform, Twist, Point, Quaternion, and Vector3 message types for spatial data exchange.
- **`nav.py`** — Navigation messages: Path, Waypoint, GoalStatus, OccupancyGrid, and MapMetaData for planning and control.
- **`sensor.py`** — Sensor data messages: Image, PointCloud2, Imu, LaserScan, and CompressedImage for hardware input.
- **`semantic.py`** — Semantic messages: Detection, SceneGraph, ObjectLabel, and SemanticClass for perception output.
- **`scene.py`** — Scene graph messages: SceneNode, Relationship, Room, and Topology for spatial-semantic hybrid maps.
- **`robot.py`** — Robot state messages: RobotState, JointState, BatteryState, and Temperature for hardware status.
- **`gnss.py`** — GNSS messages: GpsFix, UtmPose, and NavSatFix for outdoor global positioning.
