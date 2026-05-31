# Core — Framework Module Index

> Files live under `src/core/`
> 85+ .py files including subdirectories (excluding tests)
> This document indexes every non-test file by purpose.

---

## Top-Level Core Files (25)

| File | Responsibility |
|------|---------------|
| `__init__.py` | Package root — exports public API symbols |
| `module.py` | **Module** base class — In[T]/Out[T] ports, @skill/@rpc decorators, layer tags, lifecycle (setup/start/stop) |
| `blueprint.py` | **Blueprint** builder — autoconnect, per-wire transport, auto_wire, add/remove modules |
| `stream.py` | **Out[T]/In[T]** ports — 5 backpressure policies (latest/throttle/sample/buffer/callback), thread-safe |
| `registry.py` | **Plugin registry** — `@register("category", "name")` decorator, `get()` resolver |
| `config.py` | Typed config dataclasses — speed/geometry/driver/safety/lidar/camera/gnss sections |
| `config_loader.py` | Dict-based config loader — `load_robot_config()`, config discovery, validation |
| `dds.py` | DDS transport layer — CycloneDDS integration for ROS2-free pub/sub |
| `clock.py` | Sim clock — rate-limited time source for sim profiles |
| `coordinator.py` | System coordinator — module lifecycle orchestration, health checks |
| `native_factories.py` | C++ native module factory — resolve and instantiate NativeModule subclasses |
| `native_install.py` | C++ native binary installer — build/install cmake targets for aarch64 |
| `native_module.py` | **NativeModule** base — C++ process lifecycle management (subprocess + health check) |
| `plugin_seed.py` | Builtin plugin seeder — `seed_builtin_plugins()` for @register discovery |
| `worker.py` | Background worker — thread pool for offloading blocking I/O |
| `worker_manager.py` | Worker lifecycle manager — start/stop/status for background workers |
| `service_manager.py` | Systemd service manager — `svc.start/stop/restart/status` wrapper |
| `rerun_module.py` | Rerun visualization module — 3D scene debugging via Rerun SDK |
| `runtime_interface.py` | Runtime interface constants — FRAMES, TOPICS, service names |
| `runtime_switch.py` | Runtime feature flags — enable/disable features without restart |
| `runtime_policy.py` | Runtime policy engine — configurable decision policies |
| `runtime_evidence.py` | Runtime evidence collector — audit trail for decisions |
| `runtime_validation_gates.py` | Runtime validation gates — pre-flight checks before module start |
| `yaml_helpers.py` | YAML serialization helpers — safe_load/dump with numpy support |
| `swap_manager.py` | **SwapManager** — multi-robot profile hot-swap (Phase 2-3) |

---

## Blueprints & Stacks — `blueprints/` (17)

| File | Responsibility |
|------|---------------|
| `__init__.py` | Package init — exports stack factories and full_stack entry point |
| `full_stack.py` | **Top-level blueprint** — calls 9 stack factories, wires cross-stack connections |
| `full_stack_wiring.py` | Explicit wire definitions — safety→actuators, SLAM odom priority, autonomy chain, cmd_vel mux |
| `multi_robot.py` | Multi-robot blueprint — per-robot profile isolation, SwapManager integration |
| `profile_graph.py` | Profile dependency graph — nav->maps->slam->driver DAG |
| `runtime_endpoint.py` | Runtime endpoint — manual start/stop CLI per profile |
| `simulation_contract.py` | Simulation contract — profile↔sim backend mapping |
| `stub.py` | Stub blueprint — all-mock system for framework testing |
| `stacks/__init__.py` | Stack factories init — lazy import gate |
| `stacks/_registry.py` | Centralized backend registration — all `@register` calls in one place |
| `stacks/driver.py` | `driver(robot)` factory — Thunder/gRPC, stub, sim_mujoco, sim_ros2 |
| `stacks/slam.py` | `slam(profile)` factory — fastlio2, pointlio, localizer, bridge |
| `stacks/maps.py` | `maps()` factory — OccupancyGrid + ESDF + ElevationMap |
| `stacks/perception.py` | `perception(det, enc)` factory — Detector + Encoder + Reconstruction |
| `stacks/memory.py` | `memory(save_dir)` factory — SemanticMapper + Episodic + Tagged + VectorMemory |
| `stacks/planner.py` | `planner(llm)` factory — SemanticPlanner + LLM + VisualServo |
| `stacks/navigation.py` | `navigation(planner)` factory — NavigationModule + Autonomy chain |
| `stacks/safety.py` | `safety()` factory — SafetyRing + Geofence |
| `stacks/gateway.py` | `gateway(port)` factory — Gateway + MCP + Teleop |
| `stacks/exploration.py` | `exploration(backend)` factory — TARE exploration |
| `stacks/lidar.py` | `lidar(ip, enabled)` factory — Livox MID-360 hardware driver |
| `stacks/sim_lidar.py` | `sim_lidar(scene_xml)` factory — Simulated PointCloud2 from MuJoCo |

---

## Messages — `msgs/` (7)

| File | Responsibility |
|------|---------------|
| `__init__.py` | Package init — re-exports all message types |
| `geometry.py` | Geometry types — Pose, Point, Quaternion, Transform, Path |
| `sensor.py` | Sensor data types — Image, PointCloud, Imu, Odometry, JointState |
| `nav.py` | Navigation types — NavGoal, NavStatus, MissionStatus, Waypoint |
| `scene.py` | Scene graph types — SpatialRelation, SceneGraph, TrackedObject, RoomInfo |
| `semantic.py` | Semantic types — Detection, Classification, SemanticLabel |
| `robot.py` | Robot state types — RobotState, BatteryStatus, FootContact |
| `gnss.py` | GNSS types — GPSFix, RTKStatus, GeodeticPose |

---

## Transports — `transport/` (6)

| File | Responsibility |
|------|---------------|
| `__init__.py` | Package init — exports transport adapters |
| `abc.py` | Transport abstract base — Adapter interface |
| `adapter.py` | Transport adapter — wire-level connection management |
| `local.py` | Local transport — in-process callback (zero latency, default) |
| `dds.py` | DDS transport — CycloneDDS pub/sub, ROS2-compatible topics |
| `shm.py` | Shared memory transport — numpy zero-copy for high-bandwidth (point clouds) |
| `dual.py` | Dual transport — local + DDS simultaneous |
| `factory.py` | Transport factory — create adapter by name |

---

## Spec — `spec/` (6)

| File | Responsibility |
|------|---------------|
| `__init__.py` | Package init — imports all spec modules |
| `driver.py` | Robot driver spec — `implements(DriverSpec)` contract |
| `nav.py` | Navigation spec — `implements(NavSpec)` contract |
| `perception.py` | Perception spec — `implements(PerceptionSpec)` contract |
| `planning.py` | Planning spec — `implements(PlanningSpec)` contract |
| `memory.py` | Memory spec — `implements(MemorySpec)` contract |
| `safety.py` | Safety spec — `implements(SafetySpec)` contract |
| `llm.py` | LLM spec — `implements(LLMSpec)` contract |

---

## Devices — `devices/` (6)

| File | Responsibility |
|------|---------------|
| `__init__.py` | Package init |
| `base.py` | Device base class — open/close/read/write abstraction |
| `spec.py` | Device spec — protocol definition for hardware devices |
| `manager.py` | Device manager — enumerate, open, allocate devices |
| `decoder.py` | Stream decoder — NMEA/UBX binary frame parsing |
| `decoders/__init__.py` | Decoder registry |
| `decoders/nmea.py` | NMEA-0183 sentence parser |
| `drivers/__init__.py` | Serial driver registry |
| `drivers/serial_nmea0183.py` | Serial NMEA-0183 driver — RTK/GPS serial input |

---

## Introspection — `introspection/` (2)

| File | Responsibility |
|------|---------------|
| `__init__.py` | Package init |
| `dot.py` | DOT graph export — system topology visualization (Graphviz) |
| `text.py` | Text renderer — system topology as ASCII tree |

---

## Utils — `utils/` (9)

| File | Responsibility |
|------|---------------|
| `__init__.py` | Package init — exports all utility functions |
| `binary_codec.py` | Binary codec — numpy <-> bytes serialization |
| `blackbox_recorder.py` | Blackbox recorder — rolling buffer of recent events for crash diagnostics |
| `calibration_check.py` | Runtime calibration validator — sanity checks on robot_config.yaml params |
| `livox_config.py` | Livox LiDAR config helper — IP/port/broadcast setup |
| `redact.py` | Sensitive data redactor — API keys, tokens in logs |
| `robustness.py` | Robustness utilities — retry, timeout, fallback decorators |
| `sanitize.py` | Input sanitizer — path traversal prevention, shell injection guard |
| `scene_mode_detector.py` | Scene mode detector — indoor/outdoor/transition classification |
| `validation.py` | Cross-layer validators — port contract, module dependency, blueprint consistency |

---

## Other Subdirectories

| Directory | Contents |
|-----------|----------|
| `contracts/` | `__init__.py`, `messages.py` — Message contract definitions, backward compat checks |
| `resource_monitor/` | `__init__.py`, `monitor.py` — CPU/RAM/disk usage tracking |
| `utils/` | 9 utility modules (see above) |

---

## Remaining Top-Level Files

| File | Responsibility |
|------|---------------|
| `algorithm_gates.py` | Algorithm feature gates — per-backend capability reporting |
| `backend_status.py` | Backend health status — structured status reports per module |
| `dynamic_filter.py` | Dynamic obstacle filter — hit-count voting for SSE view |
| `efficiency_status.py` | Efficiency metrics — per-module latency/throughput counters |
| `gateway_runtime_acceptance.py` | Gateway runtime checks — pre-acceptance validation |
| `inspection_acceptance.py` | Inspection acceptance — manufacturing QA test runner |
| `product_field_check.py` | Field parameter checker — validates deployment config |
| `remote_ports.py` | Remote port registry — well-known port assignments |
| `ros2_context.py` | ROS2 context manager — lazy rclpy init/shutdown |
| `rpc_client.py` | RPC client — JSON-RPC over HTTP for inter-process calls |
| `same_source_map_artifacts.py` | Map artifact management — PCD/tomogram file handling |
