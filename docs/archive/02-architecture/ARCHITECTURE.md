# Hardware Architecture

This document describes how LingTu (running on the **Nav board**, an S100P
or equivalent) talks to the **Dog board** that runs brainstem and the
motor PD loop, and how the Flutter client and the Web dashboard fit in.
The Module-level software layout lives in
[`SYSTEM_OVERVIEW.md`](./SYSTEM_OVERVIEW.md).

## Boards

```
                       ┌────────────────────────┐
                       │      Flutter App       │
                       │   (phone / tablet)     │
                       └──┬──────────┬──────────┘
                          │          │
              gRPC :50051 │          │ gRPC :13145    BLE
              robot.v1    │          │ han_dog.Cms    (basic
                          │          │                 control,
                          ▼          │                 emergency stop)
              ┌─────────────────┐    │                ▼
              │   Nav board     │    │       ┌────────────────┐
              │   (S100P)       │    │       │   Dog board    │
              │ ─────────────── │    │       │  (brainstem)   │
              │ LingTu Module   │    │       │ CMS gRPC :13145│
              │ stack:          │    │       │ RL policy +    │
              │  • SLAM bridge  │    │       │  PD controller │
              │  • Navigation   │    │       │  16 joints     │
              │  • Semantic     │    │       │  IMU stream    │
              │  • Gateway 5050 │    │       │  Arbiter       │
              │  • MCP    8090  │    │       │   RC > gRPC    │
              │ ThunderDriver ──┼────┘       └────────┬───────┘
              │   gRPC client   │                     │
              └─────────────────┘                     ▼
                                          ┌────────────────────┐
                                          │   YUNZHUO RC       │
                                          │ SBUS / PPM direct  │
                                          │ Always priority 1  │
                                          └────────────────────┘
```

## Nav board (S100P)

Runs the LingTu Module stack as a single Python process (`lingtu.py`) plus
the SLAM C++ subprocesses managed via `NativeModule` and / or systemd.

| Capability | Module |
|------------|--------|
| LiDAR / IMU acquisition | `LidarModule` (or `SimLidarModule`) |
| SLAM bridge | `SlamBridgeModule` reads `/nav/odometry` + `/nav/map_cloud` over CycloneDDS |
| Maps | `OccupancyGridModule`, `ESDFModule`, `ElevationMapModule`, `VoxelGridModule`, `TraversabilityCostModule`, `MapManagerModule` |
| Semantic | `PerceptionModule`, `EncoderModule`, `SemanticMapperModule`, `EpisodicMemory`, `TaggedLocations`, `VectorMemory` |
| Planning | `NavigationModule` + `GlobalPlannerService` (`astar` / `pct`) + `WaypointTracker` |
| Local autonomy | `TerrainModule` + `LocalPlannerModule` + `PathFollowerModule` (C++ nanobind) |
| Decision | `SemanticPlannerModule`, `GoalResolver`, `VisualServoModule`, `AgentLoop` |
| Driver | `ThunderDriver` (gRPC to `:13145`) |
| Network | `GatewayModule` (`:5050`), `MCPServerModule` (`:8090`), optional `WebRTCStreamModule`, optional `RerunBridgeModule` (`:9090`) |
| Safety | `SafetyRingModule`, `CmdVelMux`, `GeofenceManagerModule` |

`ThunderDriver` is the equivalent of the older `han_dog_bridge`: it is a
gRPC client to the Dog board's CMS server, not a ROS2 node. Watchdog
behaviour (zero `cmd_vel` after 200 ms of silence) is implemented inside
the driver.

## Dog board (brainstem)

| Component | Description |
|-----------|-------------|
| CMS gRPC server | Port `13145`, `han_dog.Cms` service |
| RL policy | ~20 ms inference cycle, outputs 16 joint targets |
| PD controller | Position + velocity loops with `kp` / `kd` per joint |
| IMU | Hamilton quaternion + gyroscope, body frame |
| Joints | 4 legs × 4 joints (hip / thigh / calf / foot) |
| BLE | Minimal: e-stop, mode switch, Wi-Fi config |
| Arbiter | YUNZHUO RC (priority 1) > gRPC clients (priority 2) |

CMS RPCs:

| RPC | Type | Arbitrated? |
|-----|------|-------------|
| `Enable` / `Disable` | unary | no — hardware level |
| `Walk(Vector3)` | unary | yes — RC overrides |
| `StandUp` / `SitDown` | unary | yes |
| `ListenImu` / `ListenJoint` / `ListenHistory` | server stream | no — observation only |
| `GetParams` | unary | no |

Flutter App can connect to either or both ports:

| Mode | Nav | Dog | Use case |
|------|-----|-----|----------|
| Full | `:50051` | `:13145` | Production navigation + low-latency joint diagnostics |
| Nav-only | `:50051` | — | Without Dog-board direct access |
| Dog-only | — | `:13145` | Nav-board down; manual walk / stand / sit |
| BLE-only | — | BLE | E-stop, Wi-Fi setup |

## Safety layers (high to low priority)

1. **YUNZHUO RC** — hardware arbiter on the Dog board. Always wins.
2. **CMS arbiter** — rejects gRPC `Walk` while RC is active.
3. **ThunderDriver watchdog** — 200 ms `cmd_vel` timeout → zero velocity.
4. **`SafetyGate` / `SafetyRingModule`** — deadman, speed and tilt limits,
   near-field obstacle braking from `terrain_map`, mode guard, e-stop.
5. **`CmdVelMux`** — priority arbitration: teleop (100) > visual servo
   (80) > recovery (60) > path follower (40), each with a 0.5 s freshness
   window.
6. **Mode FSM (Nav board)** — guards on `IDLE / MANUAL / TELEOP /
   AUTONOMOUS / MAPPING / ESTOP` transitions (lease, TF, localization).
7. **Localization watchdog** — `SlamBridgeModule.localization_status`
   pushes `DEGRADED` / `LOST` to safety + navigation. The drift watchdog
   inside the gateway also restarts SLAM after a 60 s diverge window.
8. **Geofence + heartbeat** — geofence boundary monitor, plus 30 s slow /
   5 min full stop on heartbeat loss.

`han_dog_bridge` referenced in older docs is the same gRPC-client role
that `ThunderDriver` plays now; the watchdog and Walk-vector clamping
behaviour are unchanged.

## Mode state machine (Nav board)

```
                    ┌──────────┐
         ┌──────────│   IDLE   │──────────┐
         │          └─┬──┬──┬──┘          │
         ▼            │  │  │             ▼
      ┌────────┐      │  │  │       ┌──────────┐
      │ MANUAL │      │  │  │       │ MAPPING  │
      └────────┘      │  │  │       └──────────┘
                      │  │  │
         ┌────────────┘  │  └────────────┐
         ▼               │               ▼
      ┌─────────┐        │         ┌────────────┐
      │ TELEOP  │◄───────┼────────►│ AUTONOMOUS │
      └─────────┘        │         └────────────┘
                         │
              ┌──────────▼──────────┐
              │       ESTOP         │
              │ (any state → ESTOP) │
              │ (ESTOP → IDLE only) │
              └─────────────────────┘
```

Guards:

| Target | Required |
|--------|----------|
| `IDLE` | Always allowed (except from `ESTOP` without clear) |
| `MANUAL` | From `IDLE` |
| `TELEOP` | From `IDLE` / `AUTONOMOUS` + `has_lease` |
| `AUTONOMOUS` | From `IDLE` / `TELEOP` + `tf_ok` + `localization_valid` |
| `MAPPING` | From `IDLE` |
| `ESTOP` | Via `EmergencyStop` only |
| Clear `ESTOP` → `IDLE` | `tilt_safe` + `fence_safe` |

## Concurrent gRPC clients

`:50051` is HTTP/2; multiple clients can subscribe to telemetry streams
at the same time. Lease-protected RPCs (`StreamTeleop`, `SetMode`) accept
only one writer at a time via `AcquireLease`.

| Operation | Concurrent? | Mechanism |
|-----------|-------------|-----------|
| Telemetry (`StreamFastState`, `StreamSlowState`) | yes | independent streams |
| Data (`Subscribe`, `UploadFile`, OTA download) | yes | no exclusive lock |
| Control (`StreamTeleop`, `SetMode`) | one at a time | `AcquireLease` |

## Network layout

```
┌──────────────────────────────────────────────────────────────┐
│                          Robot                               │
│  ┌──────────────┐  Ethernet/USB  ┌────────────────────────┐  │
│  │  Nav board   ├─────────────── │  Dog board (brainstem) │  │
│  │  :5050 (HTTP)│                │  :13145 (gRPC)         │  │
│  │  :8090 (MCP) │                │   plus BLE             │  │
│  └──────┬───────┘                └────────────────────────┘  │
│         │                                                    │
│      Wi-Fi AP                                                │
└─────────┼────────────────────────────────────────────────────┘
          │
     ┌────┴─────┐                            ┌────────────┐
     │ Flutter  │                            │ YUNZHUO RC │
     │ Web UI   │                            │ SBUS/PPM   │
     │ Codex/MCP│                            └────────────┘
     └──────────┘
```

## Default ports and parameters

`ThunderDriver` and the Gateway pull defaults from `config/robot_config.yaml`.
Common overrides:

| Knob | Default | Source |
|------|---------|--------|
| `dog_host` / `dog_port` | `192.168.66.190` / `13145` | `cli/profiles_data.py` thunder preset |
| `gateway_port` | `5050` | `core/blueprints/stacks/gateway.py` |
| `mcp_port` | `8090` | same |
| Camera rotation | `0` | `config/robot_config.yaml: camera.rotate` |
| GNSS enabled | `false` | `config/robot_config.yaml: gnss.enabled` |

Detailed parameter tuning is in [`../TUNING.md`](../TUNING.md).

## What is *not* in this architecture

- **No** `ros2 launch` for Module-First operation. SLAM C++ binaries are
  brought up by `slam.SLAMModule._setup_*` via `NativeModule`, or by the
  systemd unit when `slam("bridge")` is selected.
- **No** `launch/subsystems/` and **no** `scripts/legacy/`. Both
  directories were deleted in commits `6fd7257` and `fe99873` and any
  doc that mentions them is obsolete.
- **No** `navigation_run.launch.py` / `navigation_bringup.launch.py`. The
  CLI (`lingtu.py`) is the only orchestrator.

## Multi-agent task orchestration

If LingTu is being driven by an external orchestrator (Askme voice
agent → `mission-orchestrator` → `nav-gateway` → LingTu gRPC) the
contract and trigger paths are documented in
[`TASK_ORCHESTRATION.md`](./TASK_ORCHESTRATION.md).
