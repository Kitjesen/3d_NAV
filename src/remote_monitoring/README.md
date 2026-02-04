# Remote Monitoring (gRPC)

This package exposes a minimal gRPC service (C++) for remote status monitoring without ROS on the client.

## What it provides

- gRPC service: `remote_monitoring.TelemetryService`
  - `GetStatus()` unary
  - `StreamStatus()` server streaming

## Proto

Proto file is installed under:

```
share/remote_monitoring/proto/telemetry.proto
```

Clients (Flutter/Android/Web) can generate stubs from it.

## Run

```
ros2 run remote_monitoring grpc_status_bridge
```

### Parameters

- `grpc_port` (int, default `50051`)
- `status_stream_hz` (double, default `1.0`)
- `rate_window_sec` (double, default `2.0`)
- `odom_topic` (string, default `/Odometry`)
- `terrain_map_topic` (string, default `/terrain_map`)
- `path_topic` (string, default `/path`)
- `tf_map_frame` (string, default `map`)
- `tf_odom_frame` (string, default `odom`)
- `tf_body_frame` (string, default `body`)

## Installation

### 1. Install gRPC dependencies

Run the installation script:

```bash
cd src/remote_monitoring/scripts
chmod +x install_grpc.sh
./install_grpc.sh
```

Or install manually:

```bash
sudo apt-get install -y libgrpc++-dev protobuf-compiler-grpc libprotobuf-dev
```

### 2. Build the package

```bash
colcon build --packages-select remote_monitoring
source install/setup.bash
```

## Notes

- `/terrain_map` is expected in `odom` frame.
- The server calculates topic rates within a rolling window.
- Build requires `protoc` and `grpc_cpp_plugin` in PATH.
