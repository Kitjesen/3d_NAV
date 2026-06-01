# Transport — Inter-Module Communication Backends

This package provides pluggable transport backends for inter-module data flow. Each backend implements the same abstract interface, allowing per-wire transport selection (callback, DDS, or shared memory).

## Files

- **`abc.py`** — Abstract base class `TransportBackend`; defines the subscribe/publish interface for all transport implementations.
- **`local.py`** — Local (in-process callback) transport: zero-copy direct function calls with 0 latency; default for same-thread wiring.
- **`dds.py`** — DDS transport via CycloneDDS: decoupled pub/sub for cross-process or cross-machine module communication on the robot.
- **`shm.py`** — Shared memory transport: zero-copy large-data transfer (point clouds, images) via multiprocessing shared buffers.
- **`dual.py`** — Dual transport: sends over both DDS and local simultaneously; receiver picks the fastest available channel.
- **`adapter.py`** — Transport adapter: converts between transport types (e.g., DDS→local) for heterogeneous module graphs.
- **`factory.py`** — Transport factory: resolves transport type strings to backend instances and handles configuration injection.
