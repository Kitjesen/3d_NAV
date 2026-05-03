# LingTu S100P Deployment

How LingTu is laid out on the Sunrise robot, how to install, update, roll back, and
diagnose the running stack. The single production entry is `python lingtu.py`,
orchestrated by `lingtu.service`. There is no Docker, no `colcon launch` orchestration,
and no `grpc_gateway` daemon in the current build.

---

## One-liner status

```bash
ssh sunrise@192.168.66.190 'systemctl status lingtu robot-brainstem robot-camera robot-lidar robot-fastlio2 robot-localizer'
ssh sunrise@192.168.66.190 'lingtu status'
```

The `lingtu` CLI gives you an 8-section snapshot — see `lingtu_cli.md`.

---

## Three-layer architecture on the robot

```
Hardware       robot-camera         Orbbec Gemini 330           [systemd]
               robot-lidar          Livox MID-360 driver         [systemd]

Control        robot-brainstem      Dart gRPC :13145             [systemd]

SLAM           robot-fastlio2       Fast-LIO2 odometry           [systemd]
               robot-localizer      ICP localizer (needs map)    [systemd]

Algorithm      lingtu               python lingtu.py nav          [systemd]
                                    HTTP :5050  MCP :8090
```

Principle: `robot-*` services are the foundation; `lingtu` is the application that
lives on top. OTA only updates `lingtu` (the contents of `/opt/lingtu/current`).
The five `robot-*` services are stable and rarely change.

Only `robot-camera.service` should own the Orbbec ROS nodes. Legacy units such
as `camera.service` or `orbbec-camera.service` must stay stopped or masked; if
they run together with `robot-camera.service`, ROS will show duplicate
`/camera/camera` and `/camera/camera_container` node names.

---

## Service inventory (current)

| Service             | Role                                  | Process                                   | Enabled |
|---------------------|---------------------------------------|-------------------------------------------|:-------:|
| `robot-lidar`       | Livox MID-360 driver                  | `livox_ros_driver2_node`                  | yes     |
| `robot-camera`      | Orbbec Gemini 330                     | `component_container` (orbbec_camera)     | yes     |
| `robot-brainstem`   | Quadruped leg control gRPC :13145     | `dart han_dog/bin/server.dart`            | yes     |
| `robot-fastlio2`    | LiDAR-IMU SLAM                        | `fastlio2 lio_node`                       | yes     |
| `robot-localizer`   | ICP localizer (uses prebuilt map)     | `localizer_node`                          | yes     |
| `lingtu`            | Algorithm stack (gateway + nav + sem) | `python3 lingtu.py nav`                   | yes     |
| `ota-agent`         | OTA poller / installer                | `python3 -m ota_agent.main`               | yes     |

Service unit files live in `docs/04-deployment/services/`:

```
robot-lidar.service
robot-camera.service
robot-brainstem.service
robot-fastlio2.service
robot-localizer.service
lingtu.service
lingtu.target            # umbrella unit pulling in robot-* + lingtu
ros2-env.sh              # shared ROS2 environment (sourced by service ExecStart)
install.sh               # idempotent installer
```

> The previous `slam.service` was removed and replaced with `robot-fastlio2.service` +
> `robot-localizer.service`. The legacy `nav-*.service` family (12 disabled stubs from a
> prior generation), `lingtu_*.service`, `askme*.service`, and the in-process
> `grpc_gateway`-style `ota-daemon.service` are all gone — see `S100P_STACK_INVENTORY.md`
> for the historical cleanup decisions.

---

## Filesystem layout on the robot

```
/opt/lingtu/
  current -> releases/v2.0.0/   # symlink — OTA flips this
  releases/
    v2.0.0/                     # immutable, current release
    v1.9.x/                     # kept for rollback
  config/
    ros2-env.sh                 # shared ROS2 environment
  nav/
    ota/                        # ota-agent state, manifests, backups
  logs/                         # algorithm-layer runtime logs
  models/                       # BPU .hbm and ONNX

/etc/systemd/system/
  robot-{lidar,camera,brainstem,fastlio2,localizer}.service
  lingtu.service
  lingtu.target
  ota-agent.service

/home/sunrise/data/
  nova/maps/                    # prebuilt PCD maps
    active/map.pcd              # localizer reads this
```

Source on the robot lives in `~/data/SLAM/navigation/` (a symlink to the active git
checkout). The `lingtu.service` unit shipped under `docs/04-deployment/services/` runs
out of `/opt/lingtu/current/`; the unit currently checked into `scripts/deploy/`
points at `~/data/inovxio/lingtu/` instead — that's the developer-shell variant used
during the migration to immutable releases. New robots should use the
`docs/04-deployment/services/` files.

---

## Installing on a new robot

```bash
# 1. Clone repo to ~/data/SLAM/navigation
ssh sunrise@<robot> 'git clone https://github.com/Kitjesen/lingtu ~/data/SLAM/navigation'

# 2. Build native modules
ssh sunrise@<robot> 'cd ~/data/SLAM/navigation && bash scripts/build_nav_core.sh'

# 3. Build ROS2 workspace
ssh sunrise@<robot> 'cd ~/data/SLAM/navigation && source /opt/ros/humble/setup.bash && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release'

# 4. Install service units
ssh sunrise@<robot> 'cd ~/data/SLAM/navigation && bash docs/04-deployment/services/install.sh'

# 5. Start everything
ssh sunrise@<robot> 'sudo systemctl start lingtu.target'
```

`install.sh` copies the six service files to `/etc/systemd/system/`, copies
`ros2-env.sh` to `/opt/lingtu/config/`, runs `daemon-reload`, and `enable`s all of
them. It is idempotent.

---

## OTA update flow

1. Push to `main` and tag `v2.x.x`.
2. CI builds + signs the artifact, publishes a GitHub Release.
3. `ota-agent` polls the configured server URL (`/opt/lingtu/nav/ota/config.yaml`).
4. On match: download, verify Ed25519 signature, verify SHA256, stage to
   `/opt/lingtu/releases/v2.x.x/`.
5. Atomic symlink swap: `ln -sfn /opt/lingtu/releases/v2.x.x /opt/lingtu/current`.
6. `systemctl restart lingtu`.
7. Health check: `curl http://localhost:5050/api/v1/health`.

Only `/opt/lingtu/current` moves. The five `robot-*` services keep running. See
`OTA_GUIDE.md` for the manifest schema and signing details.

---

## Rollback

```bash
ssh sunrise@<robot>
ls /opt/lingtu/releases/
sudo ln -sfn /opt/lingtu/releases/v1.9.0 /opt/lingtu/current
sudo systemctl restart lingtu
curl http://localhost:5050/api/v1/health
```

If `ota-agent` shipped a bad release and the robot is unreachable: power-cycle, then
roll back via SSH (the `robot-*` services come up first; `lingtu.service` follows).

---

## Common operations

| Need                              | Command                                                 |
|-----------------------------------|---------------------------------------------------------|
| Start everything                  | `sudo systemctl start lingtu.target`                    |
| Stop algorithm layer only         | `sudo systemctl stop lingtu`                            |
| Restart SLAM after drift          | `sudo systemctl restart robot-fastlio2 robot-localizer` |
| Tail algorithm logs               | `journalctl -u lingtu -f`                               |
| Tail SLAM logs                    | `journalctl -u robot-fastlio2 -f`                       |
| Verify ports                      | `ss -tnlp \| grep -E '13145\|5050\|8090'`               |
| One-screen status                 | `lingtu status`                                         |
| Watch status (1 Hz refresh)       | `lingtu watch`                                          |

---

## Diagnostics

### `lingtu` won't start

```bash
journalctl -u lingtu -n 80 --no-pager
sudo systemctl status robot-fastlio2     # depends on this; lingtu won't be useful without it
ss -tnlp | grep -E "5050|8090"           # leftover process holding the port?
```

### LiDAR no data

```bash
journalctl -u robot-lidar -n 40
lingtu doctor
ros2 topic hz /nav/lidar_scan
ros2 topic hz /nav/imu
ip -br addr | grep 192.168.1            # MID-360 host sub-net, usually eth0=192.168.1.5
```

If `lingtu doctor` reports active legacy `lidar.service`, `slam.service`, or
`localizer.service`, stop the duplicate stack before restarting the production
`robot-*` services. Two Livox drivers will fight for the same UDP ports and can leave
`/nav/lidar_scan` and `/nav/imu` with zero publishers.

Legacy command references from older docs are wrong for the current stack:

```bash
journalctl -u robot-lidar -n 40
ros2 topic hz /nav/lidar_scan            # not /livox/lidar in production
```

### Drift / lost localization

```bash
ros2 topic hz /nav/odometry              # expect live odometry
ros2 topic hz /nav/map_cloud             # expect live map cloud in mapping/localization
ros2 topic echo /localization_quality
sudo systemctl restart robot-fastlio2 robot-localizer
```

The Gateway has a SLAM drift watchdog (`src/gateway/gateway_module.py`
`_drift_watchdog_loop`) that auto-restarts SLAM services when xy or velocity diverge.
See `docs/archive/05-specialized/slam_drift_watchdog.md`.

### `robot-brainstem` (port 13145) unresponsive

```bash
journalctl -u robot-brainstem -n 30
sudo systemctl restart robot-brainstem
```

ThunderDriver and the Gateway control plane both wait for this gRPC server.

### Stale processes after a crash

```bash
ss -tnlp | grep -E "13145|5050|8090"
ps aux | grep -E "lingtu|fastlio|localizer|dart"
sudo systemctl restart lingtu
```

---

## Pinned references

- `GOVERNANCE.md` — six principles for deployment hygiene (historical 2026-04 cleanup)
- `S100P_STACK_INVENTORY.md` — what was on the robot before consolidation, and why
  the stack looks the way it does today
- `OTA_GUIDE.md` — OTA design, signing, manifest schema, rollback
- `lingtu_cli.md` — operations CLI subcommands
- `services/` — actual systemd unit files installed on the robot
