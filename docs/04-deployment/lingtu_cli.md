# `lingtu` Operations CLI

A single SSH-friendly entry point that replaces ad-hoc `curl`, `systemctl`, and
`journalctl` invocations. The script lives at `scripts/lingtu` and is deployed on the
robot at `/home/sunrise/data/SLAM/navigation/scripts/lingtu`.

This is the operations CLI run **on the robot**. The Python entry `lingtu.py` (with
profiles like `s100p`, `sim`, `dev`, `map`, `explore`, `stub`) is the application
itself, started by `lingtu.service`.

---

## Local alias (one-time setup)

```bash
# ~/.bashrc / ~/.zshrc
alias lingtu='ssh sunrise@192.168.66.190 "bash ~/data/SLAM/navigation/scripts/lingtu"'
alias lingwatch='ssh -t sunrise@192.168.66.190 "bash ~/data/SLAM/navigation/scripts/lingtu watch"'
```

After `source ~/.bashrc`:

```bash
lingtu status     # one-screen snapshot
lingwatch         # secondary screen, live refresh
```

---

## Subcommands

### `status` — 8-section snapshot

```
=== Lingtu @ 17:26:52 ===
[1] Session   mode=idle   map=corrected_20260406_224020   can_map=True
[2] SLAM      hz=10.0Hz   live_pts=0   loc=GOOD
[3] Robot     xy=(1.2, -0.4)   z=0.05   yaw=12.3 deg   v=0.0 m/s   w=0.0 rad/s
[4] Mission   state=IDLE   wp=0/0   replan=0   speed=1.0   deg=NONE
[5] Path      (no active plan)
[6] Ctrl      teleop=False(0)   safety=OK
[7] Map       active   @02:57:26   map.pcd=2.6M   predufo=-   patches=105
[8] Log       (recent drift / dufomap / error)
```

| Section       | Field                  | Healthy                            | Bad                            |
|---------------|------------------------|------------------------------------|--------------------------------|
| [1] Session   | `mode`                 | `idle` / `mapping` / `navigating`  | -                              |
| [2] SLAM      | `hz`                   | 20-47 Hz (mapping), 10 Hz (nav)    | 0 Hz means service is down     |
| [2] SLAM      | `loc`                  | `GOOD` / `OK`                      | `DEGRADED` / `LOST`            |
| [3] Robot     | `xy`                   | within ~+-50 m indoors             | `ODOM DIVERGED` => IEKF blew   |
| [4] Mission   | `state`                | `IDLE` / `EXECUTING`               | `FAILED`                       |
| [4] Mission   | `deg`                  | `NONE`                             | `CRITICAL`                     |
| [5] Path      | `pts/len/next/goal`    | populated when navigating          | -                              |
| [6] Ctrl      | `safety`               | `OK (0)` / `WARN (1)`              | `STOP (2/3)`                   |
| [7] Map       | `dufo -N pts (X%)`     | dynamic-removal stats present      | -                              |
| [8] Log       | drift / dufomap / err  | quiet                              | recent entries imply trouble   |

### `watch [interval]` — continuous monitor

```bash
lingtu watch       # 1 second
lingtu watch 2     # 2 seconds
```

Internally `watch -c -n <interval> bash <script> status`. Keep it on a side screen
during mapping / navigation runs.

### `map` — mapping session

```bash
lingtu map start              # enter mapping (starts robot-fastlio2 + slam_pgo)
lingtu map save lab_0423      # PGO save + DUFOMap clean + tomogram + occupancy rebuild
lingtu map end                # back to idle (stops SLAM)
lingtu map list               # list saved maps
lingtu map restore lab_0423   # restore predufo backup -> map.pcd (DUFOMap was too aggressive)
```

`save` takes 15-60 s. Toast / JSON return contains DUFOMap stats:

```json
{
  "success": true,
  "name": "lab_0423",
  "dynamic_filter": {
    "success": true,
    "orig_count": 170086,
    "clean_count": 169512,
    "dropped": 574,
    "elapsed_s": 0.8
  }
}
```

`restore` use case: DUFOMap removed static structure by mistake. `map.pcd.predufo` is
restored to `map.pcd`; the discarded `map.pcd` is preserved as
`map.pcd.replaced-<ts>` (double safety). After restoring, rebuild the tomogram and
occupancy grid:

```bash
curl -X POST -H 'Content-Type: application/json' \
    -d '{"action":"build_tomogram","name":"lab_0423"}' \
    http://localhost:5050/api/v1/maps
```

### `nav` — navigation session

```bash
lingtu nav start corrected_20260406_224020    # navigating + load map (starts robot-fastlio2 + robot-localizer)
lingtu nav goal 3.5 2.1 0.0                   # send map-frame goal (x, y, yaw)
lingtu nav stop                               # same as map end
```

### `svc` — systemd service control

```bash
lingtu svc status               # 6 core services: enabled / active
lingtu svc restart slam         # restart Fast-LIO2 (i.e. robot-fastlio2.service)
lingtu svc restart lingtu       # restart algorithm layer (clears odom cache)
lingtu svc restart all          # restart SLAM + lingtu
lingtu svc stop slam            # stop a single service
```

The aliases `slam` / `lingtu` / `lidar` / `camera` / `brainstem` / `localizer` map to
the corresponding systemd unit. There is no `slam.service` anymore; `slam` aliases
to `robot-fastlio2.service`.

### `log` — journalctl filters

```bash
lingtu log drift      # drift_watchdog firings
lingtu log dufomap    # DUFOMap save stats (last hour)
lingtu log error      # last 30 min ERROR-level (filters VectorMemory / WebRTC noise)
lingtu log tail       # live tail (Ctrl+C to exit)
lingtu log all        # last 10 min, everything
```

### `health` — REST passthrough

```bash
lingtu health         # GET /api/v1/health | python3 -m json.tool
```

Full module roster, per-sensor status, fine-grained Hz/FPS metrics.

---

## Workflows

### A. Mapping with dynamic-object validation

```bash
lingtu watch                                  # secondary screen
# then on the primary screen:
lingtu status                                 # confirm idle + can_map=True
lingtu map start                              # enter mapping
# wait 5-10 s; lingwatch should show mode=mapping, hz>20, robot xy=(0,0)
# drive the robot through the area; let a person walk through for ~15 s
lingtu map save lab_0423_test                 # ~30 s
# lingwatch [7] Map should show "dufo -N pts (X%)"
lingtu log dufomap | tail -5                  # confirm in the log
```

### B. Load a map and navigate

```bash
lingtu map list
lingtu nav start lab_0423_test                # navigating
# lingwatch shows mode=navigating, loc=GOOD
lingtu nav goal 3.5 2.1
# [5] Path populates, [4] Mission state=EXECUTING, wp counts up
# on success: state=SUCCEEDED
```

### C. Diagnostic playbook

```bash
# Case 1: ODOM DIVERGED on the Robot row
#   wait up to 60 s for the watchdog to auto-recover, or:
lingtu svc restart slam

# Case 2: map save failed
lingtu log error | tail -20
lingtu log dufomap

# Case 3: SLAM won't start
lingtu svc status
# if slam=inactive while mode=mapping:
lingtu map end && sleep 2 && lingtu map start
```

---

## Exit codes

| Code | Meaning                                  |
|------|------------------------------------------|
| 0    | success                                  |
| 1    | argument / usage error                   |
| 2    | runtime error (curl / ssh / systemctl)   |

---

## Related

- `docs/04-deployment/README.md` — deployment overview, service inventory
- `docs/05-specialized/dynamic_obstacle_removal.md` — DUFOMap Phase 1 + 2
- `docs/05-specialized/slam_drift_watchdog.md` — IEKF divergence watchdog
