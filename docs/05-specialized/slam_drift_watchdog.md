# SLAM Drift Watchdog

> Background thread that detects and recovers Fast-LIO2's IEKF when long
> static periods make the covariance matrix diverge and `odom` ends up at
> 1e12 m.

## Background

Fast-LIO2 fuses LiDAR + IMU through an Iterated EKF. The Kalman filter
relies on new informative observations to keep the covariance matrix
bounded. When the dog stands still:

- LiDAR sees an unchanging set of walls and ground → information gain ≈ 0.
- IMU contributes only white noise, no systematic motion.
- `P` keeps inflating → numerical divergence → `xy` shoots to 10⁹–10¹² m.

Real-world cases observed on S100P:

- Day 1: `xy = (1 528 868, 276 184)` — 1.5 Mm.
- Day 2: `xy = (-3.97 × 10¹¹, 2.46 × 10¹²)` — trillions of metres.

Downstream impact:

- `dialogue.localization = DEGRADED (slam_weak)`.
- `mission.degeneracy = CRITICAL`.
- `safety.level = WARN`.
- `NavigationModule` plans from the diverged pose → PCT planner `start_idx`
  out of bounds → entire path rejected.
- If the operator does not notice and starts mapping, divergent odometry is
  baked into the saved PCD.

## Pre-watchdog Manual Recovery

Before this watchdog existed, recovery was an SSH ritual:

1. SSH into the dog.
2. `curl -X POST http://localhost:5050/api/v1/session/end`.
3. `sudo systemctl stop slam slam_pgo localizer`.
4. `sudo systemctl restart lingtu.service` (clears the gateway's cached
   diverged odom).
5. Wait ~10 s.
6. Re-trigger the mapping mode from the web UI.

Problem: who is going to SSH in? With nobody around, the dog "plays dead".

## Watchdog Design

### Periodic Check (every 60 s)

```python
# src/gateway/gateway_module.py — _drift_watchdog_loop
time.sleep(60)
with self._state_lock:
    odom = self._odom
x, y, z = abs(odom["x"]), abs(odom["y"]), abs(odom["z"])
v = abs(odom["vx"])
if x > 1000 or y > 1000 or z > 1000 or v > 10:
    self._drift_restart_do_restart()
```

Threshold rationale:

- Indoor working radius is ~100 m, so 1000 m is 10× the operating envelope.
- The quadruped peaks at ~3 m/s, so 10 m/s is far above any feasible motion.

### Recovery Sequence

```python
# _drift_restart_do_restart
svc = get_service_manager()
mode_before = self._session_mode
svc.stop("slam", "slam_pgo", "localizer")     # kill the IEKF for real
with self._state_lock:
    self._odom = {}                            # drop diverged cache
    self._odom_timestamps.clear()
self.push_event({"type": "slam_drift", "level": "error", ...})
time.sleep(2.0)
if mode_before == "mapping":
    svc.ensure("slam", "slam_pgo")
elif mode_before == "navigating":
    svc.ensure("slam", "localizer")
# idle: do nothing, services start when the user enters a mode
```

### Cooldown (300 s)

```python
if time.time() - self._drift_last_restart_ts < COOLDOWN:
    logger.warning("cooldown not elapsed, skipping")
    continue
```

If SLAM is genuinely broken (hardware, driver), this prevents a hot loop of
restarts every 60 s. The WARN-level logs keep flowing so an operator can
spot the persistent failure.

## Environment Variables

| Variable | Default | Meaning |
|----------|---------|---------|
| `LINGTU_DRIFT_WATCHDOG` | 1 | Master switch (0 disables) |
| `LINGTU_DRIFT_WATCHDOG_INTERVAL` | 60 | Check period (s) |
| `LINGTU_DRIFT_WATCHDOG_XY_LIMIT` | 1000 | xy magnitude trip (m) |
| `LINGTU_DRIFT_WATCHDOG_V_LIMIT` | 10 | linear-velocity trip (m/s) |
| `LINGTU_DRIFT_WATCHDOG_COOLDOWN` | 300 | Minimum seconds between restarts |

To override on S100P, edit `/etc/systemd/system/lingtu.service` with
`Environment=LINGTU_DRIFT_WATCHDOG_XY_LIMIT=500` then restart `lingtu.service`.

## Observability

### CLI

```bash
lingtu status        # [3] Robot panel shows "ODOM DIVERGED"
lingtu log drift     # journal lines from drift_watchdog
lingtu log tail      # rolling journalctl
```

### Journal

Startup:

```
drift_watchdog: enabled, interval=60s, |xy|<1000m, |v|<10.0m/s
```

Trigger (ERROR):

```
drift_watchdog: IEKF DIVERGED xy=(1528868,276184) z=5 v=0.1 — restarting slam.service
drift_watchdog: restart complete (mode=navigating)
```

Cooldown:

```
drift_watchdog: still diverged (xy=1528868,276184 v=0.1) but cooldown (245s) not elapsed
```

### SSE Event

Web clients can subscribe to `type=slam_drift`:

```json
{
  "type": "slam_drift",
  "level": "error",
  "xy": 1528868,
  "v": 0.1,
  "action": "slam_restart",
  "count": 1
}
```

(The web banner that consumes this event is currently a P2 TODO.)

## Key Files

| File | Role |
|------|------|
| `src/gateway/gateway_module.py:__init__` | `_drift_*` state vars + env parsing |
| `src/gateway/gateway_module.py:start` | Spawns `_drift_watchdog_thread` |
| `src/gateway/gateway_module.py:_drift_watchdog_loop` | Periodic check |
| `src/gateway/gateway_module.py:_drift_restart_do_restart` | stop / clear / ensure |
| `memory/slam_static_drift_bug.md` | Pre-watchdog manual playbook |

## Limitations (P3+ work)

- **Symptomatic, not preventive.** The IEKF still diverges; we just clean up.
  There is a 10–20 s outage per recovery cycle.
- **Cannot save in-progress mapping.** If divergence happens mid-session,
  the few seconds of patches recorded with bad poses are still bad.
- **No upstream Fast-LIO2 changes.** Real prevention would need a static
  detector + covariance clamp inside Fast-LIO2 (fork required):

  ```cpp
  // fastlio2/src/IKFOM_toolkit/esekfom.hpp
  if (imu_still_for_seconds(30)) {
      P = P.clamp_max(1e-3);
      // optional: reset twist portion
  }
  ```

  Out of scope for this repo.
