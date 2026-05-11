# LingTu Regression Suite

Four layers, from local commit to simulation closure to S100P field run.

Commit and push acceptance criteria live in
[`COMMIT_PUSH_POLICY.md`](COMMIT_PUSH_POLICY.md). The short version is:
commit messages follow the Lore protocol in `AGENTS.md`, local commits must pass
the L1 test gate, and pushes must pass the L2 gate plus any focused checks for
the touched subsystem.

| Layer | Trigger | Content | Time | Required |
|---|---|---|---|---|
| L1 pre-commit hook | `git commit` | `pytest src/core/tests/ -q` must pass | ~90 s | Yes |
| L2 pre-push hook | `git push` | L1 plus `stub` profile build/start smoke | ~30 s extra | Yes |
| L2.5 server simulation closure | Before field claims or navigation demos | `server_sim_closure.py` strict summary over the relevant simulation gates | Host-dependent | Yes for simulation-backed navigation claims |
| L3 S100P weekly | Friday afternoon, manual | Run `p0_*.sh` four scripts and capture video | ~30 min | Yes |

---

## Install local L1 / L2 hooks

```bash
# One-shot install into .git/hooks/ (not version-controlled)
bash docs/07-testing/install_hooks.sh
```

After install:
- `git commit` runs `pytest src/core/tests/ -q`; the commit aborts on failure.
- `git push` runs pytest plus a stub `full_stack_blueprint(...).build()` graph smoke and a minimal offline stub `build().start()` lifecycle smoke.

Bypass (emergencies only):

```bash
git commit --no-verify -m "..."
git push --no-verify
```

The PR description must state the reason; the reviewer will challenge it.

---

## L2.5 server simulation closure

Use this layer before claiming that navigation, localization, planning,
tracking, exploration, or Gateway command safety works beyond unit tests. It is
hardware-free evidence only: it must report `simulation_only=true`,
`real_robot_motion=false`, and `cmd_vel_sent_to_hardware=false`.

Full closure summary:

```bash
PYTHONPATH=src:. python sim/scripts/server_sim_closure.py \
  --json-out artifacts/server_sim_closure_summary_all.json \
  --strict
```

The summary is an aggregator. If it reports `missing_or_failed`, run the
command printed for each failed gate, then rerun the summary.
When a command uses `--required` to validate only a subset, non-required
failures appear as `optional_missing_or_failed`; that output is useful setup
evidence but not a full closure pass.
Each accepted report includes `report_mtime` and `report_age_s`. Add
`--max-report-age-s <seconds>` when a review or release gate needs to reject
stale artifacts instead of only displaying their age.

The current full gate set covers:

- multi-floor exploration, LiDAR localization contract, native PCT, local planning, and nav_core tracking;
- large-terrain global planning and path-safety checks;
- native PCT through ROS2 local planner/path follower into MuJoCo motion;
- dynamic-obstacle local planner replanning;
- live MuJoCo LiDAR/IMU through Fast-LIO2 and SlamBridge;
- ONNX policy navigation dataflow;
- Gateway dry-run command safety;
- routecheck preflight without publishing goal, cmd_vel, or stop;
- ROS-native Gazebo frame/topic smoke;
- saved-map relocalization contract.

Server bootstrap verification uses the setup-safe subset it produces itself:

```bash
bash scripts/deploy/setup_server_ros_pct.sh
```

That script writes `artifacts/server_sim_closure_summary_setup.json` for the
multi-floor and routecheck-preflight gates. It is not a substitute for the full
closure summary unless those are the only changed surfaces.

---

## L3 P0 scripts on the robot

Each P0 script is self-contained, uses `set -e`, and writes its log to `~/data/nav_logs/YYYYMMDD_HHMMSS_<script>.log`.

| Script | Coverage | Input | Pass criteria |
|---|---|---|---|
| `p0_cold_boot.sh` | System cold start | None | At least 20 modules up, Gateway `/api/v1/health` returns `status="ok"` and is stable for 3 minutes |
| `p0_mapping.sh` | Map -> save -> activate | Walk the robot for ~3 min | MapManager `/api/v1/maps` save and `set_active` succeed; `map.pcd` + `tomogram.pickle` + `occupancy.npz` + `map.pgm` / `map.yaml` are produced |
| `p0_goto.sh` | Point goal -> arrive | Pre-loaded active map | `/api/v1/navigation/status` reports `state="SUCCESS"` |
| `p0_estop.sh` | Gateway stop reflex | Operator starts non-zero robot motion, then presses Enter in the script | `POST /api/v1/stop` succeeds and `/api/v1/state` reports navigation speed below `0.01 m/s` for 3 consecutive ticks within 2 seconds |

### Run on Sunrise (S100P)

```bash
ssh sunrise@192.168.66.190
cd ~/data/SLAM/navigation
git pull --ff-only origin main
bash docs/07-testing/p0_all.sh | tee ~/data/nav_logs/$(date +%Y%m%d_%H%M%S)_p0_all.log
```

After the run:
1. Record `PASS / FAIL / BLOCKED` plus reason in `vault/实机记录/YYYY-MM-DD.md`.
2. Update the matching item in `BACKLOG.md` (status + last-on-robot date).
3. Failure cases get their own ticket under `docs/archive/08-project-management/known-issues/` (when that location is created).

---

## Simulation-only policy soak on Sunrise

Use Sunrise as a compute host for MuJoCo policy validation only. This gate must
not start robot services, publish real robot commands, or run the field
`scripts/lingtu map|nav` commands.

Run it after changing MuJoCo assets, ONNX policy loading, `PolicyRunner`,
`MujocoDriverModule`, or sim navigation wiring:

```bash
ssh sunrise@192.168.66.190
cd ~/data/SLAM/navigation
mkdir -p artifacts
PYTHONPATH=src:. PYTHONIOENCODING=utf-8 LINGTU_SIM_DRIVE_MODE=policy \
  python sim/scripts/policy_nav_smoke.py \
    --world open_field \
    --direct-duration 6 \
    --nav-duration 60 \
    --goal-distance 1.0 \
    --json-out artifacts/policy_nav_smoke_open_field.json
```

Expected pass evidence: policy metadata shows `obs_history [1, 285]` to
`actions [1, 16]`, direct and full-stack nav motion both exceed the configured
thresholds, `nav_state` reaches `SUCCESS`, and `direct_fallback` remains zero.
The detailed runbook is in `sim/README.md`.

---

## Rules

- Never run validation commands ad hoc over SSH; always go through a script.
- Never bypass pre-commit / pre-push without an emergency rationale in the PR.
- New PRs must update `BACKLOG.md` first.
- Items in `blocked` must be re-evaluated every Friday.

---

## Script inventory

- `install_hooks.sh` - install L1 / L2 git hooks.
- `p0_cold_boot.sh` - P0-01 cold start.
- `p0_mapping.sh` - P0-02 mapping.
- `p0_goto.sh` - P0-03 navigate to a point.
- `p0_estop.sh` - P0-04 emergency stop.
- `p0_all.sh` - chain P0-01 through P0-04.
- `p1_tare_explore.sh` - P1-01 TARE exploration (TBD).
- `p1_gnss_fusion.sh` - P1-02 GNSS fusion (TBD).
- `p1_follow_person.sh` - P1-04 OSNet person follow (TBD).
- `p1_degraded.sh` - P1-05 sensor degradation (TBD).

The single tracked baseline document remaining in this folder is `SUPERVISION_BENCHMARK_BASELINE_2026-04-05.md` (frozen for the supervision tracker migration).
