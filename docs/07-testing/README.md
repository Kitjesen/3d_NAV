# LingTu Regression Suite

Three layers, from local commit to S100P field run.

| Layer | Trigger | Content | Time | Required |
|---|---|---|---|---|
| L1 pre-commit hook | `git commit` | `pytest src/core/tests/ -q` must pass | ~90 s | Yes |
| L2 pre-push hook | `git push` | L1 plus `stub` profile smoke build | ~30 s extra | Yes |
| L3 S100P weekly | Friday afternoon, manual | Run `p0_*.sh` four scripts and capture video | ~30 min | Yes |

---

## Install local L1 / L2 hooks

```bash
# One-shot install into .git/hooks/ (not version-controlled)
bash docs/07-testing/install_hooks.sh
```

After install:
- `git commit` runs `pytest src/core/tests/ -q`; the commit aborts on failure.
- `git push` runs pytest plus a `full_stack_blueprint(profile="stub").build().start()` smoke check.

Bypass (emergencies only):

```bash
git commit --no-verify -m "..."
git push --no-verify
```

The PR description must state the reason; the reviewer will challenge it.

---

## L3 P0 scripts on the robot

Each P0 script is self-contained, uses `set -e`, and writes its log to `~/data/nav_logs/YYYYMMDD_HHMMSS_<script>.log`.

| Script | Coverage | Input | Pass criteria |
|---|---|---|---|
| `p0_cold_boot.sh` | System cold start | None | At least 20 modules up, Gateway `/api/v1/health` returns `status="ok"` and is stable for 3 minutes |
| `p0_mapping.sh` | Map -> save -> activate | Walk the robot for ~3 min | `map.pcd` + `tomogram.pickle` + `occupancy.npz` + `map.pgm` / `map.yaml` produced |
| `p0_goto.sh` | Click goal -> arrive | Pre-loaded active map | `mission_status=SUCCESS` or `EV_PATH_COMPLETE` event |
| `p0_estop.sh` | E-stop reflex | Operator presses dashboard E-stop | Within 100 ms `CmdVelMux` outputs `Twist.zero()`; watchdog log entry present |

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

## Rules

- Never run validation commands ad hoc over SSH; always go through a script.
- Never bypass pre-commit / pre-push without an emergency rationale in the PR.
- New PRs must update `BACKLOG.md` first.
- Items in `blocked` must be re-evaluated every Friday.

---

## Script inventory

- `install_hooks.sh` — install L1 / L2 git hooks.
- `p0_cold_boot.sh` — P0-01 cold start.
- `p0_mapping.sh` — P0-02 mapping.
- `p0_goto.sh` — P0-03 navigate to a point.
- `p0_estop.sh` — P0-04 emergency stop.
- `p0_all.sh` — chain P0-01 through P0-04.
- `p1_tare_explore.sh` — P1-01 TARE exploration (TBD).
- `p1_gnss_fusion.sh` — P1-02 GNSS fusion (TBD).
- `p1_follow_person.sh` — P1-04 OSNet person follow (TBD).
- `p1_degraded.sh` — P1-05 sensor degradation (TBD).

The single tracked baseline document remaining in this folder is `SUPERVISION_BENCHMARK_BASELINE_2026-04-05.md` (frozen for the supervision tracker migration).
