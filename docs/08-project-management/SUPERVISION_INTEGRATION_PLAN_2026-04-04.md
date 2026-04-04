# Supervision Integration Plan

**Date**: 2026-04-04
**Scope**: `nova-rws` benchmark + `qp_perception` debug and analysis adapters
**Primary goal**: Introduce `roboflow/supervision` as the standard layer for benchmark video IO, visualization, counting, and zone analytics, without replacing the current `qp_perception` runtime tracker.

---

## Placement

This plan lives in `lingtu/docs/08-project-management/` because it is a cross-repo execution plan, not a single-module API note.

Key code paths covered by the plan:

- `D:\inovxio\products\nova-rws\tests\tracking_benchmark\run_tracking_test.py`
- `D:\inovxio\products\nova-rws\tests\tracking_benchmark\run_roboflow_test.py`
- `D:\inovxio\products\nova-rws\src\rws_tracking\perception\yolo_seg_tracker.py`
- `D:\inovxio\modules\perception\src\qp_perception\tracking\yolo_seg.py`
- `D:\inovxio\products\nova-rws\src\rws_tracking\safety\no_fire_zone.py`

---

## Decisions

- Use `supervision` as a benchmark, debug, and analytics layer.
- Keep `qp_perception` as the runtime tracking core.
- Do not replace current ReID, Kalman, ID remap, target ranking, or Hungarian allocation logic with `supervision`.
- Do not replace runtime safety no-fire-zone logic with `PolygonZone`.

---

## Phase Plan

### Phase 1: Baseline Alignment

Goal: make the current benchmark paths comparable before refactor.

Deliverables:

- Confirm one `supervision` version for benchmark use.
- Freeze benchmark inputs: video, detector preset, person-only filtering, output naming.
- Add a short benchmark matrix: `DeepSORT`, `ByteTrack`, `qp_perception`.

Exit criteria:

- Same test clip can run through all three benchmark lanes.
- Output videos and summary metrics are saved in a consistent layout.

### Phase 2: Adapter Layer

Goal: bridge `qp_perception` track output into `sv.Detections`.

Deliverables:

- Add a small adapter module that converts `Track` lists into `sv.Detections`.
- Preserve `bbox`, `confidence`, and `tracker_id`.
- Store extra fields like `mask_center`, `class_name`, and `source` in `detections.data`.

Exit criteria:

- `YoloSegTracker.detect_and_track(...)` results can be rendered by `supervision` annotators without custom drawing code.

### Phase 3: Benchmark Refactor

Goal: migrate hand-written benchmark video and annotation code to `supervision`.

Deliverables:

- Refactor `run_tracking_test.py` to use `VideoInfo`, `get_video_frames_generator`, `VideoSink` or `process_video`, and `FPSMonitor`.
- Replace manual `cv2.rectangle` and label blocks with `BoundingBoxAnnotator` or `RoundBoxAnnotator`, `LabelAnnotator`, and `TraceAnnotator`.
- Keep current fragmentation, gap, unique-ID, and composite-score logic unchanged.

Exit criteria:

- Benchmark output is visually equivalent or better.
- Benchmark script is shorter and easier to extend.

### Phase 4: Counting And Zone Analytics

Goal: add reusable crowd-flow and occupancy metrics on top of tracked detections.

Deliverables:

- Add optional `LineZone` support for in/out counting.
- Add optional `PolygonZone` support for occupancy and region dwell statistics.
- Define 2-3 standard benchmark scenes: doorway, corridor crossing, dense crowd region.

Exit criteria:

- Benchmark reports can output line-crossing counts and region occupancy numbers.
- Zone definitions are externalized and not hardcoded into every script.

### Phase 5: Runtime Debug Hook

Goal: expose `supervision` only as a debug and export layer in runtime-adjacent tools.

Deliverables:

- Add a debug-only export path from `qp_perception` outputs to `sv.Detections`.
- Reuse the same annotator stack for offline replay and paper or demo assets where useful.
- Keep runtime dependencies optional and non-blocking.

Exit criteria:

- Runtime core still works without `supervision`.
- Offline replay and debug tools can share one rendering pipeline.

### Phase 6: Verification And Rollout

Goal: make the migration safe and measurable.

Deliverables:

- Add unit tests for the adapter layer.
- Add at least one smoke test for benchmark scripts.
- Produce a before and after comparison on the same clip:
  - FPS
  - unique IDs
  - fragmentation
  - average track length
  - line count and zone occupancy where enabled

Exit criteria:

- No regression in current `qp_perception` tracking behavior.
- The benchmark layer is simpler and the outputs are more informative.

---

## Work Breakdown

### Track A: Immediate

- Create `qp_perception Track -> sv.Detections` adapter.
- Refactor `run_tracking_test.py`.
- Add `ByteTrack` baseline lane.

### Track B: Next

- Add `LineZone` and `PolygonZone` analytics.
- Externalize zone definitions.
- Standardize benchmark report output.

### Track C: Later

- Reuse the same rendering stack in paper and demo asset generation.
- Add debug replay hooks near runtime tooling.

---

## Non-Goals

- Replacing `YoloSegTracker` with `sv.ByteTrack`.
- Replacing `NoFireZoneManager` with image-space polygon logic.
- Moving `supervision` into the hard real-time control path.

---

## Recommended File Ownership

- Adapter and perception-side glue:
  - `D:\inovxio\modules\perception\src\qp_perception\tracking\...`
- Benchmark refactor and analytics:
  - `D:\inovxio\products\nova-rws\tests\tracking_benchmark\...`
- Runtime debug hooks only when needed:
  - `D:\inovxio\brain\lingtu\src\semantic\perception\...`

---

## First Execution Slice

Start with the smallest slice that proves the approach:

1. Add the adapter.
2. Refactor `run_tracking_test.py` to `supervision` video IO and annotators.
3. Keep the existing scoring logic.
4. Run the same clip and compare outputs against the current benchmark.

If this slice is clean, then proceed to zones and counting.
