# Supervision Benchmark Baseline

**Date**: 2026-04-05
**Phase**: Phase 1 baseline freeze
**Status**: frozen for the first controlled migration slice

## Purpose

This document freezes the current benchmark setup before any `supervision`
integration refactor. Later phases may change rendering, video IO, and
analytics plumbing, but this baseline defines what must remain comparable.

## Repos And Scripts In Scope

- `D:\inovxio\products\nova-rws\tests\tracking_benchmark\run_tracking_test.py`
- `D:\inovxio\products\nova-rws\tests\tracking_benchmark\run_roboflow_test.py`
- `D:\inovxio\products\nova-rws\tests\tracking_benchmark\benchmark_backbone.py`

## Local Environment Snapshot

- `supervision`: `0.27.0.post1`
- `ultralytics`: `8.4.21`
- `nova-rws` dependency floor: `ultralytics>=8.3.0`
- No pinned `supervision` dependency was found in `nova-rws`.

## Benchmark Assets Present

Located in `D:\inovxio\products\nova-rws\tests\tracking_benchmark\`:

- `test_people.mp4`
- `test_market.mp4`
- `test_subway.mp4`
- existing output videos:
  - `output_A_baseline.mp4`
  - `output_B_reid_best.mp4`
  - `output_C_reid_cmc.mp4`
  - `output_deepsort.mp4`
  - legacy comparison outputs such as `output_botsort*.mp4`

## Baseline Lane A: qp_perception Search And Full Run

Source script: `run_tracking_test.py`

Frozen defaults:

- model: `yolo11n-seg.pt`
- detector confidence: `0.35`
- tracker config: `botsort.yaml`
- class filter: `["person"]`
- device: auto (`""`)
- Kalman enabled via `KalmanCAConfig()`

Search window:

- `220` frames

Final exported lanes:

- `A:Baseline`
- `B:<best_reid_config>`
- `C:ReID+CMC`

Frozen output files:

- `output_A_baseline.mp4`
- `output_B_reid_best.mp4`
- `output_C_reid_cmc.mp4`

Frozen primary metrics:

- total frames
- wall time
- average inference latency
- p95 inference latency
- unique IDs
- long tracks `>30` frames
- short tracks `<10` frames
- average track length
- fragmentation breaks
- average break gap
- Re-ID recoveries
- composite score

Frozen scoring formula:

- reward lower unique-ID count relative to baseline
- reward longer average tracks
- penalize fragmentation
- penalize latency

## Baseline Lane B: Roboflow DeepSORT Benchmark

Source script: `run_roboflow_test.py`

Frozen defaults:

- detector model: `yolo11n.pt`
- detected classes: `[0, 2, 3, 5]`
- tracker: `DeepSORT` from `trackers`
- `supervision` already used for:
  - `sv.Detections.from_ultralytics(...)`
  - `sv.BoxAnnotator()`
  - `sv.LabelAnnotator()`

Frozen output file:

- `output_deepsort.mp4`

Frozen primary metrics:

- total frames
- average FPS
- average inference time
- unique track IDs
- per-ID span, coverage, and gap list

## Current Comparison Gap

The two benchmark lanes are not yet directly comparable:

- `run_tracking_test.py` is person-only and uses `yolo11n-seg.pt`.
- `run_roboflow_test.py` is multi-class and uses `yolo11n.pt`.
- `run_tracking_test.py` has richer fragmentation and composite scoring.
- `run_roboflow_test.py` already uses `supervision` for detections and drawing.

This mismatch is intentional for Phase 1. The goal here is only to freeze the
starting point before refactor.

## Migration Guardrails

The first `supervision` migration slice must preserve:

- existing `run_tracking_test.py` benchmark semantics
- existing score computation
- existing output lane naming for A/B/C runs
- current `qp_perception` tracker behavior

The first slice may change:

- video read and write plumbing
- annotation implementation
- detection and track export format used by the benchmark

The first slice must not change:

- runtime `qp_perception` tracking logic
- Re-ID gallery logic
- no-fire-zone runtime safety logic

## Next Controlled Step

Phase 2 may start only after this baseline is accepted:

1. add `Track -> sv.Detections` adapter
2. refactor `run_tracking_test.py` rendering and video IO
3. compare outputs against this frozen baseline
