# HSG-Nav: Experiments Section Draft (DEPRECATED)

> **注意**: 本文件已被 `docs/09-paper/04_experiments.md` 替代。
> 新版实验章节包含更完整的 baseline 对比、消融设计、分析框架，
> 并与 `docs/09-paper/01_abstract_intro.md` ~ `05_conclusion_refs.md` 形成完整论文。
> 请参考 `docs/09-paper/README.md` 查看最新进展。

---

# 以下为旧版内容 (保留供参考)

## 5 Experimental Setup

### 5.1 Platform

Our system runs on a **Go2 quadruped robot** (Unitree Robotics) equipped with:

- **Compute**: Jetson Orin NX 16GB
- **Perception**: Orbbec Femto RGB-D camera (640×480 @ 30fps)
- **SLAM**: Fast-LIO2 with IMU-LiDAR fusion
- **Navigation**: Nav2 stack with DWB local planner

All perception and planning modules run onboard (no cloud dependency except LLM API calls).

### 5.2 Test Environment

Indoor office-corridor environment (~200 m² traversable area):

- 3 connected rooms (office, meeting room, storage)
- 1 corridor (~15m) connecting all rooms
- ~30 distinct objects (doors, chairs, desks, fire extinguishers, signs, etc.)
- Dynamic elements: people walking, doors opening/closing, objects being moved

### 5.3 Instruction Set

We design a **3-level instruction benchmark** to systematically evaluate each capability:

| Level | Description | Count | Example |
|-------|------------|-------|---------|
| L1 (Simple) | Single-target navigation | 20 | "find the fire extinguisher" |
| L2 (Spatial) | Spatial relation conditions | 15 | "find the fire extinguisher near the door" |
| L3 (Multi-step) | Sequential / conditional | 10 | "go to the door, then find the fire extinguisher nearby" |

Each instruction is annotated with start position, ground truth target position, and arrival radius.

### 5.4 Evaluation Metrics

- **SR** (Success Rate): navigation endpoint within 1m of target
- **SPL** (Success weighted by Path Length): SR × (shortest_path / actual_path)
- **Avg Time**: mean navigation duration (seconds)
- **Avg Steps**: mean exploration steps before reaching target
- **Failure Analysis**: categorized as detection / scene_graph / planning / motion / timeout

### 5.5 Baselines and Ablations

| Configuration | Description | Validates |
|--------------|-------------|-----------|
| **Full HSG-Nav** | Complete system | Baseline |
| w/o SceneGraph | CLIP-only matching (CoW-style) | Value of scene graph |
| w/o Hierarchy | Flat object list, no Room/Group | Value of hierarchical reasoning |
| w/o RePerception | Disable arrival + continuous re-perception | Value of re-perception |
| w/o ClosedLoop | Disable execution context + replan | Value of closed-loop feedback |
| w/o Exploration | No frontier exploration | Value of active exploration |

Each ablation uses L1 full set (20 instructions × 3 trials = 60 trials).

---

## 6 Results

### 6.1 Main Results

**Table 1: Overall Performance (L1/L2/L3)**

| Config | L1 SR | L1 SPL | L2 SR | L2 SPL | L3 SR | L3 SPL | Avg Time |
|--------|-------|--------|-------|--------|-------|--------|----------|
| Full HSG-Nav | _TBD_ | _TBD_ | _TBD_ | _TBD_ | _TBD_ | _TBD_ | _TBD_ |

**Key observation**: _(To be filled after experiments)_

### 6.2 Ablation Study

**Table 2: Ablation Results on L1 (20 × 3 trials)**

| Config | SR ↑ | SPL ↑ | Avg Time ↓ | Δ SR |
|--------|------|-------|------------|------|
| Full HSG-Nav | _TBD_ | _TBD_ | _TBD_ | — |
| w/o SceneGraph | _TBD_ | _TBD_ | _TBD_ | _TBD_ |
| w/o Hierarchy | _TBD_ | _TBD_ | _TBD_ | _TBD_ |
| w/o RePerception | _TBD_ | _TBD_ | _TBD_ | _TBD_ |
| w/o ClosedLoop | _TBD_ | _TBD_ | _TBD_ | _TBD_ |
| w/o Exploration | _TBD_ | _TBD_ | _TBD_ | _TBD_ |

**Expected findings**:

1. **w/o SceneGraph** → Largest SR drop. Validates that structured scene representation is crucial for grounding.
2. **w/o Hierarchy** → Moderate SR drop on L2/L3 (spatial/multi-step need room-level reasoning).
3. **w/o RePerception** → SR drop especially in dynamic scenarios (moved objects).
4. **w/o ClosedLoop** → SR drop on L3 (multi-step requires feedback accumulation).
5. **w/o Exploration** → SR drop proportional to how many targets need exploration vs. already visible.

### 6.3 Dynamic Scene Test

To validate Re-perception (Innovation 3), we conduct a special test:

- 5 L1 instructions where the target object is **physically moved** between trials
- Compare Full vs. w/o RePerception

**Table 3: Dynamic Scene Results**

| Config | SR | SPL | Notes |
|--------|-----|-----|-------|
| Full HSG-Nav | _TBD_ | _TBD_ | Re-perception detects stale targets |
| w/o RePerception | _TBD_ | _TBD_ | Navigates to outdated position |

### 6.4 Hierarchical Scene Graph Quality

**Online vs. Offline Graph Comparison**:

To validate Innovation 1 (online incremental HSG), we compare:

- **Online HSG**: graph built during 2 minutes of exploration
- **Offline HSG**: graph built from full environment scan (simulating FSR-VLN)

| Metric | Online (2min) | Offline (full scan) |
|--------|---------------|---------------------|
| Objects detected | _TBD_ | _TBD_ |
| Room naming accuracy | _TBD_ | _TBD_ |
| L1 SR using graph | _TBD_ | _TBD_ |

### 6.5 Edge Deployment Performance

**Table 4: Jetson Orin NX 16GB Performance**

| Module | Metric | Value | Target | Pass |
|--------|--------|-------|--------|------|
| YOLO-World (L) | FPS | _TBD_ | > 10 | _TBD_ |
| CLIP (ViT-B/32) | Latency | _TBD_ ms | < 50ms | _TBD_ |
| Scene Graph Build | Latency | _TBD_ ms | < 100ms | _TBD_ |
| Fast Path | Latency | _TBD_ ms | < 200ms | _TBD_ |
| Slow Path (LLM) | Latency | _TBD_ ms | record | — |
| GPU Utilization | Peak | _TBD_% | — | — |
| Memory | Peak | _TBD_ MB | < 12GB | _TBD_ |

### 6.6 Quadruped-Specific Challenges

**Table 5: Quadruped Platform Metrics**

| Challenge | Metric | Value |
|-----------|--------|-------|
| Low viewpoint occlusion | Objects occluded rate | _TBD_% |
| Walking vibration | Blur frame ratio (Laplacian < threshold) | _TBD_% |
| Detection jitter | False positive rate due to motion | _TBD_% |

---

## 7 Analysis

### 7.1 Why Hierarchical Reasoning Matters

_(To be written based on qualitative analysis of L2/L3 results showing Room→Group→Object reasoning chain)_

### 7.2 When Re-perception Helps

_(To be written based on dynamic scene test + failure case analysis)_

### 7.3 Comparison with FSR-VLN

While direct comparison is not possible (FSR-VLN uses pre-built maps, static env, single-step instructions, humanoid robot), we make a **fair qualitative comparison**:

| Dimension | FSR-VLN | HSG-Nav |
|-----------|---------|---------|
| Environment | Pre-scanned, static | Unknown, dynamic |
| Instructions | Single-step "go to X" | Multi-step, spatial conditions |
| Platform | LoCoBot (humanoid) | Go2 (quadruped) |
| Scene Graph | HMSG (offline, 30min build) | Online incremental HSG |
| Compute | Desktop GPU | Jetson Orin NX (edge) |
| Best L1 SR | 92% (on R2R val unseen) | _TBD_% (real-world) |

**Key argument**: FSR-VLN's 92% SR is on simplified conditions. HSG-Nav operates in significantly harder conditions (unknown env, multi-step, dynamic, edge compute) while achieving _TBD_% SR.

### 7.4 Limitations

1. LLM API latency adds ~1-3s to Slow Path decisions
2. YOLO-World open-vocabulary detection has lower accuracy than closed-set detectors
3. Quadruped vibration reduces effective perception range
4. Current evaluation uses a single indoor environment

---

## Appendix: Experimental Reproducibility

All experiment configurations, instruction sets, evaluation scripts, and ablation configs are available in `experiments/`:

```
experiments/
├── instruction_set.json      # L1/L2/L3 instructions with GT
├── ablation_configs.yaml     # 5 ablation configurations
├── eval_runner.py            # Automated evaluation framework
├── jetson_benchmark.py       # Performance benchmark script
└── results/                  # Raw results (after running)
```
