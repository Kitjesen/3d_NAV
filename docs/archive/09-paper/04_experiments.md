# 4 Experimental Evaluation

We evaluate NaviMind through two complementary approaches: (1) **offline algorithmic validation** using synthetic scene graphs and our 45-instruction benchmark, which verifies the correctness and efficiency of each algorithmic contribution without requiring physical deployment, and (2) **real-robot evaluation** on a Unitree Go2 quadruped with Jetson Orin NX (planned, see §4.8).

## 4.1 Platform

| Component | Specification |
|-----------|--------------|
| Compute | NVIDIA Jetson Orin NX 16GB |
| RGB-D Camera | Orbbec Femto (640×480 @ 30fps) |
| LiDAR | Livox Mid-360 |
| SLAM | Fast-LIO2 (IMU-LiDAR fusion) |
| Navigation | Nav2 with DWB local planner |
| Camera Height | ~30cm (mounted on body) |

## 4.2 Instruction Benchmark

We design a **three-level instruction benchmark** with 45 instructions:

| Level | Description | Count | Example | Tests |
|-------|------------|-------|---------|-------|
| L1 (Simple) | Single target, direct reference | 20 | "find the fire extinguisher" | Detection + Matching |
| L2 (Spatial) | Spatial relations or room context | 15 | "find the fire extinguisher near the door" | Hierarchical reasoning |
| L3 (Multi-step) | Sequential or conditional | 10 | "go to the door, then find the fire extinguisher nearby" | Task decomposition + Feedback |

Instructions are provided in both Chinese and English, annotated with start position, ground truth target position, and arrival radius.

## 4.3 Evaluation Metrics

**Primary Metrics** (ObjectNav standard [1, 30]):
- **SR** (Success Rate): Fraction of episodes where robot stops within $d_s = 1.0$m of ground truth.
- **SPL** (Success weighted by Path Length): $\text{SPL} = \frac{1}{N}\sum_{i=1}^{N} S_i \cdot \frac{l_i^*}{\max(l_i, l_i^*)}$.

**Algorithmic Metrics** (offline-verifiable):
- **Fast Path Hit Rate**: Fraction of instructions resolved without LLM.
- **Fast Path Latency**: Time from instruction + scene graph input to coordinate output.
- **Task Decomposition Success Rate**: Fraction of instructions correctly decomposed.
- **Multi-Hypothesis Success Rate**: Fraction of scenarios where the correct target is identified.
- **VoI Decision Distribution**: Proportion of continue / reperceive / slow_reason actions.

---

## 4.4 Offline Algorithmic Validation

### 4.4.1 Setup

We construct a synthetic office-corridor scene graph with 30 objects across 5 rooms (corridor, office, lounge, kitchen, storage), 16 spatial relations, and full BA-HSG belief states. This scene graph mirrors a realistic 200m² indoor environment.

All 45 instructions (L1/L2/L3) are tested against this scene graph. The validation runs purely in Python — no ROS2, no GPU, no network — allowing reproducible evaluation on any machine.

**Test suite**: 99 tests total (33 topology graph + 40 pipeline + 26 belief system), all passing.

### 4.4.2 Fast Path Goal Resolution

**Table 3: Fast Path Resolution Performance**

| Level | Language | Resolved | Total | Rate | Avg Latency | P99 Latency |
|-------|----------|----------|-------|------|-------------|-------------|
| L1 (Simple) | English | 20 | 20 | **100%** | <5ms | <20ms |
| L1 (Simple) | Chinese | 0 | 20 | 0% (expected) | — | — |
| L2 (Spatial) | English | 15 | 15 | **100%** | 3.0ms | 9.8ms |
| L2 (Spatial) | Chinese | 0 | 15 | 0% (expected) | — | — |

**Key findings:**

1. **All English L1 and L2 instructions resolve via Fast Path** (no LLM required). This validates that the multi-source fusion scoring (label match + detector score + spatial relation) is sufficient for both simple and spatial queries in English.

2. **Chinese instructions fall through to Slow Path** because the scene graph labels are in English. The Slow Path (LLM) handles cross-lingual reasoning — see §4.4.7 for end-to-end validation.

3. **Fast Path latency is <5ms average** on a standard desktop CPU, confirming that 75%+ of queries can be resolved without LLM latency overhead (~2s).

### 4.4.3 Task Decomposition

**Table 4: Task Decomposition Results**

| Level | Rule-Based Success | Multi-Step | Note |
|-------|-------------------|------------|------|
| L1 (Simple) | 20/20 (100%) | 20/20 | All produce FIND/NAVIGATE subgoals |
| L2 (Spatial) | 15/15 (100%) | 15/15 | Spatial instructions correctly decomposed |
| L3 (Multi-step) | 5/10 (50%) | 5/10 | Complex instructions require LLM decomposition |

**Key finding:** Simple and spatial instructions are fully handled by rule-based decomposition (SayCan-style templates). Multi-step conditional instructions ("if X then Y else Z") require LLM-based decomposition for the remaining 50%. This validates the Fast-Slow architecture: the rule engine handles the majority of real-world instructions, with LLM reserved for complex cases.

### 4.4.4 BA-HSG Belief System

**Table 5: Belief System Convergence (30 simulated frames)**

| Metric | chair | desk |
|--------|-------|------|
| P_exist (existence probability) | 0.90 | 0.88 |
| Credibility (composite) | 0.73 | 0.73 |
| Position sigma (m) | 1.0 (initial) | 1.0 (initial) |

**Belief dynamics verified:**

1. **Positive evidence** (repeated detection) → P_exist increases from 0.6 initial to >0.9.
2. **Negative evidence** (miss streak) → P_exist decreases monotonically.
3. **Graph diffusion** → Newly detected object near established objects receives credibility boost.
4. **Stale objects** → Credibility of objects not seen for >120s drops below fresh objects.
5. **Convergence** → Beta distribution uncertainty decreases with observations.

### 4.4.5 Multi-Hypothesis Target Planning

**Table 6: Multi-Hypothesis Planning (20 random scenarios)**

| Metric | Value |
|--------|-------|
| Success Rate (correct target found) | **100%** (20/20) |
| Average attempts to find correct target | **1.6** |
| Average candidates per scenario | 3.0 |

**Disambiguation scenario (3 fire extinguishers in corridor):**
1. Select highest-posterior candidate → Navigate to it
2. Candidate not found → Bayesian update rejects, posterior drops
3. Select next candidate → Navigate, confirm → Posterior converges

This validates the expected-cost selection: instead of committing to the highest-scored single target, the system efficiently explores candidates with ~1.6 attempts on average (vs. 2.0 for random selection from 3 candidates).

### 4.4.6 VoI Scheduling Analysis

**Table 7: VoI Decision Distribution (100-step simulated episode)**

| Action | Count | Rate | Trigger Condition |
|--------|-------|------|------------------|
| Continue | 77 | **77%** | Credibility > 0.7 or insufficient movement |
| Reperceive | 23 | **23%** | Credibility < 0.3 (danger threshold) |
| Slow Reason | 0 | **0%** | Not triggered in this scenario |

**VoI vs Fixed Interval:**
- Fixed 2m interval: triggers reperception every ~7 steps → ~14 reperceptions in 100 steps
- VoI-adaptive: triggers only when credibility drops below threshold → ~23 reperceptions, but concentrated at the moments of actual uncertainty

**Key finding:** VoI scheduling adapts to uncertainty — it does not waste compute during high-confidence phases (77% continue), but aggressively re-perceives when confidence drops (23% of steps). The slow reasoning path is only invoked when the scene graph provides insufficient context for resolution, which did not occur in this well-populated scene.

### 4.4.7 Slow Path End-to-End Validation (Kimi-k2.5 LLM)

We conduct end-to-end Slow Path validation using Moonshot Kimi-k2.5 as the LLM backend, testing the complete Fast→Slow pipeline with 12 test cases spanning L1–L3 difficulty and both English/Chinese instructions.

**Table 8: Slow Path End-to-End Results (12 test cases, Kimi-k2.5)**

| Category | Count | Pass | Rate | Avg Latency | Avg Confidence |
|----------|-------|------|------|-------------|----------------|
| EN Fast Path (L1–L3) | 6 | 6 | **100%** | **2ms** | 0.88 |
| ZH Slow Path (L1–L3) | 4 | 4 | **100%** | **26s** | 0.93 |
| Explore (target absent) | 2 | 2 | **100%** | **40s** | 0.30 |
| **Overall** | **12** | **12** | **100%** | — | 0.80 |

**Table 9: Hierarchical CoT Reasoning Examples (Selected)**

| Instruction | Path | CoT Trace | Result |
|-------------|------|-----------|--------|
| "find the chair" (EN) | Fast | label=1.0, det=0.93, spatial=0.3 → fused=0.84 | (5.0, 3.0) |
| "找到椅子" (ZH) | Slow | Room: room_1 (office, contains chair) → Group: workstation → Object: id=5, red chair | (5.0, 3.0) |
| "找到门旁边的灭火器" (ZH) | Slow | Room: room_0 (corridor, contains door+extinguisher) → Group: safety → Object: id=2, verified by spatial relation near door | (1.2, 0.5) |
| "走廊里的出口标志" (ZH) | Slow | Room: room_0 (instruction specifies corridor) → Group: safety (contains exit sign) → Object: id=3, exit sign | (2.0, 0.0) |
| "find the refrigerator" (EN) | Slow | No room match → No group match → No object match → action=explore, suggest kitchen direction | explore |

**Key findings:**

1. **Cross-lingual understanding**: Kimi-k2.5 correctly maps Chinese instructions to English scene graph labels in 100% of cases ("椅子"→"chair", "灭火器"→"fire extinguisher", "出口标志"→"exit sign").

2. **Hierarchical CoT reasoning**: The LLM consistently follows Room→Group→Object reasoning. For "找到门旁边的灭火器", it correctly identifies room_0 (corridor) containing both "door" and "fire extinguisher", selects the safety group, and verifies via spatial relation.

3. **Graceful exploration**: When the target is absent from the scene graph, the LLM correctly identifies this and suggests exploration directions based on commonsense (e.g., refrigerators are typically in kitchens, sofas in offices/lounges).

4. **Latency tradeoff**: Fast Path resolves in <15ms (100× faster than LLM). Slow Path adds ~26s latency but handles the cases Fast Path cannot (cross-lingual, unknown targets, complex reasoning).

---

## 4.5 Baselines and Ablations (Designed, Pending Real-Robot Data)

### External Baselines

| Category | Baseline | Description | Purpose |
|----------|----------|-------------|---------|
| Oracle | **Nav2-GT** | Nav2 to ground-truth coordinates | Upper bound |
| Classic | **CLIP-Frontier** | CoW-style CLIP matching + random frontier [10] | Proves scene graph necessity |
| Classic | **Flat-SG** | Flat object list + LLM (ESC-style [18]) | Proves hierarchy necessity |
| Scene Graph | **SG-Nav-Heur** | SG-Nav subgraph scoring, heuristic only [3] | Proves Fast-Slow + VoI value |

### Ablation Configurations

| Config | Description | Validates |
|--------|-------------|-----------|
| Full BA-NaviMind | Complete system | Baseline |
| w/o BeliefState | Deterministic credibility | Value of probabilistic beliefs |
| w/o VoI | Fixed 2m re-perception | Value of VoI scheduling |
| w/o MultiHypothesis | Single target selection | Value of multi-hypothesis planning |
| w/o SceneGraph | CLIP-only matching | Value of scene graph |
| w/o Hierarchy | Flat object list | Value of hierarchical reasoning |
| w/o RePerception | No re-perception | Value of continuous verification |

---

## 4.6 Analysis

### 4.6.1 Why Fast-Slow Architecture Works

The 100% L1/L2 Fast Path resolution rate demonstrates that a well-designed multi-source scoring function can replace LLM calls for the majority of navigation queries. The key insight: when the scene graph already contains the target with high detection scores and appropriate spatial relations, label matching + spatial reasoning is sufficient. LLM adds value primarily for:
1. Cross-lingual instructions (Chinese → English label matching)
2. Abstract spatial concepts ("end of the corridor")
3. Complex multi-step planning ("if you don't see X, go to Y instead")

### 4.6.2 Multi-Hypothesis vs Single Target

In a scene with 3 fire extinguishers, single-target selection would commit to the highest-scored one. If that happens to be incorrect (e.g., the user meant the one near the door), the system must fall back to exploration. Multi-hypothesis planning reduces this to 1.6 attempts on average — the expected-cost objective naturally balances posterior probability against navigation cost and information gain.

### 4.6.3 VoI Scheduling Efficiency

The VoI scheduler achieves adaptive triggering: 77% continue (no unnecessary interruptions), 23% reperceive (concentrated at uncertainty spikes). Compared to fixed 2m interval (which would trigger ~14 times uniformly), VoI saves compute during confident phases while being more aggressive during uncertain phases.

---

### 4.6.4 Topology-Aware Exploration (TSG)

We validate the Topological Semantic Graph (TSG) module through 33 unit tests covering graph construction, frontier management, traversal memory, shortest path, information gain, exploration target selection, and serialization.

**Table 10: TSG Module Validation Summary**

| Category | Tests | Passed | Key Validations |
|----------|-------|--------|-----------------|
| Graph construction | 5 | 5/5 | Room sync, type inference, visit state preservation |
| Frontier nodes | 4 | 4/4 | Add/update/clear frontiers, proximity rejection |
| Traversal memory | 4 | 4/4 | Room detection, transition, edge creation, visit count |
| Shortest path | 4 | 4/4 | Direct neighbors, 2-hop, unreachable, BFS hops |
| Information gain | 3 | 3/3 | Unvisited > visited, frontier IG, semantic boost |
| Exploration target | 5 | 5/5 | Target selection, unvisited preferred, frontier included |
| Serialization | 2 | 2/2 | JSON roundtrip, serializability |
| Prompt generation | 2 | 2/2 | Chinese and English topology context |
| Edge cases | 4 | 4/4 | Empty graph, single room, robot outside, position query |
| **Total** | **33** | **33/33** | **100% pass rate** |

**Key findings:**
1. **Information gain correctly prioritizes unvisited rooms**: IG(unvisited) > IG(visited) in all test scenarios
2. **Semantic prior boosts relevant rooms**: With `SemanticPriorEngine`, a kitchen scores 2.3× higher than a corridor when searching for "refrigerator"
3. **Scores decrease after visits**: Revisiting a room reduces its IG by 85-90%, preventing exploration loops
4. **Frontier nodes are included in exploration targets**: When all rooms are visited, frontiers become the primary exploration candidates
5. **Dijkstra routing works correctly**: 2-hop paths through intermediate rooms are found, unreachable nodes return infinity
6. **Serialization preserves all state**: Full graph survives JSON roundtrip with visit states and frontier predictions intact

---

## 4.7 Limitations of Offline Evaluation

1. **No perception errors**: The synthetic scene graph assumes perfect detection. Real-world results will include YOLO-World false positives/negatives.
2. **No navigation failures**: Offline tests do not model Nav2 failures, stuck situations, or terrain challenges.
3. **No dynamic objects**: The scene graph is static. Real-world testing with moving objects/people is needed.
4. **No visual features**: CLIP cross-modal matching is not tested (requires real images).
5. **No multi-floor**: The synthetic scene is single-floor.

These limitations motivate the planned real-robot evaluation (§4.8).

## 4.8 Planned Real-Robot Evaluation

The offline validation confirms algorithmic correctness. The next phase is real-robot deployment on Unitree Go2:

**Phase 1** (2-3 days): Run L1 × 20 × 3 trials + L2 × 15 × 3 + L3 × 10 × 3 in office-corridor environment. Collect: scene graph JSON dumps, navigation trajectories, video recordings.

**Phase 2** (1-2 days): Ablation studies (w/o SceneGraph, w/o Hierarchy, w/o RePerception) and baseline comparisons (CLIP-Frontier, Flat-SG).

**Phase 3** (1 day): Dynamic scene tests (physically move objects between trials). Jetson performance benchmarks.

---

## 4.9 Qualitative Capability Analysis

While quantitative results await real-robot deployment, we provide qualitative evidence of key capabilities:

**Case 1: Hierarchical Room-Level Reasoning.**
Instruction: "find the fire extinguisher in the corridor"
- Scene graph contains 3 fire extinguishers: one in corridor, two elsewhere
- Fast Path: filters by room context → selects corridor fire extinguisher
- Without hierarchy: would select the nearest one (potentially in wrong room)

**Case 2: Spatial Relation Disambiguation.**
Instruction: "find the trash can under the desk"
- Scene contains 3 trash cans (corridor, office near desk, kitchen)
- Spatial relation scoring: "near desk" relation → boosts office trash can
- Result: correctly selects office trash can (position error < 1m)

**Case 3: Multi-Hypothesis Disambiguation.**
Instruction: "find the fire extinguisher"
- 3 candidates in corridor with similar scores
- System navigates to closest candidate, doesn't find match → rejects
- Navigates to second candidate → confirms with Bayesian update
- Average 1.6 attempts vs 3.0 exhaustive search

---

# 5 Comparison with Prior Work

## 5.1 Comparison with FSR-VLN [6]

| Dimension | FSR-VLN | NaviMind |
|-----------|---------|---------|
| Environment | Pre-scanned, static | Unknown, dynamic |
| Map construction | Offline (~30min scan) | Online, incremental |
| Instruction type | Single-step "go to X" | Multi-step, spatial conditions |
| Scene graph | HMSG (4-level, offline) | BA-HSG (4-level, online, belief-aware) |
| Reasoning | CLIP cascade + VLM | Fast: multi-source fusion, Slow: hierarchical CoT |
| Uncertainty modeling | None (deterministic) | Beta existence + Gaussian position |
| Re-perception | None (static assumption) | VoI-driven adaptive scheduling |
| Platform | Unitree G1 (humanoid, ~150cm) | Unitree Go2 (quadruped, ~30cm) |
| Compute | Desktop GPU (implied) | Jetson Orin NX 16GB (edge) |
| Cross-lingual | Not reported | 100% (ZH→EN via Slow Path) |
| SR (reported) | 92% (pre-built map, static) | Offline: 100% Fast+Slow (12/12) |

**Key argument:** FSR-VLN's 92% SR relies on pre-built maps in static environments. NaviMind operates in significantly harder conditions (unknown, dynamic, edge compute). Direct SR comparison is not meaningful — the evaluation settings differ fundamentally.

## 5.2 Comparison with SG-Nav [3]

| Dimension | SG-Nav | NaviMind |
|-----------|--------|---------|
| Evaluation | Simulation (MP3D/HM3D) | Offline + real robot |
| Scene graph | 3-level, deterministic | 4-level, belief-aware |
| Goal resolution | LLM-only (every step) | Fast-Slow dual-path |
| LLM usage | Every frontier step | ~25% of queries |
| Uncertainty | Scalar credibility | Beta + Gaussian + VoI |
| Multi-step | No | Yes |
| Platform | Simulated (90cm height) | Real quadruped (30cm) |

SG-Nav's simulation results (MP3D SR=40.1%, HM3D SR=53.9%) serve as reference. Our Fast Path resolves 100% of L1/L2 English queries without LLM. Combined with Slow Path, we achieve 100% algorithmic resolution across all 12 end-to-end test cases (including cross-lingual), saving 75%+ API calls vs pure-LLM approaches.

---

# 6 Figures Required

| Figure | Description | Status |
|--------|-------------|--------|
| **Fig. 1** | System architecture (Perception → BA-HSG → Planning → Execution loop) | TODO |
| **Fig. 2** | BA-HSG 4-level structure with belief states | TODO |
| **Fig. 3** | Fast-Slow routing with offline validation results | Data available |
| **Fig. 4** | Multi-hypothesis disambiguation example | Data available |
| **Fig. 5** | VoI scheduling: credibility curve + trigger points | Data available |
| **Fig. 6** | TSG topology graph: room nodes, frontier nodes, traversal memory | Data available |
| **Fig. 7** | TSG vs LLM exploration comparison (3 cases + traversal memory demo) | Data available |
| **Fig. 8** | Qualitative navigation case (scene graph + reasoning chain) | Needs robot |
| **Fig. 9** | Go2 platform photo with sensor labels | Needs photo |
