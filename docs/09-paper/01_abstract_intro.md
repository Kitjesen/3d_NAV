# NaviMind: Online Hierarchical Scene Graph Reasoning for Zero-Shot Object Navigation on Quadruped Robots

---

## Abstract

We present **NaviMind**, a zero-shot object navigation system that enables quadruped robots to navigate to targets specified by natural language in unknown, dynamic indoor environments. Unlike prior work that relies on offline pre-built maps (FSR-VLN) or flat object lists (LOVON), NaviMind constructs a **Belief-Aware Hierarchical Scene Graph (BA-HSG)** online and incrementally, organizing detected objects into a four-level hierarchy (Object → Group → Room → Floor) where each node maintains a probabilistic belief state (Beta existence prior, Gaussian position uncertainty, composite credibility).

We propose three key methodological innovations beyond engineering integration. First, **belief-aware hierarchical scene graph construction**: each node is augmented with calibrated uncertainty estimates that propagate through the graph via belief diffusion, enabling the system to distinguish high-confidence targets from noisy or stale detections. Second, **risk-sensitive multi-hypothesis goal planning**: instead of committing to a single target, we maintain a posterior over candidate targets and select the next verification/navigation action by optimizing expected success probability against navigation cost and information gain. Third, **Value-of-Information (VoI) driven reasoning scheduling**: the decision of when to trigger re-perception, invoke LLM slow reasoning, or continue execution is formalized as a VoI optimization under edge-compute budget constraints, replacing heuristic fixed-interval triggers.

These innovations are integrated within a Fast-Slow dual-path architecture: 75%+ of queries are resolved via rapid multi-source fusion (label, CLIP, spatial cues) in <1ms, while complex queries use hierarchical Chain-of-Thought LLM reasoning. The system runs entirely onboard a Unitree Go2 quadruped robot with Jetson Orin NX (16GB), addressing low-viewpoint (~30cm), visual jitter, and terrain-constrained navigation challenges.

**Keywords**: Object Navigation, Belief Scene Graph, Uncertainty-Aware Planning, Vision-Language Navigation, Quadruped Robot, Zero-Shot

---

## 1 Introduction

Object-goal navigation—the task of navigating to a target object specified by its semantic category or natural language description—is a fundamental capability for mobile robots operating in human environments [1, 2]. Recent advances in large language models (LLMs) and vision-language models (VLMs) have opened new possibilities for zero-shot approaches that require no task-specific training [3, 4, 5]. However, deploying these capabilities on real robots, particularly quadruped platforms in unknown environments, remains a significant open challenge.

### 1.1 Motivation

Current state-of-the-art approaches face fundamental limitations when applied to real-world quadruped deployment:

**Limitation 1: Offline Map Dependency.** Methods like FSR-VLN [6] achieve impressive accuracy (92% SR) by constructing a Hierarchical Multi-modal Scene Graph (HMSG) through a time-consuming offline scanning process. This offline requirement makes them inapplicable to unknown environments where the robot must navigate immediately upon deployment. Similarly, ConceptGraphs [7] and HOV-SG [8] assume pre-scanned environments for scene graph construction.

**Limitation 2: Flat Scene Representation.** Systems like LOVON [9] and CoW [10] use flat object-level matching (detect → match → navigate) without structured spatial reasoning. When instructions involve spatial relations ("the fire extinguisher near the door") or room-level context ("go to the office"), flat representations fail to provide the hierarchical context needed for disambiguation.

**Limitation 3: No Dynamic Adaptation.** Most existing systems assume static environments [6, 8]. In practice, doors open and close, objects are moved, and people walk through spaces. Without a mechanism to verify and update target beliefs during navigation, false positive detections lead to irreversible failures—the robot navigates to an incorrect target and declares success.

**Limitation 4: Quadruped-Specific Challenges.** Quadruped robots introduce unique perception challenges largely ignored by prior work: extremely low camera viewpoint (~30cm vs. ~90cm for wheeled robots or ~150cm for humanoids), significant visual jitter from walking gaits, and limited field of view that frequently results in target occlusion. Only LOVON [9] and NaVILA [11] address quadruped navigation, but neither uses structured scene graphs.

### 1.2 Key Insight

Our key insight is that a **hierarchical scene graph**, constructed online and incrementally, provides the right abstraction for bridging the gap between raw perception and LLM-based reasoning. By organizing detected objects into semantically meaningful groups (e.g., "desk + chair + monitor → office workstation") and rooms (e.g., "office"), we can:

1. **Route simple queries efficiently**: "go to the chair" can be resolved by direct CLIP matching in <1ms without LLM calls.
2. **Enable hierarchical reasoning for complex queries**: "find the fire extinguisher in the corridor" requires first identifying which region is a "corridor", then searching within it—a multi-hop reasoning task naturally expressed as Room → Group → Object traversal.
3. **Detect and correct perception errors**: The graph structure enables credibility tracking—if a detected "fire extinguisher" has no supporting context (no nearby safety equipment, wrong room type), its credibility is low.
4. **Guide exploration intelligently**: When the target is not in the current scene graph, subgraph-to-frontier interpolation leverages accumulated scene knowledge to select the most promising exploration direction.

### 1.3 Contributions

We make the following contributions:

**(1) Belief-Aware Hierarchical Scene Graph (BA-HSG).** We propose a four-level scene graph (Object → Group → Room → Floor) where each node maintains a probabilistic belief state: Beta-distributed existence probability, Gaussian position uncertainty, and composite credibility integrating detection quality, view diversity, temporal freshness, and geometric consistency. Unlike ConceptGraphs [7] and HOV-SG [8] which use point estimates, our belief states enable principled false-positive rejection and risk-sensitive planning. Beliefs propagate through the graph hierarchy via **graph diffusion**, where well-established rooms boost newly detected objects and spatially related objects share credibility evidence.

**(2) Risk-Sensitive Multi-Hypothesis Goal Planning.** Instead of deterministically selecting the highest-scoring target (as in FSR-VLN [6] and SG-Nav [3]), we maintain a posterior distribution over candidate targets and optimize an expected-cost objective that balances success probability, navigation cost, and information gain. Upon approaching a candidate, Bayesian verification updates the posterior, enabling the system to efficiently disambiguate between multiple instances (e.g., multiple chairs) without committing to a single hypothesis prematurely.

**(3) VoI-Driven Reasoning and Re-Perception Scheduling.** We formalize the decision of when to trigger re-perception, invoke slow LLM reasoning, or continue navigation as a **Value of Information (VoI) optimization** under edge-compute budget constraints. This replaces heuristic fixed-interval triggers with a principled scheduling policy that adapts to the current uncertainty state, reducing unnecessary LLM calls while maintaining navigation safety.

**(4) Fast-Slow Dual-Path Goal Resolution with Hierarchical CoT.** A multi-source confidence fusion Fast Path resolves 75%+ of queries in <1ms without LLM calls. The Slow Path uses selective grounding (~90% token reduction) and a five-step hierarchical Chain-of-Thought prompt (Room → Group → Object) for complex spatial instructions.

**(5) Topology-Aware Semantic Exploration (TSG).** When the target is absent from the scene graph, we construct a topological semantic graph with room nodes, frontier nodes (unexplored boundaries), and traversal edges. An information-gain algorithm combining semantic priors, novelty, and uncertainty reduction selects optimal exploration targets in ~1ms without LLM calls. Traversal memory prevents redundant revisits, reducing repeated exploration by 90%.

**(6) Real Quadruped Deployment and Evaluation.** We implement and validate NaviMind on a Unitree Go2 quadruped with Jetson Orin NX (16GB), with comprehensive evaluation including ablation studies, dynamic scene tests, edge deployment benchmarks, and systematic failure analysis. We address quadruped-specific challenges including Laplacian blur filtering, low-viewpoint adaptation, and Nav2 integration.

### 1.4 Paper Organization

Section 2 reviews related work in scene graph navigation, zero-shot object navigation, and quadruped VLN. Section 3 presents the NaviMind method in detail, including scene graph construction, dual-path goal resolution, re-perception, and frontier scoring. Section 4 describes our experimental setup, baselines, and evaluation protocol. Section 5 presents results and analysis. Section 6 discusses limitations and future work.
