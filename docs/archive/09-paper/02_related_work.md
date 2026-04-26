# 2 Related Work

## 2.1 3D Scene Graphs for Robot Navigation

3D scene graphs provide structured representations of environments by encoding objects, their attributes, and inter-object relationships in a graph structure. **Hydra** [15] constructs 5-level real-time scene graphs (mesh → object → place → room → building) using 3D metric-semantic meshes for room segmentation. **ConceptGraphs** [7] builds open-vocabulary 3D scene graphs incrementally from RGB-D sequences, leveraging 2D foundation models for object detection and CLIP for semantic encoding. **HOV-SG** [8] extends this to multi-floor hierarchical scene graphs with open-vocabulary queries. **vS-Graphs** [16] tightly couples visual SLAM with scene graph construction, using wall and floor detection for room inference.

Most relevant to our work, **SG-Nav** [3] (NeurIPS 2024) constructs online hierarchical 3D scene graphs with object, group, and room levels, and uses LLM-based hierarchical Chain-of-Thought (CoT) prompting to score subgraphs for frontier selection. SG-Nav achieves state-of-the-art zero-shot performance on MP3D, HM3D, and RoboTHOR, surpassing previous zero-shot methods by >10% SR. We build upon SG-Nav's hierarchical reasoning paradigm but extend it in three ways: (i) we add quality-aware CLIP feature fusion and LLM-based room naming for higher-quality scene graphs, (ii) we design a Fast-Slow dual-path architecture that avoids LLM calls for simple queries, and (iii) we deploy on a real quadruped robot with continuous re-perception.

**FSR-VLN** [6] proposes a four-level Hierarchical Multi-modal Scene Graph (HMSG: Floor → Room → View → Object) with Fast-to-Slow reasoning that uses CLIP matching to progressively narrow down candidates from room to view to object level, and VLM verification for uncertain matches. FSR-VLN achieves 92% retrieval success rate on a Unitree G1 humanoid. However, FSR-VLN requires a **time-consuming offline scanning process** to build the HMSG and **assumes static environments**, making it unsuitable for unknown or dynamic settings. Our work addresses both limitations through online incremental construction and continuous re-perception.

**DovSG** [36] addresses dynamic environments by proposing local update mechanisms for scene graphs, enabling long-term task execution as objects are moved or environments change. While DovSG's dynamic adaptation is relevant, it does not formalize the update-to-planning connection—updates are reactive rather than driving risk-sensitive decision-making. Our BA-HSG embeds belief updates into an optimizable planning objective.

**SENT-Map** [37] constructs JSON-based semantic node topology maps that are LLM-readable and editable, with planning constrained to existing nodes to avoid infeasible states. **Guide-LLM** [38] similarly uses text-based topological graphs for LLM-driven indoor navigation. These SLAM-free approaches demonstrate that structured text representations enable effective LLM planning, but they lack the geometric grounding and uncertainty modeling of 3D scene graphs.

**Our positioning**: We adopt the hierarchical scene graph paradigm of SG-Nav and the Fast-Slow reasoning architecture of FSR-VLN, while addressing their key limitations through three methodological innovations: (i) belief-aware nodes with calibrated uncertainty (vs. deterministic representations in all prior work), (ii) VoI-driven reasoning scheduling (vs. heuristic triggers), and (iii) risk-sensitive multi-hypothesis planning (vs. single-target commitment).

## 2.2 Zero-Shot Object-Goal Navigation

Zero-shot object navigation eliminates the need for task-specific training, relying instead on pre-trained foundation models for perception and reasoning.

**CLIP-based methods.** CoW [10] (CVPR 2023) uses CLIP to compute similarity between target category text and observed image patches, directly matching without spatial reasoning. ZSON [17] aligns navigation goals to CLIP embeddings for zero-shot transfer. While efficient, these methods lack structured scene understanding and fail on spatially complex queries.

**LLM-based exploration.** ESC [18] (ICML 2023) uses LLM scene captioning to guide frontier exploration by describing what the robot observes at each step. L3MVN [19] (IROS 2023) prompts LLMs with detected object lists to select frontiers. OpenFMNav [20] combines open-vocabulary foundation models for navigation. These methods improve upon CLIP-only matching by incorporating LLM reasoning, but prompt the LLM with flat object lists that lack structural context.

**Frontier-based semantic scoring.** VLFM [21] (ICRA 2024 Best Paper) generates language-grounded value maps from RGB observations using VLMs, directing frontier exploration toward semantically promising regions. Deployed on Boston Dynamics Spot, VLFM achieves SOTA SPL across multiple benchmarks. We adopt a similar vision-augmented frontier scoring approach but integrate it within a hierarchical scene graph framework.

**Our distinction**: Unlike flat-representation methods (CoW, ESC, L3MVN), we use hierarchical scene graphs to provide structured context. Unlike SG-Nav which uses LLM for every frontier decision, our Fast Path avoids LLM calls for 75%+ of queries. Unlike VLFM which operates at the image level, we combine image-level vision scoring with scene-graph-level subgraph interpolation.

## 2.3 Closed-Loop Planning, Uncertainty, and Re-Perception

Effective navigation requires adapting plans based on execution outcomes and managing perceptual uncertainty.

**Inner Monologue** [22] (CoRL 2023) grounds LLM planning with environment feedback—success detection, scene descriptions, and human feedback—enabling the LLM to replan when actions fail. **SayCan** [23] (CoRL 2022) combines LLM knowledge with robotic affordances for grounded task planning. **LERa** [24] introduces a Learn-Execute-Replan loop with experience accumulation. **T-A-L** [25] further extends this with a Think-Act-Learn paradigm that stores failed experiences for future reference. **COME-robot** [39] emphasizes failure recovery and closed-loop execution for long-horizon tasks.

SG-Nav [3] proposes a graph-based re-perception mechanism that accumulates credibility scores when the robot approaches a candidate target, rejecting false positives based on subgraph probability. However, SG-Nav's credibility is a scalar heuristic without formal uncertainty modeling.

**Uncertainty in robot navigation.** Classical work uses Bayesian filters (EKF, particle filters) for state estimation, but these focus on robot pose, not target object existence. Recent work in active perception uses information-theoretic criteria (entropy reduction, mutual information) to select sensing actions. Our VoI-driven scheduling adopts this principle for deciding when to re-perceive or invoke slow reasoning.

**Our distinction**: We formalize the credibility model as a Beta-distributed existence belief with Gaussian position uncertainty, replace heuristic re-perception triggers with VoI optimization, and introduce multi-hypothesis target planning with Bayesian verification—elevating engineering strategies into a principled algorithmic framework.

## 2.4 Vision-Language Navigation on Quadruped Robots

Quadruped robots introduce unique challenges for VLN: low camera viewpoint (~30cm), visual jitter from walking gaits, terrain-constrained locomotion, and limited field of view.

**LOVON** [9] integrates LLM hierarchical task planning with open-vocabulary detection on Unitree Go2/B2/H1-2 platforms. It uses Laplacian variance filtering for blur rejection and handles temporary target loss through recovery behaviors. However, LOVON uses a flat pipeline (detect → plan → navigate) without scene graphs or spatial reasoning.

**NaVILA** [11] (RSS 2025, NVIDIA) proposes an end-to-end Vision-Language-Action model that generates mid-level spatial commands ("forward 75cm") from visual observations, trained on human videos and simulation data. While powerful, NaVILA requires large-scale training and operates as a black box without structured scene understanding.

**OrionNav** [26] constructs semantic scene graphs using FC-CLIP segmentation and DBSCAN clustering, with LLM-based real-time planning that dynamically updates as the scene graph evolves. Deployed on quadrupeds, OrionNav is conceptually similar to our work but lacks hierarchical CoT reasoning and re-perception.

**VLN-PE** [27] (ICCV 2025) systematically evaluates the sim-to-real gap across different robot morphologies, revealing that quadrupeds suffer from restricted observation space and locomotion-induced visual degradation.

**Our positioning**: NaviMind is the first system to combine online hierarchical scene graph construction, hierarchical CoT reasoning, multi-step instruction support, and continuous re-perception on a quadruped platform. Table 1 summarizes the comparison.

**Table 1: Comparison with related systems.**

| System | Scene Graph | Hierarchy | Online | Re-perception | Exploration | Multi-step | Quadruped | Zero-shot |
||--------|:-----------:|:---------:|:------:|:-------------:|:-----------:|:----------:|:---------:|:---------:|
| SG-Nav [3] | Y | 3-level | Y | Y | LLM-only | N | N | Y |
| FSR-VLN [6] | Y | 4-level | N | N | N | N | N | Y |
| LOVON [9] | N | N | -- | N | N | Y | Y | Y |
| OrionNav [26] | Y | Flat | Y | N | LLM | N | Y | Y |
| VLFM [21] | N | N | -- | N | VLM frontier | N | N | Y |
| CoW [10] | N | N | -- | N | Random | N | N | Y |
| DovSG [36] | Y | Flat | Y | Dynamic | N | N | N | Y |
| **NaviMind (Ours)** | **Y** | **4-level** | **Y** | **Belief+VoI** | **TSG+LLM** | **Y** | **Y** | **Y** |

## 2.5 Summary

Existing approaches either (a) construct high-quality scene graphs but require offline pre-building (FSR-VLN), (b) navigate online but lack structured reasoning (LOVON, CoW), or (c) use scene graph reasoning in simulation only (SG-Nav). NaviMind uniquely combines online incremental HSG construction, dual-path hierarchical reasoning, continuous re-perception, and real quadruped deployment—addressing the limitations of all three categories simultaneously.
