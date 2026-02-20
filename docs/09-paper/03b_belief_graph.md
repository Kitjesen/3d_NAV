# 3.4 Belief-Aware Hierarchical Scene Graph (BA-HSG)

> **本章是方法学核心创新**：将 §3.2 的确定性场景图升级为概率信念场景图，将 §3.4 的工程化 re-perception 升级为形式化的风险敏感规划，并将 §3.3 的 Fast-Slow 路由升级为信息价值 (VoI) 驱动的推理调度。
>
> 整合了 reviewer 建议的 UA-HSG + VoI-Scheduler + Belief-GoalNav 三条路线的核心要素。

---

## 3.4.1 Motivation: From Deterministic to Belief-Aware Scene Graphs

Existing scene graph representations, including ConceptGraphs [7] and HOV-SG [8], treat detected objects as deterministic entities with point-estimate positions. In practice, open-vocabulary detectors produce noisy, uncertain, and sometimes hallucinated outputs. A "fire extinguisher" detected at position $\mathbf{p}$ may be:
- A true positive with uncertain localization (depth noise, viewpoint ambiguity).
- A false positive that will vanish upon closer inspection.
- A stale observation of an object that has since been moved.

Navigating to such a target deterministically—as done in FSR-VLN [6] and most existing systems—commits the robot to a single hypothesis. If wrong, the robot must travel the full distance, discover the error, and backtrack, severely degrading SPL.

We propose the **Belief-Aware Hierarchical Scene Graph (BA-HSG)**, which extends every node in $\mathcal{G}$ with a probabilistic belief state, and uses these beliefs to drive risk-sensitive goal selection, VoI-based reasoning scheduling, and information-seeking exploration.

---

## 3.4.2 Node-Level Belief State

Each object node $v_i$ is augmented with three belief components:

### Existence Belief (Beta Distribution)

We model the probability that object $v_i$ truly exists (is not a false positive or stale) using a Beta distribution:

$$P(\text{exists}_i) \sim \text{Beta}(\alpha_i, \beta_i)$$

**Positive evidence** (Bernoulli observation $z = 1$): A detection is counted as positive if it has high confidence ($s_{\text{det}} > \tau_s$), valid depth, and geometric consistency (reprojection error $\epsilon_{\text{reproj}} < \kappa$):

$$\alpha_i \leftarrow \alpha_i + z_t, \quad \beta_i \leftarrow \beta_i + (1 - z_t)$$

**Negative evidence** ($z = 0$): When the object is within the camera's expected field of view but not detected for $k$ consecutive frames:

$$\beta_i \leftarrow \beta_i + \eta_{\text{neg}}$$

where $\eta_{\text{neg}} = 0.5$ (weaker than positive evidence, since occlusion may explain non-detection).

The expected existence probability is:

$$\bar{P}_i = \frac{\alpha_i}{\alpha_i + \beta_i}$$

**Initialization**: $\alpha_i = 1 + s_{\text{det}}$, $\beta_i = 1$ (weakly informative prior biased by initial detection confidence).

### Position Uncertainty (Isotropic Gaussian)

Each object's 3D position is modeled as:

$$\mathbf{p}_i \sim \mathcal{N}(\boldsymbol{\mu}_i, \sigma_i^2 \mathbf{I})$$

Updated via Kalman-style fusion when a new observation $\mathbf{p}_{\text{obs}}$ with estimated noise $\sigma_{\text{obs}}^2$ arrives:

$$\sigma_i^2 \leftarrow \left(\frac{1}{\sigma_i^2} + \frac{1}{\sigma_{\text{obs}}^2}\right)^{-1}, \quad \boldsymbol{\mu}_i \leftarrow \sigma_i^2 \left(\frac{\boldsymbol{\mu}_i^{\text{old}}}{(\sigma_i^{\text{old}})^2} + \frac{\mathbf{p}_{\text{obs}}}{\sigma_{\text{obs}}^2}\right)$$

The observation noise $\sigma_{\text{obs}}$ increases with depth (farther objects have higher uncertainty):

$$\sigma_{\text{obs}} = \sigma_0 + \sigma_d \cdot d$$

where $\sigma_0 = 0.05$m (baseline), $\sigma_d = 0.02$ (depth-proportional), $d$ is the depth in meters.

### Composite Credibility Score

For downstream decision-making, we compute a composite credibility:

$$C_i = w_s \cdot \tilde{s}_i + w_v \cdot A_i + w_t \cdot e^{-\Delta t_i / \tau_t} + w_g \cdot e^{-\epsilon_i^{\text{reproj}} / \kappa}$$

where:
- $\tilde{s}_i = \bar{P}_i$: existence probability from Beta posterior
- $A_i = 1 - 1/n_i$: view diversity (asymptotic with detection count $n_i$)
- $\Delta t_i$: time since last observation, $\tau_t = 30$s decay constant
- $\epsilon_i^{\text{reproj}}$: reprojection error from latest observation, $\kappa = 5.0$ pixels
- Weights: $w_s = 0.4$, $w_v = 0.2$, $w_t = 0.2$, $w_g = 0.2$

**Hierarchical propagation (Graph Diffusion).** Credibility propagates through the scene graph hierarchy:
- **Upward**: Room credibility is the weighted average of its objects' credibilities: $C_k^{\text{room}} = \frac{\sum_{i \in \mathcal{O}_k} n_i \cdot C_i}{\sum_{i \in \mathcal{O}_k} n_i}$
- **Downward**: A well-established room (high $C^{\text{room}}$) provides a prior boost to newly detected objects within it: $\alpha_i^{\text{new}} \leftarrow \alpha_i^{\text{new}} + \delta_{\text{room}} \cdot C_k^{\text{room}}$
- **Lateral**: Objects with spatial relations inherit partial credibility from neighbors: if $v_j$ is "near" $v_i$ and $C_j$ is high, $v_i$ receives a small evidence boost.

This **belief diffusion on the scene graph** is the key mechanism that separates BA-HSG from deterministic scene graphs. It allows the system to reason: "this fire extinguisher has low individual detection score, but it's in a well-confirmed corridor with other high-credibility safety equipment nearby, so its effective credibility is higher."

---

## 3.4.3 Risk-Sensitive Goal Selection

When multiple candidate targets exist (common for instructions like "find a chair"), we replace the deterministic "select highest-scoring object" strategy with risk-sensitive optimization.

### Multi-Hypothesis Target Belief

Given instruction $I$ and candidates $\{v_1, \ldots, v_M\}$ from the goal resolver, we initialize a posterior:

$$p_i \propto \exp\left(\gamma_1 \cdot s_{\text{fused}}(v_i, I) + \gamma_2 \cdot C_i + \gamma_3 \cdot P(\text{room}(v_i) | I)\right)$$

where $s_{\text{fused}}$ is the Fast Path score (§3.3.1), $C_i$ is the composite credibility, and $P(\text{room}(v_i) | I)$ is a room-instruction compatibility prior (e.g., "fire extinguisher" more likely in "corridor" than "office").

### Bayesian Update on Verification

When the robot approaches candidate $v_i$ and obtains verification observation $o$:

$$p_i \leftarrow \frac{p_i \cdot L(o | v_i)}{\sum_j p_j \cdot L(o | v_j)}$$

The likelihood $L(o | v_i)$ is:
- **Positive**: Target detected at expected position with CLIP similarity > 0.7 → $L = 0.9$
- **Negative**: Target not detected in expected FOV → $L = 0.1$  
- **Ambiguous**: Detection present but low confidence → $L = 0.4$

### Expected-Cost Goal Selection

Instead of always navigating to the highest-posterior candidate, we optimize:

$$i^* = \arg\min_{i} \underbrace{\mathbb{E}[d(\mathbf{p}_{\text{robot}}, \boldsymbol{\mu}_i)]}_{\text{Nav2 cost}} - \beta \underbrace{p_i}_{\text{success prob.}} + \rho \underbrace{\text{InfoGain}(i)}_{\text{disambiguation value}}$$

where InfoGain measures how much reaching $v_i$ would help distinguish between remaining candidates (e.g., if two candidates are nearby, visiting one disambiguates both).

This formulation naturally balances exploitation (go to highest-probability target) with exploration (go to the target that provides most information), and penalizes costly detours to low-probability candidates.

---

## 3.4.4 VoI-Driven Reasoning and Re-Perception Scheduling

The reviewer correctly identifies that our "every 2m trigger re-perception" is a heuristic. We formalize the scheduling decision as a **Value of Information (VoI)** optimization.

### Action Space

At each decision point, the system chooses among three meta-actions:

| Action | Cost | Effect |
|--------|------|--------|
| $a_0$: **Continue** | $\Delta d$ (distance) | Nav2 continues, no compute cost |
| $a_1$: **Re-perceive** | $T_{\text{rp}}$ (latency, ~0.5s) | 8-direction observation, update beliefs |
| $a_2$: **Slow-reason** | $T_{\text{llm}}$ (latency, ~2s) | LLM re-evaluates goal with updated graph |

### VoI Utility Function

For each action $a$, we estimate the immediate utility:

$$U(a) = \underbrace{\Delta \mathbb{E}[S(a)]}_{\text{success probability gain}} - \lambda_t \underbrace{T(a)}_{\text{latency cost}} - \lambda_d \underbrace{d(a)}_{\text{distance cost}}$$

where $\Delta \mathbb{E}[S(a)]$ is approximated by the expected reduction in goal entropy:

$$\Delta \mathbb{E}[S] \approx k \cdot \left(H(\mathbf{p}) - \mathbb{E}[H(\mathbf{p} | \text{new obs from } a)]\right)$$

For $a_1$ (re-perceive), the entropy reduction is estimated from the directional CLIP cache: directions with high uncertainty (low observation count) yield higher VoI. For $a_2$ (slow-reason), the VoI is high when the posterior $\mathbf{p}$ is flat (many equally likely candidates) but low when one candidate dominates.

### Scheduling Algorithm

```
Algorithm 2: VoI-Driven Scheduling
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Input: Belief state B, navigation state S, compute budget
Output: Selected action a*

1: H_goal ← Entropy(target_posterior)
2: C_best ← max credibility among candidates
3:
4: // Estimate VoI for each action
5: U_continue ← -λ_t · 0 - λ_d · Δd_step
6: U_reperceive ← k · E[ΔH | reperceive] - λ_t · T_rp
7: U_slow ← k · E[ΔH | slow_reason] - λ_t · T_llm
8:
9: // Apply cooldown constraints
10: if time_since_last_reperceive < T_cooldown then
11:     U_reperceive ← -∞
12: end if
13:
14: a* ← argmax(U_continue, U_reperceive, U_slow)
15: return a*
```

**Cooldown constraints**: To prevent oscillation, we enforce minimum intervals: re-perception cooldown $T_{\text{cool}}^{\text{rp}} = 1.0$m distance, slow-reasoning cooldown $T_{\text{cool}}^{\text{llm}} = 5.0$m distance.

**Practical approximation**: On Jetson Orin NX, VoI estimation uses pre-computed lookup tables indexed by (entropy level, credibility, distance traveled, time elapsed), avoiding runtime overhead. The full VoI computation reduces to a table lookup + comparison, adding <0.1ms per decision.

---

## 3.4.5 Integration with §3.2–3.6

BA-HSG replaces the deterministic credibility model (original §3.4) and enhances all other modules:

| Original Module | Enhancement with BA-HSG |
|----------------|------------------------|
| §3.2 Scene Graph | Nodes gain $(\alpha, \beta, \sigma^2, C)$ belief state |
| §3.3 Fast Path | Candidates filtered by $\bar{P}_i > \tau_{\text{exist}}$ before scoring |
| §3.3 Slow Path | LLM prompt includes credibility annotations per object |
| §3.4 Re-perception | VoI-scheduled instead of fixed 2m interval |
| §3.5 Multi-step | Sub-goal success/failure updates beliefs of surrounding objects |
| §3.6 Frontier | Frontier VoI term: "visiting this frontier may resolve candidate ambiguity" |

### Updated Overall Pipeline

```
Algorithm 3: BA-HSG Navigation Pipeline
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Input: Instruction I, unknown environment
Output: Navigation to target

1: G ← InitBeliefGraph()                     ▷ Empty BA-HSG
2: candidates ← ∅, posterior ← uniform
3:
4: while not (success or timeout) do
5:     // Perceive & Update Beliefs
6:     obs ← GetRGBD()
7:     dets ← YOLOWorld(obs.rgb)
8:     UpdateBeliefs(G, dets, obs.depth, TF)  ▷ Beta + Gaussian + C
9:     PropagateBeliefs(G)                     ▷ Graph diffusion
10:
11:    // Goal Resolution (with belief filtering)
12:    if candidates = ∅ or NeedReresolution() then
13:        candidates ← FastSlowResolve(I, G)  ▷ §3.3, filtered by P̄ᵢ
14:        posterior ← InitPosterior(candidates)
15:    end if
16:
17:    if max(posterior) > τ_accept then
18:        // Navigate to best candidate
19:        target ← SelectTarget(posterior, nav_cost)  ▷ Risk-sensitive
20:        SendNav2Goal(target.μ)
21:
22:        // VoI-driven monitoring during navigation
23:        action ← VoISchedule(beliefs, nav_state)
24:        if action = REPERCEIVE then
25:            ReperceiveAndUpdate(G)
26:            posterior ← BayesUpdate(posterior, new_obs)
27:        elif action = SLOW_REASON then
28:            candidates ← SlowPath(I, G)
29:            posterior ← InitPosterior(candidates)
30:        end if
31:    else
32:        // Explore: all candidates low probability
33:        frontier ← ScoreFrontiers(G, I, posterior)  ▷ §3.6 + VoI
34:        SendNav2Goal(frontier.position)
35:    end if
36: end while
```

---

## 3.4.6 Theoretical Properties

**Convergence.** The Beta posterior $\bar{P}_i$ converges to the true existence probability as observation count grows, with variance $\text{Var}(\bar{P}_i) = \frac{\alpha_i \beta_i}{(\alpha_i + \beta_i)^2 (\alpha_i + \beta_i + 1)} \to 0$.

**Consistency with SG-Nav.** When all objects have $\bar{P}_i \approx 1$ (high certainty), BA-HSG reduces to the deterministic credibility model of SG-Nav [3], making it a strict generalization.

**Computational overhead.** Per-object belief update requires $O(1)$ operations (Beta parameter increment, Gaussian fusion, credibility recomputation). Graph diffusion requires $O(|\mathcal{V}| + |\mathcal{E}|)$, which for typical indoor scenes (30–100 objects) completes in <1ms on Jetson Orin NX.

---

## Key Differences from Prior Work

| Aspect | ConceptGraphs [7] | HOV-SG [8] | DovSG | SG-Nav [3] | **BA-HSG (Ours)** |
|--------|-------------------|------------|-------|------------|-------------------|
| Node belief | ✗ (point estimate) | ✗ | ✗ | Credibility score | **Beta + Gaussian + composite** |
| Uncertainty propagation | ✗ | ✗ | Local update | ✗ | **Graph diffusion** |
| Goal selection | Deterministic | Deterministic | Deterministic | Distance-weighted | **Risk-sensitive + multi-hypothesis** |
| Reasoning schedule | — | — | — | Every step | **VoI-optimized** |
| False positive handling | ✗ | ✗ | ✗ | Accumulate credibility | **Bayesian rejection + belief update** |
