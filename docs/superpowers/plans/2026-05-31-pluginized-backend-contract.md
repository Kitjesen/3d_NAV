# Pluginized Backend Contract Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make every replaceable LingTu algorithm/provider report and resolve backends through a consistent plugin contract, so swapping local detection, planning, LLM, memory, SLAM, or exploration implementations does not require editing business modules.

**Architecture:** Keep Module-First boundaries: modules own ports and runtime state; registries own provider lookup; health owns configured/effective/degraded reporting. The first landing slice adds shared backend status and migrates autonomy hot-path modules without changing algorithm output. Later slices replace hardcoded factories with registry-first factories while preserving current defaults as compatibility fallbacks.

**Tech Stack:** Python modules, `core.registry`, `core.backend_status.BackendStatus`, pytest, existing blueprint stack factories.

---

## Audit Baseline

Two read-only agents and local inspection found these pluginization gaps:

- Navigation/autonomy: `TerrainModule`, `LocalPlannerModule`, `PathFollowerModule`, and `navigation()` still branch on backend strings internally. `GlobalPlannerService` already uses `core.registry` for `planner_backend`.
- Perception/semantic: `PerceptionModule` and `PerceptionFactory` hardcode detector/encoder/tracker creation. `llm_client.create_llm_client()` hardcodes LLM backend aliases. `SemanticPlannerModule` constructs `GoalResolver` with a default `LLMConfig()` instead of the planner stack's selected backend.
- Memory/reconstruction: vector-memory encoder fallback and reconstruction backend registry exist, but health/status fields are not aligned with configured/effective/degraded semantics.
- Gateway/MCP: status surfaces do not expose a unified plugin catalog or consistent degraded reason per backend.

## File Structure

- Create: `src/core/backend_status.py` - backend status value object shared by pluginized modules.
- Test: `src/core/tests/test_backend_status.py` - regression tests for status fields and autonomy fallback health.
- Modify: `src/base_autonomy/modules/local_planner_module.py` - report nanobind to `cmu_py` fallback through shared status.
- Modify: `src/base_autonomy/modules/terrain_module.py` - report simple/native/nanobind status through shared status.
- Modify: `src/base_autonomy/modules/path_follower_module.py` - report `nav_core` to `pid` fallback through shared status.
- Later modify: `src/semantic/perception/semantic_perception/api/factory.py` - registry-first detector/encoder/tracker factories.
- Later modify: `src/semantic/planner/semantic_planner/llm_client.py` - registry-first LLM client factory.
- Later modify: `src/memory/modules/vector_memory_module.py` - align encoder fallback health with `BackendStatus`.
- Later modify: `src/gateway/gateway_module.py` and `src/gateway/routes/diagnostics.py` - expose plugin catalog and degraded status.

---

### Task 1: Shared Backend Status Contract

**Files:**
- Create: `src/core/backend_status.py`
- Create: `src/core/tests/test_backend_status.py`
- Modify: `src/base_autonomy/modules/local_planner_module.py`
- Modify: `src/base_autonomy/modules/terrain_module.py`
- Modify: `src/base_autonomy/modules/path_follower_module.py`

- [x] **Step 1: Write failing tests for shared status and autonomy fallback**

Run: `python -m pytest src/core/tests/test_backend_status.py -q`

Expected before implementation: FAIL with `ModuleNotFoundError: No module named 'core.backend_status'`.

- [x] **Step 2: Add `BackendStatus`**

Required behavior:

```python
status = BackendStatus.configured_as("nanobind")
status.as_health_fields()
# {
#   "configured_backend": "nanobind",
#   "backend": "nanobind",
#   "degraded": False,
#   "degraded_reason": "",
# }

status.use("cmu_py", reason="compatible _nav_core missing")
status.as_health_fields()["degraded"] is True
```

- [x] **Step 3: Migrate autonomy modules without changing output behavior**

Required health shape for each module:

```python
{
    "configured_backend": "<requested backend>",
    "backend": "<effective backend>",
    "degraded": bool,
    "degraded_reason": "<empty or concrete reason>",
}
```

- [x] **Step 4: Verify the slice**

Run:

```bash
python -m pytest src/core/tests/test_backend_status.py -q
python -m pytest src/core/tests/test_simplification_wave1.py::TestW1LocalPlannerNoFallback src/core/tests/test_simplification_wave1.py::TestW1TerrainNoFallback -q
python -m py_compile src/core/backend_status.py src/base_autonomy/modules/local_planner_module.py src/base_autonomy/modules/terrain_module.py src/base_autonomy/modules/path_follower_module.py
```

Expected: all pass.

---

### Task 2: Autonomy Registry Coverage

**Files:**
- Modify: `src/base_autonomy/modules/local_planner_module.py`
- Modify: `src/base_autonomy/modules/path_follower_module.py`
- Modify: `src/base_autonomy/modules/terrain_module.py`
- Test: `src/core/tests/test_backend_status.py` or `src/core/tests/test_registry.py`

- [x] **Step 1: Add registry coverage assertions**

Required assertions:

```python
from core.registry import list_plugins

def test_autonomy_backend_registry_names_are_visible():
    assert {"nanobind", "native", "simple"} <= set(list_plugins("terrain"))
    assert "nav_core" in set(list_plugins("path_follower"))
    assert "cmu" in set(list_plugins("local_planner"))
```

- [x] **Step 2: Register compatibility aliases without changing defaults**

Target aliases:

```text
local_planner: cmu, nanobind, cmu_py, simple
path_follower: nav_core, pure_pursuit, pid
terrain: nanobind, native, simple
```

- [x] **Step 3: Fail fast on unknown autonomy backend names**

The error should name the invalid backend and list available registered names.

- [x] **Step 4: Verify**

Run:

```bash
python -m pytest src/core/tests/test_backend_status.py src/core/tests/test_registry.py -q
python -m pytest src/core/tests/test_non_native_navigation_blueprint.py src/core/tests/test_terrain_local_planner_contract.py -q
```

---

### Task 3: Perception Provider Registry

**Files:**
- Modify: `src/semantic/perception/semantic_perception/api/factory.py`
- Modify: `src/semantic/perception/semantic_perception/perception_module.py`
- Test: add focused tests under `src/core/tests/` or the semantic test package.

- [x] **Step 1: Register detector providers**

Target detector plugin names:

```text
yoloe, yolo_world, bpu, sim_scene
```

- [x] **Step 2: Register encoder providers**

Target encoder plugin names:

```text
clip, mobileclip
```

- [x] **Step 3: Register tracker providers**

Target tracker plugin names:

```text
bpu, instance
```

- [x] **Step 4: Preserve current defaults**

`perception(detector="yoloe", encoder="mobileclip")` must still instantiate the same default components when dependencies are available.

- [x] **Step 5: Verify**

Run:

```bash
python -m pytest src/core/tests/test_perception_factory_registry.py src/core/tests/test_perception_module.py::TestDetectorConfiguration -q
```

---

### Task 4: LLM And Goal Resolver Plugin Alignment

**Files:**
- Modify: `src/semantic/planner/semantic_planner/llm_client.py`
- Modify: `src/semantic/planner/semantic_planner/llm_module.py`
- Modify: `src/semantic/planner/semantic_planner/semantic_planner_module.py`
- Modify: `src/core/blueprints/stacks/planner.py`
- Test: semantic planner LLM/client tests.

- [x] **Step 1: Register LLM client providers**

Target LLM plugin names:

```text
mock, kimi, moonshot, openai, claude, anthropic, qwen, dashscope
```

- [x] **Step 2: Keep alias behavior stable**

Existing aliases in `_BACKEND_ALIASES` must continue resolving to the same client classes.

- [x] **Step 3: Pass selected planner backend into `GoalResolver`**

`planner(llm="qwen")` should make the semantic slow path use the same selected backend unless explicitly overridden.

- [x] **Step 4: Verify**

Run:

```bash
python -m pytest src/core/tests/test_llm_client_registry.py src/semantic/planner/tests/test_planner_node_init.py::TestSemanticPlannerInit -q
```

---

### Task 5: Memory, Reconstruction, SLAM, Exploration, Gateway

**Files:**
- Modify: `src/memory/modules/vector_memory_module.py`
- Modify: `src/semantic/reconstruction/server/backends/registry.py` only if adapter metadata is missing.
- Modify: `src/core/blueprints/stacks/slam.py`
- Modify: `src/core/blueprints/stacks/exploration.py`
- Modify: `src/gateway/routes/diagnostics.py`

- [x] **Step 1a: Align VectorMemory encoder health fields**

`VectorMemoryModule.health()` and `get_memory_stats()` now expose nested
`encoder_backend` status without changing the existing storage `backend`
field.

- [x] **Step 1b: Align remaining SLAM/exploration/reconstruction health fields**

Every replaceable backend health payload must include:

```text
configured_backend, backend, degraded, degraded_reason
```

- [x] **Step 2: Expose plugin catalog read-only**

Diagnostics should expose categories from `core.registry.list_categories()` and plugin names from `list_plugins(category)` without allowing runtime mutation.

- [x] **Step 3: Verify remaining Task 5 surfaces**

Verified with:

```bash
python -m pytest src/memory/tests/test_memory_modules.py::TestVectorMemoryOperations -q
python -m pytest src/gateway/tests/test_gateway_runtime_status.py::test_diagnostics_plugin_catalog_exposes_registered_backends src/gateway/tests/test_gateway_runtime_status.py::test_diagnostics_plugin_catalog_route src/gateway/tests/test_gateway_runtime_status.py::test_diagnostics_plugin_catalog_route_exposes_active_backend_status -q
python -m pytest src/gateway/tests/test_gateway_route_split.py::test_gateway_module_builds_split_routes_once src/gateway/tests/test_gateway_route_split.py::test_gateway_module_keeps_client_route_inventory -q
python -m pytest src/slam/tests/test_slam_backend_status.py src/slam/tests/test_slam_bridge_tf.py src/slam/tests/test_slam_stack_services.py -q
python -m pytest src/core/tests/test_tare_exploration.py src/core/tests/test_reconstruction_backend_status.py -q
python -m pytest src/core/tests/test_backend_status.py src/core/tests/test_perception_factory_registry.py src/core/tests/test_llm_client_registry.py src/gateway/tests/test_gateway_runtime_status.py::test_diagnostics_plugin_catalog_exposes_registered_backends src/gateway/tests/test_gateway_runtime_status.py::test_diagnostics_plugin_catalog_route src/gateway/tests/test_gateway_runtime_status.py::test_diagnostics_plugin_catalog_route_exposes_active_backend_status src/slam/tests/test_slam_backend_status.py src/core/tests/test_reconstruction_backend_status.py -q
python -m py_compile src/core/backend_status.py src/gateway/routes/diagnostics.py src/slam/slam_module.py src/slam/slam_bridge_module.py src/exploration/tare_explorer_module.py src/exploration/exploration_supervisor_module.py src/core/blueprints/stacks/exploration.py src/semantic/reconstruction/server/recon_server.py
```

---

## Stop Condition

Stop only when:

- All replaceable backends have a visible registry or documented compatibility shim.
- Every backend health payload reports configured/effective/degraded status.
- Unknown backend names fail fast before robot motion.
- Safety/cmd_vel wiring remains non-bypassable through `CmdVelMux`.
- Targeted tests pass for each migrated surface.
