# semantic_planner — semantic planning and LLM reasoning package
# See MODULES.md for full module descriptions.

# --- Core Module ---
# semantic_planner_module.py  SemanticPlannerModule (Layer 4)
# planner_state.py            PlannerState FSM enum

# --- Goal resolution ---
# goal_resolver_module.py     GoalResolverModule  (In: instruction → Out: goal_pose)
# goal_resolver.py            Fast-Slow dual-process coordinator
# fast_path.py                System 1: scene graph keyword + CLIP matching (<200ms)
# slow_path.py                System 2: LLM reasoning (~2s)

# --- Execution ---
# action_executor_module.py   ActionExecutorModule + LERa 3-step recovery
# visual_servo_module.py      VisualServoModule (BBoxNavigator + PersonTracker)
# agent_loop.py               Multi-turn LLM tool-calling loop (7 tools, max 10 steps)

# --- Task decomposition ---
# task_decomposer_module.py   TaskDecomposerModule (SayCan-style)
# task_decomposer.py          Decomposer algorithm
# task_rules.py               Room–skill rule library

# --- LLM / vision ---
# llm_module.py               LLMModule (pluggable backend: kimi/openai/claude/mock)
# llm_client.py               Multi-backend LLM client with auto-fallback
# vlm_bbox_query.py           Open-vocabulary bbox detection via VLM
# prompt_templates.py         H-CoT 4-step + frontier/explored prompts

# --- Frontier exploration ---
# frontier_module.py          WavefrontFrontierExplorer (Layer 2)
# frontier_scorer.py          MTU3D + L3MVN + TSP ordering
# frontier_types.py           FrontierNode / FrontierCluster dataclasses

# --- Memory (imported from memory.modules.*) ---
# Use memory.modules.topological_module.TopologicalMemoryModule
# Use memory.modules.episodic_module.EpisodicMemoryModule
# Use memory.modules.tagged_locations_module.TaggedLocationsModule

# --- Utils ---
# room_object_kg.py           Room–object knowledge-graph prior
# chinese_tokenizer.py        jieba tokeniser + stop words
