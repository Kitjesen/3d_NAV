# Semantic — Perception, Planning & Reconstruction Index

> Files live under `src/semantic/`
> 100+ .py files including subdirectories (excluding tests)

---

## Perception — `perception/` (L3)

### Top-Level (12)

| File | Responsibility |
|------|---------------|
| `__init__.py` | Package init — package marker |
| `perception_module.py` | **PerceptionModule** — top-level perception orchestrator |
| `detector_base.py` | DetectorBase abstract interface — all backends inherit |
| `yolo_world_detector.py` | YOLO-World open-vocabulary detector (125 classes, 10-15 FPS) |
| `yoloe_detector.py` | YOLOE lightweight detector backend |
| `bpu_detector.py` | Nash BPU YOLO11s-seg (~45ms/frame, instance segmentation) |
| `grounding_dino_detector.py` | Grounding DINO high-accuracy (slow, offline/slow-path validation) |
| `clip_encoder.py` | CLIP ViT-B/32 encoder — HOV-SG encode_three_source |
| `mobileclip_encoder.py` | MobileCLIP lightweight encoder for edge deployment |
| `instance_tracker.py` | HOV-SG scene graph core — RoomNode, DBSCAN, TrackedObject management |
| `bpu_tracker.py` | BPU-specific tracker — paired with bpu_detector |
| `keyframe_selector.py` | Keyframe selection — controls CLIP encoding frequency |

### API — `api/` (6)

| File | Responsibility |
|------|---------------|
| `__init__.py` | Package init |
| `perception_api.py` | Unified perception API — single entry point for all backends |
| `detector_api.py` | Detector backend API |
| `encoder_api.py` | Encoder backend API |
| `tracker_api.py` | Tracker backend API |
| `factory.py` | Backend factory — resolve by name |
| `types.py` | Shared type definitions |
| `exceptions.py` | Perception-specific exceptions |

### Impl — `impl/` (4)

| File | Responsibility |
|------|---------------|
| `__init__.py` | Package init |
| `perception_impl.py` | Perception implementation — pipeline orchestration |
| `clip_encoder.py` | CLIP encoder implementation |
| `instance_tracker.py` | Instance tracker implementation |
| `yolo_world_detector.py` | YOLO-World detector implementation |

### Scene Graph (4)

| File | Responsibility |
|------|---------------|
| `scg_builder.py` | Scene graph builder — object nodes + relationship edges |
| `scg_path_planner.py` | Semantic relationship-based reachability search |
| `hybrid_planner.py` | Semantic scene graph + geometric costmap joint planning |
| `tracked_objects.py` | TrackedObject data structures, CLIP FIFO, confidence decay |

### Topology (3)

| File | Responsibility |
|------|---------------|
| `room_manager.py` | Room boundary estimation, room-object assignment |
| `leiden_segmentation.py` | Leiden graph clustering for semantic room regions |
| `local_rolling_grid.py` | Robot-centric sliding window occupancy grid |

### Knowledge & Geometry (8)

| File | Responsibility |
|------|---------------|
| `knowledge_graph.py` | Indoor scene ontology, triple store, SPARQL-style queries |
| `knowledge_data.py` | Predefined categories, attributes, room-object priors |
| `bpu_qp_bridge.py` | BPU quantized output to KG confidence calibration |
| `geometry_extractor.py` | Point cloud PCA, bounding box, normals |
| `polyhedron_expansion.py` | Obstacle safety boundary dilation |
| `laplacian_filter.py` | LiDAR outlier noise filtering |
| `global_coverage_mask.py` | Explored area mask for frontier guidance |
| `uncertainty_model.py` | Perception confidence, map uncertainty, VOI signal |

### Evaluation (5)

| File | Responsibility |
|------|---------------|
| `evaluation_framework.py` | Precision/recall/F1, multi-backend comparison |
| `dataset_loader.py` | HM3D/ScanQA dataset loader, unified Episode interface |
| `baseline_wrappers.py` | CLIP-Nav etc. baseline unified calling interface |
| `end_to_end_evaluation.py` | SR/SPL/SoftSPL NaviMind metrics |
| `visualization_tools.py` | Scene graph RViz markers, detection boxes, Perception Viewer MJPEG |

### Other (4)

| File | Responsibility |
|------|---------------|
| `projection.py` | 2D→3D projection, TF transform wrapper |
| `service.py` | ROS2 service definitions for perception |
| `setup.py` | Package setup/installation |
| `perception_publishers.py` | ROS2 topic publication orchestration |
| `sim_scene_observer.py` | Simulated scene observer for testing |
| `dataset_loader.py` | Dataset loading utilities |

---

## Planner — `planner/` (L4)

### Top-Level (22)

| File | Responsibility |
|------|---------------|
| `__init__.py` | Package init — package marker |
| `semantic_planner_module.py` | **SemanticPlannerModule** (L4) — top-level planner orchestrator |
| `goal_resolver.py` | Fast-Slow dual-process coordinator |
| `goal_resolver_module.py` | **GoalResolverModule** — In: instruction, Out: goal_pose |
| `fast_path.py` | System 1: scene graph keyword + CLIP matching (<200ms) |
| `slow_path.py` | System 2: LLM reasoning with ESCA selective grounding (~2s) |
| `action_executor.py` | Action executor — LERa 3-step recovery |
| `action_executor_module.py` | **ActionExecutorModule** — module wrapper |
| `agent_loop.py` | Multi-turn LLM tool-calling loop (7 tools, max 10 steps/120s) |
| `visual_servo_module.py` | **VisualServoModule** — BBoxNavigator + PersonTracker dual-channel |
| `bbox_navigator.py` | BBoxNavigator — bbox+depth→3D→PD servo |
| `person_tracker.py` | PersonTracker — VLM select + CLIP Re-ID |
| `vlm_bbox_query.py` | Open-vocabulary bbox detection via VLM |
| `llm_module.py` | **LLMModule** — pluggable LLM backend (kimi/openai/claude/qwen/mock) |
| `llm_client.py` | Multi-backend LLM client with auto-fallback |
| `task_decomposer.py` | SayCan-style task decomposition |
| `task_decomposer_module.py` | **TaskDecomposerModule** — module wrapper |
| `task_rules.py` | Room-skill rule library |
| `prompt_templates.py` | H-CoT 4-step + frontier/explored prompts |
| `planner_state.py` | PlannerState FSM enum |
| `chinese_tokenizer.py` | Jieba tokenizer + stop words for Chinese NLP |
| `episodic_memory.py` | Episodic memory for past actions/observations |

### Frontier Exploration (4)

| File | Responsibility |
|------|---------------|
| `frontier_module.py` | **FrontierModule** — wavefront frontier explorer |
| `frontier_scorer.py` | MTU3D + L3MVN + TSP frontier ordering |
| `frontier_types.py` | FrontierNode / FrontierCluster dataclasses |
| `exploration_strategy.py` | Exploration strategy selection and switching |

### Knowledge & Memory (4)

| File | Responsibility |
|------|---------------|
| `room_object_kg.py` | Room-object knowledge graph prior |
| `topological_memory.py` | Topological memory for place recognition |
| `tagged_locations.py` | Tagged location store — named poses for reuse |
| `semantic_prior.py` | Semantic priors for goal resolution |

### Other (5)

| File | Responsibility |
|------|---------------|
| `implicit_fsm_policy.py` | Implicit FSM policy for behavior transitions |
| `sgnav_reasoner.py` | SGNav spatial reasoning with scene graph |
| `vlm_scene_agent.py` | VLM-powered scene understanding agent |
| `voi_scheduler.py` | Value of Information scheduling |
| `osnet_reid.py` | OSNet person Re-Identification |
| `adacot.py` | AdaCoT adaptive chain-of-thought |
| `service.py` | ROS2 service definitions for planner |
| `setup.py` | Package setup/installation |

---

## Reconstruction — `reconstruction/` (L3)

| File | Responsibility |
|------|---------------|
| `__init__.py` | Package init |
| `reconstruction_module.py` | **ReconstructionModule** — 3D reconstruction orchestrator |
| `dataset_recorder_module.py` | **DatasetRecorderModule** — record sensor data for offline recon |
| `keyframe_exporter_module.py` | **KeyframeExporterModule** — export keyframes for NeRF/3DGS |
| `color_projector.py` | Color projection — project camera RGB onto point cloud |
| `ply_writer.py` | PLY file writer — export colored point clouds |
| `semantic_labeler.py` | Semantic label projection — assign class labels to 3D points |
| `bag_reader.py` | ROS2 bag reader — extract sensor data from recorded bags |
| `dataset_io.py` | Dataset I/O utilities |

### Server — `server/` (8)

| File | Responsibility |
|------|---------------|
| `__init__.py` | Package init |
| `recon_server.py` | Reconstruction server — HTTP API for remote reconstruction |
| `backends/__init__.py` | Backends package init |
| `backends/base.py` | Backend abstract base class |
| `backends/registry.py` | Backend registry — discover registered backends |
| `backends/tsdf_backend.py` | TSDF fusion backend (Open3D) |
| `backends/open3d_backend.py` | Open3D reconstruction backend |
| `backends/nerfstudio_backend.py` | NeRFStudio backend |
| `backends/gsfusion_backend.py` | Gaussian Splatting fusion backend |
