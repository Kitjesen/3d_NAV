# Semantic Layer Contract

LingTu's semantic layer is not a linear `Detector -> Encoder -> Memory`
pipeline. The runtime contract is centered on `PerceptionModule.scene_graph`.

## Runtime Boundary

`PerceptionModule` is the primary scene-perception module. It consumes camera
frames, depth, camera intrinsics, and odometry, then publishes:

- `scene_graph`: object, relation, and region context for semantic planning,
  memory, reconstruction, gateway, teleop, and visual servo consumers.
- `detections_3d`: projected object detections for semantic planner consumers.

Detector, encoder, tracker, and projection implementations are capabilities
inside `PerceptionModule`, not peer layers in the default full-stack runtime.

## Clean Layer Names

| Layer | Runtime role | Main modules |
| --- | --- | --- |
| L3 Scene Perception | Build live object and scene context from RGB-D and odometry. | `PerceptionModule` |
| L3 Optional Scene Reconstruction | Maintain RGB-D semantic reconstruction from the same camera and scene context. | `ReconstructionModule` |
| L4 Semantic Memory | Persist and query scene, location, temporal, and vector memory from `scene_graph + odometry`. | `SemanticMapperModule`, `VectorMemoryModule`, `EpisodicMemoryModule`, `TaggedLocationsModule`, `TemporalMemoryModule` |
| L4 Decision | Resolve instructions into navigation goals or visual-servo targets. | `SemanticPlannerModule`, `LLMModule`, `VisualServoModule` |

## Capability Components

`DetectorModule` and `EncoderModule` remain valid standalone modules for
experiments and isolated tests. They are not the default full-stack path.

`PerceptionModule` owns the production scene-perception pipeline:

```text
color/depth/camera_info/odometry
  -> detector backend
  -> optional encoder backend
  -> depth projection
  -> instance tracking
  -> scene_graph + detections_3d
```

## Wiring Contract

`PerceptionModule.scene_graph` fans out to:

- `SemanticPlannerModule`
- `VisualServoModule`
- `SemanticMapperModule`
- `VectorMemoryModule`
- `EpisodicMemoryModule`
- `TemporalMemoryModule`
- `ReconstructionModule`
- `GatewayModule`
- `MCPServerModule`
- optional frontier and teleop consumers

This fan-out is the stable boundary. Consumers must not depend on detector or
encoder internals.

## Non-Goals

- Do not split `PerceptionModule` into detector, encoder, projection, and
  tracker modules until a consumer needs independent scheduling or a shared
  embedding service.
- Do not place memory modules under perception. Memory consumes scene context;
  it does not produce the live scene graph.
- Do not introduce ROS 2 coupling into normal semantic modules.
