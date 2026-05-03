# Super-LIO Backend Status

Super-LIO is currently integrated as an external SLAM/localization backend behind
`SlamBridgeModule`, not as a normal in-process LingTu Module. It is selected with
`slam_profile="super_lio"` and consumed through the same bridge contract used by
the rest of the navigation stack.

The upstream project checked for this integration is the Super-LIO ROS 2 branch
at commit `a9220861e59194e3f10019192c6e5d61427a5b58`:

- Repository/README: <https://github.com/Liansheng-Wang/Super-LIO>
- Normal ROS 2 launch: `ros2 launch super_lio Livox_mid360.py`
- Upstream normal outputs: `/lio/odom` and `/lio/cloud_world`
- LingTu service remaps: `/lio/odom -> /nav/odometry`,
  `/lio/cloud_world -> /nav/map_cloud`

## Current Role

In the current LingTu stack, Super-LIO provides:

- odometry for pose freshness and localization health inference
- live map cloud for map-cloud freshness and Gateway visualization
- localization health source derived from `odom` plus `map_cloud`
- live map cloud snapshot source for map save when normal SLAM save services do
  not produce `map.pcd`

The bridge contract for `super_lio` is:

| Field | Value |
| --- | --- |
| `backend` | `super_lio` |
| `health_source` | `odom_map_cloud` |
| `map_save_source` | `live_map_cloud_snapshot` |
| `relocalization_supported` | `false` (legacy alias for saved-map relocalization) |
| `saved_map_relocalization_supported` | `false` |
| `restart_recovery_supported` | `true` |
| `recovery_method` | `restart_super_lio` |
| `relocalization_state` | `unsupported` unless a stale in-flight state is reported |
| `recovery_signal` | `NONE`, `LOC_DIVERGED`, or `RECOVERED` from bridge health logic |
| `recovery_action` | `restart_super_lio` when recovery is needed; `none` after recovery |

Operationally, this means Super-LIO can support live mapping/evaluation and
Gateway health reporting, but it is not yet a saved-map localization backend.

## Upstream Relocalization vs LingTu Online Relocalize

Upstream Super-LIO does include a relocation mode. It is started with
`ros2 launch super_lio relocation.py`, which runs `relocation_node` and loads a
previously saved PCD map from `lio.map.save_map_dir` plus
`lio.map.map_name`.

That is different from LingTu's current Gateway relocalization contract:

- LingTu `/api/v1/slam/relocalize` and `/api/v1/slam/auto_relocalize` expect an
  online service-style relocalization backend.
- The current `robot-super-lio.service` runs upstream `super_lio_node`, not
  `relocation_node`.
- Therefore `saved_map_relocalization_supported=false` is correct for the
  current LingTu `super_lio` bridge, even though upstream has a separate
  relocation executable.

To promote Super-LIO for saved-map navigation, add a separate LingTu relocation
profile/service that owns map path selection, initial pose injection
(`/initialpose`), and rollback into normal navigation mode.

## Experimental `super_lio_relocation` Profile

LingTu now has a guarded scaffold for that separate relocation path:

- CLI profiles: `python lingtu.py super_lio` and
  `python lingtu.py super_lio_relocation`
- REPL hot-switch commands: `slam super_lio` and
  `slam super_lio_relocation`
- Gateway profile/service name: `super_lio_relocation`
- Systemd unit: `robot-super-lio-relocation.service`
- Deploy source: `scripts/deploy/s100p/super_lio_relocation.service`
- Upstream executable: `ros2 run super_lio relocation_node`
- Map source: `/home/sunrise/data/nova/maps/active/map.pcd` by default
- Output remaps: `/lio/odom -> /nav/odometry`,
  `/lio/cloud_world -> /nav/map_cloud`

The upstream ROS wrapper prepends Super-LIO's compile-time `ROOT` directory to
`lio.map.save_map_dir`. The S100P systemd unit therefore converts an absolute
LingTu active-map path into a stable source-tree symlink before starting
`relocation_node`:

```text
/home/sunrise/data/inovxio/super-lio/src/super_lio/map/lingtu_active
  -> /home/sunrise/data/nova/maps/active
```

The value passed to Super-LIO is the relative directory
`map/lingtu_active`, so the upstream `ROOT + save_map_dir` behavior resolves to
the intended `active/map.pcd` file. The unit also checks that the final
`map.pcd` is non-empty and logs the resolved path plus byte size at startup.

This is a candidate backend for saved-map navigation startup, not the current
production relocalizer. Its LingTu contract is intentionally conservative:

| Field | Value |
| --- | --- |
| `backend` | `super_lio_relocation` |
| `health_source` | `odom_map_cloud` |
| `map_save_supported` | `false` |
| `map_save_source` | `active_map` |
| `relocalization_supported` | `false` |
| `saved_map_relocalization_supported` | `false` |
| `restart_recovery_supported` | `true` |
| `recovery_method` | `restart_super_lio_relocation` |

Use it only for field evaluation until we verify upstream relocation quality,
initial-pose behavior, drift, restart recovery, and route completion against the
current `localizer` path.

## Difference From `localizer`

`localizer` remains the saved-map relocalization path.

| Capability | `super_lio` | `localizer` |
| --- | --- | --- |
| Primary source | live LIO odometry and map cloud | ICP localizer against active saved map |
| Health source | `odom_map_cloud` | `localizer_health_topic` when fresh |
| Map save source | `live_map_cloud_snapshot` | existing active/saved map path |
| Saved-map relocalization | not supported | supported |
| Restart recovery | supported | supported as fallback |
| Primary recovery method | restart Super-LIO service | call relocalize service |

Do not treat Super-LIO as a drop-in replacement for `localizer` in saved-map
navigation. A `nav` session that must recover against an existing `map.pcd`
still needs the `localizer` backend.

## Gateway/API Fields

The current status fields are exposed through Gateway localization and health
responses. The most direct endpoint is:

```bash
curl -s http://localhost:5050/api/v1/localization/status | python3 -m json.tool
```

Check these fields in the response:

| Field | Expected Super-LIO value |
| --- | --- |
| `backend` | `super_lio` |
| `health_source` | `odom_map_cloud` |
| `map_save_source` | `live_map_cloud_snapshot` |
| `relocalization_supported` | `false` |
| `saved_map_relocalization_supported` | `false` |
| `restart_recovery_supported` | `true` |
| `recovery_method` | `restart_super_lio` |
| `relocalization_state` | usually `unsupported` |
| `recovery_signal` | `NONE`, `LOC_DIVERGED`, or `RECOVERED` |
| `recovery_action` | `restart_super_lio` during recovery; otherwise `none` |

Related endpoints:

```bash
curl -s http://localhost:5050/api/v1/health | python3 -m json.tool
curl -s http://localhost:5050/api/v1/slam/status | python3 -m json.tool
```

Use `/api/v1/localization/status` for the field contract above. Use
`/api/v1/health` for whole-system health and `/api/v1/slam/status` for active
SLAM service mode.

## Field Verification

Run this smoke check on the robot from the LingTu checkout:

```bash
cd ~/data/SLAM/navigation
bash scripts/lingtu slamcheck super_lio --duration 30 --rollback previous
```

The command switches to Super-LIO, verifies the Gateway/session/localization
contract, and switches back to the previous SLAM profile. It checks:

- `backend=super_lio`
- `health_source=odom_map_cloud`
- `map_save_source=live_map_cloud_snapshot`
- `saved_map_relocalization_supported=false`
- `restart_recovery_supported=true`
- `recovery_method=restart_super_lio`
- `robot-super-lio.service` active through `/api/v1/slam/status`

For an explicit map snapshot check, add `--save-map`:

```bash
bash scripts/lingtu slamcheck super_lio --duration 30 \
  --save-map super_lio_field_check --rollback previous
```

To try the guarded relocation backend against a saved map:

```bash
bash scripts/lingtu slamcheck super_lio_relocation \
  --map corrected_20260406_224020 \
  --initial-pose 0 0 0 \
  --duration 45 \
  --rollback previous
```

This writes `/run/lingtu/super_lio_relocation.env`, switches to
`robot-super-lio-relocation.service`, checks the Gateway contract, then rolls
back. It does not save a new map and should still report
`saved_map_relocalization_supported=false`.

Do not treat Gateway readiness alone as relocation success. After a relocation
smoke, inspect the service journal and require evidence that the upstream node
loaded the saved map and used a non-empty ICP target:

```bash
journalctl -q -u robot-super-lio-relocation.service --since '5 min ago' \
  --no-pager \
  | grep -E 'Super-LIO relocation map|Load map|Map size|target size|Global ICP'
```

Expected signs:

- `Super-LIO relocation map: .../map/lingtu_active/map.pcd bytes=<nonzero>`
- `Load map success`
- `Map size: <nonzero>`
- `INIT start... target size: <same nonzero map size>`
- Ideally `Global ICP Converged Succeed`

On the 2026-05-04 S100P smoke with map `corrected_20260406_224020`, the unit
resolved `map/lingtu_active/map.pcd`, loaded `Map size: 170086`, and reached
`Global ICP Converged Succeed`. That proves the wrapper can load the saved map;
route-level drift/recovery validation is still required before promotion.

For routine status inspection:

```bash
cd ~/data/SLAM/navigation
bash scripts/lingtu status
bash scripts/lingtu svc status
```

Expected service-level signs:

- `super_lio` appears in `scripts/lingtu svc status`.
- Super-LIO mode should not require `robot-localizer` to be the active
  localization source.
- `scripts/lingtu status` should show live SLAM/localization data instead of
  zero-Hz odometry or stale map cloud.

For API-level verification, check the compact field set:

```bash
curl -s http://localhost:5050/api/v1/localization/status \
  | python3 -c 'import json,sys; d=json.load(sys.stdin); keys=("backend","health_source","map_save_source","relocalization_supported","saved_map_relocalization_supported","restart_recovery_supported","recovery_method","relocalization_state","recovery_signal","recovery_action"); print({k:d.get(k) for k in keys})'
```

For map-save behavior in Super-LIO mode:

```bash
curl -s -X POST -H 'Content-Type: application/json' \
  -d '{"action":"save","name":"super_lio_field_check"}' \
  http://localhost:5050/api/v1/maps | python3 -m json.tool
```

The response should report `slam_profile` or source metadata indicating
Super-LIO mode, `map_save_source` or `source` as `live_map_cloud_snapshot`, and
`saved_map_relocalization_supported` as `false`.

## Known Limits

- Super-LIO does not currently support LingTu online saved-map relocalization.
- Saved maps produced from the live map cloud snapshot are useful for inspection
  and evaluation, but they do not make Super-LIO a relocalizing backend.
- Recovery is service restart based. It does not perform global relocalization.
- Health is inferred from odometry/map-cloud freshness and bridge diagnostics,
  not from a Super-LIO-specific localization quality service.
- Upstream relocation mode is separate and still needs a LingTu-owned
  profile/service wrapper validation before use in saved-map navigation.

## Next Steps

- Validate `super_lio_relocation` on S100P against a known saved route before
  promoting it beyond experimental use.
- Add a real online global relocalization path before treating Super-LIO as the
  replacement for `localizer`.
- Decide whether Super-LIO should emit a backend-specific health/quality topic
  instead of relying only on `odom_map_cloud` inference.
- Compare static drift, route closure drift, CPU, memory, and restart recovery
  against the current Fast-LIO2/localizer stack.
- Promote the backend only after map save, recovery, and long-run field tests
  match the localizer-backed navigation requirements.
