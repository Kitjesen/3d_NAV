# LingTu Web Dashboard

LingTu Web Dashboard is the operator UI for the robot-side Gateway. It shows session state, localization, SLAM, live scene data, saved maps, point clouds, and safe command controls.

## Run Locally

```bash
cd web
npm install
npm run dev
```

The Vite dev server listens on `http://localhost:3000`. It proxies `/api`, `/ws`, `/mcp`, and `/map` to `ROBOT_HOST`.

PowerShell example:

```powershell
$env:ROBOT_HOST = '192.168.66.190:5050'
npm run dev
```

Bash example:

```bash
ROBOT_HOST=192.168.66.190:5050 npm run dev
```

On the robot, Gateway serves the production build from `web/dist` at `http://<robot>:5050/`.

## Gateway Contract

The dashboard treats Gateway bootstrap as the contract source:

```text
GET /api/v1/app/bootstrap
GET /api/v1/app/capabilities
GET /api/v1/health
GET /ready
```

`src/services/api.ts` uses bootstrap `links` when available and falls back to the current default paths. Do not add new hardcoded REST paths without also exposing or checking the matching bootstrap link.

Important current links include:

| Link key | Current purpose |
| --- | --- |
| `events` | SSE telemetry stream |
| `cloud_ws`, `camera_ws` | point cloud and camera WebSocket streams |
| `session`, `session_start`, `session_end` | mapping, navigation, exploration lifecycle |
| `navigation_status`, `navigation_plan`, `navigation_cancel` | planning and autonomy status |
| `goal`, `stop` | navigation goal and stop command |
| `maps`, `map_activate`, `map_save`, `map_points` | map list, active map, save, PCD points |
| `slam_status`, `slam_switch`, `slam_relocalize`, `slam_auto_relocalize` | SLAM mode and relocalization |

## Safety Classes

Read-only UI actions:

- Open dashboard, status cards, topbar, Scene view, Map preview, SLAM status.
- SSE `/api/v1/events`, point cloud `/ws/cloud`, camera `/ws/camera`.
- Gateway bootstrap, health, readiness, map list, map points, navigation status.

State-changing but no robot motion:

- Start or end mapping/navigation/exploration session.
- Save, activate, rename, or delete a map.
- Switch SLAM mode.
- Manual or auto relocalization.
- Reset accumulated map cloud.

Robot motion capable:

- Send navigation goal from Scene view or slash command.
- Start exploration after safety/session gates pass.
- Teleop or any command that enters autonomous motion.

The UI now confirms map activation and saved-map load/relocalize. Goal sending is disabled when Gateway reports readiness blockers, localization loss, missing odometry, or an active command source.

## Sunrise No-Motion Smoke

Use `NO_PROXY` on sunrise because local Gateway requests can otherwise be routed through the configured HTTP proxy.

```bash
NO_PROXY=127.0.0.1,localhost,192.168.66.190 \
no_proxy=127.0.0.1,localhost,192.168.66.190 \
curl -sS http://127.0.0.1:5050/api/v1/app/bootstrap >/tmp/lingtu_bootstrap.json
```

From this repo, the read-only contract smoke is:

```bash
cd web
GATEWAY_URL=http://192.168.66.190:5050 npm run smoke:gateway
```

If you are running it directly on sunrise, prefer:

```bash
cd ~/data/inovxio/lingtu/web
NO_PROXY=127.0.0.1,localhost,192.168.66.190 \
no_proxy=127.0.0.1,localhost,192.168.66.190 \
GATEWAY_URL=http://127.0.0.1:5050 npm run smoke:gateway
```

## Validate

```bash
cd web
npm run lint
npx tsc -b --pretty false
npm run build
```

`npm run smoke:gateway` is intentionally not part of build because it requires a live Gateway.

## Source Map

| Path | Purpose |
| --- | --- |
| `src/App.tsx` | top-level tab layout and bootstrapping |
| `src/hooks/useSSE.ts` | bootstrap, capabilities, traffic, and telemetry stream |
| `src/services/api.ts` | centralized REST client and command receipt formatting |
| `src/components/SceneView.tsx` | 3D scene, goal placement, saved map drawer, map/SLAM actions |
| `src/components/MapView.tsx` | saved map CRUD and point cloud preview |
| `src/components/SlamPanel.tsx` | session lifecycle, SLAM state, relocalization controls |
| `scripts/gateway-smoke.mjs` | read-only Gateway contract smoke |

## Notes

- Keep command buttons bound to Gateway readiness, session gates, and control ownership.
- Keep no-motion actions explicit and confirmed when they alter robot state.
- Do not send a navigation goal until localization is tracking and `/ready` is 200.
