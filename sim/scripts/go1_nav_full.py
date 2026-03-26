"""Go1 RL walking + LiDAR + A* navigation — full pipeline.

Run: cd ~/data/SLAM/navigation && MUJOCO_GL=egl python3 sim/scripts/go1_nav_full.py
"""
import os, sys, time, math, heapq, tempfile
import numpy as np
os.environ.setdefault("MUJOCO_GL", "egl")

REPO = os.path.expanduser("~/data/SLAM/navigation")
SCENE = os.path.join(REPO, "sim/robots/go1_playground/xmls/scene_mjx_feetonly_flat_terrain.xml")
POLICY = os.path.join(REPO, "sim/robots/go1_playground/go1_policy.onnx")
OUTPUT = os.path.join(REPO, "go1_nav_full.mp4")

import mujoco
import onnxruntime as ort
import cv2
from pathlib import Path

print("=== Go1 Walking + Navigation ===")

# Inject obstacles into scene
scene_xml = open(SCENE).read()
obs_xml = (
    '<geom name="wall_L" type="box" size="4 0.1 0.5" pos="3 3.5 0.25" rgba="0.5 0.5 0.6 1"/>\n'
    '<geom name="wall_R" type="box" size="4 0.1 0.5" pos="3 -0.5 0.25" rgba="0.5 0.5 0.6 1"/>\n'
    '<geom name="block1" type="box" size="0.3 0.6 0.4" pos="2.5 1.5 0.2" rgba="0.7 0.3 0.2 1"/>\n'
    '<geom name="block2" type="box" size="0.2 0.4 0.4" pos="4 2 0.2" rgba="0.7 0.3 0.2 1"/>\n'
    '<geom name="goal_vis" type="sphere" size="0.15" pos="6 1.5 0.15" rgba="1 0.2 0.2 0.7" contype="0" conaffinity="0"/>\n'
)
scene_xml = scene_xml.replace("</worldbody>", obs_xml + "  </worldbody>")

tmp = tempfile.NamedTemporaryFile(suffix=".xml", delete=False,
    dir=os.path.dirname(SCENE), mode="w", encoding="utf-8")
tmp.write(scene_xml)
tmp.close()
model = mujoco.MjModel.from_xml_path(tmp.name)
Path(tmp.name).unlink()
data = mujoco.MjData(model)
print("  nq=%d nu=%d ngeom=%d" % (model.nq, model.nu, model.ngeom))

# Policy
mujoco.mj_resetDataKeyframe(model, data, 0)
default_angles = data.qpos[7:].copy()
policy_sess = ort.InferenceSession(POLICY, providers=ort.get_available_providers())
out_name = policy_sess.get_outputs()[0].name
action_scale = 0.5
last_action = np.zeros(12, dtype=np.float32)
nav_cmd = np.array([0.0, 0.0, 0.0], dtype=np.float32)
step_counter = [0]
n_sub = round(0.02 / model.opt.timestep)

def ctrl_callback(m, d):
    global last_action
    step_counter[0] += 1
    if step_counter[0] % n_sub != 0:
        return
    linvel = d.sensor("local_linvel").data.copy()
    gyro = d.sensor("gyro").data.copy()
    imu_xmat = d.site_xmat[m.site("imu").id].reshape(3, 3)
    gravity = imu_xmat.T @ np.array([0, 0, -1])
    jpos = (d.qpos[7:] - default_angles).astype(np.float32)
    jvel = d.qvel[6:].astype(np.float32)
    obs = np.concatenate([
        linvel, gyro, gravity, jpos, jvel, last_action, nav_cmd
    ]).astype(np.float32).reshape(1, -1)
    act = policy_sess.run([out_name], {"obs": obs})[0][0]
    last_action = act.copy()
    d.ctrl[:] = act * action_scale + default_angles

mujoco.set_mjcb_control(ctrl_callback)
mujoco.mj_resetDataKeyframe(model, data, 0)

print("  Stabilizing...")
nav_cmd[:] = [0, 0, 0]
for _ in range(1000):
    mujoco.mj_step(model, data)
print("  Standing at (%.2f, %.2f, %.2f)" % tuple(data.qpos[:3]))

# LiDAR
N_RAYS = 180
ray_angles = np.linspace(0, 2 * math.pi, N_RAYS, endpoint=False)
geomgroup = np.ones(6, dtype=np.uint8)
robot_body = model.body("trunk").id

def scan_lidar():
    pos = data.qpos[:3].copy().astype(np.float64)
    pos[2] = max(pos[2], 0.15)
    q = data.qpos[3:7]
    yaw = math.atan2(2 * (q[0]*q[3] + q[1]*q[2]), 1 - 2*(q[2]**2 + q[3]**2))
    cy, sy = math.cos(yaw), math.sin(yaw)
    dirs = np.zeros((N_RAYS, 3), dtype=np.float64)
    for i, a in enumerate(ray_angles):
        lx, ly = math.cos(a), math.sin(a)
        dirs[i] = [lx*cy - ly*sy, lx*sy + ly*cy, 0]
    dist = np.full(N_RAYS, -1.0, dtype=np.float64)
    gid = np.full(N_RAYS, -1, dtype=np.int32)
    mujoco.mj_multiRay(model, data, pos, dirs.flatten(),
                        geomgroup, 1, robot_body, gid, dist, None, N_RAYS, 10.0)
    mask = (dist > 0.2) & (dist < 10.0)
    if not mask.any():
        return np.zeros((0, 3), dtype=np.float32)
    return (pos + dirs[mask] * dist[mask, None]).astype(np.float32)

# Map + A*
GS, GR = 200, 0.2
grid = np.zeros((GS, GS), dtype=np.float32)
GOAL = np.array([6.0, 1.5])

def update_map(pts, rx, ry):
    h = GS // 2
    for p in pts:
        d = math.hypot(p[0] - rx, p[1] - ry)
        if 0.3 < d < 8:
            gx = int(p[0] / GR + h)
            gy = int(p[1] / GR + h)
            if 0 <= gx < GS and 0 <= gy < GS:
                grid[gy, gx] = min(grid[gy, gx] + 30, 100)

def run_astar(sx, sy, gx, gy):
    h = GS // 2
    si, sj = max(0, min(int(sx/GR+h), GS-1)), max(0, min(int(sy/GR+h), GS-1))
    gi, gj = max(0, min(int(gx/GR+h), GS-1)), max(0, min(int(gy/GR+h), GS-1))
    oset = [(0, si, sj)]
    came = {}
    gs = {(si, sj): 0}
    cl = set()
    while oset:
        _, cx, cy = heapq.heappop(oset)
        if (cx, cy) in cl:
            continue
        cl.add((cx, cy))
        if abs(cx-gi) <= 1 and abs(cy-gj) <= 1:
            p = []
            while (cx, cy) in came:
                p.append(((cx-h)*GR, (cy-h)*GR))
                cx, cy = came[(cx, cy)]
            return p[::-1]
        for dx, dy in [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(1,1),(-1,1),(1,-1)]:
            nx, ny = cx+dx, cy+dy
            if 0 <= nx < GS and 0 <= ny < GS and (nx, ny) not in cl and grid[ny, nx] < 50:
                c = gs[(cx, cy)] + (1.414 if dx and dy else 1)
                if c < gs.get((nx, ny), 1e9):
                    gs[(nx, ny)] = c
                    heapq.heappush(oset, (c + math.hypot(nx-gi, ny-gj), nx, ny))
                    came[(nx, ny)] = (cx, cy)
    return []

# Renderer
FPS, W, H = 15, 1280, 720
model.vis.global_.offwidth = W
model.vis.global_.offheight = H
renderer = mujoco.Renderer(model, H, W)
cam = mujoco.MjvCamera()
cam.type = mujoco.mjtCamera.mjCAMERA_FREE
cam.lookat[:] = [3, 1.5, 0.3]
cam.distance = 6.0
cam.elevation = -30
cam.azimuth = -45

vid = cv2.VideoWriter(OUTPUT, cv2.VideoWriter_fourcc(*"mp4v"), FPS, (W, H))
spf = round((1.0 / FPS) / model.opt.timestep)

# Nav state
nav_path = []
wp_idx = 0
last_plan = 0
scan_count = 0

print("  Recording 40s @ %dfps..." % FPS)
for fi in range(FPS * 40):
    t = fi / FPS
    rx, ry = float(data.qpos[0]), float(data.qpos[1])
    q = data.qpos[3:7]
    yaw = math.atan2(2*(q[0]*q[3]+q[1]*q[2]), 1-2*(q[2]**2+q[3]**2))
    gdist = math.hypot(rx - GOAL[0], ry - GOAL[1])

    if t < 3:
        nav_cmd[:] = [0, 0, 0]
        state = "STAND"
    elif gdist < 1.0 and t > 5:
        nav_cmd[:] = [0, 0, 0]
        state = "SUCCESS"
    else:
        state = "NAV"
        # LiDAR
        if fi % 3 == 0:
            pts = scan_lidar()
            if len(pts) > 0:
                update_map(pts, rx, ry)
                scan_count += 1

        # Replan
        if t - last_plan > 3 and t >= 3:
            raw = run_astar(rx, ry, GOAL[0], GOAL[1])
            if raw:
                nav_path = raw[::3]
                if nav_path:
                    nav_path.append((GOAL[0], GOAL[1]))
                wp_idx = 0
            last_plan = t

        # Pure pursuit
        if nav_path and wp_idx < len(nav_path):
            tx, ty = nav_path[wp_idx]
            dx, dy = tx - rx, ty - ry
            d = math.hypot(dx, dy)
            if d < 0.5:
                wp_idx += 1
                if wp_idx < len(nav_path):
                    tx, ty = nav_path[wp_idx]
                    dx, dy = tx - rx, ty - ry
                    d = math.hypot(dx, dy)
            if wp_idx < len(nav_path):
                des = math.atan2(dy, dx)
                yerr = des - yaw
                while yerr > math.pi: yerr -= 2*math.pi
                while yerr < -math.pi: yerr += 2*math.pi
                wz = max(-0.5, min(0.5, yerr * 1.2))
                tf = max(0.3, math.cos(yerr))
                vx = min(0.4, d * 0.4) * tf
                nav_cmd[:] = [vx, 0, wz]
            else:
                nav_cmd[:] = [0, 0, 0]

    # Step physics
    for _ in range(spf):
        mujoco.mj_step(model, data)

    # Render
    cam.lookat[:] = [rx, ry, 0.3]
    mujoco.mj_forward(model, data)
    renderer.update_scene(data, cam)
    rgb = renderer.render()
    f = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

    occ = int((grid > 40).sum())
    nwp = len(nav_path)
    cv2.putText(f, "Go1 Autonomous Navigation - LingTu", (15, 28),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(f, "t=%.1fs (%.1f,%.1f) dist=%.1f %s wp=%d/%d map=%d" % (
        t, rx, ry, gdist, state, min(wp_idx, nwp), nwp, occ),
        (15, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)

    if state == "SUCCESS":
        cv2.putText(f, "GOAL REACHED!", (W//2-180, H//2),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 3)

    vid.write(f)

    if fi % (FPS * 5) == 0:
        print("  t=%2.0fs (%.1f,%.1f) dist=%.1f %s wp=%d/%d map=%d scans=%d" % (
            t, rx, ry, gdist, state, min(wp_idx, nwp), nwp, occ, scan_count))

    if state == "SUCCESS":
        for _ in range(FPS * 3):
            nav_cmd[:] = [0, 0, 0]
            for _ in range(spf):
                mujoco.mj_step(model, data)
            mujoco.mj_forward(model, data)
            renderer.update_scene(data, cam)
            rgb = renderer.render()
            ff = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            cv2.putText(ff, "GOAL REACHED!", (W//2-180, H//2),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 3)
            vid.write(ff)
        break

mujoco.set_mjcb_control(None)
vid.release()
renderer.close()
print("\n=== Video: %s (%.1f MB) ===" % (OUTPUT, os.path.getsize(OUTPUT)/1024/1024))
