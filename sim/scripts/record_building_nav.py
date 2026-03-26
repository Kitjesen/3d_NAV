"""Record kinematic navigation demo — stable, deterministic, with LiDAR mapping.

Usage: python sim/scripts/record_building_nav.py
"""
import sys, os, time, math
import numpy as np
import cv2

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))
sys.path.insert(0, os.path.join(REPO_ROOT, "src"))
sys.path.insert(0, REPO_ROOT)

from drivers.sim.kinematic_sim_module import KinematicSimModule
from nav.navigation_module import NavigationModule
from base_autonomy.modules import LocalPlannerModule, PathFollowerModule
from core.module import Module
from core.blueprint import Blueprint
from core.stream import Out, In
from core.msgs.geometry import PoseStamped, Pose, Vector3, Quaternion
from core.msgs.nav import Odometry
from core.msgs.sensor import PointCloud
from sim.engine.core.world import ObstacleConfig

OUTPUT = os.path.join(REPO_ROOT, "building_nav_demo.mp4")
FPS, W, H = 10, 640, 480
GOAL = (8.0, 5.0)
MS, MR = 200, 0.2


class Rec(Module, layer=3):
    lidar_in: In[PointCloud]
    odom_in: In[Odometry]
    costmap_out: Out[dict]
    goal_cmd: Out[PoseStamped]

    def __init__(self, **kw):
        super().__init__(**kw)
        self.grid = np.zeros((MS, MS), dtype=np.float32)
        self.scans = 0
        self.rx = self.ry = 0.0
        self.lidar_pts = None
        self.trail = []

    def setup(self):
        self.lidar_in.subscribe(self._on_lidar)
        self.odom_in.subscribe(self._on_odom)

    def _on_odom(self, o):
        self.rx = o.pose.position.x
        self.ry = o.pose.position.y
        self.trail.append((self.rx, self.ry))

    def _on_lidar(self, cloud):
        if cloud.points is None:
            return
        self.lidar_pts = cloud.points
        h = MS // 2
        for p in cloud.points:
            d = math.hypot(p[0] - self.rx, p[1] - self.ry)
            if 0.3 < d < 15:
                gx, gy = int(p[0] / MR + h), int(p[1] / MR + h)
                if 0 <= gx < MS and 0 <= gy < MS:
                    self.grid[gy, gx] = min(self.grid[gy, gx] + 25, 100)
        self.scans += 1
        if self.scans % 3 == 0:
            self.costmap_out.publish({"grid": self.grid.copy(), "resolution": MR, "origin": [0, 0]})


def draw(rec, goal, nh, elapsed):
    frame = np.zeros((H, W, 3), dtype=np.uint8)
    frame[:] = (20, 20, 25)
    half = MS // 2
    sc = min(W, H) / (MS * MR) * 0.85
    cx, cy = W // 2, H // 2

    def w2p(wx, wy):
        return int(cx + (wx - 5) * sc), int(cy - (wy - 3) * sc)

    # Grid
    for gy in range(MS):
        for gx in range(MS):
            v = rec.grid[gy, gx]
            if v > 25:
                px, py = w2p((gx - half) * MR, (gy - half) * MR)
                if 0 <= px < W and 0 <= py < H:
                    c = min(255, int(v * 2.5))
                    cv2.circle(frame, (px, py), 2, (c // 2, c // 4, c), -1)

    # LiDAR
    if rec.lidar_pts is not None:
        for p in rec.lidar_pts[::3]:
            px, py = w2p(p[0], p[1])
            if 0 <= px < W and 0 <= py < H:
                cv2.circle(frame, (px, py), 1, (0, 200, 0), -1)

    # Trail
    for i in range(1, len(rec.trail), 2):
        p1 = w2p(rec.trail[i - 1][0], rec.trail[i - 1][1])
        p2 = w2p(rec.trail[i][0], rec.trail[i][1])
        cv2.line(frame, p1, p2, (180, 120, 40), 1)

    # Robot
    rp = w2p(rec.rx, rec.ry)
    cv2.circle(frame, rp, 8, (0, 165, 255), -1)
    cv2.circle(frame, rp, 8, (255, 255, 255), 1)

    # Goal
    gp = w2p(goal[0], goal[1])
    cv2.drawMarker(frame, gp, (0, 0, 255), cv2.MARKER_STAR, 16, 2)

    # Text
    state = nh.get("state", "?")
    wp = "%d/%d" % (nh.get("wp_index", 0), nh.get("wp_total", 0))
    dist = math.hypot(rec.rx - goal[0], rec.ry - goal[1])
    occ = int((rec.grid > 40).sum())
    for i, txt in enumerate([
        "LingTu Kinematic Nav Demo",
        "t=%.1fs dist=%.1fm" % (elapsed, dist),
        "%s wp=%s map=%d scans=%d" % (state, wp, occ, rec.scans),
        "pos=(%.1f, %.1f)" % (rec.rx, rec.ry),
    ]):
        cv2.putText(frame, txt, (10, 22 + i * 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    if state == "SUCCESS":
        cv2.putText(frame, "GOAL REACHED!", (W // 2 - 110, H // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 2)
    return frame


def main():
    print("=== Recording Kinematic Nav Demo ===")
    obstacles = [
        ObstacleConfig("wall_L", "box", [8.0, 0.15, 0.8], [6.0, 7.0, 0.4]),
        ObstacleConfig("wall_R", "box", [8.0, 0.15, 0.8], [6.0, -1.0, 0.4]),
        ObstacleConfig("block1", "box", [0.6, 1.2, 0.5], [5.0, 3.0, 0.25]),
        ObstacleConfig("block2", "box", [0.4, 0.8, 0.5], [7.0, 4.5, 0.25]),
    ]

    bp = Blueprint()
    bp.add(KinematicSimModule, obstacles=obstacles, start_pos=(2.0, 3.0, 0.35))
    bp.add(Rec)
    bp.add(LocalPlannerModule, backend="simple")
    bp.add(PathFollowerModule, backend="pid")
    bp.add(NavigationModule, planner="astar", waypoint_threshold=2.0, downsample_dist=1.0)

    bp.wire("KinematicSimModule", "odometry", "Rec", "odom_in")
    bp.wire("KinematicSimModule", "odometry", "NavigationModule", "odometry")
    bp.wire("KinematicSimModule", "odometry", "LocalPlannerModule", "odometry")
    bp.wire("KinematicSimModule", "odometry", "PathFollowerModule", "odometry")
    bp.wire("KinematicSimModule", "lidar_cloud", "Rec", "lidar_in")
    bp.wire("Rec", "costmap_out", "NavigationModule", "costmap")
    bp.wire("Rec", "goal_cmd", "NavigationModule", "goal_pose")
    bp.wire("NavigationModule", "waypoint", "LocalPlannerModule", "waypoint")
    bp.wire("LocalPlannerModule", "local_path", "PathFollowerModule", "local_path")
    bp.wire("PathFollowerModule", "cmd_vel", "KinematicSimModule", "cmd_vel")

    system = bp.build()
    system.start()
    time.sleep(2.0)

    rec = system.get_module("Rec")
    nav = system.get_module("NavigationModule")

    out = cv2.VideoWriter(OUTPUT, cv2.VideoWriter_fourcc(*"mp4v"), FPS, (W, H))

    rec.goal_cmd.publish(PoseStamped(
        pose=Pose(position=Vector3(GOAL[0], GOAL[1], 0), orientation=Quaternion(0, 0, 0, 1)),
        frame_id="map", ts=time.time()))

    print("  Recording...")
    t0 = time.time()
    frames = 0
    while time.time() - t0 < 40:
        nh = nav.health()["navigation"]
        frame = draw(rec, GOAL, nh, time.time() - t0)
        out.write(frame)
        frames += 1

        if frames % (FPS * 3) == 0:
            dist = math.hypot(rec.rx - GOAL[0], rec.ry - GOAL[1])
            print("  t=%2.0fs pos=(%.1f,%.1f) dist=%.1f map=%d %s" % (
                time.time() - t0, rec.rx, rec.ry, dist, (rec.grid > 40).sum(), nh["state"]))

        if nh["state"] == "SUCCESS":
            for _ in range(FPS * 3):
                out.write(draw(rec, GOAL, nh, time.time() - t0))
                time.sleep(0.1)
            break

        time.sleep(1.0 / FPS)

    out.release()
    system.stop()
    sz = os.path.getsize(OUTPUT) / 1024 / 1024
    print("\n=== Video: %s (%.1f MB, %d frames) ===" % (OUTPUT, sz, frames))


if __name__ == "__main__":
    main()
