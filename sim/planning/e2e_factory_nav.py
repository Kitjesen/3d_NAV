#!/usr/bin/env python3
"""
e2e_factory_nav.py — Factory 多层导航端到端验证 (纯 Python，无需 ROS2)

路线:  B1(5,3,-2.0) → 坡道R1 → G0主厅 → 仓库 → 楼梯S2 → M1 → S3 → M2 → S4 → RF(72,52,13.5)
垂直升程: 15.5m   水平距离: ~115m   层数: 5 (B1/G0/M1/M2/RF)

算法:
  平层段  → Nav3DPlanner 3D A* (从 MuJoCo 几何体提取障碍云)
  坡道/楼梯 → 预定义斜坡路点 (RISE/RUN 参数化)

输出:
  tools/e2e_factory_traj.png   — 4 面板静态图
  tools/e2e_factory_nav.gif    — 动画
"""

import math, sys, os, time
import numpy as np
import mujoco
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches as mpatches
from matplotlib.collections import LineCollection

sys.path.insert(0, os.path.dirname(__file__))
from nav3d_planner import Nav3DPlanner

# ── 路径配置 ───────────────────────────────────────────────────────────────────
REPO     = os.path.normpath(os.path.join(os.path.dirname(__file__), '..', '..'))
XML      = os.path.join(REPO, 'sim', 'worlds', 'factory_scene.xml')
OUT_PNG  = os.path.join(REPO, 'tools', 'e2e_factory_traj.png')
OUT_GIF  = os.path.join(REPO, 'tools', 'e2e_factory_nav.gif')

# ── 工厂参数 ───────────────────────────────────────────────────────────────────
RISE, RUN, N_STEPS = 0.18, 0.30, 25   # 楼梯参数

# 楼层 Z 高度
Z_B1, Z_G0, Z_M1, Z_M2, Z_RF = -2.5, 0.0, 4.5, 9.0, 13.5
ROBOT_H = 0.25   # 机器人中心离地高度 (m)

# ── 几何体分类 ─────────────────────────────────────────────────────────────────
# 不加入障碍的前缀 (可行走面 / 装饰 / 机器人)
SKIP_PREFIXES = (
    'floor_', 'platform_floor', 'dock_floor', 'outdoor_ground',
    'ramp_', 's1_', 's2_', 's3_', 's4_',   # 楼梯台阶
    'sw1_', 'sw2_', 'sw3_', 'sw4_',         # 楼梯侧墙 (细, 忽略)
    'r1_rail', 'r2_rail', 'rf_rail', 'plat_rail', 'bridge_rail',
    'r1_stripe', 'r2_stripe', 'stripe_',
    'crane_', 'beam_', 'duct_', 'antenna_', 'pipe_', 'cone',
    'tree_t', 'pot_', 'barrier_arm',
    'lidar_', 'dir_arrow', 'chassis', 'leg_', 'fork_arm',
    'start_', 'goal_', 'base_link',
    'glass_', 'plat_rail',
)

def _is_obstacle(name: str) -> bool:
    if not name:
        return False
    for p in SKIP_PREFIXES:
        if name.startswith(p):
            return False
    return True

# ── 从 MuJoCo 提取障碍点云 ────────────────────────────────────────────────────
def extract_obstacle_cloud(model, data) -> np.ndarray:
    """对每个障碍 geom 在 BBox 内均匀采样点，返回 (N,3) obstacle cloud。"""
    pts = []
    BOX, CYL, SPHERE, CAPSULE, PLANE = 6, 5, 2, 3, 0

    for i in range(model.ngeom):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i) or ''
        if not _is_obstacle(name):
            continue
        gtype = int(model.geom_type[i])
        if gtype == PLANE:
            continue

        pos = data.geom_xpos[i].copy()      # world pos
        mat = data.geom_xmat[i].reshape(3, 3).copy()
        sz  = model.geom_size[i].copy()

        if gtype == BOX:
            # 在 box 体积内均匀采样
            n = max(2, int(max(sz) / 0.15))
            ls = [np.linspace(-s, s, max(2, int(s/0.12)+1)) for s in sz]
            for lx in ls[0]:
                for ly in ls[1]:
                    for lz in ls[2]:
                        local = np.array([lx, ly, lz])
                        pts.append(pos + mat @ local)

        elif gtype in (CYL, CAPSULE):
            r, h = sz[0], sz[1]
            for th in np.linspace(0, 2*math.pi, max(8, int(2*math.pi*r/0.15))):
                for zl in np.linspace(-h, h, max(3, int(2*h/0.15)+1)):
                    local = np.array([r*math.cos(th), r*math.sin(th), zl])
                    pts.append(pos + mat @ local)

        elif gtype == SPHERE:
            r = sz[0]
            for ph in np.linspace(0, math.pi, 6):
                for th in np.linspace(0, 2*math.pi, 10):
                    local = np.array([r*math.sin(ph)*math.cos(th),
                                      r*math.sin(ph)*math.sin(th),
                                      r*math.cos(ph)])
                    pts.append(pos + mat @ local)

    return np.array(pts) if pts else np.zeros((0, 3))

# ── 斜坡 / 楼梯路点 ────────────────────────────────────────────────────────────
def ramp_waypoints(x_start, x_end, y0, y1, z0, z1, n=8) -> list:
    """沿坡道/楼梯生成中间路点（线性插值）。"""
    wpts = []
    for i in range(n + 1):
        t = i / n
        wpts.append([x_start + t*(x_end-x_start),
                     y0 + t*(y1-y0),
                     z0 + t*(z1-z0)])
    return wpts

def stair_waypoints(cx, y0, z0, n=N_STEPS, n_wpts=10) -> list:
    """楼梯路点：沿楼梯斜面插值。"""
    y_end = y0 + n * RUN
    z_end = z0 + n * RISE
    wpts = []
    for i in range(n_wpts + 1):
        t = i / n_wpts
        wpts.append([cx, y0 + t*(y_end-y0), z0 + t*(z_end-z0)])
    return wpts

# ── 平层段 A* 规划 ─────────────────────────────────────────────────────────────
def plan_flat(obs_cloud: np.ndarray, start, goal,
              z_level, z_band=0.8, voxel=0.30) -> list:
    """
    在指定楼层高度附近的平层空间中运行 Nav3DPlanner A*。
    只使用 z ∈ [z_level - z_band, z_level + z_band] 的障碍点。
    """
    s = np.array(start); g = np.array(goal)
    if np.linalg.norm(g[:2] - s[:2]) < 1.0:
        return [s.tolist(), g.tolist()]

    # 过滤该楼层的障碍点
    if len(obs_cloud) > 0:
        mask = ((obs_cloud[:, 2] >= z_level - z_band) &
                (obs_cloud[:, 2] <= z_level + z_band + 3.0))
        layer_obs = obs_cloud[mask]
    else:
        layer_obs = obs_cloud

    planner = Nav3DPlanner(voxel_size=voxel)
    if len(layer_obs) > 0:
        planner.build(layer_obs)

    path = planner.plan(start, goal)
    return path

# ── 全路径规划 ─────────────────────────────────────────────────────────────────
def plan_factory_route(obs_cloud: np.ndarray) -> list:
    """
    分段规划 B1 → G0 → M1 → M2 → RF 全路径。
    返回 [(x,y,z), ...] 路点列表。
    """
    path = []

    def add(pts):
        for p in pts:
            if not path or np.linalg.norm(np.array(p) - np.array(path[-1])) > 0.05:
                path.append(list(p))

    zr = ROBOT_H   # 机器人离地偏移

    # ── Seg 0: B1 dock 到坡道底 ────────────────────────────────────────────────
    print('[Seg 0] B1 dock → Ramp R1 bottom')
    seg0 = plan_flat(obs_cloud,
                     [5.0, 3.0,  Z_B1 + zr],
                     [30.0, 5.0, Z_B1 + zr],
                     Z_B1)
    add(seg0)

    # ── Seg 1: 坡道 R1 (B1 → G0) ──────────────────────────────────────────────
    print('[Seg 1] Ramp R1: B1 → G0')
    add(ramp_waypoints(30, 30, 5.0, 25.0, Z_B1 + zr, Z_G0 + zr, n=12))

    # ── Seg 2: G0 主厅 → 仓库门廊 → 楼梯 S2 底 ────────────────────────────────
    print('[Seg 2] G0: ramp exit → warehouse → S2 base')
    seg2_a = plan_flat(obs_cloud,
                       [30.0, 25.0, Z_G0 + zr],
                       [49.0, 34.0, Z_G0 + zr],
                       Z_G0)
    add(seg2_a)
    seg2_b = plan_flat(obs_cloud,
                       [49.0, 34.0, Z_G0 + zr],
                       [56.5, 27.5, Z_G0 + zr],
                       Z_G0)
    add(seg2_b)

    # ── Seg 3: 楼梯 S2 (G0 → M1) ─────────────────────────────────────────────
    print('[Seg 3] Stair S2: G0 → M1')
    add(stair_waypoints(56.5, 28.0, Z_G0 + zr))

    # ── Seg 4: M1 S2 顶层 → S3 底 (极短) ─────────────────────────────────────
    print('[Seg 4] M1 landing → S3 base')
    add([[56.5, 36.0, Z_M1 + zr],
         [56.5, 36.5, Z_M1 + zr]])

    # ── Seg 5: 楼梯 S3 (M1 → M2) ─────────────────────────────────────────────
    print('[Seg 5] Stair S3: M1 → M2')
    add(stair_waypoints(56.5, 36.5, Z_M1 + zr))

    # ── Seg 6: M2 S3 顶 → S4 底 ───────────────────────────────────────────────
    print('[Seg 6] M2 landing → S4 base')
    add([[56.5, 44.5, Z_M2 + zr],
         [56.5, 45.5, Z_M2 + zr]])

    # ── Seg 7: 楼梯 S4 (M2 → RF) ─────────────────────────────────────────────
    print('[Seg 7] Stair S4: M2 → RF')
    add(stair_waypoints(56.5, 45.5, Z_M2 + zr))

    # ── Seg 8: RF 平台 → 终点 ─────────────────────────────────────────────────
    print('[Seg 8] RF platform → Goal')
    seg8 = plan_flat(obs_cloud,
                     [56.5, 53.5, Z_RF + zr],
                     [72.0, 52.0, Z_RF + zr],
                     Z_RF)
    add(seg8)

    print(f'[Plan] Total waypoints: {len(path)}')
    return path

# ── 运动学仿真 ─────────────────────────────────────────────────────────────────
def simulate_kinematics(path: list, dt=0.05, max_speed=0.8) -> dict:
    """
    简单路点追踪仿真：P 控制器向下一路点运动。
    返回 trajectory dict。
    """
    pos   = np.array(path[0], dtype=float)
    traj  = [pos.copy()]
    times = [0.0]
    vel_log = []

    target_idx = 1
    t = 0.0
    ARRIVE_THRESH = 0.35

    while target_idx < len(path):
        tgt = np.array(path[target_idx])
        diff = tgt - pos
        dist = np.linalg.norm(diff)

        if dist < ARRIVE_THRESH:
            target_idx += 1
            continue

        # P 控制: 速度 = min(kp * dist, max_speed) * direction
        speed = min(1.5 * dist, max_speed)
        vel = speed * diff / dist
        pos = pos + vel * dt
        t += dt

        traj.append(pos.copy())
        times.append(t)
        vel_log.append(speed)

        if t > 300.0:
            print(f'[Sim] Timeout at t={t:.1f}s, idx={target_idx}/{len(path)}')
            break

    traj_arr = np.array(traj)
    total_dist = sum(np.linalg.norm(traj_arr[i+1]-traj_arr[i])
                     for i in range(len(traj_arr)-1))
    final_dist = np.linalg.norm(np.array(path[-1]) - traj_arr[-1])
    goal_reached = final_dist < 1.0

    return {
        'traj': traj_arr,
        'times': np.array(times),
        'vel': np.array(vel_log) if vel_log else np.array([0.0]),
        'total_dist': total_dist,
        'elapsed': t,
        'final_dist': final_dist,
        'goal_reached': goal_reached,
        'path': np.array(path),
    }

# ── 可视化 ─────────────────────────────────────────────────────────────────────
LEVEL_CFG = [
    (Z_B1, '#5588CC', 'B1 -2.5m'),
    (Z_G0, '#44CC77', 'G0   0m'),
    (Z_M1, '#CCCC44', 'M1 +4.5m'),
    (Z_M2, '#FF9944', 'M2 +9.0m'),
    (Z_RF, '#FF5555', 'RF +13.5m'),
]

def z_to_color(z: float) -> str:
    """根据 Z 高度返回楼层颜色。"""
    for zv, col, _ in reversed(LEVEL_CFG):
        if z >= zv - 0.5:
            return col
    return LEVEL_CFG[0][1]

def plot_results(result: dict, path_pts: np.ndarray, obs_cloud: np.ndarray):
    traj = result['traj']
    times = result['times']

    fig = plt.figure(figsize=(20, 14))
    fig.patch.set_facecolor('#0E1018')

    # ── Panel 1: 3D 轨迹视图 ────────────────────────────────────────────────────
    ax1 = fig.add_subplot(2, 3, (1, 4), projection='3d')
    ax1.set_facecolor('#0E1018')

    # 楼层参考平面
    for zv, col, lbl in LEVEL_CFG:
        xx, yy = np.meshgrid([0, 80], [0, 55])
        zz = np.full_like(xx, zv, dtype=float)
        ax1.plot_surface(xx, yy, zz, alpha=0.06, color=col)

    # 规划路点
    ax1.plot(path_pts[:, 0], path_pts[:, 1], path_pts[:, 2],
             'w--', lw=0.8, alpha=0.4, label='Planned path')

    # 实际轨迹（按 Z 着色）
    for i in range(len(traj)-1):
        col = z_to_color(traj[i, 2])
        ax1.plot(traj[i:i+2, 0], traj[i:i+2, 1], traj[i:i+2, 2],
                 color=col, lw=2.0, alpha=0.9)

    # 楼梯位置标记
    for label, cx, y0, z0 in [('S2', 56.5, 28.0, 0.0),
                                ('S3', 56.5, 36.5, 4.5),
                                ('S4', 56.5, 45.5, 9.0)]:
        ax1.plot([cx], [y0], [z0], 's', color='white', ms=6, alpha=0.8)
        ax1.text(cx+1, y0, z0+0.5, label, color='white', fontsize=7, alpha=0.8)

    # 起终点
    ax1.scatter(*traj[0],  c='#00FF60', s=120, zorder=10, label='Start (B1)')
    ax1.scatter(*traj[-1], c='#FF2020', s=120, zorder=10, label='Goal (RF)')

    ax1.set_xlabel('X (m)', color='#AAA', fontsize=8)
    ax1.set_ylabel('Y (m)', color='#AAA', fontsize=8)
    ax1.set_zlabel('Z (m)', color='#AAA', fontsize=8)
    ax1.tick_params(colors='#888', labelsize=7)
    ax1.set_xlim(0, 80); ax1.set_ylim(0, 55); ax1.set_zlim(-4, 18)
    ax1.view_init(elev=22, azim=-50)

    level_patches = [mpatches.Patch(color=c, label=l, alpha=0.8)
                     for _, c, l in LEVEL_CFG]
    ax1.legend(handles=level_patches + [
        plt.Line2D([0],[0], color='#00FF60', marker='o', label='Start', ls=''),
        plt.Line2D([0],[0], color='#FF2020', marker='o', label='Goal', ls=''),
    ], loc='upper left', fontsize=7, facecolor='#1A2030', labelcolor='white')
    ax1.set_title('Factory 多层导航轨迹 (3D)', color='white', fontsize=11, pad=8)

    # ── Panel 2: Z-高度时间曲线 ─────────────────────────────────────────────────
    ax2 = fig.add_subplot(2, 3, 3)
    ax2.set_facecolor('#12141C')

    z_traj = traj[:, 2]
    for zv, col, lbl in LEVEL_CFG:
        ax2.axhline(zv, color=col, lw=1.0, ls='--', alpha=0.6, label=lbl)

    ax2.plot(times, z_traj, color='#FFD700', lw=2.0, label='Robot Z')
    ax2.fill_between(times, z_traj, Z_B1 - 0.5, alpha=0.15, color='#FFD700')

    ax2.set_xlabel('Time (s)', color='#CCC', fontsize=9)
    ax2.set_ylabel('Z 高度 (m)', color='#CCC', fontsize=9)
    ax2.set_title('Z 高度 vs. 时间', color='white', fontsize=10)
    ax2.tick_params(colors='#888')
    ax2.legend(loc='upper left', fontsize=7, facecolor='#1A2030', labelcolor='white')
    ax2.set_xlim(0, times[-1])
    ax2.set_ylim(Z_B1 - 0.5, Z_RF + 2.0)
    ax2.spines['bottom'].set_color('#334')
    ax2.spines['left'].set_color('#334')
    ax2.set_facecolor('#12141C')

    # ── Panel 3: G0 平面图 ──────────────────────────────────────────────────────
    ax3 = fig.add_subplot(2, 3, 5)
    ax3.set_facecolor('#12141C')

    # G0 轨迹
    g0_mask = (traj[:, 2] >= Z_B1 + 0.5) & (traj[:, 2] <= Z_M1 - 0.3)
    if g0_mask.any():
        ax3.plot(traj[g0_mask, 0], traj[g0_mask, 1],
                 color='#44CC77', lw=2.0, label='G0 path', zorder=5)

    # 障碍物 G0 层投影（采样）
    if len(obs_cloud) > 0:
        g0_obs = obs_cloud[(obs_cloud[:, 2] > -1.5) & (obs_cloud[:, 2] < 3.0)]
        if len(g0_obs) > 0:
            stride = max(1, len(g0_obs) // 3000)
            ax3.scatter(g0_obs[::stride, 0], g0_obs[::stride, 1],
                        s=1, c='#445566', alpha=0.4, zorder=2)

    # 楼梯标记
    ax3.add_patch(plt.Rectangle((5, 28), 3, 7.5, fill=False,
                                 edgecolor='#FFD700', lw=1.5, ls='--', alpha=0.7))
    ax3.add_patch(plt.Rectangle((55, 28), 3, 7.5, fill=False,
                                 edgecolor='#FF9944', lw=1.5, ls='--', alpha=0.7))
    ax3.text(6.5, 35.5, 'S1', color='#FFD700', fontsize=7, ha='center')
    ax3.text(56.5, 35.5, 'S2', color='#FF9944', fontsize=7, ha='center')

    ax3.set_xlim(0, 80); ax3.set_ylim(0, 55)
    ax3.set_xlabel('X (m)', color='#CCC', fontsize=8)
    ax3.set_ylabel('Y (m)', color='#CCC', fontsize=8)
    ax3.set_title('G0 平面图 (俯视)', color='white', fontsize=10)
    ax3.tick_params(colors='#888')
    ax3.set_facecolor('#12141C')

    # ── Panel 4: 统计信息 ──────────────────────────────────────────────────────
    ax4 = fig.add_subplot(2, 3, 6)
    ax4.set_facecolor('#12141C')
    ax4.axis('off')

    status = '[OK] GOAL REACHED' if result['goal_reached'] else '[!!] TIMEOUT'
    stats_text = (
        f"{'=' * 32}\n"
        f"  Factory Multi-Level Nav Result\n"
        f"{'=' * 32}\n\n"
        f"  Status:     {status}\n\n"
        f"  起点:   B1  ( 5.0,  3.0, -2.0m)\n"
        f"  终点:   RF  (72.0, 52.0, 13.5m)\n\n"
        f"  楼层数:       5  (B1/G0/M1/M2/RF)\n"
        f"  垂直升程:  15.5 m\n"
        f"  路点数:    {len(path_pts):4d}\n"
        f"  轨迹点:    {len(traj):4d}\n\n"
        f"  总路程:  {result['total_dist']:6.1f} m\n"
        f"  仿真时间:{result['elapsed']:6.1f} s\n"
        f"  终点误差:{result['final_dist']:6.2f} m\n\n"
        f"  楼梯段 (3段):\n"
        f"    S2: G0(Z=0) → M1(Z=4.5m)\n"
        f"    S3: M1 → M2(Z=9.0m)\n"
        f"    S4: M2 → RF(Z=13.5m)\n"
        f"  各 {N_STEPS} 步 × RISE={RISE}m RUN={RUN}m\n"
    )
    ax4.text(0.05, 0.95, stats_text, transform=ax4.transAxes,
             fontsize=9, color='#DDDDDD', va='top', fontfamily='monospace',
             bbox=dict(boxstyle='round', facecolor='#1A2430', alpha=0.8))

    plt.suptitle('MapPilot Factory — 5-Level Navigation E2E',
                 color='white', fontsize=13, y=0.98)
    plt.tight_layout(rect=[0, 0, 1, 0.97])
    plt.savefig(OUT_PNG, dpi=150, bbox_inches='tight',
                facecolor='#0E1018')
    print(f'[Plot] Saved → {OUT_PNG}')
    plt.close(fig)

def make_gif(result: dict, path_pts: np.ndarray):
    """生成俯视动画 GIF，显示机器人在各层的移动轨迹。"""
    traj = result['traj']
    n_frames = min(120, len(traj))
    step = max(1, len(traj) // n_frames)
    frames_idx = list(range(0, len(traj), step))[:n_frames]

    fig, ax = plt.subplots(figsize=(10, 7))
    fig.patch.set_facecolor('#0E1018')
    ax.set_facecolor('#12141C')
    ax.set_xlim(-5, 85); ax.set_ylim(-5, 62)
    ax.set_xlabel('X (m)', color='#CCC'); ax.set_ylabel('Y (m)', color='#CCC')
    ax.tick_params(colors='#888')

    # 建筑轮廓
    rect = plt.Rectangle((0, 0), 80, 55, fill=False,
                           edgecolor='#445566', lw=1.5)
    ax.add_patch(rect)
    # 楼梯标记框
    for (x0, y0, w, h, col, lbl) in [
        (5,  28, 3, 7.5, '#FFD700', 'S1'),
        (55, 28, 3, 7.5, '#FF9944', 'S2→M1'),
        (55, 36.5, 3, 7.5, '#FF5555', 'S3→M2'),
        (55, 45.5, 3, 7.5, '#FF2020', 'S4→RF'),
    ]:
        ax.add_patch(plt.Rectangle((x0, y0), w, h, fill=False,
                                    edgecolor=col, lw=1.2, ls='--', alpha=0.6))
        ax.text(x0+w/2, y0+h+0.3, lbl, color=col, fontsize=6, ha='center')

    # 规划路径
    ax.plot(path_pts[:, 0], path_pts[:, 1], 'w--', lw=0.6, alpha=0.3)

    line, = ax.plot([], [], lw=2.5, alpha=0.9)
    robot_dot, = ax.plot([], [], 'o', ms=8, color='white', zorder=10)
    level_text = ax.text(0.02, 0.95, '', transform=ax.transAxes,
                          color='white', fontsize=11, va='top',
                          bbox=dict(boxstyle='round', facecolor='#1A2430'))
    dist_text  = ax.text(0.70, 0.95, '', transform=ax.transAxes,
                          color='#FFD700', fontsize=9, va='top')
    ax.scatter(*traj[0, :2],  c='#00FF60', s=100, zorder=11, label='Start')
    ax.scatter(*traj[-1, :2], c='#FF2020', s=100, zorder=11, label='Goal')
    title = ax.set_title('', color='white', fontsize=10)

    def level_name(z):
        for zv, _, lbl in reversed(LEVEL_CFG):
            if z >= zv - 0.4:
                return lbl
        return 'B1'

    def animate(fi):
        i = frames_idx[fi]
        sub = traj[:i+1]
        if len(sub) < 2:
            return line, robot_dot, level_text, dist_text, title

        # 渐变颜色
        segs = [[[sub[j,0], sub[j,1]], [sub[j+1,0], sub[j+1,1]]]
                for j in range(len(sub)-1)]
        colors = [z_to_color(sub[j, 2]) for j in range(len(sub)-1)]
        lc = LineCollection(segs, colors=colors, lw=2.5, alpha=0.85)
        ax.add_collection(lc)

        robot_dot.set_data([traj[i, 0]], [traj[i, 1]])
        robot_dot.set_color(z_to_color(traj[i, 2]))

        lbl = level_name(traj[i, 2])
        level_text.set_text(f'Level: {lbl}  Z={traj[i,2]:.1f}m')
        dist_goal = np.linalg.norm(traj[i] - traj[-1])
        dist_text.set_text(f'dist_goal={dist_goal:.1f}m')
        t_now = result['times'][i]
        title.set_text(f'Factory Navigation  t={t_now:.1f}s')
        return line, robot_dot, level_text, dist_text, title

    ani = animation.FuncAnimation(fig, animate, frames=n_frames,
                                   interval=80, blit=False)
    ani.save(OUT_GIF, writer='pillow', fps=12, dpi=90)
    print(f'[GIF]  Saved → {OUT_GIF}')
    plt.close(fig)

# ── 主程序 ─────────────────────────────────────────────────────────────────────
def main():
    print('=' * 60)
    print('  MapPilot Factory 5-Level Navigation E2E')
    print('=' * 60)
    t0 = time.time()

    # 1. 加载 MuJoCo 模型
    print(f'\n[1] Loading factory_scene.xml ...')
    model = mujoco.MjModel.from_xml_path(XML)
    data  = mujoco.MjData(model)
    mujoco.mj_kinematics(model, data)
    print(f'    {model.ngeom} geoms, {model.nbody} bodies')

    # 2. 提取障碍点云
    print(f'\n[2] Extracting obstacle cloud ...')
    obs_cloud = extract_obstacle_cloud(model, data)
    print(f'    {len(obs_cloud):,} obstacle points sampled')

    # 3. 分段规划
    print(f'\n[3] Planning multi-level route ...')
    path = plan_factory_route(obs_cloud)
    path_arr = np.array(path)
    print(f'    Path: {len(path)} waypoints, '
          f'distance={sum(np.linalg.norm(np.array(path[i+1])-np.array(path[i])) for i in range(len(path)-1)):.1f}m')

    # 4. 运动学仿真
    print(f'\n[4] Running kinematic simulation ...')
    result = simulate_kinematics(path, dt=0.05, max_speed=0.8)
    print(f'    Elapsed: {result["elapsed"]:.1f}s  '
          f'Distance: {result["total_dist"]:.1f}m  '
          f'Final dist to goal: {result["final_dist"]:.2f}m')
    status = '[OK] GOAL REACHED' if result['goal_reached'] else '[!!] TIMEOUT'
    print(f'    Status: {status}')

    # 5. 可视化
    print(f'\n[5] Generating visualization ...')
    os.makedirs(os.path.join(REPO, 'tools'), exist_ok=True)
    plot_results(result, path_arr, obs_cloud)
    make_gif(result, path_arr)

    elapsed = time.time() - t0
    print(f'\n{"=" * 60}')
    print(f'  Done in {elapsed:.1f}s')
    print(f'  PNG: {OUT_PNG}')
    print(f'  GIF: {OUT_GIF}')
    print(f'  Status: {status}')
    print(f'{"=" * 60}')

if __name__ == '__main__':
    main()
