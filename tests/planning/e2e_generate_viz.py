#!/usr/bin/env python3
"""
E2E 可视化生成器 — 论文图 + GIF 动画
从 /tmp/e2e_results/ 读取 CSV, 生成:
  /tmp/e2e_paper_figure.png   — 论文三联图 (300 DPI)
  /tmp/e2e_nav_animated.gif   — 动态轨迹动画
"""
import os, json, csv, math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.patheffects as pe
from matplotlib.gridspec import GridSpec
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import FancyArrowPatch

OUT_DIR = '/tmp/e2e_results'
SCENARIOS = [
    {"id": 1, "name": "Diagonal NE",  "goal": (5.0, 2.0), "color": "#2196F3", "thre": 0.3},
    {"id": 2, "name": "Straight E",   "goal": (6.0, 0.0), "color": "#FF5722", "thre": 0.3},
    {"id": 3, "name": "Lateral N",    "goal": (0.0, 5.0), "color": "#4CAF50", "thre": 0.3},
]

# ── load data ─────────────────────────────────────────────────────────────────

def load_scenario(sc_id):
    rows = []
    path = f'{OUT_DIR}/scenario_{sc_id}.csv'
    with open(path) as f:
        rd = csv.DictReader(f)
        for r in rd:
            rows.append({k: float(v) for k, v in r.items()})
    meta_path = f'{OUT_DIR}/scenario_{sc_id}_meta.json'
    with open(meta_path) as f:
        meta = json.load(f)
    return rows, meta


def load_all():
    data = []
    for sc in SCENARIOS:
        rows, meta = load_scenario(sc['id'])
        data.append({'sc': sc, 'rows': rows, 'meta': meta})
    return data


# ── paper figure ─────────────────────────────────────────────────────────────

def make_paper_figure(data):
    plt.rcParams.update({
        'font.family': 'serif',
        'font.size': 9,
        'axes.titlesize': 10,
        'axes.labelsize': 9,
        'legend.fontsize': 7.5,
        'xtick.labelsize': 8,
        'ytick.labelsize': 8,
        'lines.linewidth': 1.4,
        'figure.dpi': 300,
    })

    fig = plt.figure(figsize=(12, 10), facecolor='white')
    gs = GridSpec(3, 3, figure=fig,
                  height_ratios=[2.2, 1.5, 1.0],
                  hspace=0.48, wspace=0.38)

    ax_A  = fig.add_subplot(gs[0, :])          # XY trajectories
    ax_B  = fig.add_subplot(gs[1, :])          # dist convergence
    ax_C  = fig.add_subplot(gs[2, 0:2])        # velocity time-series
    ax_T  = fig.add_subplot(gs[2, 2])          # metrics table

    # ── Panel A: XY trajectories ──────────────────────────────────────────────
    for d in data:
        sc, rows, meta = d['sc'], d['rows'], d['meta']
        xs = [r['x'] for r in rows]
        ys = [r['y'] for r in rows]
        gx, gy = sc['goal']
        col = sc['color']

        # Velocity-colored scatter background
        vxs = [r['vx'] for r in rows]
        sc_plot = ax_A.scatter(xs, ys, c=vxs, cmap='cool', s=4, alpha=0.45,
                               vmin=0, vmax=0.85, zorder=2)

        # Reference straight line
        ax_A.plot([0, gx], [0, gy], '--', color=col, alpha=0.35,
                  linewidth=1.0, zorder=3)

        # Actual trajectory
        ax_A.plot(xs, ys, '-', color=col, linewidth=1.8, alpha=0.9, zorder=4,
                  label=f'SC{sc["id"]}: {sc["name"]} '
                        f'(t={meta["t_reached"]:.1f}s, ε={meta["final_dist"]:.2f}m)')

        # Start
        ax_A.scatter([0], [0], s=80, color=col, marker='o', zorder=6, edgecolors='k', lw=0.5)

        # Goal marker
        ax_A.scatter([gx], [gy], s=200, color=col, marker='*', zorder=6, edgecolors='k', lw=0.5)
        ax_A.annotate(f' G{sc["id"]}({gx:.0f},{gy:.0f})',
                      xy=(gx, gy), fontsize=7.5, color=col, va='center')

    cbar = fig.colorbar(sc_plot, ax=ax_A, pad=0.01, fraction=0.015)
    cbar.set_label('vₓ (m/s)', fontsize=8)
    ax_A.scatter([0], [0], s=120, color='k', marker='o', zorder=7, label='Start (0,0)')
    ax_A.set_xlabel('X (m)'); ax_A.set_ylabel('Y (m)')
    ax_A.set_title('(A)  E2E Navigation Trajectories — 3 Goal Configurations',
                   fontweight='bold', loc='left')
    ax_A.set_aspect('equal'); ax_A.grid(True, alpha=0.25, lw=0.5)
    ax_A.legend(loc='upper left', framealpha=0.85, fontsize=7.5)

    # ── Panel B: distance convergence ─────────────────────────────────────────
    for d in data:
        sc, rows, meta = d['sc'], d['rows'], d['meta']
        ts  = [r['t'] for r in rows]
        ds  = [r['dist_to_goal'] for r in rows]
        col = sc['color']
        ax_B.plot(ts, ds, '-', color=col, linewidth=1.5,
                  label=f'SC{sc["id"]}: {sc["name"]}')
        if meta['t_reached']:
            ax_B.axvline(meta['t_reached'], color=col, linestyle=':', alpha=0.6, lw=1.0)
            ax_B.annotate(f'{meta["t_reached"]:.1f}s',
                          xy=(meta['t_reached'], meta['final_dist']+0.1),
                          fontsize=7, color=col, ha='center')
        ax_B.axhline(sc['thre'], color=col, linestyle='-.', alpha=0.25, lw=0.8)

    ax_B.set_xlabel('Time (s)'); ax_B.set_ylabel('Distance to Goal (m)')
    ax_B.set_title('(B)  Goal Convergence', fontweight='bold', loc='left')
    ax_B.legend(loc='upper right', framealpha=0.85)
    ax_B.set_ylim(bottom=0); ax_B.grid(True, alpha=0.25, lw=0.5)

    # ── Panel C: velocity time-series ─────────────────────────────────────────
    ax_C2 = ax_C.twinx()
    for d in data:
        sc, rows = d['sc'], d['rows']
        ts  = [r['t'] for r in rows]
        vxs = [r['vx'] for r in rows]
        wzs = [r['wz'] for r in rows]
        col = sc['color']
        ax_C.plot(ts, vxs, '-',   color=col, linewidth=1.2,
                  label=f'vₓ SC{sc["id"]}', alpha=0.9)
        ax_C2.plot(ts, wzs, '--', color=col, linewidth=0.9,
                   label=f'ωz SC{sc["id"]}', alpha=0.7)

    ax_C.set_xlabel('Time (s)')
    ax_C.set_ylabel('Forward velocity vₓ (m/s)', color='#333')
    ax_C2.set_ylabel('Angular velocity ωz (rad/s)', color='#555')
    ax_C.set_title('(C)  Velocity Profiles', fontweight='bold', loc='left')
    # Combined legend
    h1, l1 = ax_C.get_legend_handles_labels()
    h2, l2 = ax_C2.get_legend_handles_labels()
    ax_C.legend(h1 + h2, l1 + l2, loc='upper right', fontsize=6.5, ncol=2, framealpha=0.8)
    ax_C.grid(True, alpha=0.25, lw=0.5)
    ax_C.axhline(0, color='k', lw=0.5)
    ax_C2.axhline(0, color='gray', lw=0.4, linestyle=':')

    # ── Panel T: metrics table ────────────────────────────────────────────────
    ax_T.axis('off')
    col_labels = ['Scenario', 'Goal', 't* (s)', 'ε_f (m)', 'RMS_e (m)', 'N_cmd']
    table_data = []
    for d in data:
        sc, meta = d['sc'], d['meta']
        gx, gy = sc['goal']
        table_data.append([
            f'SC{sc["id"]}\n{sc["name"]}',
            f'({gx:.0f},{gy:.0f})',
            f'{meta["t_reached"]:.1f}' if meta['t_reached'] else 'T/O',
            f'{meta["final_dist"]:.3f}',
            f'{meta["path_rms"]:.3f}',
            str(meta['cmd_count']),
        ])
    tbl = ax_T.table(
        cellText=table_data,
        colLabels=col_labels,
        cellLoc='center', loc='center',
        bbox=[0, 0, 1, 1],
    )
    tbl.auto_set_font_size(False)
    tbl.set_fontsize(7.5)
    for (r, c), cell in tbl.get_celld().items():
        if r == 0:
            cell.set_facecolor('#1565C0')
            cell.set_text_props(color='white', fontweight='bold')
        elif r % 2 == 1:
            cell.set_facecolor('#E3F2FD')
        cell.set_edgecolor('#CCCCCC')
    ax_T.set_title('(D)  Performance Metrics', fontweight='bold', loc='left', pad=2)

    # ── global annotation ────────────────────────────────────────────────────
    fig.text(0.5, 0.01,
             'Fig. 1.  Closed-loop E2E navigation validation of MapPilot planning stack '
             '(ROS2 Humble, C++ pathFollower/localPlanner/pct_adapter). '
             'Robot starts at origin; velocity-colored scatter shows local speed. '
             'ε_f: final goal distance; RMS_e: cross-track RMS error; N_cmd: '
             'cmd_vel messages received at 50 Hz.',
             ha='center', fontsize=7, style='italic', wrap=True,
             color='#444')

    out = '/tmp/e2e_paper_figure.png'
    plt.savefig(out, dpi=300, bbox_inches='tight', facecolor='white')
    print(f'Paper figure saved: {out}')
    plt.close(fig)


# ── GIF animation ─────────────────────────────────────────────────────────────

def make_gif(data):
    """
    Animated GIF: robot arrow + trail + stats panels.
    Each scenario plays at 5× real-time for a compact animation.
    """
    FPS      = 15
    SPEED    = 5      # playback speed multiplier
    TRAIL    = 80     # trail points
    PAUSE_FR = 20     # blank frames between scenarios

    # subsample: take every SPEED-th row → 4Hz effective data
    def subsamp(rows, every=SPEED):
        return rows[::every]

    all_frames = []   # list of (sc_info, rows_subsampled, frame_idx_in_scenario, is_pause)
    for d in data:
        sub = subsamp(d['rows'])
        for i, row in enumerate(sub):
            all_frames.append(('data', d['sc'], d['rows'], sub, i))
        for _ in range(PAUSE_FR):
            all_frames.append(('pause', d['sc'], None, None, None))

    total = len(all_frames)

    # ── figure setup ─────────────────────────────────────────────────────────
    fig = plt.figure(figsize=(11, 7), facecolor='#0d1117')
    ax_main = fig.add_axes([0.03, 0.12, 0.62, 0.82])
    ax_dist = fig.add_axes([0.70, 0.56, 0.28, 0.38])
    ax_vel  = fig.add_axes([0.70, 0.12, 0.28, 0.38])

    for ax in (ax_main, ax_dist, ax_vel):
        ax.set_facecolor('#161b22')
        for sp in ax.spines.values():
            sp.set_edgecolor('#30363d')
        ax.tick_params(colors='#8b949e')
        ax.xaxis.label.set_color('#8b949e')
        ax.yaxis.label.set_color('#8b949e')
        ax.title.set_color('#c9d1d9')

    ax_main.set_xlim(-1.5, 7.5); ax_main.set_ylim(-1.5, 7.5)
    ax_main.set_aspect('equal')
    ax_main.set_xlabel('X (m)'); ax_main.set_ylabel('Y (m)')
    ax_main.grid(True, alpha=0.15, color='#30363d')

    # Static: goal markers + reference paths
    for d in data:
        sc = d['sc']
        gx, gy = sc['goal']
        ax_main.plot([0, gx], [0, gy], '--', color=sc['color'], alpha=0.18, lw=0.8)
        ax_main.scatter([gx], [gy], s=180, color=sc['color'], marker='*',
                        zorder=5, alpha=0.7)
        ax_main.annotate(f"G{sc['id']}",
                         xy=(gx, gy), xytext=(gx+0.12, gy+0.15),
                         fontsize=7, color=sc['color'], alpha=0.8)
    ax_main.scatter([0], [0], s=100, color='white', marker='o',
                    zorder=10, label='Start')
    ax_main.set_title('MapPilot — E2E Closed-Loop Navigation', color='#c9d1d9',
                      fontsize=10, fontweight='bold', pad=6)

    # legend patches
    legend_patches = [
        mpatches.Patch(color=d['sc']['color'], label=f"SC{d['sc']['id']}: {d['sc']['name']}")
        for d in data
    ]
    ax_main.legend(handles=legend_patches, loc='lower right',
                   facecolor='#161b22', edgecolor='#30363d',
                   labelcolor='#c9d1d9', fontsize=7.5)

    # ── dynamic artists ──────────────────────────────────────────────────────
    # Each scenario gets its own trail line
    trail_lines = [ax_main.plot([], [], '-', color=d['sc']['color'],
                                linewidth=1.8, alpha=0.85, zorder=4)[0]
                   for d in data]

    # Robot arrow (single, updated each frame)
    robot_arrow = ax_main.annotate(
        '', xy=(0.5, 0), xytext=(0, 0),
        arrowprops=dict(arrowstyle='->', color='white', lw=2.0, mutation_scale=18),
        zorder=8,
    )

    # Robot body circle
    robot_body = plt.Circle((0, 0), 0.22, color='white', zorder=7, alpha=0.9)
    ax_main.add_patch(robot_body)

    # Status text
    status_txt = ax_main.text(
        0.03, 0.97, '', transform=ax_main.transAxes,
        fontsize=9, color='#c9d1d9', va='top', ha='left',
        fontweight='bold',
        bbox=dict(boxstyle='round,pad=0.3', facecolor='#21262d',
                  edgecolor='#30363d', alpha=0.85),
    )
    mode_txt = ax_main.text(
        0.03, 0.84, '', transform=ax_main.transAxes,
        fontsize=8, color='#58a6ff', va='top', ha='left',
    )

    # ── sub-panels setup ─────────────────────────────────────────────────────
    ax_dist.set_xlabel('Time (s)', fontsize=8); ax_dist.set_ylabel('Dist to goal (m)', fontsize=8)
    ax_dist.set_title('Convergence', fontsize=9)
    ax_dist.set_ylim(0, 7); ax_dist.grid(True, alpha=0.2, color='#30363d')

    ax_vel.set_xlabel('Time (s)', fontsize=8); ax_vel.set_ylabel('Velocity', fontsize=8)
    ax_vel.set_title('vₓ / ωz', fontsize=9)
    ax_vel.set_ylim(-1.2, 1.2); ax_vel.grid(True, alpha=0.2, color='#30363d')
    ax_vel.axhline(0, color='#8b949e', lw=0.5, alpha=0.5)

    dist_lines = [ax_dist.plot([], [], '-', color=d['sc']['color'],
                                linewidth=1.3, alpha=0.85)[0] for d in data]
    dist_thre_lines = []
    for d in data:
        l, = ax_dist.plot([], [], '-.', color=d['sc']['color'], alpha=0.3, lw=0.8)
        dist_thre_lines.append(l)

    vx_lines  = [ax_vel.plot([], [], '-',  color=d['sc']['color'],
                              linewidth=1.2, alpha=0.9)[0] for d in data]
    wz_lines  = [ax_vel.plot([], [], '--', color=d['sc']['color'],
                              linewidth=0.9, alpha=0.65)[0] for d in data]

    # Track drawn data per scenario (cumulative)
    sc_drawn = [{} for _ in SCENARIOS]   # {t:[], x:[], y:[], dist:[], vx:[], wz:[]}
    for i in range(len(SCENARIOS)):
        sc_drawn[i] = {'t':[], 'x':[], 'y':[], 'dist':[], 'vx':[], 'wz':[]}

    _current_sc_idx = [0]
    _pause_frame    = [False]

    # ── update function ──────────────────────────────────────────────────────
    def update(fi):
        frame_type, sc_info, all_rows, sub_rows, row_idx = all_frames[fi]

        sc_i = next(i for i, d in enumerate(data) if d['sc']['id'] == sc_info['id'])

        if frame_type == 'pause':
            robot_body.set_visible(False)
            robot_arrow.set_visible(False)
            status_txt.set_text(f'↻  SC{sc_info["id"]} complete')
            mode_txt.set_text('')
            return (robot_body, robot_arrow, status_txt, mode_txt,
                    *trail_lines, *dist_lines, *vx_lines, *wz_lines)

        robot_body.set_visible(True)
        robot_arrow.set_visible(True)

        row = sub_rows[row_idx]
        x, y = row['x'], row['y']
        yaw_rad = math.radians(row['yaw_deg'])
        vx, wz  = row['vx'], row['wz']
        dist    = row['dist_to_goal']
        t       = row['t']
        gx, gy  = sc_info['goal']

        # Accumulate drawn data
        sc_drawn[sc_i]['t'].append(t)
        sc_drawn[sc_i]['x'].append(x)
        sc_drawn[sc_i]['y'].append(y)
        sc_drawn[sc_i]['dist'].append(dist)
        sc_drawn[sc_i]['vx'].append(vx)
        sc_drawn[sc_i]['wz'].append(wz)

        # Robot body
        robot_body.center = (x, y)

        # Robot arrow (direction = yaw)
        arrow_len = 0.45
        ax = x + arrow_len * math.cos(yaw_rad)
        ay = y + arrow_len * math.sin(yaw_rad)
        robot_arrow.xy = (ax, ay)
        robot_arrow.xytext = (x, y)

        # Trail (recent TRAIL points of this scenario)
        xs_trail = sc_drawn[sc_i]['x'][-TRAIL:]
        ys_trail = sc_drawn[sc_i]['y'][-TRAIL:]
        trail_lines[sc_i].set_data(xs_trail, ys_trail)

        # Status text
        if dist < sc_info['thre']:
            mode = '● GOAL REACHED'
            mode_col = '#2ea043'
        elif abs(wz) > 0.4 and abs(vx) < 0.05:
            mode = '↻ ALIGNING'
            mode_col = '#e3b341'
        elif vx > 0.05:
            mode = '→ FOLLOWING PATH'
            mode_col = '#58a6ff'
        else:
            mode = '■ STOPPED'
            mode_col = '#8b949e'
        mode_txt.set_text(mode)
        mode_txt.set_color(mode_col)

        status_txt.set_text(
            f'SC{sc_info["id"]}: {sc_info["name"]}\n'
            f't={t:.1f}s  dist={dist:.2f}m\n'
            f'vₓ={vx:.2f}  ωz={wz:.2f}'
        )

        # Sub-panels: show all drawn scenarios
        for i, d in enumerate(data):
            td = sc_drawn[i]['t']
            dist_lines[i].set_data(td, sc_drawn[i]['dist'])
            vx_lines[i].set_data(td, sc_drawn[i]['vx'])
            wz_lines[i].set_data(td, sc_drawn[i]['wz'])
            if td:
                dist_thre_lines[i].set_data([td[0], td[-1]],
                                            [d['sc']['thre'], d['sc']['thre']])

        # Adjust x-limits of sub-panels
        all_t = [t2 for d2 in sc_drawn for t2 in d2['t']]
        if all_t:
            t_max = max(all_t)
            ax_dist.set_xlim(0, max(t_max + 2, 10))
            ax_vel.set_xlim(0, max(t_max + 2, 10))

        return (robot_body, robot_arrow, status_txt, mode_txt,
                *trail_lines, *dist_lines, *vx_lines, *wz_lines,
                *dist_thre_lines)

    anim = FuncAnimation(fig, update, frames=total,
                         interval=1000//FPS, blit=True, repeat=True)

    out = '/tmp/e2e_nav_animated.gif'
    print(f'Rendering GIF ({total} frames at {FPS}fps)...')
    writer = PillowWriter(fps=FPS)
    anim.save(out, writer=writer, dpi=100)
    print(f'GIF saved: {out}')
    plt.close(fig)


# ── main ──────────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    print('Loading scenario data...')
    data = load_all()
    for d in data:
        sc, meta = d['sc'], d['meta']
        print(f"  SC{sc['id']} {sc['name']}: "
              f"goal_reached={meta['goal_reached']} "
              f"t={meta['t_reached']}s "
              f"dist={meta['final_dist']}m "
              f"rms={meta['path_rms']}m "
              f"cmds={meta['cmd_count']}")

    print('\nGenerating paper figure...')
    make_paper_figure(data)

    print('\nGenerating animated GIF...')
    make_gif(data)

    print('\nDone.')
