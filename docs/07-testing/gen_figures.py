"""Generate publication-quality SLAM test figures — white academic theme.

Key improvements over v1:
- Solid colored lines with direction arrows (not coolwarm scatter)
- Time annotations at key waypoints
- Large, distinct start/end markers
- Clean white background, serif fonts
- Dual-algorithm comparison (Point-LIO vs Fast-LIO2) on same plot
"""
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import json
import os
from matplotlib.patches import FancyArrowPatch

plt.rcParams.update({
    'font.family': 'serif',
    'font.serif': ['Times New Roman', 'DejaVu Serif'],
    'font.size': 11,
    'axes.labelsize': 13,
    'axes.titlesize': 14,
    'legend.fontsize': 10,
    'xtick.labelsize': 10,
    'ytick.labelsize': 10,
    'figure.facecolor': 'white',
    'axes.facecolor': 'white',
    'axes.grid': True,
    'grid.alpha': 0.25,
    'grid.linestyle': '--',
    'axes.spines.top': False,
    'axes.spines.right': False,
})

OUT = 'docs/07-testing/'

# ===== Data =====
corridor_data = [
    (10, 7.11, -0.20, 0.04), (20, 16.00, -0.46, 0.03),
    (30, 18.06, 4.91, -0.14), (40, 15.23, 13.00, -0.48),
    (45, 11.39, 13.38, -0.47), (50, 6.98, 13.61, -0.51),
    (55, 2.19, 13.81, -0.49), (65, -8.11, 13.94, -0.57),
    (70, -13.07, 13.65, -0.55), (85, -29.42, 14.04, -0.62),
    (95, -34.20, 8.21, -0.40), (105, -36.94, -0.27, -0.16),
    (115, -47.49, -0.05, -0.07), (125, -59.05, 0.23, -0.30),
    (135, -70.71, 0.44, -0.35), (145, -82.60, 0.56, -0.38),
    (155, -94.11, 0.71, -0.36), (165, -102.69, 3.75, -0.32),
    (175, -99.51, 12.11, -0.70), (185, -92.27, 14.97, -0.84),
    (195, -82.34, 14.76, -0.88), (205, -71.85, 14.51, -0.83),
    (210, -67.15, 14.61, -0.86), (215, -61.19, 14.36, -0.88),
    (225, -50.63, 14.31, -0.82), (235, -39.75, 14.26, -0.77),
    (245, -33.77, 5.22, -0.34), (255, -30.43, -0.34, -0.08),
    (260, -26.05, 0.38, -0.12), (265, -21.66, -0.07, -0.10),
    (275, -11.40, 0.26, -0.06), (285, -1.23, 0.04, 0.04),
    (295, -0.01, 0.00, 0.00),
]

grass_data = [
    (3, 27.113, 3.775, -9.388),
    (15, 32.474, -16.117, -19.379),
    (18, 31.262, -20.322, -15.293),
    (21, 29.526, -24.382, -10.517),
    (24, 27.807, -28.683, -5.259),
    (27, 26.665, -31.183, -1.866),
]

slope_data = [
    (18, 46.885, -4.340, -0.753),
    (21, 53.891, -4.070, -0.869),
    (24, 60.887, -4.040, -0.659),
    (27, 67.293, -3.439, -0.635),
    (33, 75.155, 4.901, -0.830),
    (36, 75.973, 12.012, -1.200),
    (39, 75.938, 18.145, -1.470),
    (51, 78.554, 52.285, -3.459),
    (57, 78.772, 67.949, -4.415),
    (60, 78.625, 75.625, -4.743),
    (63, 79.030, 84.916, -5.214),
    (66, 79.598, 93.513, -5.459),
    (69, 79.903, 100.915, -5.695),
    (72, 80.526, 108.375, -5.976),
    (75, 79.271, 115.875, -6.179),
    (78, 74.574, 121.608, -6.209),
    (81, 66.824, 124.106, -6.186),
    (84, 59.754, 124.897, -6.255),
    (87, 52.445, 124.840, -6.222),
    (90, 44.942, 124.281, -6.285),
    (96, 25.698, 124.234, -6.380),
    (99, 15.961, 124.464, -6.406),
    (105, 0.931, 119.100, -6.066),
    (108, 0.462, 110.324, -5.724),
    (111, 1.279, 103.412, -5.138),
    (120, -0.325, 85.893, -3.947),
    (123, -1.272, 79.464, -3.741),
    (126, -2.148, 71.148, -3.254),
    (129, -2.687, 64.114, -2.698),
    (132, -3.006, 57.235, -2.128),
    (135, -3.409, 49.046, -1.421),
    (138, -4.238, 39.514, -0.615),
    (141, -4.144, 30.278, 0.289),
    (144, -4.335, 23.242, 0.984),
    (162, -1.303, -0.078, 3.998),
]

COLORS = {
    'corridor': '#1565C0',  # deeper blue
    'grass':    '#2E7D32',  # deeper green
    'slope':    '#C62828',  # deeper red
}


def extract(data):
    t = np.array([d[0] for d in data])
    x = np.array([d[1] for d in data])
    y = np.array([d[2] for d in data])
    z = np.array([d[3] for d in data])
    drift = np.sqrt((x - x[0])**2 + (y - y[0])**2 + (z - z[0])**2)
    cum = np.zeros_like(t, dtype=float)
    speed = np.zeros_like(t, dtype=float)
    for i in range(1, len(t)):
        cum[i] = cum[i-1] + np.sqrt((x[i]-x[i-1])**2 + (y[i]-y[i-1])**2 + (z[i]-z[i-1])**2)
        dt = t[i] - t[i-1]
        speed[i] = np.sqrt((x[i]-x[i-1])**2 + (y[i]-y[i-1])**2) / dt if dt > 0 else 0
    return t, x, y, z, drift, cum, speed


ct, cx, cy, cz, cd, cc, cs = extract(corridor_data)
gt, gx, gy, gz, gd, gc, gs_ = extract(grass_data)
st, sx, sy, sz, sd, s_cum, ss = extract(slope_data)


def add_direction_arrows(ax, x, y, color, n_arrows=8, hw=0.4, hl=0.6, lw=1.5):
    """Add direction arrows along a trajectory path."""
    n = len(x)
    if n < 3:
        return
    indices = np.linspace(1, n - 2, min(n_arrows, n - 2), dtype=int)
    for idx in indices:
        dx = x[idx+1] - x[idx-1]
        dy = y[idx+1] - y[idx-1]
        norm = np.sqrt(dx**2 + dy**2)
        if norm < 1e-6:
            continue
        ax.annotate('', xy=(x[idx] + dx/norm*0.01, y[idx] + dy/norm*0.01),
                     xytext=(x[idx], y[idx]),
                     arrowprops=dict(arrowstyle='->', color=color,
                                     lw=lw, mutation_scale=15))


def add_time_labels(ax, t, x, y, indices, color='#333333', fontsize=8, offset=(8, 8)):
    """Annotate selected waypoints with time labels."""
    for idx in indices:
        if 0 <= idx < len(t):
            ax.annotate(f'{t[idx]:.0f}s',
                        (x[idx], y[idx]),
                        textcoords='offset points', xytext=offset,
                        fontsize=fontsize, color=color,
                        bbox=dict(boxstyle='round,pad=0.15', fc='white',
                                  ec='#cccccc', alpha=0.85))


# ================================================================
# Fig 1 — Corridor 2D trajectory
# ================================================================
fig, ax = plt.subplots(figsize=(12, 5))
ax.plot(cx, cy, '-', color=COLORS['corridor'], lw=2.5, alpha=0.9, zorder=3)
ax.plot(cx, cy, 'o', color=COLORS['corridor'], ms=4, alpha=0.5, zorder=4)
add_direction_arrows(ax, cx, cy, COLORS['corridor'], n_arrows=10)
# Time labels at key turns
time_indices = [0, 3, 9, 17, 20, 26, -1]
add_time_labels(ax, ct, cx, cy, time_indices, offset=(10, 8))
# Start / End
ax.plot(cx[0], cy[0], 'o', color='#2E7D32', ms=16, zorder=10,
        markeredgecolor='black', markeredgewidth=1.5, label='Start')
ax.plot(cx[-1], cy[-1], 's', color='#C62828', ms=13, zorder=10,
        markeredgecolor='black', markeredgewidth=1.5, label='End')
# Loop error annotation
ax.annotate(f'Loop closure error ≈ 7.1 m',
            xy=(cx[-1], cy[-1]), xytext=(30, 25), textcoords='offset points',
            fontsize=11, fontweight='bold', color='#C62828',
            arrowprops=dict(arrowstyle='->', lw=1.5, color='#C62828'),
            bbox=dict(boxstyle='round,pad=0.3', fc='#FFF3F3', ec='#C62828', alpha=0.9))
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_title('(a) Point-LIO 2D Trajectory — Corridor (Indoor, 295 s, ~268 m)', fontweight='bold')
ax.set_aspect('equal')
ax.legend(loc='upper left', framealpha=0.9, fontsize=11)
plt.tight_layout()
fig.savefig(OUT + 'fig_corridor_traj.png', dpi=250, bbox_inches='tight')
print('Saved: fig_corridor_traj.png')
plt.close(fig)


# ================================================================
# Fig 2 — Slope 2D trajectory
# ================================================================
fig, ax = plt.subplots(figsize=(8, 10))
ax.plot(sx, sy, '-', color=COLORS['slope'], lw=2.5, alpha=0.9, zorder=3)
ax.plot(sx, sy, 'o', color=COLORS['slope'], ms=4, alpha=0.5, zorder=4)
add_direction_arrows(ax, sx, sy, COLORS['slope'], n_arrows=12)
# Time labels at key turns
time_indices_s = [0, 4, 10, 15, 20, 22, 28, -1]
add_time_labels(ax, st, sx, sy, time_indices_s, offset=(10, 8))
# Start / End
ax.plot(sx[0], sy[0], 'o', color='#2E7D32', ms=16, zorder=10,
        markeredgecolor='black', markeredgewidth=1.5, label='Start')
ax.plot(sx[-1], sy[-1], 's', color='#C62828', ms=13, zorder=10,
        markeredgecolor='black', markeredgewidth=1.5, label='End')
# Farthest point
idx_far = np.argmax(sd)
ax.plot(sx[idx_far], sy[idx_far], 'D', color='#FF8F00', ms=12, zorder=10,
        markeredgecolor='black', markeredgewidth=1.2,
        label=f'Farthest ({sd[idx_far]:.0f} m from start)')
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_title('(b) Point-LIO 2D Trajectory — Slope (Outdoor, 162 s, ~346 m)', fontweight='bold')
ax.set_aspect('equal')
ax.legend(loc='lower right', framealpha=0.9, fontsize=11)
plt.tight_layout()
fig.savefig(OUT + 'fig_slope_traj.png', dpi=250, bbox_inches='tight')
print('Saved: fig_slope_traj.png')
plt.close(fig)


# ================================================================
# Fig 3 — Grass 2D trajectory
# ================================================================
fig, ax = plt.subplots(figsize=(6, 7))
ax.plot(gx, gy, '-', color=COLORS['grass'], lw=3, alpha=0.9, zorder=3)
ax.plot(gx, gy, 'o', color=COLORS['grass'], ms=8, alpha=0.6, zorder=4)
add_direction_arrows(ax, gx, gy, COLORS['grass'], n_arrows=4)
# Time labels (all points since only 6)
for i in range(len(gt)):
    off = (10, 8) if i % 2 == 0 else (-10, -15)
    add_time_labels(ax, gt, gx, gy, [i], offset=off)
# Start / End
ax.plot(gx[0], gy[0], 'o', color='#2E7D32', ms=16, zorder=10,
        markeredgecolor='black', markeredgewidth=1.5, label='Start')
ax.plot(gx[-1], gy[-1], 's', color='#C62828', ms=13, zorder=10,
        markeredgecolor='black', markeredgewidth=1.5, label='End')
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_title('(c) Point-LIO 2D Trajectory — Grass (Outdoor, 27 s, ~47 m)', fontweight='bold')
ax.set_aspect('equal')
ax.legend(loc='upper right', framealpha=0.9, fontsize=11)
plt.tight_layout()
fig.savefig(OUT + 'fig_grass_traj.png', dpi=250, bbox_inches='tight')
print('Saved: fig_grass_traj.png')
plt.close(fig)


# ================================================================
# Fig 4 — Overlay: All 3 trajectories on one plot (CENTERED)
# ================================================================
fig, ax = plt.subplots(figsize=(12, 8))

cx0, cy0 = cx - cx[0], cy - cy[0]
gx0, gy0 = gx - gx[0], gy - gy[0]
sx0, sy0 = sx - sx[0], sy - sy[0]

for xd, yd, c, name, mk in [
    (cx0, cy0, COLORS['corridor'], f'Corridor (indoor, 295 s, ~268 m)', '-'),
    (gx0, gy0, COLORS['grass'],    f'Grass (outdoor, 27 s, ~47 m)', '-'),
    (sx0, sy0, COLORS['slope'],    f'Slope (outdoor, 162 s, ~346 m)', '-'),
]:
    ax.plot(xd, yd, mk, color=c, lw=2.5, alpha=0.9, label=name, zorder=3)
    ax.plot(xd, yd, 'o', color=c, ms=3, alpha=0.4, zorder=4)
    add_direction_arrows(ax, xd, yd, c, n_arrows=6)
    # Start (shared origin)
    ax.plot(xd[0], yd[0], 'o', color=c, ms=12, zorder=10,
            markeredgecolor='black', markeredgewidth=1.2)
    # End
    ax.plot(xd[-1], yd[-1], 's', color=c, ms=10, zorder=10,
            markeredgecolor='black', markeredgewidth=1.2)

# Legend entries for markers
ax.plot([], [], 'o', color='gray', ms=10, markeredgecolor='k', markeredgewidth=1, label='Start (origin)')
ax.plot([], [], 's', color='gray', ms=9, markeredgecolor='k', markeredgewidth=1, label='End point')

ax.set_xlabel('$\\Delta X$ from start (m)')
ax.set_ylabel('$\\Delta Y$ from start (m)')
ax.set_title('Overlaid 2D Trajectories — All Three Terrain Scenarios (Origin-Aligned)', fontweight='bold')
ax.legend(loc='upper left', framealpha=0.9, fontsize=11)
ax.set_aspect('equal')
plt.tight_layout()
fig.savefig(OUT + 'fig_overlay_traj.png', dpi=250, bbox_inches='tight')
print('Saved: fig_overlay_traj.png')
plt.close(fig)


# ================================================================
# Fig 5 — Z-axis drift comparison (all 3 on one plot)
# ================================================================
fig, ax = plt.subplots(figsize=(10, 5))
ax.plot(ct, cz, '-o', color=COLORS['corridor'], lw=2, ms=3, alpha=0.8,
        label=f'Corridor (peak = {cz.min():.2f} m)')
ax.plot(gt, gz, '-s', color=COLORS['grass'], lw=2.5, ms=6,
        label=f'Grass (peak = {gz.min():.1f} m)')
ax.plot(st, sz, '-^', color=COLORS['slope'], lw=2, ms=3,
        label=f'Slope (peak = {sz.min():.1f} m)')
ax.axhspan(-2, 2, alpha=0.08, color='green', label='$\\pm$2 m corridor spec')
ax.axhline(0, color='k', lw=0.5, alpha=0.3)
ax.set_xlabel('Time (s)')
ax.set_ylabel('Z position (m)')
ax.set_title('Z-axis Drift Comparison Across Three Terrain Scenarios', fontweight='bold')
ax.legend(loc='lower left', framealpha=0.9)
plt.tight_layout()
fig.savefig(OUT + 'fig_z_drift_compare.png', dpi=250, bbox_inches='tight')
print('Saved: fig_z_drift_compare.png')
plt.close(fig)


# ================================================================
# Fig 6 — Displacement from start (all 3)
# ================================================================
fig, ax = plt.subplots(figsize=(10, 5))
ax.plot(ct, cd, '-o', color=COLORS['corridor'], lw=2, ms=3, alpha=0.8,
        label=f'Corridor (max = {cd.max():.0f} m)')
ax.plot(gt, gd, '-s', color=COLORS['grass'], lw=2.5, ms=6,
        label=f'Grass (max = {gd.max():.0f} m)')
ax.plot(st, sd, '-^', color=COLORS['slope'], lw=2, ms=3,
        label=f'Slope (max = {sd.max():.0f} m)')
ax.fill_between(ct, cd, alpha=0.08, color=COLORS['corridor'])
ax.fill_between(st, sd, alpha=0.08, color=COLORS['slope'])
ax.set_xlabel('Time (s)')
ax.set_ylabel('Displacement from start (m)')
ax.set_title('Displacement from Start Point Over Time', fontweight='bold')
ax.legend(loc='upper left', framealpha=0.9)
plt.tight_layout()
fig.savefig(OUT + 'fig_displacement_compare.png', dpi=250, bbox_inches='tight')
print('Saved: fig_displacement_compare.png')
plt.close(fig)


# ================================================================
# Fig 7 — Cumulative distance (all 3)
# ================================================================
fig, ax = plt.subplots(figsize=(10, 5))
ax.plot(ct, cc, '-o', color=COLORS['corridor'], lw=2, ms=3,
        label=f'Corridor ({cc[-1]:.0f} m)')
ax.plot(gt, gc, '-s', color=COLORS['grass'], lw=2.5, ms=6,
        label=f'Grass ({gc[-1]:.0f} m)')
ax.plot(st, s_cum, '-^', color=COLORS['slope'], lw=2, ms=3,
        label=f'Slope ({s_cum[-1]:.0f} m)')
ax.set_xlabel('Time (s)')
ax.set_ylabel('Cumulative travel distance (m)')
ax.set_title('Cumulative Travel Distance Over Time', fontweight='bold')
ax.legend(loc='upper left', framealpha=0.9)
plt.tight_layout()
fig.savefig(OUT + 'fig_cumulative_dist.png', dpi=250, bbox_inches='tight')
print('Saved: fig_cumulative_dist.png')
plt.close(fig)


# ================================================================
# Fig 8 — Speed profiles (all 3)
# ================================================================
fig, ax = plt.subplots(figsize=(10, 5))
ax.plot(ct, cs, '-o', color=COLORS['corridor'], lw=1.5, ms=3, alpha=0.8,
        label=f'Corridor (avg = {np.mean(cs[cs>0]):.2f} m/s)')
ax.plot(gt, gs_, '-s', color=COLORS['grass'], lw=2, ms=6,
        label=f'Grass (avg = {np.mean(gs_[gs_>0]):.2f} m/s)')
ax.plot(st, ss, '-^', color=COLORS['slope'], lw=1.5, ms=3,
        label=f'Slope (avg = {np.mean(ss[ss>0]):.2f} m/s)')
ax.set_xlabel('Time (s)')
ax.set_ylabel('Estimated ground speed (m/s)')
ax.set_title('Estimated Ground Speed Across Scenarios', fontweight='bold')
ax.legend(loc='upper right', framealpha=0.9)
plt.tight_layout()
fig.savefig(OUT + 'fig_speed_compare.png', dpi=250, bbox_inches='tight')
print('Saved: fig_speed_compare.png')
plt.close(fig)


# ================================================================
# Fig 9 — Corridor XYZ components vs time
# ================================================================
fig, axes = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
for ax, (vals, lbl, c) in zip(axes, [
    (cx, 'X', COLORS['corridor']), (cy, 'Y', '#E65100'), (cz, 'Z', COLORS['grass'])
]):
    ax.plot(ct, vals, '-o', color=c, lw=1.5, ms=3)
    ax.set_ylabel(f'{lbl} (m)')
    ax.axhline(0, color='k', lw=0.3, alpha=0.3)
axes[0].set_title('Position Components vs Time — Corridor Dataset', fontweight='bold')
axes[-1].set_xlabel('Time (s)')
plt.tight_layout()
fig.savefig(OUT + 'fig_corridor_xyz.png', dpi=250, bbox_inches='tight')
print('Saved: fig_corridor_xyz.png')
plt.close(fig)


# ================================================================
# Fig 10 — Slope XYZ components vs time
# ================================================================
fig, axes = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
for ax, (vals, lbl, c) in zip(axes, [
    (sx, 'X', COLORS['slope']), (sy, 'Y', '#E65100'), (sz, 'Z', COLORS['grass'])
]):
    ax.plot(st, vals, '-o', color=c, lw=1.5, ms=3)
    ax.set_ylabel(f'{lbl} (m)')
    ax.axhline(0, color='k', lw=0.3, alpha=0.3)
axes[0].set_title('Position Components vs Time — Slope Dataset', fontweight='bold')
axes[-1].set_xlabel('Time (s)')
plt.tight_layout()
fig.savefig(OUT + 'fig_slope_xyz.png', dpi=250, bbox_inches='tight')
print('Saved: fig_slope_xyz.png')
plt.close(fig)


# ================================================================
# Fig 11 — 3D trajectory: Corridor
# ================================================================
fig = plt.figure(figsize=(10, 7))
ax = fig.add_subplot(111, projection='3d')
ax.plot(cx, cy, cz, '-', color=COLORS['corridor'], lw=2, alpha=0.9)
ax.scatter(cx, cy, cz, c=ct, cmap='viridis', s=40, edgecolors='k',
           linewidths=0.3, zorder=5)
ax.plot([cx[0]], [cy[0]], [cz[0]], 'o', color='#2E7D32', ms=14, zorder=10,
        markeredgecolor='k', markeredgewidth=1.2)
ax.plot([cx[-1]], [cy[-1]], [cz[-1]], 's', color='#C62828', ms=11, zorder=10,
        markeredgecolor='k', markeredgewidth=1.2)
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_zlabel('Z (m)')
ax.set_title('3D Trajectory — Corridor', fontweight='bold')
ax.view_init(elev=25, azim=-60)
plt.tight_layout()
fig.savefig(OUT + 'fig_corridor_3d.png', dpi=250, bbox_inches='tight')
print('Saved: fig_corridor_3d.png')
plt.close(fig)


# ================================================================
# Fig 12 — Bar chart: quantitative summary
# ================================================================
fig, ax = plt.subplots(figsize=(10, 5))
labels = ['Duration\n(s)', 'Total dist.\n(m)', 'Max displ.\n(m)',
          '|Z| peak\n(m)', 'Avg speed\n(m/s)']
c_vals = [295, cc[-1], cd.max(), abs(cz.min()), np.mean(cs[cs>0])]
g_vals = [27, gc[-1], gd.max(), abs(gz.min()), np.mean(gs_[gs_>0])]
s_vals = [162, s_cum[-1], sd.max(), abs(sz.min()), np.mean(ss[ss>0])]

x = np.arange(len(labels))
w = 0.25
b1 = ax.bar(x - w, c_vals, w, label='Corridor', color=COLORS['corridor'], alpha=0.85)
b2 = ax.bar(x,     g_vals, w, label='Grass',    color=COLORS['grass'], alpha=0.85)
b3 = ax.bar(x + w, s_vals, w, label='Slope',    color=COLORS['slope'], alpha=0.85)

for bars in [b1, b2, b3]:
    for bar in bars:
        h = bar.get_height()
        ax.text(bar.get_x() + bar.get_width()/2., h + 1,
                f'{h:.1f}', ha='center', va='bottom', fontsize=8)

ax.set_xticks(x)
ax.set_xticklabels(labels)
ax.set_title('Quantitative Comparison of SLAM Performance Across Terrain Scenarios', fontweight='bold')
ax.legend(framealpha=0.9)
plt.tight_layout()
fig.savefig(OUT + 'fig_bar_compare.png', dpi=250, bbox_inches='tight')
print('Saved: fig_bar_compare.png')
plt.close(fig)


# ================================================================
# DUAL ALGORITHM COMPARISON — Point-LIO vs Fast-LIO2
# ================================================================
dual_json = os.path.join(OUT, 'dual_slam_corridor.json')
if os.path.exists(dual_json):
    with open(dual_json) as f:
        dual = json.load(f)

    def load_traj(pts):
        t = np.array([p['t'] for p in pts])
        x = np.array([p['x'] for p in pts])
        y = np.array([p['y'] for p in pts])
        z = np.array([p['z'] for p in pts])
        return t, x, y, z

    pt, px, py, pz = load_traj(dual['pointlio'])
    ft, fx, fy, fz = load_traj(dual['fastlio2'])

    # Auto-detect divergence: Fast-LIO2 position jumps > 50m between samples
    f_good = len(ft)
    for i in range(1, len(ft)):
        dx = abs(fx[i] - fx[i-1])
        dy = abs(fy[i] - fy[i-1])
        dz = abs(fz[i] - fz[i-1])
        if dx > 50 or dy > 50 or dz > 50:
            f_good = i
            break
    ft_g, fx_g, fy_g, fz_g = ft[:f_good], fx[:f_good], fy[:f_good], fz[:f_good]

    ALGO_COLORS = {
        'pointlio': '#1565C0',   # blue
        'fastlio2': '#E65100',   # deep orange
    }

    # ================================================================
    # Fig 13 — Dual Algorithm 2D XY Trajectory Comparison (origin-aligned)
    # ================================================================
    fig, ax = plt.subplots(figsize=(12, 6))

    # Origin-align both trajectories for fair comparison
    px0, py0 = px - px[0], py - py[0]
    fx0, fy0 = fx_g - fx_g[0], fy_g - fy_g[0]

    # Point-LIO trajectory
    ax.plot(px0, py0, '-', color=ALGO_COLORS['pointlio'], lw=2.5, alpha=0.9,
            label=f'Point-LIO ({pt[-1]:.0f} s)', zorder=4)
    add_direction_arrows(ax, px0, py0, ALGO_COLORS['pointlio'], n_arrows=10)
    ax.plot(px0[-1], py0[-1], 's', color=ALGO_COLORS['pointlio'], ms=13, zorder=10,
            markeredgecolor='black', markeredgewidth=1.5)

    # Fast-LIO2 trajectory
    if f_good < len(ft):
        f2_label = f'Fast-LIO2 (diverges at {ft_g[-1]:.0f} s)'
    else:
        f2_label = f'Fast-LIO2 ({ft_g[-1]:.0f} s)'
    ax.plot(fx0, fy0, '-', color=ALGO_COLORS['fastlio2'], lw=2.5, alpha=0.9,
            label=f2_label, zorder=3)
    add_direction_arrows(ax, fx0, fy0, ALGO_COLORS['fastlio2'], n_arrows=8)
    if f_good < len(ft):
        ax.plot(fx0[-1], fy0[-1], 'X', color='#C62828', ms=16, zorder=10,
                markeredgecolor='black', markeredgewidth=1.2)
        ax.annotate(f'Tracking lost\n(t = {ft_g[-1]:.0f} s)',
                    (fx0[-1], fy0[-1]), textcoords='offset points', xytext=(-15, -25),
                    fontsize=10, fontweight='bold', color='#C62828',
                    arrowprops=dict(arrowstyle='->', lw=1.2, color='#C62828'),
                    bbox=dict(boxstyle='round,pad=0.2', fc='#FFF3F3', ec='#C62828', alpha=0.9))
    else:
        ax.plot(fx0[-1], fy0[-1], 's', color=ALGO_COLORS['fastlio2'], ms=13, zorder=10,
                markeredgecolor='black', markeredgewidth=1.5)

    # Shared start marker
    ax.plot(0, 0, 'o', color='#2E7D32', ms=16, zorder=10,
            markeredgecolor='black', markeredgewidth=1.5, label='Start (shared origin)')

    ax.set_xlabel('$\\Delta X$ from start (m)')
    ax.set_ylabel('$\\Delta Y$ from start (m)')
    ax.set_title('Point-LIO vs Fast-LIO2 — 2D Trajectory (Corridor, Origin-Aligned)',
                 fontweight='bold')
    ax.set_aspect('equal')
    ax.legend(loc='upper left', framealpha=0.9, fontsize=11)
    plt.tight_layout()
    fig.savefig(OUT + 'fig_dual_traj_xy.png', dpi=250, bbox_inches='tight')
    print('Saved: fig_dual_traj_xy.png')
    plt.close(fig)

    # ================================================================
    # Fig 14 — Dual Algorithm Z-drift Comparison
    # ================================================================
    fig, ax = plt.subplots(figsize=(10, 5))
    ax.plot(pt, pz, '-o', color=ALGO_COLORS['pointlio'], lw=2, ms=3, alpha=0.8,
            label=f'Point-LIO (peak |Z| = {np.max(np.abs(pz)):.1f} m)')
    ax.plot(ft_g, fz_g, '-s', color=ALGO_COLORS['fastlio2'], lw=2, ms=3, alpha=0.8,
            label=f'Fast-LIO2 (peak |Z| = {np.max(np.abs(fz_g)):.1f} m)')
    if f_good < len(ft):
        ax.axvline(ft_g[-1], color='#C62828', lw=1.5, ls='--', alpha=0.7,
                   label=f'Fast-LIO2 diverges at {ft_g[-1]:.0f} s')
    ax.axhspan(-2, 2, alpha=0.08, color='green', label='±2 m spec')
    ax.axhline(0, color='k', lw=0.5, alpha=0.3)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Z position (m)')
    ax.set_title('Z-axis Drift: Point-LIO vs Fast-LIO2 (Corridor)', fontweight='bold')
    ax.legend(loc='best', framealpha=0.9)
    plt.tight_layout()
    fig.savefig(OUT + 'fig_dual_z_drift.png', dpi=250, bbox_inches='tight')
    print('Saved: fig_dual_z_drift.png')
    plt.close(fig)

    # ================================================================
    # Fig 15 — Dual Algorithm XYZ Components vs Time (origin-aligned)
    # ================================================================
    fig, axes = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
    comp_names = ['$\\Delta$X', '$\\Delta$Y', '$\\Delta$Z']
    p_comps = [px - px[0], py - py[0], pz - pz[0]]
    f_comps = [fx_g - fx_g[0], fy_g - fy_g[0], fz_g - fz_g[0]]

    for ax, pv, fv, name in zip(axes, p_comps, f_comps, comp_names):
        ax.plot(pt, pv, '-', color=ALGO_COLORS['pointlio'], lw=1.8, alpha=0.9,
                label='Point-LIO')
        ax.plot(ft_g, fv, '-', color=ALGO_COLORS['fastlio2'], lw=1.8, alpha=0.9,
                label='Fast-LIO2')
        if f_good < len(ft):
            ax.axvline(ft_g[-1], color='#C62828', lw=1, ls='--', alpha=0.5)
        ax.set_ylabel(f'{name} (m)')
        ax.axhline(0, color='k', lw=0.3, alpha=0.3)
        if name == '$\\Delta$X':
            ax.legend(loc='upper right', fontsize=9, framealpha=0.8)

    axes[0].set_title('Position Components: Point-LIO vs Fast-LIO2 (Corridor)',
                       fontweight='bold')
    axes[-1].set_xlabel('Time (s)')
    plt.tight_layout()
    fig.savefig(OUT + 'fig_dual_xyz.png', dpi=250, bbox_inches='tight')
    print('Saved: fig_dual_xyz.png')
    plt.close(fig)

    # ================================================================
    # Fig 16 — Dual Algorithm Displacement from Start
    # ================================================================
    fig, ax = plt.subplots(figsize=(10, 5))
    p_disp = np.sqrt((px - px[0])**2 + (py - py[0])**2)
    f_disp = np.sqrt((fx_g - fx_g[0])**2 + (fy_g - fy_g[0])**2)
    ax.plot(pt, p_disp, '-', color=ALGO_COLORS['pointlio'], lw=2, alpha=0.9,
            label=f'Point-LIO (max = {p_disp.max():.0f} m)')
    ax.plot(ft_g, f_disp, '-', color=ALGO_COLORS['fastlio2'], lw=2, alpha=0.9,
            label=f'Fast-LIO2 (max = {f_disp.max():.0f} m)')
    if f_good < len(ft):
        ax.axvline(ft_g[-1], color='#C62828', lw=1.5, ls='--', alpha=0.7,
                   label=f'Fast-LIO2 diverges')
    ax.fill_between(pt, p_disp, alpha=0.06, color=ALGO_COLORS['pointlio'])
    ax.fill_between(ft_g, f_disp, alpha=0.06, color=ALGO_COLORS['fastlio2'])
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('2D Displacement from start (m)')
    ax.set_title('Displacement from Start: Point-LIO vs Fast-LIO2', fontweight='bold')
    ax.legend(loc='upper left', framealpha=0.9)
    plt.tight_layout()
    fig.savefig(OUT + 'fig_dual_displacement.png', dpi=250, bbox_inches='tight')
    print('Saved: fig_dual_displacement.png')
    plt.close(fig)

    print('\n4 dual-algorithm comparison figures generated.')
else:
    print(f'\nNo dual_slam_corridor.json found, skipping dual comparison figures.')


print('\nAll figures generated successfully.')
