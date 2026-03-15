"""Generate analysis figures for outdoor terrain SLAM tests."""
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np
from matplotlib.collections import LineCollection

plt.style.use('dark_background')
plt.rcParams.update({
    'font.family': 'sans-serif',
    'axes.facecolor': '#0d1117',
    'figure.facecolor': '#0d1117',
})

OUT = 'docs/07-testing/'

# ===== Corridor Data (for comparison) =====
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

# ===== Grass Data =====
grass_data = [
    (3, 27.113, 3.775, -9.388),
    (15, 32.474, -16.117, -19.379),
    (18, 31.262, -20.322, -15.293),
    (21, 29.526, -24.382, -10.517),
    (24, 27.807, -28.683, -5.259),
    (27, 26.665, -31.183, -1.866),
]

# ===== Slope Data =====
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


def compute_metrics(data):
    t = np.array([d[0] for d in data])
    x = np.array([d[1] for d in data])
    y = np.array([d[2] for d in data])
    z = np.array([d[3] for d in data])
    x0, y0, z0 = x[0], y[0], z[0]
    drift = np.sqrt((x - x0)**2 + (y - y0)**2 + (z - z0)**2)
    cum_dist = np.zeros_like(t, dtype=float)
    for i in range(1, len(t)):
        cum_dist[i] = cum_dist[i-1] + np.sqrt(
            (x[i]-x[i-1])**2 + (y[i]-y[i-1])**2 + (z[i]-z[i-1])**2)
    speed = np.zeros_like(t, dtype=float)
    for i in range(1, len(t)):
        dt = t[i] - t[i-1]
        ds = np.sqrt((x[i]-x[i-1])**2 + (y[i]-y[i-1])**2)
        speed[i] = ds / dt if dt > 0 else 0
    return t, x, y, z, drift, cum_dist, speed


ct, cx, cy, cz, c_drift, c_cum, c_speed = compute_metrics(corridor_data)
gt, gx, gy, gz, g_drift, g_cum, g_speed = compute_metrics(grass_data)
st, sx, sy, sz, s_drift, s_cum, s_speed = compute_metrics(slope_data)


# ======= Figure: Grass 2D Trajectory =======
fig, ax = plt.subplots(figsize=(10, 8))
points = np.array([gx, gy]).T.reshape(-1, 1, 2)
segments = np.concatenate([points[:-1], points[1:]], axis=1)
norm = plt.Normalize(gt.min(), gt.max())
lc = LineCollection(segments, cmap='viridis', norm=norm, linewidth=3, alpha=0.9)
lc.set_array(gt[:-1])
ax.add_collection(lc)
scatter = ax.scatter(gx, gy, c=gt, cmap='viridis', s=120, zorder=5,
                     edgecolors='white', linewidths=0.8)
ax.scatter(gx[0], gy[0], c='#00FF88', s=300, marker='*', zorder=10, label='Start')
ax.scatter(gx[-1], gy[-1], c='#FF4444', s=250, marker='X', zorder=10, label='End')

ax.annotate('START\n(27.1, 3.8)', (gx[0], gy[0]), fontsize=11, fontweight='bold',
            color='#00FF88', xytext=(15, 10), textcoords='offset points')
ax.annotate(f'END\n(26.7, -31.2)', (gx[-1], gy[-1]), fontsize=11, fontweight='bold',
            color='#FF4444', xytext=(15, 10), textcoords='offset points')

# Direction arrows
for i in range(1, len(gx)):
    dx, dy = gx[i]-gx[i-1], gy[i]-gy[i-1]
    nd = np.sqrt(dx**2 + dy**2)
    if nd > 0:
        ax.annotate('', xy=(gx[i], gy[i]),
                     xytext=(gx[i]-dx/nd*2, gy[i]-dy/nd*2),
                     arrowprops=dict(arrowstyle='->', color='white', lw=1.5, alpha=0.4))

cbar = plt.colorbar(scatter, ax=ax, shrink=0.8)
cbar.set_label('Time (seconds)', fontsize=12, color='white')
ax.set_xlabel('X Position (m)', fontsize=13)
ax.set_ylabel('Y Position (m)', fontsize=13)
ax.set_title('Point-LIO SLAM Trajectory — Leg-KILO Grass (Outdoor Rough Terrain)',
             fontsize=15, fontweight='bold', color='#0096D6', pad=15)
ax.set_aspect('equal')
ax.grid(True, alpha=0.15, color='white')
ax.legend(loc='lower left', fontsize=11, framealpha=0.3)
plt.tight_layout()
fig.savefig(OUT + 'fig_grass_trajectory_2d.png', dpi=200, bbox_inches='tight')
print('Saved: fig_grass_trajectory_2d.png')
plt.close(fig)


# ======= Figure: Grass Analysis Panels =======
fig = plt.figure(figsize=(16, 10))
gs = gridspec.GridSpec(2, 3, hspace=0.35, wspace=0.35)

# (a) XYZ vs Time
ax = fig.add_subplot(gs[0, 0])
ax.plot(gt, gx, '-o', color='#FF6B6B', markersize=6, lw=2, label='X')
ax.plot(gt, gy, '-s', color='#4ECDC4', markersize=6, lw=2, label='Y')
ax.plot(gt, gz, '-^', color='#FFE66D', markersize=6, lw=2, label='Z')
ax.axhline(0, color='white', lw=0.3, alpha=0.3)
ax.set_xlabel('Time (s)')
ax.set_ylabel('Position (m)')
ax.set_title('(a) Position Components', fontsize=12, fontweight='bold', color='#0096D6')
ax.legend(fontsize=9)
ax.grid(True, alpha=0.15)

# (b) Displacement from start
ax = fig.add_subplot(gs[0, 1])
ax.fill_between(gt, g_drift, alpha=0.25, color='#0096D6')
ax.plot(gt, g_drift, '-o', color='#0096D6', markersize=6, lw=2)
ax.axhline(g_drift.max(), color='#FF4444', lw=1, ls='--', alpha=0.6)
ax.annotate(f'Max: {g_drift.max():.1f}m', (gt[np.argmax(g_drift)], g_drift.max()),
            fontsize=10, color='#FF4444', fontweight='bold',
            xytext=(0, 10), textcoords='offset points')
ax.set_xlabel('Time (s)')
ax.set_ylabel('Distance from Start (m)')
ax.set_title('(b) Displacement from Start', fontsize=12, fontweight='bold', color='#0096D6')
ax.grid(True, alpha=0.15)

# (c) Z-axis drift (CRITICAL for outdoor)
ax = fig.add_subplot(gs[0, 2])
colors = ['#00C853' if abs(zi) < 2 else '#FFAA00' if abs(zi) < 10 else '#FF4444' for zi in gz]
ax.bar(gt, gz, width=2, color=colors, alpha=0.85)
ax.axhline(0, color='white', lw=0.8)
ax.axhspan(-2.0, 2.0, alpha=0.08, color='green', label='±2m (corridor spec)')
ax.set_xlabel('Time (s)')
ax.set_ylabel('Z Position (m)')
ax.set_title('(c) Z-axis Drift — Grass Terrain', fontsize=12, fontweight='bold', color='#FF4444')
ax.legend(fontsize=8)
ax.grid(True, alpha=0.15)

# (d) Corridor vs Grass Z comparison
ax = fig.add_subplot(gs[1, 0])
ax.plot(ct, cz, '-o', color='#4ECDC4', markersize=3, lw=1.5, alpha=0.8, label='Corridor')
ax.plot(gt, gz, '-s', color='#FF6B6B', markersize=6, lw=2, label='Grass')
ax.axhspan(-2.0, 2.0, alpha=0.08, color='green')
ax.set_xlabel('Time (s)')
ax.set_ylabel('Z Position (m)')
ax.set_title('(d) Z Drift Comparison', fontsize=12, fontweight='bold', color='#0096D6')
ax.legend(fontsize=9)
ax.grid(True, alpha=0.15)

# (e) Speed
ax = fig.add_subplot(gs[1, 1])
ax.plot(gt, g_speed, '-o', color='#00BCD4', markersize=6, lw=2)
ax.fill_between(gt, g_speed, alpha=0.2, color='#00BCD4')
if np.any(g_speed > 0):
    avg_spd = np.mean(g_speed[g_speed > 0])
    ax.axhline(avg_spd, color='#FFAA00', lw=1.5, ls='--', label=f'Avg: {avg_spd:.2f} m/s')
ax.set_xlabel('Time (s)')
ax.set_ylabel('Speed (m/s)')
ax.set_title('(e) Estimated Ground Speed', fontsize=12, fontweight='bold', color='#0096D6')
ax.legend(fontsize=9)
ax.grid(True, alpha=0.15)

# (f) Summary
ax = fig.add_subplot(gs[1, 2])
ax.axis('off')
grass_cum = g_cum[-1] if len(g_cum) > 0 else 0
grass_avg = np.mean(g_speed[g_speed > 0]) if np.any(g_speed > 0) else 0
stats = [
    ('Algorithm', 'Point-LIO (iVox)'),
    ('Dataset', 'Leg-KILO Grass'),
    ('Robot', 'Unitree Go1'),
    ('LiDAR', 'VLP-16 (16-line)'),
    ('Duration', f'{gt[-1]:.0f}s of 91.6s bag'),
    ('Total Dist', f'{grass_cum:.0f}m'),
    ('Max Disp.', f'{g_drift.max():.1f}m'),
    ('Z Peak', f'{gz.min():.1f}m'),
    ('Avg Speed', f'{grass_avg:.2f} m/s'),
    ('Crashes', '0'),
    ('Result', 'PASS'),
]
for i, (k, v) in enumerate(stats):
    c = '#00C853' if k == 'Result' else '#FF4444' if 'Z Peak' in k else 'white'
    w = 'bold' if k in ('Result', 'Z Peak') else 'normal'
    ax.text(0.05, 0.95 - i*0.082, f'{k}:', fontsize=11, color='#888888',
            transform=ax.transAxes, fontfamily='monospace')
    ax.text(0.55, 0.95 - i*0.082, v, fontsize=11, color=c, fontweight=w,
            transform=ax.transAxes, fontfamily='monospace')
ax.set_title('(f) Grass Test Summary', fontsize=12, fontweight='bold', color='#0096D6')

fig.savefig(OUT + 'fig_grass_analysis_panels.png', dpi=200, bbox_inches='tight')
print('Saved: fig_grass_analysis_panels.png')
plt.close(fig)


# ======= Figure: Cross-Scenario Comparison Dashboard =======
fig = plt.figure(figsize=(16, 8))
gs = gridspec.GridSpec(2, 4, hspace=0.4, wspace=0.35)

# (a) Corridor trajectory
ax = fig.add_subplot(gs[0, 0:2])
points = np.array([cx, cy]).T.reshape(-1, 1, 2)
segments = np.concatenate([points[:-1], points[1:]], axis=1)
norm = plt.Normalize(ct.min(), ct.max())
lc = LineCollection(segments, cmap='plasma', norm=norm, linewidth=2, alpha=0.9)
lc.set_array(ct[:-1])
ax.add_collection(lc)
ax.scatter(cx, cy, c=ct, cmap='plasma', s=30, zorder=5, edgecolors='white', linewidths=0.3)
ax.scatter(cx[0], cy[0], c='#00FF88', s=150, marker='*', zorder=10)
ax.scatter(cx[-1], cy[-1], c='#FF4444', s=120, marker='X', zorder=10)
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_title('Corridor (Indoor, 295s)', fontsize=12, fontweight='bold', color='#4ECDC4')
ax.set_aspect('equal')
ax.grid(True, alpha=0.15)

# (b) Grass trajectory
ax = fig.add_subplot(gs[0, 2:4])
points = np.array([gx, gy]).T.reshape(-1, 1, 2)
segments = np.concatenate([points[:-1], points[1:]], axis=1)
norm = plt.Normalize(gt.min(), gt.max())
lc = LineCollection(segments, cmap='viridis', norm=norm, linewidth=2.5, alpha=0.9)
lc.set_array(gt[:-1])
ax.add_collection(lc)
ax.scatter(gx, gy, c=gt, cmap='viridis', s=60, zorder=5, edgecolors='white', linewidths=0.3)
ax.scatter(gx[0], gy[0], c='#00FF88', s=150, marker='*', zorder=10)
ax.scatter(gx[-1], gy[-1], c='#FF4444', s=120, marker='X', zorder=10)
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_title('Grass (Outdoor, 27s)', fontsize=12, fontweight='bold', color='#FFE66D')
ax.set_aspect('equal')
ax.grid(True, alpha=0.15)

# (c) Z drift comparison
ax = fig.add_subplot(gs[1, 0:2])
ax.plot(ct, cz, '-', color='#4ECDC4', lw=2, alpha=0.8, label='Corridor (Z peak: -0.88m)')
ax.plot(gt, gz, '-', color='#FF6B6B', lw=2.5, label=f'Grass (Z peak: {gz.min():.1f}m)')
ax.fill_between(ct, cz, alpha=0.1, color='#4ECDC4')
ax.fill_between(gt, gz, alpha=0.15, color='#FF6B6B')
ax.axhspan(-2.0, 2.0, alpha=0.05, color='green', label='±2m spec')
ax.set_xlabel('Time (s)')
ax.set_ylabel('Z Position (m)')
ax.set_title('Z-axis Drift: Corridor vs Grass', fontsize=12, fontweight='bold', color='#0096D6')
ax.legend(fontsize=9)
ax.grid(True, alpha=0.15)

# (d) Comparison metrics bar chart
ax = fig.add_subplot(gs[1, 2:4])
metrics = ['Duration\n(s)', 'Total Dist\n(m)', 'Max Disp\n(m)', 'Z Peak\n(|m|)', 'Crashes']
corridor_vals = [295, c_cum[-1], c_drift.max(), abs(cz.min()), 0]
grass_vals = [27, g_cum[-1], g_drift.max(), abs(gz.min()), 0]

x_pos = np.arange(len(metrics))
width = 0.35
bars1 = ax.bar(x_pos - width/2, corridor_vals, width, label='Corridor',
               color='#4ECDC4', alpha=0.8)
bars2 = ax.bar(x_pos + width/2, grass_vals, width, label='Grass',
               color='#FF6B6B', alpha=0.8)

# Value labels
for bar in bars1:
    ax.text(bar.get_x() + bar.get_width()/2., bar.get_height() + 0.5,
            f'{bar.get_height():.1f}', ha='center', va='bottom', fontsize=8, color='#4ECDC4')
for bar in bars2:
    ax.text(bar.get_x() + bar.get_width()/2., bar.get_height() + 0.5,
            f'{bar.get_height():.1f}', ha='center', va='bottom', fontsize=8, color='#FF6B6B')

ax.set_xticks(x_pos)
ax.set_xticklabels(metrics, fontsize=9)
ax.set_title('Metric Comparison', fontsize=12, fontweight='bold', color='#0096D6')
ax.legend(fontsize=9)
ax.grid(True, alpha=0.15, axis='y')

fig.suptitle('Point-LIO SLAM: Corridor vs Outdoor Grass Terrain',
             fontsize=16, fontweight='bold', color='#0096D6', y=0.98)
fig.savefig(OUT + 'fig_comparison_dashboard.png', dpi=200, bbox_inches='tight')
print('Saved: fig_comparison_dashboard.png')
plt.close(fig)

# ======= Figure: Slope 2D Trajectory =======
fig, ax = plt.subplots(figsize=(10, 10))
points = np.array([sx, sy]).T.reshape(-1, 1, 2)
segments = np.concatenate([points[:-1], points[1:]], axis=1)
norm = plt.Normalize(st.min(), st.max())
lc = LineCollection(segments, cmap='magma', norm=norm, linewidth=3, alpha=0.9)
lc.set_array(st[:-1])
ax.add_collection(lc)
scatter = ax.scatter(sx, sy, c=st, cmap='magma', s=100, zorder=5,
                     edgecolors='white', linewidths=0.8)
ax.scatter(sx[0], sy[0], c='#00FF88', s=300, marker='*', zorder=10, label='Start')
ax.scatter(sx[-1], sy[-1], c='#FF4444', s=250, marker='X', zorder=10, label='End')
idx_max = np.argmax(s_drift)
ax.scatter(sx[idx_max], sy[idx_max], c='#FF8800', s=200, marker='D', zorder=10,
           label=f'Farthest ({s_drift[idx_max]:.0f}m)')

ax.annotate(f'START\n({sx[0]:.0f}, {sy[0]:.0f})', (sx[0], sy[0]),
            fontsize=11, fontweight='bold', color='#00FF88',
            xytext=(15, -20), textcoords='offset points')
ax.annotate(f'END\n({sx[-1]:.0f}, {sy[-1]:.0f})', (sx[-1], sy[-1]),
            fontsize=11, fontweight='bold', color='#FF4444',
            xytext=(15, 10), textcoords='offset points')
ax.annotate(f'FARTHEST\n{s_drift[idx_max]:.0f}m', (sx[idx_max], sy[idx_max]),
            fontsize=10, color='#FF8800',
            xytext=(-60, -25), textcoords='offset points',
            arrowprops=dict(arrowstyle='->', color='#FF8800', lw=1.5))

cbar = plt.colorbar(scatter, ax=ax, shrink=0.8)
cbar.set_label('Time (seconds)', fontsize=12, color='white')
ax.set_xlabel('X Position (m)', fontsize=13)
ax.set_ylabel('Y Position (m)', fontsize=13)
ax.set_title('Point-LIO SLAM Trajectory — Leg-KILO Slope (Outdoor Hill Terrain)',
             fontsize=15, fontweight='bold', color='#0096D6', pad=15)
ax.set_aspect('equal')
ax.grid(True, alpha=0.15, color='white')
ax.legend(loc='lower left', fontsize=11, framealpha=0.3)
plt.tight_layout()
fig.savefig(OUT + 'fig_slope_trajectory_2d.png', dpi=200, bbox_inches='tight')
print('Saved: fig_slope_trajectory_2d.png')
plt.close(fig)


# ======= Figure: Slope Analysis Panels =======
fig = plt.figure(figsize=(16, 10))
gs = gridspec.GridSpec(2, 3, hspace=0.35, wspace=0.35)

# (a) XYZ vs Time
ax = fig.add_subplot(gs[0, 0])
ax.plot(st, sx, '-o', color='#FF6B6B', markersize=4, lw=1.5, label='X')
ax.plot(st, sy, '-s', color='#4ECDC4', markersize=4, lw=1.5, label='Y')
ax.plot(st, sz, '-^', color='#FFE66D', markersize=4, lw=1.5, label='Z')
ax.axhline(0, color='white', lw=0.3, alpha=0.3)
ax.set_xlabel('Time (s)')
ax.set_ylabel('Position (m)')
ax.set_title('(a) Position Components', fontsize=12, fontweight='bold', color='#0096D6')
ax.legend(fontsize=9)
ax.grid(True, alpha=0.15)

# (b) Displacement from start
ax = fig.add_subplot(gs[0, 1])
ax.fill_between(st, s_drift, alpha=0.25, color='#E040FB')
ax.plot(st, s_drift, '-o', color='#E040FB', markersize=4, lw=2)
ax.axhline(s_drift.max(), color='#FF4444', lw=1, ls='--', alpha=0.6)
ax.annotate(f'Max: {s_drift.max():.1f}m', (st[np.argmax(s_drift)], s_drift.max()),
            fontsize=10, color='#FF4444', fontweight='bold',
            xytext=(-60, 10), textcoords='offset points')
ax.set_xlabel('Time (s)')
ax.set_ylabel('Distance from Start (m)')
ax.set_title('(b) Displacement from Start', fontsize=12, fontweight='bold', color='#0096D6')
ax.grid(True, alpha=0.15)

# (c) Z-axis drift
ax = fig.add_subplot(gs[0, 2])
colors = ['#00C853' if abs(zi) < 2 else '#FFAA00' if abs(zi) < 5 else '#FF4444' for zi in sz]
ax.bar(st, sz, width=2, color=colors, alpha=0.85)
ax.axhline(0, color='white', lw=0.8)
ax.axhspan(-2.0, 2.0, alpha=0.08, color='green', label='±2m (corridor spec)')
ax.set_xlabel('Time (s)')
ax.set_ylabel('Z Position (m)')
ax.set_title('(c) Z-axis Drift — Slope Terrain', fontsize=12, fontweight='bold', color='#0096D6')
ax.legend(fontsize=8)
ax.grid(True, alpha=0.15)

# (d) Cumulative distance
ax = fig.add_subplot(gs[1, 0])
ax.plot(st, s_cum, '-o', color='#E040FB', markersize=4, lw=2)
ax.fill_between(st, s_cum, alpha=0.2, color='#E040FB')
ax.annotate(f'Total: {s_cum[-1]:.0f}m', (st[-1], s_cum[-1]),
            fontsize=10, color='#E040FB', fontweight='bold',
            xytext=(-80, 10), textcoords='offset points')
ax.set_xlabel('Time (s)')
ax.set_ylabel('Cumulative Distance (m)')
ax.set_title('(d) Cumulative Travel Distance', fontsize=12, fontweight='bold', color='#0096D6')
ax.grid(True, alpha=0.15)

# (e) Speed
ax = fig.add_subplot(gs[1, 1])
ax.plot(st, s_speed, '-o', color='#00BCD4', markersize=4, lw=1.5)
ax.fill_between(st, s_speed, alpha=0.2, color='#00BCD4')
if np.any(s_speed > 0):
    avg_spd = np.mean(s_speed[s_speed > 0])
    ax.axhline(avg_spd, color='#FFAA00', lw=1.5, ls='--', label=f'Avg: {avg_spd:.2f} m/s')
ax.set_xlabel('Time (s)')
ax.set_ylabel('Speed (m/s)')
ax.set_title('(e) Estimated Ground Speed', fontsize=12, fontweight='bold', color='#0096D6')
ax.legend(fontsize=9)
ax.grid(True, alpha=0.15)

# (f) Summary
ax = fig.add_subplot(gs[1, 2])
ax.axis('off')
slope_avg = np.mean(s_speed[s_speed > 0]) if np.any(s_speed > 0) else 0
stats = [
    ('Algorithm', 'Point-LIO (iVox)'),
    ('Dataset', 'Leg-KILO Slope'),
    ('Robot', 'Unitree Go1'),
    ('LiDAR', 'VLP-16 (16-line)'),
    ('Duration', f'{st[-1]:.0f}s of 312s bag'),
    ('Total Dist', f'{s_cum[-1]:.0f}m'),
    ('Max Disp.', f'{s_drift.max():.1f}m'),
    ('Z Peak', f'{sz.min():.1f}m'),
    ('Z Return', f'{sz[-1]:.1f}m'),
    ('Avg Speed', f'{slope_avg:.2f} m/s'),
    ('Crashes', '0'),
    ('Result', 'PASS'),
]
for i, (k, v) in enumerate(stats):
    c = '#00C853' if k == 'Result' else '#FFAA00' if 'Z' in k else 'white'
    w = 'bold' if k in ('Result',) else 'normal'
    ax.text(0.05, 0.95 - i*0.077, f'{k}:', fontsize=11, color='#888888',
            transform=ax.transAxes, fontfamily='monospace')
    ax.text(0.55, 0.95 - i*0.077, v, fontsize=11, color=c, fontweight=w,
            transform=ax.transAxes, fontfamily='monospace')
ax.set_title('(f) Slope Test Summary', fontsize=12, fontweight='bold', color='#0096D6')

fig.savefig(OUT + 'fig_slope_analysis_panels.png', dpi=200, bbox_inches='tight')
print('Saved: fig_slope_analysis_panels.png')
plt.close(fig)


# ======= Figure: 3-Scenario Comprehensive Comparison =======
fig = plt.figure(figsize=(18, 12))
gs = gridspec.GridSpec(3, 3, hspace=0.4, wspace=0.35)

# Row 1: Trajectories
# (a) Corridor
ax = fig.add_subplot(gs[0, 0])
points = np.array([cx, cy]).T.reshape(-1, 1, 2)
segments = np.concatenate([points[:-1], points[1:]], axis=1)
lc = LineCollection(segments, cmap='plasma', norm=plt.Normalize(ct.min(), ct.max()),
                    linewidth=2, alpha=0.9)
lc.set_array(ct[:-1])
ax.add_collection(lc)
ax.scatter(cx, cy, c=ct, cmap='plasma', s=20, zorder=5, edgecolors='white', linewidths=0.2)
ax.scatter(cx[0], cy[0], c='#00FF88', s=100, marker='*', zorder=10)
ax.scatter(cx[-1], cy[-1], c='#FF4444', s=80, marker='X', zorder=10)
ax.set_title('Corridor (Indoor, 295s)', fontsize=11, fontweight='bold', color='#4ECDC4')
ax.set_aspect('equal')
ax.grid(True, alpha=0.15)
ax.set_xlabel('X (m)', fontsize=9)
ax.set_ylabel('Y (m)', fontsize=9)

# (b) Grass
ax = fig.add_subplot(gs[0, 1])
points = np.array([gx, gy]).T.reshape(-1, 1, 2)
segments = np.concatenate([points[:-1], points[1:]], axis=1)
lc = LineCollection(segments, cmap='viridis', norm=plt.Normalize(gt.min(), gt.max()),
                    linewidth=2.5, alpha=0.9)
lc.set_array(gt[:-1])
ax.add_collection(lc)
ax.scatter(gx, gy, c=gt, cmap='viridis', s=40, zorder=5, edgecolors='white', linewidths=0.2)
ax.scatter(gx[0], gy[0], c='#00FF88', s=100, marker='*', zorder=10)
ax.scatter(gx[-1], gy[-1], c='#FF4444', s=80, marker='X', zorder=10)
ax.set_title('Grass (Outdoor, 27s)', fontsize=11, fontweight='bold', color='#FFE66D')
ax.set_aspect('equal')
ax.grid(True, alpha=0.15)
ax.set_xlabel('X (m)', fontsize=9)
ax.set_ylabel('Y (m)', fontsize=9)

# (c) Slope
ax = fig.add_subplot(gs[0, 2])
points = np.array([sx, sy]).T.reshape(-1, 1, 2)
segments = np.concatenate([points[:-1], points[1:]], axis=1)
lc = LineCollection(segments, cmap='magma', norm=plt.Normalize(st.min(), st.max()),
                    linewidth=2, alpha=0.9)
lc.set_array(st[:-1])
ax.add_collection(lc)
ax.scatter(sx, sy, c=st, cmap='magma', s=25, zorder=5, edgecolors='white', linewidths=0.2)
ax.scatter(sx[0], sy[0], c='#00FF88', s=100, marker='*', zorder=10)
ax.scatter(sx[-1], sy[-1], c='#FF4444', s=80, marker='X', zorder=10)
ax.set_title('Slope (Outdoor, 162s)', fontsize=11, fontweight='bold', color='#FF6B6B')
ax.set_aspect('equal')
ax.grid(True, alpha=0.15)
ax.set_xlabel('X (m)', fontsize=9)
ax.set_ylabel('Y (m)', fontsize=9)

# Row 2: Z drift comparison + Displacement + Speed
# (d) Z drift all 3
ax = fig.add_subplot(gs[1, 0:2])
ax.plot(ct, cz, '-', color='#4ECDC4', lw=2, alpha=0.8, label=f'Corridor (peak: {cz.min():.1f}m)')
ax.plot(gt, gz, '-', color='#FFE66D', lw=2, label=f'Grass (peak: {gz.min():.1f}m)')
ax.plot(st, sz, '-', color='#FF6B6B', lw=2, label=f'Slope (peak: {sz.min():.1f}m)')
ax.fill_between(ct, cz, alpha=0.08, color='#4ECDC4')
ax.fill_between(gt, gz, alpha=0.1, color='#FFE66D')
ax.fill_between(st, sz, alpha=0.1, color='#FF6B6B')
ax.axhspan(-2.0, 2.0, alpha=0.04, color='green')
ax.set_xlabel('Time (s)')
ax.set_ylabel('Z Position (m)')
ax.set_title('Z-axis Drift: All Three Scenarios', fontsize=12, fontweight='bold', color='#0096D6')
ax.legend(fontsize=9, loc='lower left')
ax.grid(True, alpha=0.15)

# (e) Displacement all 3
ax = fig.add_subplot(gs[1, 2])
ax.plot(ct, c_drift, '-', color='#4ECDC4', lw=2, label='Corridor')
ax.plot(gt, g_drift, '-', color='#FFE66D', lw=2, label='Grass')
ax.plot(st, s_drift, '-', color='#FF6B6B', lw=2, label='Slope')
ax.set_xlabel('Time (s)')
ax.set_ylabel('Displacement (m)')
ax.set_title('Displacement from Start', fontsize=12, fontweight='bold', color='#0096D6')
ax.legend(fontsize=9)
ax.grid(True, alpha=0.15)

# Row 3: Bar chart comparison + Summary table
# (f) Bar chart
ax = fig.add_subplot(gs[2, 0:2])
metrics = ['Duration\n(s)', 'Total Dist\n(m)', 'Max Disp\n(m)', 'Z Peak\n(|m|)',
           'Avg Speed\n(m/s)', 'Crashes']
c_avg = np.mean(c_speed[c_speed > 0]) if np.any(c_speed > 0) else 0
g_avg = np.mean(g_speed[g_speed > 0]) if np.any(g_speed > 0) else 0
s_avg = np.mean(s_speed[s_speed > 0]) if np.any(s_speed > 0) else 0
corridor_v = [295, c_cum[-1], c_drift.max(), abs(cz.min()), c_avg, 0]
grass_v = [27, g_cum[-1], g_drift.max(), abs(gz.min()), g_avg, 0]
slope_v = [162, s_cum[-1], s_drift.max(), abs(sz.min()), s_avg, 0]

x_pos = np.arange(len(metrics))
width = 0.25
ax.bar(x_pos - width, corridor_v, width, label='Corridor', color='#4ECDC4', alpha=0.8)
ax.bar(x_pos, grass_v, width, label='Grass', color='#FFE66D', alpha=0.8)
ax.bar(x_pos + width, slope_v, width, label='Slope', color='#FF6B6B', alpha=0.8)
ax.set_xticks(x_pos)
ax.set_xticklabels(metrics, fontsize=9)
ax.set_title('Quantitative Comparison Across All Scenarios',
             fontsize=12, fontweight='bold', color='#0096D6')
ax.legend(fontsize=9)
ax.grid(True, alpha=0.15, axis='y')

# (g) Summary text
ax = fig.add_subplot(gs[2, 2])
ax.axis('off')
summary = [
    ('', 'Corridor', 'Grass', 'Slope'),
    ('Result', 'PASS', 'PASS', 'PASS'),
    ('Crash', '0', '0', '0'),
    ('Duration', '295s', '27s', '162s'),
    ('Distance', f'{c_cum[-1]:.0f}m', f'{g_cum[-1]:.0f}m', f'{s_cum[-1]:.0f}m'),
    ('Max Disp', f'{c_drift.max():.0f}m', f'{g_drift.max():.0f}m', f'{s_drift.max():.0f}m'),
    ('|Z| Peak', f'{abs(cz.min()):.1f}m', f'{abs(gz.min()):.1f}m', f'{abs(sz.min()):.1f}m'),
    ('Avg Spd', f'{c_avg:.1f}m/s', f'{g_avg:.1f}m/s', f'{s_avg:.1f}m/s'),
]
for i, row in enumerate(summary):
    for j, val in enumerate(row):
        x = 0.02 + j * 0.25
        y = 0.92 - i * 0.11
        if i == 0:
            c, w = '#0096D6', 'bold'
        elif i == 1:
            c, w = '#00C853', 'bold'
        else:
            c, w = 'white', 'normal'
        ax.text(x, y, val, fontsize=10, color=c, fontweight=w,
                transform=ax.transAxes, fontfamily='monospace')
ax.set_title('Summary', fontsize=12, fontweight='bold', color='#0096D6')

fig.suptitle('Point-LIO SLAM Robustness: 3-Scenario Comparison (Vibration + Terrain)',
             fontsize=16, fontweight='bold', color='#0096D6', y=0.99)
fig.savefig(OUT + 'fig_3scenario_comparison.png', dpi=200, bbox_inches='tight')
print('Saved: fig_3scenario_comparison.png')
plt.close(fig)

print('All outdoor figures generated.')
