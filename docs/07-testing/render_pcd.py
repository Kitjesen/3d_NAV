"""
Render PCD point cloud maps with plotly — interactive 3D visualization.
Opens in browser, can rotate/zoom, export as PNG.

Usage:
  python docs/07-testing/render_pcd.py                    # render all available PCDs
  python docs/07-testing/render_pcd.py map_fastlio2_corridor.pcd
"""
import os
import sys

import numpy as np


def load_pcd(path):
    """Load PCD file (ASCII or binary), return (N,3) xyz array and optional intensity."""
    import struct

    with open(path, 'rb') as f:
        header_lines = []
        while True:
            line = f.readline().decode('ascii', errors='replace').strip()
            header_lines.append(line)
            if line.startswith('DATA'):
                break

        # Parse header
        fields = []
        n_points = 0
        sizes = []
        data_format = 'ascii'
        for line in header_lines:
            if line.startswith('FIELDS'):
                fields = line.split()[1:]
            elif line.startswith('POINTS'):
                n_points = int(line.split()[1])
            elif line.startswith('SIZE'):
                sizes = [int(s) for s in line.split()[1:]]
            elif line.startswith('DATA'):
                data_format = line.split()[1]

        has_intensity = 'intensity' in fields
        x_idx = fields.index('x') if 'x' in fields else 0
        y_idx = fields.index('y') if 'y' in fields else 1
        z_idx = fields.index('z') if 'z' in fields else 2
        i_idx = fields.index('intensity') if has_intensity else -1
        point_step = sum(sizes)

        if data_format == 'binary':
            # Build numpy dtype from fields/sizes
            dt = np.dtype([(name, '<f4') for name, sz in zip(fields, sizes)])
            data = np.frombuffer(f.read(n_points * point_step), dtype=dt, count=n_points)

            xyz = np.column_stack([data['x'], data['y'], data['z']])
            intensity_arr = data['intensity'].copy() if has_intensity else None

            # Filter NaN/Inf and outliers
            valid = (np.abs(xyz) < 500).all(axis=1) & np.isfinite(xyz).all(axis=1)
            xyz = xyz[valid]
            if intensity_arr is not None:
                intensity_arr = intensity_arr[valid]

            print(f"Loaded {len(xyz)} points from {os.path.basename(path)} (binary, {n_points} total)")
            return xyz, intensity_arr

        else:  # ASCII
            rest = f.read().decode('utf-8', errors='replace')
            lines = rest.strip().split('\n')

            xyz_list = []
            intensity_list = []
            for line in lines[:n_points]:
                parts = line.strip().split()
                if len(parts) >= 3:
                    x, y, z = float(parts[x_idx]), float(parts[y_idx]), float(parts[z_idx])
                    if any(abs(v) > 500 for v in (x, y, z)):
                        continue
                    if any(v != v for v in (x, y, z)):
                        continue
                    xyz_list.append([x, y, z])
                    if has_intensity and len(parts) > i_idx:
                        intensity_list.append(float(parts[i_idx]))

            xyz = np.array(xyz_list)
            intensity = np.array(intensity_list) if intensity_list else None
            print(f"Loaded {len(xyz)} points from {os.path.basename(path)} (ascii)")
            return xyz, intensity


def render_single(path, title=None):
    """Render a single PCD file with plotly."""
    import plotly.graph_objects as go

    xyz, intensity = load_pcd(path)
    if len(xyz) == 0:
        print("No points to render!")
        return

    # Downsample if too many points (plotly gets slow > 500k)
    max_pts = 300000
    if len(xyz) > max_pts:
        idx = np.random.choice(len(xyz), max_pts, replace=False)
        xyz = xyz[idx]
        if intensity is not None:
            intensity = intensity[idx]
        print(f"Downsampled to {max_pts} points for rendering")

    # Color by height (Z)
    z_vals = xyz[:, 2]
    z_min, z_max = np.percentile(z_vals, [2, 98])

    if title is None:
        title = os.path.basename(path).replace('.pcd', '')

    fig = go.Figure(data=[go.Scatter3d(
        x=xyz[:, 0], y=xyz[:, 1], z=xyz[:, 2],
        mode='markers',
        marker=dict(
            size=1.2,
            color=z_vals,
            colorscale='Viridis',
            cmin=z_min,
            cmax=z_max,
            colorbar=dict(title='Z (m)', thickness=15),
            opacity=0.8,
        ),
        hoverinfo='skip',
    )])

    fig.update_layout(
        title=dict(text=title, font=dict(size=18)),
        scene=dict(
            xaxis_title='X (m)',
            yaxis_title='Y (m)',
            zaxis_title='Z (m)',
            aspectmode='data',
            camera=dict(
                eye=dict(x=1.5, y=1.5, z=0.8),
            ),
        ),
        width=1200,
        height=800,
        margin=dict(l=0, r=0, t=40, b=0),
        paper_bgcolor='white',
    )

    out_html = path.replace('.pcd', '.html')
    fig.write_html(out_html)
    print(f"Saved: {out_html}")

    # Also save a static image if kaleido is available
    try:
        out_png = path.replace('.pcd', '_3d.png')
        fig.write_image(out_png, width=1600, height=1000, scale=2)
        print(f"Saved: {out_png}")
    except Exception as e:
        print(f"(PNG export skipped: {e})")

    # Open in browser
    import webbrowser
    webbrowser.open(f'file:///{os.path.abspath(out_html)}')
    print("Opened in browser")


def render_comparison(paths, titles):
    """Render multiple PCDs side by side for comparison."""
    import plotly.graph_objects as go
    from plotly.subplots import make_subplots

    fig = go.Figure()

    for i, (path, title) in enumerate(zip(paths, titles)):
        xyz, _ = load_pcd(path)
        if len(xyz) == 0:
            continue

        max_pts = 200000
        if len(xyz) > max_pts:
            idx = np.random.choice(len(xyz), max_pts, replace=False)
            xyz = xyz[idx]

        z_vals = xyz[:, 2]
        colorscale = 'Blues' if i == 0 else 'Oranges'

        fig.add_trace(go.Scatter3d(
            x=xyz[:, 0], y=xyz[:, 1], z=xyz[:, 2],
            mode='markers',
            name=title,
            marker=dict(
                size=1,
                color=z_vals,
                colorscale=colorscale,
                opacity=0.6,
            ),
            hoverinfo='skip',
        ))

    fig.update_layout(
        title=dict(text='Point Cloud Map Comparison — Corridor Dataset',
                   font=dict(size=18)),
        scene=dict(
            xaxis_title='X (m)',
            yaxis_title='Y (m)',
            zaxis_title='Z (m)',
            aspectmode='data',
        ),
        width=1400,
        height=900,
        margin=dict(l=0, r=0, t=40, b=0),
        paper_bgcolor='white',
    )

    out_html = os.path.join(os.path.dirname(paths[0]), 'map_comparison.html')
    fig.write_html(out_html)
    print(f"Saved: {out_html}")

    import webbrowser
    webbrowser.open(f'file:///{os.path.abspath(out_html)}')


def main():
    fig_dir = os.path.dirname(os.path.abspath(__file__))

    if len(sys.argv) > 1:
        # Render specific file(s)
        for path in sys.argv[1:]:
            if not os.path.isabs(path):
                path = os.path.join(fig_dir, path)
            render_single(path)
    else:
        # Auto-detect available PCDs
        pcds = []
        titles = []
        for name, title in [
            ('map_pointlio_corridor.pcd', 'Point-LIO'),
            ('map_fastlio2_corridor.pcd', 'Fast-LIO2'),
        ]:
            path = os.path.join(fig_dir, name)
            if os.path.exists(path):
                pcds.append(path)
                titles.append(title)

        if len(pcds) == 0:
            print("No PCD files found in docs/07-testing/")
            print("Run capture_slam_pcd.py first.")
            return
        elif len(pcds) == 1:
            render_single(pcds[0], titles[0])
        else:
            # Render each individually + comparison
            for p, t in zip(pcds, titles):
                render_single(p, t)
            render_comparison(pcds, titles)


if __name__ == '__main__':
    main()
