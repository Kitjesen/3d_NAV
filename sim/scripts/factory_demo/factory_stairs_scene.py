#!/usr/bin/env python3
"""Generate MuJoCo XML: 3-floor factory with staircases aligned to planned path.

Path analysis (from ele_planner global path):
  Stair 1: climb x=26.5鈫?1, y~8.0, z=0.7鈫?.1
  Stair 2: climb x=26.5鈫?0, y~6.75, z=3.9鈫?.5

Each staircase: 17 steps x 18cm rise, with bright tread overlays.
Top of each staircase connects to the upper floor edge.
"""

import math
import textwrap


def _indent(text: str, n: int = 4) -> str:
    prefix = " " * n
    return "\n".join(prefix + line if line.strip() else line
                     for line in text.splitlines())


def _stair_geoms(
    name_prefix: str,
    n_steps: int,
    x_floor_edge: float,
    y_center: float,
    base_z: float,
    step_h: float = 0.18,
    step_d: float = 0.30,
    step_w: float = 2.0,
) -> str:
    """Staircase: tallest step at x_floor_edge (connects to upper floor),
    shortest step at x_floor_edge + (n-1)*step_d (on lower floor).
    Each step has a horizontal tread slab + vertical riser slab (real zigzag
    profile), plus one diagonal stringer underneath."""
    lines = []
    tread_thick = 0.015   # half-size z for tread (~3cm total)
    riser_thick = 0.0125  # half-size x for riser (~2.5cm total)

    for i in range(n_steps):
        # i=0 = bottom (lowest, farthest from upper floor)
        # i=n-1 = top (highest, at floor edge)
        dist = (n_steps - 1 - i)
        cx = x_floor_edge + dist * step_d + step_d / 2.0

        # --- Tread: horizontal slab at top of this step ---
        tread_top_z = base_z + (i + 1) * step_h
        tread_cz = tread_top_z - tread_thick  # center of the 3cm slab
        lines.append(
            f'<geom name="{name_prefix}_t{i}" type="box" '
            f'size="{step_d / 2:.4f} {step_w / 2:.4f} {tread_thick:.4f}" '
            f'pos="{cx:.4f} {y_center:.4f} {tread_cz:.4f}" '
            f'rgba="0.92 0.90 0.85 1"/>'
        )

        # --- Riser: vertical slab at the front face of each step ---
        # The front face is the higher-x edge of this tread (the side
        # facing the lower floor / +x direction).
        riser_x = cx + step_d / 2.0 - riser_thick
        riser_h = step_h - tread_thick * 2  # gap between treads
        if riser_h < 0.02:
            riser_h = 0.02
        riser_cz = tread_top_z - tread_thick * 2 - riser_h / 2.0
        lines.append(
            f'<geom name="{name_prefix}_r{i}" type="box" '
            f'size="{riser_thick:.4f} {step_w / 2:.4f} {riser_h / 2:.4f}" '
            f'pos="{riser_x:.4f} {y_center:.4f} {riser_cz:.4f}" '
            f'rgba="0.75 0.73 0.70 1"/>'
        )

    # --- Diagonal stringer (underside of staircase) ---
    total_run = n_steps * step_d
    total_rise = n_steps * step_h
    stringer_len = math.sqrt(total_run ** 2 + total_rise ** 2)
    angle_deg = math.degrees(math.atan2(total_rise, total_run))
    # Stringer center: midpoint of the diagonal
    str_cx = x_floor_edge + total_run / 2.0
    str_cz = base_z + total_rise / 2.0 - 0.03  # offset slightly below treads
    stringer_depth = 0.06  # half-size z (12cm thick slab)
    lines.append(
        f'<geom name="{name_prefix}_str" type="box" '
        f'size="{stringer_len / 2:.4f} {step_w / 2 - 0.05:.4f} {stringer_depth:.4f}" '
        f'pos="{str_cx:.4f} {y_center:.4f} {str_cz:.4f}" '
        f'euler="0 {angle_deg:.2f} 0" rgba="0.55 0.53 0.50 1"/>'
    )

    # Landing at top (bridge to floor)
    top_z = base_z + n_steps * step_h
    lines.append(
        f'<geom name="{name_prefix}_land" type="box" '
        f'size="0.5 {step_w / 2 + 0.1:.4f} 0.025" '
        f'pos="{x_floor_edge - 0.3:.4f} {y_center:.4f} {top_z:.4f}" '
        f'rgba="0.88 0.86 0.82 1"/>'
    )

    return "\n".join(lines)


def _stair_railings(
    name_prefix: str,
    n_steps: int,
    x_floor_edge: float,
    y_center: float,
    base_z: float,
    step_h: float = 0.18,
    step_d: float = 0.30,
    step_w: float = 2.0,
    rail_h: float = 0.90,
) -> str:
    total_run = n_steps * step_d
    total_rise = n_steps * step_h
    half_len = math.sqrt(total_run ** 2 + total_rise ** 2) / 2.0
    angle_deg = math.degrees(math.atan2(total_rise, total_run))

    cx = x_floor_edge + total_run / 2.0
    cz = base_z + total_rise / 2.0 + rail_h / 2.0

    lines = []
    for side, y_off in [("L", y_center - step_w / 2.0),
                         ("R", y_center + step_w / 2.0)]:
        lines.append(
            f'<geom name="{name_prefix}_r{side}" type="box" '
            f'size="{half_len:.4f} 0.025 {rail_h / 2:.4f}" '
            f'pos="{cx:.4f} {y_off:.4f} {cz:.4f}" '
            f'euler="0 {angle_deg:.2f} 0" rgba="0.30 0.30 0.32 0.85"/>'
        )
        # Bottom post
        bx = x_floor_edge + total_run - step_d / 2
        bz = base_z + step_h / 2 + rail_h / 2
        lines.append(
            f'<geom name="{name_prefix}_p{side}b" type="box" '
            f'size="0.03 0.03 {rail_h / 2 + step_h:.4f}" '
            f'pos="{bx:.4f} {y_off:.4f} {bz:.4f}" rgba="0.30 0.30 0.32 0.9"/>'
        )
        # Top post
        tx = x_floor_edge + step_d / 2
        tz = base_z + total_rise + rail_h / 2
        lines.append(
            f'<geom name="{name_prefix}_p{side}t" type="box" '
            f'size="0.03 0.03 {rail_h / 2 + step_h:.4f}" '
            f'pos="{tx:.4f} {y_off:.4f} {tz:.4f}" rgba="0.30 0.30 0.32 0.9"/>'
        )
    return "\n".join(lines)


def _stairwell(
    name_prefix: str,
    x_floor_edge: float,
    y_center: float,
    base_z: float,
    total_run: float,
    step_w: float,
    n_steps: int = 17,
    step_h: float = 0.18,
    landing_depth: float = 1.0,
) -> str:
    """Stairwell enclosure: side walls, back wall, landings, ceiling.
    Open at x_floor_edge side (entry/exit to floors)."""
    lines = []
    total_rise = n_steps * step_h
    wall_h = total_rise + 1.0  # walls extend 1m above upper floor
    wt = 0.075  # wall half-thickness

    x_top = x_floor_edge
    x_bot = x_floor_edge + total_run
    x_back = x_bot + landing_depth
    sw_len = x_back - x_top
    sw_cx = (x_top + x_back) / 2.0
    wall_cz = base_z + wall_h / 2.0

    y_lo = y_center - step_w / 2.0
    y_hi = y_center + step_w / 2.0
    back_hw = step_w / 2.0 + wt * 2

    # Side walls (semi-transparent so stairs visible)
    wc = "0.72 0.70 0.68 0.35"
    lines.append(
        f'<geom name="{name_prefix}_wL" type="box" '
        f'size="{sw_len / 2:.3f} {wt:.3f} {wall_h / 2:.3f}" '
        f'pos="{sw_cx:.3f} {y_lo - wt:.3f} {wall_cz:.3f}" '
        f'rgba="{wc}"/>')
    lines.append(
        f'<geom name="{name_prefix}_wR" type="box" '
        f'size="{sw_len / 2:.3f} {wt:.3f} {wall_h / 2:.3f}" '
        f'pos="{sw_cx:.3f} {y_hi + wt:.3f} {wall_cz:.3f}" '
        f'rgba="{wc}"/>')

    # Back wall (semi-transparent)
    lines.append(
        f'<geom name="{name_prefix}_wB" type="box" '
        f'size="{wt:.3f} {back_hw:.3f} {wall_h / 2:.3f}" '
        f'pos="{x_back + wt:.3f} {y_center:.3f} {wall_cz:.3f}" '
        f'rgba="{wc}"/>')

    # Front mid-wall (slab between lower and upper doorways)
    door_h = 2.2
    lower_top = base_z + door_h
    mid_h = base_z + total_rise - lower_top
    if mid_h > 0.05:
        lines.append(
            f'<geom name="{name_prefix}_wFm" type="box" '
            f'size="{wt:.3f} {back_hw:.3f} {mid_h / 2:.3f}" '
            f'pos="{x_top - wt:.3f} {y_center:.3f} {lower_top + mid_h / 2:.3f}" '
            f'rgba="0.72 0.70 0.68 0.5"/>')

    # Bottom landing
    lines.append(
        f'<geom name="{name_prefix}_plB" type="box" '
        f'size="{landing_depth / 2:.3f} {step_w / 2 + 0.2:.3f} 0.025" '
        f'pos="{x_bot + landing_depth / 2:.3f} {y_center:.3f} {base_z:.3f}" '
        f'rgba="0.82 0.80 0.78 1"/>')

    # Top landing
    top_z = base_z + total_rise
    lines.append(
        f'<geom name="{name_prefix}_plT" type="box" '
        f'size="0.6 {step_w / 2 + 0.2:.3f} 0.025" '
        f'pos="{x_top - 0.4:.3f} {y_center:.3f} {top_z:.3f}" '
        f'rgba="0.82 0.80 0.78 1"/>')

    # Ceiling (semi-transparent)
    ceil_z = base_z + wall_h
    lines.append(
        f'<geom name="{name_prefix}_ceil" type="box" '
        f'size="{sw_len / 2:.3f} {step_w / 2 + wt * 2:.3f} 0.04" '
        f'pos="{sw_cx:.3f} {y_center:.3f} {ceil_z:.3f}" '
        f'rgba="0.75 0.73 0.71 0.5"/>')

    return "\n".join(lines)


def _corridor(name_prefix, x_start, x_end, y_center, width, base_z, height,
              with_ceiling=True):
    """Industrial corridor with walls, floor strip, baseboards, pilasters, ceiling beam."""
    if x_end < x_start:
        x_start, x_end = x_end, x_start
    length = x_end - x_start
    if length <= 0:
        return ""

    lines = []
    x_mid = (x_start + x_end) / 2.0
    half_len = length / 2.0
    half_w = width / 2.0
    wall_t = 0.06
    wall_h = height / 2.0
    wall_z = base_z + wall_h
    y_left = y_center + half_w
    y_right = y_center - half_w

    # Walls (industrial gray-blue, more opaque than before)
    rgba_wall = "0.68 0.72 0.76 0.70"
    lines.append(f'<geom name="{name_prefix}_wL" type="box" size="{half_len:.3f} {wall_t:.3f} {wall_h:.3f}" pos="{x_mid:.3f} {y_left:.3f} {wall_z:.3f}" rgba="{rgba_wall}"/>')
    lines.append(f'<geom name="{name_prefix}_wR" type="box" size="{half_len:.3f} {wall_t:.3f} {wall_h:.3f}" pos="{x_mid:.3f} {y_right:.3f} {wall_z:.3f}" rgba="{rgba_wall}"/>')

    # Dark corridor floor strip
    lines.append(f'<geom name="{name_prefix}_floor" type="box" size="{half_len:.3f} {half_w - wall_t:.3f} 0.005" pos="{x_mid:.3f} {y_center:.3f} {base_z + 0.005:.3f}" rgba="0.45 0.44 0.43 0.90"/>')

    # Baseboards (10cm dark strips at wall base)
    bb_h = 0.05
    for side, y in [("bL", y_left), ("bR", y_right)]:
        lines.append(f'<geom name="{name_prefix}_{side}" type="box" size="{half_len:.3f} {wall_t + 0.005:.3f} {bb_h:.3f}" pos="{x_mid:.3f} {y:.3f} {base_z + bb_h:.3f}" rgba="0.25 0.25 0.27 0.85"/>')

    # Door frames (thicker 0.08m) + threshold at both ends
    frame_w = 0.04
    rgba_frame = "0.50 0.52 0.55 0.85"
    for tag, x_pos in [("A", x_start), ("B", x_end)]:
        for js, jy in [("L", y_left), ("R", y_right)]:
            lines.append(f'<geom name="{name_prefix}_d{tag}{js}" type="box" size="{frame_w:.3f} {wall_t:.3f} {wall_h:.3f}" pos="{x_pos:.3f} {jy:.3f} {wall_z:.3f}" rgba="{rgba_frame}"/>')
        lines.append(f'<geom name="{name_prefix}_d{tag}T" type="box" size="{frame_w:.3f} {half_w:.3f} {frame_w:.3f}" pos="{x_pos:.3f} {y_center:.3f} {base_z + height - frame_w:.3f}" rgba="{rgba_frame}"/>')
        lines.append(f'<geom name="{name_prefix}_d{tag}th" type="box" size="{frame_w:.3f} {half_w - wall_t:.3f} 0.015" pos="{x_pos:.3f} {y_center:.3f} {base_z + 0.015:.3f}" rgba="0.40 0.40 0.42 0.90"/>')

    # Pilasters every ~3m for long corridors
    if length > 4.0:
        n_seg = max(2, int(round(length / 3.0)))
        spacing = length / n_seg
        for i in range(1, n_seg):
            x_col = x_start + i * spacing
            for side, y in [("pL", y_left - wall_t - 0.04), ("pR", y_right + wall_t + 0.04)]:
                lines.append(f'<geom name="{name_prefix}_{side}{i}" type="box" size="0.06 0.04 {wall_h:.3f}" pos="{x_col:.3f} {y:.3f} {wall_z:.3f}" rgba="0.60 0.63 0.66 0.80"/>')

    # Ceiling + central beam
    if with_ceiling:
        ceil_z = base_z + height
        lines.append(f'<geom name="{name_prefix}_ceil" type="box" size="{half_len:.3f} {half_w:.3f} 0.03" pos="{x_mid:.3f} {y_center:.3f} {ceil_z:.3f}" rgba="0.78 0.80 0.82 0.55"/>')
        lines.append(f'<geom name="{name_prefix}_beam" type="box" size="{half_len:.3f} 0.05 0.04" pos="{x_mid:.3f} {y_center:.3f} {ceil_z - 0.04:.3f}" rgba="0.55 0.56 0.58 0.75"/>')

    return "\n".join(lines)


def _corridor_details(name_prefix: str, x_start: float, x_end: float,
                      y_center: float, width: float, base_z: float,
                      height: float) -> str:
    """Industrial fixtures inside a corridor: lights, cable tray, fire ext, pipes, signs."""
    x_lo, x_hi = min(x_start, x_end), max(x_start, x_end)
    length = x_hi - x_lo
    if length < 2.0:
        return ""
    cx = (x_lo + x_hi) / 2.0
    p = f"{name_prefix}_d"
    g = []

    # Ceiling lights every ~2.5m
    ceil_z = base_z + height - 0.03
    n_lights = max(1, int(length / 2.5))
    for i in range(n_lights):
        lx = x_lo + length * (i + 0.5) / n_lights
        g.append(f'<geom name="{p}_lt{i}" type="box" pos="{lx:.3f} {y_center:.3f} {ceil_z:.3f}" size="0.15 0.05 0.01" rgba="0.95 0.95 0.97 1"/>')

    # Cable tray along ceiling
    tray_y = y_center + width / 4.0
    g.append(f'<geom name="{p}_tray" type="box" pos="{cx:.3f} {tray_y:.3f} {base_z + height - 0.06:.3f}" size="{length / 2:.3f} 0.075 0.025" rgba="0.35 0.37 0.40 0.95"/>')

    # Fire extinguisher cabinet (midway, left wall)
    fe_y = y_center - width / 2.0 + 0.08
    g.append(f'<geom name="{p}_fire" type="box" pos="{cx:.3f} {fe_y:.3f} {base_z + 0.9:.3f}" size="0.075 0.03 0.125" rgba="0.85 0.15 0.10 1"/>')

    # Exit signs above both ends
    for i, ex in enumerate([x_lo + 0.1, x_hi - 0.1]):
        g.append(f'<geom name="{p}_ex{i}" type="box" pos="{ex:.3f} {y_center:.3f} {base_z + min(2.2, height - 0.15):.3f}" size="0.09 0.01 0.04" rgba="0.15 0.85 0.25 1"/>')

    # Floor direction arrows
    n_arrows = min(3, max(2, int(length / 3.0)))
    for i in range(n_arrows):
        ax = x_lo + length * (i + 0.5) / n_arrows
        g.append(f'<geom name="{p}_ar{i}" type="box" pos="{ax:.3f} {y_center:.3f} {base_z + 0.005:.3f}" size="0.20 0.03 0.001" rgba="0.92 0.78 0.12 0.85"/>')

    # Pipe runs along upper wall
    pipe_y = y_center - width / 2.0 + 0.12
    for i, dz in enumerate([0.08, 0.20]):
        g.append(f'<geom name="{p}_pp{i}" type="box" pos="{cx:.3f} {pipe_y:.3f} {base_z + height - dz:.3f}" size="{length / 2:.3f} 0.02 0.02" rgba="0.50 0.52 0.55 0.90"/>')

    return "\n".join(g)


def _stair_lobby(name_prefix: str, x_pos: float, y_center: float,
                 base_z: float, width: float, depth: float = 2.0,
                 height: float = 2.5) -> str:
    """Lobby/transition zone at stairwell entrance: floor, wing walls, fire door, signs."""
    lines = []
    hw = width / 2.0
    hd = depth / 2.0
    cx = x_pos - hd

    # Dark floor rectangle
    lines.append(f'<geom name="{name_prefix}_floor" type="box" pos="{cx:.3f} {y_center:.3f} {base_z + 0.003:.4f}" size="{hd:.3f} {hw:.3f} 0.003" rgba="0.72 0.74 0.76 0.95"/>')

    # Wing walls framing the stairwell entry
    wing_len = min(1.0, depth * 0.5)
    wing_h = height * 0.45
    wing_cx = x_pos - wing_len / 2.0
    for side, sign in [("L", -1), ("R", 1)]:
        wy = y_center + sign * (hw - 0.03)
        lines.append(f'<geom name="{name_prefix}_wing{side}" type="box" pos="{wing_cx:.3f} {wy:.3f} {base_z + wing_h / 2:.3f}" size="{wing_len / 2:.3f} 0.03 {wing_h / 2:.3f}" rgba="0.65 0.68 0.72 0.55"/>')

    # Fire door frame at x_pos
    door_h = 2.1
    ho = width * 0.35  # half opening
    hf = 0.05
    jamb_z = base_z + door_h / 2.0
    for js, jy in [("L", y_center - ho - hf), ("R", y_center + ho + hf)]:
        lines.append(f'<geom name="{name_prefix}_dj{js}" type="box" pos="{x_pos:.3f} {jy:.3f} {jamb_z:.3f}" size="{hf:.3f} {hf:.3f} {door_h / 2:.3f}" rgba="0.28 0.30 0.34 0.95"/>')
    lines.append(f'<geom name="{name_prefix}_dlin" type="box" pos="{x_pos:.3f} {y_center:.3f} {base_z + door_h + hf:.3f}" size="{hf:.3f} {ho + hf * 2:.3f} {hf:.3f}" rgba="0.28 0.30 0.34 0.95"/>')
    lines.append(f'<geom name="{name_prefix}_dthr" type="box" pos="{x_pos:.3f} {y_center:.3f} {base_z + 0.008:.4f}" size="{hf:.3f} {ho + hf * 2:.3f} 0.008" rgba="0.28 0.30 0.34 0.95"/>')

    # Floor number sign (blue plaque)
    lines.append(f'<geom name="{name_prefix}_sign" type="box" pos="{x_pos - 0.5:.3f} {y_center - hw + 0.08:.3f} {base_z + 1.5:.3f}" size="0.125 0.015 0.075" rgba="0.20 0.45 0.85 1"/>')

    # Emergency exit arrow on floor
    lines.append(f'<geom name="{name_prefix}_arrow" type="box" pos="{cx:.3f} {y_center:.3f} {base_z + 0.007:.4f}" size="0.15 0.04 0.003" rgba="0.15 0.80 0.30 0.90"/>')
    lines.append(f'<geom name="{name_prefix}_arrhd" type="box" pos="{cx + 0.2:.3f} {y_center:.3f} {base_z + 0.007:.4f}" size="0.06 0.08 0.003" rgba="0.15 0.80 0.30 0.90"/>')

    # Ceiling downlights
    light_z = base_z + height - 0.02
    for i, dx in enumerate([-depth * 0.3, depth * 0.3]):
        lines.append(f'<geom name="{name_prefix}_dl{i}" type="cylinder" pos="{cx + dx:.3f} {y_center:.3f} {light_z:.3f}" size="0.08 0.01" rgba="0.95 0.95 0.92 1"/>')

    return "\n".join(lines)

def _room_shell(
    name_prefix: str,
    x1: float,
    x2: float,
    y1: float,
    y2: float,
    base_z: float,
    height: float,
    door_side: str = "south",
    door_width: float = 1.5,
    door_center: float | None = None,
    with_ceiling: bool = False,
) -> str:
    x1, x2 = sorted((x1, x2))
    y1, y2 = sorted((y1, y2))
    wt = 0.05
    wall_cz = base_z + height / 2.0
    wall_rgba = "0.70 0.78 0.84 0.24"
    frame_rgba = "0.42 0.45 0.48 0.90"
    ceil_rgba = "0.82 0.84 0.86 0.18"
    door_h = max(1.95, min(height - 0.12, 2.15))
    lines = []

    def add_x_wall(seg: str, xa: float, xb: float, y: float) -> None:
        if xb - xa <= 0.08:
            return
        cx = (xa + xb) / 2.0
        lines.append(
            f'<geom name="{name_prefix}_{seg}" type="box" '
            f'size="{(xb - xa) / 2:.3f} {wt:.3f} {height / 2:.3f}" '
            f'pos="{cx:.3f} {y:.3f} {wall_cz:.3f}" rgba="{wall_rgba}"/>'
        )

    def add_y_wall(seg: str, x: float, ya: float, yb: float) -> None:
        if yb - ya <= 0.08:
            return
        cy = (ya + yb) / 2.0
        lines.append(
            f'<geom name="{name_prefix}_{seg}" type="box" '
            f'size="{wt:.3f} {(yb - ya) / 2:.3f} {height / 2:.3f}" '
            f'pos="{x:.3f} {cy:.3f} {wall_cz:.3f}" rgba="{wall_rgba}"/>'
        )

    def add_x_frame(tag: str, y: float, center: float) -> None:
        half_gap = door_width / 2.0
        left = center - half_gap
        right = center + half_gap
        lines.append(
            f'<geom name="{name_prefix}_{tag}L" type="box" '
            f'size="0.035 {wt:.3f} {door_h / 2:.3f}" '
            f'pos="{left:.3f} {y:.3f} {base_z + door_h / 2:.3f}" rgba="{frame_rgba}"/>'
        )
        lines.append(
            f'<geom name="{name_prefix}_{tag}R" type="box" '
            f'size="0.035 {wt:.3f} {door_h / 2:.3f}" '
            f'pos="{right:.3f} {y:.3f} {base_z + door_h / 2:.3f}" rgba="{frame_rgba}"/>'
        )
        lines.append(
            f'<geom name="{name_prefix}_{tag}T" type="box" '
            f'size="{door_width / 2:.3f} {wt:.3f} 0.035" '
            f'pos="{center:.3f} {y:.3f} {base_z + door_h + 0.035:.3f}" rgba="{frame_rgba}"/>'
        )

    def add_y_frame(tag: str, x: float, center: float) -> None:
        half_gap = door_width / 2.0
        low = center - half_gap
        high = center + half_gap
        lines.append(
            f'<geom name="{name_prefix}_{tag}L" type="box" '
            f'size="{wt:.3f} 0.035 {door_h / 2:.3f}" '
            f'pos="{x:.3f} {low:.3f} {base_z + door_h / 2:.3f}" rgba="{frame_rgba}"/>'
        )
        lines.append(
            f'<geom name="{name_prefix}_{tag}R" type="box" '
            f'size="{wt:.3f} 0.035 {door_h / 2:.3f}" '
            f'pos="{x:.3f} {high:.3f} {base_z + door_h / 2:.3f}" rgba="{frame_rgba}"/>'
        )
        lines.append(
            f'<geom name="{name_prefix}_{tag}T" type="box" '
            f'size="{wt:.3f} {door_width / 2:.3f} 0.035" '
            f'pos="{x:.3f} {center:.3f} {base_z + door_h + 0.035:.3f}" rgba="{frame_rgba}"/>'
        )

    if door_side == "south":
        center = door_center if door_center is not None else (x1 + x2) / 2.0
        left = max(x1 + 0.3, center - door_width / 2.0)
        right = min(x2 - 0.3, center + door_width / 2.0)
        add_x_wall("s1", x1, left, y1)
        add_x_wall("s2", right, x2, y1)
        add_x_frame("s", y1, center)
        add_x_wall("n", x1, x2, y2)
        add_y_wall("w", x1, y1, y2)
        add_y_wall("e", x2, y1, y2)
    elif door_side == "north":
        center = door_center if door_center is not None else (x1 + x2) / 2.0
        left = max(x1 + 0.3, center - door_width / 2.0)
        right = min(x2 - 0.3, center + door_width / 2.0)
        add_x_wall("s", x1, x2, y1)
        add_x_wall("n1", x1, left, y2)
        add_x_wall("n2", right, x2, y2)
        add_x_frame("n", y2, center)
        add_y_wall("w", x1, y1, y2)
        add_y_wall("e", x2, y1, y2)
    elif door_side == "west":
        center = door_center if door_center is not None else (y1 + y2) / 2.0
        low = max(y1 + 0.3, center - door_width / 2.0)
        high = min(y2 - 0.3, center + door_width / 2.0)
        add_x_wall("s", x1, x2, y1)
        add_x_wall("n", x1, x2, y2)
        add_y_wall("w1", x1, y1, low)
        add_y_wall("w2", x1, high, y2)
        add_y_frame("w", x1, center)
        add_y_wall("e", x2, y1, y2)
    else:
        center = door_center if door_center is not None else (y1 + y2) / 2.0
        low = max(y1 + 0.3, center - door_width / 2.0)
        high = min(y2 - 0.3, center + door_width / 2.0)
        add_x_wall("s", x1, x2, y1)
        add_x_wall("n", x1, x2, y2)
        add_y_wall("w", x1, y1, y2)
        add_y_wall("e1", x2, y1, low)
        add_y_wall("e2", x2, high, y2)
        add_y_frame("e", x2, center)

    if with_ceiling:
        lines.append(
            f'<geom name="{name_prefix}_ceil" type="box" '
            f'size="{(x2 - x1) / 2:.3f} {(y2 - y1) / 2:.3f} 0.035" '
            f'pos="{(x1 + x2) / 2:.3f} {(y1 + y2) / 2:.3f} {base_z + height + 0.035:.3f}" '
            f'rgba="{ceil_rgba}"/>'
        )

    return "\n".join(lines)


def _partitions(floor_id: int, base_z: float) -> str:
    rooms = []
    if floor_id == 1:
        rooms.append(_room_shell("f1_store_rm", 1.8, 7.8, 13.8, 18.4, base_z, 2.25,
                                 door_side="south", door_center=4.8, with_ceiling=True))
        rooms.append(_room_shell("f1_maint_rm", 1.8, 6.8, 1.6, 5.2, base_z, 2.25,
                                 door_side="north", door_center=4.2, with_ceiling=True))
    elif floor_id == 2:
        rooms.append(_room_shell("f2_qc_rm", 2.0, 8.8, 13.6, 18.4, base_z, 2.25,
                                 door_side="south", door_center=5.4, with_ceiling=True))
        rooms.append(_room_shell("f2_pack_rm", 2.0, 7.0, 1.6, 5.2, base_z, 2.25,
                                 door_side="east", door_center=3.5, with_ceiling=True))
    else:
        rooms.append(_room_shell("f3_ctrl_rm", 2.4, 8.8, 13.6, 18.4, base_z, 2.35,
                                 door_side="south", door_center=5.6, with_ceiling=True))
        rooms.append(_room_shell("f3_lab_rm", 2.2, 6.8, 2.0, 5.6, base_z, 2.25,
                                 door_side="east", door_center=3.8, with_ceiling=True))
    return "\n".join(room for room in rooms if room)


def _floor_markings(floor_id: int, base_z: float) -> str:
    z = base_z + 0.004
    p = f"mk{floor_id}"
    lines = []

    def add_box(name: str, sx: float, sy: float, px: float, py: float, rgba: str) -> None:
        lines.append(
            f'<geom name="{name}" type="box" size="{sx:.3f} {sy:.3f} 0.002" '
            f'pos="{px:.3f} {py:.3f} {z:.3f}" rgba="{rgba}"/>'
        )

    yellow = "0.96 0.78 0.14 0.92"
    blue = "0.22 0.58 0.92 0.65"
    green = "0.18 0.72 0.40 0.58"
    orange = "0.96 0.50 0.14 0.70"

    if floor_id == 1:
        for i, px in enumerate([3.0, 5.6, 8.2, 10.8, 13.4, 16.0]):
            add_box(f"{p}_lane{i}", 0.7, 0.06, px, 9.1, yellow)
        add_box(f"{p}_branch", 2.5, 0.06, 18.0, 8.3, yellow)
        add_box(f"{p}_load", 3.2, 1.3, 6.2, 2.6, blue)
        add_box(f"{p}_staging", 3.8, 1.4, 12.6, 15.7, green)
        add_box(f"{p}_hazard", 1.8, 1.1, 20.0, 8.0, orange)
    elif floor_id == 2:
        add_box(f"{p}_bridge", 4.2, 0.08, 23.0, 7.35, yellow)
        add_box(f"{p}_lobby", 2.0, 0.08, 19.0, 7.35, yellow)
        add_box(f"{p}_pack", 3.2, 1.2, 6.0, 4.0, blue)
        add_box(f"{p}_qc", 3.4, 1.2, 5.6, 15.9, green)
        add_box(f"{p}_buffer", 2.0, 0.9, 12.5, 11.8, orange)
    else:
        add_box(f"{p}_arrival", 3.0, 0.08, 15.6, 7.0, yellow)
        add_box(f"{p}_goal", 2.4, 1.0, 10.0, 10.0, green)
        add_box(f"{p}_dispatch", 2.8, 1.2, 13.2, 15.4, blue)
        add_box(f"{p}_lab", 1.8, 0.9, 4.5, 4.0, orange)

    return "\n".join(lines)


def _edge_rails(floor_id: int, base_z: float) -> str:
    if floor_id == 1:
        return ""

    lines = []
    rail_rgba = "0.34 0.36 0.39 0.95"
    top_z = base_z + 1.05
    mid_z = base_z + 0.62

    def add_x_span(name: str, x1: float, x2: float, y: float) -> None:
        cx = (x1 + x2) / 2.0
        half = (x2 - x1) / 2.0
        lines.append(
            f'<geom name="{name}_top" type="box" size="{half:.3f} 0.028 0.028" '
            f'pos="{cx:.3f} {y:.3f} {top_z:.3f}" rgba="{rail_rgba}"/>'
        )
        lines.append(
            f'<geom name="{name}_mid" type="box" size="{half:.3f} 0.020 0.020" '
            f'pos="{cx:.3f} {y:.3f} {mid_z:.3f}" rgba="{rail_rgba}"/>'
        )
        for idx, px in enumerate([x1, (x1 + x2) / 2.0, x2]):
            lines.append(
                f'<geom name="{name}_p{idx}" type="box" size="0.028 0.028 0.52" '
                f'pos="{px:.3f} {y:.3f} {base_z + 0.52:.3f}" rgba="{rail_rgba}"/>'
            )

    def add_y_span(name: str, x: float, y1: float, y2: float) -> None:
        cy = (y1 + y2) / 2.0
        half = (y2 - y1) / 2.0
        lines.append(
            f'<geom name="{name}_top" type="box" size="0.028 {half:.3f} 0.028" '
            f'pos="{x:.3f} {cy:.3f} {top_z:.3f}" rgba="{rail_rgba}"/>'
        )
        lines.append(
            f'<geom name="{name}_mid" type="box" size="0.020 {half:.3f} 0.020" '
            f'pos="{x:.3f} {cy:.3f} {mid_z:.3f}" rgba="{rail_rgba}"/>'
        )
        for idx, py in enumerate([y1, (y1 + y2) / 2.0, y2]):
            lines.append(
                f'<geom name="{name}_p{idx}" type="box" size="0.028 0.028 0.52" '
                f'pos="{x:.3f} {py:.3f} {base_z + 0.52:.3f}" rgba="{rail_rgba}"/>'
            )

    if floor_id == 2:
        add_y_span("f2_east_a", 22.0, 0.8, 4.8)
        add_y_span("f2_east_b", 22.0, 9.2, 19.2)
        add_x_span("f2_ext_n", 22.2, 27.0, 9.0)
        add_x_span("f2_ext_s", 22.2, 27.0, 5.0)
        add_y_span("f2_ext_e", 27.0, 5.2, 8.8)
    else:
        add_y_span("f3_east", 22.0, 0.8, 19.2)

    return "\n".join(lines)


def _building_shell(fz: list[float], roof_z: float) -> str:
    lines = []
    lines.append(
        '<geom name="site_pad" type="box" size="18 13 0.08" pos="15 10 -0.14" '
        'rgba="0.30 0.31 0.33 1"/>'
    )
    lines.append(
        '<geom name="entry_apron" type="box" size="6.2 2.6 0.03" pos="8.0 10.0 -0.03" '
        'rgba="0.48 0.50 0.53 1"/>'
    )

    solid_rgba = "0.72 0.74 0.76 0.82"
    glass_rgba = "0.58 0.72 0.82 0.16"
    frame_rgba = "0.22 0.24 0.28 0.95"
    south_y, north_y = 0.18, 19.82
    west_x, east_x = 0.18, 29.82
    story_bases = [0.0, fz[1], fz[2]]
    story_tops = [fz[1], fz[2], roof_z]

    for idx, (base, top) in enumerate(zip(story_bases, story_tops)):
        span = top - base
        solid_h = min(1.05, max(0.85, span * 0.42))
        glass_h = max(0.55, span - solid_h - 0.20)
        solid_cz = base + solid_h / 2.0
        glass_cz = base + solid_h + glass_h / 2.0
        band_z = top - 0.08

        for side, y in [("s", south_y), ("n", north_y)]:
            lines.append(
                f'<geom name="shell_{side}{idx}_solid" type="box" '
                f'size="15.0 0.16 {solid_h / 2:.3f}" pos="15.0 {y:.3f} {solid_cz:.3f}" '
                f'rgba="{solid_rgba}"/>'
            )
            lines.append(
                f'<geom name="shell_{side}{idx}_glass" type="box" '
                f'size="15.0 0.08 {glass_h / 2:.3f}" pos="15.0 {y:.3f} {glass_cz:.3f}" '
                f'rgba="{glass_rgba}"/>'
            )
        for side, x in [("w", west_x), ("e", east_x)]:
            lines.append(
                f'<geom name="shell_{side}{idx}_solid" type="box" '
                f'size="0.16 10.0 {solid_h / 2:.3f}" pos="{x:.3f} 10.0 {solid_cz:.3f}" '
                f'rgba="{solid_rgba}"/>'
            )
            lines.append(
                f'<geom name="shell_{side}{idx}_glass" type="box" '
                f'size="0.08 10.0 {glass_h / 2:.3f}" pos="{x:.3f} 10.0 {glass_cz:.3f}" '
                f'rgba="{glass_rgba}"/>'
            )

        lines.append(
            f'<geom name="shell_band_s{idx}" type="box" size="15.0 0.12 0.08" '
            f'pos="15.0 {south_y:.3f} {band_z:.3f}" rgba="{frame_rgba}"/>'
        )
        lines.append(
            f'<geom name="shell_band_n{idx}" type="box" size="15.0 0.12 0.08" '
            f'pos="15.0 {north_y:.3f} {band_z:.3f}" rgba="{frame_rgba}"/>'
        )
        lines.append(
            f'<geom name="shell_band_w{idx}" type="box" size="0.12 10.0 0.08" '
            f'pos="{west_x:.3f} 10.0 {band_z:.3f}" rgba="{frame_rgba}"/>'
        )
        lines.append(
            f'<geom name="shell_band_e{idx}" type="box" size="0.12 10.0 0.08" '
            f'pos="{east_x:.3f} 10.0 {band_z:.3f}" rgba="{frame_rgba}"/>'
        )

    for idx, (px, py) in enumerate([(1.2, 1.2), (1.2, 18.8), (28.8, 1.2), (28.8, 18.8), (15.0, 1.2), (15.0, 18.8)]):
        lines.append(
            f'<geom name="shell_col{idx}" type="box" size="0.16 0.16 {roof_z / 2:.3f}" '
            f'pos="{px:.3f} {py:.3f} {roof_z / 2:.3f}" rgba="{frame_rgba}"/>'
        )

    for idx, x in enumerate([4.5, 11.5, 18.5, 25.5]):
        lines.append(
            f'<geom name="roof_x{idx}" type="box" size="0.09 9.2 0.09" '
            f'pos="{x:.3f} 10.0 {roof_z - 0.18:.3f}" rgba="{frame_rgba}"/>'
        )
    for idx, y in enumerate([3.0, 10.0, 17.0]):
        lines.append(
            f'<geom name="roof_y{idx}" type="box" size="14.2 0.09 0.09" '
            f'pos="15.0 {y:.3f} {roof_z - 0.28:.3f}" rgba="{frame_rgba}"/>'
        )

    return "\n".join(lines)


def _scene_lights(fz: list[float]) -> str:
    lines = []
    grid = [(5.0, 4.0), (5.0, 10.0), (5.0, 16.0),
            (13.0, 4.0), (13.0, 10.0), (13.0, 16.0),
            (21.5, 7.5)]
    for floor_id, base_z in enumerate(fz, start=1):
        lum_z = base_z + 2.65
        for idx, (px, py) in enumerate(grid):
            intensity = 0.26 + 0.02 * floor_id
            lines.append(
                f'<light name="lf{floor_id}_{idx}" pos="{px:.2f} {py:.2f} {lum_z:.2f}" '
                f'diffuse="{intensity:.2f} {intensity:.2f} {intensity:.2f}" '
                f'specular="0.08 0.08 0.08" directional="false" castshadow="false"/>'
            )
    return "\n".join(lines)


def _factory_objects(floor_id: int, base_z: float) -> str:
    z = base_z
    p = f"f{floor_id}"
    items = []

    def add_box(name: str, sx: float, sy: float, sz: float,
                px: float, py: float, pz: float, rgba: str) -> None:
        items.append(
            f'<geom name="{name}" type="box" size="{sx:.3f} {sy:.3f} {sz:.3f}" '
            f'pos="{px:.3f} {py:.3f} {pz:.3f}" rgba="{rgba}"/>'
        )

    def add_cyl(name: str, radius: float, half_h: float,
                px: float, py: float, pz: float, rgba: str) -> None:
        items.append(
            f'<geom name="{name}" type="cylinder" size="{radius:.3f} {half_h:.3f}" '
            f'pos="{px:.3f} {py:.3f} {pz:.3f}" rgba="{rgba}"/>'
        )

    def add_rack(prefix: str, px: float, py: float, length: float = 1.6) -> None:
        add_box(f"{prefix}_body", length, 0.45, 1.25, px, py, z + 1.25, "0.50 0.33 0.16 1")
        add_box(f"{prefix}_shelf0", length, 0.50, 0.03, px, py, z + 0.55, "0.76 0.56 0.28 1")
        add_box(f"{prefix}_shelf1", length, 0.50, 0.03, px, py, z + 1.15, "0.76 0.56 0.28 1")
        add_box(f"{prefix}_shelf2", length, 0.50, 0.03, px, py, z + 1.75, "0.76 0.56 0.28 1")

    def add_pallet(prefix: str, px: float, py: float, stack_h: float = 0.32) -> None:
        add_box(f"{prefix}_p", 0.55, 0.45, 0.06, px, py, z + 0.06, "0.56 0.40 0.18 1")
        add_box(f"{prefix}_l", 0.42, 0.32, stack_h, px, py, z + 0.12 + stack_h, "0.68 0.58 0.26 1")

    if floor_id == 1:
        for idx, (px, py) in enumerate([(4.0, 4.2), (4.0, 15.8), (10.2, 4.2), (10.2, 15.8), (15.6, 15.8)]):
            add_rack(f"{p}_rack{idx}", px, py)
        for idx, (px, py) in enumerate([(6.0, 2.4), (8.0, 2.5), (12.8, 14.7), (14.6, 14.8), (16.2, 3.6)]):
            add_pallet(f"{p}_pal{idx}", px, py, 0.24 + 0.06 * (idx % 2))
        add_box(f"{p}_press", 1.1, 0.9, 1.05, 13.0, 10.8, z + 1.05, "0.22 0.38 0.50 1")
        add_box(f"{p}_press_head", 0.45, 0.35, 0.25, 13.0, 10.8, z + 2.15, "0.75 0.76 0.78 1")
        add_box(f"{p}_conv", 2.1, 0.28, 0.12, 8.8, 9.9, z + 0.82, "0.32 0.34 0.36 1")
        add_cyl(f"{p}_tank", 0.58, 1.05, 16.4, 4.6, z + 1.05, "0.58 0.60 0.64 1")
        add_box(f"{p}_panel", 0.35, 0.22, 0.8, 15.4, 6.1, z + 0.80, "0.18 0.24 0.28 1")
    elif floor_id == 2:
        for idx, (px, py) in enumerate([(4.6, 4.0), (8.0, 4.0), (5.8, 11.8), (9.8, 14.6)]):
            add_box(f"{p}_table{idx}", 1.2, 0.55, 0.44, px, py, z + 0.92, "0.58 0.60 0.64 1")
            add_box(f"{p}_table{idx}_top", 1.3, 0.65, 0.04, px, py, z + 1.40, "0.86 0.88 0.90 1")
        for idx, (px, py) in enumerate([(11.8, 15.0), (12.8, 12.2), (14.0, 4.2)]):
            add_pallet(f"{p}_bin{idx}", px, py, 0.18)
        add_box(f"{p}_sorter", 1.9, 0.35, 0.16, 9.5, 8.2, z + 0.86, "0.26 0.30 0.34 1")
        add_box(f"{p}_cab0", 0.5, 0.35, 0.95, 3.0, 16.2, z + 0.95, "0.26 0.34 0.44 1")
        add_box(f"{p}_cab1", 0.5, 0.35, 0.95, 6.8, 16.2, z + 0.95, "0.26 0.34 0.44 1")
    else:
        add_box(f"{p}_desk0", 1.1, 0.65, 0.40, 4.4, 15.1, z + 0.88, "0.68 0.70 0.74 1")
        add_box(f"{p}_desk1", 1.1, 0.65, 0.40, 7.0, 15.1, z + 0.88, "0.68 0.70 0.74 1")
        add_box(f"{p}_mon0", 0.22, 0.08, 0.18, 4.4, 15.4, z + 1.34, "0.14 0.18 0.22 1")
        add_box(f"{p}_mon1", 0.22, 0.08, 0.18, 7.0, 15.4, z + 1.34, "0.14 0.18 0.22 1")
        add_box(f"{p}_srv0", 0.42, 0.42, 1.05, 3.8, 4.0, z + 1.05, "0.18 0.22 0.28 1")
        add_box(f"{p}_srv1", 0.42, 0.42, 1.05, 5.2, 4.0, z + 1.05, "0.18 0.22 0.28 1")
        add_box(f"{p}_bench", 1.6, 0.55, 0.42, 4.6, 3.2, z + 0.88, "0.60 0.62 0.66 1")
        add_box(f"{p}_dispatch", 2.2, 1.0, 0.55, 13.4, 15.0, z + 0.55, "0.54 0.42 0.20 1")
        add_box(f"{p}_crate0", 0.55, 0.55, 0.35, 14.6, 16.4, z + 0.35, "0.70 0.60 0.28 1")
        add_box(f"{p}_crate1", 0.55, 0.55, 0.35, 12.2, 14.0, z + 0.35, "0.70 0.60 0.28 1")

    return "\n".join(items)

def generate_scene_xml(start_pos=None):
    if start_pos is None:
        start_pos = [10.2, 10.0, 0.6]

    step_h = 0.18
    n_steps = 17
    fz = [0.0, n_steps * step_h, 2 * n_steps * step_h]  # [0, 3.06, 6.12]

    # --- Staircase 1鈫? ---
    # Path climbs: x=26.5鈫?1, y~8.0, z=0.7鈫?.1
    s1_x_edge = 21.0     # top connects to floor 2 at x鈮?1
    s1_y = 8.0           # path y during climb 鈮?7.8-8.3
    s1_step_d = 0.32     # run = 17*0.32 = 5.44, bottom at x=26.44
    s1_w = 2.4           # slightly wider for visibility

    stair1 = _stair_geoms("s12", n_steps, s1_x_edge, s1_y, fz[0],
                           step_h, s1_step_d, s1_w)
    rail1 = _stair_railings("s12", n_steps, s1_x_edge, s1_y, fz[0],
                             step_h, s1_step_d, s1_w)

    # --- Staircase 2鈫? ---
    # Path climbs: x=26.5鈫?0, y~6.75, z=3.9鈫?.5
    s2_x_edge = 20.0     # top connects to floor 3 at x鈮?0
    s2_y = 6.75          # path y during climb 鈮?6.7-6.8
    s2_step_d = 0.38     # run = 17*0.38 = 6.46, bottom at x=26.46
    s2_w = 2.4

    stair2 = _stair_geoms("s23", n_steps, s2_x_edge, s2_y, fz[1],
                           step_h, s2_step_d, s2_w)
    rail2 = _stair_railings("s23", n_steps, s2_x_edge, s2_y, fz[1],
                             step_h, s2_step_d, s2_w)

    # Stairwell enclosures (walls + landings + ceiling)
    well1 = _stairwell("sw12", s1_x_edge, s1_y, fz[0],
                        n_steps * s1_step_d, s1_w, n_steps, step_h)
    well2 = _stairwell("sw23", s2_x_edge, s2_y, fz[1],
                        n_steps * s2_step_d, s2_w, n_steps, step_h)
    corr1 = _corridor("c1", 16.5, s1_x_edge, 8.1, 3.0, fz[0], 2.25)
    corr2a = _corridor("c2a", 16.8, s1_x_edge, 7.35, 3.8, fz[1], 2.25)
    corr2b = _corridor("c2b", s1_x_edge, 26.2, 7.35, 3.0, fz[1], 2.25)
    corr3 = _corridor("c3", 11.5, s2_x_edge, 7.05, 3.2, fz[2], 2.25)

    # Corridor interior details (lights, pipes, cable tray, fire ext, signs)
    cd1 = _corridor_details("c1", 16.5, s1_x_edge, 8.1, 3.0, fz[0], 2.25)
    cd2a = _corridor_details("c2a", 16.8, s1_x_edge, 7.35, 3.8, fz[1], 2.25)
    cd2b = _corridor_details("c2b", s1_x_edge, 26.2, 7.35, 3.0, fz[1], 2.25)
    cd3 = _corridor_details("c3", 11.5, s2_x_edge, 7.05, 3.2, fz[2], 2.25)

    # Stair lobby transition zones
    lobby1 = _stair_lobby("lb1", s1_x_edge, s1_y, fz[0], 3.2, 2.0, 2.5)
    lobby2a = _stair_lobby("lb2a", s1_x_edge, 7.35, fz[1], 3.5, 1.8, 2.5)
    lobby2b = _stair_lobby("lb2b", s2_x_edge + n_steps * s2_step_d, s2_y, fz[1], 3.0, 1.5, 2.5)
    lobby3 = _stair_lobby("lb3", s2_x_edge, s2_y, fz[2], 3.2, 2.0, 2.5)

    roof_z = fz[2] + 2.9
    shell = _building_shell(fz, roof_z)
    markings1 = _floor_markings(1, fz[0])
    markings2 = _floor_markings(2, fz[1])
    markings3 = _floor_markings(3, fz[2])
    partitions1 = _partitions(1, fz[0])
    partitions2 = _partitions(2, fz[1])
    partitions3 = _partitions(3, fz[2])
    rails2 = _edge_rails(2, fz[1])
    rails3 = _edge_rails(3, fz[2])
    scene_lights = _scene_lights(fz)

    factory1 = _factory_objects(1, fz[0])
    factory2 = _factory_objects(2, fz[1])
    factory3 = _factory_objects(3, fz[2])

    sx, sy, sz = start_pos

    # Floor 2 & 3: extend to x=22 to cover staircase landings
    # Floor 2 extension: covers staircase 2 bottom area (x=[22,27], y=[5,9])
    xml = textwrap.dedent(f"""\
    <mujoco model="factory_3floor_stairs">
      <option gravity="0 0 -9.81" timestep="0.002"/>
      <visual>
        <global offwidth="1920" offheight="1080"/>
        <quality shadowsize="4096"/>
      </visual>
      <asset>
        <texture name="checker" type="2d" builtin="checker"
                 rgb1="0.86 0.86 0.85" rgb2="0.72 0.72 0.71" width="512" height="512"/>
        <material name="mat_f1" texture="checker" texrepeat="15 10" texuniform="true"/>
        <material name="mat_f2" rgba="0.80 0.82 0.85 0.92"/>
        <material name="mat_f3" rgba="0.84 0.86 0.89 0.92"/>
      </asset>
      <worldbody>
        <light name="top" pos="15 10 12" dir="0 0 -1" diffuse="0.85 0.85 0.88" specular="0.25 0.25 0.25" directional="true"/>
{_indent(scene_lights, 8)}

        <!-- Floor 1: z=0, x=[0,30] -->
        <geom name="floor1" type="box" size="15 10 0.05" pos="15 10 -0.05" material="mat_f1"/>

        <!-- Floor 2: z={fz[1]:.2f}, main x=[0,22] + extension x=[22,27] y=[5,9] -->
        <geom name="floor2" type="box" size="11 10 0.05"
              pos="11 10 {fz[1] - 0.05:.4f}" material="mat_f2"/>
        <geom name="floor2x" type="box" size="2.5 2 0.05"
              pos="24.5 7.5 {fz[1] - 0.05:.4f}" material="mat_f2"/>

        <!-- Floor 3: z={fz[2]:.2f}, x=[0,22] -->
        <geom name="floor3" type="box" size="11 10 0.05"
              pos="11 10 {fz[2] - 0.05:.4f}" material="mat_f3"/>

{_indent(shell, 8)}
{_indent(markings1, 8)}
{_indent(markings2, 8)}
{_indent(markings3, 8)}
{_indent(partitions1, 8)}
{_indent(partitions2, 8)}
{_indent(partitions3, 8)}

        <!-- Stairwell 1->2: enclosure + stairs + rails + corridor + lobby -->
{_indent(well1, 8)}
{_indent(stair1, 8)}
{_indent(rail1, 8)}
{_indent(corr1, 8)}
{_indent(cd1, 8)}
{_indent(lobby1, 8)}

        <!-- Stairwell 2->3: enclosure + stairs + rails + corridors + lobbies -->
{_indent(well2, 8)}
{_indent(stair2, 8)}
{_indent(rail2, 8)}
{_indent(corr2a, 8)}
{_indent(cd2a, 8)}
{_indent(lobby2a, 8)}
{_indent(corr2b, 8)}
{_indent(cd2b, 8)}
{_indent(lobby2b, 8)}
{_indent(corr3, 8)}
{_indent(cd3, 8)}
{_indent(lobby3, 8)}
{_indent(rails2, 8)}
{_indent(rails3, 8)}

{_indent(factory1, 8)}
{_indent(factory2, 8)}
{_indent(factory3, 8)}

        <body name="robot" pos="{sx} {sy} {sz}">
          <freejoint name="robot_jnt"/>
          <geom name="rbody" type="box" size="0.3 0.2 0.15"
                rgba="0.1 0.6 0.9 1" conaffinity="0" contype="0" mass="10"/>
          <geom name="rhead" type="sphere" size="0.08"
                pos="0.25 0 0.10" rgba="0.9 0.2 0.2 1"
                conaffinity="0" contype="0" mass="0.5"/>
        </body>
      </worldbody>
    </mujoco>
    """)
    return xml


if __name__ == "__main__":
    print(generate_scene_xml())

