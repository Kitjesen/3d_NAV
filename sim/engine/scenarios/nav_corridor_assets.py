"""Deterministic corridor-gap assets for simulation navigation validation."""

from __future__ import annotations

import json
import pickle
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import numpy as np


@dataclass(frozen=True)
class CorridorBox:
    name: str
    position: tuple[float, float, float]
    half_size: tuple[float, float, float]
    rgba: tuple[float, float, float, float]


@dataclass(frozen=True)
class CorridorAssets:
    scene_xml: Path
    tomogram: Path
    map_pcd: Path
    metadata: Path
    start: tuple[float, float, float]
    goal: tuple[float, float, float]


DEFAULT_START = (0.0, 0.0, 0.55)
DEFAULT_GOAL = (2.4, 0.0, 0.0)
DEFAULT_RESOLUTION = 0.1
DEFAULT_ORIGIN = (-0.6, -1.5)
DEFAULT_SHAPE = (34, 38)  # rows(y), cols(x): x=[-0.6,3.1], y=[-1.5,1.8]


def corridor_boxes() -> list[CorridorBox]:
    """Return wall geometry with a single offset passable gap.

    Direct start->goal crosses the lower wall segment. A feasible path must
    move upward through the gap near y=0.75 before returning to the goal.
    """

    return [
        CorridorBox(
            name="corridor_wall_lower",
            position=(1.1, -0.55, 0.45),
            half_size=(0.08, 0.90, 0.45),
            rgba=(0.55, 0.55, 0.62, 1.0),
        ),
        CorridorBox(
            name="corridor_wall_upper",
            position=(1.1, 1.45, 0.45),
            half_size=(0.08, 0.35, 0.45),
            rgba=(0.55, 0.55, 0.62, 1.0),
        ),
        CorridorBox(
            name="left_guide_block",
            position=(0.55, -0.95, 0.30),
            half_size=(0.20, 0.20, 0.30),
            rgba=(0.70, 0.35, 0.22, 1.0),
        ),
        CorridorBox(
            name="right_guide_block",
            position=(1.85, 1.20, 0.30),
            half_size=(0.18, 0.18, 0.30),
            rgba=(0.70, 0.35, 0.22, 1.0),
        ),
    ]


def _format_vec(values: tuple[float, ...]) -> str:
    return " ".join(f"{v:.4f}".rstrip("0").rstrip(".") for v in values)


def _scene_xml(start: tuple[float, float, float], goal: tuple[float, float, float]) -> str:
    geoms = []
    for box in corridor_boxes():
        geoms.append(
            f'    <geom name="{box.name}" type="box" '
            f'size="{_format_vec(box.half_size)}" '
            f'pos="{_format_vec(box.position)}" '
            f'rgba="{_format_vec(box.rgba)}" '
            f'contype="1" conaffinity="1" group="1"/>\n'
        )
    return f"""<mujoco model="lingtu_nav_corridor">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.002"/>
  <visual>
    <global offwidth="1280" offheight="960"/>
    <headlight ambient="0.4 0.4 0.4"/>
    <quality shadowsize="2048"/>
    <map znear="0.01" zfar="100"/>
  </visual>
  <asset>
    <material name="floor_mat" rgba=".76 .78 .74 1"/>
  </asset>
  <worldbody>
    <body name="robot_placeholder" pos="{_format_vec(start)}"/>
    <light pos="4 -4 8" dir="-0.3 0.3 -1" diffuse="0.9 0.88 0.82" castshadow="false"/>
    <geom name="floor" type="plane" size="8 5 0.1" material="floor_mat"
          conaffinity="1" condim="3" friction="1 0.5 0.5" group="1"/>
{''.join(geoms)}    <geom name="goal_marker" type="sphere" size="0.12"
          pos="{_format_vec((goal[0], goal[1], 0.12))}"
          contype="0" conaffinity="0" rgba="0.1 0.35 1 0.65" group="1"/>
  </worldbody>
</mujoco>
"""


def _mark_box(
    grid: np.ndarray,
    *,
    box: CorridorBox,
    origin: tuple[float, float],
    resolution: float,
    inflation: float,
) -> None:
    h, w = grid.shape
    x0 = box.position[0] - box.half_size[0] - inflation
    x1 = box.position[0] + box.half_size[0] + inflation
    y0 = box.position[1] - box.half_size[1] - inflation
    y1 = box.position[1] + box.half_size[1] + inflation
    c0 = max(0, int(np.floor((x0 - origin[0]) / resolution)))
    c1 = min(w - 1, int(np.ceil((x1 - origin[0]) / resolution)))
    r0 = max(0, int(np.floor((y0 - origin[1]) / resolution)))
    r1 = min(h - 1, int(np.ceil((y1 - origin[1]) / resolution)))
    grid[r0 : r1 + 1, c0 : c1 + 1] = 100.0


def _tomogram(
    *,
    resolution: float,
    origin: tuple[float, float],
    shape: tuple[int, int],
    inflation: float,
) -> dict[str, Any]:
    grid = np.zeros(shape, dtype=np.float32)
    for box in corridor_boxes():
        _mark_box(grid, box=box, origin=origin, resolution=resolution, inflation=inflation)
    h, w = shape
    data = np.zeros((5, 1, h, w), dtype=np.float32)
    data[0, 0] = grid
    center = [
        origin[0] + w * resolution / 2.0,
        origin[1] + h * resolution / 2.0,
    ]
    return {
        "data": data,
        "resolution": float(resolution),
        "center": center,
        "origin": list(origin),
        "grid_info": {
            "origin": list(origin),
            "resolution": float(resolution),
            "shape": [int(h), int(w)],
        },
        "meta": {
            "source": "sim_corridor_gap_geometry",
            "obstacle_thr": 49.9,
            "inflation_m": float(inflation),
        },
    }


def _sample_box_surfaces(box: CorridorBox, step: float = 0.08) -> np.ndarray:
    cx, cy, cz = box.position
    hx, hy, hz = box.half_size
    xs = np.arange(cx - hx, cx + hx + step * 0.5, step)
    ys = np.arange(cy - hy, cy + hy + step * 0.5, step)
    zs = np.arange(max(0.0, cz - hz), cz + hz + step * 0.5, step)
    pts: list[tuple[float, float, float]] = []
    for x in xs:
        for y in ys:
            pts.append((float(x), float(y), float(cz + hz)))
    for z in zs:
        for y in ys:
            pts.append((float(cx - hx), float(y), float(z)))
            pts.append((float(cx + hx), float(y), float(z)))
        for x in xs:
            pts.append((float(x), float(cy - hy), float(z)))
            pts.append((float(x), float(cy + hy), float(z)))
    return np.asarray(pts, dtype=np.float32)


def _write_ascii_pcd(path: Path) -> None:
    points = np.concatenate([_sample_box_surfaces(box) for box in corridor_boxes()], axis=0)
    lines = [
        "# .PCD v0.7 - Point Cloud Data file format",
        "VERSION 0.7",
        "FIELDS x y z",
        "SIZE 4 4 4",
        "TYPE F F F",
        "COUNT 1 1 1",
        f"WIDTH {len(points)}",
        "HEIGHT 1",
        "VIEWPOINT 0 0 0 1 0 0 0",
        f"POINTS {len(points)}",
        "DATA ascii",
    ]
    for x, y, z in points:
        lines.append(f"{x:.4f} {y:.4f} {z:.4f}")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def build_corridor_gap_assets(
    output_dir: str | Path,
    *,
    start: tuple[float, float, float] = DEFAULT_START,
    goal: tuple[float, float, float] = DEFAULT_GOAL,
    resolution: float = DEFAULT_RESOLUTION,
    origin: tuple[float, float] = DEFAULT_ORIGIN,
    shape: tuple[int, int] = DEFAULT_SHAPE,
    inflation: float = 0.12,
) -> CorridorAssets:
    out = Path(output_dir).resolve()
    out.mkdir(parents=True, exist_ok=True)
    scene_xml = out / "corridor_gap_scene.xml"
    tomogram_path = out / "tomogram.pickle"
    map_pcd = out / "map.pcd"
    metadata = out / "metadata.json"

    scene_xml.write_text(_scene_xml(start, goal), encoding="utf-8")
    with tomogram_path.open("wb") as fh:
        pickle.dump(
            _tomogram(
                resolution=resolution,
                origin=origin,
                shape=shape,
                inflation=inflation,
            ),
            fh,
        )
    _write_ascii_pcd(map_pcd)

    metadata.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "name": "corridor_gap",
                "source": "synthetic_sim_geometry",
                "scene_xml": str(scene_xml),
                "tomogram": str(tomogram_path),
                "map_pcd": str(map_pcd),
                "start": list(start),
                "goal": list(goal),
                "resolution": resolution,
                "origin": list(origin),
                "shape": list(shape),
                "inflation_m": inflation,
                "obstacles": [asdict(box) for box in corridor_boxes()],
            },
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )

    return CorridorAssets(
        scene_xml=scene_xml,
        tomogram=tomogram_path,
        map_pcd=map_pcd,
        metadata=metadata,
        start=start,
        goal=goal,
    )
