"""MuJoCo scene metadata extraction for simulation acceptance gates."""

from __future__ import annotations

import math
import json
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any


_GROUND_NAME_TOKENS = ("ground", "floor", "road", "start_disk", "goal_disk")
_OVERHEAD_NAME_TOKENS = ("roof", "beam", "pipe", "light")


def _floats(text: str | None, *, default: tuple[float, ...] = ()) -> list[float]:
    if text is None:
        return list(default)
    out: list[float] = []
    for item in str(text).split():
        try:
            value = float(item)
        except ValueError:
            return list(default)
        if not math.isfinite(value):
            return list(default)
        out.append(value)
    return out


def _is_static_collision_geom(elem: ET.Element, name: str) -> bool:
    if elem.get("type", "box") not in {"box", "cylinder"}:
        return False
    if elem.get("contype") == "0" and elem.get("conaffinity") == "0":
        return False
    lowered = name.lower()
    if any(token in lowered for token in _GROUND_NAME_TOKENS):
        return False
    if any(token in lowered for token in _OVERHEAD_NAME_TOKENS):
        return False
    return True


def extract_robot_height_obstacle_boxes(
    scene_xml: Path,
    *,
    robot_min_z: float = 0.05,
    robot_max_z: float = 1.25,
) -> list[dict[str, Any]]:
    """Return box-like obstacles that overlap the robot body height.

    The native local-planner gate expects simple axis-aligned boxes in metadata.
    This helper keeps scene parsing in ``src`` so script gates do not each invent
    their own world parser. Cylinders are conservatively approximated as square
    boxes in XY.
    """

    scene_xml = Path(scene_xml)
    root = ET.parse(scene_xml).getroot()
    obstacles: list[dict[str, Any]] = []
    for index, geom in enumerate(root.findall(".//geom")):
        name = str(geom.get("name") or f"geom_{index}")
        if not _is_static_collision_geom(geom, name):
            continue
        pos = _floats(geom.get("pos"), default=(0.0, 0.0, 0.0))
        size = _floats(geom.get("size"), default=())
        geom_type = geom.get("type", "box")
        if len(pos) < 3 or not size:
            continue
        if geom_type == "box":
            if len(size) < 3:
                continue
            half_size = [abs(float(size[0])), abs(float(size[1])), abs(float(size[2]))]
        else:
            radius = abs(float(size[0]))
            half_z = abs(float(size[1])) if len(size) > 1 else radius
            half_size = [radius, radius, half_z]
        center_z = float(pos[2])
        bottom = center_z - half_size[2]
        top = center_z + half_size[2]
        if top < float(robot_min_z) or bottom > float(robot_max_z):
            continue
        obstacles.append(
            {
                "name": name,
                "type": geom_type,
                "position": [float(pos[0]), float(pos[1]), center_z],
                "half_size": half_size,
                "floor_id": 0,
            }
        )
    return obstacles


def write_scene_obstacle_metadata(
    *,
    scene_xml: Path,
    output: Path,
    source_map_metadata: Path | None = None,
) -> dict[str, Any]:
    """Write a native-gate compatible metadata file for a MuJoCo scene."""

    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    obstacles = extract_robot_height_obstacle_boxes(scene_xml)
    payload: dict[str, Any] = {
        "schema_version": "lingtu.mujoco_scene_obstacle_metadata.v1",
        "scene_xml": str(Path(scene_xml)),
        "source_map_metadata": str(source_map_metadata or ""),
        "obstacles": obstacles,
        "obstacle_count": len(obstacles),
    }
    output.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return payload
