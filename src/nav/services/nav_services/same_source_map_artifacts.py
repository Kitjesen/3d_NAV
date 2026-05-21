"""Same-source map artifact writer shared by simulation and map services.

The writer persists a point cloud and optional PCT tomogram with enough
provenance metadata to prove both artifacts came from the same live mapping
source. Simulation gates may call this module, but the artifact contract lives
under ``src`` so product map/relocalization/PCT flows can reuse it.
"""

from __future__ import annotations

import hashlib
import json
import math
import time
from pathlib import Path
from typing import Any

import numpy as np


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as fh:
        for chunk in iter(lambda: fh.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def write_ascii_pcd(path: Path, points: np.ndarray) -> int:
    """Write finite XYZ points to a portable ASCII PCD artifact."""
    path.parent.mkdir(parents=True, exist_ok=True)
    pts = np.asarray(points, dtype=np.float32)
    if pts.ndim != 2 or pts.shape[1] < 3:
        raise ValueError(f"expected Nx3 points, got shape={pts.shape}")
    pts = pts[:, :3]
    finite = np.isfinite(pts).all(axis=1)
    pts = pts[finite]

    header = "\n".join(
        [
            "# .PCD v0.7 - Point Cloud Data file format",
            "VERSION 0.7",
            "FIELDS x y z",
            "SIZE 4 4 4",
            "TYPE F F F",
            "COUNT 1 1 1",
            f"WIDTH {len(pts)}",
            "HEIGHT 1",
            "VIEWPOINT 0 0 0 1 0 0 0",
            f"POINTS {len(pts)}",
            "DATA ascii",
        ]
    )
    with path.open("w", encoding="ascii") as fh:
        fh.write(header)
        fh.write("\n")
        for x, y, z in pts:
            fh.write(f"{float(x):.5f} {float(y):.5f} {float(z):.5f}\n")
    return int(len(pts))


def add_points_to_voxel_store(
    store: dict[tuple[int, int, int], tuple[float, float, float]],
    points: np.ndarray,
    *,
    voxel_size: float,
    max_points: int,
) -> int:
    """Add a bounded finite XYZ point sample into an in-memory voxel store."""
    pts = np.asarray(points, dtype=np.float32)
    if pts.ndim != 2 or pts.shape[1] < 3 or pts.shape[0] == 0:
        return 0
    if len(store) >= int(max_points):
        return 0
    inv = 1.0 / max(float(voxel_size), 1.0e-6)
    added = 0
    for x, y, z in pts[:, :3]:
        if not (
            math.isfinite(float(x))
            and math.isfinite(float(y))
            and math.isfinite(float(z))
        ):
            continue
        cell = (
            int(math.floor(float(x) * inv)),
            int(math.floor(float(y) * inv)),
            int(math.floor(float(z) * inv)),
        )
        if cell in store:
            continue
        store[cell] = (float(x), float(y), float(z))
        added += 1
        if len(store) >= int(max_points):
            break
    return added


def point_bounds(points: np.ndarray) -> dict[str, list[float]] | None:
    pts = np.asarray(points, dtype=np.float32)
    if pts.ndim != 2 or pts.shape[0] == 0 or pts.shape[1] < 3:
        return None
    pts = pts[:, :3]
    finite = np.isfinite(pts).all(axis=1)
    pts = pts[finite]
    if pts.shape[0] == 0:
        return None
    return {
        "min": pts.min(axis=0).astype(float).tolist(),
        "max": pts.max(axis=0).astype(float).tolist(),
    }


def build_tomogram_artifact(
    *,
    pcd_path: Path,
    tomogram_path: Path,
    resolution: float,
    slice_dh: float,
    ground_h: float,
) -> dict[str, Any]:
    from global_planning.PCT_planner.tomography.scripts.build_tomogram import (
        build_tomogram_from_pcd,
    )

    data = build_tomogram_from_pcd(
        str(pcd_path),
        str(tomogram_path),
        resolution=float(resolution),
        slice_dh=float(slice_dh),
        ground_h=float(ground_h),
    )
    arr = np.asarray(data.get("data"))
    return {
        "path": str(tomogram_path),
        "exists": tomogram_path.exists(),
        "sha256": sha256_file(tomogram_path) if tomogram_path.exists() else "",
        "input_pcd": str(pcd_path),
        "input_pcd_sha256": sha256_file(pcd_path) if pcd_path.exists() else "",
        "same_source_input": True,
        "shape": list(arr.shape),
        "resolution": float(data.get("resolution", resolution)),
        "center": np.asarray(data.get("center", [0.0, 0.0]), dtype=float).tolist(),
        "slice_h0": float(data.get("slice_h0", 0.0)),
        "slice_dh": float(data.get("slice_dh", slice_dh)),
    }


def write_same_source_map_artifacts(
    *,
    artifact_dir: Path,
    points: np.ndarray,
    frame_id: str,
    world: Path,
    source_topics: tuple[str, ...],
    mapping_input_path: str,
    build_tomogram: bool,
    tomogram_resolution: float,
    tomogram_slice_dh: float,
    tomogram_ground_h: float,
    map_artifact_max_span_m: float,
    tomogram_max_cells: int,
    source: str = "same_source_map_artifact_writer",
    extra_metadata: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Persist same-run map artifacts for saved-map/PCT follow-up gates."""
    artifact_dir.mkdir(parents=True, exist_ok=True)
    report: dict[str, Any] = {
        "ok": False,
        "artifact_dir": str(artifact_dir),
        "blockers": [],
        "source_contract": {
            "map_save_source": source,
            "same_source_pcd": False,
            "same_source_tomogram": False,
            "source_topics": list(source_topics),
            "mapping_input_path": mapping_input_path,
            "frame_id": frame_id,
            "world": str(world),
        },
        "assets": {},
    }
    pts = np.asarray(points, dtype=np.float32)
    if pts.ndim != 2 or pts.shape[0] <= 0 or pts.shape[1] < 3:
        report["blockers"].append("no accumulated /nav/map_cloud points for map.pcd")
        return report
    pts = pts[:, :3]
    finite = np.isfinite(pts).all(axis=1)
    pts = pts[finite]
    if pts.shape[0] <= 0:
        report["blockers"].append("accumulated /nav/map_cloud points are not finite")
        return report

    bounds = point_bounds(pts)
    span = None
    if bounds is not None:
        min_xyz = np.asarray(bounds["min"], dtype=np.float64)
        max_xyz = np.asarray(bounds["max"], dtype=np.float64)
        span = (max_xyz - min_xyz).astype(float).tolist()
        max_span = max(float(value) for value in span)
        if max_span > float(map_artifact_max_span_m):
            report["blockers"].append(
                "map bounds span exceeds limit "
                f"(span={span}, max_allowed={float(map_artifact_max_span_m):.3f}m)"
            )

    pcd_path = artifact_dir / "map.pcd"
    metadata_path = artifact_dir / "metadata.json"
    point_count = write_ascii_pcd(pcd_path, pts)
    pcd_sha = sha256_file(pcd_path)
    metadata = {
        "schema_version": "lingtu.same_source_map_artifacts.v1",
        "source": source,
        "frame_id": frame_id,
        "source_topics": list(source_topics),
        "mapping_input_path": mapping_input_path,
        "world": str(world),
        "point_count": int(point_count),
        "bounds": bounds,
        "span_m": span,
        "max_allowed_span_m": float(map_artifact_max_span_m),
        "pcd": str(pcd_path),
        "pcd_sha256": pcd_sha,
        "generated_unix_s": time.time(),
    }
    if extra_metadata:
        metadata.update(dict(extra_metadata))
    metadata_path.write_text(
        json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    metadata_sha = sha256_file(metadata_path)
    report["assets"]["map_pcd"] = {
        "path": str(pcd_path),
        "exists": pcd_path.exists(),
        "point_count": int(point_count),
        "sha256": pcd_sha,
        "frame_id": frame_id,
        "bounds": metadata["bounds"],
        "span_m": span,
    }
    report["assets"]["metadata"] = {
        "path": str(metadata_path),
        "exists": metadata_path.exists(),
        "sha256": metadata_sha,
        "required_fields": [
            "source",
            "frame_id",
            "source_topics",
            "mapping_input_path",
            "world",
            "point_count",
            "pcd_sha256",
        ],
    }
    report["source_contract"]["same_source_pcd"] = bool(
        pcd_path.exists() and point_count > 0 and bool(pcd_sha)
    )

    if build_tomogram and not report["blockers"]:
        if bounds is not None and span is not None:
            nx = max(1, int(math.ceil(float(span[0]) / max(float(tomogram_resolution), 1e-6))))
            ny = max(1, int(math.ceil(float(span[1]) / max(float(tomogram_resolution), 1e-6))))
            nz = max(1, int(math.ceil(float(span[2]) / max(float(tomogram_slice_dh), 1e-6))))
            cell_count = int(nx) * int(ny) * int(nz)
            report["assets"]["tomogram_estimate"] = {
                "cells": cell_count,
                "shape_xyz": [nx, ny, nz],
                "max_allowed_cells": int(tomogram_max_cells),
            }
            if cell_count > int(tomogram_max_cells):
                report["blockers"].append(
                    "tomogram cell estimate exceeds limit "
                    f"(cells={cell_count}, max_allowed={int(tomogram_max_cells)})"
                )
        if report["blockers"]:
            report["source_contract"]["same_source_tomogram"] = False

    if build_tomogram and not report["blockers"]:
        tomogram_path = artifact_dir / "tomogram.pickle"
        try:
            tomogram = build_tomogram_artifact(
                pcd_path=pcd_path,
                tomogram_path=tomogram_path,
                resolution=tomogram_resolution,
                slice_dh=tomogram_slice_dh,
                ground_h=tomogram_ground_h,
            )
            report["assets"]["tomogram"] = tomogram
            report["source_contract"]["same_source_tomogram"] = bool(
                tomogram.get("exists")
                and tomogram.get("input_pcd_sha256") == pcd_sha
                and bool(tomogram.get("sha256"))
            )
            if not report["source_contract"]["same_source_tomogram"]:
                report["blockers"].append("tomogram was not built from the saved map.pcd")
        except Exception as exc:
            report["blockers"].append(
                f"tomogram build failed: {type(exc).__name__}: {exc}"
            )

    report["ok"] = (
        bool(report["source_contract"]["same_source_pcd"])
        and (not build_tomogram or bool(report["source_contract"]["same_source_tomogram"]))
        and not report["blockers"]
    )
    return report
