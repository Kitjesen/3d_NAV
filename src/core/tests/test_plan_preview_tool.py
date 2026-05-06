from __future__ import annotations

import importlib.util
import pickle
import sys
from pathlib import Path

import numpy as np


def _load_tool():
    path = Path(__file__).resolve().parents[3] / "scripts" / "plan_preview.py"
    spec = importlib.util.spec_from_file_location("lingtu_plan_preview_tool", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _write_tomogram(root: Path) -> Path:
    active = root / "active"
    active.mkdir(parents=True)
    data = np.full((5, 3, 4, 6), 10.0, dtype=np.float32)
    data[0, 0, 1, 1] = 80.0
    path = active / "tomogram.pickle"
    with path.open("wb") as fh:
        pickle.dump({"data": data, "resolution": 0.5, "center": [10.0, 20.0]}, fh)
    return path


def test_tomogram_info_reports_bounds_and_last_pose_out_of_bounds(tmp_path):
    tool = _load_tool()
    tomo = _write_tomogram(tmp_path)
    (tmp_path / "active" / "last_pose.txt").write_text("6.0 18.0 1.57\n", encoding="utf-8")

    info = tool.load_tomogram_info(tomo)
    last_pose = tool.read_last_pose(tmp_path)

    assert info.to_dict()["raw_shape"] == [5, 3, 4, 6]
    assert info.to_dict()["ground_shape"] == [4, 6]
    assert info.to_dict()["resolution"] == 0.5
    assert info.to_dict()["origin"] == [8.5, 19.0]
    assert info.to_dict()["max_xy"] == [11.0, 20.5]
    assert info.to_dict()["free_cells"] == 23
    assert last_pose is not None
    assert last_pose.start == [6.0, 18.0, 0.0]
    assert not info.in_bounds(last_pose.start)


def test_default_preview_cases_include_last_pose_and_internal_route(tmp_path):
    tool = _load_tool()
    tomo = _write_tomogram(tmp_path)
    (tmp_path / "active" / "last_pose.txt").write_text("6.0 18.0 1.57\n", encoding="utf-8")

    info = tool.load_tomogram_info(tomo)
    last_pose = tool.read_last_pose(tmp_path)
    cases = tool.build_preview_cases(info, last_pose=last_pose)

    assert [case.name for case in cases] == ["last_pose_to_near_free", "internal_free_route"]
    assert cases[0].start == [6.0, 18.0, 0.0]
    assert info.in_bounds(cases[0].goal)
    assert info.in_bounds(cases[1].start)
    assert info.in_bounds(cases[1].goal)


def test_explicit_goal_uses_last_pose_when_requested(tmp_path):
    tool = _load_tool()
    tomo = _write_tomogram(tmp_path)
    (tmp_path / "active" / "last_pose.txt").write_text("9.0 19.5 0.4\n", encoding="utf-8")

    info = tool.load_tomogram_info(tomo)
    last_pose = tool.read_last_pose(tmp_path)
    cases = tool.build_preview_cases(
        info,
        explicit_goal=[10.0, 20.0, 0.0],
        last_pose=last_pose,
        use_last_pose=True,
    )

    assert len(cases) == 1
    assert cases[0].name == "requested"
    assert cases[0].source == "last_pose_to_explicit_goal"
    assert cases[0].start == [9.0, 19.5, 0.0]
    assert cases[0].goal == [10.0, 20.0, 0.0]


def test_out_of_bounds_case_is_skipped_before_native_planner(tmp_path):
    tool = _load_tool()
    tomo = _write_tomogram(tmp_path)
    info = tool.load_tomogram_info(tomo)

    result = tool.summarize_case(
        info,
        tool.PreviewCase(
            "out_of_bounds",
            start=[1.0, 1.0, 0.0],
            goal=[10.0, 20.0, 0.0],
            source="test",
        ),
        tomogram_path=tomo,
        planner="pct",
        planning_frame="odom",
        obstacle_thr=49.9,
        timeout_s=0.1,
        downsample_dist=2.0,
        allow_out_of_bounds=False,
    )

    assert result["skipped"] is True
    assert result["reasons"] == ["start_out_of_bounds"]
    assert result["feasible"] is False
    assert "backend_class" not in result


def test_aarch64_pct_runtime_report_lists_missing_root_libs(tmp_path):
    tool = _load_tool()
    report = tool.pct_runtime_lib_report(repo_root=tmp_path, machine="aarch64")

    assert report["canonical_arch"] == "aarch64"
    assert report["ok"] is False
    assert "ele_planner.cpython-310-aarch64-linux-gnu.so" in report["missing"]
