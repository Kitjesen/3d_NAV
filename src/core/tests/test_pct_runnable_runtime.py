from __future__ import annotations

import sys
from pathlib import Path
import pickle

import numpy as np
from global_planning.PCT_planner_runnable import runtime as pct_runtime
from global_planning.PCT_planner_runnable.runtime import (
    inspect_pct_runtime,
    prepare_pct_runtime,
    prepare_tomogram_for_pct,
    resolve_pct_runtime_paths,
)


def _touch_pct_extensions(lib_dir: Path) -> None:
    abi = f"{sys.version_info.major}{sys.version_info.minor}"
    for name in ("a_star", "ele_planner", "traj_opt"):
        (lib_dir / f"{name}.cpython-{abi}-x86_64-linux-gnu.so").write_bytes(b"")


def test_resolve_prefers_arch_specific_lib_dir(tmp_path):
    repo = tmp_path
    lib_dir = repo / "src/global_planning/PCT_planner/planner/lib/x86_64"
    lib_dir.mkdir(parents=True)
    _touch_pct_extensions(lib_dir)

    paths = resolve_pct_runtime_paths(repo, machine="x86_64")

    assert paths.lib_dir == lib_dir
    assert paths.canonical_arch == "x86_64"


def test_resolve_prefers_runnable_native_dir_over_original_lib(tmp_path):
    repo = tmp_path
    original_dir = repo / "src/global_planning/PCT_planner/planner/lib/x86_64"
    runnable_dir = repo / "src/global_planning/PCT_planner_runnable/native/x86_64"
    original_dir.mkdir(parents=True)
    runnable_dir.mkdir(parents=True)
    _touch_pct_extensions(original_dir)
    _touch_pct_extensions(runnable_dir)

    paths = resolve_pct_runtime_paths(repo, machine="x86_64")

    assert paths.lib_dir == runnable_dir


def test_resolve_prefers_env_lib_dir_over_runnable_native_dir(tmp_path, monkeypatch):
    repo = tmp_path
    env_dir = repo / "custom_pct_lib"
    runnable_dir = repo / "src/global_planning/PCT_planner_runnable/native/x86_64"
    env_dir.mkdir(parents=True)
    runnable_dir.mkdir(parents=True)
    _touch_pct_extensions(env_dir)
    _touch_pct_extensions(runnable_dir)
    monkeypatch.setenv("LINGTU_PCT_LIB_DIR", str(env_dir))

    paths = resolve_pct_runtime_paths(repo, machine="x86_64")

    assert paths.lib_dir == env_dir


def test_inspect_pct_runtime_uses_same_env_dir_as_loader(tmp_path, monkeypatch):
    repo = tmp_path
    env_dir = repo / "custom_pct_lib"
    fallback_dir = repo / "src/global_planning/PCT_planner_runnable/native/x86_64"
    env_dir.mkdir(parents=True)
    fallback_dir.mkdir(parents=True)
    _touch_pct_extensions(env_dir)
    _touch_pct_extensions(fallback_dir)
    monkeypatch.setenv("LINGTU_PCT_LIB_DIR", str(env_dir))

    report = inspect_pct_runtime(repo, machine="x86_64")

    assert report["ok"] is True
    assert Path(report["lib_dir"]) == env_dir
    assert report["missing"] == []


def test_prepare_installs_lib_namespace_for_original_wrapper(tmp_path):
    repo = tmp_path
    planner_root = repo / "src/global_planning/PCT_planner/planner"
    lib_dir = planner_root / "lib/x86_64"
    scripts_dir = planner_root / "scripts"
    lib_dir.mkdir(parents=True)
    scripts_dir.mkdir(parents=True)
    _touch_pct_extensions(lib_dir)

    previous_lib = sys.modules.pop("lib", None)
    try:
        paths = prepare_pct_runtime(repo, machine="x86_64")
        lib_module = sys.modules["lib"]
        assert str(paths.lib_dir) in list(lib_module.__path__)
        assert str(paths.lib_root) in list(lib_module.__path__)
        assert str(paths.scripts_dir) in sys.path
    finally:
        if previous_lib is not None:
            sys.modules["lib"] = previous_lib
        else:
            sys.modules.pop("lib", None)


def test_prepare_tomogram_for_pct_transposes_builder_axes(tmp_path):
    source = tmp_path / "tomogram.pickle"
    data = np.zeros((5, 2, 3, 4), dtype=np.float32)
    with source.open("wb") as handle:
        pickle.dump(
            {
                "data": data,
                "resolution": 0.25,
                "center": [1.0, 2.0],
                "slice_h0": 0.0,
                "slice_dh": 0.5,
            },
            handle,
        )

    prepared = prepare_tomogram_for_pct(source)

    assert prepared != source
    with prepared.open("rb") as handle:
        normalized = pickle.load(handle)
    assert normalized["pct_axes_transposed"] is True
    assert normalized["data"].shape == (5, 2, 4, 3)


def test_prepare_tomogram_for_pct_returns_source_when_axes_already_transposed(tmp_path):
    source = tmp_path / "tomogram.pickle"
    data = np.zeros((5, 2, 4, 3), dtype=np.float32)
    with source.open("wb") as handle:
        pickle.dump(
            {
                "data": data,
                "resolution": 0.25,
                "center": [1.0, 2.0],
                "slice_h0": 0.0,
                "slice_dh": 0.5,
                "pct_axes_transposed": True,
            },
            handle,
        )

    prepared = prepare_tomogram_for_pct(source)

    assert prepared == source.resolve()
    assert not (tmp_path / ".pct_runnable_cache").exists()


def test_prepare_tomogram_for_pct_reuses_cache_for_same_source_stat(tmp_path):
    source = tmp_path / "tomogram.pickle"
    data = np.zeros((5, 2, 3, 4), dtype=np.float32)
    with source.open("wb") as handle:
        pickle.dump(
            {
                "data": data,
                "resolution": 0.25,
                "center": [1.0, 2.0],
                "slice_h0": 0.0,
                "slice_dh": 0.5,
            },
            handle,
        )

    first = prepare_tomogram_for_pct(source)
    second = prepare_tomogram_for_pct(source)

    assert second == first


def test_prepare_tomogram_for_pct_falls_back_when_map_cache_is_unwritable(tmp_path, monkeypatch):
    blocked_parent = tmp_path / "blocked_parent"
    blocked_parent.write_text("file blocks mkdir parents", encoding="utf-8")
    fallback_dir = tmp_path / "fallback_cache"
    monkeypatch.setattr(
        pct_runtime,
        "_pct_cache_dirs",
        lambda source: [blocked_parent / ".pct_runnable_cache", fallback_dir],
    )
    source = tmp_path / "tomogram.pickle"
    data = np.zeros((5, 2, 3, 4), dtype=np.float32)
    with source.open("wb") as handle:
        pickle.dump(
            {
                "data": data,
                "resolution": 0.25,
                "center": [1.0, 2.0],
                "slice_h0": 0.0,
                "slice_dh": 0.5,
            },
            handle,
        )

    prepared = prepare_tomogram_for_pct(source)

    assert prepared.parent == fallback_dir
    assert prepared.exists()
