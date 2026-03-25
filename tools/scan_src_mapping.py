#!/usr/bin/env python3
"""
扫描 src/ 生成 hive 迁移映射文档。

输出: docs/hive_migration_map.md

对每个文件分析:
  - 语言 (Python/C++/Rust/Dart/Config)
  - 行数
  - ROS2 依赖 (rclpy/rclcpp/launch)
  - import 关系
  - 自动分类到 hive 模块
  - 迁移类型 (纯算法/ProcessModule/不迁移)
"""

import os
import re
from collections import defaultdict
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
SRC = REPO / "src"
OUT = REPO / "docs" / "hive_migration_map.md"

# 跳过的目录
SKIP_DIRS = {
    "__pycache__", "build", "install", "log", ".omc", ".vscode",
    "target",  # Rust build
    "3rdparty", "Livox-SDK2",  # 第三方
    "OrbbecSDK_ROS2",  # submodule
    "robot_proto",  # protobuf submodule
}

# 文件扩展名 → 语言
LANG_MAP = {
    ".py": "Python",
    ".cpp": "C++",
    ".hpp": "C++",
    ".h": "C++",
    ".c": "C",
    ".rs": "Rust",
    ".dart": "Dart",
    ".yaml": "Config",
    ".yml": "Config",
    ".xml": "Config",
    ".json": "Config",
    ".launch.py": "Launch",
}

# ROS2 标记关键词
ROS2_MARKERS_PY = ["rclpy", "from rclpy", "import rclpy", "Node)", "create_subscription", "create_publisher"]
ROS2_MARKERS_CPP = ["rclcpp", "rclcpp::Node", "create_subscription", "create_publisher"]

# 自动分类规则: (路径包含, 分类)
CATEGORY_RULES = [
    ("drivers/livox", "hardware/lidar"),
    ("drivers/robot_driver", "hardware/driver"),
    ("drivers/lingtu-bridge", "hardware/bridge_rust"),
    ("slam/fastlio2", "slam/fastlio2"),
    ("slam/pointlio", "slam/pointlio"),
    ("slam/localizer", "slam/localizer"),
    ("slam/pgo", "slam/pgo"),
    ("slam/hba", "slam/hba"),
    ("slam/interface", "slam/interface"),
    ("base_autonomy/terrain_analysis_ext", "local_plan/terrain_ext"),
    ("base_autonomy/terrain_analysis", "local_plan/terrain"),
    ("base_autonomy/local_planner", "local_plan/local_planner"),
    ("base_autonomy/sensor_scan", "local_plan/scan_gen"),
    ("base_autonomy/visualization", "local_plan/viz"),
    ("global_planning/PCT_planner/tomography", "global_plan/tomography"),
    ("global_planning/PCT_planner/planner/config", "global_plan/config"),
    ("global_planning/PCT_planner/planner/scripts", "global_plan/planner"),
    ("global_planning/PCT_planner/rsc", "global_plan/resources"),
    ("global_planning/PCT_planner/launch", "global_plan/launch"),
    ("global_planning/pct_adapters", "global_plan/adapters"),
    ("global_planning/far_planner", "global_plan/far_planner"),
    ("global_planning/boundary_handler", "global_plan/boundary"),
    ("global_planning/graph_decoder", "global_plan/graph_decoder"),
    ("global_planning/visibility_graph", "global_plan/visibility"),
    ("semantic_perception/semantic_perception/api", "semantic/api"),
    ("semantic_perception/semantic_perception/impl", "semantic/impl"),
    ("semantic_perception/semantic_perception/storage", "semantic/storage"),
    ("semantic_perception/semantic_perception/belief", "semantic/belief"),
    ("semantic_perception/semantic_perception/bpu", "semantic/detectors_bpu"),
    ("semantic_perception/semantic_perception/yolo", "semantic/detectors"),
    ("semantic_perception/semantic_perception/grounding", "semantic/detectors"),
    ("semantic_perception/semantic_perception/clip", "semantic/encoders"),
    ("semantic_perception/semantic_perception/mobileclip", "semantic/encoders"),
    ("semantic_perception/semantic_perception/scg_", "semantic/scene_graph"),
    ("semantic_perception/semantic_perception/topology", "semantic/topology"),
    ("semantic_perception/semantic_perception/knowledge", "semantic/knowledge"),
    ("semantic_perception/semantic_perception/perception_node", "semantic/ros2_shell"),
    ("semantic_perception/semantic_perception/perception_pipeline", "semantic/pipeline"),
    ("semantic_perception/semantic_perception/perception_publisher", "semantic/ros2_shell"),
    ("semantic_perception/semantic_perception", "semantic/core"),
    ("semantic_perception/examples", "semantic/examples"),
    ("semantic_planner/semantic_planner/planner_node", "semantic_planner/ros2_shell"),
    ("semantic_planner/semantic_planner/init_mixin", "semantic_planner/ros2_shell"),
    ("semantic_planner/semantic_planner/callbacks_mixin", "semantic_planner/ros2_shell"),
    ("semantic_planner/semantic_planner/nav2_mixin", "semantic_planner/ros2_shell"),
    ("semantic_planner/semantic_planner/llm_client", "llm/client"),
    ("semantic_planner/semantic_planner/prompt_template", "llm/prompts"),
    ("semantic_planner/semantic_planner/chinese_tokenizer", "llm/tokenizer"),
    ("semantic_planner/semantic_planner/agent_node", "llm/agent"),
    ("semantic_planner/semantic_planner/mcp_server", "llm/mcp"),
    ("semantic_planner/semantic_planner/adacot", "llm/reasoning"),
    ("semantic_planner/semantic_planner/sgnav_reasoner", "llm/reasoning"),
    ("semantic_planner/semantic_planner/topological_memory", "memory/topological"),
    ("semantic_planner/semantic_planner/episodic_memory", "memory/episodic"),
    ("semantic_planner/semantic_planner/tagged_location", "memory/tagged"),
    ("semantic_planner/semantic_planner/goal_resolver", "semantic_planner/goal_resolver"),
    ("semantic_planner/semantic_planner/fast_path", "semantic_planner/fast_path"),
    ("semantic_planner/semantic_planner/slow_path", "semantic_planner/slow_path"),
    ("semantic_planner/semantic_planner/task_decomposer", "semantic_planner/task_decomposer"),
    ("semantic_planner/semantic_planner/task_rules", "semantic_planner/task_rules"),
    ("semantic_planner/semantic_planner/action_executor", "semantic_planner/action_executor"),
    ("semantic_planner/semantic_planner/frontier", "semantic_planner/frontier"),
    ("semantic_planner/semantic_planner/exploration", "semantic_planner/exploration"),
    ("semantic_planner/semantic_planner/person_tracker", "semantic_planner/person_tracker"),
    ("semantic_planner/semantic_planner/bbox", "semantic_planner/bbox_nav"),
    ("semantic_planner/semantic_planner/vlm_bbox", "semantic_planner/vlm_nav"),
    ("semantic_planner/semantic_planner/implicit_fsm", "semantic_planner/fsm_policy"),
    ("semantic_planner/semantic_planner/voi_scheduler", "semantic_planner/voi_scheduler"),
    ("semantic_planner/semantic_planner/semantic_prior", "semantic_planner/semantic_prior"),
    ("semantic_planner/semantic_planner/skill_registry", "semantic_planner/skill_registry"),
    ("semantic_planner/semantic_planner/room_object_kg", "semantic_planner/room_kg"),
    ("semantic_planner/semantic_planner/rerun_viewer", "semantic_planner/viz"),
    ("semantic_planner/semantic_planner", "semantic_planner/core"),
    ("nav_rings", "interaction/rings"),
    ("nav_services", "services"),
    ("reconstruction", "reconstruction"),
    ("remote_monitoring", "interaction/grpc_gateway"),
    ("semantic_common", "common"),
    ("transport", "transport"),
    ("nav_core", "nav_core"),
    ("ota_daemon", "ota"),
    ("utils/serial", "hardware/serial"),
]


def should_skip(path: Path) -> bool:
    for part in path.parts:
        if part in SKIP_DIRS:
            return True
    return False


def detect_lang(path: Path) -> str:
    name = path.name
    if name.endswith(".launch.py"):
        return "Launch"
    for ext, lang in LANG_MAP.items():
        if name.endswith(ext):
            return lang
    return "Other"


def count_lines(path: Path) -> int:
    try:
        return sum(1 for _ in open(path, encoding="utf-8", errors="ignore"))
    except Exception:
        return 0


def detect_ros2(path: Path, lang: str) -> bool:
    if lang not in ("Python", "C++", "Launch"):
        return False
    try:
        content = path.read_text(encoding="utf-8", errors="ignore")
    except Exception:
        return False
    markers = ROS2_MARKERS_PY if lang == "Python" else ROS2_MARKERS_CPP
    if lang == "Launch":
        return True  # launch files are always ROS2
    return any(m in content for m in markers)


def extract_imports(path: Path, lang: str) -> list:
    if lang != "Python":
        return []
    imports = []
    try:
        for line in open(path, encoding="utf-8", errors="ignore"):
            line = line.strip()
            if line.startswith("from ") or line.startswith("import "):
                # 提取顶层模块
                m = re.match(r"(?:from|import)\s+([\w.]+)", line)
                if m:
                    top = m.group(1).split(".")[0]
                    if top not in ("__future__", "os", "sys", "time", "json",
                                   "math", "re", "typing", "dataclasses",
                                   "collections", "pathlib", "threading",
                                   "logging", "enum", "abc", "functools",
                                   "itertools", "copy", "struct", "tempfile",
                                   "unittest", "pytest", "hashlib", "io",
                                   "asyncio", "traceback", "inspect",
                                   "signal", "subprocess", "shutil",
                                   "argparse", "textwrap", "datetime",
                                   "socket", "http", "urllib", "uuid",
                                   "warnings", "contextlib", "heapq"):
                        imports.append(top)
    except Exception:
        pass
    return sorted(set(imports))


def categorize(rel_path: str) -> str:
    rel_norm = rel_path.replace("\\", "/")
    for pattern, category in CATEGORY_RULES:
        if pattern in rel_norm:
            return category
    return "uncategorized"


def migration_type(lang: str, has_ros2: bool, category: str) -> str:
    if "ros2_shell" in category:
        return "不迁移 (ROS2 壳)"
    if "launch" in category.lower() or lang == "Launch":
        return "不迁移 (Launch)"
    if "viz" in category or "examples" in category:
        return "不迁移 (工具/示例)"
    if lang in ("C++", "C"):
        return "ProcessModule (C++子进程)"
    if lang == "Rust":
        return "ProcessModule (Rust子进程)"
    if lang == "Config":
        return "配置迁移"
    if lang == "Dart":
        return "不涉及 (客户端)"
    if has_ros2 and lang == "Python":
        return "需解耦 (含ROS2依赖)"
    if lang == "Python":
        return "直接迁移 (纯算法)"
    return "待定"


def main():
    files = []
    for root, dirs, fnames in os.walk(SRC):
        # 过滤跳过目录
        dirs[:] = [d for d in dirs if d not in SKIP_DIRS]
        for fn in sorted(fnames):
            fp = Path(root) / fn
            if should_skip(fp):
                continue
            lang = detect_lang(fp)
            if lang == "Other":
                continue
            rel = str(fp.relative_to(SRC))
            lines = count_lines(fp)
            has_ros2 = detect_ros2(fp, lang)
            imports = extract_imports(fp, lang)
            cat = categorize(rel)
            mtype = migration_type(lang, has_ros2, cat)
            files.append({
                "path": rel,
                "lang": lang,
                "lines": lines,
                "ros2": has_ros2,
                "imports": imports,
                "category": cat,
                "migration": mtype,
            })

    # 按 category 分组
    by_cat = defaultdict(list)
    for f in files:
        by_cat[f["category"]].append(f)

    # 统计
    total = len(files)
    by_lang = defaultdict(int)
    by_mtype = defaultdict(int)
    total_lines = 0
    for f in files:
        by_lang[f["lang"]] += 1
        by_mtype[f["migration"]] += 1
        total_lines += f["lines"]

    # 生成文档
    OUT.parent.mkdir(parents=True, exist_ok=True)
    with open(OUT, "w", encoding="utf-8") as w:
        w.write("# Hive 迁移映射 — src/ 完整扫描\n\n")
        w.write(f"> 自动生成, 共 **{total}** 个文件, **{total_lines:,}** 行代码\n\n")

        # 总览统计
        w.write("## 统计总览\n\n")
        w.write("### 按语言\n\n")
        w.write("| 语言 | 文件数 |\n|------|--------|\n")
        for lang, cnt in sorted(by_lang.items(), key=lambda x: -x[1]):
            w.write(f"| {lang} | {cnt} |\n")

        w.write("\n### 按迁移类型\n\n")
        w.write("| 迁移类型 | 文件数 |\n|----------|--------|\n")
        for mt, cnt in sorted(by_mtype.items(), key=lambda x: -x[1]):
            w.write(f"| {mt} | {cnt} |\n")

        # 按分类详情
        w.write("\n---\n\n## 按模块详情\n\n")
        for cat in sorted(by_cat.keys()):
            cat_files = by_cat[cat]
            cat_lines = sum(f["lines"] for f in cat_files)
            w.write(f"### `{cat}` ({len(cat_files)} 文件, {cat_lines:,} 行)\n\n")
            w.write("| 文件 | 语言 | 行数 | ROS2 | 迁移类型 | 关键依赖 |\n")
            w.write("|------|------|------|------|----------|----------|\n")
            for f in sorted(cat_files, key=lambda x: x["path"]):
                ros2_flag = "✓" if f["ros2"] else ""
                deps = ", ".join(f["imports"][:5]) if f["imports"] else ""
                name = Path(f["path"]).name
                w.write(f"| `{name}` | {f['lang']} | {f['lines']} | {ros2_flag} | {f['migration']} | {deps} |\n")
            w.write("\n")

        # sdk/modules 映射建议
        w.write("---\n\n## sdk/modules/ 目标结构建议\n\n")
        w.write("```\n")
        w.write("sdk/modules/\n")
        seen_cats = set()
        for cat in sorted(by_cat.keys()):
            top = cat.split("/")[0]
            if top not in seen_cats:
                seen_cats.add(top)
                sub_cats = [c for c in sorted(by_cat.keys()) if c.startswith(top + "/") or c == top]
                w.write(f"├── {top}/\n")
                for sc in sub_cats:
                    sub = sc.replace(top + "/", "") if "/" in sc else "(root)"
                    cnt = len(by_cat[sc])
                    w.write(f"│   ├── {sub}/ ({cnt} files)\n")
        w.write("```\n")

    print(f"Done: {OUT}")
    print(f"  {total} files, {total_lines:,} lines")
    print(f"  Languages: {dict(by_lang)}")
    print(f"  Migration: {dict(by_mtype)}")


if __name__ == "__main__":
    main()
