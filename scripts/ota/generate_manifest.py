#!/usr/bin/env python3
"""
generate_manifest.py - 为 GitHub Release 生成 OTA manifest.json

用法:
    python3 generate_manifest.py \
        --version v2.3.0 \
        --artifacts-dir ./dist/ \
        --template manifest_template.json \
        --output ./dist/manifest.json

此脚本会:
1. 读取模板文件
2. 扫描 artifacts-dir 中的文件
3. 自动计算 SHA256
4. 更新版本号
5. 输出 manifest.json
"""

import argparse
import hashlib
import json
import os
import sys
from datetime import datetime, timezone


def compute_sha256(filepath: str) -> str:
    """计算文件 SHA256"""
    sha256 = hashlib.sha256()
    with open(filepath, "rb") as f:
        for chunk in iter(lambda: f.read(8192), b""):
            sha256.update(chunk)
    return sha256.hexdigest()


def infer_category(filename: str) -> str:
    """从文件扩展名推断分类"""
    ext = filename.rsplit(".", 1)[-1].lower() if "." in filename else ""
    if ext in ("pt", "pth", "onnx", "tflite", "engine"):
        return "model"
    elif ext in ("pcd", "ply", "pickle", "pgm"):
        return "map"
    elif ext in ("yaml", "yml", "json", "xml", "cfg", "ini"):
        return "config"
    elif ext in ("bin", "hex", "img", "deb"):
        return "firmware"
    return "other"


def main():
    parser = argparse.ArgumentParser(description="Generate OTA manifest.json")
    parser.add_argument("--version", required=True, help="Release version (e.g. v2.3.0)")
    parser.add_argument("--artifacts-dir", required=True, help="Directory containing artifacts")
    parser.add_argument("--template", default=None, help="Template manifest.json (optional)")
    parser.add_argument("--output", default="manifest.json", help="Output path")
    args = parser.parse_args()

    artifacts_dir = args.artifacts_dir
    if not os.path.isdir(artifacts_dir):
        print(f"ERROR: artifacts directory not found: {artifacts_dir}", file=sys.stderr)
        sys.exit(1)

    # 加载模板 (如有)
    template_artifacts = {}
    if args.template and os.path.isfile(args.template):
        with open(args.template) as f:
            tpl = json.load(f)
            for art in tpl.get("artifacts", []):
                template_artifacts[art.get("filename", "")] = art

    # 扫描文件
    artifacts = []
    for fname in sorted(os.listdir(artifacts_dir)):
        fpath = os.path.join(artifacts_dir, fname)
        if not os.path.isfile(fpath):
            continue
        if fname in ("manifest.json",):
            continue  # 跳过 manifest 自身

        sha256 = compute_sha256(fpath)
        size = os.path.getsize(fpath)

        # 优先从模板获取元数据
        if fname in template_artifacts:
            art = dict(template_artifacts[fname])
            art["sha256"] = sha256
            art["version"] = args.version.lstrip("v")
        else:
            # 自动推断
            category = infer_category(fname)
            name = fname.rsplit(".", 1)[0] if "." in fname else fname
            art = {
                "name": name,
                "category": category,
                "version": args.version.lstrip("v"),
                "filename": fname,
                "sha256": sha256,
                "target_path": f"/opt/robot/{category}s/{fname}",
                "target_board": "nav",
                "hw_compat": ["*"],
                "apply_action": "copy_only",
                "requires_reboot": False,
                "min_battery_percent": 0,
                "changelog": "",
                "rollback_safe": True,
            }

        artifacts.append(art)
        print(f"  {fname}: sha256={sha256[:16]}... category={art['category']}")

    manifest = {
        "schema_version": "1",
        "release_version": args.version,
        "release_date": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "min_system_version": "1.0.0",
        "artifacts": artifacts,
    }

    with open(args.output, "w") as f:
        json.dump(manifest, f, indent=2, ensure_ascii=False)

    print(f"\nManifest generated: {args.output}")
    print(f"  Version: {args.version}")
    print(f"  Artifacts: {len(artifacts)}")


if __name__ == "__main__":
    main()
