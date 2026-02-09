#!/usr/bin/env python3
"""
generate_manifest.py - 为 GitHub Release 生成 OTA manifest.json

用法:
    # 基本用法
    python3 generate_manifest.py \
        --version v2.3.0 \
        --artifacts-dir ./dist/ \
        --output ./dist/manifest.json

    # 带 Ed25519 签名 (产品级)
    python3 generate_manifest.py \
        --version v2.3.0 \
        --artifacts-dir ./dist/ \
        --signing-key ./keys/ota_private.pem \
        --key-id ota-signing-key-01 \
        --output ./dist/manifest.json

    # 生成密钥对 (首次)
    python3 generate_manifest.py --generate-keys --key-dir ./keys/

此脚本会:
1. 读取模板文件
2. 扫描 artifacts-dir 中的文件
3. 自动计算 SHA256
4. 更新版本号和 dependencies
5. 可选: 对 manifest 内容签名 (Ed25519)
6. 输出 manifest.json
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


def infer_safety_level(category: str, apply_action: str) -> str:
    """推断安全等级"""
    if apply_action in ("install_deb", "flash_mcu", "install_script"):
        return "cold"
    if category == "model" and apply_action == "reload_model":
        return "warm"
    return "hot"


def sign_manifest(manifest_content: str, private_key_path: str) -> str:
    """用 Ed25519 私钥签名 manifest JSON 内容 (不含 signature 字段)
    
    返回 hex 编码的签名
    """
    try:
        from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PrivateKey
        from cryptography.hazmat.primitives.serialization import load_pem_private_key
    except ImportError:
        print("WARNING: cryptography 库未安装, 跳过签名", file=sys.stderr)
        print("  安装: pip install cryptography", file=sys.stderr)
        return ""

    with open(private_key_path, "rb") as f:
        private_key = load_pem_private_key(f.read(), password=None)

    if not isinstance(private_key, Ed25519PrivateKey):
        print("ERROR: 密钥不是 Ed25519 类型", file=sys.stderr)
        sys.exit(1)

    signature = private_key.sign(manifest_content.encode("utf-8"))
    return signature.hex()


def generate_keys(key_dir: str):
    """生成 Ed25519 密钥对"""
    try:
        from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PrivateKey
        from cryptography.hazmat.primitives.serialization import (
            Encoding,
            NoEncryption,
            PrivateFormat,
            PublicFormat,
        )
    except ImportError:
        print("ERROR: 需要 cryptography 库: pip install cryptography", file=sys.stderr)
        sys.exit(1)

    os.makedirs(key_dir, exist_ok=True)
    private_key = Ed25519PrivateKey.generate()

    private_path = os.path.join(key_dir, "ota_private.pem")
    public_path = os.path.join(key_dir, "ota_public.pem")

    with open(private_path, "wb") as f:
        f.write(private_key.private_bytes(
            Encoding.PEM, PrivateFormat.PKCS8, NoEncryption()))
    os.chmod(private_path, 0o600)

    with open(public_path, "wb") as f:
        f.write(private_key.public_key().public_bytes(
            Encoding.PEM, PublicFormat.SubjectPublicKeyInfo))

    print(f"密钥已生成:")
    print(f"  私钥 (保密): {private_path}")
    print(f"  公钥 (部署到机器人): {public_path}")
    print(f"\n将公钥复制到机器人:")
    print(f"  scp {public_path} robot:/opt/robot/ota/ota_public.pem")


def generate_system_manifest(version: str, artifacts: list, output_path: str):
    """生成 system_manifest.json — 描述整机的完整组件版本集"""
    components = {}
    for art in artifacts:
        components[art.get("name", "unknown")] = {
            "version": art.get("version", "0.0.0"),
        }
        # Include git_commit if present
        if art.get("git_commit"):
            components[art["name"]]["git_commit"] = art["git_commit"]

    system_manifest = {
        "system_version": version.lstrip("v"),
        "build_time": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "components": components,
    }

    with open(output_path, "w") as f:
        json.dump(system_manifest, f, indent=2, ensure_ascii=False)
    print(f"  System manifest: {output_path}")


def main():
    parser = argparse.ArgumentParser(description="Generate OTA manifest.json")
    parser.add_argument("--version", help="Release version (e.g. v2.3.0)")
    parser.add_argument("--artifacts-dir", help="Directory containing artifacts")
    parser.add_argument("--template", default=None, help="Template manifest.json (optional)")
    parser.add_argument("--output", default="manifest.json", help="Output path")
    parser.add_argument("--signing-key", default=None, help="Ed25519 private key PEM for signing")
    parser.add_argument("--key-id", default="ota-signing-key-01", help="Public key ID")
    parser.add_argument("--generate-keys", action="store_true", help="Generate Ed25519 key pair")
    parser.add_argument("--key-dir", default="./keys", help="Directory for generated keys")
    parser.add_argument("--channel", default="stable",
                        choices=["dev", "canary", "stable"],
                        help="Release channel (default: stable)")
    parser.add_argument("--system-manifest", default=None,
                        help="Also generate system_manifest.json at this path")
    args = parser.parse_args()

    # 密钥生成模式
    if args.generate_keys:
        generate_keys(args.key_dir)
        return

    if not args.version or not args.artifacts_dir:
        parser.error("--version and --artifacts-dir are required")

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
                "safety_level": infer_safety_level(category, "copy_only"),
                "owner_module": "system",
                "dependencies": [],
            }

        artifacts.append(art)
        print(f"  {fname}: sha256={sha256[:16]}... category={art['category']} safety={art.get('safety_level', 'hot')}")

    manifest = {
        "schema_version": "2",
        "release_version": args.version,
        "release_date": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "channel": args.channel,
        "min_system_version": "1.0.0",
        "signature": "",
        "public_key_id": args.key_id,
        "artifacts": artifacts,
    }

    # 签名: 对去除 signature 字段后的 canonical JSON 签名
    if args.signing_key:
        # canonical: sorted keys, no extra whitespace
        manifest_for_signing = dict(manifest)
        manifest_for_signing.pop("signature", None)
        canonical = json.dumps(manifest_for_signing, sort_keys=True, separators=(",", ":"), ensure_ascii=False)
        signature = sign_manifest(canonical, args.signing_key)
        if signature:
            manifest["signature"] = signature
            print(f"  签名完成 (key_id={args.key_id})")

    with open(args.output, "w") as f:
        json.dump(manifest, f, indent=2, ensure_ascii=False)

    # Generate system_manifest.json if requested
    if args.system_manifest:
        generate_system_manifest(args.version, artifacts, args.system_manifest)
    else:
        # Auto-generate next to manifest.json
        output_dir = os.path.dirname(args.output) or "."
        sys_manifest_path = os.path.join(output_dir, "system_manifest.json")
        generate_system_manifest(args.version, artifacts, sys_manifest_path)

    print(f"\nManifest generated: {args.output}")
    print(f"  Schema: v{manifest['schema_version']}")
    print(f"  Version: {args.version}")
    print(f"  Channel: {args.channel}")
    print(f"  Artifacts: {len(artifacts)}")
    print(f"  Signed: {'Yes' if manifest.get('signature') else 'No'}")


if __name__ == "__main__":
    main()
