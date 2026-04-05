#!/usr/bin/env bash
# 在 Linux 上验证 LingTu 运行环境并跑「可全量」的 Python 测试套件。
# 用法: bash scripts/deploy/verify_linux_full.sh [LINGTU_ROOT]
# 依赖: Python3.10+, pytest, 仓库内 src 在 PYTHONPATH 上

set -euo pipefail

LT="${1:-$(cd "$(dirname "$0")/../.." && pwd)}"
cd "$LT"

echo "========== LingTu Linux 全量验证 =========="
echo "ROOT=$LT"
echo ""

echo "========== 1) 系统 / Python / ROS =========="
uname -a
echo "Python: $(python3 --version 2>&1)"
if command -v ros2 >/dev/null 2>&1; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash 2>/dev/null || true
  ros2 --version 2>&1 || true
else
  echo "ros2: not in PATH (OK for pure-Python tests)"
fi

echo ""
echo "========== 2) 关键 Python 包 =========="
python3 <<'PY'
import importlib
mods = ["numpy", "scipy", "mujoco", "onnxruntime", "pytest", "cv2"]
for m in mods:
    try:
        x = importlib.import_module(m)
        v = getattr(x, "__version__", "?")
        print(f"  OK {m}: {v}")
    except Exception as e:
        print(f"  -- {m}: MISSING ({e})")
try:
    import torch
    print(f"  OK torch: {torch.__version__}")
except Exception as e:
    print(f"  -- torch: optional ({e})")
PY

echo ""
echo "========== 3) colcon =========="
if [[ -f "$LT/install/setup.bash" ]]; then
  echo "  install/setup.bash present — 可在此机执行: make test"
else
  echo "  无 install/setup.bash — 跳过 colcon test（需先 colcon build）"
fi

export PYTHONPATH="$LT/src:$LT:${PYTHONPATH:-}"

echo ""
echo "========== 4) pytest src/core/tests =========="
# test_cross_module_integration.py 为脚本式集成测试（导入即执行并 sys.exit），勿交给 pytest 收集
python3 -m pytest "$LT/src/core/tests" \
  --ignore="$LT/src/core/tests/test_cross_module_integration.py" \
  -q --tb=short

echo ""
echo "========== 4b) 脚本式集成 test_cross_module_integration.py =========="
python3 "$LT/src/core/tests/test_cross_module_integration.py"

echo ""
echo "========== 5) pytest tests/planning/test_pct_adapter_logic.py =========="
python3 -m pytest "$LT/tests/planning/test_pct_adapter_logic.py" -q --tb=short

echo ""
echo "========== 6) e2e_factory_nav (3D MuJoCo 场景) =========="
python3 "$LT/tests/planning/e2e_factory_nav.py"

echo ""
echo "========== 全部完成 =========="
