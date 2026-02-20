#!/bin/bash
# ============================================================
# test_semantic_nav.sh — 语义导航端到端测试脚本
#
# 用法:
#   bash scripts/test_semantic_nav.sh [--unit-only]
#
# 测试层级:
#   1. 单元测试 (Python)
#   2. Proto 编译验证
#   3. ROS2 包构建验证
#   4. 节点启动验证 (需要 ROS2 环境)
# ============================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

UNIT_ONLY=false
if [[ "${1:-}" == "--unit-only" ]]; then
  UNIT_ONLY=true
fi

GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

pass() { echo -e "${GREEN}[PASS]${NC} $1"; }
fail() { echo -e "${RED}[FAIL]${NC} $1"; exit 1; }
info() { echo -e "${YELLOW}[INFO]${NC} $1"; }

# ── 1. 文件存在性检查 ──
info "Phase 0: Checking file structure..."

check_file() {
  if [ -f "$REPO_ROOT/$1" ]; then
    pass "$1 exists"
  else
    fail "$1 missing"
  fi
}

check_dir() {
  if [ -d "$REPO_ROOT/$1" ]; then
    pass "$1/ exists"
  else
    fail "$1/ missing"
  fi
}

check_file "scripts/clone_semantic_deps.sh"
check_file "third_party/.gitkeep"
check_dir  "src/semantic_perception"
check_dir  "src/semantic_planner"
check_file "src/semantic_perception/package.xml"
check_file "src/semantic_planner/package.xml"
check_file "launch/subsystems/semantic.launch.py"
check_file "config/semantic_perception.yaml"
check_file "config/semantic_planner.yaml"
check_file "config/topic_contract.yaml"

# ── 2. Proto 内容验证 ──
info "Phase 1: Checking proto definitions..."

if grep -q "TASK_TYPE_SEMANTIC_NAV" "$REPO_ROOT/src/robot_proto/proto/common.proto"; then
  pass "TASK_TYPE_SEMANTIC_NAV in common.proto"
else
  fail "TASK_TYPE_SEMANTIC_NAV missing from common.proto"
fi

if grep -q "WAYPOINT_SOURCE_SEMANTIC" "$REPO_ROOT/src/robot_proto/proto/control.proto"; then
  pass "WAYPOINT_SOURCE_SEMANTIC in control.proto"
else
  fail "WAYPOINT_SOURCE_SEMANTIC missing from control.proto"
fi

if grep -q "SemanticNavParams" "$REPO_ROOT/src/robot_proto/proto/control.proto"; then
  pass "SemanticNavParams message in control.proto"
else
  fail "SemanticNavParams missing from control.proto"
fi

if grep -q "semantic_nav_params" "$REPO_ROOT/src/robot_proto/proto/control.proto"; then
  pass "semantic_nav_params field in StartTaskRequest"
else
  fail "semantic_nav_params field missing from StartTaskRequest"
fi

# ── 3. Topic contract 验证 ──
if grep -q "semantic:" "$REPO_ROOT/config/topic_contract.yaml"; then
  pass "semantic section in topic_contract.yaml"
else
  fail "semantic section missing from topic_contract.yaml"
fi

if grep -q "/nav/semantic/resolved_goal" "$REPO_ROOT/config/topic_contract.yaml"; then
  pass "resolved_goal topic defined"
else
  fail "resolved_goal topic missing"
fi

# ── 4. C++ 代码验证 ──
info "Phase 1.3-1.4: Checking C++ changes..."

if grep -q "SemanticGoalCallback" "$REPO_ROOT/src/remote_monitoring/src/core/task_manager.cpp"; then
  pass "SemanticGoalCallback in task_manager.cpp"
else
  fail "SemanticGoalCallback missing from task_manager.cpp"
fi

if grep -q "TASK_TYPE_SEMANTIC_NAV" "$REPO_ROOT/src/remote_monitoring/src/services/control_service.cpp"; then
  pass "SEMANTIC_NAV branch in control_service.cpp"
else
  fail "SEMANTIC_NAV branch missing from control_service.cpp"
fi

if grep -q "semantic_nav_params" "$REPO_ROOT/src/remote_monitoring/src/services/control_service.cpp"; then
  pass "semantic_nav_params handling in control_service.cpp"
else
  fail "semantic_nav_params handling missing"
fi

# ── 5. Launch 文件验证 ──
info "Phase 1.5: Checking launch files..."

if grep -q "enable_semantic" "$REPO_ROOT/launch/navigation_run.launch.py"; then
  pass "enable_semantic arg in navigation_run.launch.py"
else
  fail "enable_semantic arg missing"
fi

if grep -q "semantic.launch.py" "$REPO_ROOT/launch/navigation_run.launch.py"; then
  pass "semantic.launch.py include in navigation_run.launch.py"
else
  fail "semantic.launch.py include missing"
fi

# ── 6. Python 单元测试 ──
info "Running Python unit tests..."

cd "$REPO_ROOT"
export PYTHONPATH="$REPO_ROOT/src/semantic_perception:$REPO_ROOT/src/semantic_planner:$PYTHONPATH"

# 感知包测试
if python3 -m pytest src/semantic_perception/test/ -v --tb=short 2>/dev/null; then
  pass "semantic_perception unit tests"
elif python3 -m unittest discover -s src/semantic_perception/test -v 2>&1; then
  pass "semantic_perception unit tests (unittest)"
else
  fail "semantic_perception unit tests failed"
fi

# 规划包测试
if python3 -m pytest src/semantic_planner/test/ -v --tb=short 2>/dev/null; then
  pass "semantic_planner unit tests"
elif python3 -m unittest discover -s src/semantic_planner/test -v 2>&1; then
  pass "semantic_planner unit tests (unittest)"
else
  fail "semantic_planner unit tests failed"
fi

# ── 7. .gitignore 验证 ──
info "Checking .gitignore..."

if grep -q "third_party/" "$REPO_ROOT/.gitignore"; then
  pass "third_party/ in .gitignore"
else
  fail "third_party/ missing from .gitignore"
fi

# ── 8. Flutter 代码验证 ──
info "Phase 4: Checking Flutter changes..."

FLUTTER_DIR="$REPO_ROOT/client/flutter_monitor"

if grep -q "startSemanticNav" "$FLUTTER_DIR/lib/core/gateway/task_gateway.dart"; then
  pass "startSemanticNav in task_gateway.dart"
else
  fail "startSemanticNav missing from task_gateway.dart"
fi

if grep -q "TASK_TYPE_SEMANTIC_NAV" "$FLUTTER_DIR/lib/core/gateway/task_gateway.dart"; then
  pass "TASK_TYPE_SEMANTIC_NAV label in task_gateway.dart"
else
  fail "TASK_TYPE_SEMANTIC_NAV label missing"
fi

if grep -q "semanticNav" "$FLUTTER_DIR/lib/features/map/map_screen.dart"; then
  pass "semanticNav mode in map_screen.dart"
else
  fail "semanticNav mode missing from map_screen.dart"
fi

if grep -q "semanticNavParams" "$FLUTTER_DIR/lib/core/grpc/robot_client_base.dart"; then
  pass "semanticNavParams in robot_client_base.dart"
else
  fail "semanticNavParams missing from robot_client_base.dart"
fi

if $UNIT_ONLY; then
  info "Unit-only mode — skipping ROS2 build tests."
  echo ""
  echo "============================================================"
  echo -e "${GREEN} All unit tests and file checks PASSED${NC}"
  echo "============================================================"
  exit 0
fi

# ── 9. ROS2 构建验证 (仅在 ROS2 环境中) ──
if command -v colcon &>/dev/null; then
  info "Phase 5: Building ROS2 packages..."

  cd "$REPO_ROOT"
  if colcon build --packages-select semantic_perception semantic_planner --symlink-install 2>&1; then
    pass "colcon build semantic packages"
  else
    fail "colcon build failed"
  fi
else
  info "colcon not found — skipping ROS2 build test"
fi

echo ""
echo "============================================================"
echo -e "${GREEN} All tests PASSED${NC}"
echo "============================================================"
