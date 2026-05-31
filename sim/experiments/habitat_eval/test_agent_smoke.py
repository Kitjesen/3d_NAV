"""
NaviMind Agent 冒烟测试 — 不需要 Habitat 环境/数据集。

验证:
1. 所有依赖能 import
2. Agent 能处理模拟观测
3. Fast Path 能匹配目标
4. 探索策略正常工作
5. 消融开关正常
"""

import sys
import json
import math
from pathlib import Path

# 加入 src 路径
_PROJECT_ROOT = Path(__file__).resolve().parents[2]
for _pkg in ["src/semantic_perception", "src/semantic_planner", "src/semantic_common"]:
    _p = str(_PROJECT_ROOT / _pkg)
    if _p not in sys.path:
        sys.path.insert(0, _p)

import numpy as np

print("=" * 60)
print("  NaviMind Agent 冒烟测试")
print("=" * 60)

# Test 1: Import
print("\n[1/5] 依赖 import...")
from semantic.common.semantic_common import sanitize_position, safe_json_loads
from semantic.perception.semantic_perception.projection import Detection3D
from semantic.perception.semantic_perception.instance_tracker import InstanceTracker
from semantic.planner.semantic_planner.goal_resolver import GoalResolver, GoalResult
from semantic.planner.semantic_planner.llm_client import LLMConfig
from habitat_navimind_agent import NaviMindAgent, OBJECTNAV_CATEGORIES, _normalize_category
print("  ✓ 所有依赖 import 成功")

# Test 2: Agent 初始化
print("\n[2/5] Agent 初始化...")
config = {
    "sensor": {"rgb": {"width": 64, "height": 64, "hfov": 79}},
    "eval": {"max_steps": 50, "success_distance": 1.0},
    "scene_graph": {"max_objects": 50},
    "fast_path": {"threshold": 0.75},
    "exploration": {"rotate_steps": 12, "frontier_distance": 2.0},
    "ablation": {"name": "full"},
}
agent = NaviMindAgent(config)
agent.reset()
agent.set_goal("chair")
print(f"  ✓ Agent 初始化成功, 目标: chair → '{agent._instruction}'")

# Test 3: 模拟观测处理
print("\n[3/5] 模拟观测处理...")
fake_obs = {
    "rgb": np.random.randint(0, 255, (64, 64, 3), dtype=np.uint8),
    "depth": np.random.uniform(0.5, 5.0, (64, 64, 1)).astype(np.float32),
    "semantic": np.zeros((64, 64, 1), dtype=np.int32),
    "gps": np.array([1.0, 2.0], dtype=np.float32),
    "compass": np.array([0.5], dtype=np.float32),
    "_semantic_categories": {1: "chair", 2: "table"},
}
# 在语义图中放一些 chair 像素
fake_obs["semantic"][20:40, 20:40, 0] = 1  # chair instance

action = agent.act(fake_obs)
assert action in [0, 1, 2, 3], f"无效动作: {action}"
print(f"  ✓ act() 返回动作: {action}")
print(f"  ✓ 场景图物体数: {len(agent._tracker._objects)}")
print(f"  ✓ 统计: {agent.stats}")

# Test 4: 多步运行
print("\n[4/5] 多步运行...")
for step in range(10):
    # 移动一下
    fake_obs["gps"] = np.array([1.0 + step * 0.25, 2.0], dtype=np.float32)
    fake_obs["compass"] = np.array([0.5 + step * 0.1], dtype=np.float32)
    action = agent.act(fake_obs)

stats = agent.stats
print(f"  ✓ 11 步完成, 动作正常")
print(f"  ✓ Fast Path hits: {stats['fast_path_hits']}/{stats['total_resolves']}")
print(f"  ✓ 访问网格: {stats['visited_cells']} cells")

# Test 5: 消融变体
print("\n[5/5] 消融变体测试...")
ablations = ["full", "no_belief", "no_fov", "no_hierarchy", "always_llm"]
for abl in ablations:
    cfg = dict(config)
    cfg["ablation"] = {"name": abl}
    a = NaviMindAgent(cfg)
    a.reset()
    a.set_goal("bed")
    action = a.act(fake_obs)
    assert action in [0, 1, 2, 3]
    print(f"  ✓ {abl:<15} → 动作={action}, ablation={a.stats['ablation']}")

print("\n" + "=" * 60)
print("  全部冒烟测试通过!")
print("=" * 60)
