"""
拓扑语义图 (TSG) + LLM 端到端测试。

测试内容:
  1. 带拓扑信息的 GoalResolution — LLM 如何利用房间连通关系推理
  2. TSG 信息增益探索 vs LLM 探索 — 双层策略对比
  3. 探索记忆和穿越历史 — 负面记忆如何影响下一步决策
"""
import asyncio
import io
import json
import os
import sys
import time
from pathlib import Path

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src" / "semantic_planner"))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src" / "semantic_perception"))

from semantic_planner.llm_client import LLMConfig, create_llm_client
from semantic_planner.goal_resolver import GoalResolver
from semantic_planner.prompt_templates import (
    build_goal_resolution_prompt,
    build_exploration_prompt,
)
from semantic_planner.semantic_prior import SemanticPriorEngine
from semantic_perception.topology_graph import TopologySemGraph

import numpy as np

# ══════════════════════════════════════════════════════════════
#  场景: 5 房间办公楼, 含拓扑连通
# ══════════════════════════════════════════════════════════════

SCENE_GRAPH = {
    "summary": (
        "Office building: 5 rooms (corridor, office_A, office_B, kitchen, stairwell), "
        "20 objects, 6 topology edges, 2 frontier nodes."
    ),
    "rooms": [
        {
            "room_id": 0, "name": "corridor",
            "center": {"x": 0.0, "y": 0.0},
            "object_ids": [1, 2, 3, 4],
            "group_ids": [0, 1],
            "semantic_labels": ["door", "fire extinguisher", "sign", "trash can"],
        },
        {
            "room_id": 1, "name": "office_A",
            "center": {"x": 5.0, "y": 2.0},
            "object_ids": [5, 6, 7, 8],
            "group_ids": [2],
            "semantic_labels": ["desk", "chair", "monitor", "keyboard"],
        },
        {
            "room_id": 2, "name": "office_B",
            "center": {"x": 5.0, "y": -2.0},
            "object_ids": [9, 10, 11],
            "group_ids": [3],
            "semantic_labels": ["desk", "chair", "printer"],
        },
        {
            "room_id": 3, "name": "kitchen",
            "center": {"x": -4.0, "y": 3.0},
            "object_ids": [12, 13, 14],
            "group_ids": [4],
            "semantic_labels": ["refrigerator", "sink", "microwave"],
        },
        {
            "room_id": 4, "name": "stairwell",
            "center": {"x": -4.0, "y": -3.0},
            "object_ids": [15, 16],
            "group_ids": [5],
            "semantic_labels": ["stairs", "railing"],
        },
    ],
    "topology_edges": [
        {"from_room": 0, "to_room": 1, "type": "door", "mediator": "door",
         "mediator_pos": {"x": 2.5, "y": 1.0}, "distance": 5.4},
        {"from_room": 0, "to_room": 2, "type": "door", "mediator": "door",
         "mediator_pos": {"x": 2.5, "y": -1.0}, "distance": 5.4},
        {"from_room": 0, "to_room": 3, "type": "passage", "mediator": "corridor",
         "distance": 5.0},
        {"from_room": 0, "to_room": 4, "type": "passage", "mediator": "corridor",
         "distance": 5.0},
        {"from_room": 3, "to_room": 4, "type": "proximity", "distance": 6.0},
        {"from_room": 1, "to_room": 2, "type": "proximity", "distance": 4.0},
    ],
    "frontier_nodes": [
        {
            "position": {"x": 8.0, "y": 2.0},
            "direction": {"dx": 1.0, "dy": 0.0},
            "nearest_room_id": 1,
            "frontier_size": 3.0,
            "source": "door_outward",
        },
        {
            "position": {"x": -7.0, "y": 0.0},
            "direction": {"dx": -1.0, "dy": 0.0},
            "nearest_room_id": 3,
            "frontier_size": 2.5,
            "source": "sparse_sector",
        },
    ],
    "groups": [
        {"group_id": 0, "room_id": 0, "name": "structure",
         "center": {"x": 0.0, "y": 0.0}, "object_ids": [1, 3]},
        {"group_id": 1, "room_id": 0, "name": "safety",
         "center": {"x": 0.5, "y": 0.5}, "object_ids": [2, 4]},
        {"group_id": 2, "room_id": 1, "name": "workstation",
         "center": {"x": 5.0, "y": 2.0}, "object_ids": [5, 6, 7, 8]},
        {"group_id": 3, "room_id": 2, "name": "workstation",
         "center": {"x": 5.0, "y": -2.0}, "object_ids": [9, 10, 11]},
        {"group_id": 4, "room_id": 3, "name": "utility",
         "center": {"x": -4.0, "y": 3.0}, "object_ids": [12, 13, 14]},
        {"group_id": 5, "room_id": 4, "name": "structure",
         "center": {"x": -4.0, "y": -3.0}, "object_ids": [15, 16]},
    ],
    "objects": [
        {"id": 1,  "label": "door",              "position": {"x": 0.0, "y": 0.0, "z": 0.0}, "score": 0.95, "detection_count": 10, "region_id": 0},
        {"id": 2,  "label": "fire extinguisher",  "position": {"x": 0.8, "y": 0.5, "z": 0.8}, "score": 0.92, "detection_count": 5,  "region_id": 0},
        {"id": 3,  "label": "exit sign",          "position": {"x": 1.5, "y": -0.3, "z": 2.5}, "score": 0.88, "detection_count": 3,  "region_id": 0},
        {"id": 4,  "label": "trash can",          "position": {"x": -0.5, "y": 0.8, "z": 0.0}, "score": 0.85, "detection_count": 4,  "region_id": 0},
        {"id": 5,  "label": "desk",               "position": {"x": 5.2, "y": 2.3, "z": 0.0}, "score": 0.97, "detection_count": 12, "region_id": 1},
        {"id": 6,  "label": "chair",              "position": {"x": 4.8, "y": 2.0, "z": 0.0}, "score": 0.93, "detection_count": 8,  "region_id": 1},
        {"id": 7,  "label": "monitor",            "position": {"x": 5.3, "y": 2.3, "z": 0.7}, "score": 0.91, "detection_count": 7,  "region_id": 1},
        {"id": 8,  "label": "keyboard",           "position": {"x": 5.1, "y": 2.1, "z": 0.6}, "score": 0.89, "detection_count": 5,  "region_id": 1},
        {"id": 9,  "label": "desk",               "position": {"x": 5.0, "y": -2.2, "z": 0.0}, "score": 0.96, "detection_count": 10, "region_id": 2},
        {"id": 10, "label": "chair",              "position": {"x": 4.7, "y": -1.8, "z": 0.0}, "score": 0.90, "detection_count": 6,  "region_id": 2},
        {"id": 11, "label": "printer",            "position": {"x": 5.5, "y": -2.5, "z": 0.0}, "score": 0.87, "detection_count": 4,  "region_id": 2},
        {"id": 12, "label": "refrigerator",       "position": {"x": -4.2, "y": 3.5, "z": 0.0}, "score": 0.90, "detection_count": 5,  "region_id": 3},
        {"id": 13, "label": "sink",               "position": {"x": -3.8, "y": 2.8, "z": 0.8}, "score": 0.88, "detection_count": 4,  "region_id": 3},
        {"id": 14, "label": "microwave",          "position": {"x": -4.5, "y": 3.2, "z": 1.0}, "score": 0.85, "detection_count": 3,  "region_id": 3},
        {"id": 15, "label": "stairs",             "position": {"x": -4.0, "y": -3.0, "z": 0.0}, "score": 0.93, "detection_count": 6,  "region_id": 4},
        {"id": 16, "label": "railing",            "position": {"x": -3.8, "y": -3.2, "z": 0.8}, "score": 0.80, "detection_count": 3,  "region_id": 4},
    ],
    "relations": [
        {"subject_id": 2, "object_id": 1, "relation": "near", "distance": 1.0},
        {"subject_id": 6, "object_id": 5, "relation": "near", "distance": 0.5},
        {"subject_id": 7, "object_id": 5, "relation": "on"},
        {"subject_id": 8, "object_id": 5, "relation": "on"},
        {"subject_id": 10, "object_id": 9, "relation": "near", "distance": 0.5},
        {"subject_id": 11, "object_id": 9, "relation": "near", "distance": 0.8},
        {"subject_id": 12, "object_id": 13, "relation": "near", "distance": 0.8},
        {"subject_id": 15, "object_id": 16, "relation": "near", "distance": 0.3},
    ],
}

SG_JSON = json.dumps(SCENE_GRAPH, ensure_ascii=False)
ROBOT_POS = {"x": 0.0, "y": 0.0, "z": 0.0}


def make_config():
    return LLMConfig(
        backend="openai", model="kimi-k2.5",
        api_key_env="MOONSHOT_API_KEY", timeout_sec=120.0,
        max_retries=2, temperature=1.0,
        base_url="https://api.moonshot.cn/v1",
    )


def sep(char="=", w=76):
    print(char * w)


# ══════════════════════════════════════════════════════════════
#  Test 1: 带拓扑的目标解析 (LLM 看到拓扑连通关系)
# ══════════════════════════════════════════════════════════════

async def test_goal_resolution_with_topology(client, resolver):
    """测试 LLM 是否利用拓扑连通关系进行推理。"""
    sep()
    print("  TEST 1: 带拓扑信息的目标解析")
    sep()

    tests = [
        ("找到走廊里门旁边的灭火器", "zh",
         "中文+拓扑: LLM应通过Room→Group→Object推理"),
        ("find the printer in office B", "en",
         "英文+拓扑: LLM应识别office_B并选择printer"),
        ("去厨房拿杯水", "zh",
         "中文+拓扑+探索: kitchen有sink但没有cup, 应explore或navigate到sink"),
    ]

    for instruction, lang, desc in tests:
        print(f"\n  [{desc}]")
        print(f"  Instruction: {instruction}")

        # Fast Path
        t0 = time.perf_counter()
        fast = resolver.fast_resolve(instruction, SG_JSON, ROBOT_POS)
        fast_ms = (time.perf_counter() - t0) * 1000

        if fast is not None:
            print(f"  Path: FAST ({fast_ms:.1f}ms)")
            print(f"  Result: {fast.action} → {fast.target_label} ({fast.target_x:.1f}, {fast.target_y:.1f})")
            print(f"  Confidence: {fast.confidence:.2f}")
            print(f"  Scoring: {fast.reasoning[:200]}")
            continue

        # Slow Path — LLM
        print(f"  Path: SLOW (calling Kimi-k2.5...)", end="", flush=True)
        messages = build_goal_resolution_prompt(instruction, SG_JSON, ROBOT_POS, lang)

        t0 = time.perf_counter()
        try:
            raw = await client.chat(messages)
        except Exception as e:
            raw = f"ERROR: {e}"
        ms = (time.perf_counter() - t0) * 1000
        print(f" {ms:.0f}ms")

        # 打印完整的 LLM 输入
        print(f"\n  --- LLM System Prompt (first 300 chars) ---")
        print(f"  {messages[0]['content'][:300]}...")
        print(f"\n  --- LLM User Message (first 500 chars) ---")
        print(f"  {messages[1]['content'][:500]}...")

        # 打印完整的 LLM 输出
        print(f"\n  --- RAW LLM Output ---")
        for line in raw.strip().split("\n"):
            print(f"  {line}")

        # 解析结果
        try:
            result = await resolver.resolve(instruction, SG_JSON, ROBOT_POS, lang)
            print(f"\n  Parsed: {result.action} → {result.target_label}")
            print(f"  Position: ({result.target_x:.1f}, {result.target_y:.1f})")
            print(f"  Confidence: {result.confidence:.2f}")
        except Exception as e:
            print(f"\n  Parse error: {e}")

        await asyncio.sleep(8)


# ══════════════════════════════════════════════════════════════
#  Test 2: TSG 信息增益探索 vs LLM 探索
# ══════════════════════════════════════════════════════════════

async def test_tsg_vs_llm_exploration(client, resolver):
    """对比 TSG Algorithm 2 和 LLM 的探索决策。"""
    sep()
    print("\n  TEST 2: TSG 信息增益探索 vs LLM 探索")
    sep()

    tsg = TopologySemGraph()
    tsg.update_from_scene_graph(SCENE_GRAPH)

    # 注入前沿
    frontier_nodes = SCENE_GRAPH.get("frontier_nodes", [])
    if frontier_nodes:
        points = [np.array([f["position"]["x"], f["position"]["y"]]) for f in frontier_nodes]
        sizes = [f.get("frontier_size", 2.0) for f in frontier_nodes]
        tsg.update_frontiers_from_costmap(points, sizes)

    # 模拟机器人在 corridor
    tsg.record_robot_position(0.0, 0.0)

    engine = SemanticPriorEngine()

    explore_tests = [
        ("找到咖啡机", "zh", "目标不存在 — kitchen最可能有咖啡机"),
        ("find the fire extinguisher on the 2nd floor", "en", "楼梯间最可能通向2楼"),
        ("找到会议室的投影仪", "zh", "当前场景没有会议室 — 需要探索新区域"),
    ]

    for instruction, lang, desc in explore_tests:
        print(f"\n  [{desc}]")
        print(f"  Instruction: {instruction}")

        # --- TSG Algorithm 2 ---
        print(f"\n  === TSG (Algorithm 2: Information Gain) ===")
        targets = tsg.get_best_exploration_target(instruction, engine, top_k=3)
        for i, t in enumerate(targets):
            print(f"  #{i+1}: {t.node_name} (score={t.score:.3f}, IG={t.information_gain:.3f}, "
                  f"reach={t.reachability_score:.3f}, hops={t.hops})")
            print(f"       {t.reasoning}")

        topo_context = tsg.to_prompt_context(lang)
        print(f"\n  TSG Prompt Context (给LLM的拓扑摘要):")
        for line in topo_context.strip().split("\n"):
            print(f"    {line}")

        # --- LLM Exploration ---
        print(f"\n  === LLM Exploration (Kimi-k2.5) ===")
        print(f"  Calling...", end="", flush=True)

        explored = [{"x": 0.0, "y": 0.0}]
        semantic_priors = engine.get_unexplored_priors(
            instruction, SCENE_GRAPH["rooms"],
            SCENE_GRAPH["topology_edges"],
            visited_room_ids={0},
            current_room_id=0,
        )

        messages = build_exploration_prompt(
            instruction, explored, ROBOT_POS, lang,
            topology_context=topo_context,
            semantic_priors=semantic_priors[:5] if semantic_priors else None,
        )

        t0 = time.perf_counter()
        try:
            raw = await client.chat(messages)
        except Exception as e:
            raw = f"ERROR: {e}"
        ms = (time.perf_counter() - t0) * 1000
        print(f" {ms:.0f}ms")

        print(f"\n  --- RAW LLM Output ---")
        for line in raw.strip().split("\n"):
            print(f"  {line}")

        await asyncio.sleep(8)


# ══════════════════════════════════════════════════════════════
#  Test 3: 穿越记忆影响探索决策
# ══════════════════════════════════════════════════════════════

async def test_traversal_memory(client):
    """模拟机器人穿越多个房间后, TSG如何调整探索优先级。"""
    sep()
    print("\n  TEST 3: 穿越记忆 → 探索优先级变化")
    sep()

    tsg = TopologySemGraph()
    tsg.update_from_scene_graph(SCENE_GRAPH)

    points = [np.array([f["position"]["x"], f["position"]["y"]])
              for f in SCENE_GRAPH.get("frontier_nodes", [])]
    tsg.update_frontiers_from_costmap(points)

    engine = SemanticPriorEngine()
    instruction = "找到灭火器"

    # Round 1: 刚开始, 在 corridor
    print("\n  [Round 1] 机器人在 corridor, 刚开始探索")
    tsg.record_robot_position(0.0, 0.0)
    targets = tsg.get_best_exploration_target(instruction, engine, top_k=5)
    for t in targets:
        print(f"    {t.node_name}: score={t.score:.3f} ({t.reasoning})")

    # Round 2: 去了 office_A, 没找到
    print("\n  [Round 2] 去了 office_A (没找到灭火器), 回来了")
    tsg.record_robot_position(5.0, 2.0)
    tsg.record_robot_position(0.0, 0.0)
    targets = tsg.get_best_exploration_target(instruction, engine, top_k=5)
    for t in targets:
        print(f"    {t.node_name}: score={t.score:.3f} ({t.reasoning})")

    # Round 3: 又去了 kitchen, 也没找到
    print("\n  [Round 3] 又去了 kitchen (没找到), 回来了")
    tsg.record_robot_position(-4.0, 3.0)
    tsg.record_robot_position(0.0, 0.0)
    targets = tsg.get_best_exploration_target(instruction, engine, top_k=5)
    for t in targets:
        print(f"    {t.node_name}: score={t.score:.3f} ({t.reasoning})")

    # LLM round — 给 LLM 完整穿越历史
    print("\n  [LLM] 把穿越历史告诉 Kimi-k2.5, 让它建议下一步")
    topo_ctx = tsg.to_prompt_context("zh")
    print(f"  拓扑摘要:")
    for line in topo_ctx.split("\n"):
        print(f"    {line}")

    explored = [
        {"x": 5.0, "y": 2.0},
        {"x": -4.0, "y": 3.0},
    ]
    messages = build_exploration_prompt(
        instruction, explored, ROBOT_POS, "zh",
        topology_context=topo_ctx,
    )

    print(f"  Calling Kimi-k2.5...", end="", flush=True)
    t0 = time.perf_counter()
    try:
        raw = await client.chat(messages)
    except Exception as e:
        raw = f"ERROR: {e}"
    ms = (time.perf_counter() - t0) * 1000
    print(f" {ms:.0f}ms")

    print(f"\n  --- RAW LLM Output ---")
    for line in raw.strip().split("\n"):
        print(f"  {line}")


# ══════════════════════════════════════════════════════════════
#  Main
# ══════════════════════════════════════════════════════════════

async def main():
    key = os.environ.get("MOONSHOT_API_KEY", "")
    if not key:
        print("ERROR: MOONSHOT_API_KEY not set.")
        print("Set it with: $env:MOONSHOT_API_KEY='sk-...'")
        sys.exit(1)

    config = make_config()
    client = create_llm_client(config)
    resolver = GoalResolver(primary_config=config)

    sep("=")
    print("  TSG + LLM End-to-End Test")
    print(f"  Scene: 5 rooms, 20 objects, 6 topology edges, 2 frontiers")
    print(f"  LLM: Kimi-k2.5 (Moonshot)")
    sep("=")

    await test_goal_resolution_with_topology(client, resolver)
    await asyncio.sleep(5)

    await test_tsg_vs_llm_exploration(client, resolver)
    await asyncio.sleep(5)

    await test_traversal_memory(client)

    sep("=")
    print("  ALL TESTS COMPLETE")
    sep("=")


if __name__ == "__main__":
    asyncio.run(main())
