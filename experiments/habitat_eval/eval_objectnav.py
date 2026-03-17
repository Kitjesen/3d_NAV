"""
NaviMind Habitat ObjectNav 评测主脚本。

加载 HM3D ObjectNav episodes, 运行 NaviMindAgent,
记录 Success / SPL / Distance-to-Goal, 输出汇总表格。

用法:
    python eval_objectnav.py
    python eval_objectnav.py --config config/habitat_eval.yaml
    python eval_objectnav.py --max-episodes 50
    python eval_objectnav.py --category chair
"""

import argparse
import json
import math
import sys
import time
from collections import defaultdict
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np
import yaml

try:
    import habitat
    from habitat import make_dataset
    from habitat.config.default_structured_configs import (
        HabitatConfigPlugin,
        TaskConfig,
    )
    from habitat.core.env import Env
    from habitat.utils.visualizations.utils import observations_to_image
except ImportError:
    print("错误: 需要安装 habitat-lab 和 habitat-sim")
    print("  pip install habitat-sim habitat-lab")
    sys.exit(1)

from habitat_navimind_agent import NaviMindAgent, OBJECTNAV_CATEGORIES, _normalize_category


def build_habitat_config(eval_config: Dict) -> Any:
    """构建 Habitat 配置。"""
    from omegaconf import DictConfig, OmegaConf
    from habitat.config.default import get_config

    sensor_cfg = eval_config.get("sensor", {})
    eval_params = eval_config.get("eval", {})

    # 使用 Habitat 默认配置作为基础, 覆盖关键参数
    overrides = [
        "habitat.dataset.type=HM3D-v0",
        f"habitat.dataset.split={eval_params.get('split', 'val')}",
        f"habitat.environment.max_episode_steps={eval_params.get('max_steps', 500)}",
        f"habitat.task.measurements.success.success_distance={eval_params.get('success_distance', 2.0)}",
        # RGB
        f"habitat.simulator.agents.main_agent.sim_sensors.rgb_sensor.width={sensor_cfg.get('rgb', {}).get('width', 256)}",
        f"habitat.simulator.agents.main_agent.sim_sensors.rgb_sensor.height={sensor_cfg.get('rgb', {}).get('height', 256)}",
        f"habitat.simulator.agents.main_agent.sim_sensors.rgb_sensor.hfov={sensor_cfg.get('rgb', {}).get('hfov', 79)}",
        # Depth
        f"habitat.simulator.agents.main_agent.sim_sensors.depth_sensor.width={sensor_cfg.get('depth', {}).get('width', 256)}",
        f"habitat.simulator.agents.main_agent.sim_sensors.depth_sensor.height={sensor_cfg.get('depth', {}).get('height', 256)}",
        f"habitat.simulator.agents.main_agent.sim_sensors.depth_sensor.hfov={sensor_cfg.get('depth', {}).get('hfov', 79)}",
    ]

    config = get_config(
        config_path="benchmark/nav/objectnav/objectnav_v2_hm3d.yaml",
        overrides=overrides,
    )
    return config


def build_simple_habitat_config(eval_config: Dict) -> Any:
    """
    简化版 Habitat 配置构建 — 直接使用 OmegaConf。
    当 Habitat 默认配置文件不可用时的回退方案。
    """
    from omegaconf import OmegaConf

    sensor_cfg = eval_config.get("sensor", {})
    eval_params = eval_config.get("eval", {})

    config_dict = {
        "habitat": {
            "environment": {
                "max_episode_steps": eval_params.get("max_steps", 500),
            },
            "task": {
                "type": "ObjectNav-v1",
                "measurements": {
                    "success": {
                        "success_distance": eval_params.get("success_distance", 2.0),
                    },
                },
                "lab_sensors": {
                    "gps_sensor": {"dimensionality": 2},
                    "compass_sensor": {"dimensionality": 1},
                },
            },
            "dataset": {
                "type": "ObjectNav-v2",
                "split": eval_params.get("split", "val"),
                "data_path": "data/datasets/objectnav/hm3d/v2/{split}/{split}.json.gz",
                "scenes_dir": "data/scene_datasets/hm3d",
            },
            "simulator": {
                "type": "Sim-v0",
                "agents": {
                    "main_agent": {
                        "sim_sensors": {
                            "rgb_sensor": {
                                "type": "HabitatSimRGBSensor",
                                "width": sensor_cfg.get("rgb", {}).get("width", 256),
                                "height": sensor_cfg.get("rgb", {}).get("height", 256),
                                "hfov": sensor_cfg.get("rgb", {}).get("hfov", 79),
                                "position": sensor_cfg.get("rgb", {}).get("position", [0, 0.88, 0]),
                            },
                            "depth_sensor": {
                                "type": "HabitatSimDepthSensor",
                                "width": sensor_cfg.get("depth", {}).get("width", 256),
                                "height": sensor_cfg.get("depth", {}).get("height", 256),
                                "hfov": sensor_cfg.get("depth", {}).get("hfov", 79),
                                "min_depth": sensor_cfg.get("depth", {}).get("min_depth", 0.0),
                                "max_depth": sensor_cfg.get("depth", {}).get("max_depth", 10.0),
                            },
                            "semantic_sensor": {
                                "type": "HabitatSimSemanticSensor",
                                "width": sensor_cfg.get("semantic", {}).get("width", 256),
                                "height": sensor_cfg.get("semantic", {}).get("height", 256),
                                "hfov": sensor_cfg.get("semantic", {}).get("hfov", 79),
                            },
                        },
                    },
                },
                "forward_step_size": eval_config.get("action", {}).get("move_forward", 0.25),
                "turn_angle": eval_config.get("action", {}).get("turn_left", 30),
            },
        },
    }
    return OmegaConf.create(config_dict)


def get_semantic_categories(env: Any) -> Dict[int, str]:
    """从 Habitat 环境中提取语义类别映射 (instance_id → label)。"""
    categories = {}
    try:
        scene = env.sim.semantic_scene
        for obj in scene.objects:
            if obj is not None and obj.category is not None:
                categories[int(obj.id)] = obj.category.name()
    except Exception:
        pass
    return categories


def compute_spl(
    success: bool, path_length: float, shortest_path_length: float
) -> float:
    """计算 SPL (Success weighted by Path Length)。"""
    if not success or shortest_path_length <= 0:
        return 0.0
    return float(success) * shortest_path_length / max(path_length, shortest_path_length)


def run_episode(
    env: Any,
    agent: NaviMindAgent,
    max_steps: int,
    success_distance: float,
) -> Dict[str, Any]:
    """
    运行单个 episode, 返回评测指标。
    """
    obs = env.reset()
    episode = env.current_episode

    # 目标类别
    goal_category = episode.object_category if hasattr(episode, "object_category") else ""
    goal_category = _normalize_category(goal_category)

    # 目标位置 (用于计算距离)
    goal_position = None
    if hasattr(episode, "goals") and episode.goals:
        goal_pos = episode.goals[0].position
        goal_position = np.array(goal_pos, dtype=np.float64)

    # 最短路径长度 (geodesic)
    shortest_path_length = 0.0
    if hasattr(episode, "info") and "geodesic_distance" in episode.info:
        shortest_path_length = float(episode.info["geodesic_distance"])

    # 重置 agent
    agent.reset()
    agent.set_goal(goal_category, goal_position)

    # 构建语义类别映射
    semantic_categories = get_semantic_categories(env)

    path_length = 0.0
    prev_position = env.sim.get_agent_state().position.copy()
    success = False

    for step in range(max_steps):
        # 注入语义类别映射到观测
        obs["_semantic_categories"] = semantic_categories

        action = agent.act(obs)

        if action == 0:  # STOP
            # 检查是否到达目标
            agent_pos = env.sim.get_agent_state().position
            if goal_position is not None:
                dist = np.linalg.norm(agent_pos - goal_position)
                success = dist < success_distance
            break

        obs = env.step(action)

        # 累积路径长度
        current_position = env.sim.get_agent_state().position
        step_dist = np.linalg.norm(current_position - prev_position)
        path_length += step_dist
        prev_position = current_position.copy()

    # 最终距离
    final_position = env.sim.get_agent_state().position
    distance_to_goal = float("inf")
    if goal_position is not None:
        distance_to_goal = float(np.linalg.norm(final_position - goal_position))

    spl = compute_spl(success, path_length, shortest_path_length)

    return {
        "episode_id": episode.episode_id if hasattr(episode, "episode_id") else "unknown",
        "scene_id": episode.scene_id if hasattr(episode, "scene_id") else "unknown",
        "category": goal_category,
        "success": success,
        "spl": spl,
        "distance_to_goal": distance_to_goal,
        "path_length": path_length,
        "shortest_path_length": shortest_path_length,
        "steps": step + 1,
        "agent_stats": agent.stats,
    }


def print_results_table(results: List[Dict], elapsed: float) -> None:
    """打印评测结果汇总表格。"""
    if not results:
        print("无评测结果。")
        return

    # Overall
    n = len(results)
    successes = sum(1 for r in results if r["success"])
    avg_spl = np.mean([r["spl"] for r in results])
    avg_dist = np.mean([r["distance_to_goal"] for r in results])
    avg_steps = np.mean([r["steps"] for r in results])
    avg_path = np.mean([r["path_length"] for r in results])

    print("\n" + "=" * 72)
    print(f"  NaviMind Habitat ObjectNav 评测结果")
    print(f"  Episodes: {n}  |  耗时: {elapsed:.1f}s  |  平均: {elapsed/max(n,1):.1f}s/ep")
    print("=" * 72)

    # Overall metrics
    print(f"\n{'指标':<25} {'值':>10}")
    print("-" * 40)
    print(f"{'Success Rate':<25} {successes/n*100:>9.1f}%")
    print(f"{'SPL':<25} {avg_spl:>10.4f}")
    print(f"{'Avg Distance to Goal':<25} {avg_dist:>9.2f}m")
    print(f"{'Avg Steps':<25} {avg_steps:>10.1f}")
    print(f"{'Avg Path Length':<25} {avg_path:>9.2f}m")

    # Per-category breakdown
    by_cat = defaultdict(list)
    for r in results:
        by_cat[r["category"]].append(r)

    print(f"\n{'类别':<15} {'Episodes':>8} {'SR%':>8} {'SPL':>8} {'Dist':>8}")
    print("-" * 55)
    for cat in sorted(by_cat.keys()):
        cat_results = by_cat[cat]
        cat_n = len(cat_results)
        cat_sr = sum(1 for r in cat_results if r["success"]) / cat_n * 100
        cat_spl = np.mean([r["spl"] for r in cat_results])
        cat_dist = np.mean([r["distance_to_goal"] for r in cat_results])
        print(f"{cat:<15} {cat_n:>8} {cat_sr:>7.1f}% {cat_spl:>8.4f} {cat_dist:>7.2f}m")

    # Fast Path 统计
    total_resolves = sum(r["agent_stats"]["total_resolves"] for r in results)
    total_hits = sum(r["agent_stats"]["fast_path_hits"] for r in results)
    fp_rate = total_hits / max(1, total_resolves) * 100
    print(f"\n{'Fast Path Hit Rate':<25} {fp_rate:>9.1f}% ({total_hits}/{total_resolves})")
    print("=" * 72)


def save_results(results: List[Dict], output_path: str) -> None:
    """保存详细结果到 JSON。"""
    # numpy 类型转换
    def convert(obj):
        if isinstance(obj, (np.integer,)):
            return int(obj)
        if isinstance(obj, (np.floating,)):
            return float(obj)
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        if isinstance(obj, (np.bool_,)):
            return bool(obj)
        return obj

    serializable = json.loads(json.dumps(results, default=convert))
    with open(output_path, "w", encoding="utf-8") as f:
        json.dump(serializable, f, indent=2, ensure_ascii=False)
    print(f"\n详细结果已保存: {output_path}")


def main():
    parser = argparse.ArgumentParser(description="NaviMind Habitat ObjectNav 评测")
    parser.add_argument(
        "--config",
        type=str,
        default=str(Path(__file__).parent / "config" / "habitat_eval.yaml"),
        help="评测配置文件",
    )
    parser.add_argument("--max-episodes", type=int, default=0, help="最大评测 episodes 数 (0=全部)")
    parser.add_argument("--category", type=str, default="", help="只评测指定类别")
    parser.add_argument("--output", type=str, default="", help="结果输出 JSON 路径")
    parser.add_argument("--verbose", action="store_true", help="详细输出每个 episode")
    args = parser.parse_args()

    # 加载配置
    config_path = Path(args.config)
    if config_path.exists():
        with open(config_path, encoding="utf-8") as f:
            eval_config = yaml.safe_load(f)
    else:
        print(f"警告: 配置文件 {config_path} 不存在, 使用默认配置")
        eval_config = {}

    eval_params = eval_config.get("eval", {})
    max_steps = eval_params.get("max_steps", 500)
    success_distance = eval_params.get("success_distance", 2.0)

    # 构建 Habitat 环境
    print("正在初始化 Habitat 环境...")
    try:
        habitat_config = build_habitat_config(eval_config)
    except Exception as e:
        print(f"使用默认配置失败 ({e}), 切换到简化配置...")
        habitat_config = build_simple_habitat_config(eval_config)

    env = Env(config=habitat_config)
    print(f"Habitat 环境就绪, 共 {len(env.episodes)} 个 episodes")

    # 过滤类别
    episodes = env.episodes
    if args.category:
        target_cat = _normalize_category(args.category)
        episodes = [
            ep for ep in episodes
            if hasattr(ep, "object_category")
            and _normalize_category(ep.object_category) == target_cat
        ]
        print(f"已过滤类别 '{target_cat}': {len(episodes)} 个 episodes")

    if args.max_episodes > 0:
        episodes = episodes[: args.max_episodes]
        print(f"限制评测数量: {len(episodes)} 个 episodes")

    # 创建 agent
    agent = NaviMindAgent(eval_config)

    # 运行评测
    results = []
    t0 = time.time()

    for i, episode in enumerate(episodes):
        env.current_episode = episode

        ep_result = run_episode(env, agent, max_steps, success_distance)
        results.append(ep_result)

        if args.verbose or (i + 1) % 10 == 0:
            status = "PASS" if ep_result["success"] else "FAIL"
            print(
                f"[{i+1}/{len(episodes)}] {status} "
                f"cat={ep_result['category']:<15} "
                f"dist={ep_result['distance_to_goal']:.2f}m "
                f"spl={ep_result['spl']:.3f} "
                f"steps={ep_result['steps']}"
            )

    elapsed = time.time() - t0

    # 输出结果
    print_results_table(results, elapsed)

    # 保存
    output_path = args.output
    if not output_path:
        output_path = str(Path(__file__).parent / "results_objectnav.json")
    save_results(results, output_path)

    env.close()


if __name__ == "__main__":
    main()
