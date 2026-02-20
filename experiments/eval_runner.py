#!/usr/bin/env python3
"""
HSG-Nav 真机评测框架 — 自动运行指令集、记录导航结果、计算 SR/SPL。

用法:
  python3 eval_runner.py --instruction-set instruction_set.json --level L1 --trials 3
  python3 eval_runner.py --instruction-set instruction_set.json --level all --ablation wo_scene_graph
  python3 eval_runner.py --results-dir results/ --report

依赖:
  - rclpy, std_msgs, geometry_msgs, nav_msgs (ROS2)
  - JSON 指令集文件 (instruction_set.json)

输出:
  results/<timestamp>_<level>_<ablation>/
    ├── raw_logs/       # 每条指令的详细日志
    ├── summary.json    # 汇总结果
    └── report.md       # 可读报告
"""

import argparse
import json
import math
import os
import sys
import time
from dataclasses import dataclass, field, asdict
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple


@dataclass
class TrialResult:
    """单次试验结果。"""
    instruction_id: str
    trial_idx: int
    instruction_zh: str
    instruction_en: str
    level: str
    category: str

    # 导航结果
    success: bool = False
    final_position: Optional[Dict[str, float]] = None
    target_position: Optional[Dict[str, float]] = None
    distance_to_target: float = float("inf")
    arrival_radius: float = 1.0

    # 路径
    actual_path_length: float = 0.0
    shortest_path_length: float = 0.0
    spl: float = 0.0

    # 时间
    start_time: float = 0.0
    end_time: float = 0.0
    duration_sec: float = 0.0

    # 探索
    explore_steps: int = 0
    replan_count: int = 0

    # 失败原因
    failure_reason: str = ""
    failure_category: str = ""  # detection_failure / scene_graph_error / planning_error / motion_failure / timeout

    # 平台特有
    blur_frame_ratio: float = 0.0
    detection_fps: float = 0.0

    def compute_spl(self):
        """计算 SPL = SR * (shortest / max(actual, shortest))。"""
        if not self.success:
            self.spl = 0.0
            return
        denom = max(self.actual_path_length, self.shortest_path_length, 0.01)
        self.spl = self.shortest_path_length / denom


@dataclass
class LevelSummary:
    """某一级别的汇总。"""
    level: str
    total: int = 0
    successes: int = 0
    sr: float = 0.0
    avg_spl: float = 0.0
    avg_time_sec: float = 0.0
    avg_explore_steps: float = 0.0
    avg_path_length: float = 0.0
    failure_breakdown: Dict[str, int] = field(default_factory=dict)


def compute_shortest_path(start: Dict, target: Dict) -> float:
    """计算两点间直线距离 (最短路径的下界估计)。"""
    dx = target["x"] - start["x"]
    dy = target["y"] - start["y"]
    dz = target.get("z", 0) - start.get("z", 0)
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def distance_2d(a: Dict, b: Dict) -> float:
    """2D 距离。"""
    dx = a["x"] - b["x"]
    dy = a["y"] - b["y"]
    return math.sqrt(dx * dx + dy * dy)


class EvalRunner:
    """评测运行器 (可 ROS2 也可 offline mock)。"""

    def __init__(self, instruction_file: str, results_dir: str = "results"):
        with open(instruction_file, "r", encoding="utf-8") as f:
            self.instructions = json.load(f)
        self.results_dir = Path(results_dir)
        self.results_dir.mkdir(parents=True, exist_ok=True)

    def get_instructions(self, level: str) -> List[Dict]:
        """获取指定级别的指令列表。"""
        if level == "all":
            all_instr = []
            for key in ["L1_simple", "L2_spatial", "L3_multistep"]:
                if key in self.instructions:
                    all_instr.extend(self.instructions[key]["instructions"])
            return all_instr
        level_map = {"L1": "L1_simple", "L2": "L2_spatial", "L3": "L3_multistep"}
        key = level_map.get(level, level)
        if key in self.instructions:
            return self.instructions[key]["instructions"]
        return []

    def run_single_trial(
        self,
        instr: Dict,
        trial_idx: int,
        level: str,
        ros_node=None,
    ) -> TrialResult:
        """
        运行单条指令的单次试验。
        
        在真机环境中, 此方法:
          1. 将机器人移到 start_position
          2. 发布指令到 /instruction
          3. 监听 /status 直到完成或超时
          4. 记录最终位置、路径长度等
        
        当前为 offline mock 框架; 真机运行时替换 ROS2 调用。
        """
        result = TrialResult(
            instruction_id=instr["id"],
            trial_idx=trial_idx,
            instruction_zh=instr.get("instruction_zh", ""),
            instruction_en=instr.get("instruction_en", ""),
            level=level,
            category=instr.get("category", "unknown"),
            target_position=instr.get("target_position"),
            arrival_radius=instr.get("arrival_radius_m", 1.0),
            start_time=time.time(),
        )

        # 最短路径
        if instr.get("start_position") and instr.get("target_position"):
            result.shortest_path_length = compute_shortest_path(
                instr["start_position"], instr["target_position"]
            )

        if ros_node is not None:
            result = self._run_with_ros(result, instr, ros_node)
        else:
            # Offline mock: 标记为待运行
            result.failure_reason = "offline_mock: not executed on real robot"
            result.failure_category = "not_run"

        result.end_time = time.time()
        result.duration_sec = result.end_time - result.start_time
        result.compute_spl()
        return result

    def _run_with_ros(self, result: TrialResult, instr: Dict, node) -> TrialResult:
        """通过 ROS2 执行真机导航 (需在 ROS2 环境中运行)。"""
        try:
            import rclpy
            from std_msgs.msg import String
            from geometry_msgs.msg import PoseStamped

            # 发布指令
            pub = node.create_publisher(String, "instruction", 10)
            msg = String()
            msg.data = json.dumps({
                "instruction": instr.get("instruction_en", instr.get("instruction_zh", "")),
                "language": "en" if instr.get("instruction_en") else "zh",
                "explore_if_unknown": True,
                "timeout_sec": 120.0,
                "arrival_radius": instr.get("arrival_radius_m", 1.0),
            })
            pub.publish(msg)

            # 等待完成 (轮询 status)
            timeout = 120.0
            start = time.time()
            completed = False

            while time.time() - start < timeout:
                rclpy.spin_once(node, timeout_sec=1.0)
                # 读取状态 (实际实现中从 /status 话题获取)
                if hasattr(node, "_latest_status"):
                    status = node._latest_status
                    if status in ("completed", "failed", "cancelled"):
                        completed = True
                        break

            if not completed:
                result.failure_reason = "timeout"
                result.failure_category = "timeout"
            else:
                # 获取最终位置
                if hasattr(node, "_robot_position") and node._robot_position:
                    result.final_position = dict(node._robot_position)
                    if result.target_position:
                        result.distance_to_target = distance_2d(
                            result.final_position, result.target_position
                        )
                        result.success = result.distance_to_target < result.arrival_radius

                if hasattr(node, "_actual_path_length"):
                    result.actual_path_length = node._actual_path_length

        except Exception as e:
            result.failure_reason = f"ROS2 error: {e}"
            result.failure_category = "motion_failure"

        return result

    def run_level(
        self, level: str, trials: int = 3, ablation: str = "full", ros_node=None
    ) -> List[TrialResult]:
        """运行某级别的所有指令, 每条重复 trials 次。"""
        instructions = self.get_instructions(level)
        if not instructions:
            print(f"No instructions found for level '{level}'")
            return []

        print(f"\n{'='*60}")
        print(f"Running {level} evaluation: {len(instructions)} instructions x {trials} trials")
        print(f"Ablation: {ablation}")
        print(f"{'='*60}\n")

        all_results: List[TrialResult] = []
        for instr in instructions:
            for t in range(trials):
                print(f"  [{instr['id']}] trial {t+1}/{trials}: "
                      f"{instr.get('instruction_en', instr.get('instruction_zh', ''))}")
                result = self.run_single_trial(instr, t, level, ros_node)
                all_results.append(result)
                status = "✓" if result.success else "✗"
                print(f"    {status} dist={result.distance_to_target:.2f}m "
                      f"SPL={result.spl:.3f} time={result.duration_sec:.1f}s")

        return all_results

    @staticmethod
    def summarize(results: List[TrialResult], level: str) -> LevelSummary:
        """汇总一组试验结果。"""
        summary = LevelSummary(level=level, total=len(results))

        if not results:
            return summary

        successes = [r for r in results if r.success]
        summary.successes = len(successes)
        summary.sr = summary.successes / summary.total if summary.total > 0 else 0.0
        summary.avg_spl = (
            sum(r.spl for r in results) / summary.total if summary.total > 0 else 0.0
        )
        summary.avg_time_sec = (
            sum(r.duration_sec for r in results) / summary.total if summary.total > 0 else 0.0
        )
        summary.avg_explore_steps = (
            sum(r.explore_steps for r in results) / summary.total if summary.total > 0 else 0.0
        )
        summary.avg_path_length = (
            sum(r.actual_path_length for r in results) / summary.total if summary.total > 0 else 0.0
        )

        # 失败原因分类
        for r in results:
            if not r.success and r.failure_category:
                summary.failure_breakdown[r.failure_category] = (
                    summary.failure_breakdown.get(r.failure_category, 0) + 1
                )

        return summary

    def save_results(
        self,
        results: List[TrialResult],
        level: str,
        ablation: str = "full",
    ) -> Path:
        """保存结果到文件。"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        run_dir = self.results_dir / f"{timestamp}_{level}_{ablation}"
        run_dir.mkdir(parents=True, exist_ok=True)

        # Raw logs
        raw_dir = run_dir / "raw_logs"
        raw_dir.mkdir(exist_ok=True)
        for r in results:
            with open(raw_dir / f"{r.instruction_id}_t{r.trial_idx}.json", "w") as f:
                json.dump(asdict(r), f, indent=2, ensure_ascii=False)

        # Summary
        summary = self.summarize(results, level)
        with open(run_dir / "summary.json", "w") as f:
            json.dump(asdict(summary), f, indent=2, ensure_ascii=False)

        # Report
        report = self.generate_report(results, summary, ablation)
        with open(run_dir / "report.md", "w") as f:
            f.write(report)

        print(f"\nResults saved to: {run_dir}")
        return run_dir

    @staticmethod
    def generate_report(
        results: List[TrialResult],
        summary: LevelSummary,
        ablation: str,
    ) -> str:
        """生成可读的 Markdown 报告。"""
        lines = [
            f"# HSG-Nav Evaluation Report",
            f"",
            f"**Level**: {summary.level}",
            f"**Ablation**: {ablation}",
            f"**Date**: {datetime.now().strftime('%Y-%m-%d %H:%M')}",
            f"**Total Trials**: {summary.total}",
            f"",
            f"## Summary Metrics",
            f"",
            f"| Metric | Value |",
            f"|--------|-------|",
            f"| SR (Success Rate) | {summary.sr:.1%} ({summary.successes}/{summary.total}) |",
            f"| SPL | {summary.avg_spl:.3f} |",
            f"| Avg Time (s) | {summary.avg_time_sec:.1f} |",
            f"| Avg Path Length (m) | {summary.avg_path_length:.2f} |",
            f"| Avg Explore Steps | {summary.avg_explore_steps:.1f} |",
            f"",
        ]

        if summary.failure_breakdown:
            lines.extend([
                f"## Failure Breakdown",
                f"",
                f"| Category | Count |",
                f"|----------|-------|",
            ])
            for cat, count in sorted(summary.failure_breakdown.items(), key=lambda x: -x[1]):
                lines.append(f"| {cat} | {count} |")
            lines.append("")

        # Per-instruction results
        lines.extend([
            f"## Per-Instruction Results",
            f"",
            f"| ID | Instruction | SR | SPL | Avg Time |",
            f"|----|------------|-----|-----|----------|",
        ])

        # Group by instruction
        by_id: Dict[str, List[TrialResult]] = {}
        for r in results:
            by_id.setdefault(r.instruction_id, []).append(r)

        for instr_id, trials in sorted(by_id.items()):
            sr = sum(1 for t in trials if t.success) / len(trials)
            avg_spl = sum(t.spl for t in trials) / len(trials)
            avg_time = sum(t.duration_sec for t in trials) / len(trials)
            instr_text = trials[0].instruction_en or trials[0].instruction_zh
            lines.append(
                f"| {instr_id} | {instr_text[:40]} | {sr:.0%} | {avg_spl:.3f} | {avg_time:.1f}s |"
            )

        lines.append("")
        return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description="HSG-Nav Evaluation Runner")
    parser.add_argument(
        "--instruction-set", default="instruction_set.json",
        help="Path to instruction set JSON file"
    )
    parser.add_argument(
        "--level", default="L1", choices=["L1", "L2", "L3", "all"],
        help="Instruction difficulty level"
    )
    parser.add_argument("--trials", type=int, default=3, help="Number of trials per instruction")
    parser.add_argument("--ablation", default="full", help="Ablation configuration name")
    parser.add_argument("--results-dir", default="results", help="Results output directory")
    parser.add_argument("--report", action="store_true", help="Generate report from existing results")
    parser.add_argument("--ros", action="store_true", help="Run with ROS2 (requires running system)")

    args = parser.parse_args()

    runner = EvalRunner(args.instruction_set, args.results_dir)

    if args.report:
        print("Report generation from existing results not yet implemented")
        return

    ros_node = None
    if args.ros:
        try:
            import rclpy
            rclpy.init()
            from rclpy.node import Node
            ros_node = Node("eval_runner")
            print("ROS2 node initialized")
        except ImportError:
            print("WARNING: rclpy not available, running in offline mock mode")

    results = runner.run_level(args.level, args.trials, args.ablation, ros_node)

    if results:
        runner.save_results(results, args.level, args.ablation)

        summary = runner.summarize(results, args.level)
        print(f"\n{'='*60}")
        print(f"RESULTS: {summary.level} (ablation={args.ablation})")
        print(f"  SR:  {summary.sr:.1%} ({summary.successes}/{summary.total})")
        print(f"  SPL: {summary.avg_spl:.3f}")
        print(f"  Avg Time: {summary.avg_time_sec:.1f}s")
        print(f"{'='*60}")

    if ros_node:
        ros_node.destroy_node()
        import rclpy
        rclpy.shutdown()


if __name__ == "__main__":
    main()
