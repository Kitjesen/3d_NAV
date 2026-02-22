#!/usr/bin/env python3
"""
规划器对比运行器 (Comparison Runner) — 命令行工具。

用法:
    python comparison_runner.py
    python comparison_runner.py --output test_results/comparison.txt

输出:
    - 终端对比表格
    - test_results/comparison_YYYY-MM-DD.txt  (Markdown 报告)
    - test_results/comparison_YYYY-MM-DD.csv  (CSV 数据)

设计:
    ComparisonRunner.run_all(scenarios) → ComparisonReport
    ComparisonReport.to_markdown()       → str
    ComparisonReport.to_csv()            → str
"""

import csv
import io
import os
import sys
import time
from dataclasses import dataclass, field
from datetime import date
from pathlib import Path
from typing import List, Optional

import numpy as np

# 确保在无 ROS2 环境下也可独立运行
_PARENT = Path(__file__).parent.parent
sys.path.insert(0, str(_PARENT))

# 延迟导入测试模块中的包装器（避免循环）
def _import_wrappers():
    """从 test_parallel_comparison 导入包装器和辅助函数。"""
    test_dir = Path(__file__).parent.parent / "test"
    sys.path.insert(0, str(test_dir))
    from test_parallel_comparison import (
        PCTBaselineWrapper,
        HybridPlannerWrapper,
        SCGPlannerWrapper,
        PlannerResult,
        SCENARIOS,
        print_comparison_table,
        print_summary_table,
    )
    return (
        PCTBaselineWrapper,
        HybridPlannerWrapper,
        SCGPlannerWrapper,
        PlannerResult,
        SCENARIOS,
        print_comparison_table,
        print_summary_table,
    )


# ══════════════════════════════════════════════════════════════════
#  对比报告数据结构
# ══════════════════════════════════════════════════════════════════

@dataclass
class ScenarioComparison:
    """单个场景下多规划器的对比结果。"""
    scenario_name: str
    scenario_description: str
    start: np.ndarray
    goal: np.ndarray
    results: List  # List[PlannerResult]


@dataclass
class ComparisonReport:
    """完整的对比报告。"""
    run_date: str
    run_timestamp: float
    planners: List[str]
    scenario_comparisons: List[ScenarioComparison] = field(default_factory=list)
    all_results: List = field(default_factory=list)  # List[PlannerResult]

    # ── 聚合统计 ──────────────────────────────────────────────────

    def success_rate(self, planner_name: str) -> float:
        """返回指定规划器的总成功率。"""
        pr = [r for r in self.all_results if r.planner_name == planner_name]
        if not pr:
            return 0.0
        return sum(1 for r in pr if r.success) / len(pr)

    def avg_planning_time_ms(self, planner_name: str) -> Optional[float]:
        """返回指定规划器成功规划的平均时间（ms）。"""
        pr = [r for r in self.all_results
              if r.planner_name == planner_name and r.success]
        if not pr:
            return None
        return float(np.mean([r.planning_time_ms for r in pr]))

    def avg_path_length_m(self, planner_name: str) -> Optional[float]:
        """返回指定规划器成功规划的平均路径长度（m）。"""
        pr = [r for r in self.all_results
              if r.planner_name == planner_name and r.success]
        if not pr:
            return None
        return float(np.mean([r.path_length_m for r in pr]))

    def avg_num_waypoints(self, planner_name: str) -> Optional[float]:
        """返回指定规划器成功规划的平均路径点数。"""
        pr = [r for r in self.all_results
              if r.planner_name == planner_name and r.success]
        if not pr:
            return None
        return float(np.mean([r.num_waypoints for r in pr]))

    # ── 格式化输出 ────────────────────────────────────────────────

    def to_markdown(self) -> str:
        """生成 Markdown 格式的对比报告。"""
        buf = io.StringIO()

        buf.write(f"# 路径规划器并行对比报告\n\n")
        buf.write(f"**运行日期**: {self.run_date}  \n")
        buf.write(f"**规划器**: {', '.join(self.planners)}  \n")
        buf.write(f"**场景数量**: {len(self.scenario_comparisons)}  \n\n")

        # 总览表
        buf.write("## 汇总统计\n\n")
        buf.write(
            "| 规划器 | 成功率 | 均值时间(ms) | 均值路径长度(m) | 均值路径点数 |\n"
        )
        buf.write(
            "| ------ | ------ | ------------ | --------------- | ------------ |\n"
        )
        for pname in self.planners:
            rate = self.success_rate(pname)
            t_ms = self.avg_planning_time_ms(pname)
            l_m = self.avg_path_length_m(pname)
            wp = self.avg_num_waypoints(pname)

            t_str = f"{t_ms:.2f}" if t_ms is not None else "—"
            l_str = f"{l_m:.2f}" if l_m is not None else "—"
            wp_str = f"{wp:.1f}" if wp is not None else "—"

            buf.write(f"| {pname} | {rate:.0%} | {t_str} | {l_str} | {wp_str} |\n")

        buf.write("\n")

        # 各场景详情
        buf.write("## 各场景对比\n\n")
        for sc in self.scenario_comparisons:
            start_str = f"[{sc.start[0]:.0f}, {sc.start[1]:.0f}, {sc.start[2]:.0f}]"
            goal_str = f"[{sc.goal[0]:.0f}, {sc.goal[1]:.0f}, {sc.goal[2]:.0f}]"
            buf.write(
                f"### {sc.scenario_name}\n\n"
                f"**描述**: {sc.scenario_description}  \n"
                f"**起点**: {start_str}  **终点**: {goal_str}\n\n"
            )
            buf.write(
                "| 规划器 | 状态 | 时间(ms) | 路径长度(m) | 路径点数 |\n"
            )
            buf.write(
                "| ------ | ---- | -------- | ----------- | -------- |\n"
            )
            for r in sc.results:
                status = "SKIPPED" if r.skipped else ("OK" if r.success else "FAILED")
                t_str = f"{r.planning_time_ms:.2f}" if r.success else "—"
                l_str = f"{r.path_length_m:.2f}" if r.success else "—"
                wp_str = str(r.num_waypoints) if r.success else "—"
                err = ""
                if not r.success and not r.skipped and r.error_msg:
                    err = f" `{r.error_msg[:40]}`"
                buf.write(
                    f"| {r.planner_name} | {status}{err} | {t_str} | {l_str} | {wp_str} |\n"
                )
            buf.write("\n")

        return buf.getvalue()

    def to_csv(self) -> str:
        """生成 CSV 格式的原始数据。"""
        buf = io.StringIO()
        writer = csv.writer(buf)

        # 表头
        writer.writerow([
            "run_date",
            "scenario_name",
            "scenario_description",
            "start_x", "start_y", "start_z",
            "goal_x", "goal_y", "goal_z",
            "planner_name",
            "success",
            "skipped",
            "planning_time_ms",
            "path_length_m",
            "num_waypoints",
            "memory_mb",
            "error_msg",
        ])

        for sc in self.scenario_comparisons:
            for r in sc.results:
                writer.writerow([
                    self.run_date,
                    sc.scenario_name,
                    sc.scenario_description,
                    sc.start[0], sc.start[1], sc.start[2],
                    sc.goal[0], sc.goal[1], sc.goal[2],
                    r.planner_name,
                    int(r.success),
                    int(r.skipped),
                    f"{r.planning_time_ms:.4f}",
                    f"{r.path_length_m:.4f}",
                    r.num_waypoints,
                    f"{r.memory_mb:.4f}",
                    r.error_msg,
                ])

        return buf.getvalue()


# ══════════════════════════════════════════════════════════════════
#  ComparisonRunner 主类
# ══════════════════════════════════════════════════════════════════

class ComparisonRunner:
    """
    规划器并行对比运行器。

    用法:
        runner = ComparisonRunner()
        report = runner.run_all(scenarios)
        print(report.to_markdown())
        runner.save_report(report, "test_results/")
    """

    def __init__(self, verbose: bool = True):
        """
        Args:
            verbose: 是否在运行时打印进度和表格
        """
        self.verbose = verbose

        (
            PCTBaselineWrapper,
            HybridPlannerWrapper,
            SCGPlannerWrapper,
            self._PlannerResult,
            self._default_scenarios,
            self._print_table,
            self._print_summary,
        ) = _import_wrappers()

        self._wrappers = [
            PCTBaselineWrapper(),
            HybridPlannerWrapper(),
            SCGPlannerWrapper(),
        ]
        self._planner_names = [w.NAME for w in self._wrappers]

    def run_all(self, scenarios=None) -> ComparisonReport:
        """
        对所有场景运行三套规划器，返回完整对比报告。

        Args:
            scenarios: 场景列表（默认使用模块级 SCENARIOS）

        Returns:
            ComparisonReport 对象
        """
        if scenarios is None:
            scenarios = self._default_scenarios

        today = date.today().isoformat()
        report = ComparisonReport(
            run_date=today,
            run_timestamp=time.time(),
            planners=self._planner_names,
        )

        if self.verbose:
            print("\n" + "=" * 72)
            print(f"规划器并行对比 — {today}")
            print(f"场景: {len(scenarios)}  规划器: {len(self._wrappers)}")
            print("=" * 72)

        for scenario in scenarios:
            if self.verbose:
                print(f"\n[场景] {scenario['name']} — {scenario['description']}")

            sc_results = []
            for wrapper in self._wrappers:
                result = wrapper.plan(scenario)
                sc_results.append(result)
                report.all_results.append(result)

            sc = ScenarioComparison(
                scenario_name=scenario["name"],
                scenario_description=scenario["description"],
                start=scenario["start"],
                goal=scenario["goal"],
                results=sc_results,
            )
            report.scenario_comparisons.append(sc)

            if self.verbose:
                self._print_table(scenario, sc_results)

        if self.verbose:
            self._print_summary(report.all_results)

        return report

    def save_report(
        self,
        report: ComparisonReport,
        output_dir: str = "test_results",
    ) -> dict:
        """
        将报告保存为 Markdown 和 CSV 文件。

        Args:
            report: ComparisonReport 对象
            output_dir: 输出目录（自动创建）

        Returns:
            {"markdown": str, "csv": str}  — 保存的文件路径
        """
        out_path = Path(output_dir)
        out_path.mkdir(parents=True, exist_ok=True)

        md_path = out_path / f"comparison_{report.run_date}.md"
        csv_path = out_path / f"comparison_{report.run_date}.csv"

        md_content = report.to_markdown()
        csv_content = report.to_csv()

        md_path.write_text(md_content, encoding="utf-8")
        csv_path.write_text(csv_content, encoding="utf-8")

        if self.verbose:
            print(f"\n报告已保存:")
            print(f"  Markdown: {md_path}")
            print(f"  CSV:      {csv_path}")

        return {"markdown": str(md_path), "csv": str(csv_path)}


# ══════════════════════════════════════════════════════════════════
#  命令行入口
# ══════════════════════════════════════════════════════════════════

def main():
    import argparse

    parser = argparse.ArgumentParser(
        description="路径规划器并行对比工具",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  python comparison_runner.py
  python comparison_runner.py --output test_results/
  python comparison_runner.py --quiet
        """,
    )
    parser.add_argument(
        "--output", "-o",
        default="test_results",
        help="报告输出目录 (默认: test_results)",
    )
    parser.add_argument(
        "--quiet", "-q",
        action="store_true",
        help="静默模式，只保存文件不打印表格",
    )
    args = parser.parse_args()

    runner = ComparisonRunner(verbose=not args.quiet)
    report = runner.run_all()
    paths = runner.save_report(report, args.output)

    if args.quiet:
        print(f"报告已保存: {paths['markdown']}")

    # 返回码: 0=全部成功 1=有失败 2=有错误
    failed = sum(
        1 for r in report.all_results
        if not r.success and not r.skipped
    )
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
