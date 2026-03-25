"""
仿真平台 CLI 入口

由 lingtu sim 命令调用。
用法:
    python -m simulate.cli                           # 默认 MuJoCo + factory
    python -m simulate.cli --world office            # 切换场景
    python -m simulate.cli --headless                # 无头模式
    python -m simulate.cli --scenario navigation     # 导航测试
    python -m simulate.cli --scenario semantic_nav   # 语义导航测试
"""
import argparse
import sys
import time
from pathlib import Path
from typing import Optional


# ── 颜色常量 (与 lingtu CLI 一致) ────────────────────────────────────────────
_R  = "\033[0;31m"
_G  = "\033[0;32m"
_Y  = "\033[1;33m"
_C  = "\033[0;36m"
_B  = "\033[1m"
_D  = "\033[2m"
_M  = "\033[0;35m"
_N  = "\033[0m"

_OK   = f"{_G}✓{_N}"
_FAIL = f"{_R}✗{_N}"
_WARN = f"{_Y}!{_N}"
_DOT  = f"{_C}›{_N}"
_RUN  = f"{_G}▶{_N}"


def _log_info(msg: str):
    print(f"  {_DOT} {msg}")

def _log_ok(msg: str):
    print(f"  {_OK} {msg}")

def _log_warn(msg: str):
    print(f"  {_WARN} {msg}")

def _log_fail(msg: str):
    print(f"  {_FAIL} {msg}")

def _log_run(msg: str):
    print(f"  {_RUN} {_B}{msg}{_N}")


def _config_panel(*lines):
    """打印配置摘要面板 (与 lingtu CLI 风格一致)."""
    print(f"  {_C}┌──────────────────────────────┐{_N}")
    for line in lines:
        # 计算可见字符宽度用于对齐
        print(f"  {_C}│{_N} {line:<35}{_C}│{_N}")
    print(f"  {_C}└──────────────────────────────┘{_N}")
    print()


def _mini_banner():
    print(f"  {_C}灵途仿真{_N} {_D}Simulate{_N}")
    print()


# ── 场景工厂 ─────────────────────────────────────────────────────────────────

def _build_scenario(scenario_name: Optional[str], scenario_args: dict):
    """根据名称实例化场景对象"""
    if scenario_name is None:
        return None

    from sim.engine.scenarios.navigation import NavigationScenario
    from sim.engine.scenarios.semantic_nav import SemanticNavScenario

    if scenario_name == "navigation":
        return NavigationScenario(
            start=scenario_args.get("start", (0.0, 0.0, 0.35)),
            goal=scenario_args.get("goal", (10.0, 5.0)),
            goal_radius=scenario_args.get("goal_radius", 1.0),
            max_time=scenario_args.get("max_time", 120.0),
        )
    elif scenario_name == "semantic_nav":
        return SemanticNavScenario(
            instruction=scenario_args.get("instruction", "导航到目标区域"),
            target_labels=scenario_args.get("target_labels", []),
            target_positions=scenario_args.get("target_positions", []),
            goal_radius=scenario_args.get("goal_radius", 2.0),
            max_time=scenario_args.get("max_time", 180.0),
        )
    else:
        raise ValueError(
            f"Unknown scenario '{scenario_name}'. "
            f"Available: navigation, semantic_nav"
        )


# ── 引擎工厂 ─────────────────────────────────────────────────────────────────

def _build_engine(engine_name: str, world: str, headless: bool):
    """实例化仿真引擎"""
    if engine_name == "mujoco":
        return _build_mujoco_engine(world, headless)
    else:
        raise ValueError(
            f"Unknown engine '{engine_name}'. Available: mujoco"
        )


def _build_mujoco_engine(world: str, headless: bool):
    """构建 MuJoCo 引擎实例."""
    try:
        import mujoco
    except ImportError:
        _log_fail("MuJoCo 未安装: pip install mujoco")
        sys.exit(1)

    from sim.engine.worlds.registry import get_world
    try:
        world_info = get_world(world)
    except KeyError as e:
        _log_fail(str(e))
        sys.exit(1)

    # 确定 XML 来源
    world_path = world_info.get("path")
    world_xml = world_info.get("xml")

    if world_path is not None and Path(world_path).exists():
        xml_path = str(world_path)
        _log_info(f"加载世界: {xml_path}")
        m = mujoco.MjModel.from_xml_path(xml_path)
    elif world_xml is not None:
        _log_info(f"加载内联世界: {world}")
        m = mujoco.MjModel.from_xml_string(world_xml)
    else:
        _log_fail(f"世界文件不存在: {world_path}")
        sys.exit(1)

    d = mujoco.MjData(m)

    # 包装为统一接口
    engine = _MuJoCoEngine(m, d, headless=headless)
    return engine


# ── MuJoCo 引擎包装 ───────────────────────────────────────────────────────────

class _RobotState:
    """仿真机器人状态 (与 SimEngine 协议一致)."""
    __slots__ = ("position", "quat_wxyz", "linear_velocity", "angular_velocity")

    def __init__(self, pos, quat, vel, omega):
        self.position = pos
        self.quat_wxyz = quat
        self.linear_velocity = vel
        self.angular_velocity = omega


class _MuJoCoEngine:
    """最小化 MuJoCo 引擎，实现 SimEngine 协议。SimROS2Bridge 调用."""

    def __init__(self, model, data, headless: bool = False):
        import mujoco
        self._m = model
        self._d = data
        self._headless = headless
        self._viewer = None
        self._bridge_node = None  # 由 run_simulation 注入

        # cmd_vel 缓存
        self._vx = 0.0
        self._vy = 0.0
        self._wz = 0.0

        if not headless:
            try:
                import mujoco.viewer
                self._viewer = mujoco.viewer.launch_passive(model, data)
            except Exception as e:
                _log_warn(f"MuJoCo viewer 启动失败 (切换无头模式): {e}")

    def get_robot_state(self) -> Optional[_RobotState]:
        import numpy as np
        d = self._d
        pos = np.array(d.qpos[:3], dtype=float)
        quat = np.array(d.qpos[3:7], dtype=float)  # w,x,y,z
        vel = np.array(d.qvel[:3], dtype=float)
        omega = np.array(d.qvel[3:6], dtype=float)
        return _RobotState(pos, quat, vel, omega)

    def get_lidar_points(self):
        """返回简化球形射线扫描点云 (N,4) xyzi."""
        import numpy as np
        import mujoco

        m, d = self._m, self._d
        pos = d.qpos[:3]

        n_rays = 360
        angles = np.linspace(0, 2 * np.pi, n_rays, endpoint=False)
        pts = []
        for a in angles:
            direction = np.array([np.cos(a), np.sin(a), 0.0])
            geom_id = np.array([-1], dtype=np.int32)
            dist = mujoco.mj_ray(
                m, d, pos, direction, None, 1, -1, geom_id
            )
            if 0 < dist < 30.0:
                hit = pos + direction * dist
                pts.append([hit[0], hit[1], hit[2], 100.0])
        if not pts:
            return np.zeros((0, 4), dtype=np.float32)
        return np.array(pts, dtype=np.float32)

    def get_camera_data(self):
        """相机数据 (此引擎不渲染相机，返回 None)."""
        return None

    def apply_velocity(self, vx: float, vy: float, wz: float):
        """将速度指令转换为仿真力 (简化: 直接设置 qvel)."""
        import numpy as np
        self._vx = vx
        self._vy = vy
        self._wz = wz
        # 对自由关节施加速度 (简化仿真，不走 ONNX policy)
        self._d.qvel[0] = vx
        self._d.qvel[1] = vy
        self._d.qvel[5] = wz

    def set_robot_pose(self, x: float, y: float, z: float):
        """设置机器人位置"""
        self._d.qpos[0] = x
        self._d.qpos[1] = y
        self._d.qpos[2] = z
        import mujoco
        mujoco.mj_forward(self._m, self._d)

    def step(self):
        """推进物理仿真一步"""
        import mujoco
        mujoco.mj_step(self._m, self._d)
        if self._viewer is not None:
            self._viewer.sync()

    def is_running(self) -> bool:
        """检查 viewer 是否仍在运行."""
        if self._viewer is None:
            return True
        return self._viewer.is_running()

    def close(self):
        if self._viewer is not None:
            self._viewer.close()


# ── 主入口 ────────────────────────────────────────────────────────────────────

def _launch_nav_stack(enable_semantic: bool = False):
    """在子进程中启动导航栈子系统 (绕过硬件依赖)。
    直接启动 autonomy + planning 子系统，不走 navigation_run.launch.py
    (后者会尝试加载 livox_ros_driver2 等硬件包)。
    Returns:
        list[subprocess.Popen] 或 [] (如果启动失败)
    """
    import subprocess
    import shutil

    ros2_bin = shutil.which("ros2")
    if ros2_bin is None:
        _log_warn("ros2 命令不可用，跳过导航栈启动")
        return []

    # 定位 launch 文件 (相对于 repo root)
    repo_root = Path(__file__).resolve().parent.parent
    autonomy_launch = repo_root / "launch" / "subsystems" / "autonomy.launch.py"
    planning_launch = repo_root / "launch" / "subsystems" / "planning.launch.py"

    # 构建 bash 环境前缀 (source 所有可能的 install 路径)
    env_prefix = (
        "source /opt/ros/humble/setup.bash 2>/dev/null; "
        "source /opt/nav/install/setup.bash 2>/dev/null; "
        f"source {repo_root}/install/setup.bash 2>/dev/null; "
    )

    procs = []

    def _launch(name, launch_file, extra_args=""):
        cmd = f"{env_prefix} ros2 launch {launch_file} {extra_args}"
        _log_info(f"启动 {name}")
        try:
            p = subprocess.Popen(
                ["bash", "-c", cmd],
                stdout=subprocess.DEVNULL, stderr=subprocess.PIPE,
            )
            procs.append(p)
        except Exception as e:
            _log_warn(f"{name} 启动失败: {e}")

    # 1. autonomy (terrain_analysis + local_planner + pathFollower)
    if autonomy_launch.exists():
        _launch("autonomy (terrain + local_planner)", autonomy_launch)
    else:
        _log_warn(f"autonomy.launch.py 不存在: {autonomy_launch}")

    # 2. planning (global_planner + pct_path_adapter, stub profile)
    if planning_launch.exists():
        _launch("planning (stub profile)", planning_launch, "planner_profile:=stub")
    else:
        _log_warn(f"planning.launch.py 不存在: {planning_launch}")

    # 3. semantic (可选)
    if enable_semantic:
        semantic_launch = repo_root / "launch" / "subsystems" / "semantic.launch.py"
        if semantic_launch.exists():
            _launch("semantic planner", semantic_launch)

    if procs:
        _log_ok(f"导航栈已启动 ({len(procs)} 个子系统)")
    else:
        _log_warn("没有导航子系统成功启动")

    return procs


def run_simulation(
    engine: str = "mujoco",
    world: str = "factory",
    scenario: Optional[str] = None,
    headless: bool = False,
    scenario_args: Optional[dict] = None,
    enable_nav: bool = False,
    enable_semantic: bool = False,
):
    """主入口，由 lingtu sim 调用。
    Args:
        engine:        仿真引擎名称 (mujoco)
        world:         场景名称 (factory/open_field/building/empty/spiral)
        scenario:      测试场景名称 (navigation/semantic_nav/None)
        headless:      无头模式 (无 GUI)
        scenario_args: 场景额外参数 (goal, instruction 等)
        enable_nav:    同时启动导航栈 (stub profile)
        enable_semantic: 同时启动语义导航
    """
    _mini_banner()
    _log_run("启动仿真")
    print()

    from sim.engine.worlds.registry import list_worlds
    worlds = list_worlds()
    world_desc = worlds.get(world, world)

    nav_label = "off"
    if enable_semantic:
        nav_label = "semantic (stub)"
        enable_nav = True
    elif enable_nav:
        nav_label = "stub"

    _config_panel(
        f"{_D}引擎{_N}    {engine}",
        f"{_D}场景{_N}    {world}  {_D}({world_desc}){_N}",
        f"{_D}测试{_N}    {scenario or '(手动驾驶)'}"[:40],
        f"{_D}模式{_N}    {'无头' if headless else 'GUI'}",
        f"{_D}导航{_N}    {nav_label}",
    )

    # 构建引擎
    sim_engine = _build_engine(engine, world, headless)

    # 构建测试场景
    test_scenario = _build_scenario(scenario, scenario_args or {})

    # 启动 ROS2 桥接 (线程中运行)
    bridge = None
    bridge_thread = None
    try:
        from sim.engine.bridge.ros2_bridge import SimROS2Bridge
        bridge = SimROS2Bridge(sim_engine)
        # 将 ROS2 节点注入引擎 (供场景使用)
        sim_engine._bridge_node = bridge._node

        import threading
        bridge_thread = threading.Thread(
            target=bridge.run, daemon=True)
        bridge_thread.start()
        _log_ok("ROS2 桥接已启动")
    except ImportError:
        _log_warn("ROS2 不可用，跳过桥接 (仅物理仿真)")
    except Exception as e:
        _log_warn(f"ROS2 桥接启动失败: {e}")

    # 导航栈联动
    nav_procs = []
    if enable_nav:
        nav_procs = _launch_nav_stack(enable_semantic=enable_semantic)

    # 场景初始化
    if test_scenario is not None:
        test_scenario.setup(sim_engine)
        _log_info(f"场景已设置: {test_scenario.name}")

    _log_ok("仿真运行中，Ctrl+C 退出")
    print()

    # 主仿真循环
    try:
        while sim_engine.is_running():
            sim_engine.step()

            if test_scenario is not None:
                if test_scenario.is_complete(sim_engine):
                    test_scenario.teardown(sim_engine)
                    _print_scenario_result(test_scenario, sim_engine)
                    break

            time.sleep(0.002)  # 约 500Hz 最快
    except KeyboardInterrupt:
        print()
        _log_info("用户中止仿真")
    finally:
        if nav_procs:
            _log_info("停止导航栈...")
            for p in nav_procs:
                p.terminate()
            for p in nav_procs:
                try:
                    p.wait(timeout=5)
                except Exception:
                    p.kill()
        if bridge is not None:
            bridge.stop()
        sim_engine.close()
        _log_ok("仿真结束")


def _print_scenario_result(scenario, engine):
    """打印场景测试结果."""
    success = scenario.is_success(engine)
    metrics = scenario.get_metrics()

    print()
    print(f"  {_B}测试结果{_N}")
    if success:
        print(f"  {_OK} {_G}PASS{_N}  {scenario.description}")
    else:
        print(f"  {_FAIL} {_R}FAIL{_N}  {scenario.description}")

    print()
    print(f"  {_B}指标{_N}")
    for k, v in metrics.items():
        if k not in ("name", "success"):
            print(f"    {_D}{k:<25}{_N} {v}")
    print()


# ── 命令行解析 (直接调用) ───────────────────────────────────────────────────

def _parse_args(argv=None):
    p = argparse.ArgumentParser(
        prog="lingtu sim",
        description="灵途仿真平台，在仿真环境中测试导航栈",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  lingtu sim                          默认 MuJoCo + factory 场景
  lingtu sim --world open_field       切换为开阔地场景
  lingtu sim --headless               无头模式 (无 GUI)
  lingtu sim --scenario navigation    A→B 导航测试
  lingtu sim --scenario semantic_nav --instruction "导航到大门"
        """,
    )
    p.add_argument("--engine", default="mujoco",
                   choices=["mujoco"],
                   help="仿真引擎 (默认: mujoco)")
    p.add_argument("--world", default="factory",
                   help="场景名称: factory/open_field/building/empty/spiral (默认: factory)")
    p.add_argument("--headless", action="store_true",
                   help="无头模式，不启动 GUI")
    p.add_argument("--scenario", default=None,
                   choices=["navigation", "semantic_nav"],
                   help="测试场景 (默认: 手动驾驶模式)")
    # navigation 场景参数
    p.add_argument("--goal", default=None,
                   help="导航目标坐标 (格式: x,y 如 '10,5')")
    p.add_argument("--goal-radius", type=float, default=1.0,
                   help="到达判定半径 m (默认: 1.0)")
    p.add_argument("--max-time", type=float, default=120.0,
                   help="最大测试时间 s (默认: 120)")
    # semantic_nav 场景参数
    p.add_argument("--instruction", default="导航到目标区域",
                   help="语义导航指令 (默认: '导航到目标区域')")
    # 导航栈联动
    p.add_argument("--nav", action="store_true",
                   help="同时启动导航栈 (stub profile, 无需硬件)")
    p.add_argument("--semantic", action="store_true",
                   help="同时启动语义导航 (含 --nav)")
    # 工具命令
    p.add_argument("--list-worlds", action="store_true",
                   help="列出所有可用场景")
    return p.parse_args(argv)


def main(argv=None):
    args = _parse_args(argv)

    if args.list_worlds:
        from sim.engine.worlds.registry import list_worlds
        print(f"\n  {_B}可用场景{_N}")
        for name, desc in list_worlds().items():
            print(f"    {_DOT} {_G}{name:<15}{_N} {_D}{desc}{_N}")
        print()
        return

    scenario_args = {
        "max_time": args.max_time,
        "goal_radius": args.goal_radius,
        "instruction": args.instruction,
    }
    if args.goal:
        try:
            parts = args.goal.split(",")
            scenario_args["goal"] = (float(parts[0]), float(parts[1]))
        except (ValueError, IndexError):
            _log_fail(f"--goal 格式错误: '{args.goal}'，应为 'x,y' 如 '10,5'")
            sys.exit(1)

    run_simulation(
        engine=args.engine,
        world=args.world,
        scenario=args.scenario,
        headless=args.headless,
        scenario_args=scenario_args,
        enable_nav=args.nav,
        enable_semantic=args.semantic,
    )


if __name__ == "__main__":
    main()
