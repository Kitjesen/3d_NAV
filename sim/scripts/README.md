# sim/scripts — 仿真脚本索引

> **为什么是索引而不是子目录**：这些脚本的路径已是事实上的契约——
> `cli/profiles_data.py` 的 `_external_launcher`、`simulation_contract.py`、
> `test_profile_graph_snapshots.py` 的断言、以及多个 `from sim.scripts.xxx import`
> 都硬编码了 `sim/scripts/<name>`。物理移动会触发 30+ 处跨 profile/契约/测试/CI 的
> 连锁修改，成本远超收益。因此用**命名前缀 + 本索引**表达分类，保持路径稳定。
>
> 约定：新脚本按前缀命名（`*_gate.py` / `*_validation.py` / `run_*`,`demo_*` / `launch_*.sh`）。

## 1. CI 门禁 `*_gate.py`（可被 CI 当作通过/失败闸门）

- `cmu_unity_runtime_gate.py` / `cmu_unity_sim_gate.py` — CMU Unity 仿真运行时门禁
- `dynamic_obstacle_local_planner_gate.py` — 动态障碍下局部规划门禁
- `fastlio2_rosbag_replay_gate.py` — Fast-LIO2 rosbag 回放门禁
- `fastlio_speed_boundary_gate.py` — Fast-LIO 速度边界门禁
- `gateway_goal_dry_run_gate.py` — Gateway 目标 dry-run 门禁
- `gazebo_runtime_gate.py` — Gazebo 运行时门禁
- `large_loop_closure_gate.py` — 大回环闭合门禁
- `moving_obstacle_sweep_gate.py` — 移动障碍扫掠门禁
- `mujoco_fastlio2_live_gate.py` — MuJoCo + Fast-LIO2 live 门禁
- `native_pct_mujoco_gate.py` — 原生 PCT + MuJoCo 门禁
- `pct_saved_map_navigation_gate.py` — PCT 存图导航门禁
- `routecheck_preflight_gate.py` — 路线预检门禁
- `saved_map_relocalize_contract_gate.py` / `saved_map_relocalize_runtime_gate.py` — 存图重定位门禁
- `server_sim_closure.py` — 服务端仿真闭环门禁

## 2. 验证 / 诊断 `*_validation` / `*_diagnosis`

- `large_terrain_nav_validation.py` — 大地形导航验证（被 `test_large_terrain_scenario` 引用）
- `large_loop_diagnosis_matrix.py` — 大回环诊断矩阵
- `multifloor_nav_validation.py` — 多楼层导航验证
- `render_slam_validation_screenshots.py` — SLAM 验证截图渲染
- `run_slam_dataset_test_v2.py` — SLAM 数据集测试
- `cmu_unity_tomogram_capture.py` — CMU Unity tomogram 采集

## 3. 演示 / 运行入口 `run_*` / `demo_*` / `view_*` / `record_*`

- `run_sim.py` — 通用仿真启动
- `run_person_following.py` — 人物跟随
- `run_semantic_full_stack.py` — 语义全栈
- `demo_search.py` — 搜索演示
- `policy_nav_smoke.py` — policy 导航冒烟
- `go1_indoor_nav.py` / `go1_nav_full.py` — Go1 室内/全栈导航
- `nav_overlay.py` — 导航叠加可视化
- `view_scene.py` — 场景查看
- `benchmark_following.py` — 跟随基准
- `record_policy_nav_video.py` / `render_gazebo_frontier_video.py` — 录像/渲染

## 4. 启动脚本 `*.sh`

- `launch_mujoco_fastlio2_live.sh` — MuJoCo + Fast-LIO2 live（profile `sim_mujoco_live` 的 `_external_launcher`）
- `launch_cmu_unity_lingtu_runtime.sh` / `launch_cmu_unity_baseline.sh` — CMU Unity 运行时/基线
- `launch_lingtu_gazebo_industrial_demo.sh` — Gazebo 工业园 demo
- `test_*.sh`（`test_fullloop` / `test_semantic_nav` / `test_slam_datasets` / `test_slam_live` / `test_viz_*` / `test_factory_nova`）— 集成验证脚本
- `install_deps.sh` — 依赖安装
- `fastlio_speed_scan_plan.sh` / `run_legkilo_test.sh` — 速度扫描 / legkilo 数据集

## 5. 工具 / 数据

- `algorithm_dataflow_summary.py` — 算法数据流摘要（被 `test_algorithm_dataflow_summary` 引用）
- `rosbag_slam_bridge_replay.py` — rosbag → SLAM bridge 回放
