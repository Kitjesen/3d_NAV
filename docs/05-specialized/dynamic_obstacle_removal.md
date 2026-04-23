# 建图动态障碍物去除

> Live-mapping 阶段把"正在移动的人/物"从累积地图中剔除,使保存的底图
> 只含静态几何。

## 问题

当前建图链路 `Fast-LIO2 → SlamBridge → Gateway._on_map_cloud` 只做 0.15m
voxel 去重,**不做动态过滤**。人慢走穿过走廊,每个位置的体素各被占一
次,保存下来就是一条人形蠕虫拖尾。

Fast-LIO2 是纯几何 LIO,不区分静态/动态点;Localizer refine 只做 ICP 收敛,
也不删点。

## 现状基线 (v1.7.5)

```python
# gateway_module.py:864-872  (mapping 分支)
combined = np.concatenate([self._map_points, pts])
combined = _voxel_downsample(combined, 0.15)  # 只去重,不去动态
self._map_points = combined
```

## 选型对比

| 方法 | 机构/年份 | 模式 | 速度/帧 | 室内 | 集成难度 |
|---|---|---|---|---|---|
| **Dynablox** | ETH-ASL RA-L'23 | online | 58ms | ★★★★★ | 高 (ROS1 only) |
| **DUFOMap** | KTH RA-L'24 | online+offline | 0.06ms | ★★★☆☆ (稀疏 LiDAR 掉点) | 中 (pip 无 aarch64 wheel,要源码编) |
| **BeautyMap** | HKUST RA-L'24 | offline only | 中 | ★★★★☆ | 中 (保存时批处理) |
| **ERASOR** | KAIST ICRA'21 | offline | 中 | ★★★☆☆ (偏室外) | 中 |
| **Hit-count voting** | 自研 | online | <1ms | ★★★☆☆ | **极低 (30 行改 Gateway)** |

### 选型决策

**不做 Dynablox**: ROS1 Noetic only,S100P 是 ROS2 Humble,port 成本远大于收益。

**目标组合**: `Hit-count voting (实时) + DUFOMap (保存前 offline refine)`。

- Hit-count voting: 建图过程中 Web 看到的就是过滤后的,低延迟无依赖
- DUFOMap: 保存时再跑一遍,静态地图精度到 SOTA

## 三阶段路线

### Phase 1 — Hit-count voting (~1 天,立即上线)

在 `Gateway._on_map_cloud` mapping 分支加体素命中计数:

```python
# 每个体素 key → observation count
voxel_hits: Dict[Tuple[int,int,int], int]

# 来一帧:
for voxel_key in voxelize(pts):
    voxel_hits[key] += 1

# 查询/保存时过滤:
stable = {k for k,c in voxel_hits.items() if c >= MIN_HITS}
```

参数:
- `MIN_HITS = 5` (起始) — 墙被扫 30-100 次,人走过每位置 1-3 次
- `voxel_size = 0.15m` — 匹配现有下采样
- 建图过程每 N 秒清一次 `hit = 1` 的点 (可选,降内存)

**局限**:
- 需要机器人移动多帧才有效 (静置站立建图没用)
- 人长时间停留在一处会被认成"静态"
- 慢速动态物体 (如慢走的狗) 可能漏网

**收益**: 典型场景 (人快步走过走廊) 去除率 70-85%。

### Phase 2 — DUFOMap 保存时后处理 **✅ 已上线 (commit 7871561 + a444018)**

原计划依赖 Python 绑定 (`pip install dufomap`), 但**aarch64 没预编译 wheel**。
实际落地:

**2.1 C++ binary 替代 Python 绑定**

直接编 DUFOMap 的 `dufomap_run` C++ 可执行, Gateway 走 subprocess 调用。
完全脱 ROS, 零新 rclpy 依赖。

**2.2 PGO patches 作为输入源** (不是累积 map.pcd)

发现关键点: DUFOMap 做 ray-casting 需要 **每帧位姿 + 每帧激光**, 不能只
读累积地图。PGO 已经 dump 了这两个:
- `<map>/patches/*.pcd` — 每关键帧 body-frame 激光 (~1500 点)
- `<map>/poses.txt` — 每行 `patch.pcd tx ty tz qw qx qy qz`

`dynamic_filter.refilter_map` 的流程:
```
1. 读 poses.txt 建 dict
2. 对每个 patch.pcd: 读点云 + 重写 VIEWPOINT = 对应 pose
3. 写入 tmpdir/pcd/*.pcd (DUFOMap 期望的目录结构)
4. subprocess.run(dufomap_run, tmpdir, config.toml)
5. 读 tmpdir/dufomap_output.pcd → 备份原 map.pcd → 覆盖
```

**2.3 集成点**: `MapManager._map_save` + `Gateway /api/v1/map/save` 两条路径

Web 按钮走 `/api/v1/map/save` (line 2628), MCP/程序调 `/api/v1/maps action=save`。
**两处都插了 Step 1½**, 保证不管走哪条路径都过 DUFOMap。

**2.4 幂等构建脚本 `scripts/build_dufomap.sh`**

从裸 aarch64 Ubuntu 22.04 跑到 `dufomap_run` 可执行一条命令:
- apt 装 libtbb-dev + liblz4-dev (+ wget fallback 装 liblzf-dev,
  因为清华代理挂的时候直下 .deb)
- git clone recursive
- patch ufomap 三个 header 的 `#include <immintrin.h>` 加 `__BMI2__`
  守卫 (aarch64 没 x86 intrinsics)
- cmake + build

**2.5 Lingtu 专用 DUFOMap 调参** `config/dufomap.toml`

默认为 KITTI 64 线密激光调的, Livox Mid-360 稀疏, 放松阈值:
| 参数 | 默认 | Lingtu |
|---|---|---|
| `inflate_unknown` | 1 | **2** |
| `inflate_hits_dist` | 0.2 | **0.15** |
| `min_range` | 0.2 | **0.3** (过滤狗自己腿) |
| `max_range` | -1 | **30.0** (Livox 远端噪点多) |

**2.6 性能实测 (corrected_20260406_224020, 105 patches × ~1600 pts)**
- 总处理时间: **0.6s** (aarch64, DUFOMap 内部 OpenMP)
- Python wrapper 开销 (scp repack + subprocess IPC): 额外 ~0.1s
- 静态场景删除率: 0.01% (预期, 没动态物体可删)
- 有动态物体场景: 待验证 (Task #23)

### Phase 3 — 实时 DUFOMap (~1 周,可选)

将 DUFOMap 搬进 Gateway `_on_map_cloud` 实时管线,每帧调 `dufo.run`,
Web 看到的就是 DUFOMap 过滤版。

**风险**:
- Mid-360 非重复扫描单帧 ~2k 点,稀疏度可能触及 DUFOMap 74% AA 下限
- 需要 RTF 实测,如果效果不如 Phase 1 + Phase 2 组合,此阶段放弃

## 验收标准

- [x] Phase 1 上线 (commit b4530d5)
- [x] Phase 2 上线 (commit 7871561 + a444018)
- [x] DUFOMap binary aarch64 编译通过 (~/src/dufomap/build/dufomap_run)
- [x] 离线测试通过 (105 patches, 0.6s, -23 pts on static map)
- [x] Lingtu config 参数生效
- [ ] **Task #23**: 有动态物体的新图实测, 残影去除 >= 60%
- [ ] localizer 加载新 PCD 后 ICP fitness 不退化
- [ ] Gateway CPU 峰值 < 原值 +30% (保存瞬间)

## 关键文件

| 文件 | 角色 |
|---|---|
| `src/gateway/gateway_module.py:_on_map_cloud` | Phase 1: mapping 分支 voxel_hits 投票 |
| `src/gateway/gateway_module.py:/api/v1/map/save` | Phase 2: Web 按钮直调路径, Step 1½ DUFOMap |
| `src/nav/services/nav_services/map_manager_module.py:_map_save` | Phase 2: MCP 路径, Step 1½ DUFOMap |
| `src/nav/services/nav_services/dynamic_filter.py` | DUFOMap wrapper + patches repack + 备份/覆盖 |
| `scripts/build_dufomap.sh` | 幂等 aarch64 构建脚本 |
| `scripts/dufomap_offline_test.py` | 独立验证脚本 |
| `config/dufomap.toml` | Lingtu 专用 DUFOMap 调参 |
| `~/src/dufomap/build/dufomap_run` | S100P 上实际执行的 C++ binary |

## 环境变量

| 变量 | 默认 | 含义 |
|---|---|---|
| `LINGTU_MAP_MIN_HITS` | 3 | Phase 1 voxel 投票阈值 |
| `LINGTU_SAVE_DYNAMIC_FILTER` | 1 | Phase 2 总开关 (=0 关闭) |
| `LINGTU_DUFOMAP_BIN` | `~/src/dufomap/build/dufomap_run` | DUFOMap 可执行路径 |
| `LINGTU_DUFOMAP_CONFIG` | `<repo>/config/dufomap.toml` | DUFOMap 配置路径 |

## 参考

- [Dynablox (ETH-ASL RA-L'23)](https://github.com/ethz-asl/dynablox)
- [DUFOMap (KTH RA-L'24)](https://github.com/KTH-RPL/dufomap)
- [BeautyMap (HKUST RA-L'24)](https://github.com/MKJia/BeautyMap)
- [KTH DynamicMap_Benchmark (ITSC'23)](https://github.com/KTH-RPL/DynamicMap_Benchmark)
- [Awesome-LiDAR-Mapping curated list](https://github.com/hwan0806/Awesome-LiDAR-Mapping)
