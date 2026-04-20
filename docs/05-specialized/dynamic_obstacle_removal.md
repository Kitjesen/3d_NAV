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

### Phase 2 — DUFOMap 保存时后处理 (~2 天)

**2.1 aarch64 编译**

S100P 依赖:
- gcc-10 / g++-10 (Ubuntu 22.04 默认 11,需降级或并装)
- libtbb-dev, liblz4-dev
- pybind11 / Python 3.10 dev headers

```bash
sudo apt install gcc-10 g++-10 libtbb-dev liblz4-dev python3-dev
git clone https://github.com/KTH-RPL/dufomap /opt/dufomap
cd /opt/dufomap && pip install . --no-binary=:all:
```

**2.2 集成点**: `Gateway._save_current_map()` (建图结束保存前)

```python
from dufomap import dufomap
dufo = dufomap(resolution=0.15, d_s=0.2, d_p=2, num_threads=4)
for pose, frame_pts in accumulated_frames:
    dufo.run(frame_pts, pose, cloud_transform=False)
clean_pts = np.empty_like(self._map_points)
dufo.outputMap(clean_pts, voxel_map=False)
save_pcd(clean_pts)
```

**关键前提**: 需要保留每帧的原始点云 + pose,不能只保留累积云。这要求
`SlamBridge` 额外缓存 `(ts, pose, pts)` 滑窗 (~N=最近 1000 帧),占用内存
可控 (1000 × 2k 点 × 12B = 24MB)。

### Phase 3 — 实时 DUFOMap (~1 周,可选)

将 DUFOMap 搬进 Gateway `_on_map_cloud` 实时管线,每帧调 `dufo.run`,
Web 看到的就是 DUFOMap 过滤版。

**风险**:
- Mid-360 非重复扫描单帧 ~2k 点,稀疏度可能触及 DUFOMap 74% AA 下限
- 需要 RTF 实测,如果效果不如 Phase 1 + Phase 2 组合,此阶段放弃

## 验收标准

- [ ] Phase 1 完成后,人走动 30s 留下的残影 <= 原值 20%
- [ ] 保存的 PCD 静态几何 (墙/家具/地面) 完整度 >= 95%
- [ ] Phase 2 完成后,保存的 PCD 肉眼零重影
- [ ] localizer 加载新 PCD 后 ICP fitness 与旧值相当 (<0.1)
- [ ] 不增加 Gateway CPU >10% (Phase 1) / >30% (Phase 2 保存瞬间峰值)

## 关键文件

| 文件 | 角色 |
|---|---|
| `src/gateway/gateway_module.py:837` | `_on_map_cloud` 入口 |
| `src/gateway/gateway_module.py:864` | mapping 累积分支 |
| `src/slam/slam_bridge_module.py` | 上游 pose + pts 原始帧 |
| `docs/05-specialized/dynamic_obstacle_removal.md` | 本文档 |

## 参考

- [Dynablox (ETH-ASL RA-L'23)](https://github.com/ethz-asl/dynablox)
- [DUFOMap (KTH RA-L'24)](https://github.com/KTH-RPL/dufomap)
- [BeautyMap (HKUST RA-L'24)](https://github.com/MKJia/BeautyMap)
- [KTH DynamicMap_Benchmark (ITSC'23)](https://github.com/KTH-RPL/DynamicMap_Benchmark)
- [Awesome-LiDAR-Mapping curated list](https://github.com/hwan0806/Awesome-LiDAR-Mapping)
