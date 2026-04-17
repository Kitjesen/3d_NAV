# LingTu 回归流程

三层回归,覆盖从本地 commit 到 S100P 真机的全链路。

| 层 | 触发 | 内容 | 耗时 | 强制 |
|----|------|------|------|------|
| **L1** `pre-commit` hook | `git commit` | `pytest src/core/tests/ -q` 必绿 | ~90s | ✅ |
| **L2** `pre-push` hook | `git push` | L1 + `stub profile` 冒烟 build | ~30s 额外 | ✅ |
| **L3** S100P 每周 | 周五下班前人工 | 跑 `p0_*.sh` 4 个脚本 + 录视频 | ~30 分钟 | ✅ |

---

## L1 / L2 本地 hooks 安装

```bash
# 一次性安装到仓库 (`.git/hooks/` 仓库本地,不入版本)
bash docs/07-testing/install_hooks.sh
```

安装后:
- `git commit` 前自动跑 `pytest src/core/tests/ -q`,失败则 commit 被阻止
- `git push` 前再跑一次 pytest + `full_stack_blueprint(profile="stub")` 能 `build().start()`

**绕过**(紧急情况):
```bash
git commit --no-verify -m "..."
git push --no-verify
```
但必须在 PR 描述里说明原因,审核者会挑战。

---

## L3 真机 P0 脚本

每个 P0 脚本自成一体,`set -e`,日志写入 `~/data/nav_logs/YYYYMMDD_HHMMSS_<script>.log`。

| 脚本 | 覆盖 | 输入 | 通过判据 |
|------|------|------|---------|
| `p0_cold_boot.sh` | 系统冷启动 | 无 | 20+ modules OK + Gateway 5050 返回 `health.status="ok"` 并稳定 3 分钟 |
| `p0_mapping.sh` | 建图 → 保存 → 激活 | 人工手持走一圈 | 出 `map.pcd` + `tomogram.pickle` + `occupancy.npz` + `map.pgm/map.yaml` |
| `p0_goto.sh` | 点目标 → 到达 | 提前激活好一张图 | Dashboard 发 goal,`mission_status=SUCCESS` 或 `EV_PATH_COMPLETE` 事件 |
| `p0_estop.sh` | 紧急停止反射 | 人工按 Dashboard E-stop 按钮 | 100 ms 内 CmdVelMux 输出 `Twist.zero()`,log 有 watchdog 记录 |

### 运行方式(Sunrise SSH)
```bash
ssh sunrise@192.168.66.190
cd ~/data/SLAM/navigation
git pull --ff-only origin main
bash docs/07-testing/p0_all.sh | tee ~/data/nav_logs/$(date +%Y%m%d_%H%M%S)_p0_all.log
```

跑完:
1. 结果记录到 `vault/实机记录/YYYY-MM-DD.md`(PASS / FAIL / BLOCKED + 原因)
2. `BACKLOG.md` 里对应条目更新状态 + 最近真机日期
3. 失败 case 单独在 `docs/08-known-issues/` 开 issue 追踪

---

## 禁忌

- ❌ SSH 里裸敲命令跑验证 → 必须走脚本
- ❌ 跳过 pre-commit / pre-push → 除非紧急 + PR 说明
- ❌ BACKLOG 外开 PR → 新条目先入表
- ❌ 长期 `blocked` 不跟踪 → 每周 L3 必须刷新状态

---

## 脚本清单

- `install_hooks.sh` — 安装 L1/L2 本地 git hooks
- `p0_cold_boot.sh` — P0-01 冷启动
- `p0_mapping.sh` — P0-02 建图
- `p0_goto.sh` — P0-03 导航
- `p0_estop.sh` — P0-04 紧急停止
- `p0_all.sh` — 串行跑完 P0-01 到 P0-04
- `p1_tare_explore.sh` — P1-01 TARE 探索(未写,后续补)
- `p1_gnss_fusion.sh` — P1-02 GNSS 融合(未写)
- `p1_follow_person.sh` — P1-04 OSNet 跟人(未写)
- `p1_degraded.sh` — P1-05 退化(未写)
