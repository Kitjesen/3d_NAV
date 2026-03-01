/**
 * test_param_sensitivity.cpp — nav_core 参数敏感性测试
 *
 * R6: 验证关键参数在边界值/扫描值下的行为，防止调参引入回归。
 *
 * 测试覆盖:
 *   TEST(PathFollower, DirDiffThreVariation)  — dirDiffThre 三档扫描
 *   TEST(LocalPlanner, SlopeWeightImpact)     — slopeWeight 四档影响
 *   TEST(PctAdapter,  StuckBoundaryExact)     — stuckTimeoutSec 精确边界
 *   TEST(PctAdapter,  ReplanCooldownDebounce) — 冷却期内多次 stuck 不叠加
 */

#include <gtest/gtest.h>
#include "nav_core/path_follower_core.hpp"
#include "nav_core/local_planner_core.hpp"
#include "nav_core/pct_adapter_core.hpp"
#include <cmath>

using namespace nav_core;

// ═══════════════════════════════════════════════════
//  TEST 1: PathFollower — dirDiffThre 参数敏感性
// ═══════════════════════════════════════════════════

// 辅助：给定 dirDiffThre，构造角度误差恰好在阈值处的场景，
// 验证 canAccel 在阈值内/外的翻转行为。
static bool canAccelWithDirDiff(double dirDiffThre, double dirDiffRad) {
  PathFollowerParams p;
  p.dirDiffThre    = dirDiffThre;
  p.baseLookAheadDis = 0.3;
  p.maxSpeed       = 1.0;
  p.maxAccel       = 1.0;
  p.stopDisThre    = 0.05;
  p.omniDirGoalThre = 0.1; // goal proximity threshold (small — not near goal)
  p.twoWayDrive    = false;
  p.noRotAtGoal    = false;

  PathFollowerState state;

  // 构造路径: 机器人在原点, 路径沿正 X 轴延伸
  std::vector<Vec3> path = { {0.5, 0, 0}, {1.0, 0, 0}, {2.0, 0, 0} };

  // vehicleRel = (0, 0, 0), vehicleYawDiff = dirDiffRad
  // => dirDiff ≈ dirDiffRad (当 pathDir ≈ 0)
  Vec3 vehicleRel{0, 0, 0};

  auto out = computeControl(vehicleRel, dirDiffRad, path,
                            /*joySpeed=*/1.0, /*currentTime=*/0.0,
                            /*slowFactor=*/1.0, /*safetyStop=*/0,
                            p, state);
  return out.canAccel;
}

TEST(PathFollower, DirDiffThreVariation) {
  // 三档 dirDiffThre: 0.05 rad (~2.9°), 0.1 rad (~5.7°), 0.2 rad (~11.5°)
  const double thresholds[] = {0.05, 0.1, 0.2};

  for (double thre : thresholds) {
    // 角度误差 = 0.5 * thre → 远小于阈值 → canAccel = true
    EXPECT_TRUE(canAccelWithDirDiff(thre, thre * 0.5))
        << "dirDiffThre=" << thre << ": 0.5*thre should allow accel";

    // 角度误差 = 1.5 * thre → 超过阈值 → canAccel = false (not near goal)
    EXPECT_FALSE(canAccelWithDirDiff(thre, thre * 1.5))
        << "dirDiffThre=" << thre << ": 1.5*thre should block accel";
  }

  // 单调性: 更大的 dirDiffThre 允许更大的角度误差通过
  //   dirDiff = 0.12 rad, thre=0.1 → blocked; thre=0.2 → allowed
  EXPECT_FALSE(canAccelWithDirDiff(0.1, 0.12));
  EXPECT_TRUE (canAccelWithDirDiff(0.2, 0.12));
}

// ═══════════════════════════════════════════════════
//  TEST 2: LocalPlanner — slopeWeight 参数敏感性
// ═══════════════════════════════════════════════════

TEST(LocalPlanner, SlopeWeightImpact) {
  // 固定: dirDiff=0(正前方), rotDirW=1.0, groupDirW=1.0, terrainPenalty=0.3
  // relativeGoalDis=10.0 (远离目标, 使用 rotDirW 分支)
  const double dirDiffDeg       = 0.0;
  const double rotDirW          = 1.0;
  const double groupDirW        = 1.0;
  const double terrainPen       = 0.3;
  const double relativeGoalDis  = 10.0;

  PathScoreParams p;
  p.dirWeight      = 0.02;    // default
  p.omniDirGoalThre = 5.0;   // default; relativeGoalDis > 5 → rotDirW 分支

  const double slopeWeights[] = {0.0, 1.0, 3.0, 6.0};
  double prevScore = 1e9;

  for (double sw : slopeWeights) {
    p.slopeWeight = sw;
    double score = scorePath(dirDiffDeg, rotDirW, groupDirW,
                             terrainPen, relativeGoalDis, p);

    // 分数非负
    EXPECT_GE(score, 0.0) << "slopeWeight=" << sw << ": score must be >= 0";

    // slopeWeight=0 → 地形无惩罚 → 最高分 (相对其余档)
    if (sw == 0.0) {
      prevScore = score;
    } else {
      // 随 slopeWeight 增大，分数应单调下降（或相等）
      EXPECT_LE(score, prevScore)
          << "slopeWeight=" << sw << ": should not exceed score at lower weight";
      prevScore = score;
    }
  }

  // slopeWeight=6 时 terrainFactor = max(0, 1 - 6*0.3) = max(0, -0.8) = 0
  // => score = (1 - √√(0.02*0)) * rotDirW^4 * 0 = 0
  p.slopeWeight = 6.0;
  EXPECT_NEAR(scorePath(dirDiffDeg, rotDirW, groupDirW,
                        terrainPen, relativeGoalDis, p), 0.0, 1e-9)
      << "slopeWeight=6, terrainPenalty=0.3 → terrain clamped to 0 → score=0";
}

// ═══════════════════════════════════════════════════
//  TEST 3: PctAdapter — stuckTimeoutSec 精确边界
// ═══════════════════════════════════════════════════

TEST(PctAdapter, StuckBoundaryExact) {
  WaypointTrackerParams p;
  p.stuckTimeoutSec   = 10.0;
  p.replanCooldownSec = 0.001; // 极小冷却: 初始 lastReplanTime_=-1, 所以
                                // cooldown check = t-(-1) > 0.001 → 几乎总满足
  p.maxReplanCount    = 2;
  p.waypointDistance  = 0.5;
  p.arrivalThreshold  = 0.5;
  p.searchWindow      = 5;

  WaypointTracker tracker(p);

  // 设置一条远离机器人的路径 (机器人在原点, 路径在 x=10..14)
  Path path;
  for (int i = 0; i < 5; ++i) {
    Pose pose;
    pose.position = Vec3{static_cast<double>(i + 10), 0, 0};
    path.push_back(pose);
  }
  // setPath at t=1.0 → lastProgressTime_=1.0 > 0 (stuck detection enabled)
  // initial lastReplanTime_=-1.0 → cooldown check: t-(-1) > 0.001 → satisfied
  tracker.setPath(path, 1.0);

  Vec3 robotPos{0, 0, 0};
  std::vector<Vec3> empty_odom;

  // t = 10.999 — elapsed = 9.999s < timeout=10s → 不触发 stuck
  auto r1 = tracker.update(robotPos, empty_odom, 10.999);
  EXPECT_NE(r1.event, WaypointEvent::kReplanning)
      << "elapsed=9.999s: should NOT trigger stuck (timeout=10s)";
  EXPECT_NE(r1.event, WaypointEvent::kStuckFinal)
      << "elapsed=9.999s: should NOT be stuck final";

  // t = 11.001 — elapsed = 10.001s > timeout=10s → 触发第1次重规划
  auto r2 = tracker.update(robotPos, empty_odom, 11.001);
  EXPECT_EQ(r2.event, WaypointEvent::kReplanning)
      << "elapsed=10.001s: should trigger kReplanning (elapsed > stuckTimeoutSec)";
}

// ═══════════════════════════════════════════════════
//  TEST 4: PctAdapter — replanCooldownSec 防抖
// ═══════════════════════════════════════════════════

TEST(PctAdapter, ReplanCooldownDebounce) {
  WaypointTrackerParams p;
  p.stuckTimeoutSec   = 5.0;
  p.replanCooldownSec = 10.0;  // 冷却10s
  p.maxReplanCount    = 3;     // 最多3次重规划，给足机会
  p.waypointDistance  = 0.5;
  p.arrivalThreshold  = 0.5;
  p.searchWindow      = 5;

  WaypointTracker tracker(p);

  Path path;
  for (int i = 0; i < 5; ++i) {
    Pose pose;
    pose.position = Vec3{static_cast<double>(i + 20), 0, 0};
    path.push_back(pose);
  }
  // setPath at t=1.0 → lastProgressTime_=1.0 > 0 (stuck detection enabled)
  // initial lastReplanTime_=-1.0
  // 第1次重规划需满足: elapsed > stuckTimeout=5 AND t-(-1) > cooldown=10
  //   → t > 6.0 AND t > 9.0 → t > 9.0
  tracker.setPath(path, 1.0);

  Vec3 robotPos{0, 0, 0};
  std::vector<Vec3> empty_odom;

  // t = 10.0: elapsed=9.0 > 5.0 ✓; cooldown: 10.0-(-1.0)=11.0 > 10.0 ✓ → kReplanning
  // After replan: lastProgressTime_ = lastReplanTime_ = 10.0
  auto r1 = tracker.update(robotPos, empty_odom, 10.0);
  EXPECT_EQ(r1.event, WaypointEvent::kReplanning)
      << "t=10s: first replan should fire (elapsed=9 > 5, cooldown=11 > 10)";

  // t = 12.0: elapsed=12-10=2s < stuckTimeout=5 → 不触发
  auto r2 = tracker.update(robotPos, empty_odom, 12.0);
  EXPECT_EQ(r2.event, WaypointEvent::kNone)
      << "t=12s: elapsed=2s < stuckTimeout, should NOT replan";

  // t = 17.0: elapsed=17-10=7s > stuckTimeout=5
  //   BUT cooldown: 17-10=7s < cooldown=10s → 不触发
  auto r3 = tracker.update(robotPos, empty_odom, 17.0);
  EXPECT_EQ(r3.event, WaypointEvent::kNone)
      << "t=17s: cooldown not expired (17-10=7 < 10), should NOT replan";

  // t = 22.0: elapsed=22-10=12s > stuckTimeout=5 ✓
  //   AND cooldown: 22-10=12s > cooldown=10s ✓ → 触发第2次重规划
  auto r4 = tracker.update(robotPos, empty_odom, 22.0);
  EXPECT_EQ(r4.event, WaypointEvent::kReplanning)
      << "t=22s: cooldown expired (22-10=12 > 10), second replan should fire";
}
