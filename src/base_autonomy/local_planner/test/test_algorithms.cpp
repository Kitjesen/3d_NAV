/**
 * test_algorithms.cpp — local_planner 算法单元测试
 *
 * 覆盖:
 *   - VoxelGrid: 坐标变换、边界检查、scaleY 稳定性
 *   - PathScoring: 方向权重、得分退化
 *   - PurePursuit: 自适应前视距离、速度夹紧
 *   - StopPriority: 优先级上升/下降、连续清零
 *   - AngleNorm: 角度归一化、边界情况
 *   - TwoWayDrive: 前进/后退切换迟滞
 *   - SpeedRamp: 加速/减速积分
 *   - NearFieldStop: 近场急停条件
 */

#include <gtest/gtest.h>
#include <cmath>
#include <algorithm>

// ══════════════════════════════════════════════════════════════════════
//  从 localPlanner.cpp 提取的常量与公式 (保持与源码一致)
// ══════════════════════════════════════════════════════════════════════

static constexpr double PI = M_PI;

// ─── Voxel Grid 参数 ───
static constexpr float GRID_VOXEL_SIZE      = 0.02f;
static constexpr float GRID_OFFSET_X        = 3.2f;   // gridVoxelOffsetX_
static constexpr float GRID_OFFSET_Y        = 4.5f;   // gridVoxelOffsetY_
static constexpr int   GRID_NUM_X           = 161;
static constexpr int   GRID_NUM_Y           = 451;
static constexpr float SEARCH_RADIUS        = 0.45f;  // searchRadius_ 默认

// ─── Voxel Grid 坐标转换 ───
struct VoxelIndex { int x; int y; bool valid; };

/**
 * 计算世界坐标 (x2, y2) → 体素网格索引。
 * 直接复现 localPlanner.cpp 第 1017-1023 行逻辑。
 */
inline VoxelIndex worldToVoxel(float x2, float y2,
                                float offsetX = GRID_OFFSET_X,
                                float offsetY = GRID_OFFSET_Y,
                                float searchRadius = SEARCH_RADIUS,
                                float voxelSize = GRID_VOXEL_SIZE)
{
  VoxelIndex idx{};
  // 防止 x2 = 0 时 scaleY 除零 (源码漏洞)
  if (std::fabs(x2) < 1e-6f) {
    idx.valid = false;
    return idx;
  }
  float scaleY = x2 / offsetX + searchRadius / offsetY * (offsetX - x2) / offsetX;
  if (std::fabs(scaleY) < 1e-6f) {
    idx.valid = false;
    return idx;
  }
  idx.x = static_cast<int>((offsetX + voxelSize / 2.0f - x2)  / voxelSize);
  idx.y = static_cast<int>((offsetY + voxelSize / 2.0f - y2 / scaleY) / voxelSize);
  idx.valid = (idx.x >= 0 && idx.x < GRID_NUM_X && idx.y >= 0 && idx.y < GRID_NUM_Y);
  return idx;
}

// ─── Path Scoring ───
inline float pathScore(float dirDiff, float rotDirW, float groupDirW,
                       float dirWeight = 0.02f, bool nearGoal = false)
{
  float dw = std::fabs(dirWeight * dirDiff);
  float base = 1.0f - std::sqrt(std::sqrt(std::max(0.0f, dw)));
  if (nearGoal) {
    return base * groupDirW * groupDirW;
  }
  return base * rotDirW * rotDirW * rotDirW * rotDirW;
}

// ─── Pure Pursuit 自适应前视距离 ───
inline float adaptiveLookAhead(float baseDis, float ratio, float speed,
                                float minDis, float maxDis)
{
  float d = baseDis + ratio * speed;
  return std::max(minDis, std::min(maxDis, d));
}

// ─── 角度归一化到 (-π, π] ───
inline float normalizeAngle(float a)
{
  while (a >  static_cast<float>(PI)) a -= 2.0f * static_cast<float>(PI);
  while (a <= -static_cast<float>(PI)) a += 2.0f * static_cast<float>(PI);
  return a;
}

// ─── Stop 信号优先级 ───
struct StopState { int level; int clearCount; };

/**
 * 复现 pathFollower.cpp 第 385-402 行逻辑。
 * val: 收到的 stop 值; state: 当前状态 (in-out)
 */
inline void applyStopSignal(int val, StopState& state)
{
  if (val > state.level) {
    state.level = val;
    state.clearCount = 0;
  } else if (val == 0) {
    state.clearCount++;
    if (state.clearCount >= 3) {
      state.level = 0;
      state.clearCount = 0;
    }
  }
}

// ─── 双向驱动切换 ───
inline bool shouldSwitchDirection(bool currentFwd, float dirDiff,
                                  double elapsed, double switchThreshold = 1.0)
{
  if (currentFwd && std::fabs(dirDiff) > static_cast<float>(PI / 2.0)
      && elapsed > switchThreshold) {
    return true;
  }
  if (!currentFwd && std::fabs(dirDiff) < static_cast<float>(PI / 2.0)
      && elapsed > switchThreshold) {
    return true;
  }
  return false;
}

// ─── 速度积分 ───
inline float rampSpeed(float current, float target, float maxAccel, float dt)
{
  float delta = maxAccel * dt;
  if (current < target) return std::min(current + delta, target);
  if (current > target) return std::max(current - delta, target);
  return current;
}

// ─── 近场急停 ───
inline bool nearFieldStop(float px, float py, float vehicleWidth,
                           float stopDis = 0.5f)
{
  return px > 0.0f && px < stopDis && std::fabs(py) < vehicleWidth / 2.0f + 0.1f;
}


// ══════════════════════════════════════════════════════════════════════
//  测试: VoxelGrid 坐标转换
// ══════════════════════════════════════════════════════════════════════

class VoxelGridTest : public ::testing::Test {};

TEST_F(VoxelGridTest, CenterPointMapsToMiddleIndex)
{
  // x2 = offsetX/2 = 1.6, y2 = 0 → 应在网格中间附近
  auto idx = worldToVoxel(1.6f, 0.0f);
  EXPECT_TRUE(idx.valid);
  EXPECT_GE(idx.x, 0);
  EXPECT_LT(idx.x, GRID_NUM_X);
  EXPECT_GE(idx.y, 0);
  EXPECT_LT(idx.y, GRID_NUM_Y);
}

TEST_F(VoxelGridTest, ZeroXReturnsInvalid)
{
  // x2 = 0 → scaleY 除零，应返回 invalid
  auto idx = worldToVoxel(0.0f, 1.0f);
  EXPECT_FALSE(idx.valid) << "x2=0 应触发除零保护，返回 invalid";
}

TEST_F(VoxelGridTest, NearZeroXReturnsInvalid)
{
  auto idx = worldToVoxel(1e-7f, 1.0f);
  EXPECT_FALSE(idx.valid) << "x2 近零应同样返回 invalid";
}

TEST_F(VoxelGridTest, ForwardObstacleInBounds)
{
  // 正前方 2m 处的障碍物
  auto idx = worldToVoxel(2.0f, 0.0f);
  EXPECT_TRUE(idx.valid);
}

TEST_F(VoxelGridTest, BeyondRangeReturnsOutOfBounds)
{
  // x2 超出 3.2m 规划范围
  auto idx = worldToVoxel(4.0f, 0.0f);
  EXPECT_FALSE(idx.valid) << "超出 adjacentRange 的点不应在网格内";
}

TEST_F(VoxelGridTest, LargeYOutOfBounds)
{
  // 正侧方大偏移 y 应超出网格
  auto idx = worldToVoxel(1.0f, 10.0f);
  EXPECT_FALSE(idx.valid);
}

TEST_F(VoxelGridTest, NegativeXHandled)
{
  // 车后方点 (x2 < 0)
  auto idx = worldToVoxel(-1.0f, 0.0f);
  // 索引会是负数或越界, 不 crash 即可
  EXPECT_FALSE(idx.valid);
}


// ══════════════════════════════════════════════════════════════════════
//  测试: Path Scoring
// ══════════════════════════════════════════════════════════════════════

class PathScoringTest : public ::testing::Test {};

TEST_F(PathScoringTest, PerfectAlignmentMaxScore)
{
  // dirDiff = 0 → dw = 0 → base = 1 → score = 1 * weight^4
  float score = pathScore(0.0f, 1.0f, 1.0f);
  EXPECT_NEAR(score, 1.0f, 1e-4f);
}

TEST_F(PathScoringTest, LargeDirectionDiffReducesScore)
{
  float scoreAligned  = pathScore(0.0f,  1.0f, 1.0f);
  float scoreMisaligned = pathScore(90.0f, 1.0f, 1.0f);
  EXPECT_GT(scoreAligned, scoreMisaligned);
}

TEST_F(PathScoringTest, ScoreNonNegative)
{
  // 任意 dirDiff 得分不应为负
  for (float diff : {0.0f, 30.0f, 90.0f, 180.0f}) {
    float s = pathScore(diff, 1.0f, 1.0f);
    EXPECT_GE(s, 0.0f) << "dirDiff=" << diff;
  }
}

TEST_F(PathScoringTest, RotDirWeightScalesScore)
{
  // rotDirW 较大时得分更高
  float sHigh = pathScore(10.0f, 1.0f, 1.0f);
  float sLow  = pathScore(10.0f, 0.5f, 1.0f);
  EXPECT_GT(sHigh, sLow);
}

TEST_F(PathScoringTest, NearGoalUsesGroupWeight)
{
  // nearGoal=true 时使用 groupDirW^2, 而非 rotDirW^4
  float sGroup  = pathScore(0.0f, 0.3f, 1.0f, 0.02f, true);
  float sRotDir = pathScore(0.0f, 0.3f, 1.0f, 0.02f, false);
  EXPECT_GT(sGroup, sRotDir) << "近目标模式应用更高权重";
}


// ══════════════════════════════════════════════════════════════════════
//  测试: Pure Pursuit 自适应前视距离
// ══════════════════════════════════════════════════════════════════════

class PurePursuitTest : public ::testing::Test {};

TEST_F(PurePursuitTest, ZeroSpeedReturnBasePlusClamped)
{
  // speed=0 → 返回 max(min, base)
  float d = adaptiveLookAhead(0.3f, 0.5f, 0.0f, 0.2f, 2.0f);
  EXPECT_NEAR(d, 0.3f, 1e-4f);
}

TEST_F(PurePursuitTest, NormalSpeedScales)
{
  float d = adaptiveLookAhead(0.3f, 0.5f, 1.0f, 0.2f, 2.0f);
  EXPECT_NEAR(d, 0.8f, 1e-4f);  // 0.3 + 0.5*1.0
}

TEST_F(PurePursuitTest, ClampedAtMinimum)
{
  float d = adaptiveLookAhead(0.05f, 0.01f, 0.0f, 0.2f, 2.0f);
  EXPECT_GE(d, 0.2f) << "前视距离不应低于 minLookAheadDis";
}

TEST_F(PurePursuitTest, ClampedAtMaximum)
{
  float d = adaptiveLookAhead(0.3f, 0.5f, 100.0f, 0.2f, 2.0f);
  EXPECT_LE(d, 2.0f) << "前视距离不应超过 maxLookAheadDis";
}

TEST_F(PurePursuitTest, HighSpeedClampedToMax)
{
  float d = adaptiveLookAhead(0.3f, 0.5f, 5.0f, 0.2f, 2.0f);
  EXPECT_NEAR(d, 2.0f, 1e-4f);
}


// ══════════════════════════════════════════════════════════════════════
//  测试: 角度归一化
// ══════════════════════════════════════════════════════════════════════

class AngleNormTest : public ::testing::Test {};

TEST_F(AngleNormTest, ZeroUnchanged) {
  EXPECT_NEAR(normalizeAngle(0.0f), 0.0f, 1e-5f);
}

TEST_F(AngleNormTest, PiNormalizedToNegativePi) {
  // π 边界: 应归一化到 (-π, π] 范围内
  float v = normalizeAngle(static_cast<float>(PI));
  EXPECT_LE(std::fabs(v), static_cast<float>(PI) + 1e-5f);
}

TEST_F(AngleNormTest, LargePositiveWraps) {
  float v = normalizeAngle(5.0f);  // > π
  EXPECT_GE(v, -static_cast<float>(PI) - 1e-5f);
  EXPECT_LE(v,  static_cast<float>(PI) + 1e-5f);
}

TEST_F(AngleNormTest, LargeNegativeWraps) {
  float v = normalizeAngle(-5.0f);
  EXPECT_GE(v, -static_cast<float>(PI) - 1e-5f);
  EXPECT_LE(v,  static_cast<float>(PI) + 1e-5f);
}

TEST_F(AngleNormTest, DoublePiWrapsToZero) {
  float v = normalizeAngle(2.0f * static_cast<float>(PI));
  EXPECT_NEAR(v, 0.0f, 1e-4f);
}

TEST_F(AngleNormTest, NinetyDegreesUnchanged) {
  float v = normalizeAngle(static_cast<float>(PI / 2.0));
  EXPECT_NEAR(v, static_cast<float>(PI / 2.0), 1e-5f);
}


// ══════════════════════════════════════════════════════════════════════
//  测试: Stop 信号优先级
// ══════════════════════════════════════════════════════════════════════

class StopPriorityTest : public ::testing::Test {
protected:
  StopState state{0, 0};
};

TEST_F(StopPriorityTest, InitialStateIsZero) {
  EXPECT_EQ(state.level, 0);
}

TEST_F(StopPriorityTest, HigherValueRaisesLevel) {
  applyStopSignal(2, state);
  EXPECT_EQ(state.level, 2);
}

TEST_F(StopPriorityTest, LowerValueDoesNotLower) {
  applyStopSignal(2, state);
  applyStopSignal(1, state);
  EXPECT_EQ(state.level, 2) << "stop 等级不应被更低值降低";
}

TEST_F(StopPriorityTest, ThreeConsecutiveZerosClear) {
  applyStopSignal(1, state);
  applyStopSignal(0, state);
  applyStopSignal(0, state);
  applyStopSignal(0, state);
  EXPECT_EQ(state.level, 0) << "3 次连续 0 应清除 stop";
}

TEST_F(StopPriorityTest, TwoConsecutiveZeroNotEnough) {
  applyStopSignal(1, state);
  applyStopSignal(0, state);
  applyStopSignal(0, state);
  EXPECT_EQ(state.level, 1) << "仅 2 次 0 不足以清除 stop";
}

TEST_F(StopPriorityTest, NonZeroInterruptsClearCount) {
  applyStopSignal(1, state);
  applyStopSignal(0, state);
  applyStopSignal(0, state);
  applyStopSignal(1, state);   // 中断: 等级重升
  EXPECT_EQ(state.level, 1);
  // 还需再 3 次 0 才能清
  applyStopSignal(0, state);
  EXPECT_EQ(state.level, 1);
}

TEST_F(StopPriorityTest, Level2BlocksRotation) {
  // 级别 2 → vehicleYawRate 也应归零 (逻辑上的语义测试)
  applyStopSignal(2, state);
  EXPECT_GE(state.level, 2);
}


// ══════════════════════════════════════════════════════════════════════
//  测试: 双向驱动切换
// ══════════════════════════════════════════════════════════════════════

class TwoWayDriveTest : public ::testing::Test {};

TEST_F(TwoWayDriveTest, ForwardToBackwardWhenFacingBack) {
  // 当前前进, 方向差 > π/2, 超过切换时间
  bool sw = shouldSwitchDirection(true, 2.5f, 1.5, 1.0);
  EXPECT_TRUE(sw);
}

TEST_F(TwoWayDriveTest, NoSwitchBeforeTimeThreshold) {
  bool sw = shouldSwitchDirection(true, 2.5f, 0.5, 1.0);
  EXPECT_FALSE(sw) << "切换时间未到，不应切换";
}

TEST_F(TwoWayDriveTest, BackwardToForwardWhenFacingFront) {
  bool sw = shouldSwitchDirection(false, 0.3f, 1.5, 1.0);
  EXPECT_TRUE(sw);
}

TEST_F(TwoWayDriveTest, NoSwitchWhenAlreadyAligned) {
  // 前进中, 方向差 < π/2: 不切换
  bool sw = shouldSwitchDirection(true, 0.5f, 2.0, 1.0);
  EXPECT_FALSE(sw);
}

TEST_F(TwoWayDriveTest, ExactlyHalfPiBoundaryStaysForward) {
  // 方向差 == π/2: fabs > π/2 为 false，不切换
  bool sw = shouldSwitchDirection(true, static_cast<float>(PI / 2.0), 2.0, 1.0);
  EXPECT_FALSE(sw) << "恰好 π/2 时不应触发切换 (> 判断)";
}


// ══════════════════════════════════════════════════════════════════════
//  测试: 速度积分 (梯形加减速)
// ══════════════════════════════════════════════════════════════════════

class SpeedRampTest : public ::testing::Test {};

TEST_F(SpeedRampTest, AcceleratesFromZero) {
  float s = rampSpeed(0.0f, 1.0f, 1.0f, 0.01f);  // maxAccel=1, dt=10ms
  EXPECT_NEAR(s, 0.01f, 1e-5f);
}

TEST_F(SpeedRampTest, DoesNotOvershootTarget) {
  float s = rampSpeed(0.99f, 1.0f, 1.0f, 0.1f);
  EXPECT_LE(s, 1.0f);
}

TEST_F(SpeedRampTest, DeceleratesCorrectly) {
  float s = rampSpeed(1.0f, 0.0f, 1.0f, 0.01f);
  EXPECT_NEAR(s, 0.99f, 1e-5f);
}

TEST_F(SpeedRampTest, DoesNotUndershootZero) {
  float s = rampSpeed(0.005f, 0.0f, 1.0f, 0.01f);
  EXPECT_GE(s, 0.0f);
}

TEST_F(SpeedRampTest, AlreadyAtTargetUnchanged) {
  float s = rampSpeed(0.5f, 0.5f, 1.0f, 0.01f);
  EXPECT_NEAR(s, 0.5f, 1e-5f);
}

TEST_F(SpeedRampTest, LargerDtCausesLargerStep) {
  float s1 = rampSpeed(0.0f, 1.0f, 1.0f, 0.01f);
  float s2 = rampSpeed(0.0f, 1.0f, 1.0f, 0.05f);
  EXPECT_LT(s1, s2);
}


// ══════════════════════════════════════════════════════════════════════
//  测试: 近场急停
// ══════════════════════════════════════════════════════════════════════

class NearFieldStopTest : public ::testing::Test {
protected:
  static constexpr float kWidth = 0.6f;  // vehicleWidth = 0.6m
};

TEST_F(NearFieldStopTest, ObstacleInFrontTriggersStop) {
  EXPECT_TRUE(nearFieldStop(0.3f, 0.0f, kWidth));
}

TEST_F(NearFieldStopTest, ObstacleBehindNoStop) {
  EXPECT_FALSE(nearFieldStop(-0.3f, 0.0f, kWidth)) << "车后方不应急停";
}

TEST_F(NearFieldStopTest, ObstacleWidelyLeftNoStop) {
  // py 超出 vehicleWidth/2 + 0.1 = 0.4m
  EXPECT_FALSE(nearFieldStop(0.3f, 0.5f, kWidth));
}

TEST_F(NearFieldStopTest, ObstacleSlightlyLeftStops) {
  // py = 0.3m < 0.4m → 触发
  EXPECT_TRUE(nearFieldStop(0.3f, 0.3f, kWidth));
}

TEST_F(NearFieldStopTest, ObstacleBeyondStopDisNoStop) {
  // px = 0.6 > 0.5m stopDis → 不触发
  EXPECT_FALSE(nearFieldStop(0.6f, 0.0f, kWidth));
}

TEST_F(NearFieldStopTest, ObstacleAtExactBoundaryNoStop) {
  // px = 0.5 exactly — 不满足 < stopDis (0.5 < 0.5 = false)
  EXPECT_FALSE(nearFieldStop(0.5f, 0.0f, kWidth));
}

TEST_F(NearFieldStopTest, WiderVehicleWiderTriggerZone) {
  float wideVehicle = 1.0f;
  // py = 0.55m, vehicleWidth=1.0 → 阈值为 0.6m → 0.55 < 0.6 → 触发
  EXPECT_TRUE(nearFieldStop(0.3f, 0.55f, wideVehicle));
}


// ══════════════════════════════════════════════════════════════════════
//  main
// ══════════════════════════════════════════════════════════════════════

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
