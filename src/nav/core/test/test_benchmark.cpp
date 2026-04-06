/**
 * test_benchmark.cpp — nav_core performance throughput benchmarks
 *
 * Iter 16: Verify core algorithms meet real-time requirements.
 *   - PathFollower computeControl: 1000 iterations < 100ms
 *   - LocalPlanner scorePath: 1000 iterations < 100ms
 *   - PctAdapter update: 1000 iterations < 100ms
 *
 * Outputs average microseconds per call.
 */

#include <gtest/gtest.h>
#include "nav_core/path_follower_core.hpp"
#include "nav_core/local_planner_core.hpp"
#include "nav_core/local_planner_full.hpp"
#include "nav_core/terrain_core.hpp"
#include "nav_core/pct_adapter_core.hpp"
#include <chrono>
#include <cmath>
#include <cstdio>
#include <random>
#include <vector>

using namespace nav_core;

// MSVC aggregate initialization helper
static Pose makePose(double x, double y, double z, double yaw = 0.0) {
  Pose p;
  p.position.x = x;
  p.position.y = y;
  p.position.z = z;
  p.yaw = yaw;
  return p;
}

static constexpr int kIterations = 1000;
static constexpr int kMaxElapsedMs = 100;

// ═══════════════════════════════════════════════════
//  PathFollower Throughput
// ═══════════════════════════════════════════════════

TEST(Benchmark, PathFollowerThroughput) {
  PathFollowerParams p;
  p.maxSpeed = 1.0;
  p.maxAccel = 1.0;
  p.dirDiffThre = 0.1;
  p.baseLookAheadDis = 0.3;
  p.lookAheadRatio = 0.5;
  p.stopDisThre = 0.2;
  p.omniDirGoalThre = 1.0;
  p.omniDirDiffThre = 1.5;

  // 100-point path
  std::vector<Vec3> path;
  path.reserve(100);
  for (int i = 0; i < 100; i++) {
    Vec3 pt;
    pt.x = 0.1 * i;
    pt.y = 0.0;
    pt.z = 0.0;
    path.push_back(pt);
  }

  PathFollowerState state;
  Vec3 robot{0.0, 0.0, 0.0};

  auto start = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < kIterations; i++) {
    state = {};  // Reset state each iteration for consistency
    computeControl(robot, 0.0, path, 1.0, static_cast<double>(i) * 0.01,
                   1.0, 0, p, state);
  }
  auto end = std::chrono::high_resolution_clock::now();

  auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
  auto elapsed_ms = elapsed_us / 1000.0;
  double avg_us = static_cast<double>(elapsed_us) / kIterations;

  std::printf("[BENCHMARK] PathFollower::computeControl: %d calls in %.1f ms "
              "(avg %.1f us/call)\n",
              kIterations, elapsed_ms, avg_us);

  EXPECT_LT(elapsed_ms, kMaxElapsedMs)
      << "PathFollower throughput: " << kIterations << " calls should complete within "
      << kMaxElapsedMs << " ms";
}

// ═══════════════════════════════════════════════════
//  LocalPlanner Scoring Throughput
// ═══════════════════════════════════════════════════

TEST(Benchmark, LocalPlannerScoring) {
  PathScoreParams p;
  p.dirWeight = 0.02;
  p.slopeWeight = 3.0;
  p.omniDirGoalThre = 5.0;

  auto start = std::chrono::high_resolution_clock::now();
  volatile double sink = 0;  // Prevent optimization
  for (int i = 0; i < kIterations; i++) {
    double dirDiff = static_cast<double>(i % 180);
    double rotDirW = computeRotDirW(i % 36);
    double groupDirW = computeGroupDirW(i % 7);
    double terrain = 0.1 * (i % 10);
    double goalDis = 1.0 + (i % 20);

    sink = scorePath(dirDiff, rotDirW, groupDirW, terrain, goalDis, p);
  }
  auto end = std::chrono::high_resolution_clock::now();

  auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
  auto elapsed_ms = elapsed_us / 1000.0;
  double avg_us = static_cast<double>(elapsed_us) / kIterations;

  std::printf("[BENCHMARK] LocalPlanner::scorePath: %d calls in %.1f ms "
              "(avg %.1f us/call)\n",
              kIterations, elapsed_ms, avg_us);

  EXPECT_LT(elapsed_ms, kMaxElapsedMs)
      << "LocalPlanner scoring throughput: " << kIterations << " calls should complete within "
      << kMaxElapsedMs << " ms";

  (void)sink;
}

// ═══════════════════════════════════════════════════
//  PctAdapter Update Throughput
// ═══════════════════════════════════════════════════

TEST(Benchmark, PctAdapterUpdate) {
  WaypointTrackerParams p;
  p.waypointDistance = 0.5;
  p.arrivalThreshold = 0.3;
  p.stuckTimeoutSec = 1000.0;  // Won't trigger during benchmark
  p.searchWindow = 5;
  p.maxReplanCount = 2;
  p.replanCooldownSec = 5.0;

  WaypointTracker tracker(p);

  // 200-point path
  Path raw;
  raw.reserve(200);
  for (int i = 0; i < 200; i++) {
    raw.push_back(makePose(0.1 * i, 0.0, 0.0));
  }
  tracker.setPath(raw, 0.0);

  auto start = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < kIterations; i++) {
    Vec3 robot;
    robot.x = 0.01 * (i % 100);
    robot.y = 0.0;
    robot.z = 0.0;
    double t = static_cast<double>(i) * 0.01;
    tracker.update(robot, {}, t);
  }
  auto end = std::chrono::high_resolution_clock::now();

  auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
  auto elapsed_ms = elapsed_us / 1000.0;
  double avg_us = static_cast<double>(elapsed_us) / kIterations;

  std::printf("[BENCHMARK] PctAdapter::update: %d calls in %.1f ms "
              "(avg %.1f us/call)\n",
              kIterations, elapsed_ms, avg_us);

  EXPECT_LT(elapsed_ms, kMaxElapsedMs)
      << "PctAdapter update throughput: " << kIterations << " calls should complete within "
      << kMaxElapsedMs << " ms";
}

// ═══════════════════════════════════════════════════
//  scorePathFast vs scorePath (LUT acceleration)
// ═══════════════════════════════════════════════════

TEST(Benchmark, ScorePathFastVsOriginal) {
  const auto& lut = rotLUT();
  PathScoreParams p;
  p.dirWeight = 0.02;
  p.slopeWeight = 3.0;
  p.omniDirGoalThre = 5.0;

  constexpr int kIter = 100000;
  volatile double sink = 0;

  // Original scorePath
  auto t0 = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < kIter; i++) {
    double dirDiff = static_cast<double>(i % 180);
    double rotDirW = computeRotDirW(i % 36);
    double groupDirW = computeGroupDirW(i % 7);
    sink = scorePath(dirDiff, rotDirW, groupDirW, 0.1f * (i % 10), 1.0 + (i % 20), p);
  }
  auto t1 = std::chrono::high_resolution_clock::now();

  // Fast scorePathFast (LUT)
  for (int i = 0; i < kIter; i++) {
    int rotDir = i % 36;
    int grp = i % 7;
    double dirDiff = static_cast<double>(i % 180);
    sink = scorePathFast(dirDiff, lut.rotDirW4[rotDir], lut.groupDirW2[grp],
                         0.1f * (i % 10), 1.0 + (i % 20), p, lut);
  }
  auto t2 = std::chrono::high_resolution_clock::now();

  auto orig_us = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();
  auto fast_us = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
  double speedup = (fast_us > 0) ? static_cast<double>(orig_us) / fast_us : 999.0;

  std::printf("[BENCHMARK] scorePath original: %ld us (%d calls, %.2f ns/call)\n",
              orig_us, kIter, 1000.0 * orig_us / kIter);
  std::printf("[BENCHMARK] scorePathFast LUT:  %ld us (%d calls, %.2f ns/call)\n",
              fast_us, kIter, 1000.0 * fast_us / kIter);
  std::printf("[BENCHMARK] Speedup: %.2fx\n", speedup);

  (void)sink;
}

// ═══════════════════════════════════════════════════
//  Full planning cycle benchmark (scoreAndSelect)
// ═══════════════════════════════════════════════════

TEST(Benchmark, FullPlanCycle) {
  LocalPlannerParams lpp;
  lpp.adjacentRange = 3.5;
  lpp.checkObstacle = true;
  lpp.useTerrainAnalysis = true;
  lpp.dirWeight = 0.02;
  lpp.twoWayDrive = true;

  LocalPlannerCore planner(lpp);

  // Create synthetic path library — just enough for the scoring loop
  // In real usage, loadPaths() reads from disk. Here we test plan() overhead.
  planner.setVehicle(0, 0, 0, 0);
  planner.setGoal(5, 0);

  // Generate 500 random obstacle points (typical outdoor scene)
  std::mt19937 rng(42);
  std::uniform_real_distribution<float> posRange(-3.0f, 3.0f);
  std::uniform_real_distribution<float> zRange(-0.3f, 0.3f);

  std::vector<float> cloud(500 * 4);
  for (int i = 0; i < 500; i++) {
    cloud[i*4+0] = posRange(rng);  // x
    cloud[i*4+1] = posRange(rng);  // y
    cloud[i*4+2] = zRange(rng);    // z
    cloud[i*4+3] = std::fabs(zRange(rng));  // intensity/height
  }

  // Warm up (paths not loaded, so plan() returns early — tests framework overhead)
  for (int i = 0; i < 10; i++)
    planner.plan(cloud.data(), 500, i * 0.1);

  auto t0 = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < kIterations; i++) {
    planner.setVehicle(0.001 * i, 0, 0, 0.01 * (i % 10));
    planner.plan(cloud.data(), 500, i * 0.01);
  }
  auto t1 = std::chrono::high_resolution_clock::now();

  auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();
  std::printf("[BENCHMARK] Full plan cycle (500 pts, no paths): %d calls in %ld us "
              "(avg %.1f us/call)\n",
              kIterations, elapsed_us, static_cast<double>(elapsed_us) / kIterations);
}

// ═══════════════════════════════════════════════════
//  Terrain analysis benchmark (nth_element optimization)
// ═══════════════════════════════════════════════════

TEST(Benchmark, TerrainAnalysis) {
  TerrainParams tp;
  TerrainAnalysisCore terrain(tp);
  terrain.updateVehicle(0, 0, 0, 0, 0, 0);

  // Generate 2000 random points (typical LiDAR frame)
  std::mt19937 rng(123);
  std::uniform_real_distribution<float> posR(-5.0f, 5.0f);
  std::uniform_real_distribution<float> zR(-0.5f, 0.5f);

  std::vector<float> cloud(2000 * 4);
  for (int i = 0; i < 2000; i++) {
    cloud[i*4+0] = posR(rng);
    cloud[i*4+1] = posR(rng);
    cloud[i*4+2] = zR(rng);
    cloud[i*4+3] = 0.0f;
  }

  constexpr int kTerrainIter = 100;
  auto t0 = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < kTerrainIter; i++) {
    terrain.updateVehicle(0.01 * i, 0, 0, 0, 0, 0);
    terrain.process(cloud.data(), 2000, i * 0.1);
  }
  auto t1 = std::chrono::high_resolution_clock::now();

  auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();
  std::printf("[BENCHMARK] TerrainAnalysis: %d calls in %ld us "
              "(avg %.1f us/call, 2000 pts)\n",
              kTerrainIter, elapsed_us, static_cast<double>(elapsed_us) / kTerrainIter);

  EXPECT_LT(elapsed_us / 1000.0, 5000.0)  // 5s for 100 iterations
      << "Terrain should process 2000 pts in reasonable time";
}
