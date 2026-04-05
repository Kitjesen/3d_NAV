/**
 * nav_core/local_planner_full.hpp — Complete CMU local planner, zero ROS2.
 *
 * Encapsulates the full processLoop from localPlanner.cpp:
 *   1. Load pre-computed path library (PLY files)
 *   2. Per-frame: obstacle scoring → group selection → path output
 *   3. Path scale degradation when no path found
 *   4. Three-phase recovery (rotate → backup → stop)
 *   5. Near-field emergency stop detection
 *   6. Slow-down level computation
 *
 * All ROS2 IO stripped. Pure C++17, header-only.
 *
 * Usage (from nanobind Python):
 *   core = LocalPlannerCore(params)
 *   core.load_paths("/path/to/paths/")
 *   core.set_vehicle(x, y, z, yaw)
 *   core.set_goal(gx, gy)
 *   result = core.plan(obstacle_pts_flat, n_pts, timestamp)
 */
#pragma once

#include "nav_core/types.hpp"
#include "nav_core/local_planner_core.hpp"
#include <cmath>
#include <cstring>
#include <vector>
#include <array>
#include <string>
#include <fstream>
#include <sstream>
#include <algorithm>

namespace nav_core {

// ── Parameters ──

struct LocalPlannerParams {
  double vehicleLength      = 0.6;
  double vehicleWidth       = 0.6;
  double sensorOffsetX      = 0.0;
  double sensorOffsetY      = 0.0;
  bool   twoWayDrive        = true;
  double adjacentRange      = 3.5;
  double obstacleHeightThre = 0.2;
  double groundHeightThre   = 0.1;
  double costHeightThre1    = 0.15;
  double costHeightThre2    = 0.1;
  bool   checkObstacle      = true;
  bool   checkRotObstacle   = false;
  bool   useTerrainAnalysis = true;
  int    pointPerPathThre   = 2;
  double minRelZ            = -0.5;
  double maxRelZ            = 0.25;
  double dirWeight          = 0.02;
  double dirThre            = 90.0;
  bool   dirToVehicle       = false;
  double pathScale          = 1.0;
  double minPathScale       = 0.75;
  double pathScaleStep      = 0.25;
  bool   pathScaleBySpeed   = true;
  double minPathRange       = 1.0;
  double pathRangeStep      = 0.5;
  bool   pathRangeBySpeed   = true;
  bool   pathCropByGoal     = true;
  double maxSpeed           = 1.0;
  double autonomySpeed      = 1.0;
  double slopeWeight        = 0.0;
  double goalClearRange     = 0.5;
  double goalBehindRange    = 0.8;
  double nearFieldStopDis   = 0.5;
  double freezeAng          = 90.0;
  double freezeTime         = 2.0;
  double omniDirGoalThre    = 1.0;
  int    slowPathNumThre    = 5;
  int    slowGroupNumThre   = 1;

  // Recovery
  double recoveryBlockedThre = 2.0;
  double recoveryRotateTime  = 2.5;
  double recoveryBackupTime  = 1.5;
  int    recoveryMaxCycles   = 3;
};

// ── Result ──

struct LocalPlanResult {
  std::vector<Vec3> path;
  int  slowDown    = 0;     // 0-3
  bool pathFound   = false;
  bool nearFieldStop = false;
  int  recoveryState = 0;   // 0=normal, 1=rotating, 2=backing_up
};

// ── Core Algorithm ──

static constexpr int kPathNum  = 343;
static constexpr int kGroupNum = 7;
static constexpr int kRotDirs  = 36;

class LocalPlannerCore {
public:
  explicit LocalPlannerCore(const LocalPlannerParams& params = LocalPlannerParams())
    : p_(params) {}

  /// Load pre-computed path library from directory containing PLY files.
  /// Returns true on success.
  bool loadPaths(const std::string& pathsDir) {
    bool ok = true;
    ok = ok && loadStartPaths(pathsDir + "/startPaths.ply");
    ok = ok && loadPathList(pathsDir + "/pathList.ply");
    ok = ok && loadCorrespondences(pathsDir + "/correspondences.txt");
    pathsLoaded_ = ok;
    return ok;
  }

  bool pathsLoaded() const { return pathsLoaded_; }

  /// Update vehicle pose (odom frame). Call before plan().
  void setVehicle(double x, double y, double z, double yaw) {
    vx_ = x; vy_ = y; vz_ = z; vyaw_ = yaw;
    cosYaw_ = std::cos(yaw);
    sinYaw_ = std::sin(yaw);
  }

  /// Update goal position (odom frame). Call before plan().
  void setGoal(double gx, double gy) {
    goalX_ = gx; goalY_ = gy;
  }

  /// Run one planning cycle.
  /// obstacle_pts: Nx4 float array [x,y,z,intensity] in odom frame.
  /// timestamp: monotonic seconds.
  LocalPlanResult plan(const float* obstacle_pts, int n_pts, double timestamp) {
    if (!pathsLoaded_) return {};

    LocalPlanResult result;
    odomTime_ = timestamp;

    // Joy speed for autonomy mode
    double joySpeed = p_.autonomySpeed / p_.maxSpeed;
    joySpeed = std::clamp(joySpeed, 0.0, 1.0);

    // Transform goal to body frame
    double relGoalX = (goalX_ - vx_) * cosYaw_ + (goalY_ - vy_) * sinYaw_;
    double relGoalY = -(goalX_ - vx_) * sinYaw_ + (goalY_ - vy_) * cosYaw_;
    double relGoalDis = std::hypot(relGoalX, relGoalY);
    double joyDir = std::atan2(relGoalY, relGoalX) * 180.0 / M_PI;

    // Freeze logic: goal behind robot
    if (std::fabs(joyDir) > p_.freezeAng && relGoalDis < p_.goalBehindRange) {
      relGoalDis = 0; joyDir = 0;
    }
    updateFreezeState(joyDir);
    if (freezeStatus_ == 1) { relGoalDis = 0; joyDir = 0; }

    // Clamp direction for single-direction drive
    if (!p_.twoWayDrive) {
      joyDir = std::clamp(joyDir, -95.0, 95.0);
    }

    // Near-field stop check
    result.nearFieldStop = checkNearFieldStop(obstacle_pts, n_pts);

    // Transform obstacles to body frame and crop
    buildPlannerCloud(obstacle_pts, n_pts);

    // Path scoring loop with scale degradation
    double pathRange = p_.adjacentRange;
    if (p_.pathRangeBySpeed) pathRange = p_.adjacentRange * joySpeed;
    if (pathRange < p_.minPathRange) pathRange = p_.minPathRange;

    double defPathScale = p_.pathScale;
    double curPathScale = defPathScale;
    if (p_.pathScaleBySpeed) curPathScale = defPathScale * joySpeed;
    if (curPathScale < p_.minPathScale) curPathScale = p_.minPathScale;

    bool pathFound = false;

    while (curPathScale >= p_.minPathScale && pathRange >= p_.minPathRange) {
      int selectedGroupID = scoreAndSelect(
          curPathScale, pathRange, relGoalDis, joyDir, joySpeed, result.slowDown);

      if (selectedGroupID >= 0) {
        buildOutputPath(selectedGroupID, curPathScale, pathRange, relGoalDis, result);
        pathFound = true;
        recoveryState_ = 0;
        blockedStartTime_ = -1.0;
        recoveryCycleCount_ = 0;
        break;
      }

      if (curPathScale >= p_.minPathScale + p_.pathScaleStep) {
        curPathScale -= p_.pathScaleStep;
        pathRange = p_.adjacentRange * curPathScale / defPathScale;
      } else {
        pathRange -= p_.pathRangeStep;
      }
    }

    result.pathFound = pathFound;

    if (!pathFound) {
      buildRecoveryPath(result);
    }

    result.recoveryState = recoveryState_;
    return result;
  }

  const LocalPlannerParams& params() const { return p_; }

private:
  LocalPlannerParams p_;
  bool pathsLoaded_ = false;

  // Vehicle state
  double vx_ = 0, vy_ = 0, vz_ = 0, vyaw_ = 0;
  double cosYaw_ = 1.0, sinYaw_ = 0.0;
  double goalX_ = 0, goalY_ = 0;
  double odomTime_ = 0;

  // Freeze state
  int freezeStatus_ = 0;
  double freezeStartTime_ = 0;

  // Recovery state
  int    recoveryState_       = 0;
  double blockedStartTime_    = -1.0;
  double recoveryPhaseStart_  = -1.0;
  int    recoveryCycleCount_  = 0;

  // Path library
  struct PathPoint { float x, y, z; };
  std::array<std::vector<PathPoint>, kGroupNum> startPaths_;
  std::array<int, kPathNum> pathList_{};
  std::array<float, kPathNum> endDirPathList_{};

  // Correspondences: voxel_id → path_ids
  int gridVoxelNumX_ = 161;
  int gridVoxelNumY_ = 451;
  int gridVoxelNum_  = gridVoxelNumX_ * gridVoxelNumY_;
  std::vector<std::vector<int>> correspondences_;

  // Per-frame scratch buffers (body frame)
  struct BodyPoint { float x, y, z, intensity; };
  std::vector<BodyPoint> plannerCloud_;

  // Scoring arrays
  std::vector<int>   clearPathList_;
  std::vector<float> pathPenaltyList_;
  std::vector<double> clearPathPerGroupScore_;
  std::vector<int>   clearPathPerGroupNum_;
  std::vector<float> pathPenaltyPerGroupScore_;

  // ── File loading ──

  int readPlyHeader(std::ifstream& f) {
    std::string line;
    int pointNum = 0;
    std::string prev;
    while (std::getline(f, line)) {
      std::istringstream iss(line);
      std::string token;
      iss >> token;
      if (token == "end_header") break;
      if (token == "vertex" || (prev == "element" && token != "element")) {
        if (prev == "element") {
          std::istringstream iss2(line);
          std::string t1; int n;
          iss2 >> t1 >> n;
          pointNum = n;
        }
      }
      // Better parsing: look for "element vertex N"
      if (line.find("element vertex") != std::string::npos) {
        std::istringstream iss3(line);
        std::string a, b; int n;
        iss3 >> a >> b >> n;
        pointNum = n;
      }
      prev = token;
    }
    return pointNum;
  }

  bool loadStartPaths(const std::string& filename) {
    std::ifstream f(filename);
    if (!f.is_open()) return false;
    int n = readPlyHeader(f);
    for (int i = 0; i < n; i++) {
      float x, y, z; int groupID;
      f >> x >> y >> z >> groupID;
      if (groupID >= 0 && groupID < kGroupNum)
        startPaths_[groupID].push_back({x, y, z});
    }
    return true;
  }

  bool loadPathList(const std::string& filename) {
    std::ifstream f(filename);
    if (!f.is_open()) return false;
    int n = readPlyHeader(f);
    if (n != kPathNum) return false;
    for (int i = 0; i < kPathNum; i++) {
      float ex, ey, ez; int pathID, groupID;
      f >> ex >> ey >> ez >> pathID >> groupID;
      if (pathID >= 0 && pathID < kPathNum && groupID >= 0 && groupID < kGroupNum) {
        pathList_[pathID] = groupID;
        endDirPathList_[pathID] = 2.0f * std::atan2(ey, ex) * 180.0f / (float)M_PI;
      }
    }
    return true;
  }

  bool loadCorrespondences(const std::string& filename) {
    std::ifstream f(filename);
    if (!f.is_open()) return false;
    correspondences_.clear();
    correspondences_.resize(gridVoxelNum_);
    for (int i = 0; i < gridVoxelNum_; i++) {
      int gridVoxelID;
      f >> gridVoxelID;
      int pathID;
      while (f >> pathID) {
        if (pathID == -1) break;
        if (gridVoxelID >= 0 && gridVoxelID < gridVoxelNum_ &&
            pathID >= 0 && pathID < kPathNum) {
          correspondences_[gridVoxelID].push_back(pathID);
        }
      }
    }
    // Allocate scoring arrays
    int total = kRotDirs * kPathNum;
    clearPathList_.resize(total);
    pathPenaltyList_.resize(total);
    clearPathPerGroupScore_.resize(kRotDirs * kGroupNum);
    clearPathPerGroupNum_.resize(kRotDirs * kGroupNum);
    pathPenaltyPerGroupScore_.resize(kRotDirs * kGroupNum);
    return true;
  }

  // ── Frame processing ──

  void updateFreezeState(double joyDir) {
    if (std::fabs(joyDir) > p_.freezeAng && freezeStatus_ == 0) {
      freezeStartTime_ = odomTime_;
      freezeStatus_ = 1;
    } else if (odomTime_ - freezeStartTime_ > p_.freezeTime && freezeStatus_ == 1) {
      freezeStatus_ = 2;
    } else if (std::fabs(joyDir) <= p_.freezeAng && freezeStatus_ == 2) {
      freezeStatus_ = 0;
    }
  }

  bool checkNearFieldStop(const float* pts, int n) {
    if (!p_.checkObstacle) return false;
    double halfW = p_.vehicleWidth / 2.0 + 0.1;
    for (int i = 0; i < n; i++) {
      float px = pts[i*4], py = pts[i*4+1], pz = pts[i*4+2], h = pts[i*4+3];
      float dx = px - (float)vx_, dy = py - (float)vy_;
      // To body frame
      float bx = dx * (float)cosYaw_ + dy * (float)sinYaw_;
      float by = -dx * (float)sinYaw_ + dy * (float)cosYaw_;
      if (bx > 0 && bx < (float)p_.nearFieldStopDis &&
          std::fabs(by) < halfW &&
          (h > (float)p_.obstacleHeightThre || !p_.useTerrainAnalysis)) {
        return true;
      }
    }
    return false;
  }

  void buildPlannerCloud(const float* pts, int n) {
    plannerCloud_.clear();
    plannerCloud_.reserve(n);
    double adjRangeSq = p_.adjacentRange * p_.adjacentRange;
    for (int i = 0; i < n; i++) {
      float px = pts[i*4], py = pts[i*4+1], pz = pts[i*4+2], h = pts[i*4+3];
      float dx = px - (float)vx_, dy = py - (float)vy_, dz = pz - (float)vz_;
      if (dx*dx + dy*dy >= adjRangeSq) continue;
      if (!((dz > p_.minRelZ && dz < p_.maxRelZ) || p_.useTerrainAnalysis)) continue;
      float bx = dx * (float)cosYaw_ + dy * (float)sinYaw_;
      float by = -dx * (float)sinYaw_ + dy * (float)cosYaw_;
      plannerCloud_.push_back({bx, by, dz, h});
    }
  }

  int scoreAndSelect(double pathScale, double pathRange, double relGoalDis,
                     double joyDir, double joySpeed, int& slowDown) {
    const auto& lut = rotLUT();
    int totalPaths = kRotDirs * kPathNum;
    int totalGroups = kRotDirs * kGroupNum;

    std::memset(clearPathList_.data(), 0, sizeof(int) * totalPaths);
    std::memset(pathPenaltyList_.data(), 0, sizeof(float) * totalPaths);
    std::memset(clearPathPerGroupScore_.data(), 0, sizeof(double) * totalGroups);
    std::memset(clearPathPerGroupNum_.data(), 0, sizeof(int) * totalGroups);
    std::memset(pathPenaltyPerGroupScore_.data(), 0, sizeof(float) * totalGroups);

    // Precompute direction validity
    std::array<float, kRotDirs> angDiffList;
    std::array<bool, kRotDirs> rotDirValid;
    for (int d = 0; d < kRotDirs; d++) {
      float a = std::fabs((float)joyDir - (10.0f * d - 180.0f));
      angDiffList[d] = (a > 180.0f) ? 360.0f - a : a;
      float rotAngDeg = 10.0f * d - 180.0f;
      float rotDeg = 10.0f * d;
      bool skip = (angDiffList[d] > p_.dirThre && !p_.dirToVehicle) ||
                  (std::fabs(rotAngDeg) > p_.dirThre && std::fabs(joyDir) <= 90.0 && p_.dirToVehicle) ||
                  ((rotDeg > p_.dirThre && 360.0f - rotDeg > p_.dirThre) && std::fabs(joyDir) > 90.0 && p_.dirToVehicle);
      rotDirValid[d] = !skip;
    }

    // Voxel grid constants
    double invGS = 1.0 / 0.02;  // gridVoxelSize
    double offXH = 3.2 + 0.02 * 0.5;
    double offYH = 4.5 + 0.02 * 0.5;
    double scaleA = 0.45 / 4.5;
    double scaleB = 1.0 / 3.2;
    double pathRangeScaleSq = (pathRange / pathScale) * (pathRange / pathScale);
    double goalClearScaleSq = ((relGoalDis + p_.goalClearRange) / pathScale)
                             * ((relGoalDis + p_.goalClearRange) / pathScale);

    // Score obstacle points against all paths in all rotations
    int cloudSize = (int)plannerCloud_.size();
    for (int i = 0; i < cloudSize; i++) {
      float x = plannerCloud_[i].x / (float)pathScale;
      float y = plannerCloud_[i].y / (float)pathScale;
      float h = plannerCloud_[i].intensity;
      float disSq = x*x + y*y;

      if (disSq < pathRangeScaleSq && (disSq <= goalClearScaleSq || !p_.pathCropByGoal) && p_.checkObstacle) {
        for (int d = 0; d < kRotDirs; d++) {
          if (!rotDirValid[d]) continue;
          float x2 = (float)lut.c[d] * x + (float)lut.s[d] * y;
          float y2 = -(float)lut.s[d] * x + (float)lut.c[d] * y;

          double sy = x2 * scaleB + scaleA * (3.2 - x2) * scaleB;
          if (std::fabs(sy) < 1e-6) continue;

          int indX = (int)((offXH - x2) * invGS);
          int indY = (int)((offYH - y2 / sy) * invGS);
          if (indX >= 0 && indX < gridVoxelNumX_ && indY >= 0 && indY < gridVoxelNumY_) {
            int ind = gridVoxelNumY_ * indX + indY;
            for (int pathID : correspondences_[ind]) {
              int idx = kPathNum * d + pathID;
              if (h > (float)p_.obstacleHeightThre || !p_.useTerrainAnalysis) {
                clearPathList_[idx]++;
              } else if (h > (float)p_.groundHeightThre) {
                if (pathPenaltyList_[idx] < h) pathPenaltyList_[idx] = h;
              }
            }
          }
        }
      }
    }

    // Aggregate into group scores
    PathScoreParams sp;
    sp.dirWeight = p_.dirWeight;
    sp.slopeWeight = p_.slopeWeight;
    sp.omniDirGoalThre = p_.omniDirGoalThre;

    for (int i = 0; i < totalPaths; i++) {
      int rotDir = i / kPathNum;
      if (!rotDirValid[rotDir]) continue;
      if (clearPathList_[i] >= p_.pointPerPathThre) continue;

      int pathIdx = i % kPathNum;
      float rotAngDeg = 10.0f * rotDir - 180.0f;
      double dirDiff = angDiffDeg(joyDir, endDirPathList_[pathIdx] + rotAngDeg);
      double rotDirW = computeRotDirW(rotDir);
      double groupDirW = computeGroupDirW(pathList_[pathIdx]);
      double score = scorePath(dirDiff, rotDirW, groupDirW,
                               pathPenaltyList_[i], relGoalDis, sp);
      if (score > 0) {
        int groupIdx = kGroupNum * rotDir + pathList_[pathIdx];
        clearPathPerGroupScore_[groupIdx] += score;
        clearPathPerGroupNum_[groupIdx]++;
        pathPenaltyPerGroupScore_[groupIdx] += pathPenaltyList_[i];
      }
    }

    // Select best group
    int selectedGroupID = -1;
    double maxScore = 0;
    for (int i = 0; i < totalGroups; i++) {
      if (clearPathPerGroupScore_[i] > maxScore) {
        maxScore = clearPathPerGroupScore_[i];
        selectedGroupID = i;
      }
    }

    // Compute slow-down
    if (selectedGroupID >= 0) {
      int num = clearPathPerGroupNum_[selectedGroupID];
      float penaltyScore = (num > 0) ? pathPenaltyPerGroupScore_[selectedGroupID] / num : 0;
      if (penaltyScore > p_.costHeightThre1) slowDown = 1;
      else if (penaltyScore > p_.costHeightThre2) slowDown = 2;
      else if (num < p_.slowPathNumThre &&
               std::abs(selectedGroupID - 129) > p_.slowGroupNumThre) slowDown = 3;
      else slowDown = 0;
    }

    return selectedGroupID;
  }

  void buildOutputPath(int selectedGroupID, double pathScale, double pathRange,
                       double relGoalDis, LocalPlanResult& result) {
    const auto& lut = rotLUT();
    int rotDir = selectedGroupID / kGroupNum;
    int groupID = selectedGroupID % kGroupNum;
    double rc = lut.c[rotDir], rs = lut.s[rotDir];

    double pathRangeScaleSq = (pathRange / pathScale) * (pathRange / pathScale);
    double relGoalScaledSq = (relGoalDis / pathScale) * (relGoalDis / pathScale);

    auto& seg = startPaths_[groupID];
    result.path.clear();
    result.path.reserve(seg.size());

    for (auto& pt : seg) {
      double disSq = pt.x * pt.x + pt.y * pt.y;
      if (disSq > pathRangeScaleSq || disSq > relGoalScaledSq) break;
      // Rotate back to body frame, then scale
      double bx = pathScale * (rc * pt.x - rs * pt.y);
      double by = pathScale * (rs * pt.x + rc * pt.y);
      double bz = pathScale * pt.z;
      result.path.push_back({bx, by, bz});
    }
    result.pathFound = !result.path.empty();
  }

  void buildRecoveryPath(LocalPlanResult& result) {
    if (blockedStartTime_ < 0) blockedStartTime_ = odomTime_;
    double blockedDur = odomTime_ - blockedStartTime_;

    if (recoveryState_ == 0 && blockedDur >= p_.recoveryBlockedThre) {
      if (recoveryCycleCount_ >= p_.recoveryMaxCycles) {
        // Exhausted recovery cycles — stop and wait
      } else {
        recoveryState_ = 1;
        recoveryPhaseStart_ = odomTime_;
      }
    } else if (recoveryState_ == 1 &&
               odomTime_ - recoveryPhaseStart_ >= p_.recoveryRotateTime) {
      recoveryState_ = 2;
      recoveryPhaseStart_ = odomTime_;
    } else if (recoveryState_ == 2 &&
               odomTime_ - recoveryPhaseStart_ >= p_.recoveryBackupTime) {
      recoveryState_ = 0;
      blockedStartTime_ = odomTime_;
      recoveryCycleCount_++;
    }

    result.path.clear();

    if (recoveryState_ == 1) {
      // Rotate toward goal
      double goalAngle = std::atan2(goalY_ - vy_, goalX_ - vx_);
      double relAngle = goalAngle - vyaw_;
      while (relAngle > M_PI) relAngle -= 2.0 * M_PI;
      while (relAngle < -M_PI) relAngle += 2.0 * M_PI;
      double turnDir = (relAngle >= 0) ? 1.0 : -1.0;

      // Check rotation safety
      bool safe = true;
      for (auto& pt : plannerCloud_) {
        double dist = std::hypot(pt.x, pt.y);
        if (dist < 0.9 && dist > 0.1) {
          double ptAngle = std::atan2(pt.y, pt.x);
          bool inSweep = turnDir > 0 ? (ptAngle > 0 && ptAngle < 1.5)
                                     : (ptAngle < 0 && ptAngle > -1.5);
          if (inSweep && (pt.intensity > p_.obstacleHeightThre * 0.5 || !p_.useTerrainAnalysis)) {
            safe = false;
            break;
          }
        }
      }
      double dir = safe ? turnDir : -turnDir;
      for (int i = 1; i <= 6; i++) {
        double arc = dir * i * 0.25;
        result.path.push_back({0.15 * i * std::cos(arc), 0.15 * i * std::sin(arc), 0.0});
      }
    } else if (recoveryState_ == 2) {
      // Backup: check rear safety
      bool safe = true;
      for (auto& pt : plannerCloud_) {
        if (pt.x < -0.1f && pt.x > -1.2f && std::fabs(pt.y) < 0.35f) {
          if (pt.intensity > p_.obstacleHeightThre * 0.5 || !p_.useTerrainAnalysis) {
            safe = false; break;
          }
        }
      }
      if (safe) {
        for (int i = 1; i <= 5; i++)
          result.path.push_back({-0.2 * i, 0.0, 0.0});
      } else {
        recoveryState_ = 0;
        blockedStartTime_ = odomTime_;
        recoveryCycleCount_++;
        result.path.push_back({0, 0, 0});
      }
    } else {
      result.path.push_back({0, 0, 0});
    }
  }
};

}  // namespace nav_core
