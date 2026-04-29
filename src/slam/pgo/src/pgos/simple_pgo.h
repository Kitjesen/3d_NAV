#pragma once
#include "commons.h"
#include "scan_context.h"
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

struct KeyPoseWithCloud
{
    M3D r_local;
    V3D t_local;
    M3D r_global;
    V3D t_global;
    double time;
    CloudType::Ptr body_cloud;
};
struct LoopPair
{
    size_t source_id;
    size_t target_id;
    M3D r_offset;
    V3D t_offset;
    double score;
};

struct Config
{
    double key_pose_delta_deg = 10;
    double key_pose_delta_trans = 1.0;
    // Search radius default enlarged from 1.0 → 15.0 once Scan Context is on:
    // SC pre-filter rejects spatially-near-but-not-actually-revisited candidates,
    // so we can afford a much larger spatial net to catch real revisits the
    // 1m radius would have missed.
    double loop_search_radius = 15.0;
    double loop_time_tresh = 60.0;
    double loop_score_tresh = 0.15;
    int loop_submap_half_range = 5;
    double submap_resolution = 0.1;
    double min_loop_detect_duration = 10.0;

    // Scan Context loop pre-filter (Kim & Kim 2018). Provides indoor global
    // anchoring when GNSS is unavailable. When enabled, the radius search
    // becomes a sanity check on the SC-proposed candidate, and SC's coarse
    // yaw estimate is fed to ICP as an initial guess for faster/safer
    // convergence. See scan_context.h for tuning knobs.
    bool   enable_scan_context = true;
    double sc_distance_threshold = 0.4;  // <0.4 typical for true matches
    double sc_max_range = 20.0;          // m — points beyond clipped from descriptor
    int    sc_num_candidates = 5;        // top-K from SC KdTree
    int    sc_min_history_gap = 30;      // skip last N keyframes
};

class SimplePGO
{
public:
    SimplePGO(const Config &config);

    bool isKeyPose(const PoseWithTime &pose);

    bool addKeyPose(const CloudWithPose &cloud_with_pose);

    bool hasLoop(){return m_cache_pairs.size() > 0;}

    void searchForLoopPairs();

    void smoothAndUpdate();

    CloudType::Ptr getSubMap(int idx, int half_range, double resolution);
    std::vector<std::pair<size_t, size_t>> &historyPairs() { return m_history_pairs; }
    std::vector<KeyPoseWithCloud> &keyPoses() { return m_key_poses; }

    M3D offsetR() { return m_r_offset; }
    V3D offsetT() { return m_t_offset; }

private:
    Config m_config;
    std::vector<KeyPoseWithCloud> m_key_poses;
    std::vector<std::pair<size_t, size_t>> m_history_pairs;
    std::vector<LoopPair> m_cache_pairs;
    M3D m_r_offset;
    V3D m_t_offset;
    std::shared_ptr<gtsam::ISAM2> m_isam2;
    gtsam::Values m_initial_values;
    gtsam::NonlinearFactorGraph m_graph;
    pcl::IterativeClosestPoint<PointType, PointType> m_icp;

    // Scan Context place recognition. Fed in addKeyPose(); queried in
    // searchForLoopPairs() as the primary candidate proposer when enabled.
    ScanContext m_sc;
};