#include "simple_pgo.h"

SimplePGO::SimplePGO(const Config &config) : m_config(config),
    m_sc(ScanContext::Config{
        config.sc_max_range,
        /*ring_key_threshold=*/0.4,
        config.sc_distance_threshold,
        config.sc_num_candidates,
        config.sc_min_history_gap,
    })
{
    gtsam::ISAM2Params isam2_params;
    isam2_params.relinearizeThreshold = 0.01;
    isam2_params.relinearizeSkip = 1;
    m_isam2 = std::make_shared<gtsam::ISAM2>(isam2_params);
    m_initial_values.clear();
    m_graph.resize(0);
    m_r_offset.setIdentity();
    m_t_offset.setZero();

    m_icp.setMaximumIterations(50);
    m_icp.setMaxCorrespondenceDistance(10);
    m_icp.setTransformationEpsilon(1e-6);
    m_icp.setEuclideanFitnessEpsilon(1e-6);
    m_icp.setRANSACIterations(0);
}

bool SimplePGO::isKeyPose(const PoseWithTime &pose)
{
    if (m_key_poses.size() == 0)
        return true;
    const KeyPoseWithCloud &last_item = m_key_poses.back();
    double delta_trans = (pose.t - last_item.t_local).norm();
    double delta_deg = Eigen::Quaterniond(pose.r).angularDistance(Eigen::Quaterniond(last_item.r_local)) * 57.324;
    if (delta_trans > m_config.key_pose_delta_trans || delta_deg > m_config.key_pose_delta_deg)
        return true;
    return false;
}
bool SimplePGO::addKeyPose(const CloudWithPose &cloud_with_pose)
{
    bool is_key_pose = isKeyPose(cloud_with_pose.pose);
    if (!is_key_pose)
        return false;
    size_t idx = m_key_poses.size();
    M3D init_r = m_r_offset * cloud_with_pose.pose.r;
    V3D init_t = m_r_offset * cloud_with_pose.pose.t + m_t_offset;
    // 添加初始值
    m_initial_values.insert(idx, gtsam::Pose3(gtsam::Rot3(init_r), gtsam::Point3(init_t)));
    if (idx == 0)
    {
        // 添加先验约束
        gtsam::noiseModel::Diagonal::shared_ptr noise = gtsam::noiseModel::Diagonal::Variances(gtsam::Vector6::Ones() * 1e-12);
        m_graph.add(gtsam::PriorFactor<gtsam::Pose3>(idx, gtsam::Pose3(gtsam::Rot3(init_r), gtsam::Point3(init_t)), noise));
    }
    else
    {
        // 添加里程计约束
        const KeyPoseWithCloud &last_item = m_key_poses.back();
        M3D r_between = last_item.r_local.transpose() * cloud_with_pose.pose.r;
        V3D t_between = last_item.r_local.transpose() * (cloud_with_pose.pose.t - last_item.t_local);
        gtsam::noiseModel::Diagonal::shared_ptr noise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-6).finished());
        m_graph.add(gtsam::BetweenFactor<gtsam::Pose3>(idx - 1, idx, gtsam::Pose3(gtsam::Rot3(r_between), gtsam::Point3(t_between)), noise));
    }
    KeyPoseWithCloud item;
    item.time = cloud_with_pose.pose.second;
    item.r_local = cloud_with_pose.pose.r;
    item.t_local = cloud_with_pose.pose.t;
    item.body_cloud = cloud_with_pose.cloud;
    item.r_global = init_r;
    item.t_global = init_t;
    m_key_poses.push_back(item);

    // Feed Scan Context with the body-frame cloud so descriptor indices stay
    // 1:1 with m_key_poses indices. Only when SC is enabled — the descriptor
    // build is non-trivial cost (~1ms per scan).
    if (m_config.enable_scan_context && cloud_with_pose.cloud)
        m_sc.add(cloud_with_pose.cloud);

    return true;
}

CloudType::Ptr SimplePGO::getSubMap(int idx, int half_range, double resolution)
{
    assert(idx >= 0 && idx < static_cast<int>(m_key_poses.size()));
    int min_idx = std::max(0, idx - half_range);
    int max_idx = std::min(static_cast<int>(m_key_poses.size()) - 1, idx + half_range);

    CloudType::Ptr ret(new CloudType);
    for (int i = min_idx; i <= max_idx; i++)
    {

        CloudType::Ptr body_cloud = m_key_poses[i].body_cloud;
        CloudType::Ptr global_cloud(new CloudType);
        pcl::transformPointCloud(*body_cloud, *global_cloud, m_key_poses[i].t_global, Eigen::Quaterniond(m_key_poses[i].r_global));
        *ret += *global_cloud;
    }
    if (resolution > 0)
    {
        pcl::VoxelGrid<PointType> voxel_grid;
        voxel_grid.setLeafSize(resolution, resolution, resolution);
        voxel_grid.setInputCloud(ret);
        voxel_grid.filter(*ret);
    }
    return ret;
}

void SimplePGO::searchForLoopPairs()
{
    if (m_key_poses.size() < 10)
        return;
    if (m_config.min_loop_detect_duration > 0.0)
    {
        if (m_history_pairs.size() > 0)
        {
            double current_time = m_key_poses.back().time;
            double last_time = m_key_poses[m_history_pairs.back().second].time;
            if (current_time - last_time < m_config.min_loop_detect_duration)
                return;
        }
    }

    size_t cur_idx = m_key_poses.size() - 1;
    const KeyPoseWithCloud &last_item = m_key_poses.back();

    // ── Candidate proposal ────────────────────────────────────────────────
    // Two paths:
    //   (A) Scan Context primary (default for indoor). SC scores by descriptor
    //       similarity over the entire history, then we sanity-check the
    //       chosen candidate is within `loop_search_radius` so we do not add
    //       a wildly-wrong loop on the rare false-positive descriptor match.
    //   (B) Pure radius+time fallback (legacy). Used when SC is disabled.
    int loop_idx = -1;
    float sc_yaw = 0.0f;

    if (m_config.enable_scan_context && m_sc.size() == m_key_poses.size())
    {
        auto sc_result = m_sc.query(cur_idx);
        loop_idx = sc_result.first;
        sc_yaw = sc_result.second;
        if (loop_idx >= 0)
        {
            // Time gap check — same as legacy (avoid matching recent frames).
            if (std::abs(last_item.time - m_key_poses[loop_idx].time) <= m_config.loop_time_tresh)
                loop_idx = -1;
        }
        if (loop_idx >= 0)
        {
            // Spatial sanity: SC matches that are physically far away are
            // almost always bogus (similar geometry in unrelated places —
            // common in long corridors / repeated rooms). Reject.
            const V3D dt = m_key_poses[loop_idx].t_global - last_item.t_global;
            if (dt.norm() > m_config.loop_search_radius)
                loop_idx = -1;
        }
    }
    else
    {
        // Legacy radius+time path (preserved for outdoor / debug fallback).
        pcl::PointXYZ last_pose_pt;
        last_pose_pt.x = last_item.t_global(0);
        last_pose_pt.y = last_item.t_global(1);
        last_pose_pt.z = last_item.t_global(2);
        pcl::PointCloud<pcl::PointXYZ>::Ptr key_poses_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (size_t i = 0; i < m_key_poses.size() - 1; i++)
        {
            pcl::PointXYZ pt;
            pt.x = m_key_poses[i].t_global(0);
            pt.y = m_key_poses[i].t_global(1);
            pt.z = m_key_poses[i].t_global(2);
            key_poses_cloud->push_back(pt);
        }
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(key_poses_cloud);
        std::vector<int> ids;
        std::vector<float> sqdists;
        int neighbors = kdtree.radiusSearch(last_pose_pt, m_config.loop_search_radius, ids, sqdists);
        if (neighbors == 0)
            return;
        for (size_t i = 0; i < ids.size(); i++)
        {
            int idx = ids[i];
            if (std::abs(last_item.time - m_key_poses[idx].time) > m_config.loop_time_tresh)
            {
                loop_idx = idx;
                break;
            }
        }
    }

    if (loop_idx == -1)
        return;

    // ── ICP refinement ────────────────────────────────────────────────────
    // Same submap approach as before. With SC enabled we feed the coarse yaw
    // as ICP's initial guess so it converges within 5-10 iterations even with
    // significant orientation mismatch (otherwise ICP can stall in a yaw-flip
    // local minimum).
    CloudType::Ptr target_cloud = getSubMap(loop_idx, m_config.loop_submap_half_range, m_config.submap_resolution);
    CloudType::Ptr source_cloud = getSubMap(m_key_poses.size() - 1, 0, m_config.submap_resolution);
    CloudType::Ptr align_cloud(new CloudType);

    m_icp.setInputSource(source_cloud);
    m_icp.setInputTarget(target_cloud);

    if (m_config.enable_scan_context && std::abs(sc_yaw) > 1e-3f)
    {
        Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();
        const float c = std::cos(sc_yaw);
        const float s = std::sin(sc_yaw);
        init_guess(0, 0) = c; init_guess(0, 1) = -s;
        init_guess(1, 0) = s; init_guess(1, 1) =  c;
        m_icp.align(*align_cloud, init_guess);
    }
    else
    {
        m_icp.align(*align_cloud);
    }

    if (!m_icp.hasConverged() || m_icp.getFitnessScore() > m_config.loop_score_tresh)
        return;

    M4F loop_transform = m_icp.getFinalTransformation();

    LoopPair one_pair;
    one_pair.source_id = cur_idx;
    one_pair.target_id = loop_idx;
    one_pair.score = m_icp.getFitnessScore();
    M3D r_refined = loop_transform.block<3, 3>(0, 0).cast<double>() * m_key_poses[cur_idx].r_global;
    V3D t_refined = loop_transform.block<3, 3>(0, 0).cast<double>() * m_key_poses[cur_idx].t_global + loop_transform.block<3, 1>(0, 3).cast<double>();
    one_pair.r_offset = m_key_poses[loop_idx].r_global.transpose() * r_refined;
    one_pair.t_offset = m_key_poses[loop_idx].r_global.transpose() * (t_refined - m_key_poses[loop_idx].t_global);
    m_cache_pairs.push_back(one_pair);
    m_history_pairs.emplace_back(one_pair.target_id, one_pair.source_id);
}

void SimplePGO::smoothAndUpdate()
{
    bool has_loop = !m_cache_pairs.empty();
    // 添加回环因子
    if (has_loop)
    {
        for (LoopPair &pair : m_cache_pairs)
        {
            m_graph.add(gtsam::BetweenFactor<gtsam::Pose3>(pair.target_id, pair.source_id,
                                                           gtsam::Pose3(gtsam::Rot3(pair.r_offset),
                                                                        gtsam::Point3(pair.t_offset)),
                                                           gtsam::noiseModel::Diagonal::Variances(gtsam::Vector6::Ones() * pair.score)));
        }
        std::vector<LoopPair>().swap(m_cache_pairs);
    }
    // smooth and mapping
    m_isam2->update(m_graph, m_initial_values);
    m_isam2->update();
    if (has_loop)
    {
        m_isam2->update();
        m_isam2->update();
        m_isam2->update();
        m_isam2->update();
    }
    m_graph.resize(0);
    m_initial_values.clear();

    // update key poses
    gtsam::Values estimate_values = m_isam2->calculateBestEstimate();
    for (size_t i = 0; i < m_key_poses.size(); i++)
    {
        gtsam::Pose3 pose = estimate_values.at<gtsam::Pose3>(i);
        m_key_poses[i].r_global = pose.rotation().matrix().cast<double>();
        m_key_poses[i].t_global = pose.translation().matrix().cast<double>();
    }
    // update offset
    const KeyPoseWithCloud &last_item = m_key_poses.back();
    m_r_offset = last_item.r_global * last_item.r_local.transpose();
    m_t_offset = last_item.t_global - m_r_offset * last_item.t_local;
}