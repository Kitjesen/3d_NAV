#include "icp_localizer.h"

ICPLocalizer::ICPLocalizer(const ICPConfig &config) : m_config(config)
{
    m_refine_inp.reset(new CloudType);
    m_refine_tgt.reset(new CloudType);
    m_rough_inp.reset(new CloudType);
    m_rough_tgt.reset(new CloudType);

    // small_gicp registration tuning. We default to plain ICP to preserve
    // the previous behaviour bit-for-bit; switching to GICP requires
    // re-tuning refine_score_thresh because the fitness metric changes
    // (point-to-plane vs point-to-point).
    m_rough_icp.setNumThreads(m_config.num_threads);
    m_rough_icp.setRegistrationType("ICP");
    m_refine_icp.setNumThreads(m_config.num_threads);
    m_refine_icp.setRegistrationType("ICP");
}
bool ICPLocalizer::loadMap(const std::string &path)
{
    if (!std::filesystem::exists(path))
    {
        std::cerr << "Map file not found: " << path << std::endl;
        return false;
    }
    pcl::PCDReader reader;
    CloudType::Ptr cloud(new CloudType);
    reader.read(path, *cloud);
    if (m_config.refine_map_resolution > 0)
    {
        m_voxel_filter.setLeafSize(m_config.refine_map_resolution, m_config.refine_map_resolution, m_config.refine_map_resolution);
        m_voxel_filter.setInputCloud(cloud);
        m_voxel_filter.filter(*m_refine_tgt);
    }
    else
    {
        pcl::copyPointCloud(*cloud, *m_refine_tgt);
    }

    if (m_config.rough_map_resolution > 0)
    {
        m_voxel_filter.setLeafSize(m_config.rough_map_resolution, m_config.rough_map_resolution, m_config.rough_map_resolution);
        m_voxel_filter.setInputCloud(cloud);
        m_voxel_filter.filter(*m_rough_tgt);
    }
    else
    {
        pcl::copyPointCloud(*cloud, *m_rough_tgt);
    }

    // 预构建 KD-tree: setInputTarget 触发 O(N logN) 的 KD-tree 构建。
    // 在 loadMap 时做一次, 而非每次 align() 都重复, 节省 10-50ms/帧。
    m_rough_icp.setMaximumIterations(m_config.rough_max_iteration);
    m_rough_icp.setInputTarget(m_rough_tgt);
    m_refine_icp.setMaximumIterations(m_config.refine_max_iteration);
    m_refine_icp.setInputTarget(m_refine_tgt);

    return true;
}
void ICPLocalizer::setInput(const CloudType::Ptr &cloud)
{
    if (m_config.refine_scan_resolution > 0)
    {
        m_voxel_filter.setLeafSize(m_config.refine_scan_resolution, m_config.refine_scan_resolution, m_config.refine_scan_resolution);
        m_voxel_filter.setInputCloud(cloud);
        m_voxel_filter.filter(*m_refine_inp);
    }
    else
    {
        pcl::copyPointCloud(*cloud, *m_refine_inp);
    }

    if (m_config.rough_scan_resolution > 0)
    {
        m_voxel_filter.setLeafSize(m_config.rough_scan_resolution, m_config.rough_scan_resolution, m_config.rough_scan_resolution);
        m_voxel_filter.setInputCloud(cloud);
        m_voxel_filter.filter(*m_rough_inp);
    }
    else
    {
        pcl::copyPointCloud(*cloud, *m_rough_inp);
    }
}

bool ICPLocalizer::align(M4F &guess)
{
    CloudType::Ptr aligned_cloud(new CloudType);
    if (m_refine_tgt->size() == 0 || m_rough_tgt->size() == 0) {
        m_last_fitness_score = -1.0;
        m_last_iterations = -1;
        m_last_converged = false;
        m_last_pos_cov_trace = -1.0;
        return false;
    }
    // setInputTarget + setMaximumIterations were done in loadMap()
    // (KD-tree already cached).
    m_rough_icp.setInputSource(m_rough_inp);
    m_rough_icp.align(*aligned_cloud, guess);
    if (!m_rough_icp.hasConverged() ||
        m_rough_icp.getFitnessScore() > m_config.rough_score_thresh) {
        m_last_fitness_score = m_rough_icp.getFitnessScore();
        m_last_iterations = m_rough_icp.getRegistrationResult().iterations;
        m_last_converged = false;
        m_last_pos_cov_trace = -1.0;
        return false;
    }
    m_refine_icp.setInputSource(m_refine_inp);
    m_refine_icp.align(*aligned_cloud, m_rough_icp.getFinalTransformation());
    m_last_fitness_score = m_refine_icp.getFitnessScore();

    // R4: capture full diagnostics from small_gicp's refine result.
    // Position covariance is the top-left 3x3 of H^-1; we surface its
    // trace because the localizer's downstream consumer (P3) reduces
    // to a scalar health value anyway. Inverting only the 3x3 block
    // (instead of the full 6x6 H) is both cheap and well-conditioned —
    // the rotation block can be near-singular under planar scenes and
    // would otherwise blow up the trace numerically.
    const auto &refine_result = m_refine_icp.getRegistrationResult();
    m_last_iterations = refine_result.iterations;
    m_last_converged = refine_result.converged;
    {
        const Eigen::Matrix<double, 6, 6> H = m_refine_icp.getFinalHessian();
        const Eigen::Matrix3d H_pos = H.block<3, 3>(0, 0);
        // Guard against singular H_pos (planar / degenerate observation).
        const Eigen::JacobiSVD<Eigen::Matrix3d> svd(H_pos);
        const double cond = svd.singularValues()(0)
                            / std::max(svd.singularValues()(2), 1e-12);
        if (cond > 1e8 || svd.singularValues()(2) < 1e-9) {
            m_last_pos_cov_trace = -1.0;  // sentinel: ill-conditioned
        } else {
            m_last_pos_cov_trace = H_pos.inverse().trace();
        }
    }

    if (!m_last_converged || m_last_fitness_score > m_config.refine_score_thresh)
        return false;
    guess = m_refine_icp.getFinalTransformation();
    return true;
}