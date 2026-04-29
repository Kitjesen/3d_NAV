#pragma once
#include "commons.h"
#include <filesystem>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <small_gicp/pcl/pcl_registration.hpp>

struct ICPConfig
{
    double refine_scan_resolution = 0.1;
    double refine_map_resolution = 0.1;
    double refine_score_thresh = 0.1;
    int refine_max_iteration = 10;

    double rough_scan_resolution = 0.25;
    double rough_map_resolution = 0.25;
    double rough_score_thresh = 0.2;
    int rough_max_iteration = 5;

    // Number of OpenMP threads for small_gicp parallel reduction. Set 0
    // (default) to let the implementation pick something reasonable; set
    // explicitly when running alongside SLAM/PGO that already saturate
    // the CPU and we want to bound the localizer's footprint.
    int num_threads = 4;
};

class ICPLocalizer
{
public:
    ICPLocalizer(const ICPConfig &config);

    bool loadMap(const std::string &path);

    void setInput(const CloudType::Ptr &cloud);

    bool align(M4F &guess);
    ICPConfig &config() { return m_config; }
    CloudType::Ptr roughMap() { return m_rough_tgt; }
    CloudType::Ptr refineMap() { return m_refine_tgt; }

    /// Last ICP fitness score (lower = better alignment, 0 = perfect)
    double getLastFitnessScore() const { return m_last_fitness_score; }

    /// R4: three-axis health diagnostics from small_gicp. Together with
    /// fitness these give the multi-frame health gate (P3) the iter +
    /// covariance signals it was missing under PCL ICP. All values come
    /// from the *refine* stage's RegistrationResult — rough is a coarse
    /// pre-step whose iteration count is uninformative.
    int    getLastIterations()  const { return m_last_iterations; }
    bool   getLastConverged()   const { return m_last_converged; }

    /// Trace of the position covariance estimated from the registration
    /// Hessian (top-left 3x3 block of H^-1). Larger = more uncertain
    /// translation. Returns -1 when no align() has succeeded yet.
    double getLastPosCovTrace() const { return m_last_pos_cov_trace; }

private:
    ICPConfig m_config;
    pcl::VoxelGrid<PointType> m_voxel_filter;
    // Drop-in replacement for pcl::IterativeClosestPoint. small_gicp's
    // RegistrationPCL inherits from pcl::Registration so the call sites
    // (setInputSource/Target, align) are unchanged. The win is the
    // post-align getters: getFinalHessian() + getRegistrationResult()
    // expose the diagnostics PCL ICP swallowed.
    small_gicp::RegistrationPCL<PointType, PointType> m_refine_icp;
    small_gicp::RegistrationPCL<PointType, PointType> m_rough_icp;
    CloudType::Ptr m_refine_inp;
    CloudType::Ptr m_rough_inp;
    CloudType::Ptr m_refine_tgt;
    CloudType::Ptr m_rough_tgt;
    std::string m_pcd_path;
    double m_last_fitness_score{-1.0};   // -1 = not yet computed
    int    m_last_iterations{-1};
    bool   m_last_converged{false};
    double m_last_pos_cov_trace{-1.0};   // -1 = not yet computed
};