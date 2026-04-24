#include "ieskf.h"

double State::gravity = 9.81;

M3D Jr(const V3D &inp)
{
    return Sophus::SO3d::leftJacobian(inp).transpose();
}
M3D JrInv(const V3D &inp)
{
    return Sophus::SO3d::leftJacobianInverse(inp).transpose();
}

void State::operator+=(const V21D &delta)
{
    r_wi *= Sophus::SO3d::exp(delta.segment<3>(0)).matrix();
    t_wi += delta.segment<3>(3);
    r_il *= Sophus::SO3d::exp(delta.segment<3>(6)).matrix();
    t_il += delta.segment<3>(9);
    v += delta.segment<3>(12);
    bg += delta.segment<3>(15);
    ba += delta.segment<3>(18);
}

V21D State::operator-(const State &other) const
{
    V21D delta = V21D::Zero();
    delta.segment<3>(0) = Sophus::SO3d(other.r_wi.transpose() * r_wi).log();
    delta.segment<3>(3) = t_wi - other.t_wi;
    delta.segment<3>(6) = Sophus::SO3d(other.r_il.transpose() * r_il).log();
    delta.segment<3>(9) = t_il - other.t_il;
    delta.segment<3>(12) = v - other.v;
    delta.segment<3>(15) = bg - other.bg;
    delta.segment<3>(18) = ba - other.ba;
    return delta;
}

std::ostream &operator<<(std::ostream &os, const State &state)
{
    os << "==============START===============" << std::endl;
    os << "r_wi: " << state.r_wi.eulerAngles(2, 1, 0).transpose() << std::endl;
    os << "t_il: " << state.t_il.transpose() << std::endl;
    os << "r_il: " << state.r_il.eulerAngles(2, 1, 0).transpose() << std::endl;
    os << "t_wi: " << state.t_wi.transpose() << std::endl;
    os << "v: " << state.v.transpose() << std::endl;
    os << "bg: " << state.bg.transpose() << std::endl;
    os << "ba: " << state.ba.transpose() << std::endl;
    os << "g: " << state.g.transpose() << std::endl;
    os << "===============END================" << std::endl;

    return os;
}

void IESKF::predict(const Input &inp, double dt, const M12D &Q)
{
    V21D delta = V21D::Zero();
    delta.segment<3>(0) = (inp.gyro - m_x.bg) * dt;
    delta.segment<3>(3) = m_x.v * dt;
    delta.segment<3>(12) = (m_x.r_wi * (inp.acc - m_x.ba) + m_x.g) * dt;

    m_F.setIdentity();
    m_F.block<3, 3>(0, 0) = Sophus::SO3d::exp(-(inp.gyro - m_x.bg) * dt).matrix();
    m_F.block<3, 3>(0, 15) = -Jr((inp.gyro - m_x.bg) * dt) * dt;
    m_F.block<3, 3>(3, 12) = Eigen::Matrix3d::Identity() * dt;
    m_F.block<3, 3>(12, 0) = -m_x.r_wi * Sophus::SO3d::hat(inp.acc - m_x.ba) * dt;
    m_F.block<3, 3>(12, 18) = -m_x.r_wi * dt;

    m_G.setZero();
    m_G.block<3, 3>(0, 0) = -Jr((inp.gyro - m_x.bg) * dt) * dt;
    m_G.block<3, 3>(12, 3) = -m_x.r_wi * dt;
    m_G.block<3, 3>(15, 6) = Eigen::Matrix3d::Identity() * dt;
    m_G.block<3, 3>(18, 9) = Eigen::Matrix3d::Identity() * dt;

    m_x += delta;
    m_P = m_F * m_P * m_F.transpose() + m_G * Q * m_G.transpose();
    clampCovariance();
}

void IESKF::clampCovariance()
{
    for (int i = 0; i < 21; ++i)
        m_P(i, i) = std::min(m_P(i, i), P_MAX[i]);
}

void IESKF::injectZUPT(double sigma_v, double sigma_pos)
{
    // Observe velocity states [12:15] with z = 0 (robot is stationary)
    Eigen::Matrix<double, 3, 21> H_zupt = Eigen::Matrix<double, 3, 21>::Zero();
    H_zupt.block<3, 3>(0, 12) = Eigen::Matrix3d::Identity();

    Eigen::Matrix3d R_zupt = Eigen::Matrix3d::Identity() * sigma_v * sigma_v;
    V3D innov = -m_x.v;  // z - H*x = 0 - v

    Eigen::Matrix3d S = H_zupt * m_P * H_zupt.transpose() + R_zupt;
    Eigen::Matrix<double, 21, 3> K = m_P * H_zupt.transpose() * S.inverse();

    m_x += K * innov;
    m_P = (M21D::Identity() - K * H_zupt) * m_P;

    // Tighten position covariance when robot is confirmed stationary
    if (sigma_pos > 0.0)
    {
        for (int i = 3; i < 6; ++i)
            m_P(i, i) = std::min(m_P(i, i), sigma_pos * sigma_pos);
    }
    clampCovariance();
}

void IESKF::update()
{
    State predict_x = m_x;
    SharedState shared_data;
    shared_data.iter_num = 0;
    shared_data.res = 1e10;
    V21D delta = V21D::Zero();
    M21D H = M21D::Identity();
    V21D b;

    bool skip_lidar_update = false;

    for (size_t i = 0; i < m_max_iter; i++)
    {
        m_loss_func(m_x, shared_data);
        if (!shared_data.valid)
            break;
        H.setZero();
        b.setZero();
        delta = m_x - predict_x;
        M21D J = M21D::Identity();
        J.block<3, 3>(0, 0) = JrInv(delta.segment<3>(0));
        J.block<3, 3>(6, 6) = JrInv(delta.segment<3>(6));
        // P^{-1} 只需解一次, 用 LDLT 代替显式求逆 (数值稳定 + 快 2-3x)
        auto P_ldlt = m_P.ldlt();
        H += J.transpose() * P_ldlt.solve(J);
        b += J.transpose() * P_ldlt.solve(delta);

        H.block<12, 12>(0, 0) += shared_data.H;
        b.block<12, 1>(0, 0) += shared_data.b;

        // ── Degeneracy detection (DALI-SLAM / Zhang 2016 approach) ──────
        // Analyze the 6x6 pose block of the LiDAR Hessian (rotation + translation)
        // to detect degenerate directions where LiDAR provides no constraint.
        // When degenerate, remap the solution to suppress updates along those axes.
        if (i == 0)  // Only compute on first iteration (H structure is stable)
        {
            Eigen::Matrix<double, 6, 6> H_pose = shared_data.H.block<6, 6>(0, 0);
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> solver(H_pose);
            auto &evals = solver.eigenvalues();   // sorted ascending
            auto &evecs = solver.eigenvectors();

            shared_data.degeneracy.eigenvalues = evals;
            shared_data.degeneracy.min_eigenvalue = evals(0);
            shared_data.degeneracy.max_eigenvalue = evals(5);
            shared_data.degeneracy.condition_number =
                (evals(0) > 1e-10) ? evals(5) / evals(0) : 1e12;

            // Threshold: eigenvalue < 1% of max → degenerate
            // Based on Zhang 2016 "On Degeneracy of Optimization-based State Estimation"
            const double degen_thresh = evals(5) * 0.01;
            int degen_count = 0;
            Eigen::Matrix<double, 6, 1> mask = Eigen::Matrix<double, 6, 1>::Ones();

            for (int d = 0; d < 6; ++d)
            {
                if (evals(d) < degen_thresh)
                {
                    degen_count++;
                    mask(d) = 0.0;
                }
            }
            shared_data.degeneracy.degenerate_dof_count = degen_count;
            shared_data.degeneracy.dof_mask = mask;
            shared_data.degeneracy.effective_ratio =
                (6.0 - degen_count) / 6.0;
            shared_data.degeneracy.detected = (degen_count > 0);

            // ── Severe degeneracy: skip LiDAR update entirely ──────────
            // When >= 3 DOFs are degenerate or condition number is extreme,
            // LiDAR provides too few constraints — IMU prediction is safer.
            if (degen_count >= 3 || shared_data.degeneracy.condition_number > 1e8)
            {
                skip_lidar_update = true;
                break;
            }

            // ── Partial degeneracy (1-2 DOFs): project to well-constrained subspace
            if (degen_count > 0)
            {
                // Save prior contribution (must not be overwritten)
                M21D H_prior = J.transpose() * P_ldlt.solve(J);
                V21D b_prior = J.transpose() * P_ldlt.solve(delta);

                // Build projection matrix: V_good * V_good^T
                Eigen::Matrix<double, 6, 6> P_good = Eigen::Matrix<double, 6, 6>::Zero();
                for (int d = 0; d < 6; ++d)
                {
                    if (evals(d) >= degen_thresh)
                        P_good += evecs.col(d) * evecs.col(d).transpose();
                }
                Eigen::Matrix<double, 6, 6> P_bad = Eigen::Matrix<double, 6, 6>::Identity() - P_good;
                double regularize = evals(5) * 0.01;  // Stronger regularization
                Eigen::Matrix<double, 6, 6> H_lidar_remapped =
                    P_good * H_pose * P_good + P_bad * regularize;
                Eigen::Matrix<double, 6, 1> b_lidar_remapped =
                    P_good * shared_data.b.head<6>();

                // Reconstruct H = prior + remapped LiDAR (preserving prior!)
                H.block<6, 6>(0, 0) = H_prior.block<6, 6>(0, 0) + H_lidar_remapped;
                b.block<6, 1>(0, 0) = b_prior.block<6, 1>(0, 0) + b_lidar_remapped;
            }
        }

        delta = -H.ldlt().solve(b);

        m_x += delta;
        shared_data.iter_num += 1;

        if (m_stop_func(delta))
            break;
    }

    // Store degeneracy info for external access (ROS2 publisher)
    m_degeneracy = shared_data.degeneracy;

    // Severe degeneracy: revert to IMU prediction, clamp covariance to prevent unbounded growth
    if (skip_lidar_update)
    {
        m_x = predict_x;
        clampCovariance();
        return;
    }

    M21D L = M21D::Identity();
    L.block<3, 3>(0, 0) = Jr(delta.segment<3>(0));
    L.block<3, 3>(6, 6) = Jr(delta.segment<3>(6));
    m_P = L * H.ldlt().solve(L.transpose());  // P = L * H^{-1} * L^T
    clampCovariance();
}