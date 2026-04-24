#pragma once
#include <Eigen/Eigen>
#include <sophus/so3.hpp>
#include "commons.h"

using M12D = Eigen::Matrix<double, 12, 12>;
using M21D = Eigen::Matrix<double, 21, 21>;

using V12D = Eigen::Matrix<double, 12, 1>;
using V21D = Eigen::Matrix<double, 21, 1>;
using M21X12D = Eigen::Matrix<double, 21, 12>;

M3D Jr(const V3D &inp);
M3D JrInv(const V3D &inp);

// Degeneracy detection result — DALI-SLAM inspired Jacobian remapping
struct DegeneracyInfo
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    bool detected = false;           // true if any DOF is degenerate
    double condition_number = 0.0;   // H condition number (high = bad)
    double min_eigenvalue = 0.0;     // smallest eigenvalue of H_pose (6x6)
    double max_eigenvalue = 0.0;     // largest eigenvalue
    double effective_ratio = 1.0;    // ratio of well-conditioned DOFs (0~1)
    int degenerate_dof_count = 0;    // number of degenerate DOFs (0~6)
    Eigen::Matrix<double, 6, 1> eigenvalues = Eigen::Matrix<double, 6, 1>::Zero();
    // Per-DOF degeneracy mask: 1.0 = well-constrained, 0.0 = degenerate
    Eigen::Matrix<double, 6, 1> dof_mask = Eigen::Matrix<double, 6, 1>::Ones();
};

struct SharedState
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    M12D H;
    V12D b;
    double res = 1e10;
    bool valid = false;
    size_t iter_num = 0;
    DegeneracyInfo degeneracy;
};
struct Input
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    V3D acc;
    V3D gyro;
    Input() = default;
    Input(V3D &a, V3D &g) : acc(a), gyro(g) {}
    Input(double a1, double a2, double a3, double g1, double g2, double g3) : acc(a1, a2, a3), gyro(g1, g2, g3) {}
};
struct State
{
    static double gravity;
    M3D r_wi = M3D::Identity();
    V3D t_wi = V3D::Zero();
    M3D r_il = M3D::Identity();
    V3D t_il = V3D::Zero();
    V3D v = V3D::Zero();
    V3D bg = V3D::Zero();
    V3D ba = V3D::Zero();
    V3D g = V3D(0.0, 0.0, -9.81);

    void initGravityDir(const V3D &gravity_dir) { g = gravity_dir.normalized() * State::gravity; }

    void operator+=(const V21D &delta);

    V21D operator-(const State &other) const;

    friend std::ostream &operator<<(std::ostream &os, const State &state);
};

using loss_func = std::function<void(State &, SharedState &)>;
using stop_func = std::function<bool(const V21D &)>;

class IESKF
{
public:
    IESKF() = default;
    void setMaxIter(size_t iter) { m_max_iter = iter; }
    void setLossFunction(loss_func func) { m_loss_func = func; }
    void setStopFunction(stop_func func) { m_stop_func = func; }

    void predict(const Input &inp, double dt, const M12D &Q);

    void update();

    // Clamp P diagonal to physical upper bounds (prevents unbounded covariance growth)
    void clampCovariance();

    // ZUPT: inject zero-velocity pseudo-observation to constrain drift during static periods
    void injectZUPT(double sigma_v = 0.02, double sigma_pos = 0.1);

    State &x() { return m_x; }

    M21D &P() { return m_P; }

    const DegeneracyInfo &degeneracy() const { return m_degeneracy; }

    // Per-dimension P diagonal upper bounds for 21-state IESKF:
    // [θ_wi(0:3), t_wi(3:6), θ_il(6:9), t_il(9:12), v(12:15), bg(15:18), ba(18:21)]
    static constexpr double P_MAX[21] = {
        9.87,  9.87,  9.87,   // θ_wi  (π² rad²)
        1e4,   1e4,   1e4,    // t_wi  (100 m std²)
        1e-4,  1e-4,  1e-4,   // θ_il  extrinsic rotation
        1e-2,  1e-2,  1e-2,   // t_il  extrinsic translation
        1e2,   1e2,   1e2,    // v     (10 m/s std²)
        1e-2,  1e-2,  1e-2,   // bg    gyro bias
        1.0,   1.0,   1.0,    // ba    acc bias
    };

private:
    size_t m_max_iter = 10;
    State m_x;
    M21D m_P;
    loss_func m_loss_func;
    stop_func m_stop_func;
    M21D m_F;
    M21X12D m_G;
    DegeneracyInfo m_degeneracy;
};
