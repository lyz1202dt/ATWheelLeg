#include "tools/estimater.hpp"

#include <cmath>
#include <iostream>

namespace {

// 一维匀速直线运动模型：状态 [位置, 速度]，输入 [加速度]。
struct LinearMotion {
    using Scalar = double;
    static constexpr int kStateDim = 2;
    static constexpr int kInputDim = 1;

    using State = Eigen::Matrix<Scalar, kStateDim, 1>;
    using Input = Eigen::Matrix<Scalar, kInputDim, 1>;
    using Matrix = Eigen::Matrix<Scalar, kStateDim, kStateDim>;

    bool predict(const State& x, const Input& u, Scalar dt, State& out) const {
        out(0) = x(0) + dt * x(1) + Scalar(0.5) * dt * dt * u(0);
        out(1) = x(1) + dt * u(0);
        return true;
    }

    bool jacobian(const State& /*x*/, const Input& /*u*/, Scalar dt, Matrix& F) const {
        F << Scalar(1), dt,
             Scalar(0), Scalar(1);
        return true;
    }

    bool Q(const State& /*x*/, const Input& /*u*/, Scalar /*dt*/, Matrix& Q) const {
        Q << Scalar(0.01), Scalar(0),
             Scalar(0), Scalar(0.01);
        return true;
    }
};

// 只观测位置的传感器，测量维度 M = 1。
struct PositionMeasurement {
    using Scalar = double;
    static constexpr int kDim = 1;
    static constexpr int kStateDim = 2;

    using State = Eigen::Matrix<Scalar, kStateDim, 1>;
    using Measure = Eigen::Matrix<Scalar, kDim, 1>;
    using HMat = Eigen::Matrix<Scalar, kDim, kStateDim>;
    using RMat = Eigen::Matrix<Scalar, kDim, kDim>;

    bool h(const State& x, Measure& out) const {
        out(0) = x(0);
        return true;
    }

    bool H(const State& /*x*/, HMat& out) const {
        out << Scalar(1), Scalar(0);
        return true;
    }

    bool R(const State& /*x*/, RMat& out) const {
        out << Scalar(0.05);
        return true;
    }
};

// 同时观测位置与速度的传感器，测量维度 M = 2，用于验证不同测量维度共用同一个 EKF。
struct PositionVelocityMeasurement {
    using Scalar = double;
    static constexpr int kDim = 2;
    static constexpr int kStateDim = 2;

    using State = Eigen::Matrix<Scalar, kStateDim, 1>;
    using Measure = Eigen::Matrix<Scalar, kDim, 1>;
    using HMat = Eigen::Matrix<Scalar, kDim, kStateDim>;
    using RMat = Eigen::Matrix<Scalar, kDim, kDim>;

    bool h(const State& x, Measure& out) const {
        out = x;
        return true;
    }

    bool H(const State& /*x*/, HMat& out) const {
        out << Scalar(1), Scalar(0),
               Scalar(0), Scalar(1);
        return true;
    }

    bool R(const State& /*x*/, RMat& out) const {
        out << Scalar(0.05), Scalar(0),
               Scalar(0), Scalar(0.05);
        return true;
    }
};

}  // namespace

int main()
{
    using Scalar = double;

    LinearMotion motion;
    PositionMeasurement pos_meas;
    PositionVelocityMeasurement pv_meas;

    ekf::EKF<LinearMotion> ekf(motion);

    Eigen::Matrix<Scalar, 1, 1> u;
    u(0) = Scalar(0);

    // 未初始化时 predict 应失败。
    if (ekf.predict(u, Scalar(0.01))) {
        std::cerr << "predict before initialize should fail\n";
        return 1;
    }

    ekf::EKF<LinearMotion>::State x0 = ekf::EKF<LinearMotion>::State::Zero();
    ekf::EKF<LinearMotion>::Cov P0 = ekf::EKF<LinearMotion>::Cov::Identity();
    if (!ekf.initialize(x0, P0)) {
        std::cerr << "initialize failed\n";
        return 1;
    }

    const Scalar dt = Scalar(0.01);
    const Scalar true_vel = Scalar(1.0);
    Scalar true_pos = Scalar(0.0);

    // 匀速运动，加速度为 0，仅观测位置，跑 4 秒让滤波器收敛。
    const int steps = 400;
    for (int k = 0; k < steps; ++k) {
        true_pos += true_vel * dt;
        u(0) = Scalar(0);

        if (!ekf.predict(u, dt)) {
            std::cerr << "predict failed\n";
            return 1;
        }

        Eigen::Matrix<Scalar, 1, 1> z;
        z(0) = true_pos;
        if (!ekf.correct(pos_meas, z)) {
            std::cerr << "correct failed\n";
            return 1;
        }
    }

    const auto& x = ekf.state();
    std::cout << "estimated state = [" << x(0) << ", " << x(1) << "]\n";
    std::cout << "true state      = [" << true_pos << ", " << true_vel << "]\n";

    if (std::abs(x(0) - true_pos) > Scalar(1e-2)) {
        std::cerr << "position estimate off\n";
        return 1;
    }
    if (std::abs(x(1) - true_vel) > Scalar(1e-1)) {
        std::cerr << "velocity estimate off\n";
        return 1;
    }

    // 协方差应保持对称。
    const auto& P = ekf.covariance();
    if ((P - P.transpose()).cwiseAbs().maxCoeff() > Scalar(1e-12)) {
        std::cerr << "covariance not symmetric\n";
        return 1;
    }

    // 不同测量维度（M=2）也能用同一个 EKF 修正。
    Eigen::Matrix<Scalar, 2, 1> z2;
    z2(0) = true_pos;
    z2(1) = true_vel;
    if (!ekf.correct(pv_meas, z2)) {
        std::cerr << "correct (M=2) failed\n";
        return 1;
    }

    std::cout << "EKF checks passed\n";
    return 0;
}
