#pragma once

#include <Eigen/Dense>

#include <type_traits>

// 低通滤波器
class LowPassFilter {
public:
    explicit LowPassFilter(double alpha = 0.5):alpha(alpha){};
    double update(const double& raw){
        filtered_value_ = (1.0 - alpha) * filtered_value_ + alpha * raw;
        return filtered_value_;
    }
    void reset(const double& value = 0.0){
        filtered_value_ = value;
    }

private:
    double filtered_value_{0.0};
    double alpha{0.5};
};


namespace ekf {

// ---------------------------------------------------------------------------
// 模板化的 EKF：所有矩阵均为编译期固定尺寸，运行时零堆分配，适合裸机 MCU。
//
//
// 模型约定（以下成员必须存在，否则编译报错）：
//   MotionModel:
//     using Scalar;                        // 标量类型，如 double / float
//     static constexpr int kStateDim = N;  // 状态维度
//     static constexpr int kInputDim = U;  // 输入维度
//     bool predict (const State& x, const Input& u, Scalar dt, State&  out) const;
//     bool jacobian(const State& x, const Input& u, Scalar dt, Matrix& out) const;
//     bool Q       (const State& x, const Input& u, Scalar dt, Matrix& out) const;
//
//   MeasurementModel:
//     using Scalar;                        // 必须与 EKF 的 Scalar 一致
//     static constexpr int kDim      = M;  // 该传感器的测量维度
//     static constexpr int kStateDim = N;  // 必须与 EKF 的状态维度一致
//     bool h(const State& x, Measure& out) const;
//     bool H(const State& x, HMat&    out) const;
//     bool R(const State& x, RMat&    out) const;
// ---------------------------------------------------------------------------

template <class MotionModel, class Scalar = typename MotionModel::Scalar, int StateDim = MotionModel::kStateDim,int InputDim = MotionModel::kInputDim>
class EKF {
public:
    using State = Eigen::Matrix<Scalar, StateDim, 1>;
    using Input = Eigen::Matrix<Scalar, InputDim, 1>;
    using Cov   = Eigen::Matrix<Scalar, StateDim, StateDim>;

    static_assert(std::is_same_v<Scalar, typename MotionModel::Scalar>, "MotionModel::Scalar must match the EKF Scalar type");

    // motion_model 由调用方持有，且必须在 EKF 的生命周期内保持有效。
    explicit EKF(const MotionModel& motion_model)
        : motion_model_(&motion_model) {}

    // 设置初始状态和先验协方差。成功后才可调用 predict/correct。
    bool initialize(const State& initial_state, const Cov& initial_covariance) {
        state_       = initial_state;
        covariance_  = initial_covariance;
        initialized_ = true;
        return true;
    }

    // 清除当前滤波状态，使 EKF 回到未初始化状态。
    void reset() { initialized_ = false; }

    // 以运动模型频率执行预测。
    bool predict(const Input& input, Scalar dt) {
        if (!initialized_) {
            return false;
        }

        State x_pred;
        Cov F, Q;
        if (!motion_model_->predict(state_, input, dt, x_pred)) {
            return false;
        }
        if (!motion_model_->jacobian(state_, input, dt, F)) {
            return false;
        }
        if (!motion_model_->Q(state_, input, dt, Q)) {
            return false;
        }

        state_      = x_pred;
        covariance_ = F * covariance_ * F.transpose() + Q;
        // 对称化，抵消数值误差。
        covariance_ = Scalar(0.5) * (covariance_ + covariance_.transpose());
        return true;
    }

    // 使用一次到达的测量修正当前预测状态。
    //
    // 测量模型不由 EKF 所有，也没有固定注册表：调用方可以在各自的
    // 传感器数据到达时调用本函数，因此不同 MeasurementModel 可以使用
    // 不同采样频率，且每次 predict 可以没有任何测量修正。
    template <class MeasurementModel>
    bool correct(const MeasurementModel& measurement_model, const Eigen::Matrix<Scalar, MeasurementModel::kDim, 1>& measurement) {
        if (!initialized_) {
            return false;
        }

        static_assert(MeasurementModel::kStateDim == StateDim, "MeasurementModel::kStateDim must equal the EKF StateDim");
        static_assert(std::is_same_v<Scalar, typename MeasurementModel::Scalar>, "MeasurementModel::Scalar must match the EKF Scalar type");

        constexpr int M = MeasurementModel::kDim;

        using Measure = Eigen::Matrix<Scalar, M, 1>;
        using HMat    = Eigen::Matrix<Scalar, M, StateDim>;
        using RMat    = Eigen::Matrix<Scalar, M, M>;
        using SMat    = Eigen::Matrix<Scalar, M, M>;
        using KMat    = Eigen::Matrix<Scalar, StateDim, M>;
        using Joseph  = Eigen::Matrix<Scalar, StateDim, StateDim>;

        Measure h;
        HMat H;
        RMat R;
        if (!measurement_model.h(state_, h)) {
            return false;
        }
        if (!measurement_model.H(state_, H)) {
            return false;
        }
        if (!measurement_model.R(state_, R)) {
            return false;
        }

        const Measure y = measurement - h; // 新息
        const SMat S    = H * covariance_ * H.transpose() + R;

        // K = P * H^T * S^{-1}，用 ldlt().solve() 替代求逆，数值更稳。
        const KMat K = (S.ldlt().solve(H * covariance_)).transpose();

        // Joseph 形式：保证协方差对称正定。
        const Joseph IKH = Cov::Identity() - K * H;

        state_ += K * y;
        covariance_ = IKH * covariance_ * IKH.transpose() + K * R * K.transpose();
        return true;
    }

    const State& state() const { return state_; }
    const Cov& covariance() const { return covariance_; }

private:
    const MotionModel* motion_model_{nullptr};
    State state_{State::Zero()};
    Cov covariance_{Cov::Zero()};
    bool initialized_{false};
};


} // namespace ekf
