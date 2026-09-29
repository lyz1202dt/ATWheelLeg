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


namespace kf {
template<int NX, int NU, int NY>
class KalmanFilter
{
public:
    // 类型别名
    using StateVec    = Eigen::Matrix<double, NX, 1>;       // 状态向量 x
    using ControlVec  = Eigen::Matrix<double, NU, 1>;       // 控制输入 u
    using MeasVec     = Eigen::Matrix<double, NY, 1>;       // 观测向量 y
    using StateMat    = Eigen::Matrix<double, NX, NX>;      // 状态转移矩阵 A
    using ControlMat  = Eigen::Matrix<double, NX, NU>;      // 控制矩阵 B
    using MeasMat     = Eigen::Matrix<double, NY, NX>;      // 观测矩阵 H
    using ProcessNoiseMat = Eigen::Matrix<double, NX, NX>;  // 过程噪声协方差 Q
    using MeasNoiseMat    = Eigen::Matrix<double, NY, NY>;  // 观测噪声协方差 R
    using GainMat         = Eigen::Matrix<double, NX, NY>;  // 卡尔曼增益 K
    using CovMat          = Eigen::Matrix<double, NX, NX>;  // 状态协方差 P

public:
    // ============ 公共成员变量：系统矩阵 ============
    StateMat   A;    // 状态转移矩阵
    ControlMat B;    // 控制输入矩阵
    MeasMat    H;    // 观测矩阵

    // ============ 公共成员变量：噪声协方差 ============
    ProcessNoiseMat Q;   // 过程噪声协方差
    MeasNoiseMat    R;   // 观测噪声协方差

public:
    KalmanFilter(const StateMat &A,const ControlMat &B,const MeasMat &C,const ProcessNoiseMat &Q,const MeasNoiseMat &R)
        : A(A)
        , B(B)
        , H(C)
        , Q(Q)
        , R(R)
        , x_(StateVec::Zero())
        , P_(CovMat::Identity())
        , I_(StateMat::Identity())
    {}

    /**
     * @brief 重置滤波器状态和协方差
     * @param x 初始状态
     * @param p 初始协方差
     */
    void reset(const StateVec& x, const CovMat& p)
    {
        x_ = x;
        P_ = p;
    }

    /**
     * @brief 重置滤波器（仅状态）
     */
    void reset(const StateVec& x)
    {
        x_ = x;
    }

    /**
     * @brief 重置滤波器（仅协方差）
     */
    void reset(const CovMat& p)
    {
        P_ = p;
    }

    /**
     * @brief 卡尔曼滤波更新（预测 + 校正）
     * @param y   观测值
     * @param u   控制输入
     * @param ey  输出：观测残差 y - H*x（先验残差）
     */
    void update(const MeasVec& y, const ControlVec& u)
    {
        // ---------- 1. 预测（Time Update） ----------
        // 状态预测: x = A*x + B*u
        x_ = A * x_ + B * u;

        // 协方差预测: P = A*P*A' + Q
        P_ = A * P_ * A.transpose() + Q;

        // ---------- 2. 校正（Measurement Update） ----------
        // 观测残差: ey = y - H*x
        MeasVec ey_ = y - H * x_;

        // 残差协方差: S = H*P*H' + R
        Eigen::Matrix<double, NY, NY> S = H * P_ * H.transpose() + R;

        // 卡尔曼增益: K = P*H'*S^{-1}
        // 使用 solve 而非显式求逆，数值更稳定
        GainMat K = P_ * H.transpose() * S.inverse();

        // 状态更新: x = x + K*ey
        x_ = x_ + K * ey_;

        // 协方差更新: P = (I - K*H)*P
        //P_ = (I_ - K * H) * P_;

        // 若需要数值稳定性，可改用 Joseph 形式：
        P_ = (I_ - K*H) * P_ * (I_ - K*H).transpose() + K * R * K.transpose();
    }

    /**
     * @brief 仅预测（无观测时使用）
     */
    void predict(const ControlVec& u)
    {
        x_ = A * x_ + B * u;
        P_ = A * P_ * A.transpose() + Q;
    }

    // ============ 访问器 ============
    const StateVec& state() const { return x_; }
    const CovMat&   covariance() const { return P_; }

private:
    StateVec x_;   // 状态估计
    CovMat   P_;   // 状态协方差
    StateMat I_;   // 单位矩阵
};

}