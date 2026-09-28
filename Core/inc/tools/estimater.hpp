#pragma once

#include <Eigen/Dense>

//低通滤波器
class LowPassFilter {
public:
    explicit LowPassFilter(double alpha = 0.5);
    double update(const double &raw, double filter_alpha);
    void reset(const double &value=0.0);
    double alpha{0.5};
private:
    double filtered_value_{0.0};
    bool initialized_{false};
};


namespace ekf {

class MeasurementModel {
public:
    virtual ~MeasurementModel() = default;

    // 传感器理论测量值 h(x)。
    virtual bool h(const Eigen::VectorXd& x,
                   Eigen::VectorXd& result) const = 0;

    // 测量函数对状态的雅可比 H = dh(x) / dx。
    virtual bool H(const Eigen::VectorXd& x,
                   Eigen::MatrixXd& result) const = 0;

    // 测量噪声协方差 R。允许噪声随当前状态变化。
    virtual bool R(const Eigen::VectorXd& x,
                   Eigen::MatrixXd& result) const = 0;
};

class MotionModel {
public:
    virtual ~MotionModel() = default;

    // 离散时间状态转移 x(k + 1) = f(x(k), u(k), dt)。
    virtual bool predict(
        const Eigen::VectorXd& x,
        const Eigen::VectorXd& u,
        double dt,
        Eigen::VectorXd& result) const = 0;

    // 离散状态转移函数对状态的雅可比 F = df / dx。
    virtual bool jacobian(
        const Eigen::VectorXd& x,
        const Eigen::VectorXd& u,
        double dt,
        Eigen::MatrixXd& result) const = 0;

    // 一次 predict 对应的离散过程噪声协方差 Q。
    virtual bool Q(
        const Eigen::VectorXd& x,
        const Eigen::VectorXd& u,
        double dt,
        Eigen::MatrixXd& result) const = 0;
};

class EKF {
public:
    // motion_model 由调用方持有，且必须在 EKF 的生命周期内保持有效。
    explicit EKF(const MotionModel& motion_model);
    ~EKF() = default;

    // 设置初始状态和先验协方差。成功后才可调用 predict/correct。
    bool initialize(const Eigen::VectorXd& initial_state,
                    const Eigen::MatrixXd& initial_covariance);

    // 清除当前滤波状态，使 EKF 回到未初始化状态。
    void reset();

    // 以运动模型频率执行预测。
    bool predict(const Eigen::VectorXd& input, double dt);

    // 使用一次到达的测量修正当前预测状态。
    //
    // 测量模型不由 EKF 所有，也没有固定注册表：调用方可以在各自的
    // 传感器数据到达时调用本函数，因此不同 MeasurementModel 可以使用
    // 不同采样频率，且每次 predict 可以没有任何测量修正。
    bool correct(const MeasurementModel& measurement_model,
                 const Eigen::VectorXd& measurement);

    bool initialized() const;
    const Eigen::VectorXd& state() const;
    const Eigen::MatrixXd& covariance() const;

private:
    const MotionModel* motion_model_{nullptr};
    Eigen::VectorXd state_;
    Eigen::MatrixXd covariance_;
    bool initialized_{false};
};


}  // namespace ekf
