#pragma once

// 10状态，WBC全身LQR建模控制方案

#include "controllerbase.hpp"
#include "leg_calc.hpp"
#include "pid.hpp"
#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <functional>
#include <memory>


template <typename T>
class Slope{
public:
    Slope()=default;
    bool generate_traj(const T &start,const T &stop,float second){
        if (!is_finite(start) || !is_finite(stop) || !std::isfinite(second) || second < 0.0f) {
            generated = false;
            return false;
        }

        this->start = start;
        this->stop = stop;
        traj_second = second;
        generated = true;
        return true;
    }

    bool get_target(T &target,float second){

        if (traj_second < 0.0f || second >= traj_second) {
            target = stop;
            return false;
        }

        if (second < 0.0f) {
            target = start;
            return false;
        }

        const double ratio = static_cast<double>(second) / static_cast<double>(traj_second);
        target = start + (stop - start) * ratio;
        return true;
    }

    T start{};
    T stop{};
    float traj_second{0.0f};
    bool generated{false};

private:
    template <typename U>
    static auto is_finite_impl(const U &value, int) -> decltype(value.allFinite(), bool()){
        return value.allFinite();
    }

    template <typename U>
    static bool is_finite_impl(const U &value, long){
        return std::isfinite(static_cast<double>(value));
    }

    static bool is_finite(const T &value){
        return is_finite_impl(value, 0);
    }
};

class ControllerAT : public ControllerBase {
public:
    enum RobotState {
        IDEL,         // 位控处于默认站姿
        KINAMIC_TEST,   //运动学测试
        VMC_TEST,     // 测试VMC功能
        READY_STAND1,   //斜坡渐变当前期望到准备站立的姿态
        READY_STAND2,   //完成小板凳形态
        LQR_CTRL, // 接触地面，LQR平衡控制
    };
    struct Param{
        float motor_kp{30.0f};
        float motor_kd{1.0f};
    };
    using DebugLogger = std::function<void(const char*)>;

    ControllerAT(IMUBase* imu, Motor* lf, Motor* rf, Motor* lb, Motor* rb, Motor* lw, Motor* rw);
    bool update(float dt) override;
    void input(float velocity, float omega, float height, int mode) override;
    void register_debug_logger(DebugLogger logger);

    template <typename... Args>
    void debug_log(const char* format, Args... args) const {
        if (!debug_logger_ || format == nullptr) {
            return;
        }

        char buffer[256]{};
        const int written = std::snprintf(buffer, sizeof(buffer), format, args...);
        if (written < 0) {
            return;
        }
        buffer[sizeof(buffer) - 1U] = '\0';
        if(debug_logger_)
            debug_logger_(buffer);
    }

    std::unique_ptr<LegCalcBase> leg_calc_;
    RobotState state{KINAMIC_TEST}, exp_state{KINAMIC_TEST};
    Param param;
    PID left_leg_length,right_leg_length;
    Eigen::Matrix<double,4,10> K,K_left_air,K_right_air,K_air;
    bool update_lqr_k(const Eigen::Vector<float,10> &Q,const Eigen::Vector<float,4> &R);
    double ref_pos{0.0},ref_vel{0.0},ref_omega{0.0};
private:
    static double wrap_to_pi(double angle);
    static bool extract_ypr(const Eigen::Quaternionf& orientation,
                            double& yaw,
                            double& pitch,
                            double& roll);
    static double low_pass_filter(double input,
                                  double alpha,
                                  double& filtered_value,
                                  bool& initialized);

    DebugLogger debug_logger_;
    Slope<Eigen::Vector2d> left_leg_slope,right_leg_slope;
    bool reset_traj_generated{false};
    std::chrono::time_point<std::chrono::high_resolution_clock> time_point;

    double filtered_ds_{0.0};
    double filtered_dphi_{0.0};
    double filtered_dthll_{0.0};
    double filtered_dthlr_{0.0};
    double filtered_dthb_{0.0};
    bool ds_filter_initialized_{false};
    bool dphi_filter_initialized_{false};
    bool dthll_filter_initialized_{false};
    bool dthlr_filter_initialized_{false};
    bool dthb_filter_initialized_{false};

    bool yaw_tracking_initialized_{false};
    double yaw_previous_{0.0};
    double yaw_unwrapped_{0.0};
    double yaw_reference_{0.0};
};
