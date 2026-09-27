#pragma once

// 10状态，WBC全身LQR建模控制方案

#include "controllerbase.hpp"
#include "leg_calc.hpp"
#include "pid.hpp"
#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <cmath>
#include <functional>
#include <memory>
#include <type_traits>


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


class ControllerAT : public ControllerBase {
public:
    using GainSchedulerFunc = std::function<bool(
        const double& left_leg_length,
        const double& right_leg_length,
        Eigen::Matrix<double, 4, 10>& K)>;
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

    ControllerAT(IMUBase* imu,
                 Motor* lf,
                 Motor* rf,
                 Motor* lb,
                 Motor* rb,
                 Motor* lw,
                 Motor* rw,
                 GainSchedulerFunc func,
                 Param *p_param=nullptr,
                 DebugLogger debug_logger = nullptr);
    bool update(uint64_t ms) override;
    void input(float velocity, float omega, float height, int mode) override;

    std::unique_ptr<LegCalcBase> leg_calc_;
    GainSchedulerFunc gain_scheduler_func;
    RobotState exp_state{KINAMIC_TEST};
    Param param;
    Eigen::Matrix<double,4,10> K,K_left_air,K_right_air,K_air;
private:
    static double wrap_to_pi(double angle);
    static bool extract_ypr(const Eigen::Quaternionf& orientation,
                            double& yaw,
                            double& pitch,
                            double& roll);

    Slope<Eigen::Vector2d> left_leg_slope,right_leg_slope;
    bool reset_traj_generated{false};
    uint64_t time_point{0U};

    LowPassFilter ds_filter_;
    LowPassFilter dphi_filter_;
    LowPassFilter dthll_filter_;
    LowPassFilter dthlr_filter_;
    LowPassFilter dthb_filter_;
    PID left_leg_length;
    PID right_leg_length;
    RobotState state{KINAMIC_TEST};
    double ref_pos{0.0},ref_vel{0.0},ref_omega{0.0};

    bool yaw_tracking_initialized_{false};
    double yaw_previous_{0.0};
    double yaw_unwrapped_{0.0};
    double yaw_reference_{0.0};
};
