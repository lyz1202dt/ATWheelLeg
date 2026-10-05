#pragma once

// 10状态，WBC全身LQR建模控制方案

#include "controllerbase.hpp"
#include "tools/estimater.hpp"
#include "tools/leg_calc.hpp"
#include "tools/leso.hpp"
#include "tools/pid.hpp"
#include "tools/slope.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <functional>
#include <memory>



class ControllerAT : public ControllerBase {
public:

    static constexpr double Rw=0.058;
    static constexpr double Mw=0.47;
    static constexpr double Mb=10.51;
    static constexpr double BodyWidth=0.34;
    static constexpr double BaseLinkComHeight=-0.0417729612931393;
    static constexpr double CentrifugalForceFfGain=1.0;
    static constexpr double CentrifugalForceFfLimit=15.0;

    using GainSchedulerFunc = std::function<bool(
        const double& left_leg_length,
        const double& right_leg_length,
        Eigen::Matrix<double, 4, 10>& K)>;
    enum RobotState {
        IDEL,         // 位控处于默认站姿
        KINAMIC_TEST,   //运动学测试
        VMC_TEST,     // 测试VMC功能
        READY_STAND,   //准备站立姿态
        LQR_CTRL, // 接触地面，LQR平衡控制
        LQR_STEP, // LQR上台阶
        LQR_JUMP  //LQR，跳跃
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
    using LqrState = Eigen::Matrix<double, 10, 1>;
    using LqrInput = Eigen::Matrix<double, 4, 1>;
    using LqrDisturbanceObserver = LESO<double, 10, 4, 10, 4>;

    static double wrap_to_pi(double angle);
    static bool extract_ypr(const Eigen::Quaternionf& orientation,
                            double& yaw,
                            double& pitch,
                            double& roll);

    Slope<Eigen::Vector2d> left_leg_slope,right_leg_slope;
    bool reset_traj_generated{false};
    uint64_t time_point{0U};

    kf::KalmanFilter<10, 4, 11> body_kf;
    kf::KalmanFilter<2, 1, 1> wheel_kf;
    LowPassFilter ds_filter_;
    LowPassFilter dphi_filter_;
    LowPassFilter dthll_filter_;
    LowPassFilter dthlr_filter_;
    LowPassFilter dthb_filter_;
    LowPassFilter roll_rate_filter_;
    LowPassFilter left_leg_force_filter_;
    LowPassFilter right_leg_force_filter_;
    PID left_leg_length;
    PID right_leg_length;
    PID roll_pd;
    RobotState state{KINAMIC_TEST};
    double ref_pos{0.0},ref_vel{0.0},ref_phi{0.0f},ref_omega{0.0},ref_height{0.25f};

    bool yaw_tracking_initialized_{false};
    double yaw_previous_{0.0};
    double yaw_unwrapped_{0.0};
    double yaw_reference_{0.0};

    Eigen::Vector4d u{Eigen::Vector4d::Zero()};
    std::unique_ptr<LqrDisturbanceObserver> lqr_disturbance_observer_;
    Eigen::Vector4d lqr_disturbance_{Eigen::Vector4d::Zero()};
    Eigen::Vector4d previous_lqr_command_{Eigen::Vector4d::Zero()};
    bool lqr_disturbance_observer_initialized_{false};
    bool body_kf_initialized_{false};

    int sub_stage{0};
    uint64_t stage_time_tick{0};

    Eigen::Vector2d step_leg_exp_pos;
    bool finished_step{false};
    Slope<Eigen::Vector2d> step_traj_slop;
    double final_leg_omega{0.0};
};
