#pragma once

// 10状态，WBC全身LQR建模控制方案

#include "controllerbase.hpp"
#include "tools/estimater.hpp"
#include "tools/leg_calc.hpp"
#include "tools/pid.hpp"
#include "tools/slope.hpp"
#include "tools/estimater.hpp"
#include <Eigen/Dense>
#include <tinympc/types.hpp>
#include <array>
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
    using ABSchedulerFunc = std::function<bool(
        const double& left_leg_length,
        const double& right_leg_length,
        Eigen::Matrix<double, 10, 10>& A,
        Eigen::Matrix<double, 10, 4>& B)>;
    enum RobotState {
        IDEL,         // 位控处于默认站姿
        KINAMIC_TEST,   //运动学测试
        VMC_TEST,     // 测试VMC功能
        READY_STAND1,   //斜坡渐变当前期望到准备站立的姿态
        READY_STAND2,   //完成小板凳形态
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
                 ABSchedulerFunc ab_func,
                 Param *p_param=nullptr,
                 DebugLogger debug_logger = nullptr);
    ~ControllerAT() override;

    bool update(uint64_t ms) override;
    void input(float velocity, float omega, float height, int mode) override;

    std::unique_ptr<LegCalcBase> leg_calc_;
    GainSchedulerFunc gain_scheduler_func;
    ABSchedulerFunc ab_scheduler_func;
    RobotState exp_state{KINAMIC_TEST};
    Param param;
    Eigen::Matrix<double,4,10> K,K_left_air,K_right_air,K_air;
private:
    static double wrap_to_pi(double angle);
    static bool extract_ypr(const Eigen::Quaternionf& orientation,
                            double& yaw,
                            double& pitch,
                            double& roll);
    bool initialize_tiny_mpc();
    bool solve_two_wheel_mpc(const Eigen::Vector<double, 10>& x,
                             const Eigen::Vector<double, 10>& xd,
                             double left_leg_length,
                             double right_leg_length,
                             Eigen::Vector4d& control);

    Slope<Eigen::Vector2d> left_leg_slope,right_leg_slope;
    bool reset_traj_generated{false};
    uint64_t time_point{0U};

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

    static constexpr int kMpcStateSize = 10;
    static constexpr int kMpcInputSize = 4;
    static constexpr int kMpcHorizon = 10;
    static constexpr double kMpcDt = 0.002;
    static constexpr double kMpcRho = 1.0;
    static constexpr int kMpcGridSize = 7;
    static constexpr double kMpcWheelTorqueLimit = 6.0;
    static constexpr double kMpcLegTorqueLimit = 30.0;
    struct MpcModel;
    TinySolver* tiny_mpc_solver_{nullptr};
    bool tiny_mpc_initialized_{false};
    std::unique_ptr<std::array<MpcModel, kMpcGridSize * kMpcGridSize>> mpc_models_;
    int mpc_model_index_{-1};
    Eigen::Matrix<double, kMpcStateSize, 1> mpc_q_diag_{
        (Eigen::Matrix<double, kMpcStateSize, 1>()
             << 20.0, 20.0, 25.0, 20.0, 10.0, 10.0, 10.0, 10.0, 2000.0, 80.0)
            .finished()};
    Eigen::Matrix<double, kMpcInputSize, 1> mpc_r_diag_{
        (Eigen::Matrix<double, kMpcInputSize, 1>() << 4.0, 4.0, 1.0, 1.0).finished()};
};
