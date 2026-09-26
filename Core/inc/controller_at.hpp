#pragma once

// 10状态，WBC全身LQR建模控制方案

#include "controllerbase.hpp"
#include "leg_calc.hpp"
#include "pid.hpp"
#include <Eigen/Dense>
#include <memory>

class ControllerAT : public ControllerBase {
public:
    enum RobotState {
        IDEL,         // 位控处于默认站姿
        VMC_TEST,     // 测试VMC功能
        READY_STAND1,   //斜坡渐变当前期望到准备站立的姿态
        READY_STAND2,   //完成小板凳形态
        TOUCH_GROUND, // 接触地面，LQR平衡控制
        LEFT_FLY,     // 左侧悬空，LQR左侧失控
        RIGHT_FLAY,   // 右侧悬空，LQR右侧失控
        ALL_FLY,      // 全部悬空，LQR仅姿态稳定
    };
    struct Param{
        float leg_kp{10.0f};
        float leg_kd{1.0f};
    };
    ControllerAT(IMUBase* imu, Motor* lf, Motor* rf, Motor* lb, Motor* rb, Motor* lw, Motor* rw);
    bool update(float dt) override;
    void input(float velocity, float omega, float height, int mode) override;

    RobotState state{IDEL}, exp_state{IDEL};
    LegCalcBase* leg_calc{nullptr};
    Param param;
    PID left_leg_length,right_leg_legth;
    Eigen::Matrix<double,4,10> K,K_air;

private:
    bool send_idle_commands();
    std::unique_ptr<LegCalcBase> leg_storage_;
};
