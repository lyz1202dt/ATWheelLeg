#pragma once

// 10状态，WBC全身LQR建模控制方案

#include "controller.hpp"
#include "controllerbase.hpp"
#include "leg_calc.hpp"
#include "pid.hpp"
#include <Eigen/Dense>
#include <optional>
#include <tuple>

class Controller10 : public ControllerBase {
public:
    enum RobotState {
        IDEL,         // 位控处于默认站姿
        VMC_TEST,     // 测试VMC功能
        TOUCH_GROUND, // 接触地面，平衡控制
        LEFT_FLY,     // 左侧悬空
        RIGHT_FLAY,   // 右侧悬空
        ALL_FLY,      // 全部悬空
    };
    Controller10(IMUBase* imu, Motor* lf, Motor* rf, Motor* lb, Motor* rb, Motor* lw, Motor* rw);
    bool update(float dt) override;
    void input(float velocity, float omega, float height, int mode) override;

    RobotState state{IDEL}, exp_state{IDEL};
    LegCalc* leg_calc;

    Eigen::Matrix<double,4,10> K,K_air;


};
