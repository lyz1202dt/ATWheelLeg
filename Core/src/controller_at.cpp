#include "controller_at.hpp"

#include <cmath>

ControllerAT::ControllerAT(IMUBase* imu, Motor* lf, Motor* rf, Motor* lb, Motor* rb, Motor* lw, Motor* rw)
    : ControllerBase(imu, lf, rf, lb, rb, lw, rw)
    , left_leg_length(10.0f, 1.0f, 0.0f, 0.0f, 10.0f, 0.002f)
    , right_leg_legth(10.0f, 1.0f, 0.0f, 0.0f, 10.0f, 0.002f) {
    leg_calc_ = std::make_unique<LegCalc2>(0.0945, 0.0945, 0.1125, 0.1125, 0.1155, 0.2502);
}


bool ControllerAT::update(float dt) {
    (void)dt;
    if (state == IDEL) {
        lf->set_command(-0.785f, 0.0F, 0.0F, param.leg_kp, param.leg_kd);
        rf->set_command(-0.785f, 0.0F, 0.0F, param.leg_kp, param.leg_kd);
        lb->set_command(0.785f, 0.0F, 0.0F, param.leg_kp, param.leg_kd);
        rb->set_command(0.785f, 0.0F, 0.0F, param.leg_kp, param.leg_kd);
        lw->set_command(0.0F, 0.0F, 0.0F, 0.0F, 0.0F);
        rw->set_command(0.0F, 0.0F, 0.0F, 0.0F, 0.0F);
        if (exp_state == VMC_TEST) {
            state = VMC_TEST;
        }else if(exp_state==KINAMIC_TEST)
            state=KINAMIC_TEST;
    } else if (state == KINAMIC_TEST) {
        const Eigen::Vector2d leg_target(0.2, 0.0);
        Eigen::Vector2d joint_target = Eigen::Vector2d::Zero();
        if (!leg_calc_->inverse_kinematics(leg_target, joint_target) || !joint_target.allFinite()) {
            return false;
        }

        lf->set_command(static_cast<float>(joint_target[0]), 0.0F, 0.0F, param.leg_kp, param.leg_kd);
        rf->set_command(static_cast<float>(joint_target[0]), 0.0F, 0.0F, param.leg_kp, param.leg_kd);
        lb->set_command(static_cast<float>(joint_target[1]), 0.0F, 0.0F, param.leg_kp, param.leg_kd);
        rb->set_command(static_cast<float>(joint_target[1]), 0.0F, 0.0F, param.leg_kp, param.leg_kd);
        lw->set_command(0.0F, 0.0F, 0.0F, 0.0F, 0.0F);
        rw->set_command(0.0F, 0.0F, 0.0F, 0.0F, 0.0F);
        if (exp_state == IDEL) {
            state = IDEL;
        }else if(exp_state==VMC_TEST)
            state=VMC_TEST;
    }
    else if(state==VMC_TEST)
    {

        if (exp_state == IDEL) {
            state = IDEL;
        }else if(exp_state==KINAMIC_TEST)
            state=KINAMIC_TEST;
    }
    return true;
}

void ControllerAT::input(float velocity, float omega, float height, int mode) {
    (void)velocity;
    (void)omega;
    (void)height;

    if (mode == 1) {
        exp_state = IDEL;
    } else if (mode == 2) {
        exp_state = KINAMIC_TEST;
    }else if (mode == 3) {
        exp_state = VMC_TEST;
    }
}
