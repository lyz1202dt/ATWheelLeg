#include "controller_at.hpp"

#include <cmath>

ControllerAT::ControllerAT(IMUBase* imu, Motor* lf, Motor* rf, Motor* lb, Motor* rb, Motor* lw, Motor* rw)
    : ControllerBase(imu, lf, rf, lb, rb, lw, rw) ,
    left_leg_length(10.0f,1.0f,0.0f,0.0f,10.0f,0.002f),
    right_leg_legth(10.0f,1.0f,0.0f,0.0f,10.0f,0.002f){
    leg_storage_ = std::make_unique<LegCalc2>(
        0.0945, 0.0945, 0.1125, 0.1125, 0.1155, 0.2502);
    leg_calc = leg_storage_.get();
}


bool ControllerAT::update(float dt) {
    (void)dt;
    if (leg_calc == nullptr || lf == nullptr || rf == nullptr ||
        lb == nullptr || rb == nullptr || lw == nullptr || rw == nullptr) {
        return false;
    }

    if (state == IDEL || state == VMC_TEST) {
        if (!send_idle_commands()) {
            return false;
        }
        if (exp_state == VMC_TEST) {
            state = VMC_TEST;
        } else {
            state = IDEL;
        }
        return true;
    }

    state = exp_state;
    return send_idle_commands();
}

void ControllerAT::input(float velocity, float omega, float height, int mode) {
    (void)velocity;
    (void)omega;
    (void)height;

    if (mode == 2) {
        exp_state = VMC_TEST;
    } else {
        exp_state = IDEL;
    }
}

bool ControllerAT::send_idle_commands() {
    const Eigen::Vector2d leg_target(0.2, 0.0);
    Eigen::Vector2d joint_target = Eigen::Vector2d::Zero();
    if (!leg_calc->inverse_kinematics(leg_target, joint_target) ||
        !joint_target.allFinite()) {
        return false;
    }

    const bool left_front_ok = lf->set_command(
        static_cast<float>(joint_target(0)), 0.0F, 0.0F,
        param.leg_kp, param.leg_kd);
    const bool right_front_ok = rf->set_command(
        static_cast<float>(joint_target(0)), 0.0F, 0.0F,
        param.leg_kp, param.leg_kd);
    const bool left_rear_ok = lb->set_command(
        static_cast<float>(joint_target(1)), 0.0F, 0.0F,
        param.leg_kp, param.leg_kd);
    const bool right_rear_ok = rb->set_command(
        static_cast<float>(joint_target(1)), 0.0F, 0.0F,
        param.leg_kp, param.leg_kd);
    const bool left_wheel_ok =
        lw->set_command(0.0F, 0.0F, 0.0F, 0.0F, 0.0F);
    const bool right_wheel_ok =
        rw->set_command(0.0F, 0.0F, 0.0F, 0.0F, 0.0F);
    return left_front_ok && right_front_ok && left_rear_ok &&
           right_rear_ok && left_wheel_ok && right_wheel_ok;
}
