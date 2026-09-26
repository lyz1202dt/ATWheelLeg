#include "controller_at.hpp"

#include <cmath>

ControllerAT::ControllerAT(IMUBase* imu, Motor* lf, Motor* rf, Motor* lb, Motor* rb, Motor* lw, Motor* rw)
    : ControllerBase(imu, lf, rf, lb, rb, lw, rw)
    , left_leg_length(600.0f, 20.0f, 0.0f, 0.0f, 200.0f, 0.002f)
    , right_leg_legth(600.0f, 20.0f, 0.0f, 0.0f, 200.0f, 0.002f) {
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
        const Eigen::Vector2d leg_target(0.25, 0.0);
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
        Eigen::Vector2d left_leg_pos,right_leg_pos,left_leg_vel,right_leg_vel,left_leg_force,right_leg_force;
        auto left_joint_pos=Eigen::Vector2d(lf->state.rad,lb->state.rad);
        auto right_joint_pos=Eigen::Vector2d(rf->state.rad,rb->state.rad);

        leg_calc_->forward_kinematics(left_joint_pos, left_leg_pos);
        leg_calc_->forward_velocity(left_joint_pos, Eigen::Vector2d(lf->state.vel,lb->state.vel), left_leg_vel);
        leg_calc_->forward_dynamics(left_joint_pos, Eigen::Vector2d(lf->state.toqeue,lb->state.toqeue), left_leg_force);

        leg_calc_->forward_kinematics(right_joint_pos, right_leg_pos);
        leg_calc_->forward_velocity(right_joint_pos, Eigen::Vector2d(rf->state.vel,rb->state.vel), right_leg_vel);
        leg_calc_->forward_dynamics(right_joint_pos, Eigen::Vector2d(rf->state.toqeue,rb->state.toqeue), right_leg_force);
        
        Eigen::Vector2d left_leg_exp_force,right_leg_exp_force,left_joint_exp_torque,right_joint_exp_torque;
        left_leg_exp_force[0]=left_leg_length.update(left_leg_pos[0],left_leg_vel[0],0.23f);
        right_leg_exp_force[0]=left_leg_length.update(right_leg_pos[0],right_leg_vel[0],0.23f);
        left_leg_exp_force[1]=right_leg_exp_force[1]=0.0f;

        leg_calc_->inverse_dynamics(left_joint_pos, left_leg_exp_force, left_joint_exp_torque);
        leg_calc_->inverse_dynamics(right_joint_pos, right_leg_exp_force, right_joint_exp_torque);

        lf->set_command(0.0f, 0.0f, left_joint_exp_torque[0], 0.0f, 0.0f);
        rf->set_command(0.0f, 0.0f, right_joint_exp_torque[0], 0.0f, 0.0f);
        lb->set_command(0.0f, 0.0f, left_joint_exp_torque[1], 0.0f, 0.0f);
        rb->set_command(0.0f, 0.0f, right_joint_exp_torque[1], 0.0f, 0.0f);
        
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
