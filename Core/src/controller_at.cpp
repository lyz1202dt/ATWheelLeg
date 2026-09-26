#include "controller_at.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <chrono>
#include <cmath>
#include <utility>

ControllerAT::ControllerAT(IMUBase* imu, Motor* lf, Motor* rf, Motor* lb, Motor* rb, Motor* lw, Motor* rw)
    : ControllerBase(imu, lf, rf, lb, rb, lw, rw)
    , left_leg_length(1000.0f, 30.0f, 0.0f, 0.0f, 200.0f, 0.002f)
    , right_leg_legth(1000.0f, 30.0f, 0.0f, 0.0f, 200.0f, 0.002f) {
    leg_calc_ = std::make_unique<LegCalc2>(0.0945, 0.0945, 0.1125, 0.1125, 0.1155, 0.2502);
}

void ControllerAT::register_debug_logger(DebugLogger logger) {
    debug_logger_ = std::move(logger);
}

bool ControllerAT::update(float dt) {
    (void)dt;
    if (state == IDEL) {
        lf->set_command(-0.785f, 0.0F, 0.0F, param.motor_kp, param.motor_kd);
        rf->set_command(-0.785f, 0.0F, 0.0F, param.motor_kp, param.motor_kd);
        lb->set_command(0.785f, 0.0F, 0.0F, param.motor_kp, param.motor_kd);
        rb->set_command(0.785f, 0.0F, 0.0F, param.motor_kp, param.motor_kd);
        lw->set_command(0.0F, 0.0F, 0.0F, 0.0F, 0.0F);
        rw->set_command(0.0F, 0.0F, 0.0F, 0.0F, 0.0F);
        if (exp_state == VMC_TEST) {
            state = VMC_TEST;
        }else if(exp_state==KINAMIC_TEST)
            state=KINAMIC_TEST;
        else if(exp_state==READY_STAND1)
            state=READY_STAND1;
    } else if (state == KINAMIC_TEST) {
        const Eigen::Vector2d leg_target(0.25, 0.0);
        Eigen::Vector2d joint_target = Eigen::Vector2d::Zero();
        if (!leg_calc_->inverse_kinematics(leg_target, joint_target) || !joint_target.allFinite()) {
            return false;
        }

        lf->set_command(static_cast<float>(joint_target[0]), 0.0F, 0.0F, param.motor_kp, param.motor_kd);
        rf->set_command(static_cast<float>(joint_target[0]), 0.0F, 0.0F, param.motor_kp, param.motor_kd);
        lb->set_command(static_cast<float>(joint_target[1]), 0.0F, 0.0F, param.motor_kp, param.motor_kd);
        rb->set_command(static_cast<float>(joint_target[1]), 0.0F, 0.0F, param.motor_kp, param.motor_kd);
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
    else if(state==READY_STAND1)    //到达准备站立状态1
    {
        Eigen::Vector2d reset_leg_pos(0.1,0.3);
        Eigen::Vector2d joint_pos;
        leg_calc_->inverse_kinematics(reset_leg_pos, joint_pos);
        if(!reset_traj_generated)
        {
            reset_traj_generated=true;
            left_leg_slope.generate_traj(Eigen::Vector2d(lf->state.rad,lb->state.rad), joint_pos, 5.0f);
            right_leg_slope.generate_traj(Eigen::Vector2d(rf->state.rad,rb->state.rad), joint_pos, 5.0f);
            time_point=std::chrono::high_resolution_clock::now();
        }
        auto duration=std::chrono::duration<double>(std::chrono::high_resolution_clock::now()-time_point).count();
        left_leg_slope.get_target(joint_pos, duration);
        lf->set_command(joint_pos[0],0.0f,0.0f,param.motor_kp,param.motor_kd);
        lb->set_command(joint_pos[1],0.0f,0.0f,param.motor_kp,param.motor_kd);
        bool ret=right_leg_slope.get_target(joint_pos, duration);
        rf->set_command(joint_pos[0],0.0f,0.0f,param.motor_kp,param.motor_kd);
        rb->set_command(joint_pos[1],0.0f,0.0f,param.motor_kp,param.motor_kd);
        if(!ret)    //复位完成，切到下一个状态
        {
            reset_traj_generated=false;
            exp_state=state=TOUCH_GROUND;
            if(debug_logger_)
                debug_logger_("TOUCH_GROUND");
        }
    }
    else if(state==TOUCH_GROUND)
    {
        lf->set_command(0.0f, 1.0f, 0.0f, 0.0f, 0.0f);
        rf->set_command(0.0f, 1.0f, 0.0f, 0.0f, 0.0f);
        lb->set_command(0.0f, 1.0f, 0.0f, 0.0f, 0.0f);
        rb->set_command(0.0f, 1.0f, 0.0f, 0.0f, 0.0f);
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
    }else if (mode == 4) {
        exp_state = READY_STAND1;
    }
}
