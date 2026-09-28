#include "controller/controller_at.hpp"
#include <algorithm>
#include <Eigen/src/Core/Matrix.h>
#include <cmath>
#include <utility>

ControllerAT::ControllerAT(IMUBase* imu,
                           Motor* lf,
                           Motor* rf,
                           Motor* lb,
                           Motor* rb,
                           Motor* lw,
                           Motor* rw,
                           GainSchedulerFunc func,
                           Param *p_param,
                           DebugLogger debug_logger)
    : ControllerBase(imu, lf, rf, lb, rb, lw, rw, std::move(debug_logger))
    , gain_scheduler_func(std::move(func))
    , left_leg_length(1000.0f, 15.0f, 0.0f, 0.0f, 200.0f, 0.002f)
    , right_leg_length(1000.0f, 15.0f, 0.0f, 0.0f, 200.0f, 0.002f) {
    if (p_param != nullptr) {
        param = *p_param;
    }
    leg_calc_ = std::make_unique<OffsetParallelCalc>(0.0945, 0.0945, 0.1125, 0.1125, 0.1155, 0.2502);
}

double ControllerAT::wrap_to_pi(double angle) {
    constexpr double kPi = 3.14159265358979323846;
    constexpr double kTwoPi = 2.0 * kPi;
    if (!std::isfinite(angle)) {
        return 0.0;
    }

    angle = std::fmod(angle + kPi, kTwoPi);
    if (angle < 0.0) {
        angle += kTwoPi;
    }
    return angle - kPi;
}

bool ControllerAT::extract_ypr(const Eigen::Quaternionf& orientation,
                               double& yaw,
                               double& pitch,
                               double& roll) {
    const Eigen::Matrix3d rotation = orientation.toRotationMatrix().cast<double>();
    yaw = std::atan2(rotation(1, 0), rotation(0, 0));
    pitch = std::atan2(
        -rotation(2, 0),
        std::hypot(rotation(0, 0), rotation(1, 0)));
    roll = std::atan2(rotation(2, 1), rotation(2, 2));

    return true;
}

bool ControllerAT::update(uint64_t ms) {
    if (state != LQR_CTRL) {
        yaw_tracking_initialized_ = false;
    }

    const Eigen::Vector2d left_joint_position(lf->state.rad, lb->state.rad);
    const Eigen::Vector2d right_joint_position(rf->state.rad, rb->state.rad);
    Eigen::Vector2d left_leg_position;
    Eigen::Vector2d right_leg_position;
    if (!leg_calc_->forward_kinematics(left_joint_position, left_leg_position) ||
        !leg_calc_->forward_kinematics(right_joint_position, right_leg_position) ||
        !left_leg_position.allFinite() || !right_leg_position.allFinite() ||
        gain_scheduler_func == nullptr ||
        !gain_scheduler_func(left_leg_position[0], right_leg_position[0], K) ||
        !K.allFinite()) {
        return false;
    }

    K_air.setZero();
    K_air(2, 4) = K(2, 4);
    K_air(2, 5) = K(2, 5);
    K_air(3, 6) = K(3, 6);
    K_air(3, 7) = K(3, 7);

    if (state == IDEL) {
        lf->set_command(-0.785f, 0.0F, 0.0F, param.motor_kp, param.motor_kd);
        rf->set_command(-0.785f, 0.0F, 0.0F, param.motor_kp, param.motor_kd);
        lb->set_command(0.785f, 0.0F, 0.0F, param.motor_kp, param.motor_kd);
        rb->set_command(0.785f, 0.0F, 0.0F, param.motor_kp, param.motor_kd);
        lw->set_command(0.0F, 0.0F, 0.0F, 0.0F, 0.01F);
        rw->set_command(0.0F, 0.0F, 0.0F, 0.0F, 0.01F);
        if (exp_state == VMC_TEST) {
            state = VMC_TEST;
        }else if(exp_state==KINAMIC_TEST)
            state=KINAMIC_TEST;
        else if(exp_state==LQR_CTRL)
            state=LQR_CTRL;
    } else if (state == KINAMIC_TEST) {
        const Eigen::Vector2d leg_target(0.2, 0.0);
        Eigen::Vector2d joint_target = Eigen::Vector2d::Zero();
        if (!leg_calc_->inverse_kinematics(leg_target, joint_target) || !joint_target.allFinite()) {
            return false;
        }

        lf->set_command(static_cast<float>(joint_target[0]), 0.0F, 0.0F, param.motor_kp, param.motor_kd);
        rf->set_command(static_cast<float>(joint_target[0]), 0.0F, 0.0F, param.motor_kp, param.motor_kd);
        lb->set_command(static_cast<float>(joint_target[1]), 0.0F, 0.0F, param.motor_kp, param.motor_kd);
        rb->set_command(static_cast<float>(joint_target[1]), 0.0F, 0.0F, param.motor_kp, param.motor_kd);
        lw->set_command(0.0F, 0.0F, 0.0F, 0.0F, 0.01F);
        rw->set_command(0.0F, 0.0F, 0.0F, 0.0F, 0.01F);

        Eigen::Vector2d pos;
        leg_calc_->forward_kinematics(Eigen::Vector2d(lf->state.rad,lb->state.rad), pos);
        debug_log("left_leg_length=%lf",pos[0]);
        if (exp_state == IDEL) {
            state = IDEL;
        }else if(exp_state==VMC_TEST)
            state=VMC_TEST;
        else if(exp_state==LQR_CTRL)
            state=LQR_CTRL;
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

        debug_log("left_length:%lf,right_length:%lf",left_leg_pos[0],right_leg_pos[0]);

        lf->set_command(0.0f, 0.0f, left_joint_exp_torque[0], 0.0f, 0.0f);
        rf->set_command(0.0f, 0.0f, right_joint_exp_torque[0], 0.0f, 0.0f);
        lb->set_command(0.0f, 0.0f, left_joint_exp_torque[1], 0.0f, 0.0f);
        rb->set_command(0.0f, 0.0f, right_joint_exp_torque[1], 0.0f, 0.0f);
        
        if (exp_state == IDEL) {
            state = IDEL;
        }else if(exp_state==KINAMIC_TEST)
            state=KINAMIC_TEST;
        else if(exp_state==LQR_CTRL)
            state=LQR_CTRL;
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
            time_point=ms;
        }
        double duration=(ms-time_point)*0.001;
        left_leg_slope.get_target(joint_pos, duration);
        lf->set_command(joint_pos[0],0.0f,0.0f,param.motor_kp,param.motor_kd);
        lb->set_command(joint_pos[1],0.0f,0.0f,param.motor_kp,param.motor_kd);
        bool ret=right_leg_slope.get_target(joint_pos, duration);
        rf->set_command(joint_pos[0],0.0f,0.0f,param.motor_kp,param.motor_kd);
        rb->set_command(joint_pos[1],0.0f,0.0f,param.motor_kp,param.motor_kd);
        if(!ret)    //复位完成，切到下一个状态
        {
            reset_traj_generated=false;
            exp_state=state=LQR_CTRL;
            debug_log("TOUCH_GROUND:%d",state);
        }
    }
    else if(state==LQR_CTRL)
    {
        //常量定义
        constexpr double Rw=0.058;

        Eigen::Quaternionf orientation;
        Eigen::Vector3d angular_velocity;
        bool imu_state_valid;
        imu->lock_memory();
        orientation = imu->orientation;
        angular_velocity = imu->angular_velocity;
        imu_state_valid = imu->state_valid_;
        imu->unlock_memory();

        if (!imu_state_valid) {
            return false;
        }

        double yaw;
        double pitch;
        double roll;
        if (!extract_ypr(orientation, yaw, pitch, roll)) {
            return false;
        }

        if (!yaw_tracking_initialized_) {
            yaw_tracking_initialized_ = true;
            yaw_previous_ = yaw;
            yaw_unwrapped_ = yaw;
            yaw_reference_ = yaw;
        } else {
            yaw_unwrapped_ += wrap_to_pi(yaw - yaw_previous_);
            yaw_previous_ = yaw;
        }

        // if(std::abs(eular[1])>0.3||std::abs(eular[2])>0.3)  //姿态发散，切入位控
        // {
        //     const Eigen::Vector2d leg_target(0.25, 0.0);
        //     Eigen::Vector2d joint_target = Eigen::Vector2d::Zero();
        //     if (!leg_calc_->inverse_kinematics(leg_target, joint_target) || !joint_target.allFinite()) {
        //         return false;
        //     }

        //     lf->set_command(static_cast<float>(joint_target[0]), 0.0F, 0.0F, param.motor_kp, param.motor_kd);
        //     rf->set_command(static_cast<float>(joint_target[0]), 0.0F, 0.0F, param.motor_kp, param.motor_kd);
        //     lb->set_command(static_cast<float>(joint_target[1]), 0.0F, 0.0F, param.motor_kp, param.motor_kd);
        //     rb->set_command(static_cast<float>(joint_target[1]), 0.0F, 0.0F, param.motor_kp, param.motor_kd);
        //     lw->set_command(0.0F, 0.0F, 0.0F, 0.0F, 0.0F);
        //     rw->set_command(0.0F, 0.0F, 0.0F, 0.0F, 0.0F);
        //     return true;
        // }

        //计算腿的位置，速度，受力
        Eigen::Vector2d left_joint_pos(lf->state.rad,lb->state.rad);
        Eigen::Vector2d right_joint_pos(rf->state.rad,rb->state.rad);
        Eigen::Vector2d left_leg_pos,right_leg_pos,left_leg_vel,right_leg_vel,left_leg_force,right_leg_force;
        leg_calc_->forward_kinematics(left_joint_pos, left_leg_pos);
        leg_calc_->forward_kinematics(right_joint_pos, right_leg_pos);
        leg_calc_->forward_velocity(left_joint_pos, Eigen::Vector2d(lf->state.vel,lb->state.vel), left_leg_vel);
        leg_calc_->forward_velocity(right_joint_pos, Eigen::Vector2d(rf->state.vel,rb->state.vel), right_leg_vel);
        leg_calc_->forward_dynamics(left_joint_pos, Eigen::Vector2d(lf->state.toqeue,lb->state.toqeue), left_leg_force);
        leg_calc_->forward_dynamics(right_joint_pos, Eigen::Vector2d(rf->state.toqeue,rb->state.toqeue), right_leg_force);

        
        //填写状态向量
        // x = sp.Matrix([
        //     s, ds,
        //     phi, dphi,
        //     thll, dthll,
        //     thlr, dthlr,
        //     thb, dthb])
        
        Eigen::Vector<double,10> x,xd;
        xd.setZero();   //参考输入
        const double s=0.5*(lw->state.rad+rw->state.rad)*Rw;
        double ds=0.5*(lw->state.vel+rw->state.vel)*Rw;
        const double phi = yaw_unwrapped_ - yaw_reference_;
        double dphi=angular_velocity[2];
        const double thb=pitch;
        double dthb=angular_velocity[1];
        const double thll=wrap_to_pi(thb+left_leg_pos[1]);
        const double thlr=wrap_to_pi(thb+right_leg_pos[1]);
        double dthll=dthb+left_leg_vel[1];
        double dthlr=dthb+right_leg_vel[1];

        ds = ds_filter_.update(ds, 0.07);
        dphi = dphi_filter_.update(dphi, 0.7);
        dthll = dthll_filter_.update(dthll, 0.7);
        dthlr = dthlr_filter_.update(dthlr, 0.7);
        dthb = dthb_filter_.update(dthb, 0.7);

        x<<s,ds,phi,dphi,thll,dthll,thlr,dthlr,thb,dthb;
        debug_log("x:[%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf]",
                  x[0], x[1], x[2], x[3], x[4],
                  x[5], x[6], x[7], x[8], x[9]);
        
        // xd[0]=s;
        // xd[2]=phi;

        //u = sp.Matrix([Twl, Twr, Tbl, Tbr])
        Eigen::Vector4d u=Eigen::Vector4d::Zero();  //计算LQR控制律
        
        //接触状态判断
        //debug_log("left_force:%lf,right_force:%lf", left_leg_force[0],right_leg_force[0]);
        debug_log("left_length:%lf,right_length:%lf", left_leg_pos[0],right_leg_pos[0]);
        
        
        if(left_leg_force[0]>25.0&&right_leg_force[0]>25.0) //两轮接地
        {
            u=K*(xd-x);
        }
        // else if(left_leg_force[0]>30.0&&right_leg_force[0]<30.0) //左轮接地
        // {
        //     u=K_air*(xd-x);
        // }
        // else if(left_leg_force[0]<30.0&&right_leg_force[0]>30.0) //右轮接地
        // {
        //     u=K_air*(xd-x);
        // }
        else //左右轮都没有接地
        {
            u=K_air*(xd-x);
        }

        //腿长PD控制器
        Eigen::Vector2d left_leg_exp_force,right_leg_exp_force,left_joint_torque,right_joint_torque;
        left_leg_exp_force[1]=u[2];
        right_leg_exp_force[1]=u[3];
        left_leg_exp_force[0]=left_leg_length.update(left_leg_pos[0],left_leg_vel[0],ref_height);
        right_leg_exp_force[0]=right_leg_length.update(right_leg_pos[0],right_leg_vel[0],ref_height);
        leg_calc_->inverse_dynamics(left_joint_pos, left_leg_exp_force, left_joint_torque);
        leg_calc_->inverse_dynamics(right_joint_pos, right_leg_exp_force, right_joint_torque);

        lf->set_command(0.0f, 1.0f, left_joint_torque[0], 0.0f, 0.0f);
        lb->set_command(0.0f, 1.0f, left_joint_torque[1], 0.0f, 0.0f);
        rf->set_command(0.0f, 1.0f, right_joint_torque[0], 0.0f, 0.0f);
        rb->set_command(0.0f, 1.0f, right_joint_torque[1], 0.0f, 0.0f);
        lw->set_command(0.0f, 0.0f, u[0], 0.0f, 0.0f);
        rw->set_command(0.0f, 0.0f, u[1], 0.0f, 0.0f);


        if(exp_state==KINAMIC_TEST)
            state=KINAMIC_TEST;
        else if(exp_state==IDEL)
            state=IDEL;
    }
    return true;
}


void ControllerAT::input(float velocity, float omega, float height, int mode) {
    (void)velocity;
    (void)omega;
    ref_height=height;

    if (mode == 1) {
        exp_state = IDEL;
    } else if (mode == 2) {
        exp_state = KINAMIC_TEST;
    }else if (mode == 3) {
        exp_state = VMC_TEST;
    }else if (mode == 4) {
        exp_state = LQR_CTRL;
    }
}
