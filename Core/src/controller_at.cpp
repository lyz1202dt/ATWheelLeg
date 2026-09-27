#include "controller_at.hpp"
#include <algorithm>
#include <Eigen/src/Core/Matrix.h>
#include <array>
#include <chrono>
#include <cmath>
#include <complex>
#include <utility>

ControllerAT::ControllerAT(IMUBase* imu, Motor* lf, Motor* rf, Motor* lb, Motor* rb, Motor* lw, Motor* rw)
    : ControllerBase(imu, lf, rf, lb, rb, lw, rw)
    , left_leg_length(1000.0f, 15.0f, 0.0f, 0.0f, 200.0f, 0.002f)
    , right_leg_length(1000.0f, 15.0f, 0.0f, 0.0f, 40.0f, 0.002f) {
    leg_calc_ = std::make_unique<OffsetParallelCalc>(0.0945, 0.0945, 0.1125, 0.1125, 0.1155, 0.2502);
}

void ControllerAT::register_debug_logger(DebugLogger logger) {
    debug_logger_ = std::move(logger);
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
    Eigen::Quaterniond quaternion(
        static_cast<double>(orientation.w()),
        static_cast<double>(orientation.x()),
        static_cast<double>(orientation.y()),
        static_cast<double>(orientation.z()));

    const double norm = quaternion.norm();
    if (!std::isfinite(norm) || norm < 1.0e-9) {
        return false;
    }
    quaternion.normalize();

    const Eigen::Matrix3d rotation = quaternion.toRotationMatrix();
    yaw = std::atan2(rotation(1, 0), rotation(0, 0));
    pitch = std::atan2(
        -rotation(2, 0),
        std::hypot(rotation(0, 0), rotation(1, 0)));
    roll = std::atan2(rotation(2, 1), rotation(2, 2));

    return std::isfinite(yaw) && std::isfinite(pitch) && std::isfinite(roll);
}

double ControllerAT::low_pass_filter(double input,
                                     double alpha,
                                     double& filtered_value,
                                     bool& initialized) {
    if (!std::isfinite(input)) {
        return initialized ? filtered_value : 0.0;
    }

    if (!initialized) {
        filtered_value = input;
        initialized = true;
        return filtered_value;
    }

    const double clamped_alpha =
        std::clamp(std::isfinite(alpha) ? alpha : 1.0, 0.0, 1.0);
    filtered_value = (1.0 - clamped_alpha) * filtered_value + clamped_alpha * input;
    return filtered_value;
}

bool ControllerAT::update(float dt) {
    (void)dt;
    if (state != LQR_CTRL) {
        yaw_tracking_initialized_ = false;
    }

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

        ds = low_pass_filter(ds, 0.07, filtered_ds_, ds_filter_initialized_);
        dphi = low_pass_filter(dphi, 0.7, filtered_dphi_, dphi_filter_initialized_);
        dthll = low_pass_filter(dthll, 0.7, filtered_dthll_, dthll_filter_initialized_);
        dthlr = low_pass_filter(dthlr, 0.7, filtered_dthlr_, dthlr_filter_initialized_);
        dthb = low_pass_filter(dthb, 0.7, filtered_dthb_, dthb_filter_initialized_);

        x<<s,ds,phi,dphi,thll,dthll,thlr,dthlr,thb,dthb;
        debug_log("x:[%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf]",
                  x[0], x[1], x[2], x[3], x[4],
                  x[5], x[6], x[7], x[8], x[9]);
        
        // xd[0]=s;
        // xd[2]=phi;
        //TODO:根据腿长拟合不同的K矩阵

        //u = sp.Matrix([Twl, Twr, Tbl, Tbr])
        Eigen::Vector4d u=Eigen::Vector4d::Zero();  //计算LQR控制律
        
        //接触状态判断
        debug_log("left_force:%lf,right_force:%lf", left_leg_force[0],right_leg_force[0]);
        u=K*(xd-x);
        // if(left_leg_force[0]>30.0&&right_leg_force[0]>30.0) //两轮接地
        // {
            
        // }
        // else if(left_leg_force[0]>30.0&&right_leg_force[0]<30.0) //左轮接地
        // {
        //     u=K_air*(xd-x);
        // }
        // else if(left_leg_force[0]<30.0&&right_leg_force[0]>30.0) //右轮接地
        // {
        //     u=K_air*(xd-x);
        // }
        // else //左右轮都没有接地
        // {
        //     u=K_air*(xd-x);
        // }

        //腿长PD控制器
        Eigen::Vector2d left_leg_exp_force,right_leg_exp_force,left_joint_torque,right_joint_torque;
        left_leg_exp_force[1]=u[2];
        right_leg_exp_force[1]=u[3];
        left_leg_exp_force[0]=left_leg_length.update(left_leg_pos[0],left_leg_vel[0],0.25f);
        right_leg_exp_force[0]=right_leg_length.update(right_leg_pos[0],right_leg_vel[0],0.25f);
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
    (void)height;

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


bool ControllerAT::update_lqr_k(const Eigen::Vector<float,10> &Q,const Eigen::Vector<float,4> &R)
{
    using Matrix10d  = Eigen::Matrix<double, 10, 10>;
    using Matrix104d = Eigen::Matrix<double, 10, 4>;
    using Matrix4d   = Eigen::Matrix<double, 4, 4>;
    using Matrix20d  = Eigen::Matrix<double, 20, 20>;
    using Matrix10cd = Eigen::Matrix<std::complex<double>, 10, 10>;

    Eigen::Matrix<double, 10, 10> A;
A <<
    0.00000000, 1.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000,
    0.00000000, 0.00000000, 0.00000000, 0.00000000, -13.55160363, 0.00000000, -13.55160363, 0.00000000, 0.00000000, 0.00000000,
    0.00000000, 0.00000000, 0.00000000, 1.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000,
    0.00000000, 0.00000000, 0.00000000, 0.00000000, -2.14332607, 0.00000000, 2.14332607, 0.00000000, 0.00000000, 0.00000000,
    0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 1.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000,
    0.00000000, 0.00000000, 0.00000000, 0.00000000, 213.48521397, 0.00000000, -49.86561124, 0.00000000, 0.00000000, 0.00000000,
    0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 1.00000000, 0.00000000, 0.00000000,
    0.00000000, 0.00000000, 0.00000000, 0.00000000, -49.86561124, 0.00000000, 213.48521397, 0.00000000, 0.00000000, 0.00000000,
    0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 1.00000000,
    0.00000000, 0.00000000, 0.00000000, 0.00000000, 20.24352472, 0.00000000, 20.24352472, 0.00000000, -28.77747990, 0.00000000;

Eigen::Matrix<double, 10, 4> B;
B <<
    0.00000000, 0.00000000, 0.00000000, 0.00000000,
    5.75310641, 5.75310641, -0.93168984, -0.93168984,
    0.00000000, 0.00000000, 0.00000000, 0.00000000,
    -3.99485616, 3.99485616, -0.14735637, 0.14735637,
    0.00000000, 0.00000000, 0.00000000, 0.00000000,
    -71.63664239, 17.64525726, 14.67737768, -3.42832366,
    0.00000000, 0.00000000, 0.00000000, 0.00000000,
    17.64525726, -71.63664239, -3.42832366, 14.67737768,
    0.00000000, 0.00000000, 0.00000000, 0.00000000,
    -2.92121265, -2.92121265, -5.28991389, -5.28991389;

    Matrix10d state_cost = Matrix10d::Zero();
    for (Eigen::Index i = 0; i < 10; ++i) {
        const double weight = static_cast<double>(Q[i]);
        if (!std::isfinite(weight) || weight < 0.0) {
            return false;
        }
        state_cost(i, i) = weight;
    }

    Matrix4d input_cost_inverse = Matrix4d::Zero();
    for (Eigen::Index i = 0; i < 4; ++i) {
        const double weight = static_cast<double>(R[i]);
        if (!std::isfinite(weight) || weight <= 0.0) {
            return false;
        }
        input_cost_inverse(i, i) = 1.0 / weight;
    }

    Matrix20d hamiltonian = Matrix20d::Zero();
    hamiltonian.template block<10, 10>(0, 0) = A;
    hamiltonian.template block<10, 10>(0, 10) =
        -B * input_cost_inverse * B.transpose();
    hamiltonian.template block<10, 10>(10, 0) = -state_cost;
    hamiltonian.template block<10, 10>(10, 10) = -A.transpose();

    Eigen::ComplexEigenSolver<Matrix20d> eigen_solver(hamiltonian);
    if (eigen_solver.info() != Eigen::Success) {
        return false;
    }

    const auto eigenvalues = eigen_solver.eigenvalues();
    const auto eigenvectors = eigen_solver.eigenvectors();
    std::array<int, 10> stable_indices{};
    Eigen::Index stable_count = 0;
    for (Eigen::Index i = 0; i < eigenvalues.size(); ++i) {
        if (eigenvalues[i].real() < -1.0e-8) {
            if (stable_count >= static_cast<Eigen::Index>(stable_indices.size())) {
                return false;
            }
            stable_indices[static_cast<std::size_t>(stable_count++)] =
                static_cast<int>(i);
        }
    }

    if (stable_count != 10) {
        return false;
    }

    Matrix10cd u1;
    Matrix10cd u2;
    for (Eigen::Index col = 0; col < 10; ++col) {
        u1.col(col) =
            eigenvectors.template block<10, 1>(0, stable_indices[col]);
        u2.col(col) =
            eigenvectors.template block<10, 1>(10, stable_indices[col]);
    }

    const auto u1_decomposition = u1.fullPivLu();
    if (!u1_decomposition.isInvertible()) {
        return false;
    }

    const Matrix10cd p_complex = u2 * u1.inverse();
    const double max_imaginary = p_complex.imag().cwiseAbs().maxCoeff();
    if (!std::isfinite(max_imaginary) || max_imaginary > 1.0e-5) {
        return false;
    }

    Matrix10d p = p_complex.real();
    p = 0.5 * (p + p.transpose());

    const Eigen::Matrix<double, 4, 10> gain =
        input_cost_inverse * B.transpose() * p;
    if (!gain.allFinite()) {
        return false;
    }

    const Matrix10d residual =
        A.transpose() * p + p * A -
        p * B * input_cost_inverse * B.transpose() * p + state_cost;
    const double residual_norm = residual.norm();
    const double residual_scale =
        1.0 + state_cost.norm() + A.norm() * p.norm() +
        (p * B * input_cost_inverse * B.transpose() * p).norm();
    if (!std::isfinite(residual_norm) ||
        !std::isfinite(residual_scale) ||
        residual_norm > 1.0e-6 * residual_scale) {
        return false;
    }

    K = gain;
    K_air.setZero();

    // 左腿只反馈左腿绝对角度和角速度
    K_air(2, 4) = K(2, 4);  // thll
    K_air(2, 5) = K(2, 5);  // dthll

    // 右腿只反馈右腿绝对角度和角速度
    K_air(3, 6) = K(3, 6);  // thlr
    K_air(3, 7) = K(3, 7);

    return true;
}
