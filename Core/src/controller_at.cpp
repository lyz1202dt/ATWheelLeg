#include "controller_at.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <array>
#include <chrono>
#include <cmath>
#include <complex>
#include <utility>

ControllerAT::ControllerAT(IMUBase* imu, Motor* lf, Motor* rf, Motor* lb, Motor* rb, Motor* lw, Motor* rw)
    : ControllerBase(imu, lf, rf, lb, rb, lw, rw)
    , left_leg_length(700.0f, 20.0f, 0.0f, 0.0f, 200.0f, 0.002f)
    , right_leg_legth(700.0f, 20.0f, 0.0f, 0.0f, 200.0f, 0.002f) {
    leg_calc_ = std::make_unique<OffsetParallelCalc>(0.0945, 0.0945, 0.1125, 0.1125, 0.1155, 0.2502);
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
            exp_state=state=LQR_CTRL;
            debug_log("TOUCH_GROUND:%d",state);
        }
    }
    else if(state==LQR_CTRL)
    {
        //常量定义
        constexpr double Rw=0.058;

        Eigen::Vector3d eular=imu->orientation.toRotationMatrix().eulerAngles(0, 1, 2).cast<double>(); //0-x,1-y,2-z  先提取z轴
        debug_log("x:%lf,y:%lf", eular[0],eular[1]);
        if(std::abs(eular[0])>0.3||std::abs(eular[1])>0.3)  //姿态发散，切入位控
        {
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
            return true;
        }

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
        
        Eigen::Vector<double,10> x;
        const double s=0.5*(lw->state.rad+rw->state.rad)*Rw;
        const double ds=0.5*(lw->state.vel+rw->state.vel)*Rw;
        const double phi=eular[2];
        const double dphi=imu->angular_velocity[2];
        const double thb=eular[1];
        const double dthb=imu->angular_velocity[1];
        const double thll=thb+left_leg_pos[1];
        const double thlr=thb+right_leg_pos[1];
        const double dthll=dthb+left_leg_vel[1];
        const double dthlr=dthb+right_leg_vel[1];
        x<<s,ds,phi,dphi,thll,dthll,thlr,dthlr,thb,dthb;
        //debug_log(const char *format, Args args...)

        //TODO:根据腿长拟合不同的K矩阵

        //u = sp.Matrix([Twl, Twr, Tbl, Tbr])
        Eigen::Vector4d u=Eigen::Vector4d::Zero();  //计算LQR控制律
        
        //接触状态判断
        debug_log("left_force:%lf,right_force:%lf", left_leg_force[0],right_leg_force[0]);
        if(left_leg_force[0]>30.0&&right_leg_force[0]>30.0) //两轮接地
        {
            u=-K*x;
        }
        else if(left_leg_force[0]>30.0&&right_leg_force[0]<30.0) //左轮接地
        {
            u=-K_air*x;
        }
        else if(left_leg_force[0]<30.0&&right_leg_force[0]>30.0) //右轮接地
        {
            u=-K_air*x;
        }
        else //左右轮都没有接地
        {
            u=-K_air*x;
        }

        //腿长PD控制器
        Eigen::Vector2d left_leg_exp_force,right_leg_exp_force,left_joint_torque,right_joint_torque;
        left_leg_exp_force[1]=u[2];
        right_leg_exp_force[1]=u[3];
        left_leg_exp_force[0]=left_leg_length.update(left_leg_pos[0],left_leg_vel[0],0.23f);
        right_leg_exp_force[0]=left_leg_length.update(right_leg_pos[0],right_leg_vel[0],0.23f);
        leg_calc_->inverse_dynamics(left_joint_pos, left_leg_exp_force, left_joint_torque);
        leg_calc_->inverse_dynamics(right_joint_pos, right_leg_exp_force, right_joint_torque);

        lf->set_command(0.0f, 1.0f, left_joint_torque[0], 0.0f, 0.0f);
        lb->set_command(0.0f, 1.0f, left_joint_torque[1], 0.0f, 0.0f);
        rf->set_command(0.0f, 1.0f, right_joint_torque[0], 0.0f, 0.0f);
        rb->set_command(0.0f, 1.0f, right_joint_torque[1], 0.0f, 0.0f);
        lw->set_command(0.0f, 0.0f, u[0], 0.0f, 0.0f);
        rw->set_command(0.0f, 0.0f, u[1], 0.0f, 0.0f);
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
    0.00000000, 0.00000000, 0.00000000, 0.00000000, 1.04657099, 0.00000000, 3.62114144, 0.00000000, 0.00000000, 0.00000000,
    0.00000000, 0.00000000, 0.00000000, 1.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000,
    0.00000000, 0.00000000, 0.00000000, 0.00000000, -82.96206979, 0.00000000, 202.45066195, 0.00000000, 0.00000000, 0.00000000,
    0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 1.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000,
    0.00000000, 0.00000000, 0.00000000, 0.00000000, 128.00000000, 0.00000000, -64.00000000, 0.00000000, 0.00000000, 0.00000000,
    0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 1.00000000, 0.00000000, 0.00000000,
    0.00000000, 0.00000000, 0.00000000, 0.00000000, -28.00000000, 0.00000000, 192.00000000, 0.00000000, 0.00000000, 0.00000000,
    0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 1.00000000,
    0.00000000, 0.00000000, 0.00000000, 0.00000000, 1.12321884, 0.00000000, 1.47554772, 0.00000000, -2.37621800, 0.00000000;

    Matrix104d B;
    B <<
    0.00000000, 0.00000000, 0.00000000, 0.00000000,
    -6.39170188, -51.94137680, -0.76184574, 0.32004766,
    0.00000000, 0.00000000, 0.00000000, 0.00000000,
    -204.15972806, -159.77328584, 0.14716707, 18.36519794,
    0.00000000, 0.00000000, 0.00000000, 0.00000000,
    32.00000000, -96.00000000, 4.00000000, -6.00000000,
    0.00000000, 0.00000000, 0.00000000, 0.00000000,
    -96.00000000, -256.00000000, -2.25000000, 16.00000000,
    0.00000000, 0.00000000, 0.00000000, 0.00000000,
    0.05734234, 0.78015583, -2.66759543, -2.64094828;

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
