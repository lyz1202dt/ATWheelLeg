#include "controller/controller_at.hpp"
#include <Eigen/src/Core/DiagonalMatrix.h>
#include <Eigen/src/Core/Matrix.h>
#include <algorithm>
#include <cmath>
#include <utility>

namespace {

constexpr double kControlDt        = 0.002;
constexpr double kLqrLesoBandwidth = 30.0;
constexpr double kGravity          = 9.80665;
constexpr double kBodyKfNominalLegLength = 0.25;
constexpr double kBodyKfSVariance        = 50.0;
constexpr double kBodyKfPhiVariance      = 1e-4;
constexpr double kBodyKfLegAngleVariance = 1e-4;
constexpr double kBodyKfBodyAngleVariance = 1e-5;
constexpr double kBodyKfAccelVariance     = 25.0;

using BodyKf = kf::KalmanFilter<10, 4, 11>;

Eigen::Matrix<double, 10, 10> make_lqr_a_025() {
    Eigen::Matrix<double, 10, 10> a;
    a << 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -13.551603629409335, 0.0, -13.551603629409325, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -2.1433260747833671, 0.0, 2.1433260747834382, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 213.48521396900344, 0.0, -49.865611238637868, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -49.865611238637847, 0.0, 213.48521396900355, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 20.243524724661789, 0.0, 20.243524724661786, 0.0,
        -28.777479900672802, 0.0;
    return a;
}

Eigen::Matrix<double, 10, 4> make_lqr_b_025() {
    Eigen::Matrix<double, 10, 4> b;
    b << 0.0, 0.0, 0.0, 0.0, 5.7531064060851165, 5.7531064060851165, -0.93168983906140512, -0.93168983906140523, 0.0, 0.0, 0.0, 0.0,
        -3.9948561620043392, 3.9948561620043392, -0.14735637052853356, 0.14735637052853545, 0.0, 0.0, 0.0, 0.0, -71.636642386104896,
        17.645257263241902, 14.677377680462749, -3.4283236567516195, 0.0, 0.0, 0.0, 0.0, 17.645257263241909, -71.636642386104896,
        -3.4283236567516173, 14.677377680462749, 0.0, 0.0, 0.0, 0.0, -2.9212126457989163, -2.9212126457989163, -5.2899138885268169,
        -5.289913888526816;
    return b;
}

Eigen::Matrix<double, 10, 10> make_body_kf_q() {
    Eigen::Matrix<double, 10, 1> q;
    // Q is treated as continuous-time process noise and is multiplied by dt
    // in KalmanFilter::update(y, u, dt). The velocity states need enough
    // process noise to let the direct low-pass velocity observations correct
    // model/input mismatch at 500 Hz.
    q << 1e-5, 5.0,  // s, ds
        1e-6, 2.0,   // phi, dphi
        1e-5, 20.0,  // thll, dthll
        1e-5, 20.0,  // thlr, dthlr
        1e-6, 10.0;  // thb, dthb
    return q.asDiagonal();
}

Eigen::Matrix<double, 11, 11> make_body_kf_r() {
    Eigen::Matrix<double, 11, 1> r;
    r << kBodyKfSVariance, // s
        50.0,  // ds, low-pass wheel velocity
        kBodyKfPhiVariance,  // phi
        2e-2,  // dphi, low-pass yaw rate
        kBodyKfLegAngleVariance,  // thll
        5e-2,  // dthll, low-pass left leg angular velocity
        kBodyKfLegAngleVariance,  // thlr
        5e-2,  // dthlr, low-pass right leg angular velocity
        kBodyKfBodyAngleVariance,  // thb
        2e-2,  // dthb, low-pass pitch rate
        kBodyKfAccelVariance;  // ax, includes sensor noise plus linearized-model mismatch
    return r.asDiagonal();
}

Eigen::Matrix<double, 10, 10> make_body_kf_p0() {
    Eigen::Matrix<double, 10, 1> p0;
    p0 << 5e-2, 5e-1, // s, ds
        2e-2, 1e-1,   // phi, dphi
        4e-2, 1e-1,   // thll, dthll
        4e-2, 1e-1,   // thlr, dthlr
        1e-2, 1e-1;   // thb, dthb
    return p0.asDiagonal();
}

Eigen::Matrix<double, 1, 10> make_body_accel_h(
    const Eigen::Matrix<double, 10, 10>& A, double left_leg_length, double right_leg_length) {
    return A.row(1) + 0.5 * left_leg_length * A.row(5) + 0.5 * right_leg_length * A.row(7);
}

Eigen::Matrix<double, 1, 4> make_body_accel_d(
    const Eigen::Matrix<double, 10, 4>& B, double left_leg_length, double right_leg_length) {
    return B.row(1) + 0.5 * left_leg_length * B.row(5) + 0.5 * right_leg_length * B.row(7);
}

BodyKf::MeasControlMat make_body_kf_d(const BodyKf& body_kf, double left_leg_length, double right_leg_length) {
    BodyKf::MeasControlMat d = BodyKf::MeasControlMat::Zero();
    d.row(10) = make_body_accel_d(body_kf.B, left_leg_length, right_leg_length);
    return d;
}

void update_body_accel_observation(BodyKf& body_kf, double left_leg_length, double right_leg_length) {
    body_kf.H.row(10) = make_body_accel_h(body_kf.A, left_leg_length, right_leg_length);
}

double forward_linear_acceleration(
    const Eigen::Quaternionf& orientation, const Eigen::Vector3d& acceleration, double yaw) {
    const Eigen::Matrix3d base_link_to_world = orientation.toRotationMatrix().cast<double>();
    const Eigen::Vector3d specific_force_in_world = base_link_to_world * acceleration;
    const Eigen::Vector3d linear_acceleration_in_world = specific_force_in_world + Eigen::Vector3d(0.0, 0.0, -kGravity);
    const Eigen::Vector3d forward_in_world(std::cos(yaw), std::sin(yaw), 0.0);
    return forward_in_world.dot(linear_acceleration_in_world);
}

BodyKf::StateVec make_body_kf_state(
    double s, double ds, double phi, double dphi, double thll, double dthll, double thlr, double dthlr, double thb, double dthb) {
    BodyKf::StateVec state;
    state << s, ds, phi, dphi, thll, dthll, thlr, dthlr, thb, dthb;
    return state;
}

BodyKf::ControlVec make_body_kf_input_from_motor_torque(
    double left_wheel_torque, double right_wheel_torque,
    const Eigen::Vector2d& left_leg_effort, const Eigen::Vector2d& right_leg_effort) {
    BodyKf::ControlVec input;
    // leg_effort[1] is the generalized torque around the simplified rigid leg root.
    input << left_wheel_torque, right_wheel_torque, left_leg_effort[1], right_leg_effort[1];
    if (!input.allFinite()) {
        input.setZero();
    }
    return input;
}

BodyKf::StateVec make_body_control_state(
    const BodyKf& body_kf, double s, double phi, double thll, double thlr, double thb) {
    BodyKf::StateVec state = body_kf.state();
    state[0] = s;
    state[2] = phi;
    state[4] = thll;
    state[6] = thlr;
    state[8] = thb;
    return state;
}

void anchor_body_kf_direct_positions(
    BodyKf& body_kf, BodyKf::StateVec& state, double s, double phi, double thll, double thlr, double thb) {
    auto covariance = body_kf.covariance();
    state = make_body_control_state(body_kf, s, phi, thll, thlr, thb);

    const auto anchor = [&covariance](const Eigen::Index index, const double variance) {
        covariance.row(index).setZero();
        covariance.col(index).setZero();
        covariance(index, index) = variance;
    };
    anchor(0, kBodyKfSVariance);
    anchor(2, kBodyKfPhiVariance);
    anchor(4, kBodyKfLegAngleVariance);
    anchor(6, kBodyKfLegAngleVariance);
    anchor(8, kBodyKfBodyAngleVariance);

    body_kf.reset(state, covariance);
}

} // namespace

ControllerAT::ControllerAT(
    IMUBase* imu, Motor* lf, Motor* rf, Motor* lb, Motor* rb, Motor* lw, Motor* rw, GainSchedulerFunc func, Param* p_param,
    DebugLogger debug_logger)
    : ControllerBase(imu, lf, rf, lb, rb, lw, rw, std::move(debug_logger))
    , gain_scheduler_func(std::move(func))
    , body_kf(
          make_lqr_a_025(), make_lqr_b_025(),
          Eigen::Matrix<double, 11, 10>::Zero(),
          make_body_kf_q(),
          make_body_kf_r())
    , wheel_kf(
          (Eigen::Matrix2d() << 1.0, 0.002, 0.0, 1.0).finished(), (Eigen::Matrix<double, 2, 1>() << 0.5 * 0.002 * 0.002, 0.002).finished(),
          (Eigen::Matrix<double, 1, 2>() << 1.0, 0.0).finished(), (Eigen::Matrix2d() << 0.005, 0.0, 0.0, 0.002).finished(),
          Eigen::Matrix<double, 1, 1>::Constant(3.0))
    , ds_filter_(0.07)
    , dphi_filter_(0.7)
    , dthll_filter_(0.7)
    , dthlr_filter_(0.7)
    , dthb_filter_(0.7)
    , roll_rate_filter_(0.7)
    , left_leg_force_filter_(0.3)
    , right_leg_force_filter_(0.3)
    , left_leg_length(600.0f, 100.0f, 0.0f, 0.0f, 200.0f, 0.002f)
    , right_leg_length(600.0f, 100.0f, 0.0f, 0.0f, 200.0f, 0.002f)
    , roll_pd(500.0f, 200.0f, 0.0f, 0.0f, 80.0f, 0.002f) {
    if (p_param != nullptr) {
        param = *p_param;
    }
    leg_calc_ = std::make_unique<OffsetParallelCalc>(0.0945, 0.0945, 0.1125, 0.1125, 0.1155, 0.2502);
    wheel_kf.reset(Eigen::Vector2d::Zero(), Eigen::Matrix2d::Identity() * 10.0);

    body_kf.H.block<10, 10>(0, 0).setIdentity(); // s, ds, phi, dphi, thll, dthll, thlr, dthlr, thb, dthb
    // IMU forward acceleration observes the body horizontal acceleration:
    // sb ~= s + 0.5*ll*thll + 0.5*lr*thlr, so
    // ax ~= dds + 0.5*ll*ddthll + 0.5*lr*ddthlr.
    update_body_accel_observation(body_kf, kBodyKfNominalLegLength, kBodyKfNominalLegLength);
    body_kf.reset(Eigen::Vector<double, 10>::Zero(), make_body_kf_p0());

    const auto lqr_a = make_lqr_a_025();
    const auto lqr_b = make_lqr_b_025();
    lqr_disturbance_observer_ =
        std::make_unique<LqrDisturbanceObserver>(lqr_a, lqr_b, lqr_b, Eigen::Matrix<double, 10, 10>::Identity(), kLqrLesoBandwidth);
}

double ControllerAT::wrap_to_pi(double angle) {
    constexpr double kPi    = 3.14159265358979323846;
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

bool ControllerAT::extract_ypr(const Eigen::Quaternionf& orientation, double& yaw, double& pitch, double& roll) {
    const Eigen::Matrix3d rotation = orientation.toRotationMatrix().cast<double>();
    yaw                            = std::atan2(rotation(1, 0), rotation(0, 0));
    pitch                          = std::atan2(-rotation(2, 0), std::hypot(rotation(0, 0), rotation(1, 0)));
    roll                           = std::atan2(rotation(2, 1), rotation(2, 2));

    return true;
}

bool ControllerAT::update(uint64_t ms) {

    Eigen::Quaternionf orientation;
    Eigen::Vector3d angular_velocity;
    Eigen::Vector3d acceleration;
    bool imu_state_valid;
    imu->lock_memory();
    orientation      = imu->orientation;
    angular_velocity = imu->angular_velocity;
    acceleration     = imu->acceleration;
    imu_state_valid  = imu->state_valid_;
    imu->unlock_memory();

    if (!imu_state_valid || !acceleration.allFinite()) {
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
        yaw_previous_             = yaw;
        yaw_unwrapped_            = yaw;
        yaw_reference_            = yaw;
    } else {
        yaw_unwrapped_ += wrap_to_pi(yaw - yaw_previous_);
        yaw_previous_ = yaw;
    }


    if (state != LQR_CTRL && state != LQR_STEP && state != LQR_JUMP) {
        yaw_tracking_initialized_ = false;
        lqr_disturbance_observer_initialized_ = false;
        lqr_disturbance_.setZero();
        body_kf_initialized_ = false;
        previous_lqr_command_.setZero();
    }

    const Eigen::Vector2d left_joint_position(lf->state.rad, lb->state.rad);
    const Eigen::Vector2d right_joint_position(rf->state.rad, rb->state.rad);
    Eigen::Vector2d left_leg_position;
    Eigen::Vector2d right_leg_position;
    if (!leg_calc_->forward_kinematics(left_joint_position, left_leg_position)
        || !leg_calc_->forward_kinematics(right_joint_position, right_leg_position) || !left_leg_position.allFinite()
        || !right_leg_position.allFinite() || gain_scheduler_func == nullptr
        || !gain_scheduler_func(left_leg_position[0], right_leg_position[0], K) || !K.allFinite()) {
        return false;
    }

    K_air.setZero();
    K_air(2, 4) = K(2, 4);
    K_air(2, 5) = K(2, 5);
    K_air(3, 6) = K(3, 6);
    K_air(3, 7) = K(3, 7);

    // 单轮接地时保留接地轮的平衡控制，以及两条腿各自的姿态稳定项。
    // u = [Twl, Twr, Tbl, Tbr]，因此左轮接地对应 K 的第 0 行，
    // 右轮接地对应 K 的第 1 行。
    K_right_air        = K_air;
    K_right_air.row(0) = K.row(0);

    K_left_air        = K_air;
    K_left_air.row(1) = K.row(1);

    if (state == IDEL) {
        lf->set_command(-0.785f, 0.0F, 0.0F, param.motor_kp, param.motor_kd);
        rf->set_command(-0.785f, 0.0F, 0.0F, param.motor_kp, param.motor_kd);
        lb->set_command(0.785f, 0.0F, 0.0F, param.motor_kp, param.motor_kd);
        rb->set_command(0.785f, 0.0F, 0.0F, param.motor_kp, param.motor_kd);
        lw->set_command(0.0F, 0.0F, 0.0F, 0.0F, 0.01F);
        rw->set_command(0.0F, 0.0F, 0.0F, 0.0F, 0.01F);
        if (exp_state == VMC_TEST) {
            state = VMC_TEST;
        } else if (exp_state == KINAMIC_TEST)
            state = KINAMIC_TEST;
        else if (exp_state == LQR_CTRL)
            state = LQR_CTRL;
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
        leg_calc_->forward_kinematics(Eigen::Vector2d(lf->state.rad, lb->state.rad), pos);
        debug_log("left_leg_length=%lf", pos[0]);
        if (exp_state == IDEL) {
            state = IDEL;
        } else if (exp_state == VMC_TEST)
            state = VMC_TEST;
        else if (exp_state == LQR_CTRL)
            state = LQR_CTRL;
    } else if (state == VMC_TEST) {
        Eigen::Vector2d left_leg_pos, right_leg_pos, left_leg_vel, right_leg_vel, left_leg_force, right_leg_force;
        auto left_joint_pos  = Eigen::Vector2d(lf->state.rad, lb->state.rad);
        auto right_joint_pos = Eigen::Vector2d(rf->state.rad, rb->state.rad);

        leg_calc_->forward_kinematics(left_joint_pos, left_leg_pos);
        leg_calc_->forward_velocity(left_joint_pos, Eigen::Vector2d(lf->state.vel, lb->state.vel), left_leg_vel);
        leg_calc_->forward_dynamics(left_joint_pos, Eigen::Vector2d(lf->state.toqeue, lb->state.toqeue), left_leg_force);

        leg_calc_->forward_kinematics(right_joint_pos, right_leg_pos);
        leg_calc_->forward_velocity(right_joint_pos, Eigen::Vector2d(rf->state.vel, rb->state.vel), right_leg_vel);
        leg_calc_->forward_dynamics(right_joint_pos, Eigen::Vector2d(rf->state.toqeue, rb->state.toqeue), right_leg_force);

        Eigen::Vector2d left_leg_exp_force, right_leg_exp_force, left_joint_exp_torque, right_joint_exp_torque;
        left_leg_exp_force[0]  = left_leg_length.update(left_leg_pos[0], left_leg_vel[0], 0.23f);
        right_leg_exp_force[0] = left_leg_length.update(right_leg_pos[0], right_leg_vel[0], 0.23f);
        left_leg_exp_force[1] = right_leg_exp_force[1] = 0.0f;

        leg_calc_->inverse_dynamics(left_joint_pos, left_leg_exp_force, left_joint_exp_torque);
        leg_calc_->inverse_dynamics(right_joint_pos, right_leg_exp_force, right_joint_exp_torque);

        debug_log("left_length:%lf,right_length:%lf", left_leg_pos[0], right_leg_pos[0]);

        lf->set_command(0.0f, 0.0f, left_joint_exp_torque[0], 0.0f, 0.0f);
        rf->set_command(0.0f, 0.0f, right_joint_exp_torque[0], 0.0f, 0.0f);
        lb->set_command(0.0f, 0.0f, left_joint_exp_torque[1], 0.0f, 0.0f);
        rb->set_command(0.0f, 0.0f, right_joint_exp_torque[1], 0.0f, 0.0f);

        if (exp_state == IDEL) {
            state = IDEL;
        } else if (exp_state == KINAMIC_TEST)
            state = KINAMIC_TEST;
        else if (exp_state == LQR_CTRL)
            state = LQR_CTRL;
    } else if (state == READY_STAND) // 到达准备站立状态1
    {
        Eigen::Vector2d reset_leg_pos(0.1, 0.3);
        Eigen::Vector2d joint_pos;
        leg_calc_->inverse_kinematics(reset_leg_pos, joint_pos);
        if (!reset_traj_generated) {
            sub_stage            = 0;
            reset_traj_generated = true;
        }


        // TODO:检查机器人是否翻倒，如果翻倒，还需要让腿向前摆一次让机器人大致正过来
        if (sub_stage == 0) {
            // TODO:判定机器人大致朝向
            sub_stage = 5; // 直接进行小板凳起立
            debug_log("enter stage5");

        }
        // TODO:中间的是让机器人正过来的步骤
        else if (sub_stage == 5) {
            Eigen::Vector2d cur_left_joint_pos  = Eigen::Vector2d(lf->state.rad, lb->state.rad);
            Eigen::Vector2d cur_right_joint_pos = Eigen::Vector2d(rf->state.rad, rb->state.rad);
            left_leg_slope.generate_traj(
                cur_left_joint_pos, joint_pos, std::max((cur_left_joint_pos - joint_pos).norm(), 0.01) / 0.7); // 加速度为0.7rad/s
            right_leg_slope.generate_traj(cur_right_joint_pos, joint_pos, std::max((cur_right_joint_pos - joint_pos).norm(), 0.01) / 0.7);
            time_point = ms;
            sub_stage  = 6;
        } else if (sub_stage == 6) {
            double duration = (ms - time_point) * 0.001;
            left_leg_slope.get_target(joint_pos, duration);
            lf->set_command(joint_pos[0], 0.0f, 0.0f, param.motor_kp, param.motor_kd);
            lb->set_command(joint_pos[1], 0.0f, 0.0f, param.motor_kp, param.motor_kd);
            bool ret = right_leg_slope.get_target(joint_pos, duration);
            rf->set_command(joint_pos[0], 0.0f, 0.0f, param.motor_kp, param.motor_kd);
            rb->set_command(joint_pos[1], 0.0f, 0.0f, param.motor_kp, param.motor_kd);
            if (!ret)                                                                                          // 复位完成，切到下一个状态
                sub_stage = 7;
        } else if (sub_stage == 7) {
            if (imu->angular_velocity.norm() < 0.1) {
                ref_pos   = 0.5 * (lw->state.rad + rw->state.rad) * Rw;                                        // 重置参考量
                ref_phi   = yaw_unwrapped_ - yaw_reference_;
                exp_state = state    = LQR_CTRL;
                reset_traj_generated = false;
            }
        }
    } else if (state == LQR_CTRL || state == LQR_JUMP) {
        // IMU acceleration is treated as specific force:
        // rotate it to world frame, restore linear acceleration by adding gravity,
        // then project it onto the horizontal forward direction.
        const double ax = forward_linear_acceleration(orientation, acceleration, yaw);

        if (std::abs(roll) > 0.35f || std::abs(pitch) > 0.3f) {
            state = READY_STAND;
            return true;
        }

        // 计算腿的位置，速度，受力
        Eigen::Vector2d left_joint_pos(lf->state.rad, lb->state.rad);
        Eigen::Vector2d right_joint_pos(rf->state.rad, rb->state.rad);
        Eigen::Vector2d left_leg_pos, right_leg_pos, left_leg_vel, right_leg_vel, left_leg_force, right_leg_force;
        leg_calc_->forward_kinematics(left_joint_pos, left_leg_pos);
        leg_calc_->forward_kinematics(right_joint_pos, right_leg_pos);
        leg_calc_->forward_velocity(left_joint_pos, Eigen::Vector2d(lf->state.vel, lb->state.vel), left_leg_vel);
        leg_calc_->forward_velocity(right_joint_pos, Eigen::Vector2d(rf->state.vel, rb->state.vel), right_leg_vel);
        leg_calc_->forward_dynamics(left_joint_pos, Eigen::Vector2d(lf->state.toqeue, lb->state.toqeue), left_leg_force);
        leg_calc_->forward_dynamics(right_joint_pos, Eigen::Vector2d(rf->state.toqeue, rb->state.toqeue), right_leg_force);

        const double left_leg_force_filtered  = left_leg_force_filter_.update(left_leg_force[0]);
        const double right_leg_force_filtered = right_leg_force_filter_.update(right_leg_force[0]);

        // 填写状态向量
        //  x = sp.Matrix([
        //      s, ds,
        //      phi, dphi,
        //      thll, dthll,
        //      thlr, dthlr,
        //      thb, dthb])

        Eigen::Vector<double, 10> x, xd;
        xd.setZero();                                  // 参考输入
        const double wheel_position = 0.5 * (lw->state.rad + rw->state.rad) * Rw;
        double s                    = wheel_position;
        double ds                   = 0.5 * (lw->state.vel + rw->state.vel) * Rw;
        const double phi            = yaw_unwrapped_ - yaw_reference_;
        double dphi                 = angular_velocity[2];
        const double thb            = pitch;
        double dthb                 = angular_velocity[1];
        const double thll           = wrap_to_pi(thb + left_leg_pos[1]);
        const double thlr           = wrap_to_pi(thb + right_leg_pos[1]);
        double dthll                = dthb + left_leg_vel[1];
        double dthlr                = dthb + right_leg_vel[1];

        ds    = ds_filter_.update(ds);
        dphi  = dphi_filter_.update(dphi);
        dthll = dthll_filter_.update(dthll);
        dthlr = dthlr_filter_.update(dthlr);
        dthb  = dthb_filter_.update(dthb);



        wheel_kf.update(Eigen::Matrix<double, 1, 1>::Constant(wheel_position), Eigen::Matrix<double, 1, 1>::Constant(ax));
        Eigen::Vector2d wheel_state= wheel_kf.state();


        if (std::abs(wheel_state[0] - ref_pos) > 5.0f) // 防止位置误差过大导致控制器发散
            ref_pos = wheel_state[0] + (ref_pos-wheel_state[0]) / std::abs(wheel_state[0] - ref_pos) * 5.0f;

        update_body_accel_observation(body_kf, left_leg_pos[0], right_leg_pos[0]);
        const BodyKf::MeasControlMat body_kf_d = make_body_kf_d(body_kf, left_leg_pos[0], right_leg_pos[0]);
        const BodyKf::ControlVec body_kf_u =
            make_body_kf_input_from_motor_torque(lw->state.toqeue, rw->state.toqeue, left_leg_force, right_leg_force);
        BodyKf::MeasVec body_kf_y;
        body_kf_y << s, ds, phi, dphi, thll, dthll, thlr, dthlr, thb, dthb, ax;
        if (!body_kf_initialized_) {
            body_kf.reset(make_body_kf_state(s, ds, phi, dphi, thll, dthll, thlr, dthlr, thb, dthb), make_body_kf_p0());
            body_kf_initialized_ = true;
        }
        body_kf.update(body_kf_y, body_kf_u, kControlDt, body_kf_d);
        anchor_body_kf_direct_positions(body_kf, x, s, phi, thll, thlr, thb);

        debug_log(
            "x:[%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf]",x[0],x[1],x[2],x[3],x[4],x[5],x[6],x[7],x[8],x[9]);
        debug_log(
            "raw:[%lf,%lf,%lf,%lf,%lf] rawvel:[%lf,%lf,%lf,%lf,%lf]", s, phi, thll, thlr, thb, ds, dphi, dthll, dthlr, dthb);

        // 填写参考输入
        ref_pos += ref_vel * kControlDt;
        ref_phi += ref_omega * kControlDt;
        xd[0] = ref_pos;
        xd[1] = ref_vel;
        xd[2] = ref_phi;
        xd[3] = ref_omega;

        // u = sp.Matrix([Twl, Twr, Tbl, Tbr])
        u = Eigen::Vector4d::Zero(); // LQR控制律

        const auto kfx = body_kf.state();
const auto f = body_kf.A * kfx + body_kf.B * body_kf_u;
const double ax_hat =
    (body_kf.H.row(10) * kfx)(0) + (body_kf_d.row(10) * body_kf_u)(0);

debug_log(
    "kfpos:[%lf,%lf,%lf,%lf,%lf] kfvel:[%lf,%lf,%lf,%lf,%lf] ax:%lf ax_hat:%lf ax_err:%lf fveldot:[%lf,%lf,%lf,%lf,%lf] u:[%lf,%lf,%lf,%lf]",
    kfx[0], kfx[2], kfx[4], kfx[6], kfx[8],
    kfx[1], kfx[3], kfx[5], kfx[7], kfx[9],
    ax, ax_hat, ax - ax_hat,
    f[1], f[3], f[5], f[7], f[9],
    body_kf_u[0], body_kf_u[1], body_kf_u[2], body_kf_u[3]);



        const bool left_in_contact  = left_leg_force_filtered > 25.0;
        const bool right_in_contact = right_leg_force_filtered > 25.0;

        // if (lqr_disturbance_observer_ != nullptr && lqr_disturbance_observer_->observer_gain_valid()) {
        //     if (!lqr_disturbance_observer_initialized_) {
        //         lqr_disturbance_observer_->reset(x, Eigen::Vector4d::Zero());
        //         lqr_disturbance_.setZero();
        //         previous_lqr_command_.setZero();
        //         lqr_disturbance_observer_initialized_ = true;
        //     } else if (!lqr_disturbance_observer_->update(x, previous_lqr_command_, kControlDt, lqr_disturbance_)) {
        //         lqr_disturbance_observer_->reset(x, Eigen::Vector4d::Zero());
        //         lqr_disturbance_.setZero();
        //         previous_lqr_command_.setZero();
        //         lqr_disturbance_observer_initialized_ = true;
        //     }
        // } else {
        //     lqr_disturbance_.setZero();
        //     previous_lqr_command_.setZero();
        //     lqr_disturbance_observer_initialized_ = false;
        // }

        // debug_log(
        //     "left_length:%lf,right_length:%lf\nleft_force:%lf,right_force:%lf\n,disturbance_:[%lf,%lf,%lf,%lf]", left_leg_pos[0],
        //     right_leg_pos[0], left_leg_force_filtered, right_leg_force_filtered, lqr_disturbance_[0], lqr_disturbance_[1],
        //     lqr_disturbance_[2], lqr_disturbance_[3]);

        if (left_in_contact && right_in_contact)         // 两轮接地
        {
            u = K * (xd - x);
        } else if (left_in_contact && !right_in_contact) // 左轮接地
        {
            u = K_right_air * (xd - x);
        } else if (!left_in_contact && right_in_contact) // 右轮接地
        {
            u = K_left_air * (xd - x);
        } else                                           // 左右轮都没有接地
        {
            u = K_air * (xd - x);
        }

        // u -= lqr_disturbance_;
        // if (!u.allFinite()) {
        //     return false;
        // }
        previous_lqr_command_ = u;


        double left_action_ff = 0.0, right_action_ff = 0.0;
        if (state == LQR_JUMP) {
            if (sub_stage == 0)                 // 第一步
            {
                stage_time_tick = ms;
                sub_stage       = 1;
                ref_height      = 0.22f;        // 降到最低位置
            } else if (sub_stage == 1) {
                if (ms - stage_time_tick > 500) // 等待500ms到达最低位置
                {
                    sub_stage = 2;
                }
            } else if (sub_stage == 2)          // 开始跳跃，并等待腿长足够长
            {
                ref_height      = 0.4f;
                left_action_ff  = 70.0f;
                right_action_ff = 70.0f;
                if ((left_leg_pos[0] + right_leg_pos[0]) * 0.5 > 0.4) {
                    sub_stage = 3;
                }
            } else if (sub_stage == 3)          // 收腿
            {
                left_action_ff  = 0.0f;
                right_action_ff = 0.0f;
                ref_height      = 0.21f;
                stage_time_tick = ms;
                sub_stage       = 4;
            } else if (sub_stage == 4) {
                if (ms - stage_time_tick > 500) // 等待500ms展开腿部准备落地缓冲
                {
                    sub_stage = 5;
                }
            } else if (sub_stage == 5) {
                ref_height = 0.25f;
            }
        }

        constexpr double body_width        = 0.34;
        const double leg_height_roll       = std::atan2(right_leg_pos[0] - left_leg_pos[0], body_width);
        const double roll_error            = wrap_to_pi(-roll - leg_height_roll);
        const double leg_length_difference = body_width * std::tan(roll_error);
        const float left_ref_height =
            static_cast<float>(std::clamp(static_cast<double>(ref_height) + 0.5 * leg_length_difference, 0.25, 0.4));
        const float right_ref_height =
            static_cast<float>(std::clamp(static_cast<double>(ref_height) - 0.5 * leg_length_difference, 0.25, 0.4));

        const double roll_rate     = roll_rate_filter_.update(angular_velocity[0]);
        const double roll_pd_force = roll_pd.update(roll, roll_rate, 0.0f);

        const double safe_body_width    = std::max(std::abs(BodyWidth), 1.0e-6);
        const double average_leg_length = 0.5 * (left_leg_pos[0] + right_leg_pos[0]);
        const double com_height         = average_leg_length + Rw + BaseLinkComHeight;
        // Use commanded tangential speed and yaw rate for feedforward so IMU
        // and wheel-speed estimation noise does not directly modulate leg force.
        const double centripetal_force  = Mb * wheel_state[1] * imu->angular_velocity.z();
        const double centripetal_torque = centripetal_force * com_height;
        const double centrifugal_force_ff_raw =
            left_in_contact && right_in_contact ? CentrifugalForceFfGain * centripetal_torque / safe_body_width : 0.0;
        const double centrifugal_force_ff = std::clamp(centrifugal_force_ff_raw, -CentrifugalForceFfLimit, CentrifugalForceFfLimit);

        // 腿长PD控制器
        Eigen::Vector2d left_leg_exp_force, right_leg_exp_force, left_joint_torque, right_joint_torque;
        left_leg_exp_force[1]   = u[2];
        right_leg_exp_force[1]  = u[3];
        double left_gravity_ff  = 0.0;
        double right_gravity_ff = 0.0;
        if (left_in_contact && right_in_contact) {
            left_gravity_ff  = 0.5 * Mb * 9.8 * std::cos(thll);
            right_gravity_ff = 0.5 * Mb * 9.8 * std::cos(thlr);
        } else if (left_in_contact) {
            left_gravity_ff = 0.5 * Mb * 9.8 * std::cos(thll);
        } else if (right_in_contact) {
            right_gravity_ff = 0.5 * Mb * 9.8 * std::cos(thlr);
        }
        left_leg_exp_force[0] = left_leg_length.update(left_leg_pos[0], left_leg_vel[0], left_ref_height) + left_gravity_ff
                              - centrifugal_force_ff + roll_pd_force + left_action_ff;
        right_leg_exp_force[0] = right_leg_length.update(right_leg_pos[0], right_leg_vel[0], right_ref_height) + right_gravity_ff
                               + centrifugal_force_ff - roll_pd_force + right_action_ff;
        leg_calc_->inverse_dynamics(left_joint_pos, left_leg_exp_force, left_joint_torque);
        leg_calc_->inverse_dynamics(right_joint_pos, right_leg_exp_force, right_joint_torque);

        lf->set_command(0.0f, 1.0f, left_joint_torque[0], 0.0f, 0.0f);
        lb->set_command(0.0f, 1.0f, left_joint_torque[1], 0.0f, 0.0f);
        rf->set_command(0.0f, 1.0f, right_joint_torque[0], 0.0f, 0.0f);
        rb->set_command(0.0f, 1.0f, right_joint_torque[1], 0.0f, 0.0f);
        lw->set_command(0.0f, 0.0f, u[0], 0.0f, 0.0f);
        rw->set_command(0.0f, 0.0f, u[1], 0.0f, 0.0f);


        if (exp_state == KINAMIC_TEST)
            state = KINAMIC_TEST;
        else if (exp_state == IDEL)
            state = IDEL;
        else if (state == LQR_JUMP && exp_state == LQR_CTRL) // 从跳跃LQR前进切换
        {
            sub_stage = 0;
            state     = LQR_CTRL;
        } else if (exp_state == LQR_JUMP)                    // 从LQR前进向跳跃切换
        {
            state = exp_state;
        } else if (
            exp_state == LQR_STEP && (!finished_step)) { // 没有完成过上台阶，才能切入上台阶，如果刚刚从上台阶状态返回，那么不继续切入上台阶
            sub_stage = 0;
            state     = LQR_STEP;
        } else if (exp_state == LQR_CTRL) {
            finished_step = false;
        }
    } else if (state == LQR_STEP) {
        // IMU acceleration is treated as specific force:
        // rotate it to world frame, restore linear acceleration by adding gravity,
        // then project it onto the horizontal forward direction.
        const double ax = forward_linear_acceleration(orientation, acceleration, yaw);

        // 计算腿的位置，速度，受力
        Eigen::Vector2d left_joint_pos(lf->state.rad, lb->state.rad);
        Eigen::Vector2d right_joint_pos(rf->state.rad, rb->state.rad);
        Eigen::Vector2d left_leg_pos, right_leg_pos, left_leg_vel, right_leg_vel, left_leg_force, right_leg_force;
        leg_calc_->forward_kinematics(left_joint_pos, left_leg_pos);
        leg_calc_->forward_kinematics(right_joint_pos, right_leg_pos);
        leg_calc_->forward_velocity(left_joint_pos, Eigen::Vector2d(lf->state.vel, lb->state.vel), left_leg_vel);
        leg_calc_->forward_velocity(right_joint_pos, Eigen::Vector2d(rf->state.vel, rb->state.vel), right_leg_vel);
        leg_calc_->forward_dynamics(left_joint_pos, Eigen::Vector2d(lf->state.toqeue, lb->state.toqeue), left_leg_force);
        leg_calc_->forward_dynamics(right_joint_pos, Eigen::Vector2d(rf->state.toqeue, rb->state.toqeue), right_leg_force);

        const double left_leg_force_filtered  = left_leg_force_filter_.update(left_leg_force[0]);
        const double right_leg_force_filtered = right_leg_force_filter_.update(right_leg_force[0]);

        // 填写状态向量
        //  x = sp.Matrix([
        //      s, ds,
        //      phi, dphi,
        //      thll, dthll,
        //      thlr, dthlr,
        //      thb, dthb])

        Eigen::Vector<double, 10> x, xd;
        xd.setZero();              // 参考输入
        const double wheel_position = 0.5 * (lw->state.rad + rw->state.rad) * Rw;
        double s                    = wheel_position;
        double ds                   = 0.5 * (lw->state.vel + rw->state.vel) * Rw;
        const double phi            = yaw_unwrapped_ - yaw_reference_;
        double dphi                 = angular_velocity[2];
        const double thb            = pitch;
        double dthb                 = angular_velocity[1];
        const double thll           = wrap_to_pi(thb + left_leg_pos[1]);
        const double thlr           = wrap_to_pi(thb + right_leg_pos[1]);
        double dthll                = dthb + left_leg_vel[1];
        double dthlr                = dthb + right_leg_vel[1];

        ds    = ds_filter_.update(ds);
        dphi  = dphi_filter_.update(dphi);
        dthll = dthll_filter_.update(dthll);
        dthlr = dthlr_filter_.update(dthlr);
        dthb  = dthb_filter_.update(dthb);


        if (sub_stage < 3) {
            ref_pos += ref_vel * kControlDt;
            ref_phi += ref_omega * kControlDt;
        }
        xd[0] = ref_pos;
        xd[1] = ref_vel;
        xd[2] = ref_phi;
        xd[3] = ref_omega;


        wheel_kf.update(Eigen::Matrix<double, 1, 1>::Constant(wheel_position), Eigen::Matrix<double, 1, 1>::Constant(ax));
        Eigen::Vector2d wheel_state = wheel_kf.state();


        double wheel_exp_vel = 0.0;
        if (sub_stage == 0) {
            ref_height      = 0.37f;
            stage_time_tick = ms;
            sub_stage       = 1;
        } else if (sub_stage == 1) // 防止站立过程中的力矩干扰撞墙判断
        {
            if (ms - stage_time_tick > 800)
                sub_stage = 2;
        } else if (sub_stage == 2) // 执行LQR控制，同时检查是否撞墙
        {
            debug_log("pitch=%lf", pitch);
            if (pitch > 0.05) {
                leg_calc_->forward_kinematics(
                    Eigen::Vector2d((lf->state.rad + rf->state.rad) * 0.5, (lb->state.rad + rb->state.rad) * 0.5), step_leg_exp_pos);
                debug_log("switch to stage3,pos=(%lf,%lf)", step_leg_exp_pos[0], step_leg_exp_pos[1]);
                sub_stage       = 3;
                stage_time_tick = ms;
                final_leg_omega = std::max(0.5 * (dthll + dthlr), 1.0);
            }
        } else if (sub_stage == 3) // 腿向后伸，轮子向前转
        {
            constexpr double k_vel = 2.0;
            wheel_exp_vel          = 10.0;
            step_leg_exp_pos[0]    = 0.4;
            step_leg_exp_pos[1] += k_vel * final_leg_omega * 0.002;
            if (step_leg_exp_pos[1] > 1.4) {
                sub_stage = 6;
                debug_log("switch to stage4");
                stage_time_tick = ms;
            }
            // } else if (sub_stage == 4)                         // 持续前进1s
            // {
            //     wheel_exp_vel = 4.0;
            //     if (ms - stage_time_tick > 1000) {
            //         debug_log("switch to stage5");
            //         sub_stage = 5;
            //     }
            // } else if (sub_stage == 5)                         // 腿向后转起来
            // {
            //     wheel_exp_vel = 0.0;
            //     step_leg_exp_pos[1] += 0.001;
            //     if (step_leg_exp_pos[1] > 1.5) {
            //         sub_stage = 6;
            //         debug_log("switch to stage6");
            //     }
        } else if (sub_stage == 6)                         // 腿转平后，往回缩
        {
            step_leg_exp_pos[0] -= 0.0005;
            if (step_leg_exp_pos[0] < 0.21) {
                debug_log("switch to stage7");
                stage_time_tick = ms;
                sub_stage       = 7;
            }
        } else if (sub_stage == 7)                         // 切入准备起立状态
        {
            wheel_exp_vel = 2.0;
            step_leg_exp_pos[1] -= 0.0005;
            if (step_leg_exp_pos[1] < 0.1) {
                sub_stage       = 8;
                stage_time_tick = ms;
                debug_log("switch to stage8");
            }
        } else if (sub_stage == 8) {
            wheel_exp_vel = 2.0;
            if (ms - stage_time_tick > 1000)               // 前进1s
            {
                sub_stage = 9;
            }
        } else if (sub_stage == 9) {
            wheel_exp_vel = 0.0;
            if (exp_state == LQR_CTRL)
                state = LQR_CTRL;
            finished_step = true;
        }




        if (sub_stage < 3) {                               // 小于3是执行LQR控制
            if (std::abs(wheel_state[0] - ref_pos) > 1.5f) // 防止位置误差过大导致控制器发散
                wheel_state[0] = ref_pos + (wheel_state[0] - ref_pos) / std::abs(wheel_state[0] - ref_pos) * 1.5f;

            update_body_accel_observation(body_kf, left_leg_pos[0], right_leg_pos[0]);
            const BodyKf::MeasControlMat body_kf_d = make_body_kf_d(body_kf, left_leg_pos[0], right_leg_pos[0]);
            const BodyKf::ControlVec body_kf_u =
                make_body_kf_input_from_motor_torque(lw->state.toqeue, rw->state.toqeue, left_leg_force, right_leg_force);
            BodyKf::MeasVec body_kf_y;
            body_kf_y << s, ds, phi, dphi, thll, dthll, thlr, dthlr, thb, dthb, ax;
            if (!body_kf_initialized_) {
                body_kf.reset(make_body_kf_state(s, ds, phi, dphi, thll, dthll, thlr, dthlr, thb, dthb), make_body_kf_p0());
                body_kf_initialized_ = true;
            }
            body_kf.update(body_kf_y, body_kf_u, kControlDt, body_kf_d);
            anchor_body_kf_direct_positions(body_kf, x, s, phi, thll, thlr, thb);

            // 填写参考输入


            u = K * (xd - x);
            previous_lqr_command_ = u;

            constexpr double body_width        = 0.34;
            const double leg_height_roll       = std::atan2(right_leg_pos[0] - left_leg_pos[0], body_width);
            const double roll_error            = wrap_to_pi(-roll - leg_height_roll);
            const double leg_length_difference = body_width * std::tan(roll_error);
            const float left_ref_height =
                static_cast<float>(std::clamp(static_cast<double>(ref_height) + 0.5 * leg_length_difference, 0.25, 0.4));
            const float right_ref_height =
                static_cast<float>(std::clamp(static_cast<double>(ref_height) - 0.5 * leg_length_difference, 0.25, 0.4));

            const double roll_rate     = roll_rate_filter_.update(angular_velocity[0]);
            const double roll_pd_force = roll_pd.update(roll, roll_rate, 0.0f);

            const double safe_body_width    = std::max(std::abs(BodyWidth), 1.0e-6);
            const double average_leg_length = 0.5 * (left_leg_pos[0] + right_leg_pos[0]);
            const double com_height         = average_leg_length + Rw + BaseLinkComHeight;

            // 腿长PD控制器
            Eigen::Vector2d left_leg_exp_force, right_leg_exp_force, left_joint_torque, right_joint_torque;
            left_leg_exp_force[1]   = u[2];
            right_leg_exp_force[1]  = u[3];
            double left_gravity_ff  = 0.5 * Mb * 9.8 * std::cos(thll);
            double right_gravity_ff = 0.5 * Mb * 9.8 * std::cos(thlr);

            left_leg_exp_force[0] =
                left_leg_length.update(left_leg_pos[0], left_leg_vel[0], left_ref_height) + left_gravity_ff + roll_pd_force;
            right_leg_exp_force[0] =
                right_leg_length.update(right_leg_pos[0], right_leg_vel[0], right_ref_height) + right_gravity_ff - roll_pd_force;
            leg_calc_->inverse_dynamics(left_joint_pos, left_leg_exp_force, left_joint_torque);
            leg_calc_->inverse_dynamics(right_joint_pos, right_leg_exp_force, right_joint_torque);

            lf->set_command(0.0f, 1.0f, left_joint_torque[0], 0.0f, 0.0f);
            lb->set_command(0.0f, 1.0f, left_joint_torque[1], 0.0f, 0.0f);
            rf->set_command(0.0f, 1.0f, right_joint_torque[0], 0.0f, 0.0f);
            rb->set_command(0.0f, 1.0f, right_joint_torque[1], 0.0f, 0.0f);
            lw->set_command(0.0f, 0.0f, u[0], 0.0f, 0.0f);
            rw->set_command(0.0f, 0.0f, u[1], 0.0f, 0.0f);
        } else {
            Eigen::Vector2d joint_pos = Eigen::Vector2d(lf->state.rad + rf->state.rad, lb->state.rad + rb->state.rad);
            leg_calc_->inverse_kinematics(step_leg_exp_pos, joint_pos);

            lf->set_command(joint_pos[0], 0.0f, 0.0, param.motor_kp, param.motor_kd);
            lb->set_command(joint_pos[1], 0.0f, 0.0, param.motor_kp, param.motor_kd);
            rf->set_command(joint_pos[0], 0.0f, 0.0, param.motor_kp, param.motor_kd);
            rb->set_command(joint_pos[1], 0.0f, 0.0, param.motor_kp, param.motor_kd);
            lw->set_command(0.0f, wheel_exp_vel, 0.0, 0.0f, 0.3f);
            rw->set_command(0.0f, wheel_exp_vel, 0.0, 0.0f, 0.3f);
        }



        if (exp_state == LQR_CTRL) {
            sub_stage = 0;
            state     = LQR_CTRL;
        }
    }
    return true;
}


void ControllerAT::input(float velocity, float omega, float height, int mode) {
    ref_vel   = velocity;
    ref_omega = omega;

    if (exp_state == LQR_CTRL) // LQR行走时可自由控制高度
        ref_height = height;

    if (mode == 1) {
        exp_state = IDEL;
    } else if (mode == 2) {
        exp_state = KINAMIC_TEST;
    } else if (mode == 3) {
        exp_state = VMC_TEST;
    } else if (mode == 4) {
        exp_state = LQR_CTRL;
    } else if (mode == 5) {
        exp_state = LQR_STEP;
    } else if (mode == 6) {
        exp_state = LQR_JUMP;
    }
}
