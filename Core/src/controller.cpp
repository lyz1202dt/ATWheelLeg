#include "controller.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kStateSwitchDelay = 0.3;
constexpr double kPitchLimit = 0.3;
constexpr double kContactForceThreshold = 8.0;
constexpr double kHipTorqueLimit = 12.0;
constexpr double kWheelTorqueLimit = 2.0;

using StateVector = Eigen::Matrix<double, 6, 1>;

constexpr LqrGainDebuger::StateWeight kDefaultLqrQDiag = {
    1.0, 1.0, 10.0, 1.0, 1.0, 1.0};
constexpr LqrGainDebuger::InputWeight kDefaultLqrRDiag = {1.0, 1.0};

}  // namespace

Controller::Controller(IMUBase* imu_in,
                       Motor* lf_in,
                       Motor* rf_in,
                       Motor* lb_in,
                       Motor* rb_in,
                       Motor* lw_in,
                       Motor* rw_in)
    : imu(imu_in),
      lf(lf_in),
      rf(rf_in),
      lb(lb_in),
      rb(rb_in),
      lw(lw_in),
      rw(rw_in),
      leg_(kHipHalfDistance, kUpperLinkLength, kLowerLinkLength)
{
    gain_debuger_.bind_k_mat(&ground_k_mat_, &air_k_mat_);
    std::string error;
    (void)gain_debuger_.solve_lqr_gain(
        kDefaultLqrQDiag, kDefaultLqrRDiag, error);
}

bool Controller::set_params(const Params& params)
{
    const std::array<double, 12> values = {
        params.body_width,
        params.base_link_com_height,
        params.centrifugal_accel_filter_alpha,
        params.centrifugal_force_ff_gain,
        params.centrifugal_force_ff_limit,
        params.leg_exp_length,
        params.vmc_kp,
        params.vmc_kd,
        params.leg_angle_diff_kp,
        params.leg_angle_diff_kd,
        params.wheel_diff_kp,
        params.wheel_diff_ki,
    };
    for (const double value : values) {
        if (!std::isfinite(value)) {
            return false;
        }
    }
    if (std::abs(params.body_width) < 1.0e-6 ||
        params.centrifugal_accel_filter_alpha < 0.0 ||
        params.centrifugal_accel_filter_alpha > 1.0 ||
        params.centrifugal_force_ff_limit < 0.0 ||
        params.leg_exp_length <= 0.0 ||
        params.vmc_kp < 0.0 ||
        params.vmc_kd < 0.0 ||
        params.leg_angle_diff_kp < 0.0 ||
        params.leg_angle_diff_kd < 0.0 ||
        params.wheel_diff_kp < 0.0 ||
        params.wheel_diff_ki < 0.0) {
        return false;
    }

    params_ = params;
    expected_height_ = params_.leg_exp_length;
    return true;
}

bool Controller::update_lqr_gain(
    const LqrGainDebuger::StateWeight& q_diag,
    const LqrGainDebuger::InputWeight& r_diag,
    std::string& error)
{
    return gain_debuger_.solve_lqr_gain(q_diag, r_diag, error);
}

void Controller::input(float velocity, float omega, float height, int mode)
{
    if (std::isfinite(velocity)) {
        expected_velocity_ = static_cast<double>(velocity);
    }
    if (std::isfinite(omega)) {
        expected_omega_ = static_cast<double>(omega);
    }
    if (std::isfinite(height) && height > 0.0F) {
        expected_height_ = static_cast<double>(height);
    }
    requested_mode_ = (mode >= 0 && mode <= 3) ? mode : 0;
}

Eigen::Vector2d Controller::lqr_control() const
{
    return lqr_control_;
}

bool Controller::update(float dt)
{
    if (!std::isfinite(dt) || dt <= 0.0F) {
        dt = 0.001F;
    }

    if (imu == nullptr || !imu->is_ready()) {
        set_safe_commands();
        return false;
    }

    Eigen::Quaternionf orientation = Eigen::Quaternionf::Identity();
    Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
    bool imu_state_valid = false;
    imu->lock_memory();
    orientation = imu->orientation;
    angular_velocity = imu->angular_velocity;
    acceleration = imu->acceleration;
    imu_state_valid = imu->state_valid_;
    imu->unlock_memory();
    if (!imu_state_valid) {
        set_safe_commands();
        return false;
    }

    LegState left_leg;
    LegState right_leg;
    if (!read_leg_state(lf, lb, left_leg) ||
        !read_leg_state(rf, rb, right_leg) ||
        lw == nullptr || rw == nullptr) {
        set_safe_commands();
        return false;
    }

    double yaw = 0.0;
    double pitch = 0.0;
    double roll = 0.0;
    if (!orientation_yaw_pitch_roll(orientation, yaw, pitch, roll)) {
        set_safe_commands();
        return false;
    }

    const Eigen::Matrix3d body_to_world =
        Eigen::Quaterniond(static_cast<double>(orientation.w()),
                           static_cast<double>(orientation.x()),
                           static_cast<double>(orientation.y()),
                           static_cast<double>(orientation.z()))
            .normalized()
            .toRotationMatrix();
    const Eigen::Matrix3d level_to_world =
        Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    const Eigen::Vector3d level_acceleration =
        level_to_world.transpose() * body_to_world * acceleration;
    const double lateral_acceleration = low_pass_filter(
        level_acceleration.y(),
        params_.centrifugal_accel_filter_alpha,
        filtered_lateral_acceleration_,
        lateral_acceleration_filter_initialized_);

    const double left_normal_force = left_leg.leg_effort[0];
    const double right_normal_force = right_leg.leg_effort[0];
    update_state_machine(pitch, left_normal_force, right_normal_force, dt);

    expected_position_ += expected_velocity_ * static_cast<double>(dt);

    const double leg_angle =
        0.5 * (left_leg.leg_position[1] + right_leg.leg_position[1]);
    const double leg_angle_velocity =
        0.5 * (left_leg.leg_velocity[1] + right_leg.leg_velocity[1]);
    const double average_leg_length =
        0.5 * (left_leg.leg_position[0] + right_leg.leg_position[0]);
    const double x =
        0.5 * (static_cast<double>(lw->state.rad) +
               static_cast<double>(rw->state.rad)) *
        kWheelRadius;
    double dx =
        0.5 * (static_cast<double>(lw->state.vel) +
               static_cast<double>(rw->state.vel)) *
        kWheelRadius;
    const double theta = normalize_angle(pitch + leg_angle);
    double dtheta = angular_velocity.y() + leg_angle_velocity;
    const double phi = -pitch;
    double dphi = -angular_velocity.y();

    dx = low_pass_filter(dx, 0.07, filtered_dx_, dx_filter_initialized_);
    dtheta =
        low_pass_filter(dtheta, 0.7, filtered_dtheta_, dtheta_filter_initialized_);
    dphi = low_pass_filter(dphi, 0.7, filtered_dphi_, dphi_filter_initialized_);

    if (!std::isfinite(x) || !std::isfinite(dx) ||
        !std::isfinite(theta) || !std::isfinite(dtheta) ||
        !std::isfinite(phi) || !std::isfinite(dphi) ||
        !std::isfinite(average_leg_length)) {
        set_safe_commands();
        return false;
    }

    StateVector state;
    state << x, dx, theta, dtheta, phi, dphi;
    StateVector expected_state = StateVector::Zero();
    const double position_error = expected_position_ - x;
    expected_state[0] = std::clamp(position_error, -5.0, 5.0) + x;

    lqr_control_.setZero();
    if (state_ == State::Balance) {
        lqr_control_ = ground_k_mat_ * (expected_state - state);
    } else if (state_ == State::Airborne) {
        lqr_control_ = -air_k_mat_ * state;
    }

    const double safe_body_width = std::max(std::abs(params_.body_width), 1.0e-6);
    const double leg_height_roll =
        std::atan2(right_leg.leg_position[0] - left_leg.leg_position[0],
                   safe_body_width);
    const double combined_roll = normalize_angle(roll + leg_height_roll);
    const double roll_error = normalize_angle(expected_roll_ - combined_roll);
    const double leg_length_difference =
        safe_body_width * std::tan(roll_error);
    const double left_expected_length = std::clamp(
        expected_height_ + 0.5 * leg_length_difference, 0.19, 0.34);
    const double right_expected_length = std::clamp(
        expected_height_ - 0.5 * leg_length_difference, 0.19, 0.34);

    const double vmc_mass_component =
        state_ == State::Balance
            ? kBaselinkMass * 0.5 * 9.8 * std::cos(theta)
            : 0.0;
    const double com_height =
        average_leg_length + kWheelRadius + params_.base_link_com_height;
    const double centrifugal_force_ff_raw =
        state_ == State::Balance
            ? params_.centrifugal_force_ff_gain * kBaselinkMass *
                  lateral_acceleration * com_height / safe_body_width
            : 0.0;
    const double centrifugal_force_ff = std::clamp(
        centrifugal_force_ff_raw,
        -params_.centrifugal_force_ff_limit,
        params_.centrifugal_force_ff_limit);

    double left_leg_length_torque =
        params_.vmc_kp *
            (left_expected_length - left_leg.leg_position[0]) -
        params_.vmc_kd * left_leg.leg_velocity[0] + vmc_mass_component;
    double right_leg_length_torque =
        params_.vmc_kp *
            (right_expected_length - right_leg.leg_position[0]) -
        params_.vmc_kd * right_leg.leg_velocity[0] + vmc_mass_component;
    if (state_ == State::Balance) {
        left_leg_length_torque -= centrifugal_force_ff;
        right_leg_length_torque += centrifugal_force_ff;
    }

    const double leg_angle_difference =
        normalize_angle(left_leg.leg_position[1] - right_leg.leg_position[1]);
    const double leg_angle_difference_velocity =
        left_leg.leg_velocity[1] - right_leg.leg_velocity[1];
    const double spin_error = expected_omega_ - angular_velocity.z();
    if (state_ == State::Balance) {
        wheel_spin_error_integral_ = std::clamp(
            wheel_spin_error_integral_ + spin_error * static_cast<double>(dt),
            -kWheelSpinIntegralLimit,
            kWheelSpinIntegralLimit);
    } else {
        wheel_spin_error_integral_ = 0.0;
    }

    const double spin_control_torque_difference =
        state_ == State::Balance
            ? params_.wheel_diff_kp * spin_error +
                  params_.wheel_diff_ki * wheel_spin_error_integral_
            : 0.0;
    const double spin_compensation_torque =
        -spin_control_torque_difference * average_leg_length / kWheelRadius;
    const double leg_angle_sync_torque =
        -params_.leg_angle_diff_kp * leg_angle_difference -
        params_.leg_angle_diff_kd * leg_angle_difference_velocity +
        spin_compensation_torque;

    const double left_leg_angle_torque =
        lqr_control_[1] + leg_angle_sync_torque;
    const double right_leg_angle_torque =
        lqr_control_[1] - leg_angle_sync_torque;
    const double left_wheel_torque =
        lqr_control_[0] - spin_control_torque_difference;
    const double right_wheel_torque =
        lqr_control_[0] + spin_control_torque_difference;

    Eigen::Vector2d left_joint_torque = Eigen::Vector2d::Zero();
    Eigen::Vector2d right_joint_torque = Eigen::Vector2d::Zero();
    if (!leg_.inverse_dynamics(
            left_leg.joint_position,
            Eigen::Vector2d(left_leg_length_torque, left_leg_angle_torque),
            left_joint_torque) ||
        !leg_.inverse_dynamics(
            right_leg.joint_position,
            Eigen::Vector2d(right_leg_length_torque, right_leg_angle_torque),
            right_joint_torque)) {
        set_safe_commands();
        return false;
    }

    if (state_ == State::Recovery) {
        Eigen::Vector2d recovery_joint_position;
        if (!leg_.inverse_kinematics(
                Eigen::Vector2d(0.27, 0.0), recovery_joint_position)) {
            set_safe_commands();
            return false;
        }
        const bool left_front_ok = lf->set_command(
            static_cast<float>(recovery_joint_position[0]),
            0.0F,
            0.0F,
            50.0F,
            2.0F);
        const bool right_front_ok = rf->set_command(
            static_cast<float>(recovery_joint_position[0]),
            0.0F,
            0.0F,
            50.0F,
            2.0F);
        const bool left_rear_ok = lb->set_command(
            static_cast<float>(recovery_joint_position[1]),
            0.0F,
            0.0F,
            50.0F,
            2.0F);
        const bool right_rear_ok = rb->set_command(
            static_cast<float>(recovery_joint_position[1]),
            0.0F,
            0.0F,
            50.0F,
            2.0F);
        const bool left_wheel_ok =
            lw->set_command(0.0F, 0.0F, 0.0F, 0.0F, 0.0F);
        const bool right_wheel_ok =
            rw->set_command(0.0F, 0.0F, 0.0F, 0.0F, 0.0F);
        return left_front_ok && right_front_ok && left_rear_ok &&
               right_rear_ok && left_wheel_ok && right_wheel_ok;
    }

    if (state_ == State::VmcTest) {
        if (!leg_.inverse_dynamics(
                left_leg.joint_position,
                Eigen::Vector2d(left_leg_length_torque, leg_angle_sync_torque),
                left_joint_torque) ||
            !leg_.inverse_dynamics(
                right_leg.joint_position,
                Eigen::Vector2d(right_leg_length_torque,
                                -leg_angle_sync_torque),
                right_joint_torque)) {
            set_safe_commands();
            return false;
        }
    }

    const bool left_front_ok = lf->set_command(
        0.0F,
        0.0F,
        static_cast<float>(clamp_torque(left_joint_torque[0], kHipTorqueLimit)),
        0.0F,
        0.0F);
    const bool left_rear_ok = lb->set_command(
        0.0F,
        0.0F,
        static_cast<float>(clamp_torque(left_joint_torque[1], kHipTorqueLimit)),
        0.0F,
        0.0F);
    const bool right_front_ok = rf->set_command(
        0.0F,
        0.0F,
        static_cast<float>(clamp_torque(right_joint_torque[0], kHipTorqueLimit)),
        0.0F,
        0.0F);
    const bool right_rear_ok = rb->set_command(
        0.0F,
        0.0F,
        static_cast<float>(clamp_torque(right_joint_torque[1], kHipTorqueLimit)),
        0.0F,
        0.0F);
    const bool left_wheel_ok = lw->set_command(
        0.0F,
        0.0F,
        static_cast<float>(clamp_torque(left_wheel_torque, kWheelTorqueLimit)),
        0.0F,
        0.0F);
    const bool right_wheel_ok = rw->set_command(
        0.0F,
        0.0F,
        static_cast<float>(clamp_torque(right_wheel_torque, kWheelTorqueLimit)),
        0.0F,
        0.0F);
    return left_front_ok && left_rear_ok && right_front_ok &&
           right_rear_ok && left_wheel_ok && right_wheel_ok;
}

double Controller::normalize_angle(double angle)
{
    while (angle > kPi) {
        angle -= 2.0 * kPi;
    }
    while (angle < -kPi) {
        angle += 2.0 * kPi;
    }
    return angle;
}

bool Controller::orientation_yaw_pitch_roll(const Eigen::Quaternionf& orientation,
                                            double& yaw,
                                            double& pitch,
                                            double& roll)
{
    Eigen::Quaterniond quaternion(
        static_cast<double>(orientation.w()),
        static_cast<double>(orientation.x()),
        static_cast<double>(orientation.y()),
        static_cast<double>(orientation.z()));
    if (!std::isfinite(quaternion.norm()) || quaternion.norm() < 1.0e-9) {
        return false;
    }
    quaternion.normalize();
    const Eigen::Vector3d ypr =
        quaternion.toRotationMatrix().eulerAngles(2, 1, 0);
    yaw = ypr[0];
    pitch = ypr[1];
    roll = ypr[2];
    return std::isfinite(yaw) && std::isfinite(pitch) && std::isfinite(roll);
}

double Controller::low_pass_filter(double input,
                                   double alpha,
                                   double& filtered_value,
                                   bool& initialized)
{
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
    filtered_value =
        (1.0 - clamped_alpha) * filtered_value + clamped_alpha * input;
    return filtered_value;
}

double Controller::clamp_torque(double torque, double limit)
{
    if (!std::isfinite(torque)) {
        return 0.0;
    }
    return std::clamp(torque, -limit, limit);
}

bool Controller::read_leg_state(Motor* front_hip,
                                Motor* rear_hip,
                                LegState& leg_state) const
{
    if (front_hip == nullptr || rear_hip == nullptr) {
        return false;
    }

    leg_state.joint_position << front_hip->state.rad, rear_hip->state.rad;
    leg_state.joint_velocity << front_hip->state.vel, rear_hip->state.vel;
    const Eigen::Vector2d joint_torque(front_hip->state.toqeue,
                                       rear_hip->state.toqeue);
    if (!leg_.forward_kinematics(leg_state.joint_position,
                                  leg_state.leg_position) ||
        !leg_.forward_velocity(leg_state.joint_position,
                               leg_state.joint_velocity,
                               leg_state.leg_velocity) ||
        !leg_.forward_dynamics(leg_state.joint_position,
                               joint_torque,
                               leg_state.leg_effort)) {
        leg_state = {};
        return false;
    }

    leg_state.valid = true;
    return leg_state.leg_position.allFinite() &&
           leg_state.leg_velocity.allFinite() &&
           leg_state.leg_effort.allFinite();
}

void Controller::set_safe_commands()
{
    const auto stop_motor = [](Motor* motor) {
        return motor == nullptr ||
               motor->set_command(0.0F, 0.0F, 0.0F, 0.0F, 0.0F);
    };

    (void)stop_motor(lf);
    (void)stop_motor(rf);
    (void)stop_motor(lb);
    (void)stop_motor(rb);
    (void)stop_motor(lw);
    (void)stop_motor(rw);
    lqr_control_.setZero();
}

void Controller::update_state_machine(double pitch,
                                       double left_normal_force,
                                       double right_normal_force,
                                       double dt)
{
    if (requested_mode_ != 0) {
        state_ = static_cast<State>(requested_mode_);
        state_switch_elapsed_ = 0.0;
        return;
    }

    State next_state = State::Airborne;
    if (pitch > kPitchLimit || pitch < -kPitchLimit) {
        next_state = State::Recovery;
    } else if (left_normal_force > kContactForceThreshold &&
               right_normal_force > kContactForceThreshold) {
        next_state = State::Balance;
    }

    if (next_state == state_) {
        state_switch_elapsed_ = 0.0;
        return;
    }

    state_switch_elapsed_ += dt;
    if (state_switch_elapsed_ >= kStateSwitchDelay) {
        state_ = next_state;
        state_switch_elapsed_ = 0.0;
        if (state_ != State::Balance) {
            wheel_spin_error_integral_ = 0.0;
        }
    }
}
