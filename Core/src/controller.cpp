#include "controller.hpp"

#include <Eigen/Geometry>
#include <autodiff/forward/real.hpp>
#include <autodiff/forward/real/eigen.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <iterator>
#include <type_traits>

namespace {

double clamp_unit(const double value)
{
    return std::clamp(value, -1.0, 1.0);
}

bool reachable(const double distance, const double link_a, const double link_b)
{
    constexpr double kEpsilon = 1.0e-9;
    return distance > kEpsilon && distance <= link_a + link_b &&
           distance >= std::abs(link_a - link_b);
}

} // namespace

LegCalc::LegCalc(double hip_half_distance,
                 double upper_link_length,
                 double lower_link_length)
    : l0_(hip_half_distance),
      l1_(upper_link_length),
      l2_(lower_link_length)
{
}

template <typename Scalar>
bool LegCalc::forward_kinematics_impl(
    const Eigen::Matrix<Scalar, 2, 1>& joint_position,
    Eigen::Matrix<Scalar, 2, 1>& leg_position) const
{
    using Vector2 = Eigen::Matrix<Scalar, 2, 1>;
    using std::atan2;
    using std::cos;
    using std::sin;
    using std::sqrt;

    leg_position.setZero();
    if constexpr (std::is_floating_point_v<Scalar>) {
        if (!std::isfinite(joint_position[0]) || !std::isfinite(joint_position[1])) {
            return false;
        }
    }

    const Scalar l0_scalar = static_cast<Scalar>(l0_);
    const Scalar l1_scalar = static_cast<Scalar>(l1_);
    const Scalar l2_scalar = static_cast<Scalar>(l2_);
    const Scalar epsilon = static_cast<Scalar>(1.0e-9);

    const Vector2 front_elbow(
        l0_scalar + l1_scalar * cos(joint_position[0]),
        l1_scalar * sin(joint_position[0]));
    const Vector2 rear_elbow(
        -l0_scalar + l1_scalar * cos(joint_position[1]),
        l1_scalar * sin(joint_position[1]));

    const Vector2 delta = rear_elbow - front_elbow;
    const Scalar distance_squared = delta.dot(delta);
    if (!(distance_squared > epsilon * epsilon)) {
        return false;
    }

    const Scalar distance = sqrt(distance_squared);
    if (!(distance <= static_cast<Scalar>(2.0) * l2_scalar + epsilon)) {
        return false;
    }

    const Vector2 midpoint = static_cast<Scalar>(0.5) * (front_elbow + rear_elbow);
    const Vector2 half_delta = static_cast<Scalar>(0.5) * delta;
    const Scalar height_squared = l2_scalar * l2_scalar - half_delta.dot(half_delta);
    if (height_squared < -epsilon) {
        return false;
    }

    const Scalar height =
        sqrt(height_squared > static_cast<Scalar>(0.0) ? height_squared : static_cast<Scalar>(0.0));
    const Vector2 normal = Vector2(-delta[1], delta[0]) / distance;
    const Vector2 candidate_a = midpoint + height * normal;
    const Vector2 candidate_b = midpoint - height * normal;
    const Vector2 wheel = candidate_b[1] > candidate_a[1] ? candidate_b : candidate_a;

    leg_position[0] = sqrt(wheel.dot(wheel));
    leg_position[1] = atan2(-wheel[0], wheel[1]);
    return true;
}

bool LegCalc::forward_kinematics(const Eigen::Vector2d& joint_position,
                                 Eigen::Vector2d& leg_position) const
{
    return forward_kinematics_impl(joint_position, leg_position);
}

Eigen::Matrix2d LegCalc::calc_jacobian(const Eigen::Vector2d& joint_position) const
{
    autodiff::Vector2real q;
    q << joint_position[0], joint_position[1];

    const auto leg_position_function = [this](const autodiff::Vector2real& q_auto) {
        autodiff::Vector2real output;
        forward_kinematics_impl(q_auto, output);
        return output;
    };

    autodiff::Vector2real leg_position;
    Eigen::Matrix2d jacobian;
    autodiff::jacobian(
        leg_position_function,
        autodiff::wrt(q),
        autodiff::at(q),
        leg_position,
        jacobian);
    return jacobian;
}

bool LegCalc::inverse_kinematics(const Eigen::Vector2d& leg_position,
                                 Eigen::Vector2d& joint_position) const
{
    const double leg_length = leg_position[0];
    const double leg_angle = leg_position[1];
    const double x = -leg_length * std::sin(leg_angle);
    const double y = leg_length * std::cos(leg_angle);

    const double front_dx = x - l0_;
    const double front_dy = y;
    const double front_distance = std::hypot(front_dx, front_dy);
    if (!reachable(front_distance, l1_, l2_)) {
        return false;
    }
    const double front_angle = std::atan2(front_dy, front_dx);
    const double front_offset = std::acos(clamp_unit(
        (l1_ * l1_ + front_distance * front_distance - l2_ * l2_) /
        (2.0 * l1_ * front_distance)));

    const double rear_dx = x + l0_;
    const double rear_dy = y;
    const double rear_distance = std::hypot(rear_dx, rear_dy);
    if (!reachable(rear_distance, l1_, l2_)) {
        return false;
    }
    const double rear_angle = std::atan2(rear_dy, rear_dx);
    const double rear_offset = std::acos(clamp_unit(
        (l1_ * l1_ + rear_distance * rear_distance - l2_ * l2_) /
        (2.0 * l1_ * rear_distance)));

    joint_position = {front_angle - front_offset, rear_angle + rear_offset};
    return joint_position.allFinite();
}

Controller::Controller(IMUBase* imu_in, Motor* lf_in, Motor* rf_in, Motor* lb_in, Motor* rb_in, Motor* lw_in, Motor* rw_in)
    : ControllerBase(imu_in, lf_in, rf_in, lb_in, rb_in, lw_in, rw_in)
    , leg_(kHipHalfDistance, kUpperLinkLength, kLowerLinkLength) {
    std::string error;
    (void)gain_scheduler_.solve_lqr_gain(kDefaultLqrQDiag, kDefaultLqrRDiag, error);
}

bool Controller::set_params(const Params& params) {
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
    if (std::abs(params.body_width) < 1.0e-6 || params.centrifugal_accel_filter_alpha < 0.0 || params.centrifugal_accel_filter_alpha > 1.0
        || params.centrifugal_force_ff_limit < 0.0 || params.leg_exp_length <= 0.0 || params.vmc_kp < 0.0 || params.vmc_kd < 0.0
        || params.leg_angle_diff_kp < 0.0 || params.leg_angle_diff_kd < 0.0 || params.wheel_diff_kp < 0.0 || params.wheel_diff_ki < 0.0) {
        return false;
    }

    params_          = params;
    expected_height_ = params_.leg_exp_length;
    return true;
}

bool Controller::set_gain_table(const std::vector<double>& lengths, const std::vector<double>& values, std::string& error) {
    return gain_scheduler_.load_table(lengths, values, error);
}

bool Controller::update_lqr_gain(const LqrStateWeight& q_diag, const LqrInputWeight& r_diag, std::string& error) {
    return gain_scheduler_.solve_lqr_gain(q_diag, r_diag, error);
}

void Controller::use_k_tab(bool mode) { gain_scheduler_.use_k_tab(mode); }

void Controller::input(float velocity, float omega, float height, int mode) {
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

Eigen::Vector2d Controller::lqr_control() const { return lqr_control_; }

bool Controller::update(float dt) {
    if (!std::isfinite(dt) || dt <= 0.0F) {
        dt = static_cast<float>(kDefaultDt);
    }

    if (imu == nullptr || !imu->is_ready() || lf == nullptr || rf == nullptr || lb == nullptr || rb == nullptr || lw == nullptr || rw == nullptr) {
        set_safe_commands();
        return false;
    }

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
    if (!imu_state_valid) {
        set_safe_commands();
        return false;
    }

    LegState left_leg;
    LegState right_leg;
    if (!read_leg_position(lf, lb, left_leg) || !read_leg_position(rf, rb, right_leg)) {
        set_safe_commands();
        return false;
    }

    Eigen::Quaterniond normalized_orientation;
    double yaw;
    double pitch;
    double roll;
    if (!orientation_yaw_pitch_roll(orientation, normalized_orientation, yaw, pitch, roll)) {
        set_safe_commands();
        return false;
    }

    const Eigen::Matrix3d body_to_world   = normalized_orientation.toRotationMatrix();
    const Eigen::Matrix3d level_to_world     = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    const Eigen::Vector3d level_acceleration = level_to_world.transpose() * body_to_world * acceleration;
    const double lateral_acceleration        = low_pass_filter(
        level_acceleration.y(), params_.centrifugal_accel_filter_alpha, filtered_lateral_acceleration_,
        lateral_acceleration_filter_initialized_);

    const double left_normal_force  = left_leg.leg_effort[0];
    const double right_normal_force = right_leg.leg_effort[0];
    update_state_machine(pitch, left_normal_force, right_normal_force, dt);

    expected_position_ += expected_velocity_ * static_cast<double>(dt);

    const double leg_angle          = 0.5 * (left_leg.leg_position[1] + right_leg.leg_position[1]);
    const double leg_angle_velocity = 0.5 * (left_leg.leg_velocity[1] + right_leg.leg_velocity[1]);
    const double average_leg_length = 0.5 * (left_leg.leg_position[0] + right_leg.leg_position[0]);
    const double x                  = 0.5 * (static_cast<double>(lw->state.rad) + static_cast<double>(rw->state.rad)) * kWheelRadius;
    double dx                       = 0.5 * (static_cast<double>(lw->state.vel) + static_cast<double>(rw->state.vel)) * kWheelRadius;
    const double theta              = normalize_angle(pitch + leg_angle);
    double dtheta                   = angular_velocity.y() + leg_angle_velocity;
    const double phi                = -pitch;
    double dphi                     = -angular_velocity.y();

    dx     = low_pass_filter(dx, 0.07, filtered_dx_, dx_filter_initialized_);
    dtheta = low_pass_filter(dtheta, 0.7, filtered_dtheta_, dtheta_filter_initialized_);
    dphi   = low_pass_filter(dphi, 0.7, filtered_dphi_, dphi_filter_initialized_);

    if (!std::isfinite(x) || !std::isfinite(dx) || !std::isfinite(theta) || !std::isfinite(dtheta) || !std::isfinite(phi)
        || !std::isfinite(dphi) || !std::isfinite(average_leg_length)) {
        set_safe_commands();
        return false;
    }

    StateVector state;
    state << x, dx, theta, dtheta, phi, dphi;
    StateVector expected_state  = StateVector::Zero();
    const double position_error = expected_position_ - x;
    expected_state[0]           = std::clamp(position_error, -5.0, 5.0) + x;

    if (!gain_scheduler_.update(average_leg_length)) {
        lqr_control_.setZero();
        set_safe_commands();
        return false;
    }

    lqr_control_.setZero();
    if (state_ == State::Balance) {
        lqr_control_ = gain_scheduler_.ground_gain() * (expected_state - state);
    } else if (state_ == State::Airborne) {
        lqr_control_ = -gain_scheduler_.air_gain() * state;
    }

    const double safe_body_width       = std::max(std::abs(params_.body_width), 1.0e-6);
    const double leg_height_roll       = std::atan2(right_leg.leg_position[0] - left_leg.leg_position[0], safe_body_width);
    const double combined_roll         = normalize_angle(roll + leg_height_roll);
    const double roll_error            = normalize_angle(expected_roll_ - combined_roll);
    const double leg_length_difference = safe_body_width * std::tan(roll_error);
    const double left_expected_length  = std::clamp(expected_height_ + 0.5 * leg_length_difference, 0.19, 0.34);
    const double right_expected_length = std::clamp(expected_height_ - 0.5 * leg_length_difference, 0.19, 0.34);

    const double vmc_mass_component = state_ == State::Balance ? kBaselinkMass * 0.5 * 9.8 * std::cos(theta) : 0.0;
    const double com_height         = average_leg_length + kWheelRadius + params_.base_link_com_height;
    const double centrifugal_force_ff_raw =
        state_ == State::Balance ? params_.centrifugal_force_ff_gain * kBaselinkMass * lateral_acceleration * com_height / safe_body_width
                                 : 0.0;
    const double centrifugal_force_ff =
        std::clamp(centrifugal_force_ff_raw, -params_.centrifugal_force_ff_limit, params_.centrifugal_force_ff_limit);

    double left_leg_length_torque =
        params_.vmc_kp * (left_expected_length - left_leg.leg_position[0]) - params_.vmc_kd * left_leg.leg_velocity[0] + vmc_mass_component;
    double right_leg_length_torque = params_.vmc_kp * (right_expected_length - right_leg.leg_position[0])
                                   - params_.vmc_kd * right_leg.leg_velocity[0] + vmc_mass_component;
    if (state_ == State::Balance) {
        left_leg_length_torque -= centrifugal_force_ff;
        right_leg_length_torque += centrifugal_force_ff;
    }

    const double leg_angle_difference          = normalize_angle(left_leg.leg_position[1] - right_leg.leg_position[1]);
    const double leg_angle_difference_velocity = left_leg.leg_velocity[1] - right_leg.leg_velocity[1];
    const double spin_error                    = expected_omega_ - angular_velocity.z();
    if (state_ == State::Balance) {
        wheel_spin_error_integral_ = std::clamp(
            wheel_spin_error_integral_ + spin_error * static_cast<double>(dt), -kWheelSpinIntegralLimit, kWheelSpinIntegralLimit);
    } else {
        wheel_spin_error_integral_ = 0.0;
    }

    const double spin_control_torque_difference =
        state_ == State::Balance ? params_.wheel_diff_kp * spin_error + params_.wheel_diff_ki * wheel_spin_error_integral_ : 0.0;
    const double spin_compensation_torque = -spin_control_torque_difference * average_leg_length / kWheelRadius;
    const double leg_angle_sync_torque    = -params_.leg_angle_diff_kp * leg_angle_difference
                                       - params_.leg_angle_diff_kd * leg_angle_difference_velocity + spin_compensation_torque;

    const double left_wheel_torque      = lqr_control_[0] - spin_control_torque_difference;
    const double right_wheel_torque     = lqr_control_[0] + spin_control_torque_difference;

    if (state_ == State::Recovery) {
        return send_recovery_commands();
    }

    const bool vmc_test = state_ == State::VmcTest;
    const double left_leg_angle_torque =
        vmc_test ? leg_angle_sync_torque : lqr_control_[1] + leg_angle_sync_torque;
    const double right_leg_angle_torque =
        vmc_test ? -leg_angle_sync_torque : lqr_control_[1] - leg_angle_sync_torque;

    Eigen::Vector2d left_joint_torque  = Eigen::Vector2d::Zero();
    Eigen::Vector2d right_joint_torque = Eigen::Vector2d::Zero();
    if (!leg_.inverse_dynamics(left_leg.joint_position,
                               Eigen::Vector2d(left_leg_length_torque, left_leg_angle_torque),
                               left_joint_torque)
        || !leg_.inverse_dynamics(right_leg.joint_position,
                                  Eigen::Vector2d(right_leg_length_torque, right_leg_angle_torque),
                                  right_joint_torque)) {
        set_safe_commands();
        return false;
    }

    return send_torque_commands(left_joint_torque,
                                right_joint_torque,
                                left_wheel_torque,
                                right_wheel_torque);
}

double Controller::normalize_angle(double angle) {
    const double wrapped_angle = std::fmod(angle, 2.0 * kPi);
    if (wrapped_angle > kPi) {
        return wrapped_angle - 2.0 * kPi;
    }
    if (wrapped_angle < -kPi) {
        return wrapped_angle + 2.0 * kPi;
    }
    return wrapped_angle;
}

bool Controller::orientation_yaw_pitch_roll(const Eigen::Quaternionf& orientation,
                                            Eigen::Quaterniond& normalized_orientation,
                                            double& yaw,
                                            double& pitch,
                                            double& roll) {
    normalized_orientation = Eigen::Quaterniond(
        static_cast<double>(orientation.w()), static_cast<double>(orientation.x()), static_cast<double>(orientation.y()),
        static_cast<double>(orientation.z()));
    const double norm = normalized_orientation.norm();
    if (!std::isfinite(norm) || norm < 1.0e-9) {
        return false;
    }
    normalized_orientation.normalize();
    const Eigen::Vector3d ypr = normalized_orientation.toRotationMatrix().eulerAngles(2, 1, 0);
    yaw                       = ypr[0];
    pitch                     = ypr[1];
    roll                      = ypr[2];
    return std::isfinite(yaw) && std::isfinite(pitch) && std::isfinite(roll);
}

double Controller::low_pass_filter(double input, double alpha, double& filtered_value, bool& initialized) {
    if (!initialized) {
        filtered_value = input;
        initialized    = true;
        return filtered_value;
    }

    filtered_value = (1.0 - alpha) * filtered_value + alpha * input;
    return filtered_value;
}

double Controller::clamp_torque(double torque, double limit) {
    if (!std::isfinite(torque)) {
        return 0.0;
    }
    return std::clamp(torque, -limit, limit);
}

bool Controller::read_leg_position(Motor* front_hip, Motor* rear_hip, LegState& leg_position) const {
    leg_position.joint_position << front_hip->state.rad, rear_hip->state.rad;
    leg_position.joint_velocity << front_hip->state.vel, rear_hip->state.vel;
    const Eigen::Vector2d joint_torque(front_hip->state.toqeue, rear_hip->state.toqeue);
    if (!leg_.forward_kinematics(leg_position.joint_position, leg_position.leg_position)
        || !leg_.forward_velocity(leg_position.joint_position, leg_position.joint_velocity, leg_position.leg_velocity)
        || !leg_.forward_dynamics(leg_position.joint_position, joint_torque, leg_position.leg_effort)) {
        return false;
    }

    return leg_position.leg_position.allFinite() && leg_position.leg_velocity.allFinite() && leg_position.leg_effort.allFinite();
}

bool Controller::send_recovery_commands() {
    Eigen::Vector2d recovery_joint_position;
    if (!leg_.inverse_kinematics(Eigen::Vector2d(0.27, 0.0), recovery_joint_position)) {
        set_safe_commands();
        return false;
    }

    const bool left_front_ok  = lf->set_command(static_cast<float>(recovery_joint_position[0]), 0.0F, 0.0F, 50.0F, 2.0F);
    const bool right_front_ok = rf->set_command(static_cast<float>(recovery_joint_position[0]), 0.0F, 0.0F, 50.0F, 2.0F);
    const bool left_rear_ok   = lb->set_command(static_cast<float>(recovery_joint_position[1]), 0.0F, 0.0F, 50.0F, 2.0F);
    const bool right_rear_ok  = rb->set_command(static_cast<float>(recovery_joint_position[1]), 0.0F, 0.0F, 50.0F, 2.0F);
    const bool left_wheel_ok  = lw->set_command(0.0F, 0.0F, 0.0F, 0.0F, 0.0F);
    const bool right_wheel_ok = rw->set_command(0.0F, 0.0F, 0.0F, 0.0F, 0.0F);
    return left_front_ok && right_front_ok && left_rear_ok && right_rear_ok && left_wheel_ok && right_wheel_ok;
}

bool Controller::send_torque_commands(const Eigen::Vector2d& left_joint_torque,
                                      const Eigen::Vector2d& right_joint_torque,
                                      double left_wheel_torque,
                                      double right_wheel_torque) {
    const bool left_front_ok =
        lf->set_command(0.0F, 0.0F, static_cast<float>(clamp_torque(left_joint_torque[0], kHipTorqueLimit)), 0.0F, 0.0F);
    const bool left_rear_ok =
        lb->set_command(0.0F, 0.0F, static_cast<float>(clamp_torque(left_joint_torque[1], kHipTorqueLimit)), 0.0F, 0.0F);
    const bool right_front_ok =
        rf->set_command(0.0F, 0.0F, static_cast<float>(clamp_torque(right_joint_torque[0], kHipTorqueLimit)), 0.0F, 0.0F);
    const bool right_rear_ok =
        rb->set_command(0.0F, 0.0F, static_cast<float>(clamp_torque(right_joint_torque[1], kHipTorqueLimit)), 0.0F, 0.0F);
    const bool left_wheel_ok =
        lw->set_command(0.0F, 0.0F, static_cast<float>(clamp_torque(left_wheel_torque, kWheelTorqueLimit)), 0.0F, 0.0F);
    const bool right_wheel_ok =
        rw->set_command(0.0F, 0.0F, static_cast<float>(clamp_torque(right_wheel_torque, kWheelTorqueLimit)), 0.0F, 0.0F);
    return left_front_ok && left_rear_ok && right_front_ok && right_rear_ok && left_wheel_ok && right_wheel_ok;
}

void Controller::set_safe_commands() {
    const auto stop_motor = [](Motor* motor) { return motor == nullptr || motor->set_command(0.0F, 0.0F, 0.0F, 0.0F, 0.0F); };

    (void)stop_motor(lf);
    (void)stop_motor(rf);
    (void)stop_motor(lb);
    (void)stop_motor(rb);
    (void)stop_motor(lw);
    (void)stop_motor(rw);
    lqr_control_.setZero();
}

void Controller::update_state_machine(double pitch, double left_normal_force, double right_normal_force, double dt) {
    if (requested_mode_ != 0) {
        state_                = static_cast<State>(requested_mode_);
        state_switch_elapsed_ = 0.0;
        return;
    }

    State next_state = State::Airborne;
    if (pitch > kPitchLimit || pitch < -kPitchLimit) {
        next_state = State::Recovery;
    } else if (left_normal_force > kContactForceThreshold && right_normal_force > kContactForceThreshold) {
        next_state = State::Balance;
    }

    if (next_state == state_) {
        state_switch_elapsed_ = 0.0;
        return;
    }

    state_switch_elapsed_ += dt;
    if (state_switch_elapsed_ >= kStateSwitchDelay) {
        state_                = next_state;
        state_switch_elapsed_ = 0.0;
        if (state_ != State::Balance) {
            wheel_spin_error_integral_ = 0.0;
        }
    }
}




LqrGainScheduler::LqrGainScheduler() = default;

bool LqrGainScheduler::load_table(const std::vector<double>& lengths,
                                  const std::vector<double>& values,
                                  std::string& error) {
    error.clear();
    if (lengths.empty()) {
        error = "K.lengths must contain at least one leg length";
        return false;
    }
    if (values.size() != lengths.size() * kStateSize * kInputSize) {
        error = "K.values must contain exactly 12 values for each K.lengths entry";
        return false;
    }

    table_.clear();
    table_.reserve(lengths.size());
    for (size_t entry_index = 0; entry_index < lengths.size(); ++entry_index) {
        const double length = lengths[entry_index];
        if (!std::isfinite(length)) {
            error = "K.lengths contains a non-finite value";
            table_.clear();
            return false;
        }

        GainMatrix gain;
        for (std::size_t row = 0; row < kInputSize; ++row) {
            for (std::size_t column = 0; column < kStateSize; ++column) {
                const std::size_t value_index = entry_index * kStateSize * kInputSize + row * kStateSize + column;
                const double value       = values[value_index];
                if (!std::isfinite(value)) {
                    error = "K.values contains a non-finite value";
                    table_.clear();
                    return false;
                }
                gain(row, column) = value;
            }
        }
        table_.emplace_back(length, gain);
    }

    std::sort(table_.begin(), table_.end(), [](const TableEntry& lhs, const TableEntry& rhs) { return lhs.first < rhs.first; });

    for (size_t index = 1; index < table_.size(); ++index) {
        if (table_[index].first <= table_[index - 1].first) {
            error = "K.lengths entries must be unique";
            table_.clear();
            return false;
        }
    }

    return update(table_.front().first);
}

bool LqrGainScheduler::update(double leg_length) {
    if (!std::isfinite(leg_length)) {
        return false;
    }

    if (!use_k_tab_) {
        return true;
    }

    if (table_.empty()) {
        return false;
    }

    if (leg_length <= table_.front().first || table_.size() == 1U) {
        ground_gain_ = table_.front().second;
    } else if (leg_length >= table_.back().first) {
        ground_gain_ = table_.back().second;
    } else {
        const auto upper = std::lower_bound(
            table_.begin(), table_.end(), leg_length, [](const TableEntry& entry, double length) { return entry.first < length; });
        const auto lower   = std::prev(upper);
        const double ratio = (leg_length - lower->first) / (upper->first - lower->first);
        ground_gain_       = (1.0 - ratio) * lower->second + ratio * upper->second;
    }

    air_gain_.setZero();
    air_gain_(1, 2) = ground_gain_(1, 2);
    air_gain_(1, 3) = ground_gain_(1, 3);
    return true;
}

bool LqrGainScheduler::empty() const { return table_.empty(); }

const LqrGainScheduler::GainMatrix& LqrGainScheduler::ground_gain() const { return ground_gain_; }

const LqrGainScheduler::GainMatrix& LqrGainScheduler::air_gain() const { return air_gain_; }


bool LqrGainScheduler::solve_lqr_gain(const StateWeight& q_diag,
                                      const InputWeight& r_diag,
                                      std::string& error) {
    using Matrix6d  = Eigen::Matrix<double, kStateSize, kStateSize>;
    using Matrix62d = Eigen::Matrix<double, kStateSize, kInputSize>;
    using Matrix2d  = Eigen::Matrix<double, kInputSize, kInputSize>;
    using Matrix12d = Eigen::Matrix<double, 2 * kStateSize, 2 * kStateSize>;
    using Matrix6cd = Eigen::Matrix<std::complex<double>, kStateSize, kStateSize>;

    error.clear();

    Matrix6d A;
    A << 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -13.9285, 0.0, 0.6373, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 98.2488, 0.0, 17.6903,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 44.9938, 0.0, 54.3689, 0.0;

    Matrix62d B;
    B << 0.0, 0.0, 15.4595, -3.3942, 0.0, 0.0, -65.5078, 39.5039, 0.0, 0.0, -7.9383, 50.5446;

    Matrix6d Q = Matrix6d::Zero();
    for (std::size_t i = 0; i < kStateSize; ++i) {
        if (q_diag[i] < 0.0 || !std::isfinite(q_diag[i])) {
            error = "q_diag values must be finite and greater than or equal to 0";
            return false;
        }
        Q(i, i) = q_diag[i];
    }

    Matrix2d R_inv = Matrix2d::Zero();
    for (std::size_t i = 0; i < kInputSize; ++i) {
        if (r_diag[i] <= 0.0 || !std::isfinite(r_diag[i])) {
            error = "r_diag values must be finite and greater than 0";
            return false;
        }
        R_inv(i, i) = 1.0 / r_diag[i];
    }

    Matrix12d H = Matrix12d::Zero();
    H.template block<kStateSize, kStateSize>(0, 0)                 = A;
    H.template block<kStateSize, kStateSize>(0, kStateSize)         = -B * R_inv * B.transpose();
    H.template block<kStateSize, kStateSize>(kStateSize, 0)         = -Q;
    H.template block<kStateSize, kStateSize>(kStateSize, kStateSize) = -A.transpose();

    Eigen::ComplexEigenSolver<Matrix12d> eigen_solver(H);
    if (eigen_solver.info() != Eigen::Success) {
        error = "Hamiltonian eigen decomposition failed";
        return false;
    }

    const auto eigenvalues  = eigen_solver.eigenvalues();
    const auto eigenvectors = eigen_solver.eigenvectors();
    std::array<int, kStateSize> stable_indices{};
    std::size_t stable_count = 0;
    for (int i = 0; i < eigenvalues.size(); ++i) {
        if (eigenvalues[i].real() < -1.0e-8) {
            if (stable_count >= stable_indices.size()) {
                error = "Riccati Hamiltonian has too many stable eigenvalues";
                return false;
            }
            stable_indices[stable_count++] = i;
        }
    }

    if (stable_count != kStateSize) {
        error = "Riccati Hamiltonian did not provide a 6-dimensional stable subspace";
        return false;
    }

    Matrix6cd U1;
    Matrix6cd U2;
    for (std::size_t col = 0; col < kStateSize; ++col) {
        U1.col(col) = eigenvectors.template block<kStateSize, 1>(0, stable_indices[col]);
        U2.col(col) = eigenvectors.template block<kStateSize, 1>(kStateSize, stable_indices[col]);
    }

    const auto U1_decomposition = U1.fullPivLu();
    if (!U1_decomposition.isInvertible()) {
        error = "Riccati stable subspace is singular";
        return false;
    }

    const Matrix6cd P_complex = U2 * U1.inverse();
    const double max_imag     = P_complex.imag().cwiseAbs().maxCoeff();
    if (max_imag > 1.0e-5) {
        error = "Riccati solution has a significant imaginary component";
        return false;
    }

    Matrix6d P = P_complex.real();
    P          = 0.5 * (P + P.transpose());

    const LqrGainMatrix gain = R_inv * B.transpose() * P;
    if (!gain.allFinite()) {
        error = "Computed LQR gain contains a non-finite value";
        return false;
    }

    const Matrix6d residual = A.transpose() * P + P * A - P * B * R_inv * B.transpose() * P + Q;
    const double scale      = 1.0 + Q.norm() + A.norm() * P.norm() + (P * B * R_inv * B.transpose() * P).norm();
    if (!std::isfinite(residual.norm()) || residual.norm() > 1.0e-6 * scale) {
        error = "Riccati residual check failed";
        return false;
    }

    // Keep the airborne controller semantics used by LqrGainScheduler:
    // only the pitch and pitch-rate feedback terms remain active in the air.
    ground_gain_ = gain;
    air_gain_.setZero();
    air_gain_(1, 2) = gain(1, 2);
    air_gain_(1, 3) = gain(1, 3);

    return true;
}

void LqrGainScheduler::use_k_tab(bool mode) { use_k_tab_ = mode; }
