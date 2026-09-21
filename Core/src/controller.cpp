#include "controller.hpp"

#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>

extern "C" {

volatile float lqr_q_diag[6] = {
    10.0F, 400.0F, 100.0F, 40.0F, 600.0F, 50.0F,
};

volatile float lqr_r_diag[2] = {
    8.0F, 0.5F,
};

// Increment this value from a debugger to request a new gain calculation.
volatile uint32_t lqr_gain_update_request = 1U;

}

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kStateSwitchDelay = 0.3;
constexpr double kPitchLimit = 0.3;
constexpr double kContactForceThreshold = 8.0;

using StateVector = Eigen::Matrix<double, 6, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Matrix62d = Eigen::Matrix<double, 6, 2>;
using Matrix2d = Eigen::Matrix<double, 2, 2>;
using Matrix12d = Eigen::Matrix<double, 12, 12>;
using Matrix6cd = Eigen::Matrix<std::complex<double>, 6, 6>;

} // namespace

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

bool Controller::set_K(const Eigen::Matrix<double, 2, 6>& gain)
{
    if (!gain.allFinite()) {
        return false;
    }

    gain_sequence_.fetch_add(1U, std::memory_order_acq_rel);
    gain_ = gain;
    air_gain_.setZero();
    air_gain_(1, 2) = gain_(1, 2);
    air_gain_(1, 3) = gain_(1, 3);
    gain_sequence_.fetch_add(1U, std::memory_order_release);
    return true;
}

bool Controller::calculate_lqr_gain(const volatile float q_diag[6],
                                    const volatile float r_diag[2],
                                    Eigen::Matrix<double, 2, 6>& gain) const
{
    if (q_diag == nullptr || r_diag == nullptr) {
        gain.setZero();
        return false;
    }
    return solve_lqr_gain(q_diag, r_diag, gain);
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

    if (imu == nullptr) {
        set_safe_commands();
        return false;
    }
    if (!imu->is_ready()) {
        set_safe_commands();
        return false;
    }

    Eigen::Quaternionf orientation = Eigen::Quaternionf::Identity();
    Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
    if (!imu->read_state(orientation, angular_velocity, acceleration)) {
        set_safe_commands();
        return false;
    }
    (void)acceleration;

    LegState left_leg;
    LegState right_leg;
    if (!read_leg_state(lf, lb, left_leg) ||
        !read_leg_state(rf, rb, right_leg) ||
        lw == nullptr || rw == nullptr) {
        set_safe_commands();
        return false;
    }

    const double pitch = normalized_pitch(orientation);
    const double left_normal_force = left_leg.leg_effort[0];
    const double right_normal_force = right_leg.leg_effort[0];
    update_state_machine(pitch, left_normal_force, right_normal_force, dt);

    expected_position_ += expected_velocity_ * static_cast<double>(dt);

    const double leg_angle = 0.5 * (left_leg.leg_position[1] + right_leg.leg_position[1]);
    const double leg_angle_velocity =
        0.5 * (left_leg.leg_velocity[1] + right_leg.leg_velocity[1]);
    double x = 0.5 * (static_cast<double>(lw->state.rad) +
                      static_cast<double>(rw->state.rad)) *
               kWheelRadius;
    double dx = 0.5 * (static_cast<double>(lw->state.vel) +
                       static_cast<double>(rw->state.vel)) *
                kWheelRadius;
    const double theta = normalize_angle(pitch + leg_angle);
    double dtheta = angular_velocity.y() + leg_angle_velocity;
    const double phi = -pitch;
    double dphi = -angular_velocity.y();

    dx = low_pass_filter(dx, 0.07, filtered_dx_, dx_filter_initialized_);
    dtheta = low_pass_filter(dtheta, 0.7, filtered_dtheta_, dtheta_filter_initialized_);
    dphi = low_pass_filter(dphi, 0.7, filtered_dphi_, dphi_filter_initialized_);

    if (!std::isfinite(x) || !std::isfinite(dx) ||
        !std::isfinite(theta) || !std::isfinite(dtheta) ||
        !std::isfinite(phi) || !std::isfinite(dphi)) {
        set_safe_commands();
        return false;
    }

    StateVector state;
    StateVector expected_state = StateVector::Zero();
    state << x, dx, theta, dtheta, phi, dphi;

    const double position_error = expected_position_ - x;
    expected_state[0] = expected_position_;
    if (position_error > 5.0) {
        expected_state[0] = x + 5.0;
    } else if (position_error < -5.0) {
        expected_state[0] = x - 5.0;
    }

    Eigen::Matrix<double, 2, 6> gain;
    Eigen::Matrix<double, 2, 6> air_gain;
    read_gain(gain, air_gain);

    lqr_control_.setZero();
    if (state_ == State::Balance) {
        lqr_control_ = gain * (expected_state - state);
    } else if (state_ == State::Airborne) {
        lqr_control_ = -air_gain * state;
    }

    const double leg_length_torque_left =
        vmc_kp_ * (expected_height_ - left_leg.leg_position[0]) -
        vmc_kd_ * left_leg.leg_velocity[0];
    const double leg_length_torque_right =
        vmc_kp_ * (expected_height_ - right_leg.leg_position[0]) -
        vmc_kd_ * right_leg.leg_velocity[0];

    const double leg_angle_difference =
        normalize_angle(left_leg.leg_position[1] - right_leg.leg_position[1]);
    const double leg_angle_difference_velocity =
        left_leg.leg_velocity[1] - right_leg.leg_velocity[1];
    const double spin_error =
        expected_omega_ - angular_velocity.z();

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
            ? wheel_difference_kp_ * spin_error +
                  wheel_difference_ki_ * wheel_spin_error_integral_
            : 0.0;
    const double average_leg_length =
        0.5 * (left_leg.leg_position[0] + right_leg.leg_position[0]);
    const double spin_compensation_torque =
        -spin_control_torque_difference * average_leg_length / kWheelRadius;
    const double leg_angle_sync_torque =
        -leg_angle_difference_kp_ * leg_angle_difference -
        leg_angle_difference_kd_ * leg_angle_difference_velocity +
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
    const bool left_torque_ok = leg_.inverse_dynamics(
        left_leg.joint_position,
        Eigen::Vector2d(leg_length_torque_left, left_leg_angle_torque),
        left_joint_torque);
    const bool right_torque_ok = leg_.inverse_dynamics(
        right_leg.joint_position,
        Eigen::Vector2d(leg_length_torque_right, right_leg_angle_torque),
        right_joint_torque);
    if (!left_torque_ok || !right_torque_ok) {
        set_safe_commands();
        return false;
    }

    if (state_ == State::Recovery) {
        Eigen::Vector2d recovery_joint_position;
        if (!leg_.inverse_kinematics(
                Eigen::Vector2d(0.27, 0.0),
                recovery_joint_position)) {
            set_safe_commands();
            return false;
        }

        const bool left_ok = lf->set_command(
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
        const bool left_wheel_ok = lw->set_command(0.0F, 0.0F, 0.0F, 0.0F, 0.0F);
        const bool right_wheel_ok = rw->set_command(0.0F, 0.0F, 0.0F, 0.0F, 0.0F);
        return left_ok && right_front_ok && left_rear_ok &&
               right_rear_ok && left_wheel_ok && right_wheel_ok;
    }

    if (state_ == State::VmcTest) {
        const bool left_test_ok = leg_.inverse_dynamics(
            left_leg.joint_position,
            Eigen::Vector2d(leg_length_torque_left, leg_angle_sync_torque),
            left_joint_torque);
        const bool right_test_ok = leg_.inverse_dynamics(
            right_leg.joint_position,
            Eigen::Vector2d(leg_length_torque_right, -leg_angle_sync_torque),
            right_joint_torque);
        if (!left_test_ok || !right_test_ok) {
            set_safe_commands();
            return false;
        }
    }

    const bool left_front_ok = lf->set_command(
        0.0F, 0.0F, static_cast<float>(clamp_torque(left_joint_torque[0], 12.0)), 0.0F, 0.0F);
    const bool left_rear_ok = lb->set_command(
        0.0F, 0.0F, static_cast<float>(clamp_torque(left_joint_torque[1], 12.0)), 0.0F, 0.0F);
    const bool right_front_ok = rf->set_command(
        0.0F, 0.0F, static_cast<float>(clamp_torque(right_joint_torque[0], 12.0)), 0.0F, 0.0F);
    const bool right_rear_ok = rb->set_command(
        0.0F, 0.0F, static_cast<float>(clamp_torque(right_joint_torque[1], 12.0)), 0.0F, 0.0F);
    const bool left_wheel_ok = lw->set_command(
        0.0F, 0.0F, static_cast<float>(clamp_torque(left_wheel_torque, 10.0)), 0.0F, 0.0F);
    const bool right_wheel_ok = rw->set_command(
        0.0F, 0.0F, static_cast<float>(clamp_torque(right_wheel_torque, 10.0)), 0.0F, 0.0F);
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

double Controller::normalized_pitch(const Eigen::Quaternionf& orientation)
{
    Eigen::Quaterniond quaternion(
        static_cast<double>(orientation.w()),
        static_cast<double>(orientation.x()),
        static_cast<double>(orientation.y()),
        static_cast<double>(orientation.z()));
    if (!std::isfinite(quaternion.norm()) || quaternion.norm() < 1.0e-9) {
        return 0.0;
    }
    quaternion.normalize();
    const Eigen::Matrix3d rotation = quaternion.toRotationMatrix();
    return std::asin(std::clamp(-rotation(2, 0), -1.0, 1.0));
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
    filtered_value = (1.0 - clamped_alpha) * filtered_value + clamped_alpha * input;
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
    const Eigen::Vector2d joint_torque(front_hip->state.toqeue, rear_hip->state.toqeue);
    if (!leg_.forward_kinematics(leg_state.joint_position, leg_state.leg_position) ||
        !leg_.forward_velocity(
            leg_state.joint_position,
            leg_state.joint_velocity,
            leg_state.leg_velocity) ||
        !leg_.forward_dynamics(
            leg_state.joint_position,
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

void Controller::read_gain(Eigen::Matrix<double, 2, 6>& gain,
                           Eigen::Matrix<double, 2, 6>& air_gain) const
{
    for (;;) {
        const uint32_t sequence_begin =
            gain_sequence_.load(std::memory_order_acquire);
        if ((sequence_begin & 1U) != 0U) {
            continue;
        }

        gain = gain_;
        air_gain = air_gain_;

        const uint32_t sequence_end =
            gain_sequence_.load(std::memory_order_acquire);
        if (sequence_begin == sequence_end) {
            return;
        }
    }
}

bool Controller::solve_lqr_gain(const volatile float q_diag[6],
                                const volatile float r_diag[2],
                                Eigen::Matrix<double, 2, 6>& gain) const
{
    Matrix6d system_matrix;
    system_matrix << 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, -13.9285, 0.0, 0.6373, 0.0,
        0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 98.2488, 0.0, 17.6903, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
        0.0, 0.0, 44.9938, 0.0, 54.3689, 0.0;

    Matrix62d input_matrix;
    input_matrix << 0.0, 0.0,
        15.4595, -3.3942,
        0.0, 0.0,
        -65.5078, 39.5039,
        0.0, 0.0,
        -7.9383, 50.5446;

    Matrix6d q_matrix = Matrix6d::Zero();
    for (size_t index = 0; index < 6U; ++index) {
        if (!std::isfinite(q_diag[index]) || q_diag[index] < 0.0F) {
            gain.setZero();
            return false;
        }
        q_matrix(index, index) = static_cast<double>(q_diag[index]);
    }

    Matrix2d r_inverse = Matrix2d::Zero();
    for (size_t index = 0; index < 2U; ++index) {
        if (!std::isfinite(r_diag[index]) || r_diag[index] <= 0.0F) {
            gain.setZero();
            return false;
        }
        r_inverse(index, index) = 1.0 / static_cast<double>(r_diag[index]);
    }

    Matrix12d hamiltonian = Matrix12d::Zero();
    hamiltonian.block<6, 6>(0, 0) = system_matrix;
    hamiltonian.block<6, 6>(0, 6) =
        -input_matrix * r_inverse * input_matrix.transpose();
    hamiltonian.block<6, 6>(6, 0) = -q_matrix;
    hamiltonian.block<6, 6>(6, 6) = -system_matrix.transpose();

    Eigen::ComplexEigenSolver<Matrix12d> eigen_solver(hamiltonian);
    if (eigen_solver.info() != Eigen::Success) {
        gain.setZero();
        return false;
    }

    const auto eigenvalues = eigen_solver.eigenvalues();
    const auto eigenvectors = eigen_solver.eigenvectors();
    std::array<int, 6> stable_indices = {};
    size_t stable_count = 0U;
    for (int index = 0; index < eigenvalues.size(); ++index) {
        if (eigenvalues[index].real() < -1.0e-8) {
            if (stable_count >= stable_indices.size()) {
                gain.setZero();
                return false;
            }
            stable_indices[stable_count++] = index;
        }
    }
    if (stable_count != 6U) {
        gain.setZero();
        return false;
    }

    Matrix6cd u1;
    Matrix6cd u2;
    for (size_t column = 0; column < 6U; ++column) {
        u1.col(static_cast<Eigen::Index>(column)) =
            eigenvectors.block<6, 1>(0, stable_indices[column]);
        u2.col(static_cast<Eigen::Index>(column)) =
            eigenvectors.block<6, 1>(6, stable_indices[column]);
    }

    const auto u1_decomposition = u1.fullPivLu();
    if (!u1_decomposition.isInvertible()) {
        gain.setZero();
        return false;
    }

    const Matrix6cd p_complex = u2 * u1.inverse();
    if (p_complex.imag().cwiseAbs().maxCoeff() > 1.0e-5) {
        gain.setZero();
        return false;
    }

    Matrix6d p = p_complex.real();
    p = 0.5 * (p + p.transpose());
    gain = r_inverse * input_matrix.transpose() * p;
    if (!gain.allFinite()) {
        gain.setZero();
        return false;
    }

    const Matrix6d residual =
        system_matrix.transpose() * p + p * system_matrix -
        p * input_matrix * r_inverse * input_matrix.transpose() * p + q_matrix;
    const double scale =
        1.0 + q_matrix.norm() + system_matrix.norm() * p.norm() +
        (p * input_matrix * r_inverse * input_matrix.transpose() * p).norm();
    if (!std::isfinite(residual.norm()) ||
        residual.norm() > 1.0e-6 * scale) {
        gain.setZero();
        return false;
    }

    return true;
}
