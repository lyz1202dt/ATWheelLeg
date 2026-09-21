#include "tools/leg_calc.hpp"

#include <Eigen/LU>

#include <algorithm>
#include <cmath>

namespace {

double clamp_unit(double value)
{
    return std::clamp(value, -1.0, 1.0);
}

bool reachable(double distance, double link_a, double link_b)
{
    constexpr double kEpsilon = 1.0e-9;
    return distance > kEpsilon && distance <= link_a + link_b &&
           distance >= std::abs(link_a - link_b);
}

bool is_finite_vector(const Eigen::Vector2d& vector)
{
    return vector.allFinite();
}

bool solve_map(const Eigen::Matrix2d& matrix,
               const Eigen::Vector2d& input,
               Eigen::Vector2d& output)
{
    Eigen::FullPivLU<Eigen::Matrix2d> solver(matrix);
    solver.setThreshold(1.0e-9);
    if (!solver.isInvertible()) {
        output.setZero();
        return false;
    }

    output = solver.solve(input);
    return is_finite_vector(output);
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

Eigen::Matrix2d LegCalc::calc_jacobian(const Eigen::Vector2d& joint_position) const
{
    autodiff::Vector2real q;
    q << joint_position[0], joint_position[1];

    const auto leg_state_function = [this](const autodiff::Vector2real& q_auto) {
        autodiff::Vector2real output;
        forward_kinematics(q_auto, output);
        return output;
    };

    autodiff::Vector2real leg_state;
    Eigen::Matrix2d jacobian;
    autodiff::jacobian(
        leg_state_function,
        autodiff::wrt(q),
        autodiff::at(q),
        leg_state,
        jacobian);
    return jacobian;
}

bool LegCalc::inverse_kinematics(const Eigen::Vector2d& leg_state,
                                 Eigen::Vector2d& joint_position) const
{
    const double leg_length = leg_state[0];
    const double leg_angle = leg_state[1];
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

bool LegCalc::forward_velocity(const Eigen::Vector2d& joint_position,
                               const Eigen::Vector2d& joint_velocity,
                               Eigen::Vector2d& leg_velocity) const
{
    if (!is_finite_vector(joint_position) || !is_finite_vector(joint_velocity)) {
        leg_velocity.setZero();
        return false;
    }

    leg_velocity = calc_jacobian(joint_position) * joint_velocity;
    return is_finite_vector(leg_velocity);
}

bool LegCalc::inverse_velocity(const Eigen::Vector2d& joint_position,
                               const Eigen::Vector2d& leg_velocity,
                               Eigen::Vector2d& joint_velocity) const
{
    if (!is_finite_vector(joint_position) || !is_finite_vector(leg_velocity)) {
        joint_velocity.setZero();
        return false;
    }

    return solve_map(calc_jacobian(joint_position), leg_velocity, joint_velocity);
}

bool LegCalc::forward_dynamics(const Eigen::Vector2d& joint_position,
                               const Eigen::Vector2d& joint_torque,
                               Eigen::Vector2d& leg_effort) const
{
    if (!is_finite_vector(joint_position) || !is_finite_vector(joint_torque)) {
        leg_effort.setZero();
        return false;
    }

    return solve_map(
        calc_jacobian(joint_position).transpose(),
        joint_torque,
        leg_effort);
}

bool LegCalc::inverse_dynamics(const Eigen::Vector2d& joint_position,
                               const Eigen::Vector2d& leg_effort,
                               Eigen::Vector2d& joint_torque) const
{
    if (!is_finite_vector(joint_position) || !is_finite_vector(leg_effort)) {
        joint_torque.setZero();
        return false;
    }

    joint_torque = calc_jacobian(joint_position).transpose() * leg_effort;
    return is_finite_vector(joint_torque);
}
