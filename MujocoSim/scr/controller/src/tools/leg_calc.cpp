#include "tools/leg_calc.hpp"

#include <algorithm>
#include <cmath>

namespace {

double clamp_unit(const double value) {
    return std::clamp(value, -1.0, 1.0);
}

bool reachable(const double distance, const double link_a, const double link_b) {
    constexpr double kEpsilon = 1e-9;
    return distance > kEpsilon && distance <= link_a + link_b && distance >= std::abs(link_a - link_b);
}

bool is_finite_vector(const Eigen::Vector2d& vector) {
    return std::isfinite(vector[0]) && std::isfinite(vector[1]);
}

bool solve_map(const Eigen::Matrix2d& matrix, const Eigen::Vector2d& input, Eigen::Vector2d& output) {
    Eigen::FullPivLU<Eigen::Matrix2d> solver(matrix);
    solver.setThreshold(1e-9);
    if (!solver.isInvertible()) {
        output.setZero();
        return false;
    }

    output = solver.solve(input);
    return is_finite_vector(output);
}

} // namespace

Eigen::Matrix2d LegCalc::calc_jacobian(const Eigen::Vector2d& radian) const {
    autodiff::Vector2real q;
    q << radian[0], radian[1];

    const auto leg_state_func = [this](const autodiff::Vector2real& q_auto) {
        autodiff::Vector2real out;
        forward_kinematics(q_auto,out);
        return out;
    };

    autodiff::Vector2real leg_state;
    Eigen::Matrix2d jacobian;
    autodiff::jacobian(leg_state_func, autodiff::wrt(q), autodiff::at(q), leg_state, jacobian);

    return jacobian;
}


LegCalc::LegCalc(const double& l0, const double& l1, const double& l2): l0(l0),l1(l1),l2(l2) {

}

bool LegCalc::inverse_kinematics(const Eigen::Matrix<double, 2, 1> &leg,Eigen::Matrix<double, 2, 1> &rad) const {
    const double leg_length = leg[0];
    const double leg_angle = leg[1];
    const double x = -leg_length * std::sin(leg_angle);
    const double y = leg_length * std::cos(leg_angle);

    const double front_dx = x - l0;
    const double front_dy = y;
    const double front_distance = std::hypot(front_dx, front_dy);
    if (!reachable(front_distance, l1, l2)) {
        return false;
    }
    const double front_angle = std::atan2(front_dy, front_dx);
    const double front_offset = std::acos(
        clamp_unit((l1 * l1 + front_distance * front_distance - l2 * l2) / (2.0 * l1 * front_distance)));

    const double rear_dx = x + l0;
    const double rear_dy = y;
    const double rear_distance = std::hypot(rear_dx, rear_dy);
    if (!reachable(rear_distance, l1, l2)) {
        return false;
    }
    const double rear_angle = std::atan2(rear_dy, rear_dx);
    const double rear_offset = std::acos(
        clamp_unit((l1 * l1 + rear_distance * rear_distance - l2 * l2) / (2.0 * l1 * rear_distance)));

    rad = {front_angle - front_offset, rear_angle + rear_offset};
    return true;
}

bool LegCalc::forward_velocity(
    const Eigen::Vector2d &rad,const Eigen::Vector2d &joint_velocity,Eigen::Vector2d &leg_velocity) const {
    if (!is_finite_vector(rad) || !is_finite_vector(joint_velocity)) {
        leg_velocity.setZero();
        return false;
    }

    leg_velocity = calc_jacobian(rad) * joint_velocity;
    return is_finite_vector(leg_velocity);
}

bool LegCalc::inverse_velocity(
    const Eigen::Vector2d &rad,const Eigen::Vector2d &leg_velocity,Eigen::Vector2d &joint_velocity) const {
    if (!is_finite_vector(rad) || !is_finite_vector(leg_velocity)) {
        joint_velocity.setZero();
        return false;
    }

    return solve_map(calc_jacobian(rad), leg_velocity, joint_velocity);
}

    //正动力学
bool LegCalc::forward_dynamics(const Eigen::Vector2d &rad,const Eigen::Vector2d &torque,Eigen::Vector2d &effort) const {
    if (!is_finite_vector(rad) || !is_finite_vector(torque)) {
        effort.setZero();
        return false;
    }

    return solve_map(calc_jacobian(rad).transpose(), torque, effort);
}

    //逆动力学
bool LegCalc::inverse_dynamics(const Eigen::Vector2d &rad,const Eigen::Vector2d &effort,Eigen::Vector2d &torque) const {
    if (!is_finite_vector(rad) || !is_finite_vector(effort)) {
        torque.setZero();
        return false;
    }

    torque = calc_jacobian(rad).transpose() * effort;
    return is_finite_vector(torque);
}
