#include "leg_calc.hpp"

#include <Eigen/LU>

#include <cmath>

namespace {

bool is_finite_vector(const Eigen::Vector2d& vector)
{
    return vector.allFinite();
}

bool is_finite_matrix(const Eigen::Matrix2d& matrix)
{
    return matrix.array().isFinite().all();
}

bool solve_map(const Eigen::Matrix2d& matrix,
               const Eigen::Vector2d& input,
               Eigen::Vector2d& output)
{
    if (!is_finite_matrix(matrix) || !is_finite_vector(input)) {
        output.setZero();
        return false;
    }

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

bool LegCalcBase::forward_velocity(const Eigen::Vector2d& joint_position,
                                   const Eigen::Vector2d& joint_velocity,
                                   Eigen::Vector2d& leg_velocity) const
{
    if (!is_finite_vector(joint_position) || !is_finite_vector(joint_velocity)) {
        leg_velocity.setZero();
        return false;
    }

    const Eigen::Matrix2d jacobian = calc_jacobian(joint_position);
    if (!is_finite_matrix(jacobian)) {
        leg_velocity.setZero();
        return false;
    }

    leg_velocity = jacobian * joint_velocity;
    return is_finite_vector(leg_velocity);
}

bool LegCalcBase::inverse_velocity(const Eigen::Vector2d& joint_position,
                                   const Eigen::Vector2d& leg_velocity,
                                   Eigen::Vector2d& joint_velocity) const
{
    if (!is_finite_vector(joint_position) || !is_finite_vector(leg_velocity)) {
        joint_velocity.setZero();
        return false;
    }

    return solve_map(calc_jacobian(joint_position), leg_velocity, joint_velocity);
}

bool LegCalcBase::forward_dynamics(const Eigen::Vector2d& joint_position,
                                   const Eigen::Vector2d& joint_torque,
                                   Eigen::Vector2d& leg_effort) const
{
    if (!is_finite_vector(joint_position) || !is_finite_vector(joint_torque)) {
        leg_effort.setZero();
        return false;
    }

    return solve_map(calc_jacobian(joint_position).transpose(), joint_torque, leg_effort);
}

bool LegCalcBase::inverse_dynamics(const Eigen::Vector2d& joint_position,
                                   const Eigen::Vector2d& leg_effort,
                                   Eigen::Vector2d& joint_torque) const
{
    if (!is_finite_vector(joint_position) || !is_finite_vector(leg_effort)) {
        joint_torque.setZero();
        return false;
    }

    const Eigen::Matrix2d jacobian = calc_jacobian(joint_position);
    if (!is_finite_matrix(jacobian)) {
        joint_torque.setZero();
        return false;
    }

    joint_torque = jacobian.transpose() * leg_effort;
    return is_finite_vector(joint_torque);
}
