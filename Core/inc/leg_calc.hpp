#pragma once

#include <Eigen/Dense>
#include <autodiff/forward/real.hpp>
#include <autodiff/forward/real/eigen.hpp>

#include <cmath>
#include <type_traits>

class LegCalc {
public:
    LegCalc(double hip_half_distance, double upper_link_length, double lower_link_length);

    // Input: front/rear hip angles. Output: leg length and leg angle.
    // The leg angle is zero when the leg points vertically down.
    template <typename Scalar>
    bool forward_kinematics(const Eigen::Matrix<Scalar, 2, 1>& joint_position,
                            Eigen::Matrix<Scalar, 2, 1>& leg_state) const
    {
        using Vector2 = Eigen::Matrix<Scalar, 2, 1>;
        using std::atan2;
        using std::cos;
        using std::sin;
        using std::sqrt;

        leg_state.setZero();
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

        leg_state[0] = sqrt(wheel.dot(wheel));
        leg_state[1] = atan2(-wheel[0], wheel[1]);
        return true;
    }

    bool inverse_kinematics(const Eigen::Vector2d& leg_state, Eigen::Vector2d& joint_position) const;
    bool forward_velocity(const Eigen::Vector2d& joint_position,
                          const Eigen::Vector2d& joint_velocity,
                          Eigen::Vector2d& leg_velocity) const;
    bool inverse_velocity(const Eigen::Vector2d& joint_position,
                          const Eigen::Vector2d& leg_velocity,
                          Eigen::Vector2d& joint_velocity) const;
    bool forward_dynamics(const Eigen::Vector2d& joint_position,
                          const Eigen::Vector2d& joint_torque,
                          Eigen::Vector2d& leg_effort) const;
    bool inverse_dynamics(const Eigen::Vector2d& joint_position,
                          const Eigen::Vector2d& leg_effort,
                          Eigen::Vector2d& joint_torque) const;

private:
    Eigen::Matrix2d calc_jacobian(const Eigen::Vector2d& joint_position) const;

    double l0_;
    double l1_;
    double l2_;
};
