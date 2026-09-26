#include <array>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>

#include "controller.hpp"

namespace {

constexpr double kLegCalc2Tolerance = 1.0e-6;
constexpr double kPi = 3.14159265358979323846;

Eigen::Vector2d polar_to_cartesian(const Eigen::Vector2d& polar_position)
{
    return polar_position[0] *
           Eigen::Vector2d(std::cos(polar_position[1]),
                           std::sin(polar_position[1]));
}

double wrapped_angle_error(const double actual, const double expected)
{
    return std::abs(std::remainder(actual - expected, 2.0 * kPi));
}

bool test_leg_calc2()
{
    const std::array<Eigen::Vector2d, 7> leg_targets = {
        Eigen::Vector2d(0.18, -0.20),
        Eigen::Vector2d(0.20, -0.10),
        Eigen::Vector2d(0.20, 0.00),
        Eigen::Vector2d(0.20, 0.10),
        Eigen::Vector2d(0.22, 0.20),
        Eigen::Vector2d(0.24, -0.15),
        Eigen::Vector2d(0.26, 0.00),
    };

    const LegCalc2 leg(0.0945, 0.0945, 0.1125, 0.1125, 0.1155, 0.2502);
    bool passed = true;
    double max_target_error = 0.0;
    double max_joint_round_trip_error = 0.0;

    std::cout << std::setprecision(12);
    std::cout << "LegCalc2 IK -> FK checks:\n";
    for (std::size_t index = 0; index < leg_targets.size(); ++index) {
        const Eigen::Vector2d& target = leg_targets[index];
        Eigen::Vector2d joint_position;
        Eigen::Vector2d round_trip_position;

        if (!leg.inverse_kinematics(target, joint_position) ||
            !joint_position.allFinite() ||
            !leg.forward_kinematics(joint_position, round_trip_position) ||
            !round_trip_position.allFinite()) {
            std::cerr << "  target " << index
                      << " failed to complete IK -> FK: target="
                      << target.transpose() << '\n';
            passed = false;
            continue;
        }

        const double target_error =
            (polar_to_cartesian(round_trip_position) -
             polar_to_cartesian(target))
                .norm();
        max_target_error = std::max(max_target_error, target_error);
        std::cout << "  target " << index << ": target=" << target.transpose()
                  << " joint=" << joint_position.transpose()
                  << " round_trip=" << round_trip_position.transpose()
                  << " error=" << target_error << '\n';
        if (target_error > kLegCalc2Tolerance) {
            std::cerr << "  target " << index
                      << " exceeded the IK -> FK tolerance\n";
            passed = false;
        }
    }

    std::size_t valid_forward_cases = 0;
    std::size_t checked_round_trip_cases = 0;
    for (int front_index = -6; front_index <= 6; ++front_index) {
        for (int rear_index = -6; rear_index <= 6; ++rear_index) {
            const Eigen::Vector2d original_joint(
                0.2 * static_cast<double>(front_index),
                0.2 * static_cast<double>(rear_index));
            Eigen::Vector2d forward_position;
            if (!leg.forward_kinematics(original_joint, forward_position) ||
                !forward_position.allFinite()) {
                continue;
            }

            ++valid_forward_cases;
            Eigen::Vector2d recovered_joint;
            Eigen::Vector2d round_trip_position;
            if (!leg.inverse_kinematics(forward_position, recovered_joint) ||
                !recovered_joint.allFinite() ||
                !leg.forward_kinematics(recovered_joint, round_trip_position) ||
                !round_trip_position.allFinite()) {
                std::cerr << "  joint case failed to complete FK -> IK -> FK: q="
                          << original_joint.transpose()
                          << " p=" << forward_position.transpose() << '\n';
                passed = false;
                continue;
            }

            ++checked_round_trip_cases;
            const double position_error =
                (polar_to_cartesian(round_trip_position) -
                 polar_to_cartesian(forward_position))
                    .norm();
            const double joint_error = std::hypot(
                wrapped_angle_error(recovered_joint[0], original_joint[0]),
                wrapped_angle_error(recovered_joint[1], original_joint[1]));
            max_target_error = std::max(max_target_error, position_error);
            max_joint_round_trip_error =
                std::max(max_joint_round_trip_error, joint_error);

            if (position_error > kLegCalc2Tolerance) {
                std::cerr << "  joint case exceeded the FK -> IK -> FK tolerance: q="
                          << original_joint.transpose()
                          << " p=" << forward_position.transpose()
                          << " recovered_q=" << recovered_joint.transpose()
                          << " recovered_p=" << round_trip_position.transpose()
                          << " error=" << position_error << '\n';
                passed = false;
            }
        }
    }

    if (valid_forward_cases == 0 || checked_round_trip_cases != valid_forward_cases) {
        std::cerr << "  LegCalc2 did not round-trip all valid forward-kinematics cases: "
                  << checked_round_trip_cases << '/' << valid_forward_cases << '\n';
        passed = false;
    }

    std::cout << "LegCalc2 summary: valid FK cases=" << valid_forward_cases
              << ", round-trip cases=" << checked_round_trip_cases
              << ", max position error=" << max_target_error
              << ", max joint angle difference=" << max_joint_round_trip_error
              << '\n';
    return passed;
}

} // namespace

int main()
{
    if (!test_leg_calc2()) {
        return 1;
    }

    LegCalc leg(0.11, 0.1844, 0.3130);
    Eigen::Vector2d joint_position;
    if (!leg.inverse_kinematics(Eigen::Vector2d(0.27, 0.0), joint_position)) {
        std::cerr << "inverse kinematics failed\n";
        return 1;
    }

    Eigen::Vector2d leg_position;
    if (!leg.forward_kinematics(joint_position, leg_position) ||
        (leg_position - Eigen::Vector2d(0.27, 0.0)).norm() > 1.0e-6) {
        std::cerr << "kinematics round trip failed\n";
        return 1;
    }

    LegCalc3 serial_leg(0.08, 0.20, 0.17);
    const Eigen::Vector2d serial_joint(0.35, 0.90);
    Eigen::Vector2d serial_position;
    if (!serial_leg.forward_kinematics(serial_joint, serial_position)) {
        std::cerr << "serial forward kinematics failed\n";
        return 1;
    }

    Eigen::Vector2d serial_joint_recovered;
    if (!serial_leg.inverse_kinematics(serial_position, serial_joint_recovered) ||
        (serial_joint_recovered - serial_joint).norm() > 1.0e-6) {
        std::cerr << "serial kinematics round trip failed\n";
        return 1;
    }

    Eigen::Vector2d serial_zero_position;
    if (!serial_leg.forward_kinematics(Eigen::Vector2d::Zero(), serial_zero_position) ||
        (serial_zero_position - Eigen::Vector2d(0.45, 0.0)).norm() > 1.0e-6) {
        std::cerr << "serial zero-angle convention failed\n";
        return 1;
    }

    const Eigen::Vector2d serial_velocity_input(0.2, -0.15);
    Eigen::Vector2d serial_velocity;
    if (!serial_leg.forward_velocity(serial_joint, serial_velocity_input, serial_velocity)) {
        std::cerr << "serial velocity mapping failed\n";
        return 1;
    }

    constexpr double kFiniteDifferenceStep = 1.0e-7;
    Eigen::Vector2d serial_position_plus;
    Eigen::Vector2d serial_position_minus;
    if (!serial_leg.forward_kinematics(
            serial_joint + kFiniteDifferenceStep * serial_velocity_input,
            serial_position_plus) ||
        !serial_leg.forward_kinematics(
            serial_joint - kFiniteDifferenceStep * serial_velocity_input,
            serial_position_minus) ||
        ((serial_position_plus - serial_position_minus) /
             (2.0 * kFiniteDifferenceStep) -
         serial_velocity)
                .norm() >
            1.0e-6) {
        std::cerr << "serial Jacobian check failed\n";
        return 1;
    }

    Controller controller(nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr);
    std::string error;
    if (!controller.update_lqr_gain(
            {1.0, 1.0, 10.0, 1.0, 1.0, 1.0}, {1.0, 1.0}, error)) {
        std::cerr << "LQR gain update failed: " << error << '\n';
        return 1;
    }

    std::cout << "Core checks passed\n";
    return 0;
}
