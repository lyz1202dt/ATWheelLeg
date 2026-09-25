#include <iostream>

#include "controller.hpp"

int main()
{
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
