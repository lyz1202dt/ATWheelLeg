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

    Eigen::Vector2d leg_state;
    if (!leg.forward_kinematics(joint_position, leg_state) ||
        (leg_state - Eigen::Vector2d(0.27, 0.0)).norm() > 1.0e-6) {
        std::cerr << "kinematics round trip failed\n";
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
