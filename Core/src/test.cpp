#include <iostream>

#include "controller.hpp"
#include "leg_calc.hpp"

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
    if (!controller.set_gain_table(
            {0.20, 0.28, 0.34},
            {
                -0.2175, -1.0147, -8.1376, -2.5916, 6.9973, 1.8424,
                0.0778, 0.3826, 6.1639, 2.3130, 24.8878, 6.6832,
                -0.2247, -1.0360, -8.8680, -2.8228, 6.1353, 1.6259,
                0.0683, 0.3305, 5.7885, 2.0287, 26.0139, 6.9852,
                -0.2247, -1.0360, -8.8680, -2.8228, 6.1353, 1.6259,
                0.0683, 0.3305, 5.7885, 2.0287, 26.0139, 6.9852,
            },
            error)) {
        std::cerr << "LQR gain table validation failed: " << error << '\n';
        return 1;
    }

    std::cout << "Core checks passed\n";
    return 0;
}
