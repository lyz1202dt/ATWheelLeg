#include "controller/controller_at.hpp"

#include <cmath>
#include <iostream>

namespace {

class TestMotor : public Motor {
public:
    TestMotor() : Motor(nullptr) {
        state.rad = 0.0f;
        state.vel = 0.0f;
        state.toqeue = 0.0f;
    }

    bool set_command(float, float, float torque, float, float) override {
        command_torque = torque;
        return true;
    }

    float command_torque{0.0f};
};

} // namespace

int main() {
    IMUBase imu;
    TestMotor lf, rf, lb, rb, lw, rw;
    int model_requests = 0;
    auto gain = [](const double&, const double&, Eigen::Matrix<double, 4, 10>& K) {
        K.setZero();
        return true;
    };
    auto model = [&model_requests](const double&, const double&,
                                   Eigen::Matrix<double, 10, 10>& A,
                                   Eigen::Matrix<double, 10, 4>& B) {
        ++model_requests;
        A = 0.95 * Eigen::Matrix<double, 10, 10>::Identity();
        B.setZero();
        B(0, 0) = 0.5;
        B(2, 1) = 0.5;
        B(4, 2) = 0.5;
        B(6, 3) = 0.5;
        return true;
    };

    ControllerAT controller(&imu, &lf, &rf, &lb, &rb, &lw, &rw, gain, model);
    if (model_requests != 50) {
        std::cerr << "MPC models were not initialized on the 7x7 grid\n";
        return 1;
    }

    Eigen::Vector2d joint_position, leg_torque;
    if (!controller.leg_calc_->inverse_kinematics(Eigen::Vector2d(0.25, 0.0), joint_position) ||
        !controller.leg_calc_->inverse_dynamics(joint_position, Eigen::Vector2d(300.0, 0.0), leg_torque)) {
        std::cerr << "Could not set up two-wheel contact\n";
        return 1;
    }
    lf.state.rad = rf.state.rad = static_cast<float>(joint_position[0]);
    lb.state.rad = rb.state.rad = static_cast<float>(joint_position[1]);
    lf.state.toqeue = rf.state.toqeue = static_cast<float>(leg_torque[0]);
    lb.state.toqeue = rb.state.toqeue = static_cast<float>(leg_torque[1]);

    controller.input(3.0f, 2.0f, 0.25f, 4);
    if (!controller.update(0) || !controller.update(2)) {
        std::cerr << "Controller failed to enter balance control\n";
        return 1;
    }
    if (!std::isfinite(lw.command_torque) || !std::isfinite(rw.command_torque) ||
        std::abs(lw.command_torque) + std::abs(rw.command_torque) < 1e-6f ||
        std::abs(lw.command_torque) > 2.0001f || std::abs(rw.command_torque) > 2.0001f) {
        std::cerr << "Two-wheel MPC did not return bounded, nonzero wheel commands\n";
        return 1;
    }
    if (model_requests != 50) {
        std::cerr << "MPC model was recomputed during control\n";
        return 1;
    }
    return 0;
}
