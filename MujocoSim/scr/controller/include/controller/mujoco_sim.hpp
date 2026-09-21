#pragma once

#include <array>
#include <string>
#include <vector>

#include <controller_interface/chainable_controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>

namespace lqr_controller {

class MujocoSimController : public controller_interface::ChainableControllerInterface {
public:
    MujocoSimController();

    controller_interface::CallbackReturn on_init() override;
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

protected:
    std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;
    controller_interface::return_type update_reference_from_subscribers() override;
    controller_interface::return_type update_and_write_commands(
        const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
    static constexpr size_t kMotorCount = 6;
    static constexpr size_t kStateInterfacesPerMotor = 3;
    static constexpr size_t kTargetInterfacesPerMotor = 5;

    struct MotorState {
        double position{0.0};
        double velocity{0.0};
        double effort{0.0};
    };

    struct MotorReference {
        double position{0.0};
        double velocity{0.0};
        double effort{0.0};
        double kp{0.0};
        double kd{0.0};
    };

    void read_joint_states();
    double clamp_effort(double effort) const;

    std::array<std::string, kMotorCount> motor_joint_names_{};
    std::array<MotorState, kMotorCount> motor_state_{};
    std::array<MotorReference, kMotorCount> motor_reference_{};

    double effort_limit_{20.0};
};

}  // namespace lqr_controller
