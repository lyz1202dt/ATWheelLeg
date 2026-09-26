#pragma once

#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <cstddef>
#include <string>

#include "motorbase.hpp"

namespace lqr_controller {

class VirMotor final : public Motor {
public:
    VirMotor();

    void set_node(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                  const std::string& name,
                  std::size_t throttle_index = 0U);

    bool bind(hardware_interface::LoanedStateInterface* position_state,
              hardware_interface::LoanedStateInterface* velocity_state,
              hardware_interface::LoanedStateInterface* effort_state,
              hardware_interface::LoanedCommandInterface* position_command,
              hardware_interface::LoanedCommandInterface* velocity_command,
              hardware_interface::LoanedCommandInterface* effort_command,
              hardware_interface::LoanedCommandInterface* kp_command,
              hardware_interface::LoanedCommandInterface* kd_command,
              double effort_limit);

    bool init() override;
    bool enable() override;
    bool disable() override;
    int has_error() override;
    bool clear_error(int cmd) override;
    bool set_command(float pos,
                     float vel,
                     float tor,
                     float kp,
                     float kd) override;
    bool read_state() override;

    bool inverse{false};
    float offset{0.0f};

    bool print_log{false};

private:
    static double clamp(double value, double limit);

    hardware_interface::LoanedStateInterface* position_state_ = nullptr;
    hardware_interface::LoanedStateInterface* velocity_state_ = nullptr;
    hardware_interface::LoanedStateInterface* effort_state_ = nullptr;
    hardware_interface::LoanedCommandInterface* position_command_ = nullptr;
    hardware_interface::LoanedCommandInterface* velocity_command_ = nullptr;
    hardware_interface::LoanedCommandInterface* effort_command_ = nullptr;
    hardware_interface::LoanedCommandInterface* kp_command_ = nullptr;
    hardware_interface::LoanedCommandInterface* kd_command_ = nullptr;
    double effort_limit_ = 20.0;
    bool bound_ = false;
    rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
    std::string name_{"vir_motor"};
    std::size_t throttle_index_{0U};
};

}  // namespace lqr_controller
