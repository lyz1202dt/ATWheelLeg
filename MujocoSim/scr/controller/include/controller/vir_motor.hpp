#pragma once

#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>

#include <string>

#include "motorbase.hpp"

namespace lqr_controller {

class VirMotor final : public Motor {
public:
    VirMotor();

    bool bind(hardware_interface::LoanedStateInterface* position_state,
              hardware_interface::LoanedStateInterface* velocity_state,
              hardware_interface::LoanedStateInterface* effort_state,
              hardware_interface::LoanedCommandInterface* effort_command,
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

private:
    static double clamp(double value, double limit);

    hardware_interface::LoanedStateInterface* position_state_ = nullptr;
    hardware_interface::LoanedStateInterface* velocity_state_ = nullptr;
    hardware_interface::LoanedStateInterface* effort_state_ = nullptr;
    hardware_interface::LoanedCommandInterface* effort_command_ = nullptr;
    double effort_limit_ = 20.0;
    bool bound_ = false;
};

}  // namespace lqr_controller
