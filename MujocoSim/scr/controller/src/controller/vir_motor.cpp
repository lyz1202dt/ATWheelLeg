#include <controller/vir_motor.hpp>

#include <algorithm>
#include <cmath>

namespace lqr_controller {

VirMotor::VirMotor()
    : Motor(nullptr)
{
}

bool VirMotor::bind(hardware_interface::LoanedStateInterface* position_state,
                    hardware_interface::LoanedStateInterface* velocity_state,
                    hardware_interface::LoanedStateInterface* effort_state,
                    hardware_interface::LoanedCommandInterface* position_command,
                    hardware_interface::LoanedCommandInterface* velocity_command,
                    hardware_interface::LoanedCommandInterface* effort_command,
                    hardware_interface::LoanedCommandInterface* kp_command,
                    hardware_interface::LoanedCommandInterface* kd_command,
                    double effort_limit)
{
    if (position_state == nullptr || velocity_state == nullptr ||
        effort_state == nullptr || position_command == nullptr ||
        velocity_command == nullptr || effort_command == nullptr ||
        kp_command == nullptr || kd_command == nullptr ||
        !std::isfinite(effort_limit) || effort_limit <= 0.0) {
        bound_ = false;
        return false;
    }

    position_state_ = position_state;
    velocity_state_ = velocity_state;
    effort_state_ = effort_state;
    position_command_ = position_command;
    velocity_command_ = velocity_command;
    effort_command_ = effort_command;
    kp_command_ = kp_command;
    kd_command_ = kd_command;
    effort_limit_ = effort_limit;
    bound_ = true;
    return true;
}

bool VirMotor::init()
{
    return bound_;
}

bool VirMotor::enable()
{
    return bound_;
}

bool VirMotor::disable()
{
    if (!bound_) {
        return false;
    }
    position_command_->set_value(0.0);
    velocity_command_->set_value(0.0);
    effort_command_->set_value(0.0);
    kp_command_->set_value(0.0);
    kd_command_->set_value(0.0);
    return true;
}

int VirMotor::has_error()
{
    return 0;
}

bool VirMotor::clear_error(int)
{
    return bound_;
}

bool VirMotor::set_command(float pos,
                           float vel,
                           float tor,
                           float kp,
                           float kd)
{
    if (!bound_) {
        return false;
    }

    position_command_->set_value(std::isfinite(pos) ? static_cast<double>(pos) : 0.0);
    velocity_command_->set_value(std::isfinite(vel) ? static_cast<double>(vel) : 0.0);
    effort_command_->set_value(clamp(static_cast<double>(tor), effort_limit_));
    kp_command_->set_value(std::isfinite(kp) ? static_cast<double>(kp) : 0.0);
    kd_command_->set_value(std::isfinite(kd) ? static_cast<double>(kd) : 0.0);
    return true;
}

bool VirMotor::read_state()
{
    if (!bound_) {
        return false;
    }

    const double position = position_state_->get_value();
    const double velocity = velocity_state_->get_value();
    const double effort = effort_state_->get_value();
    if (!std::isfinite(position) || !std::isfinite(velocity) ||
        !std::isfinite(effort)) {
        return false;
    }

    state.r = 0;
    state.rad = static_cast<float>(position);
    state.continue_rad = static_cast<float>(position);
    state.vel = static_cast<float>(velocity);
    state.toqeue = static_cast<float>(effort);
    return true;
}

double VirMotor::clamp(double value, double limit)
{
    if (!std::isfinite(value)) {
        return 0.0;
    }
    return std::clamp(value, -limit, limit);
}

}  // namespace lqr_controller
