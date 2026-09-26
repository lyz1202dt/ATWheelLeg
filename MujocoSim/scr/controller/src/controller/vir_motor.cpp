#include <controller/vir_motor.hpp>

#include <algorithm>
#include <cmath>
#include <rclcpp/logging.hpp>

namespace lqr_controller {

VirMotor::VirMotor()
    : Motor(nullptr)
{
}

void VirMotor::set_node(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
    const std::string& name,
    std::size_t throttle_index)
{
    node_ = node;
    if (!name.empty()) {
        name_ = name;
    }
    throttle_index_ = throttle_index;
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

    const double sign = inverse ? -1.0 : 1.0;
    const double position = std::isfinite(pos) ? static_cast<double>(pos) : 0.0;
    const double velocity = std::isfinite(vel) ? static_cast<double>(vel) : 0.0;
    const double effort = clamp(static_cast<double>(tor), effort_limit_);
    const double stiffness = std::isfinite(kp) ? static_cast<double>(kp) : 0.0;
    const double damping = std::isfinite(kd) ? static_cast<double>(kd) : 0.0;

    // The controller works in the logical (program-facing) frame, while the
    // hardware interface uses the raw frame. `offset` is the physical position
    // the motor should reach when the desired position is 0, so a desired
    // position of 0 maps to `offset` on the hardware.
    const double command_position =
        sign * position + static_cast<double>(offset);
    const double command_velocity = sign * velocity;
    const double command_effort = sign * effort;
    const double command_kp = stiffness;
    const double command_kd = damping;

    position_command_->set_value(command_position);
    velocity_command_->set_value(command_velocity);
    effort_command_->set_value(command_effort);
    kp_command_->set_value(command_kp);
    kd_command_->set_value(command_kd);

    if(node_!=nullptr&&print_log)
    {
        RCLCPP_INFO(node_->get_logger(),"motor:%s,pos=%f,vel=%f,torque=%f",name_.c_str(),command_position,command_velocity,command_effort);
    }
    
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

    const double sign = inverse ? -1.0 : 1.0;
    const double offset_rad = static_cast<double>(offset);

    state.r = 0;
    state.rad = static_cast<float>(sign * (position - offset_rad));
    state.continue_rad = static_cast<float>(sign * (position - offset_rad));
    state.vel = static_cast<float>(sign * velocity);
    state.toqeue = static_cast<float>(sign * effort);
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
