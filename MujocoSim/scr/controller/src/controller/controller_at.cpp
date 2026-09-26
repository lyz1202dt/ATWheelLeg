#include <controller/controller_at.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <pluginlib/class_list_macros.hpp>

#include "controller_at.hpp"

namespace lqr_controller {

namespace {

constexpr size_t kMotorCount = 6U;
constexpr size_t kTargetInterfacesPerMotor = 5U;
constexpr char kReferencePrefix[] = "mujoco_sim_controller";

template <typename Interface>
Interface* find_interface(std::vector<Interface>& interfaces,
                          const std::string& name)
{
    const auto iterator = std::find_if(
        interfaces.begin(),
        interfaces.end(),
        [&name](const auto& interface) { return interface.get_name() == name; });
    return iterator == interfaces.end() ? nullptr : &(*iterator);
}

bool finite_float_parameter(const rclcpp::Parameter& parameter,
                            float& value,
                            std::string& error)
{
    double parameter_value = 0.0;
    if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        parameter_value = parameter.as_double();
    } else if (
        parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        parameter_value = static_cast<double>(parameter.as_int());
    } else {
        error = parameter.get_name() + " must be a number";
        return false;
    }

    if (!std::isfinite(parameter_value) ||
        parameter_value < std::numeric_limits<float>::lowest() ||
        parameter_value > std::numeric_limits<float>::max()) {
        error = parameter.get_name() + " must be a finite float value";
        return false;
    }

    value = static_cast<float>(parameter_value);
    return true;
}

}  // namespace

LQRControllerAT::LQRControllerAT()
{
    controller_ = std::make_unique<::ControllerAT>(
        &imu_,
        &lf_motor_,
        &rf_motor_,
        &lb_motor_,
        &rb_motor_,
        &lw_motor_,
        &rw_motor_);
}

controller_interface::CallbackReturn LQRControllerAT::on_init()
{
    auto_declare<std::string>("imu_topic", imu_topic_);
    auto_declare<std::string>("imu_pose_topic", imu_pose_topic_);
    auto_declare<std::string>("cmd_vel_topic", cmd_vel_topic_);
    auto_declare<double>("effort_limit", effort_limit_);
    auto_declare<int>("mode", requested_mode_);
    auto_declare<double>(
        "expected_velocity",
        static_cast<double>(expected_velocity_.load(std::memory_order_relaxed)));
    auto_declare<double>(
        "expected_omega",
        static_cast<double>(expected_omega_.load(std::memory_order_relaxed)));

    parameter_callback_handle_ = get_node()->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& parameters) {
            return on_set_parameters(parameters);
        });

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LQRControllerAT::on_configure(
    const rclcpp_lifecycle::State& previous_state)
{
    (void)previous_state;

    imu_topic_ = get_node()->get_parameter("imu_topic").as_string();
    imu_pose_topic_ = get_node()->get_parameter("imu_pose_topic").as_string();
    cmd_vel_topic_ = get_node()->get_parameter("cmd_vel_topic").as_string();
    effort_limit_ = get_node()->get_parameter("effort_limit").as_double();
    requested_mode_ = get_node()->get_parameter("mode").as_int();

    float expected_velocity = expected_velocity_.load(std::memory_order_relaxed);
    float expected_omega = expected_omega_.load(std::memory_order_relaxed);
    std::string parameter_error;
    if (!finite_float_parameter(
            get_node()->get_parameter("expected_velocity"),
            expected_velocity,
            parameter_error) ||
        !finite_float_parameter(
            get_node()->get_parameter("expected_omega"),
            expected_omega,
            parameter_error)) {
        RCLCPP_ERROR(
            get_node()->get_logger(),
            "Invalid AT controller command parameter: %s",
            parameter_error.c_str());
        return controller_interface::CallbackReturn::ERROR;
    }
    expected_velocity_.store(expected_velocity, std::memory_order_relaxed);
    expected_omega_.store(expected_omega, std::memory_order_relaxed);

    if (!std::isfinite(effort_limit_) || effort_limit_ <= 0.0 ||
        requested_mode_ < 0 || requested_mode_ > 2 ||
        !imu_.configure(get_node(), imu_topic_, imu_pose_topic_)) {
        RCLCPP_ERROR(
            get_node()->get_logger(),
            "Invalid AT controller parameters or IMU configuration");
        return controller_interface::CallbackReturn::ERROR;
    }

    cmd_vel_subscriber_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
        cmd_vel_topic_,
        10,
        [this](const geometry_msgs::msg::Twist& msg) { cmd_vel_callback(msg); });
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LQRControllerAT::on_activate(
    const rclcpp_lifecycle::State& previous_state)
{
    (void)previous_state;
    if (!bind_motor_interfaces()) {
        RCLCPP_ERROR(
            get_node()->get_logger(),
            "Failed to bind AT virtual motors to chained ros2_control interfaces");
        return controller_interface::CallbackReturn::ERROR;
    }

    const bool enabled = lf_motor_.enable() && rf_motor_.enable() &&
                         lb_motor_.enable() && rb_motor_.enable() &&
                         lw_motor_.enable() && rw_motor_.enable();
    return enabled ? controller_interface::CallbackReturn::SUCCESS
                   : controller_interface::CallbackReturn::ERROR;
}

controller_interface::CallbackReturn LQRControllerAT::on_deactivate(
    const rclcpp_lifecycle::State& previous_state)
{
    (void)previous_state;
    (void)lf_motor_.disable();
    (void)rf_motor_.disable();
    (void)lb_motor_.disable();
    (void)rb_motor_.disable();
    (void)lw_motor_.disable();
    (void)rw_motor_.disable();
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type LQRControllerAT::update(
    const rclcpp::Time& time,
    const rclcpp::Duration& period)
{
    (void)time;
    if (!read_motor_states() || controller_ == nullptr) {
        RCLCPP_ERROR_THROTTLE(
            get_node()->get_logger(),
            *get_node()->get_clock(),
            1000,
            "Invalid AT motor state or controller");
        return controller_interface::return_type::ERROR;
    }

    controller_->input(
        expected_velocity_.load(std::memory_order_relaxed),
        expected_omega_.load(std::memory_order_relaxed),
        0.2F,
        requested_mode_);
    return controller_->update(static_cast<float>(period.seconds()))
               ? controller_interface::return_type::OK
               : controller_interface::return_type::ERROR;
}

controller_interface::InterfaceConfiguration
LQRControllerAT::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration configuration;
    configuration.type =
        controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto* joint_name : motor_joint_names_) {
        const std::string prefix =
            std::string(kReferencePrefix) + "/" + joint_name + "/";
        configuration.names.emplace_back(prefix + "position");
        configuration.names.emplace_back(prefix + "velocity");
        configuration.names.emplace_back(prefix + "effort");
        configuration.names.emplace_back(prefix + "kp");
        configuration.names.emplace_back(prefix + "kd");
    }
    return configuration;
}

controller_interface::InterfaceConfiguration
LQRControllerAT::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration configuration;
    configuration.type =
        controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto* joint_name : motor_joint_names_) {
        configuration.names.emplace_back(
            std::string(joint_name) + "/position");
        configuration.names.emplace_back(
            std::string(joint_name) + "/velocity");
        configuration.names.emplace_back(
            std::string(joint_name) + "/effort");
    }
    return configuration;
}

bool LQRControllerAT::bind_motor_interfaces()
{
    std::array<VirMotor*, kMotorCount> motors = {
        &lf_motor_, &rf_motor_, &lb_motor_,
        &rb_motor_, &lw_motor_, &rw_motor_};

    for (size_t index = 0; index < kMotorCount; ++index) {
        const std::string joint_name(motor_joint_names_[index]);
        const std::string prefix =
            std::string(kReferencePrefix) + "/" + joint_name + "/";
        auto* position_state =
            find_interface(state_interfaces_, joint_name + "/position");
        auto* velocity_state =
            find_interface(state_interfaces_, joint_name + "/velocity");
        auto* effort_state =
            find_interface(state_interfaces_, joint_name + "/effort");
        auto* position_command =
            find_interface(command_interfaces_, prefix + "position");
        auto* velocity_command =
            find_interface(command_interfaces_, prefix + "velocity");
        auto* effort_command =
            find_interface(command_interfaces_, prefix + "effort");
        auto* kp_command =
            find_interface(command_interfaces_, prefix + "kp");
        auto* kd_command =
            find_interface(command_interfaces_, prefix + "kd");
        if (position_state == nullptr || velocity_state == nullptr ||
            effort_state == nullptr || position_command == nullptr ||
            velocity_command == nullptr || effort_command == nullptr ||
            kp_command == nullptr || kd_command == nullptr ||
            !motors[index]->bind(
                position_state,
                velocity_state,
                effort_state,
                position_command,
                velocity_command,
                effort_command,
                kp_command,
                kd_command,
                effort_limit_)) {
            return false;
        }
    }
    return true;
}

bool LQRControllerAT::read_motor_states()
{
    return lf_motor_.read_state() && rf_motor_.read_state() &&
           lb_motor_.read_state() && rb_motor_.read_state() &&
           lw_motor_.read_state() && rw_motor_.read_state();
}

void LQRControllerAT::cmd_vel_callback(const geometry_msgs::msg::Twist& msg)
{
    if (std::isfinite(msg.linear.x)) {
        expected_velocity_.store(
            static_cast<float>(msg.linear.x), std::memory_order_relaxed);
    }
    if (std::isfinite(msg.angular.z)) {
        expected_omega_.store(
            static_cast<float>(msg.angular.z), std::memory_order_relaxed);
    }
}

rcl_interfaces::msg::SetParametersResult LQRControllerAT::on_set_parameters(
    const std::vector<rclcpp::Parameter>& parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    float expected_velocity = expected_velocity_.load(std::memory_order_relaxed);
    float expected_omega = expected_omega_.load(std::memory_order_relaxed);
    bool expected_velocity_changed = false;
    bool expected_omega_changed = false;
    std::string error;

    for (const auto& parameter : parameters) {
        if (parameter.get_name() == "expected_velocity") {
            if (!finite_float_parameter(parameter, expected_velocity, error)) {
                result.successful = false;
                result.reason = error;
                return result;
            }
            expected_velocity_changed = true;
        } else if (parameter.get_name() == "expected_omega") {
            if (!finite_float_parameter(parameter, expected_omega, error)) {
                result.successful = false;
                result.reason = error;
                return result;
            }
            expected_omega_changed = true;
        }
    }

    if (expected_velocity_changed) {
        expected_velocity_.store(expected_velocity, std::memory_order_relaxed);
    }
    if (expected_omega_changed) {
        expected_omega_.store(expected_omega, std::memory_order_relaxed);
    }

    return result;
}

}  // namespace lqr_controller

PLUGINLIB_EXPORT_CLASS(
    lqr_controller::LQRControllerAT,
    controller_interface::ControllerInterface)
