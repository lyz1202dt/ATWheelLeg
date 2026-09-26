#include <controller/controller_at.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <rclcpp/logging.hpp>
#include <string>
#include <vector>

#include <pluginlib/class_list_macros.hpp>

#include "controller_at.hpp"

namespace lqr_controller {

namespace {

constexpr size_t kMotorCount               = 6U;
constexpr size_t kTargetInterfacesPerMotor = 5U;
constexpr char kReferencePrefix[]          = "mujoco_sim_controller";

template <typename Interface>
Interface* find_interface(std::vector<Interface>& interfaces, const std::string& name) {
    const auto iterator =
        std::find_if(interfaces.begin(), interfaces.end(), [&name](const auto& interface) { return interface.get_name() == name; });
    return iterator == interfaces.end() ? nullptr : &(*iterator);
}

inline float rad2angle(float rad) { return rad * 180.0f / 3.14159265f; }
inline float angle2rad(float rad) { return rad * 3.14159265f / 180.0f; }
} // namespace

LQRControllerAT::LQRControllerAT() {
    lf_motor_.inverse = false;
    lb_motor_.inverse = false;
    lw_motor_.inverse = false;
    lf_motor_.inverse = false;
    lb_motor_.inverse = false;
    lw_motor_.inverse = false;
    lf_motor_.offset  = 1.171866325;  // angle2rad(67.143f);
    rf_motor_.offset  = 1.171866325;  // angle2rad(67.143f);
    lb_motor_.offset  = -1.136586325; // angle2rad(-65.1216f);
    rb_motor_.offset  = -1.136586325; // angle2rad(-65.1216f);

    auto controller = std::make_unique<::ControllerAT>(&imu_, &lf_motor_, &rf_motor_, &lb_motor_, &rb_motor_, &lw_motor_, &rw_motor_);
    controller->register_debug_logger(
        [this](const char* message) { RCLCPP_INFO(get_node()->get_logger(), "%s", message == nullptr ? "" : message); });
    controller_ = std::move(controller);
}

controller_interface::CallbackReturn LQRControllerAT::on_init() {
    const auto node = get_node();
    lf_motor_.set_node(node, "base_link_left_front1_link_joint", 0U);
    rf_motor_.set_node(node, "base_link_right_front1_link_joint", 1U);
    lb_motor_.set_node(node, "base_link_left_rear1_link_joint", 2U);
    rb_motor_.set_node(node, "base_link_right_rear1_link_joint", 3U);
    lw_motor_.set_node(node, "left_rear2_link_left_wheel_link_joint", 4U);
    rw_motor_.set_node(node, "right_rear2_link_right_wheel_link_joint", 5U);

    auto_declare<std::string>("imu_topic", imu_topic_);
    auto_declare<std::string>("imu_pose_topic", imu_pose_topic_);
    auto_declare<std::string>("cmd_vel_topic", cmd_vel_topic_);
    auto_declare<double>("effort_limit", effort_limit_);
    auto_declare<int>("mode", requested_mode_);
    auto_declare<double>("expected_velocity", static_cast<double>(expected_velocity_.load(std::memory_order_relaxed)));
    auto_declare<double>("expected_omega", static_cast<double>(expected_omega_.load(std::memory_order_relaxed)));

    parameter_callback_handle_ = get_node()->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& parameters) -> rcl_interfaces::msg::SetParametersResult {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            for (const auto& param : parameters) {
                const auto& param_name=param.get_name();
                if(param_name=="expected_velocity")
                {
                    expected_velocity_=param.as_double();
                }
                else if(param_name=="expected_omega")
                {
                    expected_omega_=param.as_double();
                }
                else if(param_name=="mode")
                {
                    requested_mode_=param.as_int();
                }
            }
            return result;
        });
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LQRControllerAT::on_configure(const rclcpp_lifecycle::State& previous_state) {
    (void)previous_state;

    imu_topic_      = get_node()->get_parameter("imu_topic").as_string();
    imu_pose_topic_ = get_node()->get_parameter("imu_pose_topic").as_string();
    cmd_vel_topic_  = get_node()->get_parameter("cmd_vel_topic").as_string();
    effort_limit_   = get_node()->get_parameter("effort_limit").as_double();
    requested_mode_ = get_node()->get_parameter("mode").as_int();

    if (!std::isfinite(effort_limit_) || effort_limit_ <= 0.0 || requested_mode_ < 0 || requested_mode_ > 2
        || !imu_.configure(get_node(), imu_topic_, imu_pose_topic_)) {
        RCLCPP_ERROR(get_node()->get_logger(), "Invalid AT controller parameters or IMU configuration");
        return controller_interface::CallbackReturn::ERROR;
    }

    cmd_vel_subscriber_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
        cmd_vel_topic_, 10, [this](const geometry_msgs::msg::Twist& msg) { cmd_vel_callback(msg); });
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LQRControllerAT::on_activate(const rclcpp_lifecycle::State& previous_state) {
    (void)previous_state;
    if (!bind_motor_interfaces()) {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to bind AT virtual motors to chained ros2_control interfaces");
        return controller_interface::CallbackReturn::ERROR;
    }

    const bool enabled =
        lf_motor_.enable() && rf_motor_.enable() && lb_motor_.enable() && rb_motor_.enable() && lw_motor_.enable() && rw_motor_.enable();
    return enabled ? controller_interface::CallbackReturn::SUCCESS : controller_interface::CallbackReturn::ERROR;
}

controller_interface::CallbackReturn LQRControllerAT::on_deactivate(const rclcpp_lifecycle::State& previous_state) {
    (void)previous_state;
    (void)lf_motor_.disable();
    (void)rf_motor_.disable();
    (void)lb_motor_.disable();
    (void)rb_motor_.disable();
    (void)lw_motor_.disable();
    (void)rw_motor_.disable();
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type LQRControllerAT::update(const rclcpp::Time& time, const rclcpp::Duration& period) {
    (void)time;
    if (!read_motor_states() || controller_ == nullptr) {
        RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000, "Invalid AT motor state or controller");
        return controller_interface::return_type::ERROR;
    }

    controller_->input(
        expected_velocity_.load(std::memory_order_relaxed), expected_omega_.load(std::memory_order_relaxed), 0.2F, requested_mode_);
    controller_->update(static_cast<float>(period.seconds()));
    return controller_interface::return_type::OK;
}

controller_interface::InterfaceConfiguration LQRControllerAT::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration configuration;
    configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto* joint_name : motor_joint_names_) {
        const std::string prefix = std::string(kReferencePrefix) + "/" + joint_name + "/";
        configuration.names.emplace_back(prefix + "position");
        configuration.names.emplace_back(prefix + "velocity");
        configuration.names.emplace_back(prefix + "effort");
        configuration.names.emplace_back(prefix + "kp");
        configuration.names.emplace_back(prefix + "kd");
    }
    return configuration;
}

controller_interface::InterfaceConfiguration LQRControllerAT::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration configuration;
    configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto* joint_name : motor_joint_names_) {
        configuration.names.emplace_back(std::string(joint_name) + "/position");
        configuration.names.emplace_back(std::string(joint_name) + "/velocity");
        configuration.names.emplace_back(std::string(joint_name) + "/effort");
    }
    return configuration;
}

bool LQRControllerAT::bind_motor_interfaces() {
    std::array<VirMotor*, kMotorCount> motors = {&lf_motor_, &rf_motor_, &lb_motor_, &rb_motor_, &lw_motor_, &rw_motor_};

    for (size_t index = 0; index < kMotorCount; ++index) {
        const std::string joint_name(motor_joint_names_[index]);
        const std::string prefix = std::string(kReferencePrefix) + "/" + joint_name + "/";
        auto* position_state     = find_interface(state_interfaces_, joint_name + "/position");
        auto* velocity_state     = find_interface(state_interfaces_, joint_name + "/velocity");
        auto* effort_state       = find_interface(state_interfaces_, joint_name + "/effort");
        auto* position_command   = find_interface(command_interfaces_, prefix + "position");
        auto* velocity_command   = find_interface(command_interfaces_, prefix + "velocity");
        auto* effort_command     = find_interface(command_interfaces_, prefix + "effort");
        auto* kp_command         = find_interface(command_interfaces_, prefix + "kp");
        auto* kd_command         = find_interface(command_interfaces_, prefix + "kd");
        if (position_state == nullptr || velocity_state == nullptr || effort_state == nullptr || position_command == nullptr
            || velocity_command == nullptr || effort_command == nullptr || kp_command == nullptr || kd_command == nullptr
            || !motors[index]->bind(
                position_state, velocity_state, effort_state, position_command, velocity_command, effort_command, kp_command, kd_command,
                effort_limit_)) {
            return false;
        }
    }
    return true;
}

bool LQRControllerAT::read_motor_states() {
    return lf_motor_.read_state() && rf_motor_.read_state() && lb_motor_.read_state() && rb_motor_.read_state() && lw_motor_.read_state()
        && rw_motor_.read_state();
}

void LQRControllerAT::cmd_vel_callback(const geometry_msgs::msg::Twist& msg) {
    if (std::isfinite(msg.linear.x)) {
        expected_velocity_.store(static_cast<float>(msg.linear.x), std::memory_order_relaxed);
    }
    if (std::isfinite(msg.angular.z)) {
        expected_omega_.store(static_cast<float>(msg.angular.z), std::memory_order_relaxed);
    }
}


} // namespace lqr_controller

PLUGINLIB_EXPORT_CLASS(lqr_controller::LQRControllerAT, controller_interface::ControllerInterface)
