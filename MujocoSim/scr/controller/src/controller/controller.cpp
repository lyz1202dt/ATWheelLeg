#include <controller/controller.hpp>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include <pluginlib/class_list_macros.hpp>

namespace lqr_controller {

namespace {

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

bool numeric_array_parameter(const rclcpp::Parameter& parameter,
                             std::vector<double>& values)
{
    if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
        values = parameter.as_double_array();
        return true;
    }
    if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) {
        const auto integer_values = parameter.as_integer_array();
        values.clear();
        values.reserve(integer_values.size());
        for (const auto value : integer_values) {
            values.push_back(static_cast<double>(value));
        }
        return true;
    }
    return false;
}

}  // namespace

LQRController::LQRController()
    : controller_(
          &imu_,
          &lf_motor_,
          &rf_motor_,
          &lb_motor_,
          &rb_motor_,
          &lw_motor_,
          &rw_motor_)
{
}

controller_interface::CallbackReturn LQRController::on_init()
{
    auto_declare<std::string>("imu_topic", imu_topic_);
    auto_declare<std::string>("imu_pose_topic", imu_pose_topic_);
    auto_declare<std::string>("cmd_vel_topic", cmd_vel_topic_);
    auto_declare<double>("effort_limit", effort_limit_);
    auto_declare<double>("desired_height", desired_height_);
    auto_declare<int>("mode", requested_mode_);
    auto_declare<std::vector<double>>("q_diag", to_vector(q_diag_));
    auto_declare<std::vector<double>>("r_diag", to_vector(r_diag_));
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LQRController::on_configure(
    const rclcpp_lifecycle::State& previous_state)
{
    (void)previous_state;
    RCLCPP_INFO(get_node()->get_logger(), "Configuring virtual IMU and motors");

    imu_topic_ = get_node()->get_parameter("imu_topic").as_string();
    imu_pose_topic_ = get_node()->get_parameter("imu_pose_topic").as_string();
    cmd_vel_topic_ = get_node()->get_parameter("cmd_vel_topic").as_string();
    effort_limit_ = get_node()->get_parameter("effort_limit").as_double();
    desired_height_ =
        static_cast<float>(get_node()->get_parameter("desired_height").as_double());
    requested_mode_ = get_node()->get_parameter("mode").as_int();

    std::vector<double> q_values;
    std::vector<double> r_values;
    if (!numeric_array_parameter(
            get_node()->get_parameter("q_diag"), q_values) ||
        !numeric_array_parameter(
            get_node()->get_parameter("r_diag"), r_values) ||
        q_values.size() != q_diag_.size() ||
        r_values.size() != r_diag_.size()) {
        RCLCPP_ERROR(
            get_node()->get_logger(),
            "q_diag must have 6 values and r_diag must have 2 values");
        return controller_interface::CallbackReturn::ERROR;
    }
    for (size_t index = 0; index < q_diag_.size(); ++index) {
        if (!std::isfinite(q_values[index]) || q_values[index] < 0.0) {
            RCLCPP_ERROR(
                get_node()->get_logger(),
                "q_diag values must be finite and non-negative");
            return controller_interface::CallbackReturn::ERROR;
        }
        q_diag_[index] = static_cast<float>(q_values[index]);
    }
    for (size_t index = 0; index < r_diag_.size(); ++index) {
        if (!std::isfinite(r_values[index]) || r_values[index] <= 0.0) {
            RCLCPP_ERROR(
                get_node()->get_logger(),
                "r_diag values must be finite and positive");
            return controller_interface::CallbackReturn::ERROR;
        }
        r_diag_[index] = static_cast<float>(r_values[index]);
    }

    if (!std::isfinite(effort_limit_) || effort_limit_ <= 0.0 ||
        !std::isfinite(desired_height_) || desired_height_ <= 0.0F ||
        requested_mode_ < 0 || requested_mode_ > 3 ||
        !imu_.configure(get_node(), imu_topic_, imu_pose_topic_) ||
        !configure_lqr_gain()) {
        RCLCPP_ERROR(
            get_node()->get_logger(),
            "Failed to configure virtual controller adapters or LQR gain");
        return controller_interface::CallbackReturn::ERROR;
    }

    cmd_vel_subscriber_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
        cmd_vel_topic_,
        10,
        [this](const geometry_msgs::msg::Twist& msg) { cmd_vel_callback(msg); });

    RCLCPP_INFO(get_node()->get_logger(), "Virtual controller configured");
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LQRController::on_activate(
    const rclcpp_lifecycle::State& previous_state)
{
    (void)previous_state;
    RCLCPP_INFO(get_node()->get_logger(), "Binding virtual motors");
    if (!bind_motor_interfaces()) {
        RCLCPP_ERROR(
            get_node()->get_logger(),
            "Failed to bind virtual motors to ros2_control interfaces");
        return controller_interface::CallbackReturn::ERROR;
    }

    const bool enabled = lf_motor_.enable() && rf_motor_.enable() &&
                         lb_motor_.enable() && rb_motor_.enable() &&
                         lw_motor_.enable() && rw_motor_.enable();
    RCLCPP_INFO(
        get_node()->get_logger(),
        "Virtual motor binding %s",
        enabled ? "succeeded" : "failed");
    return enabled ? controller_interface::CallbackReturn::SUCCESS
                   : controller_interface::CallbackReturn::ERROR;
}

controller_interface::CallbackReturn LQRController::on_deactivate(
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

controller_interface::return_type LQRController::update(
    const rclcpp::Time& time,
    const rclcpp::Duration& period)
{
    (void)time;
    if (!read_motor_states()) {
        RCLCPP_ERROR_THROTTLE(
            get_node()->get_logger(),
            *get_node()->get_clock(),
            1000,
            "Invalid motor state interface value");
        return controller_interface::return_type::ERROR;
    }

    const double dt = period.seconds();
    controller_.input(
        expected_velocity_.load(std::memory_order_relaxed),
        expected_omega_.load(std::memory_order_relaxed),
        desired_height_,
        requested_mode_);

    // Core::Controller owns the state machine, LQR/VMC calculation, and all
    // motor commands. This wrapper only adapts ROS 2 data to its interfaces.
    (void)controller_.update(static_cast<float>(dt));
    return controller_interface::return_type::OK;
}

controller_interface::InterfaceConfiguration
LQRController::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration configuration;
    configuration.type =
        controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto* joint_name : motor_joint_names_) {
        configuration.names.emplace_back(
            std::string(joint_name) + "/effort");
    }
    return configuration;
}

controller_interface::InterfaceConfiguration
LQRController::state_interface_configuration() const
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

bool LQRController::bind_motor_interfaces()
{
    std::array<VirMotor*, kMotorCount> motors = {
        &lf_motor_, &rf_motor_, &lb_motor_,
        &rb_motor_, &lw_motor_, &rw_motor_};

    for (size_t index = 0; index < kMotorCount; ++index) {
        const std::string joint_name(motor_joint_names_[index]);
        auto* position_state = find_interface(
            state_interfaces_, joint_name + "/position");
        auto* velocity_state = find_interface(
            state_interfaces_, joint_name + "/velocity");
        auto* effort_state = find_interface(
            state_interfaces_, joint_name + "/effort");
        auto* effort_command = find_interface(
            command_interfaces_, joint_name + "/effort");
        if (position_state == nullptr || velocity_state == nullptr ||
            effort_state == nullptr || effort_command == nullptr ||
            !motors[index]->bind(
                position_state,
                velocity_state,
                effort_state,
                effort_command,
                effort_limit_)) {
            return false;
        }
    }
    return true;
}

bool LQRController::configure_lqr_gain()
{
    Eigen::Matrix<double, 2, 6> gain;
    if (!controller_.calculate_lqr_gain(
            q_diag_.data(), r_diag_.data(), gain)) {
        return false;
    }
    return controller_.set_K(gain);
}

bool LQRController::read_motor_states()
{
    return lf_motor_.read_state() && rf_motor_.read_state() &&
           lb_motor_.read_state() && rb_motor_.read_state() &&
           lw_motor_.read_state() && rw_motor_.read_state();
}

void LQRController::cmd_vel_callback(const geometry_msgs::msg::Twist& msg)
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

std::vector<double> LQRController::to_vector(
    const std::array<float, 6>& values)
{
    return std::vector<double>(values.begin(), values.end());
}

std::vector<double> LQRController::to_vector(
    const std::array<float, 2>& values)
{
    return std::vector<double>(values.begin(), values.end());
}

}  // namespace lqr_controller

PLUGINLIB_EXPORT_CLASS(
    lqr_controller::LQRController,
    controller_interface::ControllerInterface)
