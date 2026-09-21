#include <controller/controller.hpp>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include <pluginlib/class_list_macros.hpp>

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

template <size_t Size>
bool parameter_to_array(const rclcpp::Parameter& parameter,
                        std::array<double, Size>& values,
                        std::string& error)
{
    std::vector<double> parameter_values;
    if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
        parameter_values = parameter.as_double_array();
    } else if (
        parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) {
        const auto integer_values = parameter.as_integer_array();
        parameter_values.reserve(integer_values.size());
        for (const int64_t value : integer_values) {
            parameter_values.push_back(static_cast<double>(value));
        }
    } else {
        error = parameter.get_name() + " must be a numeric array";
        return false;
    }

    if (parameter_values.size() != Size) {
        error = parameter.get_name() + " must contain exactly " +
                std::to_string(Size) + " values";
        return false;
    }

    for (size_t index = 0; index < Size; ++index) {
        if (!std::isfinite(parameter_values[index])) {
            error = parameter.get_name() + " contains a non-finite value";
            return false;
        }
        values[index] = parameter_values[index];
    }
    return true;
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
    auto_declare<int>("mode", requested_mode_);

    auto_declare<double>("body_width", controller_params_.body_width);
    auto_declare<double>(
        "base_link_com_height", controller_params_.base_link_com_height);
    auto_declare<double>(
        "centrifugal_accel_filter_alpha",
        controller_params_.centrifugal_accel_filter_alpha);
    auto_declare<double>(
        "centrifugal_force_ff_gain",
        controller_params_.centrifugal_force_ff_gain);
    auto_declare<double>(
        "centrifugal_force_ff_limit",
        controller_params_.centrifugal_force_ff_limit);
    auto_declare<double>("leg_exp_length", controller_params_.leg_exp_length);
    auto_declare<double>("vmc_kp", controller_params_.vmc_kp);
    auto_declare<double>("vmc_kd", controller_params_.vmc_kd);
    auto_declare<double>(
        "leg_angle_diff_kp", controller_params_.leg_angle_diff_kp);
    auto_declare<double>(
        "leg_angle_diff_kd", controller_params_.leg_angle_diff_kd);
    auto_declare<double>("wheel_diff_kp", controller_params_.wheel_diff_kp);
    auto_declare<double>("wheel_diff_ki", controller_params_.wheel_diff_ki);
    auto_declare<std::vector<double>>(
        "q_diag", std::vector<double>{1.0, 1.0, 10.0, 1.0, 1.0, 1.0});
    auto_declare<std::vector<double>>(
        "r_diag", std::vector<double>{1.0, 1.0});

    parameter_callback_handle_ = get_node()->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& parameters) {
            return on_set_parameters(parameters);
        });

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LQRController::on_configure(
    const rclcpp_lifecycle::State& previous_state)
{
    (void)previous_state;

    imu_topic_ = get_node()->get_parameter("imu_topic").as_string();
    imu_pose_topic_ = get_node()->get_parameter("imu_pose_topic").as_string();
    cmd_vel_topic_ = get_node()->get_parameter("cmd_vel_topic").as_string();
    effort_limit_ = get_node()->get_parameter("effort_limit").as_double();
    requested_mode_ = get_node()->get_parameter("mode").as_int();

    controller_params_.body_width =
        get_node()->get_parameter("body_width").as_double();
    controller_params_.base_link_com_height =
        get_node()->get_parameter("base_link_com_height").as_double();
    controller_params_.centrifugal_accel_filter_alpha =
        get_node()->get_parameter("centrifugal_accel_filter_alpha").as_double();
    controller_params_.centrifugal_force_ff_gain =
        get_node()->get_parameter("centrifugal_force_ff_gain").as_double();
    controller_params_.centrifugal_force_ff_limit =
        get_node()->get_parameter("centrifugal_force_ff_limit").as_double();
    controller_params_.leg_exp_length =
        get_node()->get_parameter("leg_exp_length").as_double();
    controller_params_.vmc_kp = get_node()->get_parameter("vmc_kp").as_double();
    controller_params_.vmc_kd = get_node()->get_parameter("vmc_kd").as_double();
    controller_params_.leg_angle_diff_kp =
        get_node()->get_parameter("leg_angle_diff_kp").as_double();
    controller_params_.leg_angle_diff_kd =
        get_node()->get_parameter("leg_angle_diff_kd").as_double();
    controller_params_.wheel_diff_kp =
        get_node()->get_parameter("wheel_diff_kp").as_double();
    controller_params_.wheel_diff_ki =
        get_node()->get_parameter("wheel_diff_ki").as_double();

    std::string configuration_error;
    if (!std::isfinite(effort_limit_) || effort_limit_ <= 0.0 ||
        requested_mode_ < 0 || requested_mode_ > 3 ||
        !imu_.configure(get_node(), imu_topic_, imu_pose_topic_) ||
        !configure_controller(configuration_error)) {
        RCLCPP_ERROR(
            get_node()->get_logger(),
            "Failed to configure controller parameters or IMU: %s",
            configuration_error.c_str());
        return controller_interface::CallbackReturn::ERROR;
    }

    cmd_vel_subscriber_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
        cmd_vel_topic_,
        10,
        [this](const geometry_msgs::msg::Twist& msg) { cmd_vel_callback(msg); });
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LQRController::on_activate(
    const rclcpp_lifecycle::State& previous_state)
{
    (void)previous_state;
    if (!bind_motor_interfaces()) {
        RCLCPP_ERROR(
            get_node()->get_logger(),
            "Failed to bind virtual motors to chained ros2_control interfaces");
        return controller_interface::CallbackReturn::ERROR;
    }

    const bool enabled = lf_motor_.enable() && rf_motor_.enable() &&
                         lb_motor_.enable() && rb_motor_.enable() &&
                         lw_motor_.enable() && rw_motor_.enable();
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

    controller_.input(
        expected_velocity_.load(std::memory_order_relaxed),
        expected_omega_.load(std::memory_order_relaxed),
        static_cast<float>(controller_params_.leg_exp_length),
        requested_mode_);
    (void)controller_.update(static_cast<float>(period.seconds()));
    return controller_interface::return_type::OK;
}

controller_interface::InterfaceConfiguration
LQRController::command_interface_configuration() const
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

bool LQRController::configure_controller(std::string& error)
{
    if (!controller_.set_params(controller_params_)) {
        error = "invalid controller parameters";
        return false;
    }

    LqrGainDebuger::StateWeight q_diag;
    LqrGainDebuger::InputWeight r_diag;
    if (!parameter_to_array(
            get_node()->get_parameter("q_diag"), q_diag, error) ||
        !parameter_to_array(
            get_node()->get_parameter("r_diag"), r_diag, error)) {
        return false;
    }
    if (!controller_.update_lqr_gain(q_diag, r_diag, error)) {
        return false;
    }

    q_diag_ = q_diag;
    r_diag_ = r_diag;
    return true;
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

rcl_interfaces::msg::SetParametersResult LQRController::on_set_parameters(
    const std::vector<rclcpp::Parameter>& parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    LqrGainDebuger::StateWeight q_diag = q_diag_;
    LqrGainDebuger::InputWeight r_diag = r_diag_;
    bool lqr_weights_changed = false;
    std::string error;

    for (const auto& parameter : parameters) {
        if (parameter.get_name() == "q_diag") {
            if (!parameter_to_array(parameter, q_diag, error)) {
                result.successful = false;
                result.reason = error;
                return result;
            }
            lqr_weights_changed = true;
        } else if (parameter.get_name() == "r_diag") {
            if (!parameter_to_array(parameter, r_diag, error)) {
                result.successful = false;
                result.reason = error;
                return result;
            }
            lqr_weights_changed = true;
        }
    }

    if (lqr_weights_changed &&
        !controller_.update_lqr_gain(q_diag, r_diag, error)) {
        result.successful = false;
        result.reason = error;
        RCLCPP_ERROR(
            get_node()->get_logger(), "Failed to update LQR gain: %s",
            error.c_str());
        return result;
    }

    if (lqr_weights_changed) {
        q_diag_ = q_diag;
        r_diag_ = r_diag;
        RCLCPP_INFO(
            get_node()->get_logger(),
            "LQR gain updated from q_diag/r_diag parameters");
    }
    return result;
}

}  // namespace lqr_controller

PLUGINLIB_EXPORT_CLASS(
    lqr_controller::LQRController,
    controller_interface::ControllerInterface)
