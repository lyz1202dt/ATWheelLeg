#include <controller/controller.hpp>

#include <algorithm>
#include <cmath>
#include <memory>
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

bool numeric_array_parameter(const rclcpp::Parameter& parameter,
                             std::vector<double>& values,
                             std::string& error)
{
    if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
        values = parameter.as_double_array();
        return true;
    }
    if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) {
        const auto integer_values = parameter.as_integer_array();
        values.clear();
        values.reserve(integer_values.size());
        for (const int64_t value : integer_values) {
            values.push_back(static_cast<double>(value));
        }
        return true;
    }

    error = parameter.get_name() + " must be a numeric array";
    return false;
}

}  // namespace

LQRController::LQRController()
{
    lqr_controller_ = std::make_unique<::Controller>(
        &imu_,
        &lf_motor_,
        &rf_motor_,
        &lb_motor_,
        &rb_motor_,
        &lw_motor_,
        &rw_motor_);
    controller_ = lqr_controller_.get();
}

controller_interface::CallbackReturn LQRController::on_init()
{
    const auto node = get_node();
    lf_motor_.set_node(node, "left_front_hip_joint", 0U);
    rf_motor_.set_node(node, "right_front_hip_joint", 1U);
    lb_motor_.set_node(node, "left_rear_hip_joint", 2U);
    rb_motor_.set_node(node, "right_rear_hip_joint", 3U);
    lw_motor_.set_node(node, "left_wheel_joint", 4U);
    rw_motor_.set_node(node, "right_wheel_joint", 5U);

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
    auto_declare<bool>("use_k_tab", use_k_tab_);
    auto_declare<std::vector<double>>(
        "q_diag", std::vector<double>{1.0, 1.0, 10.0, 1.0, 1.0, 1.0});
    auto_declare<std::vector<double>>(
        "r_diag", std::vector<double>{1.0, 1.0});
    auto_declare<std::vector<double>>("K.lengths", std::vector<double>{});
    auto_declare<std::vector<double>>("K.values", std::vector<double>{});

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
    use_k_tab_ = get_node()->get_parameter("use_k_tab").as_bool();

    std::string configuration_error;
    if (!numeric_array_parameter(
            get_node()->get_parameter("K.lengths"), gain_lengths_,
            configuration_error) ||
        !numeric_array_parameter(
            get_node()->get_parameter("K.values"), gain_values_,
            configuration_error)) {
        RCLCPP_ERROR(
            get_node()->get_logger(),
            "Invalid LQR gain table parameters: %s",
            configuration_error.c_str());
        return controller_interface::CallbackReturn::ERROR;
    }
        imu_.configure(get_node(), imu_topic_, imu_pose_topic_);
        configure_controller(configuration_error);

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

    if (controller_ == nullptr) {
        return controller_interface::return_type::ERROR;
    }

    controller_->input(
        expected_velocity_.load(std::memory_order_relaxed),
        expected_omega_.load(std::memory_order_relaxed),
        static_cast<float>(controller_params_.leg_exp_length),
        requested_mode_);
    (void)controller_->update(static_cast<float>(period.seconds()));
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
    if (lqr_controller_ == nullptr) {
        error = "LQR controller is not initialized";
        return false;
    }

    if (!lqr_controller_->set_params(controller_params_)) {
        error = "invalid controller parameters";
        return false;
    }

    LqrStateWeight q_diag;
    LqrInputWeight r_diag;
    if (!parameter_to_array(
            get_node()->get_parameter("q_diag"), q_diag, error) ||
        !parameter_to_array(
            get_node()->get_parameter("r_diag"), r_diag, error)) {
        return false;
    }
    if (!lqr_controller_->update_lqr_gain(q_diag, r_diag, error)) {
        return false;
    }
    if (use_k_tab_ &&
        !lqr_controller_->set_gain_table(gain_lengths_, gain_values_, error)) {
        return false;
    }
    if (!use_k_tab_ &&
        (!gain_lengths_.empty() || !gain_values_.empty()) &&
        !lqr_controller_->set_gain_table(gain_lengths_, gain_values_, error)) {
        return false;
    }
    lqr_controller_->use_k_tab(use_k_tab_);

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

    LqrStateWeight q_diag = q_diag_;
    LqrInputWeight r_diag = r_diag_;
    std::vector<double> gain_lengths = gain_lengths_;
    std::vector<double> gain_values = gain_values_;
    bool use_k_tab = use_k_tab_;
    bool lqr_weights_changed = false;
    bool gain_table_changed = false;
    bool use_k_tab_changed = false;
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
        } else if (parameter.get_name() == "K.lengths") {
            if (!numeric_array_parameter(parameter, gain_lengths, error)) {
                result.successful = false;
                result.reason = error;
                return result;
            }
            gain_table_changed = true;
        } else if (parameter.get_name() == "K.values") {
            if (!numeric_array_parameter(parameter, gain_values, error)) {
                result.successful = false;
                result.reason = error;
                return result;
            }
            gain_table_changed = true;
        } else if (parameter.get_name() == "use_k_tab") {
            if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
                result.successful = false;
                result.reason = "use_k_tab must be a bool";
                return result;
            }
            use_k_tab = parameter.as_bool();
            use_k_tab_changed = true;
        }
    }

    if (lqr_weights_changed &&
        (lqr_controller_ == nullptr ||
         !lqr_controller_->update_lqr_gain(q_diag, r_diag, error))) {
        result.successful = false;
        result.reason = error;
        RCLCPP_ERROR(
            get_node()->get_logger(), "Failed to update LQR gain: %s",
            error.c_str());
        return result;
    }

    if ((gain_table_changed || use_k_tab) &&
        (lqr_controller_ == nullptr ||
         !lqr_controller_->set_gain_table(gain_lengths, gain_values, error))) {
        result.successful = false;
        result.reason = error;
        RCLCPP_ERROR(
            get_node()->get_logger(), "Failed to update LQR gain table: %s",
            error.c_str());
        return result;
    }

    if (use_k_tab_changed) {
        if (lqr_controller_ == nullptr) {
            result.successful = false;
            result.reason = "LQR controller is not initialized";
            return result;
        }
        lqr_controller_->use_k_tab(use_k_tab);
    }

    if (lqr_weights_changed || gain_table_changed || use_k_tab_changed) {
        q_diag_ = q_diag;
        r_diag_ = r_diag;
        gain_lengths_ = gain_lengths;
        gain_values_ = gain_values;
        use_k_tab_ = use_k_tab;
        RCLCPP_INFO(
            get_node()->get_logger(),
            "LQR gain source updated: %s",
            use_k_tab_ ? "K table interpolation" : "online q_diag/r_diag solve");
    }
    return result;
}

}  // namespace lqr_controller

PLUGINLIB_EXPORT_CLASS(
    lqr_controller::LQRController,
    controller_interface::ControllerInterface)
