#include "../../include/controller/controller.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <pluginlib/class_list_macros.hpp>

namespace lqr_controller {

namespace {

constexpr size_t kMotorCount = 6U;
constexpr size_t kTargetInterfacesPerMotor = 5U;
constexpr char kReferencePrefix[] = "mujoco_sim_controller";

Eigen::MatrixXd make_lqr_a()
{
    Eigen::MatrixXd a(6, 6);
    a << 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 0.0, -13.9285, 0.0, 0.6373, 0.0,
         0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
         0.0, 0.0, 98.2488, 0.0, 17.6903, 0.0,
         0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
         0.0, 0.0, 0.0, 44.9938, 0.0, 54.3689;
    return a;
}

Eigen::MatrixXd make_lqr_b()
{
    Eigen::MatrixXd b(6, 2);
    b << 0.0, 0.0,
         15.4595, -3.3942,
         0.0, 0.0,
         -65.5078, 39.5039,
         0.0, 0.0,
         -7.9383, 50.5446;
    return b;
}

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
    : lqr_calc_(make_lqr_a(), make_lqr_b())
{
    auto gain_provider = [this](double leg_length, LqrGainMatrix& gain) {
        return get_lqr_gain(leg_length, gain);
    };

    auto controller = std::make_unique<::Controller>(
        &imu_,
        &lf_motor_,
        &rf_motor_,
        &lb_motor_,
        &rb_motor_,
        &lw_motor_,
        &rw_motor_,
        std::move(gain_provider));
    controller_impl_ = controller.get();
    controller_ = controller.get();
    lqr_controller_ = std::move(controller);
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

    if (!imu_.configure(get_node(), imu_topic_, imu_pose_topic_)) {
        RCLCPP_ERROR(
            get_node()->get_logger(),
            "Failed to configure IMU topics");
        return controller_interface::CallbackReturn::ERROR;
    }

    if (!configure_controller(configuration_error)) {
        RCLCPP_ERROR(
            get_node()->get_logger(),
            "Failed to configure controller: %s",
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
    (void)period;
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
    const bool valid = controller_->update(
        static_cast<uint64_t>(time.nanoseconds() / 1000000LL));
    return valid ? controller_interface::return_type::OK
                 : controller_interface::return_type::ERROR;
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
    if (controller_impl_ == nullptr) {
        error = "LQR controller is not initialized";
        return false;
    }

    if (!controller_impl_->set_params(controller_params_)) {
        error = "invalid controller parameters";
        return false;
    }

    return configure_lqr_gain(error);
}

bool LQRController::configure_lqr_gain(std::string& error)
{
    LqrStateWeight q_diag;
    LqrInputWeight r_diag;
    if (!parameter_to_array(
            get_node()->get_parameter("q_diag"), q_diag, error) ||
        !parameter_to_array(
            get_node()->get_parameter("r_diag"), r_diag, error)) {
        return false;
    }

    LQRCalc::Vector q_vector(static_cast<Eigen::Index>(q_diag.size()));
    LQRCalc::Vector r_vector(static_cast<Eigen::Index>(r_diag.size()));
    for (std::size_t index = 0; index < q_diag.size(); ++index) {
        q_vector[static_cast<Eigen::Index>(index)] = q_diag[index];
    }
    for (std::size_t index = 0; index < r_diag.size(); ++index) {
        r_vector[static_cast<Eigen::Index>(index)] = r_diag[index];
    }

    LQRCalc::Matrix calculated_gain;
    if (!lqr_calc_.calculate(q_vector, r_vector, calculated_gain, error)) {
        return false;
    }

    if (calculated_gain.rows() != 2 || calculated_gain.cols() != 6) {
        error = "6-state LQR calculator returned a gain with invalid dimensions";
        return false;
    }

    LqrGainMatrix online_gain;
    online_gain = calculated_gain;

    std::vector<std::pair<double, LqrGainMatrix>> gain_table;
    if (use_k_tab_ || !gain_lengths_.empty() || !gain_values_.empty()) {
        if (!build_gain_table(gain_lengths_, gain_values_, gain_table, error)) {
            return false;
        }
    }
    if (use_k_tab_ && gain_table.empty()) {
        error = "K.lengths and K.values are required when use_k_tab is true";
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(gain_mutex_);
        online_gain_ = online_gain;
        gain_table_ = std::move(gain_table);
        q_diag_ = q_diag;
        r_diag_ = r_diag;
    }
    return true;
}

bool LQRController::get_lqr_gain(double leg_length, LqrGainMatrix& gain) const
{
    if (!std::isfinite(leg_length)) {
        return false;
    }

    std::lock_guard<std::mutex> lock(gain_mutex_);
    if (!use_k_tab_) {
        gain = online_gain_;
        return gain.allFinite();
    }
    if (gain_table_.empty()) {
        return false;
    }

    if (leg_length <= gain_table_.front().first ||
        gain_table_.size() == 1U) {
        gain = gain_table_.front().second;
    } else if (leg_length >= gain_table_.back().first) {
        gain = gain_table_.back().second;
    } else {
        const auto upper = std::lower_bound(
            gain_table_.begin(),
            gain_table_.end(),
            leg_length,
            [](const auto& entry, double length) {
                return entry.first < length;
            });
        const auto lower = std::prev(upper);
        const double ratio =
            (leg_length - lower->first) / (upper->first - lower->first);
        gain = (1.0 - ratio) * lower->second + ratio * upper->second;
    }
    return gain.allFinite();
}

bool LQRController::build_gain_table(
    const std::vector<double>& lengths,
    const std::vector<double>& values,
    std::vector<std::pair<double, LqrGainMatrix>>& table,
    std::string& error) const
{
    error.clear();
    if (lengths.empty()) {
        error = "K.lengths must contain at least one leg length";
        return false;
    }
    if (values.size() != lengths.size() * 12U) {
        error = "K.values must contain exactly 12 values for each K.lengths entry";
        return false;
    }

    table.clear();
    table.reserve(lengths.size());
    for (std::size_t entry_index = 0; entry_index < lengths.size();
         ++entry_index) {
        const double length = lengths[entry_index];
        if (!std::isfinite(length)) {
            error = "K.lengths contains a non-finite value";
            table.clear();
            return false;
        }

        LqrGainMatrix gain;
        for (std::size_t row = 0; row < 2U; ++row) {
            for (std::size_t column = 0; column < 6U; ++column) {
                const std::size_t value_index =
                    entry_index * 12U + row * 6U + column;
                const double value = values[value_index];
                if (!std::isfinite(value)) {
                    error = "K.values contains a non-finite value";
                    table.clear();
                    return false;
                }
                gain(static_cast<Eigen::Index>(row),
                     static_cast<Eigen::Index>(column)) = value;
            }
        }
        table.emplace_back(length, gain);
    }

    std::sort(
        table.begin(),
        table.end(),
        [](const auto& lhs, const auto& rhs) {
            return lhs.first < rhs.first;
        });
    for (std::size_t index = 1; index < table.size(); ++index) {
        if (table[index].first <= table[index - 1U].first) {
            error = "K.lengths entries must be unique";
            table.clear();
            return false;
        }
    }
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
    LqrGainMatrix calculated_online_gain = LqrGainMatrix::Zero();
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

    if (lqr_weights_changed) {
        LQRCalc::Vector q_vector(static_cast<Eigen::Index>(q_diag.size()));
        LQRCalc::Vector r_vector(static_cast<Eigen::Index>(r_diag.size()));
        for (std::size_t index = 0; index < q_diag.size(); ++index) {
            q_vector[static_cast<Eigen::Index>(index)] = q_diag[index];
        }
        for (std::size_t index = 0; index < r_diag.size(); ++index) {
            r_vector[static_cast<Eigen::Index>(index)] = r_diag[index];
        }

        LQRCalc::Matrix calculated_gain;
        if (!lqr_calc_.calculate(q_vector, r_vector, calculated_gain, error) ||
            calculated_gain.rows() != 2 || calculated_gain.cols() != 6) {
            if (error.empty()) {
                error = "6-state LQR calculator returned a gain with invalid dimensions";
            }
            result.successful = false;
            result.reason = error;
            RCLCPP_ERROR(
                get_node()->get_logger(),
                "Failed to update LQR gain: %s",
                error.c_str());
            return result;
        }
        calculated_online_gain = calculated_gain;
    }

    std::vector<std::pair<double, LqrGainMatrix>> gain_table;
    if (gain_table_changed || use_k_tab_changed) {
        if (use_k_tab || !gain_lengths.empty() || !gain_values.empty()) {
            if (!build_gain_table(
                    gain_lengths, gain_values, gain_table, error)) {
                result.successful = false;
                result.reason = error;
                RCLCPP_ERROR(
                    get_node()->get_logger(),
                    "Failed to update LQR gain table: %s",
                    error.c_str());
                return result;
            }
        }
        if (use_k_tab && gain_table.empty()) {
            result.successful = false;
            result.reason =
                "K.lengths and K.values are required when use_k_tab is true";
            return result;
        }
    }

    if (lqr_weights_changed || gain_table_changed || use_k_tab_changed) {
        std::lock_guard<std::mutex> lock(gain_mutex_);
        if (lqr_weights_changed) {
            online_gain_ = calculated_online_gain;
        }
        q_diag_ = q_diag;
        r_diag_ = r_diag;
        if (gain_table_changed || use_k_tab_changed) {
            gain_lengths_ = gain_lengths;
            gain_values_ = gain_values;
            gain_table_ = std::move(gain_table);
        }
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
