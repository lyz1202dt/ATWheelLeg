#include "../../include/controller/controller_at.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <limits>
#include <memory>
#include <rclcpp/logging.hpp>
#include <sstream>
#include <string>
#include <vector>

#include <pluginlib/class_list_macros.hpp>

#include "../../../../../Core/inc/controller/controller_at.hpp"

namespace lqr_controller {

namespace {

constexpr size_t kMotorCount               = 6U;
constexpr size_t kTargetInterfacesPerMotor = 5U;
constexpr char kReferencePrefix[]          = "mujoco_sim_controller";
constexpr size_t kLqrStateSize             = 10U;
constexpr size_t kLqrInputSize             = 4U;

template <typename Interface>
Interface* find_interface(std::vector<Interface>& interfaces, const std::string& name) {
    const auto iterator =
        std::find_if(interfaces.begin(), interfaces.end(), [&name](const auto& interface) { return interface.get_name() == name; });
    return iterator == interfaces.end() ? nullptr : &(*iterator);
}

inline float rad2angle(float rad) { return rad * 180.0f / 3.14159265f; }
inline float angle2rad(float rad) { return rad * 3.14159265f / 180.0f; }

template <size_t Size>
bool parameter_to_array(const rclcpp::Parameter& parameter,
                        std::array<float, Size>& values,
                        std::string& error) {
    std::vector<double> parameter_values;
    if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
        parameter_values = parameter.as_double_array();
    } else if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) {
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
        const float value = static_cast<float>(parameter_values[index]);
        if (!std::isfinite(parameter_values[index]) || !std::isfinite(value)) {
            error = parameter.get_name() + " contains a non-finite value";
            return false;
        }
        values[index] = value;
    }
    return true;
}

std::string lqr_gain_to_string(const Eigen::Matrix<double, kLqrInputSize, kLqrStateSize>& gain) {
    static const Eigen::IOFormat matrix_format(
        Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n", "[", "]", "[", "]");
    std::ostringstream stream;
    stream << gain.format(matrix_format);
    return stream.str();
}
} // namespace

LQRControllerAT::LQRControllerAT() {
    lf_motor_.inverse = false;
    lb_motor_.inverse = false;
    lw_motor_.inverse = false;
    rf_motor_.inverse = false;
    rb_motor_.inverse = false;
    rw_motor_.inverse = false;
    lf_motor_.offset  = 1.171866325;  // angle2rad(67.143f);
    rf_motor_.offset  = 1.171866325;  // angle2rad(67.143f);
    lb_motor_.offset  = -1.136586325; // angle2rad(-65.1216f);
    rb_motor_.offset  = -1.136586325; // angle2rad(-65.1216f);

    auto gain_scheduler = [this](const double& left_leg_length,
                                 const double& right_leg_length,
                                 Eigen::Matrix<double, 4, 10>& gain) {
        (void)left_leg_length;
        (void)right_leg_length;

        if (!K_.allFinite()) {
            return false;
        }
        gain = K_;
        return gain.allFinite();
    };

    auto controller = std::make_unique<::ControllerAT>(
        &imu_, &lf_motor_, &rf_motor_, &lb_motor_, &rb_motor_, &lw_motor_, &rw_motor_, gain_scheduler);
    controller_at_ = controller.get();
    controller->set_debug_logger(
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
    auto_declare<std::vector<double>>(
        "q_diag",
        std::vector<double>(q_diag_.begin(), q_diag_.end()));
    auto_declare<std::vector<double>>(
        "r_diag",
        std::vector<double>(r_diag_.begin(), r_diag_.end()));

    parameter_callback_handle_ = get_node()->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& parameters) {
            return on_set_parameters(parameters);
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

    std::string lqr_error;
    if (!configure_lqr_gain(lqr_error)) {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to configure AT LQR gain: %s", lqr_error.c_str());
        return controller_interface::CallbackReturn::ERROR;
    }

    if (!std::isfinite(effort_limit_) || effort_limit_ <= 0.0 || requested_mode_ < 0 || requested_mode_ > 2
        || !imu_.configure(get_node(), imu_topic_, imu_pose_topic_)) {
        RCLCPP_ERROR(get_node()->get_logger(), "Invalid AT controller parameters or IMU configuration");
        return controller_interface::CallbackReturn::ERROR;
    }

    cmd_vel_subscriber_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
        cmd_vel_topic_, 10, [this](const geometry_msgs::msg::Twist& msg) { cmd_vel_callback(msg); });
    return controller_interface::CallbackReturn::SUCCESS;
}

bool LQRControllerAT::configure_lqr_gain(std::string& error) {
    std::array<float, kLqrStateSize> q_diag;
    std::array<float, kLqrInputSize> r_diag;
    if (!parameter_to_array(get_node()->get_parameter("q_diag"), q_diag, error) ||
        !parameter_to_array(get_node()->get_parameter("r_diag"), r_diag, error)) {
        return false;
    }

    Eigen::Vector<float, kLqrStateSize> q;
    Eigen::Vector<float, kLqrInputSize> r;
    for (size_t index = 0; index < kLqrStateSize; ++index) {
        q[static_cast<Eigen::Index>(index)] = q_diag[index];
    }
    for (size_t index = 0; index < kLqrInputSize; ++index) {
        r[static_cast<Eigen::Index>(index)] = r_diag[index];
    }

    if (controller_at_ == nullptr || !update_lqr_k(q, r)) {
        error = "ControllerAT rejected q_diag/r_diag";
        return false;
    }

    RCLCPP_INFO(
        get_node()->get_logger(),
        "AT LQR K matrix configured:\n%s",
        lqr_gain_to_string(K_).c_str());

    q_diag_ = q_diag;
    r_diag_ = r_diag;
    return true;
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
    (void)period;
    if (!read_motor_states() || controller_ == nullptr) {
        RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000, "Invalid AT motor state or controller");
        return controller_interface::return_type::ERROR;
    }

    controller_->input(
        expected_velocity_.load(std::memory_order_relaxed), expected_omega_.load(std::memory_order_relaxed), 0.2F, requested_mode_);
    controller_->update(static_cast<uint64_t>(time.nanoseconds() / 1000000LL));
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

rcl_interfaces::msg::SetParametersResult LQRControllerAT::on_set_parameters(
    const std::vector<rclcpp::Parameter>& parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    auto q_diag = q_diag_;
    auto r_diag = r_diag_;
    bool lqr_weights_changed = false;
    std::string error;

    for (const auto& parameter : parameters) {
        const auto& parameter_name = parameter.get_name();
        if (parameter_name == "q_diag") {
            if (!parameter_to_array(parameter, q_diag, error)) {
                result.successful = false;
                result.reason = error;
                return result;
            }
            lqr_weights_changed = true;
        } else if (parameter_name == "r_diag") {
            if (!parameter_to_array(parameter, r_diag, error)) {
                result.successful = false;
                result.reason = error;
                return result;
            }
            lqr_weights_changed = true;
        } else if (parameter_name == "expected_velocity") {
            expected_velocity_ = static_cast<float>(parameter.as_double());
        } else if (parameter_name == "expected_omega") {
            expected_omega_ = static_cast<float>(parameter.as_double());
        } else if (parameter_name == "mode") {
            requested_mode_ = static_cast<int>(parameter.as_int());
        }
    }

    if (lqr_weights_changed) {
        Eigen::Vector<float, kLqrStateSize> q;
        Eigen::Vector<float, kLqrInputSize> r;
        for (size_t index = 0; index < kLqrStateSize; ++index) {
            q[static_cast<Eigen::Index>(index)] = q_diag[index];
        }
        for (size_t index = 0; index < kLqrInputSize; ++index) {
            r[static_cast<Eigen::Index>(index)] = r_diag[index];
        }

        if (controller_at_ == nullptr || !update_lqr_k(q, r)) {
            result.successful = false;
            result.reason = "ControllerAT rejected q_diag/r_diag";
            RCLCPP_ERROR(get_node()->get_logger(), "%s", result.reason.c_str());
            return result;
        }
        q_diag_ = q_diag;
        r_diag_ = r_diag;
            RCLCPP_INFO(
            get_node()->get_logger(),
            "AT LQR K matrix updated from q_diag/r_diag:\n%s",
            lqr_gain_to_string(K_).c_str());
    }

    return result;
}


bool LQRControllerAT::update_lqr_k(const Eigen::Vector<float, 10>& Q,
                                const Eigen::Vector<float, 4>& R) {
    using Matrix10d  = Eigen::Matrix<double, 10, 10>;
    using Matrix4d   = Eigen::Matrix<double, 4, 4>;
    using Matrix20d  = Eigen::Matrix<double, 20, 20>;
    using Matrix10cd = Eigen::Matrix<std::complex<double>, 10, 10>;

    Matrix10d A;
    A <<
        0.00000000, 1.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000,
        0.00000000, 0.00000000, 0.00000000, 0.00000000, -13.55160363, 0.00000000, -13.55160363, 0.00000000, 0.00000000, 0.00000000,
        0.00000000, 0.00000000, 0.00000000, 1.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000,
        0.00000000, 0.00000000, 0.00000000, 0.00000000, -2.14332607, 0.00000000, 2.14332607, 0.00000000, 0.00000000, 0.00000000,
        0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 1.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000,
        0.00000000, 0.00000000, 0.00000000, 0.00000000, 213.48521397, 0.00000000, -49.86561124, 0.00000000, 0.00000000, 0.00000000,
        0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 1.00000000, 0.00000000, 0.00000000,
        0.00000000, 0.00000000, 0.00000000, 0.00000000, -49.86561124, 0.00000000, 213.48521397, 0.00000000, 0.00000000, 0.00000000,
        0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 1.00000000,
        0.00000000, 0.00000000, 0.00000000, 0.00000000, 20.24352472, 0.00000000, 20.24352472, 0.00000000, -28.77747990, 0.00000000;

    Eigen::Matrix<double, 10, 4> B;
    B <<
        0.00000000, 0.00000000, 0.00000000, 0.00000000,
        5.75310641, 5.75310641, -0.93168984, -0.93168984,
        0.00000000, 0.00000000, 0.00000000, 0.00000000,
        -3.99485616, 3.99485616, -0.14735637, 0.14735637,
        0.00000000, 0.00000000, 0.00000000, 0.00000000,
        -71.63664239, 17.64525726, 14.67737768, -3.42832366,
        0.00000000, 0.00000000, 0.00000000, 0.00000000,
        17.64525726, -71.63664239, -3.42832366, 14.67737768,
        0.00000000, 0.00000000, 0.00000000, 0.00000000,
        -2.92121265, -2.92121265, -5.28991389, -5.28991389;

    Matrix10d state_cost = Matrix10d::Zero();
    for (Eigen::Index index = 0; index < 10; ++index) {
        const double weight = static_cast<double>(Q[index]);
        if (!std::isfinite(weight) || weight < 0.0) {
            return false;
        }
        state_cost(index, index) = weight;
    }

    Matrix4d input_cost_inverse = Matrix4d::Zero();
    for (Eigen::Index index = 0; index < 4; ++index) {
        const double weight = static_cast<double>(R[index]);
        if (!std::isfinite(weight) || weight <= 0.0) {
            return false;
        }
        input_cost_inverse(index, index) = 1.0 / weight;
    }

    Matrix20d hamiltonian = Matrix20d::Zero();
    hamiltonian.template block<10, 10>(0, 0) = A;
    hamiltonian.template block<10, 10>(0, 10) =
        -B * input_cost_inverse * B.transpose();
    hamiltonian.template block<10, 10>(10, 0) = -state_cost;
    hamiltonian.template block<10, 10>(10, 10) = -A.transpose();

    Eigen::ComplexEigenSolver<Matrix20d> eigen_solver(hamiltonian);
    if (eigen_solver.info() != Eigen::Success) {
        return false;
    }

    const auto eigenvalues = eigen_solver.eigenvalues();
    const auto eigenvectors = eigen_solver.eigenvectors();
    std::array<int, 10> stable_indices{};
    Eigen::Index stable_count = 0;
    for (Eigen::Index index = 0; index < eigenvalues.size(); ++index) {
        if (eigenvalues[index].real() < -1.0e-8) {
            if (stable_count >= static_cast<Eigen::Index>(stable_indices.size())) {
                return false;
            }
            stable_indices[static_cast<std::size_t>(stable_count++)] =
                static_cast<int>(index);
        }
    }

    if (stable_count != 10) {
        return false;
    }

    Matrix10cd u1;
    Matrix10cd u2;
    for (Eigen::Index column = 0; column < 10; ++column) {
        u1.col(column) =
            eigenvectors.template block<10, 1>(0, stable_indices[column]);
        u2.col(column) =
            eigenvectors.template block<10, 1>(10, stable_indices[column]);
    }

    const auto u1_decomposition = u1.fullPivLu();
    if (!u1_decomposition.isInvertible()) {
        return false;
    }

    const Matrix10cd p_complex = u2 * u1.inverse();
    const double max_imaginary = p_complex.imag().cwiseAbs().maxCoeff();
    if (!std::isfinite(max_imaginary) || max_imaginary > 1.0e-5) {
        return false;
    }

    Matrix10d p = p_complex.real();
    p = 0.5 * (p + p.transpose());

    const Eigen::Matrix<double, 4, 10> gain =
        input_cost_inverse * B.transpose() * p;
    if (!gain.allFinite()) {
        return false;
    }

    const Matrix10d residual =
        A.transpose() * p + p * A -
        p * B * input_cost_inverse * B.transpose() * p + state_cost;
    const double residual_norm = residual.norm();
    const double residual_scale =
        1.0 + state_cost.norm() + A.norm() * p.norm() +
        (p * B * input_cost_inverse * B.transpose() * p).norm();
    if (!std::isfinite(residual_norm) ||
        !std::isfinite(residual_scale) ||
        residual_norm > 1.0e-6 * residual_scale) {
        return false;
    }

    K_ = gain;
    return true;
}

} // namespace lqr_controller

PLUGINLIB_EXPORT_CLASS(lqr_controller::LQRControllerAT, controller_interface::ControllerInterface)
