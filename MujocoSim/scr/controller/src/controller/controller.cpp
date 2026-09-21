#include <controller/controller.hpp>

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <mutex>
#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>


#include <pluginlib/class_list_macros.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <robot_interfaces/msg/motor_state.hpp>
#include <robot_interfaces/msg/motor_target.hpp>

using namespace std::chrono_literals;

namespace lqr_controller {

namespace {

constexpr size_t kMotorCount                                    = 6;
constexpr size_t kTargetInterfacesPerMotor                      = 5;
constexpr size_t kLqrStateSize                                  = 6;
constexpr size_t kLqrInputSize                                  = 2;
constexpr const char* kReferencePrefix                          = "mujoco_sim_controller";
constexpr const char* kStateTopic                               = "robot_state";
constexpr const char* kTargetTopic                              = "robot_target";
constexpr const char* kCmdVelTopic                              = "cmd_vel";
constexpr double kHipHalfDistance                               = 0.11;
constexpr double kUpperLinkLength                               = 0.1844;
constexpr double kLowerLinkLength                               = 0.3130;
constexpr double kWheelRadius                                   = 0.1;
constexpr double kWheelSpinIntegralLimit                        = 20.0;
constexpr std::array<const char*, kMotorCount> kMotorJointNames = {
    "left_front_hip_joint", "left_rear_hip_joint", "left_wheel_joint", "right_front_hip_joint", "right_rear_hip_joint", "right_wheel_joint",
};
constexpr std::array<const char*, 3> kStateInterfaceNames                          = {"position", "velocity", "effort"};
constexpr std::array<const char*, kTargetInterfacesPerMotor> kTargetInterfaceNames = {
    "position", "velocity", "effort", "kp", "kd",
};
constexpr std::array<double, kLqrStateSize> kDefaultQDiag = {10.0, 400.0, 100.0, 40.0, 600.0, 50.0};
constexpr std::array<double, kLqrInputSize> kDefaultRDiag = {8.0, 0.5};

robot_interfaces::msg::MotorState& motor_state_at(robot_interfaces::msg::RobotState& msg, const size_t index) {
    switch (index) {
    case 0: return msg.l1;
    case 1: return msg.l2;
    case 2: return msg.lw;
    case 3: return msg.r1;
    case 4: return msg.r2;
    default: return msg.rw;
    }
}

robot_interfaces::msg::MotorTarget& target_at(robot_interfaces::msg::RobotTarget& msg, const size_t index) {
    switch (index) {
    case 0: return msg.l1;
    case 1: return msg.l2;
    case 2: return msg.lw;
    case 3: return msg.r1;
    case 4: return msg.r2;
    default: return msg.rw;
    }
}

template <typename Interface>
Interface* find_interface(std::vector<Interface>& interfaces, const std::string& name) {
    const auto interface =
        std::find_if(interfaces.begin(), interfaces.end(), [&name](const auto& item) { return item.get_name() == name; });
    if (interface == interfaces.end()) {
        return nullptr;
    }
    return &(*interface);
}

double clamp_unit(const double value) { return std::clamp(value, -1.0, 1.0); }

double normalize_angle(const double value) {
    double angle = value;
    while (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    return angle;
}

double normalized_zyx_pitch(const Eigen::Quaterniond& q) {
    const Eigen::Matrix3d rotation = q.toRotationMatrix();
    return std::asin(clamp_unit(-rotation(2, 0)));
}

double low_pass_filter(const double input, const double alpha, double& filtered_value, bool& initialized) {
    if (!std::isfinite(input)) {
        return initialized ? filtered_value : 0.0;
    }

    if (!initialized) {
        filtered_value = input;
        initialized    = true;
        return filtered_value;
    }

    const double clamped_alpha = std::clamp(std::isfinite(alpha) ? alpha : 1.0, 0.0, 1.0);
    filtered_value             = (1.0 - clamped_alpha) * filtered_value + clamped_alpha * input;
    return filtered_value;
}

std::string motor_state_interface_name(const size_t motor_index, const size_t interface_index) {
    return std::string(kMotorJointNames[motor_index]) + "/" + kStateInterfaceNames[interface_index];
}

std::string motor_command_interface_name(const size_t motor_index, const size_t interface_index, const bool use_mujoco_chain) {
    const auto joint_interface = std::string(kMotorJointNames[motor_index]) + "/" + kTargetInterfaceNames[interface_index];
    if (use_mujoco_chain) {
        return std::string(kReferencePrefix) + "/" + joint_interface;
    }
    return joint_interface;
}

template <size_t N>
std::vector<double> array_to_vector(const std::array<double, N>& values) {
    return std::vector<double>(values.begin(), values.end());
}

bool parameter_to_double_vector(const rclcpp::Parameter& param, std::vector<double>& values, std::string& error) {
    if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
        values = param.as_double_array();
        return true;
    }
    if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) {
        const auto int_values = param.as_integer_array();
        values.clear();
        values.reserve(int_values.size());
        for (const auto value : int_values) {
            values.push_back(static_cast<double>(value));
        }
        return true;
    }

    error = param.get_name() + " must be a numeric array";
    return false;
}

template <size_t N>
bool parse_diag_values(
    const std::vector<double>& values, const char* name, const bool strictly_positive, std::array<double, N>& diag, std::string& error) {
    if (values.size() != N) {
        error = std::string(name) + " must contain exactly " + std::to_string(N) + " values";
        return false;
    }

    for (size_t i = 0; i < N; ++i) {
        const double value = values[i];
        if (!std::isfinite(value)) {
            error = std::string(name) + " contains a non-finite value at index " + std::to_string(i);
            return false;
        }
        if (strictly_positive) {
            if (value <= 0.0) {
                error = std::string(name) + " values must be greater than 0";
                return false;
            }
        } else if (value < 0.0) {
            error = std::string(name) + " values must be greater than or equal to 0";
            return false;
        }
        diag[i] = value;
    }
    return true;
}

template <size_t N>
bool get_diag_parameter(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr& node, const char* name, const bool strictly_positive, std::array<double, N>& diag,
    std::string& error) {
    rclcpp::Parameter param;
    if (!node->get_parameter(name, param)) {
        error = std::string("Missing parameter ") + name;
        return false;
    }

    std::vector<double> values;
    if (!parameter_to_double_vector(param, values, error)) {
        return false;
    }
    return parse_diag_values(values, name, strictly_positive, diag, error);
}

} // namespace

LQRController::LQRController()
    : leg(kHipHalfDistance, kUpperLinkLength, kLowerLinkLength) {
    imu_state_.orientation.w = 1.0;
    air_K.setZero();
    K.setZero();
}

controller_interface::CallbackReturn LQRController::on_init() {
    auto node = get_node();
    auto_declare<std::string>("imu_topic", imu_topic_);
    auto_declare<std::string>("imu_pose_topic", imu_pose_topic_);
    auto_declare<double>("torque_limit", torque_limit_);
    auto_declare<double>("leg_angle_diff_kp", leg_angle_diff_kp_);
    auto_declare<double>("leg_angle_diff_kd", leg_angle_diff_kd_);
    auto_declare<double>("wheel_diff_kp", wheel_diff_kp_);
    auto_declare<double>("wheel_diff_ki", wheel_diff_ki_);
    auto_declare<std::vector<double>>("q_diag", array_to_vector(kDefaultQDiag));
    auto_declare<std::vector<double>>("r_diag", array_to_vector(kDefaultRDiag));

    state     = node->declare_parameter<int>("state", 1);
    exp_x     = node->declare_parameter<double>("exp_pos",0.0);
    exp_omega = node->declare_parameter<double>("exp_omega",0.0);


    std::string error;
    if (!get_diag_parameter(node, "q_diag", false, q_diag_, error) || !get_diag_parameter(node, "r_diag", true, r_diag_, error)) {
        RCLCPP_ERROR(node->get_logger(), "Invalid LQR parameters: %s", error.c_str());
        return controller_interface::CallbackReturn::ERROR;
    }

    Eigen::Matrix<double, kLqrInputSize, kLqrStateSize> initial_gain;
    if (!solve_lqr_gain(q_diag_, r_diag_, initial_gain, error)) {
        RCLCPP_ERROR(node->get_logger(), "Failed to solve initial Riccati equation: %s", error.c_str());
        return controller_interface::CallbackReturn::ERROR;
    }
    {
        std::lock_guard<std::mutex> lock(lqr_gain_mutex_);
        K           = initial_gain;
        air_K(1,2)=K(1,2);
        air_K(1,3)=K(1,3);
    }

    last_state_switch_time = get_node()->get_clock()->now();

    param_cb_ = node->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        auto next_q_diag            = q_diag_;
        auto next_r_diag            = r_diag_;
        double next_torque_limit    = torque_limit_;
        double next_leg_diff_kp     = leg_angle_diff_kp_;
        double next_leg_diff_kd     = leg_angle_diff_kd_;
        double next_wheel_diff_kp   = wheel_diff_kp_;
        double next_wheel_diff_ki   = wheel_diff_ki_;
        int next_state              = state;
        bool should_update_lqr_gain = false;
        std::string error;

        for (const auto& param : params) {
            if (param.get_name() == "torque_limit") {
                next_torque_limit = param.as_double();
            } else if (param.get_name() == "leg_angle_diff_kp") {
                next_leg_diff_kp = param.as_double();
            } else if (param.get_name() == "leg_angle_diff_kd") {
                next_leg_diff_kd = param.as_double();
            } else if (param.get_name() == "wheel_diff_kp") {
                next_wheel_diff_kp = param.as_double();
            } else if (param.get_name() == "wheel_diff_ki") {
                next_wheel_diff_ki = param.as_double();
            } else if (param.get_name() == "state") {
                next_state = param.as_int();
            } else if (param.get_name() == "q_diag" || param.get_name() == "r_diag") {
                std::vector<double> values;
                if (!parameter_to_double_vector(param, values, error)) {
                    result.successful = false;
                    result.reason     = error;
                    return result;
                }

                if (param.get_name() == "q_diag") {
                    if (!parse_diag_values(values, "q_diag", false, next_q_diag, error)) {
                        result.successful = false;
                        result.reason     = error;
                        return result;
                    }
                } else if (!parse_diag_values(values, "r_diag", true, next_r_diag, error)) {
                    result.successful = false;
                    result.reason     = error;
                    return result;
                }
                should_update_lqr_gain = true;
            }
            else if(param.get_name() =="exp_pos")
            {
                exp_x=param.as_double();
            }
            else if(param.get_name() =="exp_omega")
            {
                exp_omega=param.as_double();
            }
        }

        Eigen::Matrix<double, kLqrInputSize, kLqrStateSize> next_gain;
        if (should_update_lqr_gain && !solve_lqr_gain(next_q_diag, next_r_diag, next_gain, error)) {
            result.successful = false;
            result.reason     = error;
            return result;
        }

        torque_limit_      = next_torque_limit;
        leg_angle_diff_kp_ = next_leg_diff_kp;
        leg_angle_diff_kd_ = next_leg_diff_kd;
        wheel_diff_kp_     = next_wheel_diff_kp;
        wheel_diff_ki_     = next_wheel_diff_ki;
        state              = next_state;

        if (should_update_lqr_gain) {
            q_diag_ = next_q_diag;
            r_diag_ = next_r_diag;
            {
                std::lock_guard<std::mutex> lock(lqr_gain_mutex_);
                K           = next_gain;
                air_K(1,2)=K(1,2);
                air_K(1,3)=K(1,3);
            }
            RCLCPP_INFO(
                get_node()->get_logger(), "Updated LQR gain K: [%.4f %.4f %.4f %.4f %.4f %.4f; %.4f %.4f %.4f %.4f %.4f %.4f]",
                next_gain(0, 0), next_gain(0, 1), next_gain(0, 2), next_gain(0, 3), next_gain(0, 4), next_gain(0, 5), next_gain(1, 0),
                next_gain(1, 1), next_gain(1, 2), next_gain(1, 3), next_gain(1, 4), next_gain(1, 5));
        }
        return result;
    });

    return controller_interface::CallbackReturn::SUCCESS;
}

bool LQRController::solve_lqr_gain(
    const std::array<double, 6>& q_diag, const std::array<double, 2>& r_diag, Eigen::Matrix<double, 2, 6>& gain, std::string& error) const {
    using Matrix6d  = Eigen::Matrix<double, kLqrStateSize, kLqrStateSize>;
    using Matrix62d = Eigen::Matrix<double, kLqrStateSize, kLqrInputSize>;
    using Matrix2d  = Eigen::Matrix<double, kLqrInputSize, kLqrInputSize>;
    using Matrix12d = Eigen::Matrix<double, 2 * kLqrStateSize, 2 * kLqrStateSize>;
    using Matrix6cd = Eigen::Matrix<std::complex<double>, kLqrStateSize, kLqrStateSize>;

    Matrix6d A;
    A << 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, -13.9285, 0.0, 0.6373, 0.0,
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 98.2488, 0.0, 17.6903, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
    0.0, 0.0, 44.9938, 0.0, 54.3689, 0.0;

    Matrix62d B;
    B << 0.0, 0.0,
    15.4595, -3.3942,
    0.0, 0.0,
    -65.5078, 39.5039,
    0.0, 0.0,
    -7.9383, 50.5446;

    Matrix6d Q = Matrix6d::Zero();
    for (size_t i = 0; i < kLqrStateSize; ++i) {
        Q(i, i) = q_diag[i];
    }

    Matrix2d R_inv = Matrix2d::Zero();
    for (size_t i = 0; i < kLqrInputSize; ++i) {
        if (r_diag[i] <= 0.0 || !std::isfinite(r_diag[i])) {
            error = "r_diag values must be finite and greater than 0";
            return false;
        }
        R_inv(i, i) = 1.0 / r_diag[i];
    }

    Matrix12d H                                                                  = Matrix12d::Zero();
    H.template block<kLqrStateSize, kLqrStateSize>(0, 0)                         = A;
    H.template block<kLqrStateSize, kLqrStateSize>(0, kLqrStateSize)             = -B * R_inv * B.transpose();
    H.template block<kLqrStateSize, kLqrStateSize>(kLqrStateSize, 0)             = -Q;
    H.template block<kLqrStateSize, kLqrStateSize>(kLqrStateSize, kLqrStateSize) = -A.transpose();

    Eigen::ComplexEigenSolver<Matrix12d> eigen_solver(H);
    if (eigen_solver.info() != Eigen::Success) {
        error = "Hamiltonian eigen decomposition failed";
        return false;
    }

    const auto eigenvalues  = eigen_solver.eigenvalues();
    const auto eigenvectors = eigen_solver.eigenvectors();
    std::array<int, kLqrStateSize> stable_indices{};
    size_t stable_count = 0;
    for (int i = 0; i < eigenvalues.size(); ++i) {
        if (eigenvalues[i].real() < -1.0e-8) {
            if (stable_count >= stable_indices.size()) {
                error = "Riccati Hamiltonian has too many stable eigenvalues";
                return false;
            }
            stable_indices[stable_count++] = i;
        }
    }

    if (stable_count != kLqrStateSize) {
        error = "Riccati Hamiltonian did not provide a 6-dimensional stable subspace";
        return false;
    }

    Matrix6cd U1;
    Matrix6cd U2;
    for (size_t col = 0; col < kLqrStateSize; ++col) {
        U1.col(col) = eigenvectors.template block<kLqrStateSize, 1>(0, stable_indices[col]);
        U2.col(col) = eigenvectors.template block<kLqrStateSize, 1>(kLqrStateSize, stable_indices[col]);
    }

    const auto U1_decomposition = U1.fullPivLu();
    if (!U1_decomposition.isInvertible()) {
        error = "Riccati stable subspace is singular";
        return false;
    }

    const Matrix6cd P_complex = U2 * U1.inverse();
    const double max_imag     = P_complex.imag().cwiseAbs().maxCoeff();
    if (max_imag > 1.0e-5) {
        error = "Riccati solution has a significant imaginary component";
        return false;
    }

    Matrix6d P = P_complex.real();
    P          = 0.5 * (P + P.transpose());

    gain = R_inv * B.transpose() * P;
    if (!gain.allFinite()) {
        error = "Computed LQR gain contains a non-finite value";
        return false;
    }

    const Matrix6d residual = A.transpose() * P + P * A - P * B * R_inv * B.transpose() * P + Q;
    const double scale      = 1.0 + Q.norm() + A.norm() * P.norm() + (P * B * R_inv * B.transpose() * P).norm();
    if (!std::isfinite(residual.norm()) || residual.norm() > 1.0e-6 * scale) {
        error = "Riccati residual check failed";
        return false;
    }

    return true;
}

controller_interface::CallbackReturn LQRController::on_configure(const rclcpp_lifecycle::State& previous_state) {
    (void)previous_state;

    imu_topic_            = get_node()->get_parameter("imu_topic").as_string();
    imu_pose_topic_       = get_node()->get_parameter("imu_pose_topic").as_string();
    use_mujoco_sim_chain_ = use_sim_time_parameter();
    torque_limit_         = get_node()->get_parameter("torque_limit").as_double();
    leg_angle_diff_kp_    = get_node()->get_parameter("leg_angle_diff_kp").as_double();
    leg_angle_diff_kd_    = get_node()->get_parameter("leg_angle_diff_kd").as_double();
    wheel_diff_kp_        = get_node()->get_parameter("wheel_diff_kp").as_double();
    wheel_diff_ki_        = get_node()->get_parameter("wheel_diff_ki").as_double();

    robot_exp_vel = get_node()->create_subscription<geometry_msgs::msg::Twist>(
        kCmdVelTopic, 10, [this](const geometry_msgs::msg::Twist& msg) { expected_velocity_ = msg; });
    imu_pose_subscriber_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
        imu_pose_topic_, rclcpp::SensorDataQoS(), [this](const geometry_msgs::msg::PoseStamped& msg) { imu_pose_callback(msg); });
    imu_subscriber_ = get_node()->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, rclcpp::SensorDataQoS(), [this](const sensor_msgs::msg::Imu& msg) { imu_callback(msg); });

    for (size_t i = 0; i < kMotorCount; ++i) {
        auto& target  = target_at(robot_target_, i);
        target.rad    = 0.0F;
        target.omega  = 0.0F;
        target.torque = 0.0F;
    }

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LQRController::on_activate(const rclcpp_lifecycle::State& previous_state) {
    (void)previous_state;

    state_velocity_filtered_.fill(0.0);
    state_velocity_filter_initialized_.fill(false);
    wheel_spin_error_integral_ = 0.0;

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LQRController::on_deactivate(const rclcpp_lifecycle::State& previous_state) {
    (void)previous_state;

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type LQRController::update(const rclcpp::Time& time, const rclcpp::Duration& period) {
    for (size_t motor_index = 0; motor_index < kMotorCount; ++motor_index) {
        const auto position_interface = find_interface(state_interfaces_, motor_state_interface_name(motor_index, 0));
        const auto velocity_interface = find_interface(state_interfaces_, motor_state_interface_name(motor_index, 1));
        const auto effort_interface   = find_interface(state_interfaces_, motor_state_interface_name(motor_index, 2));
        if (position_interface == nullptr || velocity_interface == nullptr || effort_interface == nullptr) {
            RCLCPP_ERROR_THROTTLE(
                get_node()->get_logger(), *get_node()->get_clock(), 1000, "Missing state interfaces for %s", kMotorJointNames[motor_index]);
            return controller_interface::return_type::ERROR;
        }

        auto& state  = motor_state_at(robot_state_, motor_index);
        state.rad    = static_cast<float>(position_interface->get_value());
        state.omega  = static_cast<float>(velocity_interface->get_value());
        state.torque = static_cast<float>(effort_interface->get_value());
    }

    update_motor_commands(time, period);

    for (size_t i = 0; i < kMotorCount; ++i) {
        const auto position_interface = find_interface(command_interfaces_, motor_command_interface_name(i, 0, use_mujoco_sim_chain()));
        const auto velocity_interface = find_interface(command_interfaces_, motor_command_interface_name(i, 1, use_mujoco_sim_chain()));
        const auto effort_interface   = find_interface(command_interfaces_, motor_command_interface_name(i, 2, use_mujoco_sim_chain()));
        const auto kp_interface       = find_interface(command_interfaces_, motor_command_interface_name(i, 3, use_mujoco_sim_chain()));
        const auto kd_interface       = find_interface(command_interfaces_, motor_command_interface_name(i, 4, use_mujoco_sim_chain()));
        if (position_interface == nullptr || velocity_interface == nullptr || effort_interface == nullptr || kp_interface == nullptr
            || kd_interface == nullptr) {
            RCLCPP_ERROR_THROTTLE(
                get_node()->get_logger(), *get_node()->get_clock(), 1000, "Missing command interfaces for %s", kMotorJointNames[i]);
            return controller_interface::return_type::ERROR;
        }

        const auto& target = target_at(robot_target_, i);
        position_interface->set_value(static_cast<double>(target.rad));
        velocity_interface->set_value(static_cast<double>(target.omega));
        effort_interface->set_value(clamp_torque(static_cast<double>(target.torque)));
        kp_interface->set_value(static_cast<double>(target.kp));
        kd_interface->set_value(static_cast<double>(target.kd));
        // kp_interface->set_value(static_cast<double>(target.kp));
        // kd_interface->set_value(static_cast<double>(target.kd));
    }

    // 清零kp和kd指令
    robot_target_.l1.kp = robot_target_.l2.kp = robot_target_.r1.kp = robot_target_.r2.kp = 0.0;
    robot_target_.l1.kd = robot_target_.l2.kd = robot_target_.r1.kd = robot_target_.r2.kd = 0.0;
    robot_target_.lw.kp = robot_target_.rw.kp = 0.0;
    robot_target_.lw.kd = robot_target_.rw.kd = 0.0;

    // 清零torque指令
    robot_target_.l1.torque = robot_target_.l2.torque = robot_target_.r1.torque = robot_target_.r2.torque = 0.0;
    robot_target_.lw.torque = robot_target_.rw.torque = 0.0;

    return controller_interface::return_type::OK;
}

void LQRController::update_motor_commands(const rclcpp::Time& time, const rclcpp::Duration& period) {
    (void)time;

    // 提取机体 pitch。Eigen::eulerAngles(2, 1, 0) 可能把接近直立的姿态表示成 pitch 接近 pi。
    Eigen::Quaterniond q;
    q.w() = imu_state_.orientation.w;
    q.x() = imu_state_.orientation.x;
    q.y() = imu_state_.orientation.y;
    q.z() = imu_state_.orientation.z;
    q.normalize();
    const double pitch = normalized_zyx_pitch(q);
    // RCLCPP_INFO_THROTTLE(get_node()->get_logger(),*get_node()->get_clock(),100,"(r=%.4f,p=%.4f,y=%.4f)",rpy[0],rpy[1],rpy[2]);

    // 提取等效摆的角度和长度
    Eigen::Vector2d left_joint_pos(robot_state_.l1.rad, robot_state_.l2.rad), left_leg_pos(0.0, 0.0);
    const bool left_leg_ok = leg.forward_kinematics(left_joint_pos, left_leg_pos);
    Eigen::Vector2d left_joint_vel(robot_state_.l1.omega, robot_state_.l2.omega), left_leg_vel(0.0, 0.0);
    if (left_leg_ok) {
        leg.forward_velocity(left_joint_pos, left_joint_vel, left_leg_vel);
    }

    Eigen::Vector2d right_joint_pos(robot_state_.r1.rad, robot_state_.r2.rad), right_leg_pos(0.0, 0.0);
    const bool right_leg_ok = leg.forward_kinematics(right_joint_pos, right_leg_pos);
    Eigen::Vector2d right_joint_vel(robot_state_.r1.omega, robot_state_.r2.omega), right_leg_vel(0.0, 0.0);
    if (right_leg_ok) {
        leg.forward_velocity(right_joint_pos, right_joint_vel, right_leg_vel);
    }

    // 准备状态空间方程相关数据
    const double leg_angle     = 0.5 * (left_leg_pos[1] + right_leg_pos[1]);
    const double leg_angle_vel = 0.5 * (left_leg_vel[1] + right_leg_vel[1]);
    double x                   = 0.5 * (robot_state_.lw.rad + robot_state_.rw.rad) * kWheelRadius;
    double dx                  = 0.5 * (robot_state_.lw.omega + robot_state_.rw.omega) * kWheelRadius;
    double theta               = normalize_angle(pitch + leg_angle);
    double dtheta              = imu_state_.angular_velocity.y + leg_angle_vel; // 将平均腿摆速度加入解算
    double phi                 = -pitch;
    double dphi                = -imu_state_.angular_velocity.y;

    dx     = low_pass_filter(dx, 0.07, state_velocity_filtered_[0], state_velocity_filter_initialized_[0]);
    dtheta = low_pass_filter(dtheta, 0.7, state_velocity_filtered_[1], state_velocity_filter_initialized_[1]);
    dphi   = low_pass_filter(dphi, 0.7, state_velocity_filtered_[2], state_velocity_filter_initialized_[2]);

    // 支撑力计算
    Eigen::Vector2d left_leg_force, right_leg_force;
    left_leg_force.setZero();
    right_leg_force.setZero();
    leg.forward_dynamics(left_joint_pos, Eigen::Vector2d(robot_state_.l1.torque, robot_state_.l2.torque), left_leg_force);
    leg.forward_dynamics(right_joint_pos, Eigen::Vector2d(robot_state_.r1.torque, robot_state_.r2.torque), right_leg_force);


    //评估机器人状态，选择控制策略
    auto now = get_node()->get_clock()->now();
    if (pitch > 0.3 || pitch < -0.3) {                                // 姿态极大倾斜，系统已经失稳
        if (now - last_state_switch_time > 300ms)                     // 上一次状态切换在300ms前
        {
            if (state != 1)
                last_state_switch_time = now;

            state = 1;
        }
    } else if (left_leg_force[0] > 8.0 && right_leg_force[0] > 8.0) { // 左右腿与地面的压力都要大于8N，才认为可控
        if (now - last_state_switch_time > 300ms)                     // 上一次状态切换在300ms前
        {
            if (state != 2)
                last_state_switch_time = now;
            state = 2;
        }
    } else {
        if (now - last_state_switch_time > 300ms)                     // 上一次状态切换在300ms前
        {
            if (state != 3)
                last_state_switch_time = now;
            state = 3;
        }
    }


    if(state==0)
    {
        u.setZero();
    }else if(state==1)
    {
        u.setZero();
    }else if(state==2)
    {
        Eigen::Vector<double, 6> X, exp_X;
        exp_X.setZero();

        if(exp_x-x>5.0)         //防止x数值爆炸
            exp_X[0]=x+5.0;
        else if(exp_x-x<-5.0)
            exp_X[0]=x-5.0;
        else
            exp_X[0]=exp_x;

        X << x, dx, theta, dtheta, phi, dphi; //填写状态向量

        lqr_gain_mutex_.lock();
        u = K * (exp_X - X);
        lqr_gain_mutex_.unlock();
    }
    else if(state==3)
    {
        Eigen::Vector<double, 6> X;
        X << x, dx, theta, dtheta, phi, dphi; //填写状态向量
        lqr_gain_mutex_.lock();
        u = -air_K*X;
        lqr_gain_mutex_.unlock();
    }

    
    // 腿长VMC部分，计算关节为了维持当前腿长所需要施加的力矩
    double left_leg_dis_vmc_T           = vmc_kp * (0.3 - left_leg_pos[0]) - vmc_kd * left_leg_vel[0];
    double right_leg_dis_vmc_T          = vmc_kp * (0.3 - right_leg_pos[0]) - vmc_kd * right_leg_vel[0];
    const double leg_angle_diff         = normalize_angle(left_leg_pos[1] - right_leg_pos[1]);
    const double leg_angle_diff_vel     = left_leg_vel[1] - right_leg_vel[1];
    const double spin_omega             = imu_state_.angular_velocity.z;
    const double spin_error             = exp_omega - spin_omega;
    if (state == 2) {
        wheel_spin_error_integral_ = std::clamp(
            wheel_spin_error_integral_ + spin_error * period.seconds(), -kWheelSpinIntegralLimit, kWheelSpinIntegralLimit);
    } else {
        wheel_spin_error_integral_ = 0.0;
    }
    const double spin_control_torque_diff =
        state == 2 ? wheel_diff_kp_ * spin_error + wheel_diff_ki_ * wheel_spin_error_integral_ : 0.0;
    const double average_leg_length     = 0.5 * (left_leg_pos[0] + right_leg_pos[0]);
    const double spin_compensation_torque = -spin_control_torque_diff * average_leg_length / kWheelRadius;
    const double leg_angle_sync_torque  =
        -leg_angle_diff_kp_ * leg_angle_diff - leg_angle_diff_kd_ * leg_angle_diff_vel + spin_compensation_torque;
    const double left_leg_angle_torque  = u[1] + leg_angle_sync_torque;
    const double right_leg_angle_torque = u[1] - leg_angle_sync_torque;
    const double left_wheel_torque      = u[0] - spin_control_torque_diff;
    const double right_wheel_torque     = u[0] + spin_control_torque_diff;

    //VMC映射
    Eigen::Vector2d left_torque, right_torque;
    leg.inverse_dynamics(left_joint_pos, Eigen::Vector2d(left_leg_dis_vmc_T, left_leg_angle_torque), left_torque);
    leg.inverse_dynamics(right_joint_pos, Eigen::Vector2d(right_leg_dis_vmc_T, right_leg_angle_torque), right_torque);

    //设置各关节期望力矩
    robot_target_.l1.torque = std::clamp<double>(left_torque[0], -12.0, 12.0);
    robot_target_.l2.torque = std::clamp<double>(left_torque[1], -12.0, 12.0);
    robot_target_.r1.torque = std::clamp<double>(right_torque[0], -12.0, 12.0);
    robot_target_.r2.torque = std::clamp<double>(right_torque[1], -12.0, 12.0);
    robot_target_.lw.torque = std::clamp<double>(left_wheel_torque, -10.0, 10.0);
    robot_target_.rw.torque = std::clamp<double>(right_wheel_torque, -10.0, 10.0);
    
    if (state == 0)        // VMC测试
    {
        leg.inverse_dynamics(left_joint_pos, Eigen::Vector2d(left_leg_dis_vmc_T, leg_angle_sync_torque), left_torque);
        leg.inverse_dynamics(right_joint_pos, Eigen::Vector2d(right_leg_dis_vmc_T, -leg_angle_sync_torque), right_torque);
        robot_target_.l1.torque = std::clamp<double>(left_torque[0], -12.0, 12.0);
        robot_target_.l2.torque = std::clamp<double>(left_torque[1], -12.0, 12.0);
        robot_target_.r1.torque = std::clamp<double>(right_torque[0], -12.0, 12.0);
        robot_target_.r2.torque = std::clamp<double>(right_torque[1], -12.0, 12.0);
    } else if (state == 1) // 位控控制
    {
        Eigen::Vector2d rad = {0.0, 0.0};
        if (leg.inverse_kinematics({0.27, 0.0}, rad)) {
            robot_target_.l1.rad = robot_target_.r1.rad = static_cast<float>(rad[0]);
            robot_target_.l2.rad = robot_target_.r2.rad = static_cast<float>(rad[1]);
        }
        robot_target_.l1.kp = robot_target_.l2.kp = robot_target_.r1.kp = robot_target_.r2.kp = 50.0;
        robot_target_.l1.kd = robot_target_.l2.kd = robot_target_.r1.kd = robot_target_.r2.kd = 2.0;
    } else if (state == 2) // 平衡控制
    {
        
    } else if (state == 3) // 离地状态
    {
    //     lqr_gain_mutex_.lock();
    //     u = air_K * (exp_X - X);
    //     lqr_gain_mutex_.unlock();
    //     robot_target_.l1.torque = std::clamp<double>(left_torque[0], -12.0, 12.0);
    //     robot_target_.l2.torque = std::clamp<double>(left_torque[1], -12.0, 12.0);
    //     robot_target_.r1.torque = std::clamp<double>(right_torque[0], -12.0, 12.0);
    //     robot_target_.r2.torque = std::clamp<double>(right_torque[1], -12.0, 12.0);
    }

    RCLCPP_INFO_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 100,
        "state=%d\nF=(%.4f,%.4f)\nu:(T=%.4f,Tp=%.4f)\nlength=(%.4f,%.4f)",
        state, left_leg_force[0], right_leg_force[0],u[0], u[1],left_leg_pos[0],right_leg_pos[0]);
}

void LQRController::imu_pose_callback(const geometry_msgs::msg::PoseStamped& msg) {
    imu_state_.orientation.x = msg.pose.orientation.x;
    imu_state_.orientation.y = msg.pose.orientation.y;
    imu_state_.orientation.z = msg.pose.orientation.z;
    imu_state_.orientation.w = msg.pose.orientation.w;
}

void LQRController::imu_callback(const sensor_msgs::msg::Imu& msg) {
    imu_state_.angular_velocity    = msg.angular_velocity;
    imu_state_.linear_acceleration = msg.linear_acceleration;
}

double LQRController::clamp_torque(const double value) const {
    if (!std::isfinite(value)) {
        return 0.0;
    }
    return std::clamp(value, -torque_limit_, torque_limit_);
}

bool LQRController::use_mujoco_sim_chain() const { return use_mujoco_sim_chain_; }

bool LQRController::use_sim_time_parameter() const {
    rclcpp::Parameter use_sim_time;
    if (get_node()->get_parameter("use_sim_time", use_sim_time)) {
        return use_sim_time.as_bool();
    }
    return false;
}

controller_interface::InterfaceConfiguration LQRController::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration cfg;
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto& joint_name : kMotorJointNames) {
        const auto name = std::string(joint_name);
        if (use_mujoco_sim_chain()) {
            const auto prefix = std::string(kReferencePrefix) + "/" + name;
            cfg.names.push_back(prefix + "/position");
            cfg.names.push_back(prefix + "/velocity");
            cfg.names.push_back(prefix + "/effort");
            cfg.names.push_back(prefix + "/kp");
            cfg.names.push_back(prefix + "/kd");
        } else {
            cfg.names.push_back(name + "/position");
            cfg.names.push_back(name + "/velocity");
            cfg.names.push_back(name + "/effort");
            cfg.names.push_back(name + "/kp");
            cfg.names.push_back(name + "/kd");
        }
    }
    return cfg;
}

controller_interface::InterfaceConfiguration LQRController::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration cfg;
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto& joint_name : kMotorJointNames) {
        const auto name = std::string(joint_name);
        cfg.names.push_back(name + "/position");
        cfg.names.push_back(name + "/velocity");
        cfg.names.push_back(name + "/effort");
    }

    return cfg;
}


} // namespace lqr_controller

PLUGINLIB_EXPORT_CLASS(lqr_controller::LQRController, controller_interface::ControllerInterface)
