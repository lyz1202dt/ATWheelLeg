#include <controller/mpc_controller.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <mutex>
#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <tinympc/tiny_api.hpp>

#include <pluginlib/class_list_macros.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <robot_interfaces/msg/motor_state.hpp>
#include <robot_interfaces/msg/motor_target.hpp>

using namespace std::chrono_literals;

namespace mpc_controller {

namespace {

constexpr size_t kMotorCount                                    = 6;
constexpr size_t kTargetInterfacesPerMotor                      = 5;
constexpr size_t kMpcStateSize                                  = 6;
constexpr size_t kMpcInputSize                                  = 2;
constexpr int kMpcHorizon                                       = 10;
constexpr double kMpcDt                                         = 0.002;
constexpr double kMpcRho                                        = 1.0;
constexpr int kMpcMaxIter                                       = 50;
constexpr double kMpcAbsTol                                     = 2.0e-3;
constexpr const char* kReferencePrefix                          = "mujoco_sim_controller";
constexpr double kHipHalfDistance                               = 0.11;
constexpr double kUpperLinkLength                               = 0.1844;
constexpr double kLowerLinkLength                               = 0.3130;
constexpr double kWheelRadius                                   = 0.1;
constexpr double kWheelSpinIntegralLimit                        = 20.0;
constexpr double kAirThetaKp                                    = 6.5;
constexpr double kAirThetaKd                                    = 2.8;
constexpr std::array<const char*, kMotorCount> kMotorJointNames = {
    "left_front_hip_joint", "left_rear_hip_joint", "left_wheel_joint", "right_front_hip_joint", "right_rear_hip_joint", "right_wheel_joint",
};
constexpr std::array<const char*, 3> kStateInterfaceNames                          = {"position", "velocity", "effort"};
constexpr std::array<const char*, kTargetInterfacesPerMotor> kTargetInterfaceNames = {
    "position", "velocity", "effort", "kp", "kd",
};
constexpr std::array<double, kMpcStateSize> kDefaultQDiag = {10.0, 400.0, 100.0, 40.0, 600.0, 50.0};
constexpr std::array<double, kMpcInputSize> kDefaultRDiag = {8.0, 0.5};

using MpcStateVector = Eigen::Matrix<double, kMpcStateSize, 1>;

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

void delete_tiny_solver(TinySolver* solver) {
    if (solver == nullptr) {
        return;
    }

    delete solver->solution;
    delete solver->settings;
    delete solver->cache;
    delete solver->work;
    delete solver;
}

} // namespace

MPCController::MPCController()
    : leg(kHipHalfDistance, kUpperLinkLength, kLowerLinkLength) {
    imu_state_.orientation.w = 1.0;
    u.setZero();
}

MPCController::~MPCController() { release_mpc_solver(); }

controller_interface::CallbackReturn MPCController::on_init() {
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
        RCLCPP_ERROR(node->get_logger(), "Invalid MPC parameters: %s", error.c_str());
        return controller_interface::CallbackReturn::ERROR;
    }

    if (!configure_mpc_solver(q_diag_, r_diag_, torque_limit_, error)) {
        RCLCPP_ERROR(node->get_logger(), "Failed to configure TinyMPC solver: %s", error.c_str());
        return controller_interface::CallbackReturn::ERROR;
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
        bool should_update_mpc_solver = false;
        std::string error;

        for (const auto& param : params) {
            if (param.get_name() == "torque_limit") {
                next_torque_limit = param.as_double();
                if (!std::isfinite(next_torque_limit) || next_torque_limit <= 0.0) {
                    result.successful = false;
                    result.reason     = "torque_limit must be finite and greater than 0";
                    return result;
                }
                should_update_mpc_solver = true;
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
                should_update_mpc_solver = true;
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

        if (should_update_mpc_solver && !configure_mpc_solver(next_q_diag, next_r_diag, next_torque_limit, error)) {
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

        if (should_update_mpc_solver) {
            q_diag_ = next_q_diag;
            r_diag_ = next_r_diag;
            RCLCPP_INFO(
                get_node()->get_logger(), "Updated TinyMPC weights q=[%.4f %.4f %.4f %.4f %.4f %.4f], r=[%.4f %.4f]",
                q_diag_[0], q_diag_[1], q_diag_[2], q_diag_[3], q_diag_[4], q_diag_[5], r_diag_[0], r_diag_[1]);
        }
        return result;
    });

    return controller_interface::CallbackReturn::SUCCESS;
}

bool MPCController::configure_mpc_solver(
    const std::array<double, 6>& q_diag, const std::array<double, 2>& r_diag, const double input_torque_limit, std::string& error) {
    if (!std::isfinite(input_torque_limit) || input_torque_limit <= 0.0) {
        error = "input torque limit must be finite and greater than 0";
        return false;
    }

    Eigen::Matrix<double, kMpcStateSize, kMpcStateSize> A_cont;
    A_cont << 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, -13.9285, 0.0, 0.6373, 0.0,
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 98.2488, 0.0, 17.6903, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
    0.0, 0.0, 44.9938, 0.0, 54.3689, 0.0;

    Eigen::Matrix<double, kMpcStateSize, kMpcInputSize> B_cont;
    B_cont << 0.0, 0.0,
    15.4595, -3.3942,
    0.0, 0.0,
    -65.5078, 39.5039,
    0.0, 0.0,
    -7.9383, 50.5446;

    tinyMatrix Adyn = tinyMatrix::Identity(kMpcStateSize, kMpcStateSize) + kMpcDt * A_cont;
    tinyMatrix Bdyn = kMpcDt * B_cont;
    tinyVector fdyn = tinyVector::Zero(kMpcStateSize);

    tinyMatrix Q = tinyMatrix::Zero(kMpcStateSize, kMpcStateSize);
    for (size_t i = 0; i < kMpcStateSize; ++i) {
        if (!std::isfinite(q_diag[i]) || q_diag[i] < 0.0) {
            error = "q_diag values must be finite and greater than or equal to 0";
            return false;
        }
        Q(i, i) = q_diag[i];
    }

    tinyMatrix R = tinyMatrix::Zero(kMpcInputSize, kMpcInputSize);
    for (size_t i = 0; i < kMpcInputSize; ++i) {
        if (!std::isfinite(r_diag[i]) || r_diag[i] <= 0.0) {
            error = "r_diag values must be finite and greater than 0";
            return false;
        }
        R(i, i) = r_diag[i];
    }

    TinySolver* next_solver = nullptr;
    int status = tiny_setup(&next_solver, Adyn, Bdyn, fdyn, Q, R, kMpcRho, kMpcStateSize, kMpcInputSize, kMpcHorizon, 0);
    if (status != 0 || next_solver == nullptr) {
        delete_tiny_solver(next_solver);
        error = "tiny_setup failed";
        return false;
    }

    tinyMatrix x_min = tinyMatrix::Constant(kMpcStateSize, kMpcHorizon, -1.0e17);
    tinyMatrix x_max = tinyMatrix::Constant(kMpcStateSize, kMpcHorizon, 1.0e17);
    tinyMatrix u_min = tinyMatrix::Zero(kMpcInputSize, kMpcHorizon - 1);
    tinyMatrix u_max = tinyMatrix::Zero(kMpcInputSize, kMpcHorizon - 1);
    u_min.row(0).setConstant(-10.0);
    u_max.row(0).setConstant(10.0);
    u_min.row(1).setConstant(-input_torque_limit);
    u_max.row(1).setConstant(input_torque_limit);

    status = tiny_set_bound_constraints(next_solver, x_min, x_max, u_min, u_max);
    if (status != 0) {
        delete_tiny_solver(next_solver);
        error = "tiny_set_bound_constraints failed";
        return false;
    }

    next_solver->settings->max_iter          = kMpcMaxIter;
    next_solver->settings->check_termination = kMpcMaxIter + 1;
    next_solver->settings->abs_pri_tol       = kMpcAbsTol;
    next_solver->settings->abs_dua_tol       = kMpcAbsTol;

    TinySolver* old_solver = nullptr;
    {
        std::lock_guard<std::mutex> lock(mpc_solver_mutex_);
        old_solver  = static_cast<TinySolver*>(mpc_solver_);
        mpc_solver_ = next_solver;
    }
    delete_tiny_solver(old_solver);
    return true;
}

bool MPCController::solve_mpc_control(const MpcStateVector& state, const MpcStateVector& reference, Eigen::Vector2d& control) {
    std::lock_guard<std::mutex> lock(mpc_solver_mutex_);
    auto* solver = static_cast<TinySolver*>(mpc_solver_);
    if (solver == nullptr) {
        return false;
    }

    TinyWorkspace* work = solver->work;
    work->x.col(0)     = state;
    for (int i = 0; i < kMpcHorizon; ++i) {
        work->Xref.col(i) = reference;
        if (i < kMpcHorizon - 1) {
            work->Uref.col(i).setZero();
        }
    }

    (void)tiny_solve(solver);
    control = work->u.col(0);
    return std::isfinite(control[0]) && std::isfinite(control[1]);
}

void MPCController::release_mpc_solver() {
    TinySolver* solver = nullptr;
    {
        std::lock_guard<std::mutex> lock(mpc_solver_mutex_);
        solver      = static_cast<TinySolver*>(mpc_solver_);
        mpc_solver_ = nullptr;
    }
    delete_tiny_solver(solver);
}

controller_interface::CallbackReturn MPCController::on_configure(const rclcpp_lifecycle::State& previous_state) {
    (void)previous_state;

    imu_topic_            = get_node()->get_parameter("imu_topic").as_string();
    imu_pose_topic_       = get_node()->get_parameter("imu_pose_topic").as_string();
    use_mujoco_sim_chain_ = use_sim_time_parameter();
    torque_limit_         = get_node()->get_parameter("torque_limit").as_double();
    leg_angle_diff_kp_    = get_node()->get_parameter("leg_angle_diff_kp").as_double();
    leg_angle_diff_kd_    = get_node()->get_parameter("leg_angle_diff_kd").as_double();
    wheel_diff_kp_        = get_node()->get_parameter("wheel_diff_kp").as_double();
    wheel_diff_ki_        = get_node()->get_parameter("wheel_diff_ki").as_double();

    std::string error;
    if (!configure_mpc_solver(q_diag_, r_diag_, torque_limit_, error)) {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to configure TinyMPC solver: %s", error.c_str());
        return controller_interface::CallbackReturn::ERROR;
    }

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

controller_interface::CallbackReturn MPCController::on_activate(const rclcpp_lifecycle::State& previous_state) {
    (void)previous_state;

    state_velocity_filtered_.fill(0.0);
    state_velocity_filter_initialized_.fill(false);
    wheel_spin_error_integral_ = 0.0;

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MPCController::on_deactivate(const rclcpp_lifecycle::State& previous_state) {
    (void)previous_state;

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type MPCController::update(const rclcpp::Time& time, const rclcpp::Duration& period) {
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

void MPCController::update_motor_commands(const rclcpp::Time& time, const rclcpp::Duration& period) {
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
        MpcStateVector X, exp_X;
        exp_X.setZero();

        if(exp_x-x>5.0)         //防止x数值爆炸
            exp_X[0]=x+5.0;
        else if(exp_x-x<-5.0)
            exp_X[0]=x-5.0;
        else
            exp_X[0]=exp_x;

        X << x, dx, theta, dtheta, phi, dphi; //填写状态向量

        if (!solve_mpc_control(X, exp_X, u)) {
            u.setZero();
            RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000, "TinyMPC failed to produce finite control");
        }
    }
    else if(state==3)
    {
        u.setZero();
        u[1] = -(kAirThetaKp * theta + kAirThetaKd * dtheta);
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
    
    RCLCPP_INFO_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 100,
        "state=%d\nF=(%.4f,%.4f)\nu:(T=%.4f,Tp=%.4f)\nlength=(%.4f,%.4f)",
        state, left_leg_force[0], right_leg_force[0],u[0], u[1],left_leg_pos[0],right_leg_pos[0]);
}

void MPCController::imu_pose_callback(const geometry_msgs::msg::PoseStamped& msg) {
    imu_state_.orientation.x = msg.pose.orientation.x;
    imu_state_.orientation.y = msg.pose.orientation.y;
    imu_state_.orientation.z = msg.pose.orientation.z;
    imu_state_.orientation.w = msg.pose.orientation.w;
}

void MPCController::imu_callback(const sensor_msgs::msg::Imu& msg) {
    imu_state_.angular_velocity    = msg.angular_velocity;
    imu_state_.linear_acceleration = msg.linear_acceleration;
}

double MPCController::clamp_torque(const double value) const {
    if (!std::isfinite(value)) {
        return 0.0;
    }
    return std::clamp(value, -torque_limit_, torque_limit_);
}

bool MPCController::use_mujoco_sim_chain() const { return use_mujoco_sim_chain_; }

bool MPCController::use_sim_time_parameter() const {
    rclcpp::Parameter use_sim_time;
    if (get_node()->get_parameter("use_sim_time", use_sim_time)) {
        return use_sim_time.as_bool();
    }
    return false;
}

controller_interface::InterfaceConfiguration MPCController::command_interface_configuration() const {
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

controller_interface::InterfaceConfiguration MPCController::state_interface_configuration() const {
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


} // namespace mpc_controller

PLUGINLIB_EXPORT_CLASS(mpc_controller::MPCController, controller_interface::ControllerInterface)
