#include <controller/mujoco_sim.hpp>

#include <algorithm>
#include <cmath>

#include <pluginlib/class_list_macros.hpp>

namespace lqr_controller {

namespace {

constexpr size_t kMotorCount = 6;
constexpr size_t kTargetInterfacesPerMotor = 5;
constexpr const char* kReferencePrefix = "mujoco_sim_controller";

size_t target_index(const size_t motor_index, const size_t interface_index) {
    return motor_index * kTargetInterfacesPerMotor + interface_index;
}

}  // namespace

MujocoSimController::MujocoSimController() = default;

controller_interface::CallbackReturn MujocoSimController::on_init() {
    const std::array<std::string, kMotorCount> default_joint_names = {
        "left_front_hip_joint",
        "left_rear_hip_joint",
        "left_wheel_joint",
        "right_front_hip_joint",
        "right_rear_hip_joint",
        "right_wheel_joint",
    };

    auto_declare<double>("effort_limit", effort_limit_);
    auto_declare<std::vector<std::string>>(
        "joint_names",
        std::vector<std::string>(
            default_joint_names.begin(), default_joint_names.end()));

    if (!load_joint_names()) {
        RCLCPP_ERROR(
            get_node()->get_logger(),
            "joint_names must contain exactly %zu joint names",
            kMotorCount);
        return controller_interface::CallbackReturn::ERROR;
    }

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MujocoSimController::on_configure(const rclcpp_lifecycle::State& previous_state) {
    (void)previous_state;

    effort_limit_ = get_node()->get_parameter("effort_limit").as_double();
    if (!load_joint_names() || !std::isfinite(effort_limit_) ||
        effort_limit_ <= 0.0) {
        RCLCPP_ERROR(
            get_node()->get_logger(),
            "Invalid middle controller parameters");
        return controller_interface::CallbackReturn::ERROR;
    }

    reference_interfaces_.assign(kMotorCount * kTargetInterfacesPerMotor, 0.0);

    RCLCPP_INFO(
        get_node()->get_logger(), "MuJoCo sim middle controller exports motor targets under '%s'",
        kReferencePrefix);
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MujocoSimController::on_activate(const rclcpp_lifecycle::State& previous_state) {
    (void)previous_state;
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MujocoSimController::on_deactivate(const rclcpp_lifecycle::State& previous_state) {
    (void)previous_state;
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration MujocoSimController::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration cfg;
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto& name : motor_joint_names_) {
        cfg.names.push_back(name + "/effort");
    }
    return cfg;
}

controller_interface::InterfaceConfiguration MujocoSimController::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration cfg;
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto& name : motor_joint_names_) {
        cfg.names.push_back(name + "/position");
        cfg.names.push_back(name + "/velocity");
        cfg.names.push_back(name + "/effort");
    }
    return cfg;
}

std::vector<hardware_interface::CommandInterface> MujocoSimController::on_export_reference_interfaces() {
    std::vector<hardware_interface::CommandInterface> interfaces;
    interfaces.reserve(reference_interfaces_.size());

    for (size_t i = 0; i < kMotorCount; ++i) {
        const auto joint_prefix = motor_joint_names_[i] + "/";
        interfaces.emplace_back(kReferencePrefix, joint_prefix + "position", &reference_interfaces_[target_index(i, 0)]);
        interfaces.emplace_back(kReferencePrefix, joint_prefix + "velocity", &reference_interfaces_[target_index(i, 1)]);
        interfaces.emplace_back(kReferencePrefix, joint_prefix + "effort", &reference_interfaces_[target_index(i, 2)]);
        interfaces.emplace_back(kReferencePrefix, joint_prefix + "kp", &reference_interfaces_[target_index(i, 3)]);
        interfaces.emplace_back(kReferencePrefix, joint_prefix + "kd", &reference_interfaces_[target_index(i, 4)]);
    }
    return interfaces;
}

controller_interface::return_type MujocoSimController::update_reference_from_subscribers() {
    return controller_interface::return_type::OK;
}

controller_interface::return_type MujocoSimController::update_and_write_commands(
    const rclcpp::Time& time, const rclcpp::Duration& period) {
    (void)time;
    (void)period;

    read_joint_states();

    for (size_t i = 0; i < kMotorCount; ++i) {
        motor_reference_[i].position = reference_interfaces_[target_index(i, 0)];
        motor_reference_[i].velocity = reference_interfaces_[target_index(i, 1)];
        motor_reference_[i].effort = reference_interfaces_[target_index(i, 2)];
        motor_reference_[i].kp = reference_interfaces_[target_index(i, 3)];
        motor_reference_[i].kd = reference_interfaces_[target_index(i, 4)];

        const double position_error = motor_reference_[i].position - motor_state_[i].position;
        const double velocity_error = motor_reference_[i].velocity - motor_state_[i].velocity;
        const double effort =
            motor_reference_[i].kp * position_error + motor_reference_[i].kd * velocity_error + motor_reference_[i].effort;
        command_interfaces_[i].set_value(clamp_effort(effort));
    }

    return controller_interface::return_type::OK;
}

void MujocoSimController::read_joint_states() {
    for (size_t motor_index = 0; motor_index < kMotorCount; ++motor_index) {
        const size_t state_index = motor_index * kStateInterfacesPerMotor;
        motor_state_[motor_index].position = state_interfaces_[state_index].get_value();
        motor_state_[motor_index].velocity = state_interfaces_[state_index + 1].get_value();
        motor_state_[motor_index].effort = state_interfaces_[state_index + 2].get_value();
    }
}

double MujocoSimController::clamp_effort(const double effort) const {
    if (!std::isfinite(effort)) {
        return 0.0;
    }
    return std::clamp(effort, -effort_limit_, effort_limit_);
}

bool MujocoSimController::load_joint_names() {
    const auto configured_names =
        get_node()->get_parameter("joint_names").as_string_array();
    if (configured_names.size() != kMotorCount) {
        return false;
    }

    std::copy(
        configured_names.begin(),
        configured_names.end(),
        motor_joint_names_.begin());
    return true;
}

}  // namespace lqr_controller

PLUGINLIB_EXPORT_CLASS(lqr_controller::MujocoSimController, controller_interface::ChainableControllerInterface)
