#pragma once

#include <controller_interface/controller_interface.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <Eigen/Dense>

#include <array>
#include <atomic>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <string>
#include <vector>

#include <controller.hpp>

#include "controller/vir_imu.hpp"
#include "controller/vir_motor.hpp"

namespace lqr_controller {

class LQRController final : public controller_interface::ControllerInterface {
public:
    LQRController();

    controller_interface::CallbackReturn on_init() override;
    controller_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State& previous_state) override;
    controller_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State& previous_state) override;
    controller_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State& previous_state) override;

    controller_interface::return_type update(
        const rclcpp::Time& time,
        const rclcpp::Duration& period) override;

    controller_interface::InterfaceConfiguration command_interface_configuration()
        const override;
    controller_interface::InterfaceConfiguration state_interface_configuration()
        const override;

private:
    static constexpr size_t kMotorCount = 6;
    static constexpr size_t kStateInterfacesPerMotor = 3;

    bool bind_motor_interfaces();
    bool configure_controller(std::string& error);
    bool read_motor_states();
    void cmd_vel_callback(const geometry_msgs::msg::Twist& msg);
    rcl_interfaces::msg::SetParametersResult on_set_parameters(
        const std::vector<rclcpp::Parameter>& parameters);

    VirIMU imu_;
    VirMotor lf_motor_;
    VirMotor rf_motor_;
    VirMotor lb_motor_;
    VirMotor rb_motor_;
    VirMotor lw_motor_;
    VirMotor rw_motor_;
    ::Controller controller_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;

    std::array<const char*, kMotorCount> motor_joint_names_ = {
        "left_front_hip_joint",
        "right_front_hip_joint",
        "left_rear_hip_joint",
        "right_rear_hip_joint",
        "left_wheel_joint",
        "right_wheel_joint",
    };

    std::atomic<float> expected_velocity_{0.0F};
    std::atomic<float> expected_omega_{0.0F};
    int requested_mode_{0};
    double effort_limit_{20.0};
    std::string imu_topic_{"/imu_imu_sensor/imu"};
    std::string imu_pose_topic_{"/imu_pose_sensor/pose"};
    std::string cmd_vel_topic_{"/cmd_vel"};
    Controller::Params controller_params_{};
    bool use_k_tab_{true};
    std::vector<double> gain_lengths_;
    std::vector<double> gain_values_;
    LqrStateWeight q_diag_ = {1.0, 1.0, 10.0, 1.0, 1.0, 1.0};
    LqrInputWeight r_diag_ = {1.0, 1.0};
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
        parameter_callback_handle_;
};

}  // namespace lqr_controller
