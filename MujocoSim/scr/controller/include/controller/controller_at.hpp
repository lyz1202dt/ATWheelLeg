#pragma once

#include <controller_interface/controller_interface.hpp>
#include <Eigen/Dense>
#include <geometry_msgs/msg/twist.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <array>
#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "controller/vir_imu.hpp"
#include "controller/vir_motor.hpp"
#include "controllerbase.hpp"
#include "tools/lqr_calc.hpp"

class ControllerAT;

namespace lqr_controller {

class LQRControllerAT final : public controller_interface::ControllerInterface {
public:
    LQRControllerAT();

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
    static constexpr size_t kMotorCount = 6U;
    static constexpr size_t kLegGridSize = 7U;
    static constexpr size_t kGainTableSize = kLegGridSize * kLegGridSize;
    using GainMatrix = Eigen::Matrix<double, 4, 10>;

    bool update_lqr_k(const std::array<float, 10>& q_diag,
                      const std::array<float, 4>& r_diag,
                      std::string& error);
    bool bind_motor_interfaces();
    bool configure_lqr_gain(std::string& error);
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

    // The ROS wrapper talks to ControllerAT through ControllerBase so the
    // concrete core controller can be replaced without changing the wrapper.
    std::unique_ptr<ControllerBase> controller_;
    ControllerAT* controller_at_{nullptr};

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
        cmd_vel_subscriber_;

    std::array<const char*, kMotorCount> motor_joint_names_ = {
        "base_link_left_front1_link_joint",
        "base_link_right_front1_link_joint",
        "base_link_left_rear1_link_joint",
        "base_link_right_rear1_link_joint",
        "left_rear2_link_left_wheel_link_joint",
        "right_rear2_link_right_wheel_link_joint",
    };

    std::atomic<float> expected_velocity_{0.0F};
    std::atomic<float> expected_omega_{0.0F};
    std::atomic<float> expected_height_{0.25F};
    int requested_mode_{0};
    double effort_limit_{20.0};
    std::string imu_topic_{"/imu_imu_sensor/imu"};
    std::string imu_pose_topic_{"/imu_pose_sensor/pose"};
    std::string cmd_vel_topic_{"/cmd_vel"};
    std::array<float, 10> q_diag_ = {1.0F, 1.0F, 1.0F, 1.0F, 1.0F,
                                     1.0F, 1.0F, 1.0F, 1.0F, 1.0F};
    std::array<float, 4> r_diag_ = {1.0F, 1.0F, 1.0F, 1.0F};
    std::array<GainMatrix, kGainTableSize> K_table_{};
    bool gain_table_configured_{false};
    mutable std::mutex gain_mutex_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
        parameter_callback_handle_;
};

}  // namespace lqr_controller
