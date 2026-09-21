#pragma once

#include <controller_interface/controller_interface.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <Eigen/Dense>

#include <array>
#include <atomic>
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
    bool configure_lqr_gain();
    bool read_motor_states();
    void cmd_vel_callback(const geometry_msgs::msg::Twist& msg);
    static std::vector<double> to_vector(
        const std::array<float, 6>& values);
    static std::vector<double> to_vector(
        const std::array<float, 2>& values);

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
    float desired_height_{0.21F};
    int requested_mode_{0};
    double effort_limit_{20.0};
    std::string imu_topic_{"/imu_imu_sensor/imu"};
    std::string imu_pose_topic_{"/imu_pose_sensor/pose"};
    std::string cmd_vel_topic_{"/cmd_vel"};
    std::array<float, 6> q_diag_ = {
        10.0F, 400.0F, 100.0F, 40.0F, 600.0F, 50.0F};
    std::array<float, 2> r_diag_ = {8.0F, 0.5F};
};

}  // namespace lqr_controller
