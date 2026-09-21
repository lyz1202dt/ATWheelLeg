#pragma once

#include "tools/leg_calc.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <array>
#include <mutex>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <robot_interfaces/msg/robot_state.hpp>
#include <robot_interfaces/msg/robot_target.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <Eigen/Dense>

namespace mpc_controller {

class MPCController : public controller_interface::ControllerInterface {
public:
    MPCController();
    ~MPCController() override;

    controller_interface::CallbackReturn on_init() override;
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

    controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

private:
    void update_motor_commands(const rclcpp::Time& time,const rclcpp::Duration& period);
    void imu_pose_callback(const geometry_msgs::msg::PoseStamped& msg);
    void imu_callback(const sensor_msgs::msg::Imu& msg);
    double clamp_torque(double value) const;
    bool use_mujoco_sim_chain() const;
    bool use_sim_time_parameter() const;
    bool configure_mpc_solver(
        const std::array<double, 6>& q_diag,
        const std::array<double, 2>& r_diag,
        double input_torque_limit,
        std::string& error);
    bool solve_mpc_control(
        const Eigen::Matrix<double, 6, 1>& state,
        const Eigen::Matrix<double, 6, 1>& reference,
        Eigen::Vector2d& control);
    void release_mpc_solver();

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr imu_pose_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    rclcpp_lifecycle::LifecycleNode::OnSetParametersCallbackHandle::SharedPtr param_cb_;

    sensor_msgs::msg::Imu imu_state_;
    robot_interfaces::msg::RobotState robot_state_;
    robot_interfaces::msg::RobotTarget robot_target_;

    double torque_limit_{20.0};
    std::string imu_topic_{"/imu_imu_sensor/imu"};
    std::string imu_pose_topic_{"/imu_pose_sensor/pose"};
    bool use_mujoco_sim_chain_{false};


    //MPC自动控制相关
    LegCalc leg;
    int state{1};
    rclcpp::Time last_state_switch_time;
    void* mpc_solver_{nullptr};
    mutable std::mutex mpc_solver_mutex_;
    Eigen::Vector2d u;              //控制向量
    double exp_x{0.0};              //期望位置
    double exp_omega{0.0};          //期望自旋角速度
    std::array<double, 6> q_diag_{{10.0, 400.0, 100.0, 40.0, 600.0, 50.0}};
    std::array<double, 2> r_diag_{{8.0, 0.5}};

    double vmc_kp{600.0};
    double vmc_kd{40.0};
    double leg_angle_diff_kp_{30.0};
    double leg_angle_diff_kd_{4.0};
    double wheel_diff_kp_{0.1};
    double wheel_diff_ki_{0.002};
    double wheel_spin_error_integral_{0.0};
    std::array<double, 3> state_velocity_filtered_{};
    std::array<bool, 3> state_velocity_filter_initialized_{};
};

}  // namespace mpc_controller
