#pragma once

#include <rclcpp/subscription.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <Eigen/Dense>

#include <atomic>
#include <mutex>
#include <string>

#include "imubase.hpp"

namespace lqr_controller {

class VirIMU final : public IMUBase {
public:
    VirIMU() = default;

    bool configure(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                   const std::string& imu_topic,
                   const std::string& pose_topic);

    bool is_ready() override;
    bool update(Eigen::Quaternionf& q,
                Eigen::Vector3d& angular,
                Eigen::Vector3d& acc,
                const float& dt) override;

private:
    void imu_callback(const sensor_msgs::msg::Imu& msg);
    void pose_callback(const geometry_msgs::msg::PoseStamped& msg);
    void publish_current_state_locked();
    static bool quaternion_from_msg(const geometry_msgs::msg::Quaternion& msg,
                                   Eigen::Quaternionf& quaternion);

    std::mutex state_mutex_;
    Eigen::Quaternionf orientation_ = Eigen::Quaternionf::Identity();
    Eigen::Vector3d angular_velocity_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d acceleration_ = Eigen::Vector3d::Zero();
    bool imu_received_ = false;
    bool orientation_received_ = false;
    std::atomic<bool> ready_{false};

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_;
};

}  // namespace lqr_controller
