#pragma once

#include <rclcpp/subscription.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <string>

#include "imubase.hpp"

namespace lqr_controller {

class VirIMU final : public IMUBase {
public:
    VirIMU() = default;

    bool configure(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                   const std::string& imu_topic,
                   const std::string& pose_topic);

private:
    void imu_callback(const sensor_msgs::msg::Imu& msg);
    void pose_callback(const geometry_msgs::msg::PoseStamped& msg);

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_;
};

}  // namespace lqr_controller
