#include <controller/vir_imu.hpp>

namespace lqr_controller {

bool VirIMU::configure(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                       const std::string& imu_topic,
                       const std::string& pose_topic)
{
    imu_subscriber_ = node->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic,
        rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::Imu& msg) { imu_callback(msg); });
    pose_subscriber_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
        pose_topic,
        rclcpp::SensorDataQoS(),
        [this](const geometry_msgs::msg::PoseStamped& msg) { pose_callback(msg); });
    return true;
}

void VirIMU::imu_callback(const sensor_msgs::msg::Imu& msg)
{
    lock_memory();
    angular_velocity.x() = msg.angular_velocity.x;
    angular_velocity.y() = msg.angular_velocity.y;
    angular_velocity.z() = msg.angular_velocity.z;
    acceleration.x() = msg.linear_acceleration.x;
    acceleration.y() = msg.linear_acceleration.y;
    acceleration.z() = msg.linear_acceleration.z;
    unlock_memory();
}

void VirIMU::pose_callback(const geometry_msgs::msg::PoseStamped& msg)
{
    lock_memory();
    orientation.x() = msg.pose.orientation.x;
    orientation.y() = msg.pose.orientation.y;
    orientation.z() = msg.pose.orientation.z;
    orientation.w() = msg.pose.orientation.w;
    unlock_memory();
}

}  // namespace lqr_controller
