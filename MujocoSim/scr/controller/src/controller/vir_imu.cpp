#include <controller/vir_imu.hpp>

#include <cmath>

namespace lqr_controller {

bool VirIMU::configure(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                       const std::string& imu_topic,
                       const std::string& pose_topic)
{
    if (node == nullptr || imu_topic.empty()) {
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        imu_received_ = false;
        orientation_received_ = false;
    }
    lock_memory();
    state_valid_ = false;
    unlock_memory();
    ready_.store(false, std::memory_order_release);

    imu_subscriber_ = node->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic,
        rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::Imu& msg) { imu_callback(msg); });

    pose_subscriber_.reset();
    if (!pose_topic.empty()) {
        pose_subscriber_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
            pose_topic,
            rclcpp::SensorDataQoS(),
            [this](const geometry_msgs::msg::PoseStamped& msg) { pose_callback(msg); });
    }

    return imu_subscriber_ != nullptr;
}

bool VirIMU::is_ready()
{
    return ready_.load(std::memory_order_acquire);
}

bool VirIMU::update(const float& dt)
{
    (void)dt;
    return is_ready();
}

void VirIMU::imu_callback(const sensor_msgs::msg::Imu& msg)
{
    const Eigen::Quaternionf message_orientation(
        static_cast<float>(msg.orientation.w),
        static_cast<float>(msg.orientation.x),
        static_cast<float>(msg.orientation.y),
        static_cast<float>(msg.orientation.z));

    const Eigen::Vector3d next_angular_velocity(
        msg.angular_velocity.x,
        msg.angular_velocity.y,
        msg.angular_velocity.z);
    const Eigen::Vector3d next_acceleration(
        msg.linear_acceleration.x,
        msg.linear_acceleration.y,
        msg.linear_acceleration.z);
    const bool imu_valid =
        next_angular_velocity.allFinite() && next_acceleration.allFinite();
    const bool orientation_valid =
        message_orientation.coeffs().allFinite() &&
        message_orientation.squaredNorm() > 1.0e-8F;

    std::lock_guard<std::mutex> lock(state_mutex_);
    imu_received_ = imu_valid;
    if (orientation_valid) {
        orientation_received_ = true;
    }

    const bool next_state_valid = imu_received_ && orientation_received_;
    lock_memory();
    angular_velocity = next_angular_velocity;
    acceleration = next_acceleration;
    if (orientation_valid) {
        orientation = message_orientation.normalized();
    }
    state_valid_ = next_state_valid;
    unlock_memory();
    ready_.store(next_state_valid, std::memory_order_release);
}

void VirIMU::pose_callback(const geometry_msgs::msg::PoseStamped& msg)
{
    Eigen::Quaternionf pose_orientation;
    if (!quaternion_from_msg(msg.pose.orientation, pose_orientation)) {
        return;
    }

    std::lock_guard<std::mutex> lock(state_mutex_);
    orientation_received_ = true;
    const bool next_state_valid = imu_received_ && orientation_received_;
    lock_memory();
    orientation = pose_orientation;
    state_valid_ = next_state_valid;
    unlock_memory();
    ready_.store(next_state_valid, std::memory_order_release);
}

bool VirIMU::quaternion_from_msg(const geometry_msgs::msg::Quaternion& msg,
                                 Eigen::Quaternionf& quaternion)
{
    quaternion = Eigen::Quaternionf(
        static_cast<float>(msg.w),
        static_cast<float>(msg.x),
        static_cast<float>(msg.y),
        static_cast<float>(msg.z));
    if (!quaternion.coeffs().allFinite() ||
        quaternion.squaredNorm() < 1.0e-8F) {
        return false;
    }

    quaternion.normalize();
    return quaternion.coeffs().allFinite();
}

}  // namespace lqr_controller
