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
        orientation_ = Eigen::Quaternionf::Identity();
        angular_velocity_.setZero();
        acceleration_.setZero();
        imu_received_ = false;
        orientation_received_ = false;
    }
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

bool VirIMU::update(Eigen::Quaternionf& q,
                    Eigen::Vector3d& angular,
                    Eigen::Vector3d& acc,
                    const float& dt)
{
    (void)dt;
    return is_ready() && read_state(q, angular, acc);
}

void VirIMU::imu_callback(const sensor_msgs::msg::Imu& msg)
{
    const Eigen::Quaternionf message_orientation(
        static_cast<float>(msg.orientation.w),
        static_cast<float>(msg.orientation.x),
        static_cast<float>(msg.orientation.y),
        static_cast<float>(msg.orientation.z));

    std::lock_guard<std::mutex> lock(state_mutex_);
    angular_velocity_ = Eigen::Vector3d(
        msg.angular_velocity.x,
        msg.angular_velocity.y,
        msg.angular_velocity.z);
    acceleration_ = Eigen::Vector3d(
        msg.linear_acceleration.x,
        msg.linear_acceleration.y,
        msg.linear_acceleration.z);
    imu_received_ = angular_velocity_.allFinite() && acceleration_.allFinite();

    if (message_orientation.coeffs().allFinite() &&
        message_orientation.squaredNorm() > 1.0e-8F) {
        orientation_ = message_orientation.normalized();
        orientation_received_ = true;
    }

    publish_current_state_locked();
}

void VirIMU::pose_callback(const geometry_msgs::msg::PoseStamped& msg)
{
    Eigen::Quaternionf pose_orientation;
    if (!quaternion_from_msg(msg.pose.orientation, pose_orientation)) {
        return;
    }

    std::lock_guard<std::mutex> lock(state_mutex_);
    orientation_ = pose_orientation;
    orientation_received_ = true;
    publish_current_state_locked();
}

void VirIMU::publish_current_state_locked()
{
    publish_state(orientation_, angular_velocity_, acceleration_);
    ready_.store(imu_received_ && orientation_received_,
                 std::memory_order_release);
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
