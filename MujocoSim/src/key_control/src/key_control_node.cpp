// Keyboard teleop node for velocity commands.
//
// Modelled after turtlesim's turtle_teleop_key, but with incremental (per-press)
// velocity control instead of continuous key-held control:
//   [Up]     linear.x  += linear_step
//   [Down]   linear.x  -= linear_step
//   [Left]   angular.z += angular_step   (counter-clockwise / left turn)
//   [Right]  angular.z -= angular_step   (clockwise / right turn)
//   [Space]  reset both velocities to 0
//
// The current velocity is published as geometry_msgs/msg/Twist on a wall timer
// at a fixed rate (default 50 Hz).

#include <atomic>
#include <chrono>
#include <cstdio>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

namespace key_control
{

enum class Key
{
    Up,
    Down,
    Left,
    Right,
    Space,
    Unknown,
};

class KeyControlNode : public rclcpp::Node
{
public:
    KeyControlNode()
    : Node("key_control_node")
    {
        cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/turtle1/cmd_vel");
        linear_step_ = declare_parameter<double>("linear_step", 0.1);
        angular_step_ = declare_parameter<double>("angular_step", 0.1);
        publish_rate_ = declare_parameter<double>("publish_rate", 50.0);

        publisher_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);

        const auto period = std::chrono::milliseconds(
            static_cast<int64_t>(1000.0 / publish_rate_));
        timer_ = create_wall_timer(period, [this]() { publishTimerCallback(); });

        setRawMode();
        keyboard_thread_ = std::thread(&KeyControlNode::keyboardLoop, this);

        printUsage();
    }

    ~KeyControlNode() override
    {
        stop_ = true;
        if (keyboard_thread_.joinable()) {
            keyboard_thread_.join();
        }
        restoreTerminal();
    }

private:
    void printUsage() const
    {
        RCLCPP_INFO(get_logger(), "===============================");
        RCLCPP_INFO(get_logger(), "Keyboard velocity control:");
        RCLCPP_INFO(get_logger(), "  [Up]     linear.x  += %.2f m/s", linear_step_);
        RCLCPP_INFO(get_logger(), "  [Down]   linear.x  -= %.2f m/s", linear_step_);
        RCLCPP_INFO(get_logger(), "  [Left]   angular.z += %.2f rad/s (counter-clockwise)", angular_step_);
        RCLCPP_INFO(get_logger(), "  [Right]  angular.z -= %.2f rad/s (clockwise)", angular_step_);
        RCLCPP_INFO(get_logger(), "  [Space]  reset all velocities to 0");
        RCLCPP_INFO(get_logger(), "Publishing on '%s' at %.1f Hz", cmd_vel_topic_.c_str(), publish_rate_);
        RCLCPP_INFO(get_logger(), "Ctrl-C to quit");
        RCLCPP_INFO(get_logger(), "===============================");
    }

    void setRawMode()
    {
        tcgetattr(STDIN_FILENO, &orig_termios_);
        termios raw = orig_termios_;
        raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
        raw.c_cc[VMIN] = 1;
        raw.c_cc[VTIME] = 0;
        tcsetattr(STDIN_FILENO, TCSANOW, &raw);
    }

    void restoreTerminal() const
    {
        tcsetattr(STDIN_FILENO, TCSANOW, &orig_termios_);
    }

    static bool waitForInput(int fd, int timeout_ms)
    {
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(fd, &readfds);
        timeval timeout{timeout_ms / 1000, (timeout_ms % 1000) * 1000};
        return select(fd + 1, &readfds, nullptr, nullptr, &timeout) > 0;
    }

    Key readKey()
    {
        unsigned char c = 0;
        if (!waitForInput(STDIN_FILENO, 200)) {
            return Key::Unknown;
        }
        if (::read(STDIN_FILENO, &c, 1) != 1) {
            return Key::Unknown;
        }

        if (c == ' ') {
            return Key::Space;
        }

        // Arrow keys are sent as the escape sequence ESC '[' 'A'/'B'/'C'/'D'.
        if (c == 0x1b) {
            if (!waitForInput(STDIN_FILENO, 50)) {
                return Key::Unknown;  // Lone ESC key.
            }
            unsigned char c1 = 0;
            if (::read(STDIN_FILENO, &c1, 1) != 1 || c1 != '[') {
                return Key::Unknown;
            }
            if (!waitForInput(STDIN_FILENO, 50)) {
                return Key::Unknown;
            }
            unsigned char c2 = 0;
            if (::read(STDIN_FILENO, &c2, 1) != 1) {
                return Key::Unknown;
            }
            switch (c2) {
                case 'A': return Key::Up;
                case 'B': return Key::Down;
                case 'C': return Key::Right;
                case 'D': return Key::Left;
                default: return Key::Unknown;
            }
        }

        return Key::Unknown;
    }

    void keyboardLoop()
    {
        while (rclcpp::ok() && !stop_) {
            const Key key = readKey();
            if (key == Key::Unknown) {
                continue;
            }
            processKey(key);
        }
    }

    void processKey(const Key key)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        switch (key) {
            case Key::Up:
                linear_x_ += linear_step_;
                break;
            case Key::Down:
                linear_x_ -= linear_step_;
                break;
            case Key::Left:
                angular_z_ += angular_step_;
                break;
            case Key::Right:
                angular_z_ -= angular_step_;
                break;
            case Key::Space:
                linear_x_ = 0.0;
                angular_z_ = 0.0;
                break;
            default:
                break;
        }
        RCLCPP_INFO(
            get_logger(),
            "vel: linear.x = %.2f m/s, angular.z = %.2f rad/s",
            linear_x_, angular_z_);
    }

    void publishTimerCallback()
    {
        geometry_msgs::msg::Twist msg;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            msg.linear.x = linear_x_;
            msg.angular.z = angular_z_;
        }
        publisher_->publish(msg);
    }

    std::string cmd_vel_topic_;
    double linear_step_{0.1};
    double angular_step_{0.1};
    double publish_rate_{50.0};

    double linear_x_{0.0};
    double angular_z_{0.0};
    std::mutex mutex_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::thread keyboard_thread_;
    std::atomic<bool> stop_{false};
    termios orig_termios_{};
};

}  // namespace key_control

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<key_control::KeyControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
