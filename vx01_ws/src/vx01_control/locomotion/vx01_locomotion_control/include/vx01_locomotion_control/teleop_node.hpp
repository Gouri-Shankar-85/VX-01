#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <termios.h>

namespace vx01_locomotion_control {

class TeleopNode : public rclcpp::Node {
public:
    explicit TeleopNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~TeleopNode();

private:
    void readLoop();
    void publishVelocity();
    void printHelp() const;

    char getKey();
    void restoreTerminal();

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr pub_timer_;

    double linear_step_;
    double angular_step_;
    double max_linear_vel_;
    double max_angular_vel_;

    double vx_{0.0}, vy_{0.0}, omega_{0.0};

    struct termios orig_termios_;
    bool terminal_saved_{false};
};

}  // namespace vx01_locomotion_control
