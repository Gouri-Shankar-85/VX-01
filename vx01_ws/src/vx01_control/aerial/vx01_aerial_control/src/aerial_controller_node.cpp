#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <vx01_msgs/msg/robot_mode.hpp>

class AerialControllerNode : public rclcpp::Node {
public:
    AerialControllerNode() : Node("aerial_controller_node"), current_mode_("") {
        sub_mode_ = this->create_subscription<vx01_msgs::msg::RobotMode>(
            "/robot_mode", 10,
            std::bind(&AerialControllerNode::modeCallback, this, std::placeholders::_1));

        sub_cmd_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&AerialControllerNode::cmdCallback, this, std::placeholders::_1));

        pub_mavros_cmd_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/mavros/setpoint_velocity/cmd_vel_unstamped", 10);
        
        RCLCPP_INFO(this->get_logger(), "VX-01 Aerial Controller Initialized.");
    }

private:
    void modeCallback(const vx01_msgs::msg::RobotMode::SharedPtr msg) {
        current_mode_ = msg->mode;
    }

    void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        if (current_mode_ == "DRONE") {
            // Forward manual control to MAVROS when flying
            pub_mavros_cmd_->publish(*msg);
        }
    }

    rclcpp::Subscription<vx01_msgs::msg::RobotMode>::SharedPtr sub_mode_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_mavros_cmd_;
    std::string current_mode_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AerialControllerNode>());
    rclcpp::shutdown();
    return 0;
}
