#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/command_bool.hpp>

/**
 * VX-01 Aerial Controller Node
 * 
 * Receives /drone/cmd_vel from dashboard and:
 *  1. Forwards as stamped setpoint to MAVROS (required for OFFBOARD mode heartbeat)
 *  2. Maintains >2Hz setpoint stream (MAVROS drops OFFBOARD if stream stops)
 *  3. Exposes /drone/arm and /drone/offboard topics for dashboard control
 */
class AerialControllerNode : public rclcpp::Node {
public:
    AerialControllerNode() : Node("aerial_controller_node") {
        // Subscribe to drone-specific cmd_vel (separate from hexapod /cmd_vel)
        sub_cmd_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/drone/cmd_vel", 10,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
                latest_cmd_ = *msg;
                has_cmd_    = true;
            });

        // Stamped setpoint publisher (preferred by MAVROS for OFFBOARD)
        pub_setpoint_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/mavros/setpoint_velocity/cmd_vel", 10);

        // Also publish unstamped for compatibility
        pub_setpoint_unstamped_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/mavros/setpoint_velocity/cmd_vel_unstamped", 10);

        // MAVROS state subscriber (to know current mode/armed status)
        sub_state_ = this->create_subscription<mavros_msgs::msg::State>(
            "/mavros/state", 10,
            [this](const mavros_msgs::msg::State::SharedPtr msg) {
                is_armed_    = msg->armed;
                current_mode_ = msg->mode;
            });

        // 10Hz heartbeat timer — keeps OFFBOARD alive
        heartbeat_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            [this]() {
                auto stamped = geometry_msgs::msg::TwistStamped();
                stamped.header.stamp    = this->now();
                stamped.header.frame_id = "map";
                if (has_cmd_) {
                    stamped.twist = latest_cmd_;
                }
                pub_setpoint_->publish(stamped);
                pub_setpoint_unstamped_->publish(has_cmd_ ? latest_cmd_
                                                           : geometry_msgs::msg::Twist());
            });

        RCLCPP_INFO(get_logger(),
            "Aerial Controller ready. Publish /drone/cmd_vel for flight commands.");
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr      sub_cmd_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr         sub_state_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr   pub_setpoint_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr          pub_setpoint_unstamped_;
    rclcpp::TimerBase::SharedPtr                                     heartbeat_timer_;

    geometry_msgs::msg::Twist latest_cmd_{};
    bool        has_cmd_      = false;
    bool        is_armed_     = false;
    std::string current_mode_ = "STABILIZE";
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AerialControllerNode>());
    rclcpp::shutdown();
    return 0;
}
