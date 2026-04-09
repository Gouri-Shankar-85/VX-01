#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/command_bool.hpp>

using namespace std::chrono_literals;

class AerialControllerNode : public rclcpp::Node {
public:
    AerialControllerNode() : Node("aerial_controller_node")
    {
        sub_cmd_ = create_subscription<geometry_msgs::msg::TwistStamped>(
            "/drone/cmd_vel", 10,
            [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
                latest_cmd_          = *msg;
                last_cmd_time_       = now();
                has_received_cmd_    = true;
            });

        sub_state_ = create_subscription<mavros_msgs::msg::State>(
            "/mavros/state", 10,
            [this](const mavros_msgs::msg::State::SharedPtr msg) {
                is_connected_ = msg->connected;
                is_armed_     = msg->armed;
                current_mode_ = msg->mode;
            });

        pub_vel_ = create_publisher<geometry_msgs::msg::TwistStamped>(
            "/mavros/setpoint_velocity/cmd_vel", 10);

        sub_arm_ = create_subscription<std_msgs::msg::Bool>(
            "/drone/arm", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                sendArmCommand(msg->data);
            });

        arm_client_  = create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
        mode_client_ = create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");

        heartbeat_timer_ = create_wall_timer(100ms, [this]() { publishSetpoint(); });

        RCLCPP_INFO(get_logger(),
            "AerialController ready. "
            "Publish geometry_msgs/TwistStamped to /drone/cmd_vel for flight. "
            "Publish std_msgs/Bool to /drone/arm to arm (true) or disarm (false).");
    }

private:

    void publishSetpoint()
    {
        if (!is_connected_) return;

        geometry_msgs::msg::TwistStamped cmd;
        cmd.header.stamp    = now();
        cmd.header.frame_id = "base_link";

        if (has_received_cmd_) {
            double age = (now() - last_cmd_time_).seconds();
            if (age < 0.5) {
                cmd.twist = latest_cmd_.twist;
            }
        }

        pub_vel_->publish(cmd);
    }

    void sendArmCommand(bool arm)
    {
        if (!arm_client_->wait_for_service(1s)) {
            RCLCPP_WARN(get_logger(), "Arming service not available");
            return;
        }
        auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        req->value = arm;
        arm_client_->async_send_request(req,
            [this, arm](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future) {
                auto resp = future.get();
                if (resp->success) {
                    RCLCPP_INFO(get_logger(), "%s successful", arm ? "ARM" : "DISARM");
                } else {
                    RCLCPP_WARN(get_logger(), "%s FAILED (result=%d)",
                        arm ? "ARM" : "DISARM", resp->result);
                }
            });
    }

    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr   sub_cmd_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr     sub_state_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr         sub_arm_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr      pub_vel_;
    rclcpp::TimerBase::SharedPtr                                 heartbeat_timer_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr     arm_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr         mode_client_;

    geometry_msgs::msg::TwistStamped latest_cmd_{};
    rclcpp::Time              last_cmd_time_{0, 0, RCL_ROS_TIME};
    bool                      has_received_cmd_ = false;
    bool                      is_connected_     = false;
    bool                      is_armed_         = false;
    std::string               current_mode_     = "STABILIZE";
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AerialControllerNode>());
    rclcpp::shutdown();
    return 0;
}
