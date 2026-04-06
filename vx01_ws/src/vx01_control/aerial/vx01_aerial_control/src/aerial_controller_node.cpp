/**
 * VX-01 Aerial Controller Node
 *
 * Responsibilities:
 *   1. Bridge /drone/cmd_vel → /mavros/setpoint_velocity/cmd_vel_unstamped
 *      (ArduPilot GUIDED mode uses the unstamped Twist topic, NOT TwistStamped)
 *   2. Publish a 10 Hz heartbeat so ArduPilot stays in GUIDED velocity mode
 *      (ArduPilot exits GUIDED if no setpoint arrives for >3s)
 *   3. Auto-hover: if /drone/cmd_vel goes silent for >0.5s, zero the setpoint
 *      so the drone holds position rather than drifting on the last command
 *   4. Expose /drone/arm (Bool) and /drone/disarm for clean arming/disarming
 *      without going through the dashboard's ROS service calls every time
 *
 * ArduPilot GUIDED velocity control:
 *   Topic: /mavros/setpoint_velocity/cmd_vel_unstamped  (geometry_msgs/Twist)
 *   Frame: body frame (base_link). lx=forward, ly=left, lz=up (ENU)
 *   Rate:  must publish ≥2 Hz or ArduPilot exits velocity sub-mode
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
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
        // ── Subscriptions ─────────────────────────────────────────────────────
        sub_cmd_ = create_subscription<geometry_msgs::msg::Twist>(
            "/drone/cmd_vel", 10,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
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

        // ── Publishers ────────────────────────────────────────────────────────
        // ArduPilot GUIDED velocity: use cmd_vel_unstamped (Twist, not TwistStamped)
        pub_vel_ = create_publisher<geometry_msgs::msg::Twist>(
            "/mavros/setpoint_velocity/cmd_vel_unstamped", 10);

        // ── Arm / Disarm topics ───────────────────────────────────────────────
        sub_arm_ = create_subscription<std_msgs::msg::Bool>(
            "/drone/arm", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                sendArmCommand(msg->data);
            });

        // ── Service clients (for arm/mode) ────────────────────────────────────
        arm_client_  = create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
        mode_client_ = create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");

        // ── 10 Hz heartbeat ───────────────────────────────────────────────────
        // Keeps ArduPilot in GUIDED velocity sub-mode. If /drone/cmd_vel goes
        // silent for >500ms we zero the command to auto-hover.
        heartbeat_timer_ = create_wall_timer(100ms, [this]() { publishSetpoint(); });

        RCLCPP_INFO(get_logger(),
            "AerialController ready. "
            "Publish geometry_msgs/Twist to /drone/cmd_vel for flight. "
            "Publish std_msgs/Bool to /drone/arm to arm (true) or disarm (false).");
    }

private:

    // ── Setpoint publisher ────────────────────────────────────────────────────
    void publishSetpoint()
    {
        if (!is_connected_) return;   // don't spam before MAVROS is up

        geometry_msgs::msg::Twist cmd;

        if (has_received_cmd_) {
            // Auto-hover: zero the command if no update in 500ms
            const double age = (now() - last_cmd_time_).seconds();
            if (age < 0.5) {
                cmd = latest_cmd_;
            } else {
                // timeout → hold position (hover) by zeroing all velocities
                cmd = geometry_msgs::msg::Twist{};
                if (age > 1.0) {
                    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 5000,
                        "No /drone/cmd_vel for %.1fs — hovering", age);
                }
            }
        }
        // else: pre-arm, publish zero to pre-fill the topic so MAVROS sees it

        pub_vel_->publish(cmd);
    }

    // ── Arm/Disarm helper ─────────────────────────────────────────────────────
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

    // ── Members ───────────────────────────────────────────────────────────────
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr   sub_cmd_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr     sub_state_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr         sub_arm_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr      pub_vel_;
    rclcpp::TimerBase::SharedPtr                                 heartbeat_timer_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr     arm_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr         mode_client_;

    geometry_msgs::msg::Twist latest_cmd_{};
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
