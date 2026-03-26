#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <termios.h>
#include <unistd.h>
#include <csignal>
#include <cstdio>
#include <map>

namespace hexapod_gait
{

static const char HELP[] = R"(
Hexapod Teleop
--------------
  w / s  : forward / backward
  a / d  : strafe left / right
  q / e  : rotate CCW / CW
  space  : stop
  x      : quit
)";

class HexapodTeleopNode : public rclcpp::Node
{
public:
    HexapodTeleopNode() : Node("hexapod_teleop_node")
    {
        declare_parameter("linear_scale",  1.0);
        declare_parameter("angular_scale", 1.0);
        linear_scale_  = get_parameter("linear_scale").as_double();
        angular_scale_ = get_parameter("angular_scale").as_double();

        pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        RCLCPP_INFO(get_logger(), "%s", HELP);
    }

    void run()
    {
        setRawMode(true);
        char c;
        while (rclcpp::ok()) {
            if (read(STDIN_FILENO, &c, 1) < 0) break;

            geometry_msgs::msg::Twist msg;
            bool publish = true;

            switch (c) {
                case 'w': msg.linear.x  =  linear_scale_;  break;
                case 's': msg.linear.x  = -linear_scale_;  break;
                case 'a': msg.linear.y  =  linear_scale_;  break;
                case 'd': msg.linear.y  = -linear_scale_;  break;
                case 'q': msg.angular.z =  angular_scale_; break;
                case 'e': msg.angular.z = -angular_scale_; break;
                case ' ': break;
                case 'x': publish = false; rclcpp::shutdown(); break;
                default:  publish = false; break;
            }

            if (publish) pub_->publish(msg);
        }
        setRawMode(false);
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    double linear_scale_;
    double angular_scale_;
    struct termios original_termios_;

    void setRawMode(bool enable)
    {
        if (enable) {
            tcgetattr(STDIN_FILENO, &original_termios_);
            struct termios raw = original_termios_;
            raw.c_lflag &= ~(ICANON | ECHO);
            raw.c_cc[VMIN]  = 1;
            raw.c_cc[VTIME] = 0;
            tcsetattr(STDIN_FILENO, TCSANOW, &raw);
        } else {
            tcsetattr(STDIN_FILENO, TCSANOW, &original_termios_);
        }
    }
};

} // namespace hexapod_gait

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<hexapod_gait::HexapodTeleopNode>();
    node->run();
    rclcpp::shutdown();
    return 0;
}
