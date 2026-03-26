#include "vx01_locomotion_control/teleop_node.hpp"

#include <unistd.h>
#include <cstdio>
#include <thread>
#include <iostream>

namespace vx01_locomotion_control {

static constexpr char KEY_W     = 'w';
static constexpr char KEY_S     = 's';
static constexpr char KEY_A     = 'a';
static constexpr char KEY_D     = 'd';
static constexpr char KEY_Q     = 'q';
static constexpr char KEY_E     = 'e';
static constexpr char KEY_SPACE = ' ';
static constexpr char KEY_X     = 'x';

TeleopNode::TeleopNode(const rclcpp::NodeOptions & options)
: Node("teleop_node", options)
{
    declare_parameter("linear_step",    0.02);
    declare_parameter("angular_step",   0.1);
    declare_parameter("max_linear_vel", 0.15);
    declare_parameter("max_angular_vel",0.5);

    linear_step_    = get_parameter("linear_step").as_double();
    angular_step_   = get_parameter("angular_step").as_double();
    max_linear_vel_ = get_parameter("max_linear_vel").as_double();
    max_angular_vel_= get_parameter("max_angular_vel").as_double();

    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    pub_timer_ = create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&TeleopNode::publishVelocity, this));

    struct termios raw;
    tcgetattr(STDIN_FILENO, &orig_termios_);
    terminal_saved_ = true;
    raw = orig_termios_;
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VMIN]  = 0;
    raw.c_cc[VTIME] = 1;
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);

    printHelp();

    std::thread([this]() { readLoop(); }).detach();
}

TeleopNode::~TeleopNode()
{
    restoreTerminal();
}

void TeleopNode::restoreTerminal()
{
    if (terminal_saved_) {
        tcsetattr(STDIN_FILENO, TCSANOW, &orig_termios_);
        terminal_saved_ = false;
    }
}

char TeleopNode::getKey()
{
    char c = 0;
    read(STDIN_FILENO, &c, 1);
    return c;
}

void TeleopNode::readLoop()
{
    while (rclcpp::ok()) {
        char c = getKey();
        if (c == 0) continue;

        bool changed = true;
        switch (c) {
            case KEY_W:
                vx_ = std::clamp(vx_ + linear_step_, -max_linear_vel_, max_linear_vel_);
                break;
            case KEY_S:
                vx_ = std::clamp(vx_ - linear_step_, -max_linear_vel_, max_linear_vel_);
                break;
            case KEY_A:
                omega_ = std::clamp(omega_ + angular_step_, -max_angular_vel_, max_angular_vel_);
                break;
            case KEY_D:
                omega_ = std::clamp(omega_ - angular_step_, -max_angular_vel_, max_angular_vel_);
                break;
            case KEY_Q:
                vy_ = std::clamp(vy_ + linear_step_, -max_linear_vel_, max_linear_vel_);
                break;
            case KEY_E:
                vy_ = std::clamp(vy_ - linear_step_, -max_linear_vel_, max_linear_vel_);
                break;
            case KEY_SPACE:
                vx_ = vy_ = omega_ = 0.0;
                break;
            case KEY_X:
                vx_ = vy_ = omega_ = 0.0;
                rclcpp::shutdown();
                break;
            default:
                changed = false;
                break;
        }

        if (changed) {
            RCLCPP_INFO(get_logger(),
                "vx=%.3f  vy=%.3f  omega=%.3f", vx_, vy_, omega_);
        }
    }
}

void TeleopNode::publishVelocity()
{
    geometry_msgs::msg::Twist msg;
    msg.linear.x  = vx_;
    msg.linear.y  = vy_;
    msg.angular.z = omega_;
    cmd_vel_pub_->publish(msg);
}

void TeleopNode::printHelp() const
{
    std::cout << R"(
  ╔══════════════════════════════════════╗
  ║   VX01 Hexapod Teleop               ║
  ╠══════════════════════════════════════╣
  ║  W / S   — forward / backward       ║
  ║  A / D   — turn left / turn right   ║
  ║  Q / E   — strafe left / right      ║
  ║  SPACE   — stop (zero velocity)     ║
  ║  X       — stop and exit            ║
  ╚══════════════════════════════════════╝
)" << std::endl;
}

}  // namespace vx01_locomotion_control

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<vx01_locomotion_control::TeleopNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
