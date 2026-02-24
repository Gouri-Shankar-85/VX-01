#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>

#include "vx01_hexapod_locomotion/hexapod_locomotion.hpp"

class HexapodTest : public rclcpp::Node
{
public:
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HexapodTest>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
