#include "vx01_locomotion_control/locomotion_action_server.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    // MultiThreadedExecutor is mandatory:
    //  – Thread A: long-running execute_walk() inside handle_accepted()
    //  – Thread B: cancel callbacks, FJT result callbacks, feedback publishing
    auto executor =
        std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

    auto node = std::make_shared<vx01_locomotion_control::VX01LocomotionServer>(
        rclcpp::NodeOptions()
            .automatically_declare_parameters_from_overrides(false));

    executor->add_node(node);
    executor->spin();

    rclcpp::shutdown();
    return 0;
}
