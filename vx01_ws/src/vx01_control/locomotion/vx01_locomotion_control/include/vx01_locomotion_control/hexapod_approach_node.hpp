#ifndef VX01_LOCOMOTION_CONTROL_HEXAPOD_APPROACH_NODE_HPP
#define VX01_LOCOMOTION_CONTROL_HEXAPOD_APPROACH_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include "vx01_msgs/msg/victim_array.hpp"
#include <memory>
#include <string>

namespace vx01_locomotion_control {

class HexapodApproachNode : public rclcpp::Node {
public:
    explicit HexapodApproachNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    enum class State { IDLE, TURNING, WALKING, ARRIVED };

    // TF
    std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Lifecycle service clients
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr walk_change_state_;
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr turn_change_state_;
    rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr    walk_get_state_;
    rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr    turn_get_state_;

    // Victim subscription
    rclcpp::Subscription<vx01_msgs::msg::VictimArray>::SharedPtr victim_sub_;

    // Control timer
    rclcpp::TimerBase::SharedPtr control_timer_;

    // State
    State  state_;
    double victim_x_;   // in base_link frame
    double victim_y_;
    bool   victim_valid_;

    // Parameters
    double arrival_distance_;
    double turn_threshold_;

    void cbVictims(const vx01_msgs::msg::VictimArray::SharedPtr msg);
    void controlLoop();

    void activateWalk();
    void deactivateWalk();
    void activateTurn(int direction);
    void deactivateTurn();

    void changeState(
        rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr client,
        uint8_t transition_id);

    uint8_t getState(
        rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr client);
};

}
#endif