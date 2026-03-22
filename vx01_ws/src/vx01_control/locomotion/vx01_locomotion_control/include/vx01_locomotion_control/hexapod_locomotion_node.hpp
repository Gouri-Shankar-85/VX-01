#ifndef VX01_LOCOMOTION_CONTROL_HEXAPOD_LOCOMOTION_NODE_HPP
#define VX01_LOCOMOTION_CONTROL_HEXAPOD_LOCOMOTION_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include "vx01_hexapod_locomotion/hexapod_locomotion.hpp"

#include <vector>
#include <string>
#include <memory>

namespace vx01_locomotion_control {

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandleFJT = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

class HexapodLocomotionNode : public rclcpp::Node {

public:
    explicit HexapodLocomotionNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    std::shared_ptr<vx01_hexapod_locomotion::HexapodLocomotion> locomotion_;

    std::vector<rclcpp_action::Client<FollowJointTrajectory>::SharedPtr> action_clients_;
    std::vector<std::vector<std::string>> joint_names_;
    std::vector<std::string> controller_names_;

    rclcpp::TimerBase::SharedPtr gait_timer_;

    double standby_coxa_;
    double standby_femur_;
    double standby_tibia_;
    double standby_duration_;
    double step_period_;
    double update_rate_;

    bool   standby_done_;
    int    last_sent_block_;
    double block_period_;
    std::vector<double> last_sent_angles_;  
    void sendStandbyPose();
    void startWalking();
    void gaitUpdate();

    void sendLegTrajectory(int leg_index,
                           double theta1, double theta2, double theta3,
                           double duration_sec);

    bool allClientsReady();
};

}

#endif