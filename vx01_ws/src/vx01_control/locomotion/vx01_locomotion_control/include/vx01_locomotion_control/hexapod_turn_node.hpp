#ifndef VX01_LOCOMOTION_CONTROL_HEXAPOD_TURN_NODE_HPP
#define VX01_LOCOMOTION_CONTROL_HEXAPOD_TURN_NODE_HPP

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
using GoalHandleFJT         = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

class HexapodTurnNode : public rclcpp::Node {

public:
    explicit HexapodTurnNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    std::shared_ptr<vx01_hexapod_locomotion::HexapodLocomotion> locomotion_;

    std::vector<rclcpp_action::Client<FollowJointTrajectory>::SharedPtr> action_clients_;
    std::vector<std::vector<std::string>> joint_names_;
    std::vector<std::string>              controller_names_;

    rclcpp::TimerBase::SharedPtr gait_timer_;

    double L1_, L2_, L3_;
    double body_radius_;
    double beta_angle_;
    std::vector<double> leg_angles_;  

    double home_x_, home_y_, home_z_;
    double step_length_, step_height_, step_period_;

    double standby_coxa_, standby_femur_, standby_tibia_;
    double standby_duration_;
    double update_rate_;

    // turn_direction: +1 = CCW (left), -1 = CW (right)
    int    turn_direction_;

    bool   standby_done_;
    int    last_sent_block_;
    double block_period_;
    std::vector<double> last_sent_angles_;   

    void sendStandbyPose();
    void startTurning();

    void gaitUpdate();

    void sampleTurnSwingAt(int leg_index, double global_t,
                           double& theta1, double& theta2, double& theta3);

    void sampleTurnDragAt(int leg_index, double global_t,
                          double& theta1, double& theta2, double& theta3);

    void sendLegTrajectory(int leg_index,
                           double theta1, double theta2, double theta3,
                           double duration_sec);
    bool allClientsReady();
};

}  

#endif  