#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include "vx01_hexapod_locomotion/hexapod_locomotion.hpp"
#include "vx01_hexapod_locomotion/control/leg_controller.hpp"

#include <vector>
#include <string>
#include <memory>
#include <array>

namespace vx01_locomotion_control {

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandleFJT         = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

class TripodWalkNode : public rclcpp::Node {
public:
    explicit TripodWalkNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    void declareParameters();
    void loadParameters();
    void initLocomotion();
    void initActionClients();
    void initTF();
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

    void gaitCycleTimer();
    void executeGaitBlock();

    void sendLegTrajectory(int leg_id, bool is_swing, double block_duration);

    trajectory_msgs::msg::JointTrajectory buildSwingTrajectory(
        int leg_id, double block_duration);
    trajectory_msgs::msg::JointTrajectory buildStanceTrajectory(
        int leg_id, double block_duration);

    void broadcastLegFrames();
    void scaledFootTarget(int leg_id, double& foot_x, double& foot_y) const;

    std::array<double, 3> computeSwingWaypoint(int leg_id, double t_norm);
    std::array<double, 3> computeStanceWaypoint(int leg_id, double t_norm);

    std::shared_ptr<vx01_hexapod_locomotion::HexapodLocomotion> locomotion_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::TimerBase::SharedPtr gait_timer_;

    std::array<rclcpp_action::Client<FollowJointTrajectory>::SharedPtr, 6> action_clients_;
    std::array<std::vector<std::string>, 6> joint_names_;
    std::array<std::string, 6>             controller_names_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::Buffer>               tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener>    tf_listener_;

    double L1_, L2_, L3_;
    double body_radius_, beta_angle_;
    double home_x_, home_y_, home_z_;
    double step_length_, step_height_, step_period_;
    double max_linear_vel_, max_angular_vel_;
    int    n_waypoints_;
    std::string base_frame_;
    std::array<std::string, 6> coxa_frames_;

    double cmd_vx_{0.0}, cmd_vy_{0.0}, cmd_omega_{0.0};
    bool   walking_{false};
    int    current_block_{0};
    std::array<bool, 6> goal_active_{};
};

}  // namespace vx01_locomotion_control
