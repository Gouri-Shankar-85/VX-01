#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/empty.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include "vx01_hexapod_locomotion/hexapod_locomotion.hpp"
#include "vx01_hexapod_locomotion/kinematics/inverse_kinematics.hpp"

#include <vector>
#include <string>
#include <memory>
#include <array>
#include <atomic>

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
    void initJointStateSubscriber();

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

    void gaitCycleTimer();
    void executeGaitBlock();
    void sendStandUpTrajectory();
    void sendLegTrajectory(int leg_id, bool is_swing, double block_duration);
    void sendHalfCycle();
    void sendFullHalfCycleTraj(int leg_id, bool is_swing, double duration, int this_cycle);

    // Per-leg stride amplitude in leg-local X and Y for current cmd_vel
    void legStride(int leg_id, double& stride_x, double& stride_y) const;

    bool computeIK(double lx, double ly, double lz,
                   double& t1, double& t2, double& t3) const;

    std::array<double, 3> swingWaypoint(int leg_id, double t) const;
    std::array<double, 3> stanceWaypoint(int leg_id, double t) const;

    trajectory_msgs::msg::JointTrajectory buildSwingTrajectory(
        int leg_id, double block_duration);
    trajectory_msgs::msg::JointTrajectory buildStanceTrajectory(
        int leg_id, double block_duration);

    std::shared_ptr<vx01_hexapod_locomotion::HexapodLocomotion> locomotion_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr    cmd_vel_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr         go_home_sub_;
    rclcpp::TimerBase::SharedPtr gait_timer_;
    rclcpp::TimerBase::SharedPtr stand_up_timer_;

    std::array<rclcpp_action::Client<FollowJointTrajectory>::SharedPtr, 6> action_clients_;
    std::array<std::vector<std::string>, 6> joint_names_;
    std::array<std::string, 6>              controller_names_;

    std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    double L1_, L2_, L3_;
    double body_radius_, beta_angle_;
    double home_x_, home_y_, home_z_;
    double step_length_, step_height_, step_period_;
    double stand_duration_;
    double max_linear_vel_, max_angular_vel_;
    int    n_waypoints_;
    std::string base_frame_;
    std::array<std::string, 6> coxa_frames_;
    std::array<double, 6>      leg_angles_;

    double cmd_vx_{0.0}, cmd_vy_{0.0}, cmd_omega_{0.0};
    bool   walking_{false};
    int    half_cycle_group_{0};  // 0=GroupA swings, 1=GroupB swings
    std::atomic<int> legs_done_{0};
    std::atomic<int> cycle_id_{0};   // monotonically incrementing generation ID
    std::array<bool, 6>    goal_active_{};
    std::array<double, 18> current_joint_state_{};
    bool   joint_state_received_{false};
};

}  // namespace vx01_locomotion_control