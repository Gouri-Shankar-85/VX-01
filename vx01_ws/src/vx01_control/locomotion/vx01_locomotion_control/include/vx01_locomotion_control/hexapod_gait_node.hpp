#ifndef HEXAPOD_GAIT_NODE_HPP
#define HEXAPOD_GAIT_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/twist.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <array>
#include <vector>
#include <string>
#include <memory>

namespace hexapod_gait
{

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandleFJT = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

struct LegGeometry
{
    std::string coxa_frame;
    std::string base_frame;
    std::array<std::string, 3> joint_names;
};

enum class LegPhase { SWING, STANCE };

class HexapodGaitNode : public rclcpp::Node
{
public:
    explicit HexapodGaitNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    // Parameters
    double L1_, L2_, L3_;
    double home_reach_;
    double home_z_;
    double step_length_;
    double step_height_;
    double step_period_;
    double standby_duration_;
    double update_rate_;
    std::vector<std::string> controller_names_;

    // Velocity command (normalised, set by teleop)
    double cmd_vx_;
    double cmd_vy_;
    double cmd_omega_;

    // TF2
    std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Per-leg state
    static constexpr int NUM_LEGS = 6;
    std::array<LegGeometry, NUM_LEGS>                                       leg_geometry_;
    std::array<rclcpp_action::Client<FollowJointTrajectory>::SharedPtr, NUM_LEGS> action_clients_;
    std::array<std::array<double, 3>, NUM_LEGS>                             last_angles_;
    std::array<std::array<double, 3>, NUM_LEGS>                             foot_base_;

    // Gait state
    // Tripod gait: 2 blocks per full cycle.
    //   Block 0: legs {0,2,4} SWING, legs {1,3,5} STANCE
    //   Block 1: legs {1,3,5} SWING, legs {0,2,4} STANCE
    int    gait_block_;
    double gait_time_;
    double block_period_;
    bool   initialized_;

    rclcpp::TimerBase::SharedPtr       gait_timer_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

    // Internal helpers
    void loadParameters();
    void initLegGeometry();
    void initHomeFootPositions();
    void waitForControllers();
    void sendStandby();
    void startGait();
    void gaitTick();
    void executeBlock();
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

    bool footBaseToCoxa(int leg, const std::array<double, 3> & foot_base,
                        std::array<double, 3> & foot_coxa) ;

    bool solveIK(double x, double y, double z,
                 double & t1, double & t2, double & t3) const;

    std::array<double, 3> swingFootBase(int leg, double global_t) const;
    std::array<double, 3> stanceFootBase(int leg) const;

    void sendTrajectory(int leg,
                        const std::vector<std::array<double, 3>> & waypoints,
                        const std::vector<double> & times);

    bool allControllersReady() const;
};

} // namespace hexapod_gait

#endif // HEXAPOD_GAIT_NODE_HPP
