#ifndef VX01_LOCOMOTION_CONTROL_HEXAPOD_TURN_NODE_HPP
#define VX01_LOCOMOTION_CONTROL_HEXAPOD_TURN_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include "vx01_hexapod_locomotion/hexapod_locomotion.hpp"
#include <vector>
#include <string>
#include <memory>
#include <thread>
#include <atomic>

namespace vx01_locomotion_control {

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using CallbackReturn        = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class HexapodTurnNode : public rclcpp_lifecycle::LifecycleNode {
public:
    explicit HexapodTurnNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~HexapodTurnNode();

    CallbackReturn on_configure(const rclcpp_lifecycle::State&)  override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State&)   override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State&)    override;
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State&)   override;

private:
    std::shared_ptr<vx01_hexapod_locomotion::HexapodLocomotion> locomotion_;
    std::vector<rclcpp_action::Client<FollowJointTrajectory>::SharedPtr> action_clients_;
    std::vector<std::vector<std::string>> joint_names_;
    std::vector<std::string> controller_names_;

    // gait_timer_ is created on the executor thread via startup_check_timer_
    rclcpp::TimerBase::SharedPtr gait_timer_;
    // polls standby_done_ from the executor thread, then creates gait_timer_
    rclcpp::TimerBase::SharedPtr startup_check_timer_;

    std::thread       startup_thread_;
    std::atomic<bool> standby_done_{false};
    std::atomic<bool> stop_requested_{false};

    double L1_, L2_, L3_;
    double body_radius_, beta_angle_;
    double home_x_, home_y_, home_z_;
    double step_length_, step_height_, step_period_;
    double standby_coxa_, standby_femur_, standby_tibia_;
    double standby_duration_, update_rate_, block_period_;
    int    turn_direction_;
    std::vector<double> leg_angles_;

    int    last_sent_block_;
    std::vector<double> last_sent_angles_;

    void declareParameters();
    void loadParameters();
    void startupSequence();
    void sendStandbyPose();
    void startTurning();
    void gaitUpdate();
    void sampleTurnSwingAt(int leg_index, double global_t,
                           double& theta1, double& theta2, double& theta3);
    void sampleTurnDragAt(int leg_index, double global_t,
                          double& theta1, double& theta2, double& theta3);
    void sendLegTrajectory(int leg_index, double theta1, double theta2, double theta3,
                           double duration_sec);
};

}
#endif