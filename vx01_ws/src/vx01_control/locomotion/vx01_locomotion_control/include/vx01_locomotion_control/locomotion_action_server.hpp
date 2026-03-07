#ifndef VX01_LOCOMOTION_CONTROL_ACTION_SERVER_HPP
#define VX01_LOCOMOTION_CONTROL_ACTION_SERVER_HPP

// ── ROS 2 ──────────────────────────────────────────────────────────────────
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

// ── ROS 2 message types ────────────────────────────────────────────────────
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

// ── Generated Walk action ──────────────────────────────────────────────────
#include <vx01_locomotion_control/action/walk.hpp>

// ── Hexapod locomotion library ─────────────────────────────────────────────
#include "vx01_hexapod_locomotion/hexapod_locomotion.hpp"

// ── STL ────────────────────────────────────────────────────────────────────
#include <array>
#include <string>
#include <vector>
#include <memory>
#include <atomic>

namespace vx01_locomotion_control {

// ─────────────────────────────────────────────────────────────────────────────
//  Robot constants – sourced from engineering drawings & URDF
// ─────────────────────────────────────────────────────────────────────────────

static constexpr int NUM_LEGS       = 6;
static constexpr int JOINTS_PER_LEG = 3;
static constexpr int TOTAL_JOINTS   = NUM_LEGS * JOINTS_PER_LEG;  // 18

// ── Link lengths (mm) ─────────────────────────────────────────────────────
static constexpr double L1_MM = 60.55;
static constexpr double L2_MM = 73.84;
static constexpr double L3_MM = 112.16;

// ── Gait defaults ─────────────────────────────────────────────────────────
// NOTE: step_length and step_height are deliberately small.
// The home position (223 mm reach) is near the kinematic extension limit.
// Larger strides would push the limb outside the reachable workspace.
static constexpr double DEFAULT_STEP_LENGTH =  20.0;   // T – safe stride (mm)
static constexpr double DEFAULT_STEP_HEIGHT =  15.0;   // A – safe lift (mm)
static constexpr double DEFAULT_STEP_PERIOD =   2.0;   // s – full cycle

// ── Home foot position (leg-local frame, mm) ─────────────────────────────
// These values are derived analytically from the desired standing posture:
//   femur URDF = -0.785 rad (-45°), tibia URDF = +0.600 rad (+34°)
// With elbow-DOWN IK and DH=URDF (no sign flip), the FK of those angles gives:
//   home_x = 223.03 mm,  home_z = -72.82 mm  (foot is 72.8 mm below coxa)
static constexpr double HOME_X = 223.03;
static constexpr double HOME_Y =   0.0;
static constexpr double HOME_Z = -72.82;

// ── Body geometry ─────────────────────────────────────────────────────────
static constexpr double BETA_ANGLE_RAD = 1.09792;   // 62.91°

// ── Joint names – must match controller_manager YAML exactly ─────────────
static const std::array<std::array<std::string, JOINTS_PER_LEG>, NUM_LEGS>
JOINT_NAMES = {{
    {{"coxa_leg0_joint", "femur_leg0_joint", "tibia_leg0_joint"}},
    {{"coxa_leg1_joint", "femur_leg1_joint", "tibia_leg1_joint"}},
    {{"coxa_leg2_joint", "femur_leg2_joint", "tibia_leg2_joint"}},
    {{"coxa_leg3_joint", "femur_leg3_joint", "tibia_leg3_joint"}},
    {{"coxa_leg4_joint", "femur_leg4_joint", "tibia_leg4_joint"}},
    {{"coxa_leg5_joint", "femur_leg5_joint", "tibia_leg5_joint"}}
}};

// ─────────────────────────────────────────────────────────────────────────────
//  VX01LocomotionServer
// ─────────────────────────────────────────────────────────────────────────────
class VX01LocomotionServer : public rclcpp::Node
{
public:
    using Walk           = vx01_locomotion_control::action::Walk;
    using WalkGoalHandle = rclcpp_action::ServerGoalHandle<Walk>;
    using JointTraj      = trajectory_msgs::msg::JointTrajectory;
    using JointTrajPub   = rclcpp::Publisher<JointTraj>;

    explicit VX01LocomotionServer(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    ~VX01LocomotionServer() override = default;

private:
    // ── Walk action-server callbacks ─────────────────────────────────────
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const Walk::Goal> goal);

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<WalkGoalHandle> goal_handle);

    void handle_accepted(
        const std::shared_ptr<WalkGoalHandle> goal_handle);

    // ── Core walking loop (runs in detached thread) ───────────────────────
    void execute_walk(const std::shared_ptr<WalkGoalHandle> goal_handle);

    // ── Locomotion helpers ────────────────────────────────────────────────
    void standup_sequence();      // smooth interpolation from zeros to home
    void move_to_stand();         // single-shot stand at slow traj speed
    void send_all_legs(double traj_dt);

    // Publish one JointTrajectory to a single leg's topic.
    // DH angles are sent directly without sign negation (elbow-DOWN IK maps
    // naturally to URDF-commanded angles for the VX-01 joint convention).
    void publish_leg_trajectory(
        int leg_index,
        const std::array<double, JOINTS_PER_LEG>& dh_angles,
        double traj_dt);

    // ── Parameter helpers ─────────────────────────────────────────────────
    void declare_parameters();
    void load_parameters();
    void init_hexapod();

    // ── Members ──────────────────────────────────────────────────────────
    rclcpp_action::Server<Walk>::SharedPtr walk_server_;

    // One publisher per leg → /leg_N_controller/joint_trajectory
    std::array<JointTrajPub::SharedPtr, NUM_LEGS> traj_pubs_;

    std::unique_ptr<vx01_hexapod_locomotion::HexapodLocomotion> hexapod_;
    std::atomic<bool> cancel_requested_{false};

    // ── Runtime parameters ────────────────────────────────────────────────
    double p_L1_, p_L2_, p_L3_;
    double p_step_length_, p_step_height_, p_step_period_;
    double p_home_x_, p_home_y_, p_home_z_;
    double p_update_rate_hz_;
    double p_traj_dt_;
    double p_stand_traj_dt_;
    int    p_standup_steps_;
};

}  // namespace vx01_locomotion_control

#endif  // VX01_LOCOMOTION_CONTROL_ACTION_SERVER_HPP