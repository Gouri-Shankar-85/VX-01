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
//  Robot constants
// ─────────────────────────────────────────────────────────────────────────────

static constexpr int NUM_LEGS       = 6;
static constexpr int JOINTS_PER_LEG = 3;
static constexpr int TOTAL_JOINTS   = NUM_LEGS * JOINTS_PER_LEG;   // 18

// ── Link lengths (mm) from leg.JPG CAD drawing ───────────────────────────
static constexpr double L1_MM = 60.55;    // COXA
static constexpr double L2_MM = 73.84;    // FEMUR
static constexpr double L3_MM = 112.16;   // TIBIA

// ── Gait defaults (from bezier_curve.JPG: T=110, A=22.78, S=108.67) ──────
static constexpr double DEFAULT_STEP_LENGTH = 110.0;   // T – stride length (mm)
static constexpr double DEFAULT_STEP_HEIGHT =  22.78;  // A – foot lift (mm)
static constexpr double DEFAULT_STEP_PERIOD =   2.0;   // s – full 6-block cycle

// ── Home foot position (leg-local frame, mm) ─────────────────────────────
// FK verified: theta1=0, theta2=39.6 deg, theta3=-118.29 deg
// gives approximately x=170, z=-100 for the default tutorial robot.
// For THIS robot (L1=60.55, L2=73.84, L3=112.16) we use a safe standing pose.
static constexpr double HOME_X = 170.0;   // forward reach along coxa axis (mm)
static constexpr double HOME_Y =   0.0;
static constexpr double HOME_Z = -80.0;   // foot below coxa pivot (mm)

// ── Body geometry from hexapod.JPG CAD drawing ───────────────────────────
// beta = 62.91 deg = 1.09792 rad (angle between adjacent leg mounts)
static constexpr double BETA_ANGLE_RAD = 1.09792;   // 62.91 deg

// ── Body radius = distance from centre to coxa pivot (mm) ────────────────
// From hexapod.JPG: half the 100 mm inner span = 50 mm centre-to-mount
// (adjust to match your URDF's base_link → coxa_pivot TF translation)
static constexpr double BODY_RADIUS_MM = 50.0;

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
    void standup_sequence();
    void move_to_stand();
    void send_all_legs(double traj_dt);

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

    std::array<JointTrajPub::SharedPtr, NUM_LEGS> traj_pubs_;

    std::unique_ptr<vx01_hexapod_locomotion::HexapodLocomotion> hexapod_;
    std::atomic<bool> cancel_requested_{false};

    // ── Runtime parameters ────────────────────────────────────────────────
    double p_L1_, p_L2_, p_L3_;
    double p_step_length_, p_step_height_, p_step_period_;
    double p_home_x_, p_home_y_, p_home_z_;
    double p_body_radius_;
    double p_update_rate_hz_;
    double p_traj_dt_;
    double p_stand_traj_dt_;
    int    p_standup_steps_;
};

}  // namespace vx01_locomotion_control

#endif  // VX01_LOCOMOTION_CONTROL_ACTION_SERVER_HPP