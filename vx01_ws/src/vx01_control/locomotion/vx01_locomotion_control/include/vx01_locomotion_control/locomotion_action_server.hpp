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
//  Robot constants – sourced directly from engineering drawings & URDF
//
//  Drawing: vx01_leg.JPG
//    L1 (coxa)  = 60.55 mm
//    L2 (femur) = 73.84 mm
//    L3 (tibia) = 112.16 mm
//
//  Drawing: vx01_bezier_curve.JPG
//    S (reach / home_x) = 108.67 mm
//    T (stride length)  = 110.00 mm
//    A (step height)    =  22.78 mm
//
//  Drawing: vx01_hexapod.JPG
//    Body outer half-span = 110 mm  (220 mm / 2)
//    Inter-leg angle      = 62.91°
//
//  URDF: leg_N.xacro – coxa joint rpy-z values (mounting angles)
//  URDF: leg_N.xacro – coxa joint xyz magnitudes (pivot radii)
// ─────────────────────────────────────────────────────────────────────────────

static constexpr int NUM_LEGS       = 6;
static constexpr int JOINTS_PER_LEG = 3;
static constexpr int TOTAL_JOINTS   = NUM_LEGS * JOINTS_PER_LEG;  // 18

// ── Link lengths (mm) ─────────────────────────────────────────────────────
static constexpr double L1_MM = 60.55;    // coxa
static constexpr double L2_MM = 73.84;    // femur
static constexpr double L3_MM = 112.16;   // tibia

// ── Gait defaults (mm / s) ────────────────────────────────────────────────
static constexpr double DEFAULT_STEP_LENGTH = 110.00;   // T – stride
static constexpr double DEFAULT_STEP_HEIGHT =  22.78;   // A – lift
static constexpr double DEFAULT_STEP_PERIOD =   2.0;    // s – full 6-block cycle

// ── Home foot position in leg-local frame (mm) ───────────────────────────
static constexpr double HOME_X = 108.67;   // S – reach along coxa axis
static constexpr double HOME_Y =   0.0;
static constexpr double HOME_Z = -80.0;    // standing height below hip joint

// ── Coxa mounting angles (rad) – from URDF coxa_legN_joint rpy-z ─────────
//   leg0: rpy="0 0 -1.5708"  →  -π/2
//   leg1: rpy="0 0 -0.7854"  →  -π/4
//   leg2: rpy="0 0  0.7854"  →  +π/4
//   leg3: rpy="0 0  1.5708"  →  +π/2
//   leg4: rpy="0 0  2.392"   →  +2.392 rad
//   leg5: rpy="0 0 -2.3562"  →  -2.3562 rad
static constexpr std::array<double, NUM_LEGS> LEG_MOUNTING_ANGLES_RAD = {
    -M_PI_2,          // leg 0  –90°
    -M_PI_4,          // leg 1  –45°
     M_PI_4,          // leg 2  +45°
     M_PI_2,          // leg 3  +90°
     2.392,           // leg 4  +137°
    -2.3562           // leg 5  –135°
};

// ── Per-leg coxa pivot radius from body centre (mm) ──────────────────────
//   Computed from URDF coxa_legN_joint xyz: r = sqrt(x² + y²) * 1000
//   leg0: (-0.0013383, -0.10566) → 105.67 mm
//   leg1: ( 0.1215,   -0.06284) → 136.79 mm
//   leg2: ( 0.1215,    0.06282) → 136.78 mm
//   leg3: (-0.0013383,  0.10564) → 105.65 mm
//   leg4: (-0.12417,    0.06282) → 139.16 mm
//   leg5: (-0.12417,   -0.06284) → 139.17 mm
static constexpr std::array<double, NUM_LEGS> LEG_PIVOT_RADII_MM = {
    105.67,   // leg 0
    136.79,   // leg 1
    136.78,   // leg 2
    105.65,   // leg 3
    139.16,   // leg 4
    139.17    // leg 5
};

// ── beta_angle passed to HexapodLocomotion (62.91° from drawing) ─────────
static constexpr double BETA_ANGLE_RAD = 1.09792;  // 62.91 * π/180

// ── Joint names – must match vx01_controller_manager.yaml exactly ────────
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
//  VX01LocomotionServer – ROS 2 action server node
//
//  Topology:
//    [Walk action client]
//          │  vx01/walk  (Walk action)
//          ▼
//    [VX01LocomotionServer]
//          │  leg_N_controller/follow_joint_trajectory  (FJT action)
//          ▼
//    [6 × JointTrajectoryController]  →  Gazebo / real hardware
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
    // ── Walk action-server callbacks ──────────────────────────────────────
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const Walk::Goal> goal);

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<WalkGoalHandle> goal_handle);

    void handle_accepted(
        const std::shared_ptr<WalkGoalHandle> goal_handle);

    // ── Core walking loop (runs in its own thread) ─────────────────────────
    void execute_walk(const std::shared_ptr<WalkGoalHandle> goal_handle);

    // ── Locomotion helpers ─────────────────────────────────────────────────
    //  Move all legs to the home standing position and send to controllers.
    void move_to_stand();

    //  Send current joint angles (from hexapod_) to all 6 FJT servers.
    //  traj_dt : time_from_start for the single trajectory waypoint.
    void send_all_legs(double traj_dt);

    //  Publish one JointTrajectory message to a single leg's topic.
    //  NOTE: joint angles are negated here because all URDF joint axes
    //  are defined as xyz="0 0 -1" (negative Z), while the DH-based IK
    //  computes angles for positive Z rotation convention.
    void publish_leg_trajectory(
        int  leg_index,
        const std::array<double, JOINTS_PER_LEG>& dh_angles,
        double traj_dt);

    // ── Parameter helpers ─────────────────────────────────────────────────
    void declare_parameters();
    void load_parameters();

    // ── Build the hexapod locomotion object with per-leg x_start ──────────
    //  The stock HexapodLocomotion::initializeLegControllers() uses a single
    //  body_radius for all legs.  Because the VX-01 has unequal pivot radii
    //  we build the LegControllers ourselves via the public interface.
    void init_hexapod();

    // ── Members ───────────────────────────────────────────────────────────
    rclcpp_action::Server<Walk>::SharedPtr           walk_server_;

    // One publisher per leg → /leg_N_controller/joint_trajectory
    // Topic-based streaming avoids FJT action preemption (code=5).
    std::array<JointTrajPub::SharedPtr, NUM_LEGS>    traj_pubs_;

    // Top-level locomotion engine from the hexapod library
    std::unique_ptr<vx01_hexapod_locomotion::HexapodLocomotion> hexapod_;

    std::atomic<bool> cancel_requested_{false};

    // Parameters (set at startup, overridable per-goal)
    double p_L1_, p_L2_, p_L3_;
    double p_step_length_, p_step_height_, p_step_period_;
    double p_home_x_, p_home_y_, p_home_z_;
    double p_update_rate_hz_;     // control loop Hz
    double p_traj_dt_;            // trajectory waypoint duration (s)
    double p_stand_traj_dt_;      // slower duration used for stand moves
};

}  // namespace vx01_locomotion_control

#endif  // VX01_LOCOMOTION_CONTROL_ACTION_SERVER_HPP