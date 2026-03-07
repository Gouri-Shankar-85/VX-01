#ifndef VX01_LOCOMOTION_CONTROL_ACTION_SERVER_HPP
#define VX01_LOCOMOTION_CONTROL_ACTION_SERVER_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <vx01_locomotion_control/action/walk.hpp>
#include "vx01_hexapod_locomotion/hexapod_locomotion.hpp"
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
static constexpr int TOTAL_JOINTS   = NUM_LEGS * JOINTS_PER_LEG;

// ── Link lengths (mm) from leg.JPG CAD ───────────────────────────────────
static constexpr double L1_MM = 60.55;
static constexpr double L2_MM = 73.84;
static constexpr double L3_MM = 112.16;

// ── Gait defaults from bezier_curve.JPG drawing ───────────────────────────
static constexpr double DEFAULT_STEP_LENGTH = 110.0;   // T (mm)
static constexpr double DEFAULT_STEP_HEIGHT =  22.78;  // A (mm)
static constexpr double DEFAULT_STEP_PERIOD =   2.0;   // s

// ── Home foot position (leg-local frame, mm) ──────────────────────────────
// Computed by FK from desired URDF stand angles:
//   coxa=0.0 rad, femur=-0.785 rad (-45 deg), tibia=+0.600 rad (+34 deg)
// FK result: x=223.03 mm, y=0 mm, z=-72.82 mm
// Round-trip IK:  DH theta2=-0.0577, theta3=-0.600
//   URDF femur = theta2 + FEMUR_URDF_OFFSET = -0.0577 + (-0.7273) = -0.785  OK
//   URDF tibia = -theta3 = +0.600                                            OK
static constexpr double HOME_X =  223.03;
static constexpr double HOME_Y =    0.0;
static constexpr double HOME_Z =  -72.82;

// ── DH-to-URDF angle conversion ───────────────────────────────────────────
// The URDF joint zero positions differ from the DH frame zeros.
// Conversion applied inside publish_leg_trajectory():
//
//   coxa_urdf  = -dh_theta1
//   femur_urdf =  dh_theta2 + FEMUR_URDF_OFFSET
//   tibia_urdf = -dh_theta3
//
// FEMUR_URDF_OFFSET verified: IK at (223.03, 0, -72.82) gives theta2=-0.0577
//   -0.0577 + (-0.7273) = -0.7850 ≈ -0.785 (desired URDF femur)
static constexpr double FEMUR_URDF_OFFSET = -0.7273;   // rad

// ── Body geometry from hexapod.JPG CAD ───────────────────────────────────
static constexpr double BETA_ANGLE_RAD = 1.09792;   // 62.91 deg
static constexpr double BODY_RADIUS_MM = 50.0;       // centre to coxa pivot (mm)

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
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const Walk::Goal> goal);

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<WalkGoalHandle> goal_handle);

    void handle_accepted(
        const std::shared_ptr<WalkGoalHandle> goal_handle);

    void execute_walk(const std::shared_ptr<WalkGoalHandle> goal_handle);

    void standup_sequence();
    void move_to_stand();
    void send_all_legs(double traj_dt);

    void publish_leg_trajectory(
        int leg_index,
        const std::array<double, JOINTS_PER_LEG>& dh_angles,
        double traj_dt);

    void declare_parameters();
    void load_parameters();
    void init_hexapod();

    rclcpp_action::Server<Walk>::SharedPtr walk_server_;
    std::array<JointTrajPub::SharedPtr, NUM_LEGS> traj_pubs_;
    std::unique_ptr<vx01_hexapod_locomotion::HexapodLocomotion> hexapod_;
    std::atomic<bool> cancel_requested_{false};

    double p_L1_, p_L2_, p_L3_;
    double p_body_radius_;
    double p_step_length_, p_step_height_, p_step_period_;
    double p_home_x_, p_home_y_, p_home_z_;
    double p_femur_urdf_offset_;
    double p_update_rate_hz_;
    double p_traj_dt_;
    double p_stand_traj_dt_;
    int    p_standup_steps_;
};

}  // namespace vx01_locomotion_control

#endif