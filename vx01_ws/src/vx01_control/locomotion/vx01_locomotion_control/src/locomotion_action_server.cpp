#include "vx01_locomotion_control/locomotion_action_server.hpp"

#include <chrono>
#include <cmath>
#include <thread>
#include <stdexcept>

using namespace std::chrono_literals;

namespace vx01_locomotion_control {

// ─────────────────────────────────────────────────────────────────────────────
//  Constructor
// ─────────────────────────────────────────────────────────────────────────────
VX01LocomotionServer::VX01LocomotionServer(const rclcpp::NodeOptions& options)
    : Node("vx01_locomotion_server", options)
{
    declare_parameters();
    load_parameters();
    init_hexapod();

    for (int i = 0; i < NUM_LEGS; ++i) {
        const std::string topic =
            "leg_" + std::to_string(i) + "_controller/joint_trajectory";
        traj_pubs_[i] = create_publisher<JointTraj>(topic, 1);
    }

    using std::placeholders::_1;
    using std::placeholders::_2;

    walk_server_ = rclcpp_action::create_server<Walk>(
        this,
        "vx01/walk",
        std::bind(&VX01LocomotionServer::handle_goal,     this, _1, _2),
        std::bind(&VX01LocomotionServer::handle_cancel,   this, _1),
        std::bind(&VX01LocomotionServer::handle_accepted, this, _1));

    RCLCPP_INFO(get_logger(),
        "VX01LocomotionServer ready -- action: vx01/walk");
    RCLCPP_INFO(get_logger(),
        "  L1=%.2f  L2=%.2f  L3=%.2f mm  body_radius=%.2f mm",
        p_L1_, p_L2_, p_L3_, p_body_radius_);
    RCLCPP_INFO(get_logger(),
        "  home=(%.2f, %.2f, %.2f) mm  T=%.1f mm  A=%.2f mm  period=%.2f s",
        p_home_x_, p_home_y_, p_home_z_,
        p_step_length_, p_step_height_, p_step_period_);
    RCLCPP_INFO(get_logger(),
        "  femur_urdf_offset=%.4f rad  (DH->URDF femur correction)",
        p_femur_urdf_offset_);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Parameter helpers
// ─────────────────────────────────────────────────────────────────────────────
void VX01LocomotionServer::declare_parameters()
{
    declare_parameter("L1",                 L1_MM);
    declare_parameter("L2",                 L2_MM);
    declare_parameter("L3",                 L3_MM);
    declare_parameter("body_radius",        BODY_RADIUS_MM);
    declare_parameter("step_length",        DEFAULT_STEP_LENGTH);
    declare_parameter("step_height",        DEFAULT_STEP_HEIGHT);
    declare_parameter("step_period",        DEFAULT_STEP_PERIOD);
    declare_parameter("home_x",             HOME_X);
    declare_parameter("home_y",             HOME_Y);
    declare_parameter("home_z",             HOME_Z);
    declare_parameter("femur_urdf_offset",  FEMUR_URDF_OFFSET);
    declare_parameter("update_rate_hz",     50.0);
    declare_parameter("traj_dt",            0.02);
    declare_parameter("stand_traj_dt",      0.08);
    declare_parameter("standup_steps",      40);
}

void VX01LocomotionServer::load_parameters()
{
    p_L1_               = get_parameter("L1").as_double();
    p_L2_               = get_parameter("L2").as_double();
    p_L3_               = get_parameter("L3").as_double();
    p_body_radius_      = get_parameter("body_radius").as_double();
    p_step_length_      = get_parameter("step_length").as_double();
    p_step_height_      = get_parameter("step_height").as_double();
    p_step_period_      = get_parameter("step_period").as_double();
    p_home_x_           = get_parameter("home_x").as_double();
    p_home_y_           = get_parameter("home_y").as_double();
    p_home_z_           = get_parameter("home_z").as_double();
    p_femur_urdf_offset_= get_parameter("femur_urdf_offset").as_double();
    p_update_rate_hz_   = get_parameter("update_rate_hz").as_double();
    p_traj_dt_          = get_parameter("traj_dt").as_double();
    p_stand_traj_dt_    = get_parameter("stand_traj_dt").as_double();
    p_standup_steps_    = get_parameter("standup_steps").as_int();
}

// ─────────────────────────────────────────────────────────────────────────────
//  Build HexapodLocomotion
// ─────────────────────────────────────────────────────────────────────────────
void VX01LocomotionServer::init_hexapod()
{
    hexapod_ = std::make_unique<vx01_hexapod_locomotion::HexapodLocomotion>(
        p_L1_, p_L2_, p_L3_,
        p_body_radius_,
        BETA_ANGLE_RAD);

    hexapod_->setStepLength(p_step_length_);
    hexapod_->setStepHeight(p_step_height_);
    hexapod_->setStepPeriod(p_step_period_);
    hexapod_->setHomePosition(p_home_x_, p_home_y_, p_home_z_);

    RCLCPP_INFO(get_logger(), "Hexapod locomotion engine initialised.");
}

// ─────────────────────────────────────────────────────────────────────────────
//  Action callbacks
// ─────────────────────────────────────────────────────────────────────────────
rclcpp_action::GoalResponse VX01LocomotionServer::handle_goal(
    const rclcpp_action::GoalUUID& /*uuid*/,
    std::shared_ptr<const Walk::Goal> goal)
{
    RCLCPP_INFO(get_logger(),
        "Received Walk goal  vx=%.3f m/s  vy=%.3f m/s  "
        "omega=%.3f rad/s  duration=%.1f s",
        goal->velocity_x, goal->velocity_y,
        goal->velocity_omega, goal->duration);

    // Reject only a truly empty goal (no motion AND no time limit)
    if (goal->duration < 1e-6 &&
        std::abs(goal->velocity_x)     < 1e-6 &&
        std::abs(goal->velocity_y)     < 1e-6 &&
        std::abs(goal->velocity_omega) < 1e-6)
    {
        RCLCPP_WARN(get_logger(), "Rejecting zero-velocity indefinite goal.");
        return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse VX01LocomotionServer::handle_cancel(
    const std::shared_ptr<WalkGoalHandle> /*goal_handle*/)
{
    RCLCPP_INFO(get_logger(), "Cancel requested - stopping gait.");
    cancel_requested_.store(true);
    return rclcpp_action::CancelResponse::ACCEPT;
}

void VX01LocomotionServer::handle_accepted(
    const std::shared_ptr<WalkGoalHandle> goal_handle)
{
    std::thread([this, goal_handle]() {
        execute_walk(goal_handle);
    }).detach();
}

// ─────────────────────────────────────────────────────────────────────────────
//  execute_walk
//
//  Changes from previous version:
//  1. duration=0 (or default 10s from send_walk_goal.py) → now walks until
//     Ctrl-C kills the NODE (not the client).  The client is changed to send
//     duration=0 by default, which means "walk indefinitely".
//  2. The while-loop checks rclcpp::ok() so it exits cleanly on SIGINT.
//  3. No more "succeeded after N seconds" unless timed_goal is true.
// ─────────────────────────────────────────────────────────────────────────────
void VX01LocomotionServer::execute_walk(
    const std::shared_ptr<WalkGoalHandle> goal_handle)
{
    const auto goal = goal_handle->get_goal();

    const double step_length =
        (goal->step_length > 1e-6) ? goal->step_length : p_step_length_;
    const double step_height =
        (goal->step_height > 1e-6) ? goal->step_height : p_step_height_;
    const double step_period =
        (goal->step_period > 1e-6) ? goal->step_period : p_step_period_;

    hexapod_->setStepLength(step_length);
    hexapod_->setStepHeight(step_height);
    hexapod_->setStepPeriod(step_period);

    hexapod_->setVelocity(
        goal->velocity_x     * 1000.0,
        goal->velocity_y     * 1000.0,
        goal->velocity_omega);

    RCLCPP_INFO(get_logger(),
        "Walking  T=%.1f mm  A=%.2f mm  period=%.2f s  "
        "vx=%.1f mm/s  vy=%.1f mm/s  omega=%.3f rad/s",
        step_length, step_height, step_period,
        goal->velocity_x * 1000.0,
        goal->velocity_y * 1000.0,
        goal->velocity_omega);

    cancel_requested_.store(false);

    // ── Phase 1: Stand up smoothly ────────────────────────────────────────
    hexapod_->stand();
    standup_sequence();
    std::this_thread::sleep_for(400ms);

    // ── Phase 2: Walk ─────────────────────────────────────────────────────
    hexapod_->walk();

    const double dt         = 1.0 / p_update_rate_hz_;
    const bool   timed_goal = (goal->duration > 1e-6);
    double       elapsed    = 0.0;

    rclcpp::Rate rate(p_update_rate_hz_);
    auto feedback = std::make_shared<Walk::Feedback>();

    // Walk until: cancel requested, time limit reached (if set), or node killed
    while (rclcpp::ok()) {

        if (cancel_requested_.load() || goal_handle->is_canceling()) {
            RCLCPP_INFO(get_logger(),
                "Walk cancelled after %.2f s.", elapsed);
            hexapod_->stop();
            move_to_stand();
            auto result          = std::make_shared<Walk::Result>();
            result->success      = false;
            result->message      = "Cancelled by client";
            result->elapsed_time = elapsed;
            goal_handle->canceled(result);
            return;
        }

        if (timed_goal && elapsed >= goal->duration) break;

        hexapod_->update(dt);
        send_all_legs(p_traj_dt_);

        feedback->gait_block   = static_cast<int32_t>(
            std::fmod(elapsed / (step_period / 6.0), 6.0));
        feedback->elapsed_time = elapsed;
        feedback->joint_angles = hexapod_->getJointAngles();
        goal_handle->publish_feedback(feedback);

        elapsed += dt;
        rate.sleep();
    }

    hexapod_->stand();
    move_to_stand();

    auto result          = std::make_shared<Walk::Result>();
    result->success      = true;
    result->message      = "Walk completed";
    result->elapsed_time = elapsed;
    goal_handle->succeed(result);

    RCLCPP_INFO(get_logger(), "Walk goal succeeded (%.2f s walked).", elapsed);
}

// ─────────────────────────────────────────────────────────────────────────────
//  standup_sequence
//
//  Linearly interpolates from joint zeros to the home position over
//  p_standup_steps_ steps, one trajectory waypoint per step.
//  All 6 legs move simultaneously.
// ─────────────────────────────────────────────────────────────────────────────
void VX01LocomotionServer::standup_sequence()
{
    const auto home_angles = hexapod_->getJointAngles();
    if (static_cast<int>(home_angles.size()) < TOTAL_JOINTS) {
        RCLCPP_ERROR(get_logger(),
            "standup_sequence: only %zu angles returned (need %d)",
            home_angles.size(), TOTAL_JOINTS);
        return;
    }

    const int  N        = p_standup_steps_;
    const long sleep_ms = static_cast<long>(p_stand_traj_dt_ * 1000.0);

    RCLCPP_INFO(get_logger(),
        "Stand-up: %d steps x %ld ms (total %.1f s) ...",
        N, sleep_ms, N * p_stand_traj_dt_);

    for (int s = 1; s <= N; ++s) {
        const double alpha = static_cast<double>(s) / static_cast<double>(N);

        for (int leg = 0; leg < NUM_LEGS; ++leg) {
            std::array<double, JOINTS_PER_LEG> pt;
            for (int j = 0; j < JOINTS_PER_LEG; ++j)
                pt[j] = alpha * home_angles[leg * JOINTS_PER_LEG + j];
            publish_leg_trajectory(leg, pt, p_stand_traj_dt_);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
    }

    RCLCPP_INFO(get_logger(), "Stand-up complete.");
}

// ─────────────────────────────────────────────────────────────────────────────
//  move_to_stand
// ─────────────────────────────────────────────────────────────────────────────
void VX01LocomotionServer::move_to_stand()
{
    hexapod_->stand();
    send_all_legs(p_stand_traj_dt_);
}

// ─────────────────────────────────────────────────────────────────────────────
//  send_all_legs
// ─────────────────────────────────────────────────────────────────────────────
void VX01LocomotionServer::send_all_legs(double traj_dt)
{
    const auto dh_angles = hexapod_->getJointAngles();

    if (static_cast<int>(dh_angles.size()) < TOTAL_JOINTS) {
        RCLCPP_ERROR_ONCE(get_logger(),
            "getJointAngles() returned %zu values (expected %d)",
            dh_angles.size(), TOTAL_JOINTS);
        return;
    }

    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        std::array<double, JOINTS_PER_LEG> la;
        for (int j = 0; j < JOINTS_PER_LEG; ++j)
            la[j] = dh_angles[leg * JOINTS_PER_LEG + j];
        publish_leg_trajectory(leg, la, traj_dt);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  publish_leg_trajectory
//
//  DH → URDF conversion (derived from URDF joint zero positions):
//
//   coxa_urdf  = -dh_theta1
//     The URDF coxa joint rotates about -Z (downward), DH rotates about +Z.
//     Sign flip required.
//
//   femur_urdf = dh_theta2 + femur_urdf_offset  (-0.7273 rad)
//     The URDF femur joint zero angle is NOT horizontal; it has a built-in
//     offset from the DH convention.
//     Derivation:  IK at home (223.03, 0, -72.82) → DH theta2 = -0.0577 rad
//     URDF wants femur = -0.785 rad at home stand pose.
//     offset = -0.785 - (-0.0577) = -0.7273 rad
//
//   tibia_urdf = -dh_theta3
//     The URDF tibia joint rotates in the opposite direction to the DH frame.
//     Sign flip required.
//
//  Verification at home position:
//    DH: (t1=0, t2=-0.0577, t3=-0.600)
//    URDF: coxa=0.000  femur=-0.785  tibia=+0.600  ← exactly what you asked for
// ─────────────────────────────────────────────────────────────────────────────
void VX01LocomotionServer::publish_leg_trajectory(
    int  leg_index,
    const std::array<double, JOINTS_PER_LEG>& dh_angles,
    double traj_dt)
{
    JointTraj msg;
    msg.header.stamp = now();

    for (int j = 0; j < JOINTS_PER_LEG; ++j)
        msg.joint_names.push_back(JOINT_NAMES[leg_index][j]);

    trajectory_msgs::msg::JointTrajectoryPoint pt;

    // coxa  → negate
    pt.positions.push_back(-dh_angles[0]);

    // femur → add URDF zero offset
    pt.positions.push_back(dh_angles[1] + p_femur_urdf_offset_);

    // tibia → negate
    pt.positions.push_back(-dh_angles[2]);

    pt.velocities.assign(JOINTS_PER_LEG, 0.0);
    pt.time_from_start = rclcpp::Duration::from_seconds(traj_dt);
    msg.points.push_back(pt);

    traj_pubs_[leg_index]->publish(msg);
}

}  // namespace vx01_locomotion_control