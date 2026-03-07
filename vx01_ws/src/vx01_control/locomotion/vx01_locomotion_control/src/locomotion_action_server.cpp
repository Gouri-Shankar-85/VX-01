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

    // One JointTrajectory publisher per leg
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
}

// ─────────────────────────────────────────────────────────────────────────────
//  Parameter helpers
// ─────────────────────────────────────────────────────────────────────────────
void VX01LocomotionServer::declare_parameters()
{
    declare_parameter("L1",              L1_MM);
    declare_parameter("L2",              L2_MM);
    declare_parameter("L3",              L3_MM);
    declare_parameter("body_radius",     BODY_RADIUS_MM);
    declare_parameter("step_length",     DEFAULT_STEP_LENGTH);
    declare_parameter("step_height",     DEFAULT_STEP_HEIGHT);
    declare_parameter("step_period",     DEFAULT_STEP_PERIOD);
    declare_parameter("home_x",          HOME_X);
    declare_parameter("home_y",          HOME_Y);
    declare_parameter("home_z",          HOME_Z);
    declare_parameter("update_rate_hz",  50.0);
    declare_parameter("traj_dt",         0.02);
    declare_parameter("stand_traj_dt",   0.08);
    declare_parameter("standup_steps",   40);
}

void VX01LocomotionServer::load_parameters()
{
    p_L1_             = get_parameter("L1").as_double();
    p_L2_             = get_parameter("L2").as_double();
    p_L3_             = get_parameter("L3").as_double();
    p_body_radius_    = get_parameter("body_radius").as_double();
    p_step_length_    = get_parameter("step_length").as_double();
    p_step_height_    = get_parameter("step_height").as_double();
    p_step_period_    = get_parameter("step_period").as_double();
    p_home_x_         = get_parameter("home_x").as_double();
    p_home_y_         = get_parameter("home_y").as_double();
    p_home_z_         = get_parameter("home_z").as_double();
    p_update_rate_hz_ = get_parameter("update_rate_hz").as_double();
    p_traj_dt_        = get_parameter("traj_dt").as_double();
    p_stand_traj_dt_  = get_parameter("stand_traj_dt").as_double();
    p_standup_steps_  = get_parameter("standup_steps").as_int();
}

// ─────────────────────────────────────────────────────────────────────────────
//  Build HexapodLocomotion
//
//  FIX #1: body_radius is now a parameter (was hardcoded 127.0).
//  FIX #3: leg angles use beta=62.91 deg layout (fixed inside the library).
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
//  Walk action-server callbacks
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

    // Reject a goal that has zero velocity AND zero duration
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
//  Main walking execution loop
//
//  FIX #4 (Leg 5 / gait): The tripod gait now uses 2 swing blocks per group
//  (fixed in gait_pattern.cpp).  The velocity scaling here feeds the correct
//  signed Y-offset into IK so all 6 legs step properly.
//
//  FIX: Walk runs INDEFINITELY (duration == 0) until cancelled.
//  The while-loop condition checks cancel AND timed_goal correctly.
// ─────────────────────────────────────────────────────────────────────────────
void VX01LocomotionServer::execute_walk(
    const std::shared_ptr<WalkGoalHandle> goal_handle)
{
    const auto goal = goal_handle->get_goal();

    // ── Per-goal gait overrides (0 = use param default) ──────────────────
    const double step_length =
        (goal->step_length > 1e-6) ? goal->step_length : p_step_length_;
    const double step_height =
        (goal->step_height > 1e-6) ? goal->step_height : p_step_height_;
    const double step_period =
        (goal->step_period > 1e-6) ? goal->step_period : p_step_period_;

    hexapod_->setStepLength(step_length);
    hexapod_->setStepHeight(step_height);
    hexapod_->setStepPeriod(step_period);

    // Convert m/s → mm/s for the locomotion library
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

    // ── Phase 1: Stand up and reach home position ─────────────────────────
    hexapod_->stand();
    standup_sequence();
    std::this_thread::sleep_for(400ms);   // let controllers settle

    // ── Phase 2: Walk ─────────────────────────────────────────────────────
    hexapod_->walk();

    const double dt         = 1.0 / p_update_rate_hz_;
    const bool   timed_goal = (goal->duration > 1e-6);
    double       elapsed    = 0.0;

    rclcpp::Rate rate(p_update_rate_hz_);
    auto feedback = std::make_shared<Walk::Feedback>();

    while (rclcpp::ok()) {

        // ── Check cancel ──────────────────────────────────────────────────
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

        // ── Check time limit ──────────────────────────────────────────────
        if (timed_goal && elapsed >= goal->duration) break;

        // ── Step the gait engine and push joint angles to controllers ─────
        hexapod_->update(dt);
        send_all_legs(p_traj_dt_);

        // ── Publish feedback ──────────────────────────────────────────────
        feedback->gait_block   = static_cast<int32_t>(
            std::fmod(elapsed / (step_period / 6.0), 6.0));
        feedback->elapsed_time = elapsed;
        feedback->joint_angles = hexapod_->getJointAngles();
        goal_handle->publish_feedback(feedback);

        elapsed += dt;
        rate.sleep();
    }

    // ── Walk complete – return to stand ──────────────────────────────────
    hexapod_->stand();
    move_to_stand();

    auto result          = std::make_shared<Walk::Result>();
    result->success      = true;
    result->message      = "Walk completed successfully";
    result->elapsed_time = elapsed;
    goal_handle->succeed(result);

    RCLCPP_INFO(get_logger(), "Walk goal succeeded (%.2f s walked).", elapsed);
}

// ─────────────────────────────────────────────────────────────────────────────
//  standup_sequence
//
//  FIX: Interpolate from CURRENT joint state (zeros at startup) to the home
//  position computed by the library.  Each leg is sent independently with the
//  same slow traj_dt so no leg is left behind.
//
//  FIX for Leg 5: All 6 legs are iterated 0..5 in the same step — previously
//  the loop was correct but the gait block count bug caused leg 5 to be in the
//  wrong phase.  With gait_pattern.cpp fixed (2 SWING blocks) leg 5 now
//  receives the correct Bezier arc during swing.
// ─────────────────────────────────────────────────────────────────────────────
void VX01LocomotionServer::standup_sequence()
{
    const auto home_angles = hexapod_->getJointAngles();
    if (static_cast<int>(home_angles.size()) < TOTAL_JOINTS) {
        RCLCPP_ERROR(get_logger(),
            "standup_sequence: getJointAngles() returned %zu values, expected %d",
            home_angles.size(), TOTAL_JOINTS);
        return;
    }

    const int    N        = p_standup_steps_;
    const double step_dt  = p_stand_traj_dt_;
    const long   sleep_ms = static_cast<long>(step_dt * 1000.0);

    RCLCPP_INFO(get_logger(),
        "Stand-up: %d steps x %.1f ms each ...", N, step_dt * 1000.0);

    for (int s = 1; s <= N; ++s) {
        const double alpha = static_cast<double>(s) / static_cast<double>(N);

        for (int leg = 0; leg < NUM_LEGS; ++leg) {
            std::array<double, JOINTS_PER_LEG> pt_angles;
            for (int j = 0; j < JOINTS_PER_LEG; ++j) {
                // Linear interpolation from 0 (startup) to home angle
                pt_angles[j] = alpha * home_angles[leg * JOINTS_PER_LEG + j];
            }
            publish_leg_trajectory(leg, pt_angles, step_dt);
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
        std::array<double, JOINTS_PER_LEG> leg_angles;
        for (int j = 0; j < JOINTS_PER_LEG; ++j)
            leg_angles[j] = dh_angles[leg * JOINTS_PER_LEG + j];
        publish_leg_trajectory(leg, leg_angles, traj_dt);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  publish_leg_trajectory
//
//  FIX: theta1 (coxa DH angle) must be NEGATED before sending to the
//  controller because the URDF coxa joint rotates in the OPPOSITE direction
//  to the DH convention used in our kinematics.
//  theta2 and theta3 (femur/tibia) map directly (same sign).
//
//  Why only coxa?
//  The DH frame for joint 1 has its z-axis pointing UP (+z_0 direction).
//  In the URDF the coxa servo axis points DOWN (-z_body direction).
//  This gives a sign inversion on theta1 only.
//  Femur and tibia DH z-axes already match the URDF servo axes.
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

    // joint 0 = coxa  → negate DH theta1 to match URDF servo direction
    pt.positions.push_back(-dh_angles[0]);

    // joints 1,2 = femur / tibia → direct DH value
    pt.positions.push_back(dh_angles[1]);
    pt.positions.push_back(dh_angles[2]);

    pt.velocities.assign(JOINTS_PER_LEG, 0.0);
    pt.time_from_start = rclcpp::Duration::from_seconds(traj_dt);
    msg.points.push_back(pt);

    traj_pubs_[leg_index]->publish(msg);
}

}  // namespace vx01_locomotion_control