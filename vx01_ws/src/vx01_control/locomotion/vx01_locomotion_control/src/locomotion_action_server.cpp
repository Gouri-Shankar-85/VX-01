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

    // ── Build the locomotion engine ─────────────────────────────────────────
    init_hexapod();

    // ── Create one JointTrajectory publisher per leg ─────────────────────────
    // Direct topic publishing avoids the FJT action preemption problem
    // (ResultCode=5) that occurs when a new goal is sent every 20 ms while
    // the previous 20 ms trajectory is still executing.
    for (int i = 0; i < NUM_LEGS; ++i) {
        const std::string topic =
            "leg_" + std::to_string(i) + "_controller/joint_trajectory";
        traj_pubs_[i] = create_publisher<JointTraj>(topic, 1);
    }

    // ── Advertise the Walk action server ────────────────────────────────────
    using std::placeholders::_1;
    using std::placeholders::_2;

    walk_server_ = rclcpp_action::create_server<Walk>(
        this,
        "vx01/walk",
        std::bind(&VX01LocomotionServer::handle_goal,     this, _1, _2),
        std::bind(&VX01LocomotionServer::handle_cancel,   this, _1),
        std::bind(&VX01LocomotionServer::handle_accepted, this, _1));

    RCLCPP_INFO(get_logger(),
        "VX01LocomotionServer ready  –  action: vx01/walk");
    RCLCPP_INFO(get_logger(),
        "  L1=%.2f  L2=%.2f  L3=%.2f mm",
        p_L1_, p_L2_, p_L3_);
    RCLCPP_INFO(get_logger(),
        "  home=(%.2f, %.2f, %.2f) mm  "
        "T=%.1f mm  A=%.2f mm  period=%.2f s",
        p_home_x_, p_home_y_, p_home_z_,
        p_step_length_, p_step_height_, p_step_period_);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Parameter helpers
// ─────────────────────────────────────────────────────────────────────────────
void VX01LocomotionServer::declare_parameters()
{
    declare_parameter("L1", L1_MM);
    declare_parameter("L2", L2_MM);
    declare_parameter("L3", L3_MM);
    declare_parameter("step_length",  DEFAULT_STEP_LENGTH);
    declare_parameter("step_height",  DEFAULT_STEP_HEIGHT);
    declare_parameter("step_period",  DEFAULT_STEP_PERIOD);
    declare_parameter("home_x",  HOME_X);
    declare_parameter("home_y",  HOME_Y);
    declare_parameter("home_z",  HOME_Z);
    declare_parameter("update_rate_hz", 50.0);
    declare_parameter("traj_dt",        0.02);
    declare_parameter("stand_traj_dt",  0.5);
    declare_parameter("standup_steps",  40);    // steps in stand-up interpolation
}

void VX01LocomotionServer::load_parameters()
{
    p_L1_             = get_parameter("L1").as_double();
    p_L2_             = get_parameter("L2").as_double();
    p_L3_             = get_parameter("L3").as_double();
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
// ─────────────────────────────────────────────────────────────────────────────
void VX01LocomotionServer::init_hexapod()
{
    hexapod_ = std::make_unique<vx01_hexapod_locomotion::HexapodLocomotion>(
        p_L1_, p_L2_, p_L3_,
        /*body_radius=*/127.0,
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
    RCLCPP_INFO(get_logger(), "Cancel requested – stopping gait.");
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
// ─────────────────────────────────────────────────────────────────────────────
void VX01LocomotionServer::execute_walk(
    const std::shared_ptr<WalkGoalHandle> goal_handle)
{
    const auto goal = goal_handle->get_goal();

    // ── Apply per-goal gait overrides ────────────────────────────────────────
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
        goal->velocity_x     * 1000.0,   // m/s → mm/s
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

    hexapod_->stand();
    standup_sequence();
    std::this_thread::sleep_for(400ms);   // let controllers settle

    // ── Phase 2: Start gait ───────────────────────────────────────────────────
    hexapod_->walk();

    const double dt         = 1.0 / p_update_rate_hz_;
    const bool   timed_goal = (goal->duration > 1e-6);
    double       elapsed    = 0.0;

    rclcpp::Rate rate(p_update_rate_hz_);
    auto feedback = std::make_shared<Walk::Feedback>();

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

    // ── Done ─────────────────────────────────────────────────────────────────
    hexapod_->stand();
    move_to_stand();

    auto result          = std::make_shared<Walk::Result>();
    result->success      = true;
    result->message      = "Walk completed successfully";
    result->elapsed_time = elapsed;
    goal_handle->succeed(result);

    RCLCPP_INFO(get_logger(), "Walk goal succeeded (%.2f s walked).", elapsed);
}

void VX01LocomotionServer::standup_sequence()
{
    // Get the home joint angles (library has already been told stand())
    const auto home_angles = hexapod_->getJointAngles();
    if (static_cast<int>(home_angles.size()) < TOTAL_JOINTS) return;

    const int    N         = p_standup_steps_;
    const double step_dt   = p_stand_traj_dt_;   // time per interpolated step (s)
    const double sleep_ms  = step_dt * 1000.0;

    RCLCPP_INFO(get_logger(),
        "Stand-up: interpolating to home over %d steps (%.1f ms each) ...", N, sleep_ms);

    for (int s = 1; s <= N; ++s) {
        const double alpha = static_cast<double>(s) / N;   // 0 → 1

        for (int leg = 0; leg < NUM_LEGS; ++leg) {
            std::array<double, JOINTS_PER_LEG> pt_angles;
            for (int j = 0; j < JOINTS_PER_LEG; ++j) {
                const double home_j = home_angles[leg * JOINTS_PER_LEG + j];
                // Start from 0 (initial joint position) and lerp to home
                pt_angles[j] = alpha * home_j;
            }
            publish_leg_trajectory(leg, pt_angles, step_dt);
        }
        std::this_thread::sleep_for(
            std::chrono::milliseconds(static_cast<int>(sleep_ms)));
    }

    RCLCPP_INFO(get_logger(), "Stand-up complete.");
}

// ─────────────────────────────────────────────────────────────────────────────
//  move_to_stand – single-shot stand command at slow trajectory speed
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
    for (int j = 0; j < JOINTS_PER_LEG; ++j)
        pt.positions.push_back(dh_angles[j]);   // DH = URDF directly (elbow-down)
    pt.velocities.assign(JOINTS_PER_LEG, 0.0);
    pt.time_from_start = rclcpp::Duration::from_seconds(traj_dt);
    msg.points.push_back(pt);

    traj_pubs_[leg_index]->publish(msg);
}

}  // namespace vx01_locomotion_control