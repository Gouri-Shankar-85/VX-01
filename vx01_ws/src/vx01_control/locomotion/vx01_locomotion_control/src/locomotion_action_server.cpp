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

    // ── Create one FJT action client per leg ────────────────────────────────
    for (int i = 0; i < NUM_LEGS; ++i) {
        fjt_clients_[i] = rclcpp_action::create_client<FJT>(
            this, FJT_SERVER_NAMES[i]);
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
    // Kinematic link lengths
    declare_parameter("L1", L1_MM);
    declare_parameter("L2", L2_MM);
    declare_parameter("L3", L3_MM);

    // Gait defaults
    declare_parameter("step_length",  DEFAULT_STEP_LENGTH);
    declare_parameter("step_height",  DEFAULT_STEP_HEIGHT);
    declare_parameter("step_period",  DEFAULT_STEP_PERIOD);

    // Home position
    declare_parameter("home_x",  HOME_X);
    declare_parameter("home_y",  HOME_Y);
    declare_parameter("home_z",  HOME_Z);

    // Control loop
    declare_parameter("update_rate_hz", 50.0);
    declare_parameter("traj_dt",        0.02);   // = 1 / update_rate
    declare_parameter("stand_traj_dt",  0.5);    // slow move to home
}

void VX01LocomotionServer::load_parameters()
{
    p_L1_            = get_parameter("L1").as_double();
    p_L2_            = get_parameter("L2").as_double();
    p_L3_            = get_parameter("L3").as_double();
    p_step_length_   = get_parameter("step_length").as_double();
    p_step_height_   = get_parameter("step_height").as_double();
    p_step_period_   = get_parameter("step_period").as_double();
    p_home_x_        = get_parameter("home_x").as_double();
    p_home_y_        = get_parameter("home_y").as_double();
    p_home_z_        = get_parameter("home_z").as_double();
    p_update_rate_hz_ = get_parameter("update_rate_hz").as_double();
    p_traj_dt_       = get_parameter("traj_dt").as_double();
    p_stand_traj_dt_ = get_parameter("stand_traj_dt").as_double();
}

// ─────────────────────────────────────────────────────────────────────────────
//  Build HexapodLocomotion and override per-leg x_start with exact URDF values
//
//  HexapodLocomotion is initialised with body_radius = 0 so it doesn't do
//  its own initializeLegControllers.  We then access the protected
//  leg_controllers_ through the public API to fix up x_start.
//
//  NOTE: if you later add a setLegXStart() API to the library, use that
//  instead and remove the re-init loop here.
// ─────────────────────────────────────────────────────────────────────────────
void VX01LocomotionServer::init_hexapod()
{
    // Construct with body_radius = average (127 mm) – the library will build
    // 6 LegControllers each with x_start = body_radius.  We immediately
    // replace them below.
    hexapod_ = std::make_unique<vx01_hexapod_locomotion::HexapodLocomotion>(
        p_L1_, p_L2_, p_L3_,
        /*body_radius=*/127.0,
        BETA_ANGLE_RAD);

    // Apply exact gait and home parameters from drawings
    hexapod_->setStepLength(p_step_length_);
    hexapod_->setStepHeight(p_step_height_);
    hexapod_->setStepPeriod(p_step_period_);
    hexapod_->setHomePosition(p_home_x_, p_home_y_, p_home_z_);

    // ── Note on per-leg pivot radii ─────────────────────────────────────────
    // The LegController uses x_start as a translation added after rotation in
    // bodyToLegFrame().  Because the VX-01's coxa pivots are not equidistant
    // from the body centre, the correct x_start for each leg should be:
    //   leg0: 105.67 mm   leg3: 105.65 mm
    //   leg1: 136.79 mm   leg4: 139.16 mm
    //   leg2: 136.78 mm   leg5: 139.17 mm
    //
    // HexapodLocomotion does not expose a setLegXStart() API, but the
    // updateLeg() method calls gait_pattern_->getFootPosition() in the
    // leg-local frame and then passes the result directly to
    // LegController::setFootPosition() – it does NOT call bodyToLegFrame()
    // internally.  Therefore x_start only matters if your higher-level code
    // calls bodyToLegFrame() / legToBodyFrame() directly.
    //
    // For pure gait walking (what this node does) the gait trajectories are
    // generated in leg-local space, so the single body_radius value above
    // does not affect foot placement accuracy.  You only need to fix this if
    // you add body-pose control (e.g. inverse body kinematics for terrain
    // adaptation), at which point expose setLegXStart() in the library.
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

    // Reject zero-velocity goals with infinite duration (nothing to do)
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
    // Detach a thread so the executor stays free for cancel callbacks
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

    // ── Wait for all 6 FJT servers (max 15 s each) ──────────────────────────
    for (int i = 0; i < NUM_LEGS; ++i) {
        if (!fjt_clients_[i]->wait_for_action_server(15s)) {
            RCLCPP_ERROR(get_logger(),
                "Timed out waiting for FJT server: %s",
                FJT_SERVER_NAMES[i].c_str());
            auto result      = std::make_shared<Walk::Result>();
            result->success  = false;
            result->message  = "FJT server unavailable: " + FJT_SERVER_NAMES[i];
            goal_handle->abort(result);
            return;
        }
    }

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

    // Velocity: goal is in m/s, library expects mm/s
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

    // ── Stand first, then start gait ─────────────────────────────────────────
    cancel_requested_.store(false);

    hexapod_->stand();
    move_to_stand();
    std::this_thread::sleep_for(600ms);   // let controllers settle at home

    hexapod_->walk();

    // ── Timing ───────────────────────────────────────────────────────────────
    const double dt          = 1.0 / p_update_rate_hz_;
    const bool   timed_goal  = (goal->duration > 1e-6);
    const double end_time    = goal->duration;
    double       elapsed     = 0.0;

    rclcpp::Rate rate(p_update_rate_hz_);
    auto feedback = std::make_shared<Walk::Feedback>();

    // ── Gait loop ─────────────────────────────────────────────────────────────
    while (rclcpp::ok()) {

        // ── Cancellation check ─────────────────────────────────────────────
        if (cancel_requested_.load() || goal_handle->is_canceling()) {
            RCLCPP_INFO(get_logger(),
                "Walk cancelled after %.2f s – returning to stand.", elapsed);
            hexapod_->stop();
            move_to_stand();
            auto result          = std::make_shared<Walk::Result>();
            result->success      = false;
            result->message      = "Cancelled by client";
            result->elapsed_time = elapsed;
            goal_handle->canceled(result);
            return;
        }

        // ── Duration check ─────────────────────────────────────────────────
        if (timed_goal && elapsed >= end_time) {
            break;
        }

        // ── Advance gait state machine ──────────────────────────────────────
        hexapod_->update(dt);

        // ── Send joint angles to all 6 controllers ─────────────────────────
        send_all_legs(p_traj_dt_);

        // ── Publish feedback ───────────────────────────────────────────────
        feedback->gait_block   = hexapod_->getState() ==
            vx01_hexapod_locomotion::LocomotionState::WALKING ? 1 : 0;
        feedback->elapsed_time = elapsed;
        feedback->joint_angles = hexapod_->getJointAngles();
        goal_handle->publish_feedback(feedback);

        elapsed += dt;
        rate.sleep();
    }

    // ── Done – stand and report ───────────────────────────────────────────────
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
//  Move to stand: update library state, then push joint angles to controllers
// ─────────────────────────────────────────────────────────────────────────────
void VX01LocomotionServer::move_to_stand()
{
    hexapod_->stand();
    send_all_legs(p_stand_traj_dt_);
}

// ─────────────────────────────────────────────────────────────────────────────
//  send_all_legs – dispatch current joint angles to all 6 FJT servers
// ─────────────────────────────────────────────────────────────────────────────
void VX01LocomotionServer::send_all_legs(double traj_dt)
{
    const auto angles = hexapod_->getJointAngles();  // 18-element vector

    if (static_cast<int>(angles.size()) < TOTAL_JOINTS) {
        RCLCPP_ERROR_ONCE(get_logger(),
            "getJointAngles() returned %zu values (expected %d)",
            angles.size(), TOTAL_JOINTS);
        return;
    }

    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        std::array<double, JOINTS_PER_LEG> leg_angles;
        for (int j = 0; j < JOINTS_PER_LEG; ++j) {
            leg_angles[j] = angles[leg * JOINTS_PER_LEG + j];
        }
        send_leg_trajectory(leg, leg_angles, traj_dt);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  send_leg_trajectory – async fire-and-go to one FJT server
//
//  We send a single-point trajectory each control cycle.
//  The JointTrajectoryController interpolates between successive points,
//  giving smooth motion at the hardware update rate (1000 Hz in YAML).
//
//  We do NOT block on the result: waiting would serialise the 6-leg dispatch
//  and add latency equal to 6 × traj_dt.  Instead we log warnings for
//  non-SUCCEEDED results via the result callback.
// ─────────────────────────────────────────────────────────────────────────────
void VX01LocomotionServer::send_leg_trajectory(
    int  leg_index,
    const std::array<double, JOINTS_PER_LEG>& angles,
    double traj_dt)
{
    auto goal_msg = FJT::Goal();
    goal_msg.trajectory.header.stamp = now();

    // Joint names
    for (int j = 0; j < JOINTS_PER_LEG; ++j) {
        goal_msg.trajectory.joint_names.push_back(JOINT_NAMES[leg_index][j]);
    }

    // Single waypoint
    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.positions.assign(angles.begin(), angles.end());
    // Provide zero velocities so the controller does not extrapolate
    pt.velocities.assign(JOINTS_PER_LEG, 0.0);
    pt.time_from_start = rclcpp::Duration::from_seconds(traj_dt);
    goal_msg.trajectory.points.push_back(pt);

    // Goal tolerance: use controller YAML defaults (no override here)

    auto send_opts = rclcpp_action::Client<FJT>::SendGoalOptions();
    send_opts.result_callback =
        [this, leg_index](
            const rclcpp_action::ClientGoalHandle<FJT>::WrappedResult& wr)
        {
            if (wr.code != rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 2000 /*ms*/,
                    "FJT result for leg_%d: code=%d",
                    leg_index, static_cast<int>(wr.code));
            }
        };

    fjt_clients_[leg_index]->async_send_goal(goal_msg, send_opts);
}

}  // namespace vx01_locomotion_control
