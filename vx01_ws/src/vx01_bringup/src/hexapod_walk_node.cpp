/**
 * hexapod_walk_node.cpp
 *
 * ROS 2 Humble node for VX-01 hexapod locomotion.
 *
 * Architecture
 * ─────────────
 *  • Uses the vx01_hexapod_locomotion library (IK + gait pattern).
 *  • Sends joint-position goals via six FollowJointTrajectory ACTION CLIENTS
 *    (one per leg controller) — the method confirmed to work on this robot.
 *  • Subscribes to /cmd_vel (geometry_msgs/Twist) for velocity commands
 *    from the teleop keyboard node.
 *
 * Start-up sequence
 * ──────────────────
 *  1. INIT  : Wait for all six action servers to become available.
 *  2. STAND : Send all legs to the home (standing) position and wait for
 *             completion before starting the gait loop.
 *  3. WALK  : Run a 50 Hz timer that drives the gait pattern, computes IK,
 *             and sends one FollowJointTrajectory goal per leg per block
 *             (every block_period seconds).
 *
 * Joint naming (from xacro files)
 * ────────────────────────────────
 *  leg_N_controller controls: [coxa_legN_joint, femur_legN_joint, tibia_legN_joint]
 *  N = 0 … 5
 *
 * Angle convention
 * ─────────────────
 *  IK angles (theta1, theta2, theta3) are sent DIRECTLY to the JointTrajectoryController.
 *  The URDF axis="0 0 -1" on femur and tibia is already accounted for inside the
 *  DH / IK model — the simulation accepts these values and produces correct motion.
 *
 * Robot dimensions (from engineering drawings)
 * ──────────────────────────────────────────────
 *  L1 = 60.55 mm  (coxa)
 *  L2 = 73.84 mm  (femur)
 *  L3 = 112.16 mm (tibia)
 *  body_radius = 100 mm
 *  beta_angle  = 62.91 deg = 1.0977 rad
 *
 * Gait parameters (from bezier_curve engineering drawing)
 * ─────────────────────────────────────────────────────────
 *  S (reach depth / track_width) = 108.67 mm
 *  T (stride length / step_length) = 110.00 mm
 *  A (step height) = 22.78 mm
 *  block_period = step_period / 6  (default step_period = 3.0 s)
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <array>
#include <vector>
#include <string>
#include <memory>
#include <functional>
#include <atomic>
#include <mutex>
#include <cmath>

// ── Locomotion library ────────────────────────────────────────────────────
#include "vx01_hexapod_locomotion/hexapod_locomotion.hpp"

using FollowJT      = control_msgs::action::FollowJointTrajectory;
using GoalHandleFJT = rclcpp_action::ClientGoalHandle<FollowJT>;

// ═════════════════════════════════════════════════════════════════════════
class HexapodWalkNode : public rclcpp::Node
{
public:
    // ── Constants ────────────────────────────────────────────────────────
    static constexpr int    NUM_LEGS     = 6;
    static constexpr double L1           = 60.55;    // mm coxa
    static constexpr double L2           = 73.84;    // mm femur
    static constexpr double L3           = 112.16;   // mm tibia
    static constexpr double BODY_RADIUS  = 100.0;    // mm
    static constexpr double BETA_DEG     = 62.91;    // degrees
    static constexpr double STEP_PERIOD  = 3.0;      // s  (full 6-block cycle)
    static constexpr double UPDATE_HZ    = 50.0;     // control loop rate
    static constexpr double HOME_X       = 170.0;    // mm  leg-local x
    static constexpr double HOME_Y       = 0.0;      // mm  leg-local y
    static constexpr double HOME_Z       = -90.0;    // mm  leg-local z (standing height)

    // Joint names per leg (must match YAML and URDF)
    const std::array<std::array<std::string,3>, NUM_LEGS> JOINT_NAMES = {{
        {"coxa_leg0_joint","femur_leg0_joint","tibia_leg0_joint"},
        {"coxa_leg1_joint","femur_leg1_joint","tibia_leg1_joint"},
        {"coxa_leg2_joint","femur_leg2_joint","tibia_leg2_joint"},
        {"coxa_leg3_joint","femur_leg3_joint","tibia_leg3_joint"},
        {"coxa_leg4_joint","femur_leg4_joint","tibia_leg4_joint"},
        {"coxa_leg5_joint","femur_leg5_joint","tibia_leg5_joint"}
    }};

    // Action server names (must match YAML controller names)
    const std::array<std::string, NUM_LEGS> ACTION_NAMES = {
        "/leg_0_controller/follow_joint_trajectory",
        "/leg_1_controller/follow_joint_trajectory",
        "/leg_2_controller/follow_joint_trajectory",
        "/leg_3_controller/follow_joint_trajectory",
        "/leg_4_controller/follow_joint_trajectory",
        "/leg_5_controller/follow_joint_trajectory"
    };

    // ── Constructor ───────────────────────────────────────────────────────
    explicit HexapodWalkNode()
    : Node("hexapod_walk_node"),
      locomotion_(L1, L2, L3, BODY_RADIUS, BETA_DEG * M_PI / 180.0),
      state_(State::INIT),
      stand_goals_done_(0),
      gait_time_(0.0),
      current_block_(0),
      last_block_(-1)
    {
        // Set gait parameters
        locomotion_.setStepPeriod(STEP_PERIOD);
        locomotion_.setHomePosition(HOME_X, HOME_Y, HOME_Z);

        // Velocity command subscriber
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&HexapodWalkNode::cmdVelCallback, this, std::placeholders::_1));

        // Create one action client per leg
        for (int i = 0; i < NUM_LEGS; ++i) {
            action_clients_[i] = rclcpp_action::create_client<FollowJT>(
                this, ACTION_NAMES[i]);
            legs_ready_[i]  = false;
            legs_busy_[i]   = false;
        }

        // Main state-machine timer at UPDATE_HZ
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / UPDATE_HZ),
            std::bind(&HexapodWalkNode::timerCallback, this));

        RCLCPP_INFO(this->get_logger(),
            "HexapodWalkNode started. Waiting for action servers...");
    }

private:
    // ── State machine ─────────────────────────────────────────────────────
    enum class State { INIT, STANDING, WALKING };

    // ── Members ───────────────────────────────────────────────────────────
    vx01_hexapod_locomotion::HexapodLocomotion locomotion_;

    std::array<rclcpp_action::Client<FollowJT>::SharedPtr, NUM_LEGS> action_clients_;
    std::array<bool, NUM_LEGS> legs_ready_;
    std::array<std::atomic<bool>, NUM_LEGS> legs_busy_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    State state_;
    std::atomic<int>  stand_goals_done_;
    std::mutex        vel_mutex_;
    double            vel_x_{0.0}, vel_y_{0.0}, vel_omega_{0.0};

    // Gait timing (mirrors HexapodLocomotion internals, driven here for action timing)
    double gait_time_;
    int    current_block_;
    int    last_block_;

    // ── /cmd_vel callback ─────────────────────────────────────────────────
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(vel_mutex_);
        // Convert ROS Twist (m/s) → mm/s for the locomotion library
        vel_x_     = msg->linear.x  * 1000.0;
        vel_y_     = msg->linear.y  * 1000.0;
        vel_omega_ = msg->angular.z;

        // If we were standing and a non-zero velocity arrives, start walking
        if (state_ == State::STANDING &&
            (std::abs(vel_x_) > 1.0 || std::abs(vel_y_) > 1.0 || std::abs(vel_omega_) > 0.01))
        {
            RCLCPP_INFO(this->get_logger(),
                "cmd_vel received — starting walk  vx=%.1f vy=%.1f omega=%.3f",
                vel_x_, vel_y_, vel_omega_);

            locomotion_.setVelocity(vel_x_, vel_y_, vel_omega_);
            locomotion_.walk();
            gait_time_    = 0.0;
            current_block_ = 0;
            last_block_    = -1;
            state_ = State::WALKING;
        }
        else if (state_ == State::WALKING)
        {
            locomotion_.setVelocity(vel_x_, vel_y_, vel_omega_);

            // Stop walking if velocity drops to zero
            if (std::abs(vel_x_) < 1.0 && std::abs(vel_y_) < 1.0 &&
                std::abs(vel_omega_) < 0.01)
            {
                RCLCPP_INFO(this->get_logger(), "cmd_vel zero — returning to stand");
                locomotion_.stop();
                state_ = State::STANDING;
                sendStandGoal(); // Re-send stand to get all legs home
            }
        }
    }

    // ── Main timer callback ───────────────────────────────────────────────
    void timerCallback()
    {
        switch (state_)
        {
            case State::INIT:
                initStep();
                break;
            case State::STANDING:
                // Nothing — we're holding the stand pose sent earlier
                break;
            case State::WALKING:
                walkStep();
                break;
        }
    }

    // ── INIT: wait for all servers, then stand ────────────────────────────
    void initStep()
    {
        bool all_ready = true;
        for (int i = 0; i < NUM_LEGS; ++i) {
            if (!legs_ready_[i]) {
                if (action_clients_[i]->action_server_is_ready()) {
                    legs_ready_[i] = true;
                    RCLCPP_INFO(this->get_logger(),
                        "leg_%d action server ready", i);
                } else {
                    all_ready = false;
                }
            }
        }

        if (all_ready) {
            RCLCPP_INFO(this->get_logger(),
                "All action servers ready — moving to stand position...");
            state_ = State::STANDING;
            locomotion_.stand();
            sendStandGoal();
        }
    }

    // ── Send all legs to home position ────────────────────────────────────
    void sendStandGoal()
    {
        stand_goals_done_ = 0;

        // Duration for the stand move (nice slow movement to home)
        const double stand_duration = 2.0; // seconds

        for (int i = 0; i < NUM_LEGS; ++i)
        {
            double t1, t2, t3;
            locomotion_.getLegAngles(i, t1, t2, t3);

            sendLegGoal(i, {t1, t2, t3}, stand_duration,
                /* on_done = */
                [this, i](bool success) {
                    if (success) {
                        stand_goals_done_++;
                        RCLCPP_DEBUG(this->get_logger(),
                            "Stand goal done for leg %d (%d/6)",
                            i, (int)stand_goals_done_);
                        if (stand_goals_done_ == NUM_LEGS) {
                            RCLCPP_INFO(this->get_logger(),
                                "All legs at stand position. Ready to walk.");
                        }
                    } else {
                        RCLCPP_WARN(this->get_logger(),
                            "Stand goal FAILED for leg %d", i);
                    }
                });
        }
    }

    // ── WALK: advance gait clock, send goals each new block ───────────────
    void walkStep()
    {
        const double dt           = 1.0 / UPDATE_HZ;
        const double block_period = STEP_PERIOD / 6.0;

        // Advance gait time
        gait_time_ += dt;
        if (gait_time_ >= block_period) {
            gait_time_ -= block_period;
            current_block_ = (current_block_ + 1) % 6;
        }

        // Only send new goals when block changes
        if (current_block_ == last_block_) {
            return;
        }
        last_block_ = current_block_;

        // Advance the locomotion library to the new block
        // (we manage the clock externally so we can sync with action timing)
        // Update the library once per block
        {
            std::lock_guard<std::mutex> lock(vel_mutex_);
            locomotion_.setVelocity(vel_x_, vel_y_, vel_omega_);
        }

        // Run ONE full block update in the library to compute all 18 angles
        locomotion_.update(block_period);

        // Send a goal for each leg for this block
        for (int i = 0; i < NUM_LEGS; ++i)
        {
            if (legs_busy_[i]) {
                // Previous goal still running — cancel and resend
                // (For smooth motion we just let the new goal overwrite)
            }

            double t1, t2, t3;
            locomotion_.getLegAngles(i, t1, t2, t3);

            RCLCPP_DEBUG(this->get_logger(),
                "Leg %d block %d: t1=%.3f t2=%.3f t3=%.3f",
                i, current_block_, t1, t2, t3);

            sendLegGoal(i, {t1, t2, t3}, block_period, nullptr);
        }
    }

    // ── Core helper: send one FollowJointTrajectory goal to one leg ───────
    /**
     * @param leg_index   0-5
     * @param angles      {theta1, theta2, theta3} in radians (IK output)
     * @param duration    time in seconds to reach the target
     * @param on_done     optional callback(bool success) when goal finishes
     */
    void sendLegGoal(int leg_index,
                     const std::array<double,3>& angles,
                     double duration,
                     std::function<void(bool)> on_done)
    {
        auto goal_msg = FollowJT::Goal();

        // Joint names
        goal_msg.trajectory.joint_names = {
            JOINT_NAMES[leg_index][0],
            JOINT_NAMES[leg_index][1],
            JOINT_NAMES[leg_index][2]
        };

        // Single waypoint at time=duration
        trajectory_msgs::msg::JointTrajectoryPoint pt;
        pt.positions  = {angles[0], angles[1], angles[2]};
        pt.velocities = {0.0, 0.0, 0.0};
        pt.time_from_start = rclcpp::Duration::from_seconds(duration);
        goal_msg.trajectory.points.push_back(pt);

        // Send goal options
        auto send_goal_options = rclcpp_action::Client<FollowJT>::SendGoalOptions();

        send_goal_options.goal_response_callback =
            [this, leg_index](const GoalHandleFJT::SharedPtr& handle) {
                if (!handle) {
                    RCLCPP_WARN(this->get_logger(),
                        "Goal REJECTED by leg_%d server", leg_index);
                    legs_busy_[leg_index] = false;
                } else {
                    legs_busy_[leg_index] = true;
                }
            };

        send_goal_options.result_callback =
            [this, leg_index, on_done](const GoalHandleFJT::WrappedResult& result) {
                legs_busy_[leg_index] = false;
                bool success = (result.code == rclcpp_action::ResultCode::SUCCEEDED);
                if (!success) {
                    RCLCPP_DEBUG(this->get_logger(),
                        "Leg %d goal ended with code %d",
                        leg_index, (int)result.code);
                }
                if (on_done) on_done(success);
            };

        action_clients_[leg_index]->async_send_goal(goal_msg, send_goal_options);
    }
};

// ═════════════════════════════════════════════════════════════════════════
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HexapodWalkNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}