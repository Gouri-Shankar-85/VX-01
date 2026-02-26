/**
 * hexapod_walk_node.cpp
 *
 * ROS 2 node for VX-01 hexapod locomotion.
 *
 * Architecture — Full-Cycle Trajectory Streaming
 * ───────────────────────────────────────────────
 * Previous approach sent a new goal every block_period (0.5 s) and
 * cancelled in-flight goals at each block boundary.  This caused every
 * trajectory to be aborted ~100 ms after sending — legs never moved.
 *
 * Correct approach:
 *   1. Compute ONE trajectory per leg covering a FULL gait cycle
 *      (step_period = 3 s) sampled at TRAJ_HZ = 30 Hz → 90 waypoints.
 *   2. Send that goal once per leg.  The JTC's spline interpolator
 *      follows the complete smooth Bezier arc with ZERO cancellations.
 *   3. When the goal COMPLETES (3 s later), auto-loop for next cycle.
 *   4. When velocity changes: cancel current goals, resend new cycle.
 *   5. When stop: cancel + send stand goal.
 *
 * Tripod gait (6 blocks per cycle, block_period = step_period/6 = 0.5 s):
 *   Legs 0,2,4:  LIFT_UP | HOLD | LIFT_DOWN | HOLD | DRAG | HOLD
 *   Legs 1,3,5:  HOLD    | DRAG | HOLD      | LIFT_UP | HOLD | LIFT_DOWN
 *
 * Bezier swing trajectory (in stride/height space):
 *   P1=(−T/2, 0)  P2=(0, A)  P3=(+T/2, 0)
 *   where  T=stride length, A=step height
 *   x=foot position along stride axis, z=vertical lift
 *
 * IK mapping (leg-local frame):
 *   ik_x = HOME_X (constant reach along coxa axis)
 *   ik_y = stride_offset * velocity_scale  (→ theta1 coxa swing)
 *   ik_z = HOME_Z + vertical_lift           (→ theta2/3 femur+tibia)
 *
 * Joint limits (±45° = ±0.7854 rad, from YAML hardware_interface limits)
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
#include <mutex>
#include <cmath>
#include <algorithm>

#include "vx01_hexapod_locomotion/hexapod_locomotion.hpp"

using FollowJT      = control_msgs::action::FollowJointTrajectory;
using GoalHandleFJT = rclcpp_action::ClientGoalHandle<FollowJT>;


// ═════════════════════════════════════════════════════════════════════════
class HexapodWalkNode : public rclcpp::Node
{
public:
    // ── Robot / gait constants ────────────────────────────────────────────
    static constexpr int    NUM_LEGS       = 6;
    static constexpr double L1             = 60.55;    // coxa  mm
    static constexpr double L2             = 73.84;    // femur mm
    static constexpr double L3             = 112.16;   // tibia mm
    static constexpr double BODY_RADIUS    = 100.0;    // mm
    static constexpr double BETA_DEG       = 62.91;
    static constexpr double STEP_PERIOD    = 3.0;      // s — full 6-block cycle
    static constexpr double HOME_X         = 230.0;    // mm leg-local reach
    static constexpr double HOME_Y         = 0.0;
    static constexpr double HOME_Z         = -60.0;    // mm standing height
    static constexpr double JOINT_LIMIT    = 0.785398; // ±45° rad
    static constexpr double STAND_DURATION = 3.0;      // s for initial stand move
    // Gait params (Bezier shape)
    static constexpr double STRIDE         = 110.0;    // T mm
    static constexpr double STEP_HEIGHT    = 22.78;    // A mm
    // Trajectory sampling: 30 Hz → 90 waypoints per 3 s cycle
    static constexpr int    TRAJ_HZ        = 30;

    // ── Gait phase table ──────────────────────────────────────────────────
    enum class Phase { LIFT_UP, HOLD, LIFT_DOWN, DRAG };

    // Group A = legs 0,2,4;  Group B = legs 1,3,5
    static const Phase GAIT_A[6];
    static const Phase GAIT_B[6];

    const Phase* legPhaseTable(int leg) const {
        return (leg==0 || leg==2 || leg==4) ? GAIT_A : GAIT_B;
    }

    // ── Joint / action names ──────────────────────────────────────────────
    const std::array<std::array<std::string,3>,NUM_LEGS> JOINT_NAMES = {{
        {"coxa_leg0_joint","femur_leg0_joint","tibia_leg0_joint"},
        {"coxa_leg1_joint","femur_leg1_joint","tibia_leg1_joint"},
        {"coxa_leg2_joint","femur_leg2_joint","tibia_leg2_joint"},
        {"coxa_leg3_joint","femur_leg3_joint","tibia_leg3_joint"},
        {"coxa_leg4_joint","femur_leg4_joint","tibia_leg4_joint"},
        {"coxa_leg5_joint","femur_leg5_joint","tibia_leg5_joint"}
    }};
    const std::array<std::string,NUM_LEGS> ACTION_NAMES = {{
        "/leg_0_controller/follow_joint_trajectory",
        "/leg_1_controller/follow_joint_trajectory",
        "/leg_2_controller/follow_joint_trajectory",
        "/leg_3_controller/follow_joint_trajectory",
        "/leg_4_controller/follow_joint_trajectory",
        "/leg_5_controller/follow_joint_trajectory"
    }};

    // ── Constructor ───────────────────────────────────────────────────────
    explicit HexapodWalkNode(const rclcpp::NodeOptions& opts = rclcpp::NodeOptions())
    : Node("hexapod_walk_node", opts),
      locomotion_(L1, L2, L3, BODY_RADIUS, BETA_DEG * M_PI / 180.0),
      state_(State::INIT)
    {
        locomotion_.setStepPeriod(STEP_PERIOD);
        locomotion_.setHomePosition(HOME_X, HOME_Y, HOME_Z);

        cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&HexapodWalkNode::cmdVelCallback, this, std::placeholders::_1));

        for (int i=0; i<NUM_LEGS; ++i) {
            action_clients_[i] = rclcpp_action::create_client<FollowJT>(
                this, ACTION_NAMES[i]);
            legs_ready_[i]   = false;
            legs_cycling_[i] = false;
        }

        // 10 Hz timer only used in INIT state to poll server availability
        timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&HexapodWalkNode::timerCallback, this));

        clock_ = this->get_clock();

        RCLCPP_INFO(get_logger(), "HexapodWalkNode started — waiting for action servers...");
    }

private:
    enum class State { INIT, STANDING, WALKING };

    vx01_hexapod_locomotion::HexapodLocomotion locomotion_;
    std::array<rclcpp_action::Client<FollowJT>::SharedPtr, NUM_LEGS> action_clients_;
    std::array<bool, NUM_LEGS> legs_ready_;
    std::array<bool, NUM_LEGS> legs_cycling_;  // true while cycle goal is running

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    State state_;

    rclcpp::Clock::SharedPtr clock_;

    std::mutex vel_mutex_;
    double vel_x_{0.0}, vel_y_{0.0}, vel_omega_{0.0};

    // ── Math helpers ──────────────────────────────────────────────────────
    static double bz(double t, double p1, double p2, double p3) {
        double u = 1.0 - t;
        return u*u*p1 + 2.0*u*t*p2 + t*t*p3;
    }

    // Foot position in (stride-offset, vertical-lift) space for phase+t
    void phaseFootPos(Phase ph, double t,
                      double& stride_out, double& lift_out) const
    {
        switch (ph) {
            case Phase::LIFT_UP: {
                // First half of Bezier arc
                double tb = t * 0.5;
                stride_out = bz(tb, -STRIDE/2.0, 0.0, STRIDE/2.0);
                lift_out   = bz(tb,  0.0, STEP_HEIGHT, 0.0);
                break;
            }
            case Phase::LIFT_DOWN: {
                // Second half of Bezier arc
                double tb = 0.5 + t * 0.5;
                stride_out = bz(tb, -STRIDE/2.0, 0.0, STRIDE/2.0);
                lift_out   = bz(tb,  0.0, STEP_HEIGHT, 0.0);
                break;
            }
            case Phase::DRAG:
                // Stance: sweep from front to rear (body moves forward)
                stride_out = STRIDE/2.0 - t * STRIDE;
                lift_out   = 0.0;
                break;
            case Phase::HOLD:
            default:
                stride_out = 0.0;
                lift_out   = 0.0;
                break;
        }
    }

    // Elbow-up IK with joint limit check
    bool ikEU(double xp, double yp, double zp,
              double& t1, double& t2, double& t3) const
    {
        t1 = std::atan2(yp, xp);
        double ct1 = std::cos(t1);
        if (std::abs(ct1) < 1e-9) return false;
        double r2  = xp / ct1 - L1;
        double r1  = std::sqrt(zp*zp + r2*r2);
        double phi2 = std::atan2(zp, r2);
        double cp1 = (L3*L3 - L2*L2 - r1*r1) / (-2.0*L2*r1);
        if (cp1 < -1.0 || cp1 > 1.0) return false;
        double phi1 = std::acos(cp1);
        t2 = phi2 - phi1;                        // elbow-up: femur tips down
        double cp3 = (r1*r1 - L2*L2 - L3*L3) / (-2.0*L2*L3);
        if (cp3 < -1.0 || cp3 > 1.0) return false;
        t3 = M_PI - std::acos(cp3);              // elbow-up: tibia comes up
        // Enforce joint limits
        return (std::abs(t1) <= JOINT_LIMIT &&
                std::abs(t2) <= JOINT_LIMIT &&
                std::abs(t3) <= JOINT_LIMIT);
    }

    double clampAngle(double a) const {
        return std::max(-JOINT_LIMIT, std::min(JOINT_LIMIT, a));
    }

    // ── Build full-cycle trajectory for one leg ───────────────────────────
    FollowJT::Goal buildCycleGoal(int leg, double vx) const
    {
        const double block_period   = STEP_PERIOD / 6.0;
        const double nominal_speed  = STRIDE / (2.0 * block_period);
        const double scale          = (nominal_speed > 1e-6) ? (vx / nominal_speed) : 0.0;
        const Phase* table          = legPhaseTable(leg);
        const double dt             = 1.0 / TRAJ_HZ;

        // Home IK (fallback for any failing waypoint)
        double h1=0, h2=0, h3=0;
        bool home_ok = ikEU(HOME_X, 0.0, HOME_Z, h1, h2, h3);
        if (!home_ok) {
            RCLCPP_ERROR(get_logger(), "Home position IK FAILED — check HOME_X/Z");
        }

        FollowJT::Goal goal;
        goal.trajectory.joint_names = {
            JOINT_NAMES[leg][0], JOINT_NAMES[leg][1], JOINT_NAMES[leg][2]
        };

        int n = static_cast<int>(std::ceil(STEP_PERIOD * TRAJ_HZ));
        goal.trajectory.points.reserve(n + 1);

        for (int i = 0; i <= n; ++i) {
            double t_abs  = std::min(static_cast<double>(i) * dt, STEP_PERIOD);
            int    block  = static_cast<int>(t_abs / block_period) % 6;
            double t_norm = std::fmod(t_abs, block_period) / block_period;

            double stride, lift;
            phaseFootPos(table[block], t_norm, stride, lift);

            double ik_x = HOME_X;
            double ik_y = stride * scale;
            double ik_z = HOME_Z + lift;

            double t1, t2, t3;
            if (!ikEU(ik_x, ik_y, ik_z, t1, t2, t3)) {
                t1 = h1; t2 = h2; t3 = h3;
                RCLCPP_WARN_THROTTLE(this->get_logger(), *clock_, 2000,
                    "IK fallback leg=%d t=%.2f (x=%.1f y=%.1f z=%.1f)",
                    leg, t_abs, ik_x, ik_y, ik_z);
            }

            trajectory_msgs::msg::JointTrajectoryPoint pt;
            pt.positions  = {clampAngle(t1), clampAngle(t2), clampAngle(t3)};
            pt.velocities = {0.0, 0.0, 0.0};
            pt.time_from_start = rclcpp::Duration::from_seconds(t_abs);
            goal.trajectory.points.push_back(pt);
        }

        return goal;
    }

    // ── Send full-cycle goal to one leg, auto-loop on success ─────────────
    void sendCycleGoal(int leg, double vx)
    {
        FollowJT::Goal goal = buildCycleGoal(leg, vx);

        auto send_opts = rclcpp_action::Client<FollowJT>::SendGoalOptions();

        send_opts.goal_response_callback =
            [this, leg](const GoalHandleFJT::SharedPtr& handle) {
                if (!handle) {
                    RCLCPP_WARN(get_logger(), "Cycle goal REJECTED by leg_%d", leg);
                    legs_cycling_[leg] = false;
                } else {
                    legs_cycling_[leg] = true;
                    RCLCPP_DEBUG(get_logger(), "Leg %d cycle goal accepted", leg);
                }
            };

        send_opts.result_callback =
            [this, leg](const GoalHandleFJT::WrappedResult& result) {
                legs_cycling_[leg] = false;

                if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                    // Cycle done — loop if still walking
                    if (state_ == State::WALKING) {
                        double vx;
                        { std::lock_guard<std::mutex> lk(vel_mutex_); vx = vel_x_; }
                        RCLCPP_DEBUG(get_logger(), "Leg %d cycle complete — looping", leg);
                        sendCycleGoal(leg, vx);
                    }
                } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
                    // Intentional cancel (velocity change or stop) — no action
                    RCLCPP_DEBUG(get_logger(), "Leg %d cycle cancelled (expected)", leg);
                } else {
                    RCLCPP_WARN(get_logger(),
                        "Leg %d cycle ended code=%d", leg, (int)result.code);
                }
            };

        action_clients_[leg]->async_send_goal(goal, send_opts);
    }

    // ── Cancel all currently-running cycle goals ──────────────────────────
    void cancelAllCycleGoals()
    {
        for (int i = 0; i < NUM_LEGS; ++i) {
            if (legs_cycling_[i]) {
                action_clients_[i]->async_cancel_all_goals();
            }
        }
    }

    // ── Send stand (home) position to all legs ────────────────────────────
    void sendStandGoal()
    {
        locomotion_.stand();  // computes home IK angles

        for (int i = 0; i < NUM_LEGS; ++i) {
            double t1, t2, t3;
            locomotion_.getLegAngles(i, t1, t2, t3);

            FollowJT::Goal goal;
            goal.trajectory.joint_names = {
                JOINT_NAMES[i][0], JOINT_NAMES[i][1], JOINT_NAMES[i][2]
            };
            trajectory_msgs::msg::JointTrajectoryPoint pt;
            pt.positions  = {clampAngle(t1), clampAngle(t2), clampAngle(t3)};
            pt.velocities = {0.0, 0.0, 0.0};
            pt.time_from_start = rclcpp::Duration::from_seconds(STAND_DURATION);
            goal.trajectory.points.push_back(pt);

            auto send_opts = rclcpp_action::Client<FollowJT>::SendGoalOptions();
            send_opts.result_callback =
                [this, i](const GoalHandleFJT::WrappedResult& result) {
                    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                        RCLCPP_DEBUG(get_logger(), "Stand done leg %d", i);
                    }
                };
            action_clients_[i]->async_send_goal(goal, send_opts);
        }
        RCLCPP_INFO(get_logger(),
            "Stand goal sent (%.1f s) — angles: t2=%.2f t3=%.2f rad",
            STAND_DURATION,
            locomotion_.getStepHeight(),  // just for info
            STAND_DURATION);
    }

    // ── Transition to WALKING state ───────────────────────────────────────
    void startWalking(double vx)
    {
        RCLCPP_INFO(get_logger(), "Starting walk — vx=%.1f mm/s", vx);
        locomotion_.setVelocity(vx, 0.0, 0.0);
        locomotion_.walk();
        state_ = State::WALKING;

        for (int i = 0; i < NUM_LEGS; ++i) {
            sendCycleGoal(i, vx);
        }
    }

    // ── /cmd_vel subscriber ───────────────────────────────────────────────
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        double new_vx    = msg->linear.x  * 1000.0;  // m/s → mm/s
        double new_vy    = msg->linear.y  * 1000.0;
        double new_omega = msg->angular.z;

        {
            std::lock_guard<std::mutex> lk(vel_mutex_);
            vel_x_     = new_vx;
            vel_y_     = new_vy;
            vel_omega_ = new_omega;
        }

        bool nonzero = (std::abs(new_vx)    > 1.0 ||
                        std::abs(new_vy)    > 1.0 ||
                        std::abs(new_omega) > 0.01);

        if (state_ == State::STANDING && nonzero) {
            startWalking(new_vx);

        } else if (state_ == State::WALKING && nonzero) {
            // Speed changed — cancel old cycles and send new ones
            RCLCPP_INFO(get_logger(),
                "Velocity update vx=%.1f — restarting walk cycles", new_vx);
            cancelAllCycleGoals();
            for (int i = 0; i < NUM_LEGS; ++i) {
                sendCycleGoal(i, new_vx);
            }

        } else if (state_ == State::WALKING && !nonzero) {
            RCLCPP_INFO(get_logger(), "Stop command — returning to stand");
            cancelAllCycleGoals();
            locomotion_.stand();
            state_ = State::STANDING;
            sendStandGoal();
        }
    }

    // ── 10 Hz timer — INIT state only ─────────────────────────────────────
    void timerCallback()
    {
        if (state_ != State::INIT) return;

        bool all_ready = true;
        for (int i = 0; i < NUM_LEGS; ++i) {
            if (!legs_ready_[i]) {
                if (action_clients_[i]->action_server_is_ready()) {
                    legs_ready_[i] = true;
                    RCLCPP_INFO(get_logger(), "leg_%d action server ready", i);
                } else {
                    all_ready = false;
                }
            }
        }

        if (all_ready) {
            RCLCPP_INFO(get_logger(),
                "All action servers ready — moving to stand position");
            state_ = State::STANDING;
            sendStandGoal();
        }
    }
};

// Out-of-class constexpr definitions
const HexapodWalkNode::Phase HexapodWalkNode::GAIT_A[6] = {
    HexapodWalkNode::Phase::LIFT_UP,
    HexapodWalkNode::Phase::HOLD,
    HexapodWalkNode::Phase::LIFT_DOWN,
    HexapodWalkNode::Phase::HOLD,
    HexapodWalkNode::Phase::DRAG,
    HexapodWalkNode::Phase::HOLD
};
const HexapodWalkNode::Phase HexapodWalkNode::GAIT_B[6] = {
    HexapodWalkNode::Phase::HOLD,
    HexapodWalkNode::Phase::DRAG,
    HexapodWalkNode::Phase::HOLD,
    HexapodWalkNode::Phase::LIFT_UP,
    HexapodWalkNode::Phase::HOLD,
    HexapodWalkNode::Phase::LIFT_DOWN
};

// ─────────────────────────────────────────────────────────────────────────
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HexapodWalkNode>());
    rclcpp::shutdown();
    return 0;
}