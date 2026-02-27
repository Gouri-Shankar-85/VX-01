/**
 * hexapod_walk_node.cpp  —  VX01 Hexapod  (ZERO IK / DIRECT ANGLES)
 *
 * ALL joint angles computed directly from geometry — no IK solver at all.
 * Tibia is FIXED at -0.75 rad (damaged servos — never commanded otherwise).
 *
 * ── Robot geometry (L1=60.55  L2=73.84  L3=112.16 mm) ────────────────────────
 *
 *   foot_z = L2·sin(femur) + L3·sin(femur + TIBIA_FIXED)
 *   foot_x = L1 + L2·cos(femur) + L3·cos(femur + TIBIA_FIXED)
 *
 *   Simulator initial_positions.yaml sets all joints to 0.0.
 *   At femur=0.0 → foot_z = -76.5 mm  (already on ground, reasonable start)
 *
 *   FEMUR_STAND = +0.0873 rad (+5 deg) → foot_z = -62.6 mm  [standing height]
 *   FEMUR_LIFT  = +0.1745 rad (+10 deg) → foot_z = -48.2 mm  [swing apex, 14mm lift]
 *   TOTAL_REACH = 222.5 mm  (foot X from leg origin at FEMUR_STAND)
 *
 *   All angles well within joint limit ±0.785398 rad (±45 deg).
 *
 * ── Direct angle computation for walking ──────────────────────────────────────
 *
 *   coxa  = atan2(stride_y, TOTAL_REACH)
 *             → pure trig, no IK, sweeps foot ±40mm in stride direction
 *   femur = FEMUR_STAND + lift_frac × (FEMUR_LIFT − FEMUR_STAND)
 *             → linear blend: 0=on ground, 1=full apex lift
 *   tibia = TIBIA_FIXED = −0.75 rad  (never changes)
 *
 * ── Startup sequence ──────────────────────────────────────────────────────────
 *   1. INIT     : wait for all 6 JTC action servers
 *   2. STANDING : coxa=0, femur=0 → +0.0873, tibia=-0.75  (smooth stand, 3s)
 *                 Uses 2-point trajectory from sim start pose directly to stand pose.
 *   3. WALKING  : tripod gait on /cmd_vel (direct angles, zero IK)
 *
 * ── Tripod gait ───────────────────────────────────────────────────────────────
 *   Group A (legs 0,2,4): SWING in first half-cycle,  STANCE in second half
 *   Group B (legs 1,3,5): STANCE in first half-cycle, SWING  in second half
 *   Always 3 legs on ground → stable tripod support at every instant.
 *
 *   SWING  : Bezier arc  stride_y: -S→0→+S,  lift_frac: 0→1→0
 *   STANCE : Linear push stride_y: +S→-S,     lift_frac: 0
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

using FollowJT      = control_msgs::action::FollowJointTrajectory;
using GoalHandleFJT = rclcpp_action::ClientGoalHandle<FollowJT>;

class HexapodWalkNode : public rclcpp::Node
{
public:
    static constexpr int NUM_LEGS = 6;

    // ── Fixed constants — computed from robot geometry, never change ───────
    //
    //   foot_z = L2·sin(femur) + L3·sin(femur + TIBIA_FIXED)
    //
    //   TIBIA_FIXED  = -0.75   → tibia always here (damaged servos)
    //   FEMUR_ZERO   =  0.0    → sim start position, foot_z = -76.5 mm
    //   FEMUR_STAND  = +0.0873 → foot_z = -62.6 mm  (body 62mm above ground)
    //   FEMUR_LIFT   = +0.1745 → foot_z = -48.2 mm  (14mm lift at swing apex)
    //   TOTAL_REACH  = 222.5   → foot X at FEMUR_STAND (used for coxa atan2)
    //
    static constexpr double TIBIA_FIXED  = -0.7500;  // rad  FIXED — do not change
    static constexpr double FEMUR_ZERO   =  0.0000;  // rad  sim initial position
    static constexpr double FEMUR_STAND  =  0.0873;  // rad = +5 deg,  z=-62.6mm
    static constexpr double FEMUR_LIFT   =  0.1745;  // rad = +10 deg, z=-48.2mm
    static constexpr double TOTAL_REACH  = 222.5;    // mm

    static constexpr double JOINT_LIMIT  =  0.7854;  // rad = ±45 deg

    const std::array<std::array<std::string,3>, NUM_LEGS> JOINT_NAMES = {{
        {"coxa_leg0_joint","femur_leg0_joint","tibia_leg0_joint"},
        {"coxa_leg1_joint","femur_leg1_joint","tibia_leg1_joint"},
        {"coxa_leg2_joint","femur_leg2_joint","tibia_leg2_joint"},
        {"coxa_leg3_joint","femur_leg3_joint","tibia_leg3_joint"},
        {"coxa_leg4_joint","femur_leg4_joint","tibia_leg4_joint"},
        {"coxa_leg5_joint","femur_leg5_joint","tibia_leg5_joint"}
    }};

    const std::array<std::string, NUM_LEGS> ACTION_NAMES = {{
        "/leg_0_controller/follow_joint_trajectory",
        "/leg_1_controller/follow_joint_trajectory",
        "/leg_2_controller/follow_joint_trajectory",
        "/leg_3_controller/follow_joint_trajectory",
        "/leg_4_controller/follow_joint_trajectory",
        "/leg_5_controller/follow_joint_trajectory"
    }};

    explicit HexapodWalkNode(const rclcpp::NodeOptions& opts = rclcpp::NodeOptions())
    : Node("hexapod_walk_node", opts), state_(State::INIT)
    {
        // All key constants are compile-time (geometry-derived), not parameters.
        // Only gait timing and velocity are runtime parameters.
        declare_parameter("step_period",            4.0);   // full gait cycle (s)
        declare_parameter("stride_length",         80.0);   // total stride Y (mm)
        declare_parameter("traj_hz",                50);    // trajectory sample rate
        declare_parameter("stand_duration",          3.0);  // stand-up duration (s)
        declare_parameter("vel_debounce_threshold",  5.0);  // mm/s change to replan

        step_period_    = get_parameter("step_period").as_double();
        stride_length_  = get_parameter("stride_length").as_double();
        traj_hz_        = get_parameter("traj_hz").as_int();
        stand_duration_ = get_parameter("stand_duration").as_double();
        vel_debounce_   = get_parameter("vel_debounce_threshold").as_double();

        clock_ = get_clock();

        cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&HexapodWalkNode::cmdVelCallback, this, std::placeholders::_1));

        for (int i = 0; i < NUM_LEGS; ++i) {
            action_clients_[i] = rclcpp_action::create_client<FollowJT>(this, ACTION_NAMES[i]);
            legs_ready_[i]     = false;
            legs_cycling_[i]   = false;
        }

        timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&HexapodWalkNode::timerCallback, this));

        RCLCPP_INFO(get_logger(), "================================================");
        RCLCPP_INFO(get_logger(), " HexapodWalkNode  —  ZERO IK / DIRECT ANGLES");
        RCLCPP_INFO(get_logger(), "  TIBIA_FIXED  = %+.4f rad  (FIXED, damaged servos)", TIBIA_FIXED);
        RCLCPP_INFO(get_logger(), "  FEMUR_ZERO   = %+.4f rad  foot_z ~ -76.6 mm  [sim start]", FEMUR_ZERO);
        RCLCPP_INFO(get_logger(), "  FEMUR_STAND  = %+.4f rad  foot_z ~ -62.6 mm  [standing]", FEMUR_STAND);
        RCLCPP_INFO(get_logger(), "  FEMUR_LIFT   = %+.4f rad  foot_z ~ -48.2 mm  [swing apex]", FEMUR_LIFT);
        RCLCPP_INFO(get_logger(), "  TOTAL_REACH  = %.1f mm", TOTAL_REACH);
        RCLCPP_INFO(get_logger(), "  stride=%.0fmm  period=%.1fs  hz=%d",
                    stride_length_, step_period_, traj_hz_);
        RCLCPP_INFO(get_logger(), "================================================");
    }

private:
    enum class State { INIT, STANDING, WALKING };

    double step_period_;
    double stride_length_;
    int    traj_hz_;
    double stand_duration_;
    double vel_debounce_;

    std::array<rclcpp_action::Client<FollowJT>::SharedPtr, NUM_LEGS> action_clients_;
    std::array<bool, NUM_LEGS> legs_ready_;
    std::array<bool, NUM_LEGS> legs_cycling_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Clock::SharedPtr clock_;

    State  state_;
    std::mutex vel_mutex_;
    double vel_x_{0.0}, vel_y_{0.0}, vel_omega_{0.0};
    double last_sent_vx_{0.0};

    // ── Clamp to joint limit ──────────────────────────────────────────────
    static double clamp(double v) {
        return std::max(-JOINT_LIMIT, std::min(JOINT_LIMIT, v));
    }

    // ── Quadratic Bezier scalar ───────────────────────────────────────────
    static double bz(double t, double p0, double p1, double p2) {
        double u = 1.0 - t;
        return u*u*p0 + 2.0*u*t*p1 + t*t*p2;
    }

    // ── Core: compute joint angles directly from foot position ────────────
    //
    //  stride_y  : foot Y target in stride direction (mm)
    //              + = forward,  − = backward
    //              range: [−stride_length/2 .. +stride_length/2]
    //
    //  lift_frac : swing lift fraction [0.0 .. 1.0]
    //              0.0 = foot on ground (FEMUR_STAND)
    //              1.0 = full apex lift  (FEMUR_LIFT)
    //
    //  coxa  = atan2(stride_y, TOTAL_REACH)   ← direct trig, no iteration
    //  femur = FEMUR_STAND + lift_frac × (FEMUR_LIFT − FEMUR_STAND)
    //  tibia = TIBIA_FIXED                     ← never changes
    //
    static void directAngles(double stride_y, double lift_frac,
                              double& coxa_out, double& femur_out)
    {
        coxa_out  = std::atan2(stride_y, TOTAL_REACH);
        femur_out = FEMUR_STAND + lift_frac * (FEMUR_LIFT - FEMUR_STAND);
    }

    // ── Gait phase at absolute time ───────────────────────────────────────
    //
    // Tripod gait, duty factor = 0.5:
    //   Group A (legs 0,2,4): SWING [0..half),  STANCE [half..period)
    //   Group B (legs 1,3,5): STANCE [0..half), SWING  [half..period)
    //
    // SWING phase (foot in air, moving forward):
    //   stride_y  = Bezier(−S, 0, +S)   smooth arc rear→front
    //   lift_frac = Bezier(0, 1, 0)     ground→apex→ground
    //
    // STANCE phase (foot on ground, pushing backward):
    //   stride_y  = linear +S → −S     foot pushes body forward
    //   lift_frac = 0.0                foot stays on ground
    //
    void legPhaseAtTime(int leg, double t_abs, double vx_scale,
                        double& stride_y_out, double& lift_frac_out) const
    {
        const double half  = step_period_ * 0.5;
        const bool group_b = (leg == 1 || leg == 3 || leg == 5);

        // Offset group B by half period → always in antiphase with group A
        double t_loc = std::fmod(t_abs + (group_b ? half : 0.0), step_period_);

        // Half-stride scaled by velocity ratio
        const double S = (stride_length_ * 0.5) * vx_scale;

        if (t_loc < half) {
            // ── SWING ─────────────────────────────────────────────────────
            const double s   = t_loc / half;          // [0..1]
            stride_y_out     = bz(s, -S,  0.0,  +S);  // rear → front (Bezier)
            lift_frac_out    = bz(s, 0.0, 1.0, 0.0);  // lift arc     (Bezier)
        } else {
            // ── STANCE ────────────────────────────────────────────────────
            const double s   = (t_loc - half) / half; // [0..1]
            stride_y_out     = +S - s * (2.0 * S);    // front → rear (linear)
            lift_frac_out    = 0.0;                    // foot on ground
        }
    }

    // ── Build one full-cycle trajectory for one leg (ZERO IK) ─────────────
    FollowJT::Goal buildCycleGoal(int leg, double vx) const
    {
        // Velocity scale factor
        const double nom_spd = (step_period_ > 1e-9)
                               ? stride_length_ / (step_period_ * 0.5)
                               : 1.0;
        const double vx_scale = (nom_spd > 1e-6)
                                ? std::max(-1.5, std::min(1.5, vx / nom_spd))
                                : 0.0;

        const double dt = 1.0 / static_cast<double>(traj_hz_);
        const int    n  = static_cast<int>(std::ceil(step_period_ * traj_hz_));

        FollowJT::Goal goal;
        goal.trajectory.joint_names = {
            JOINT_NAMES[leg][0], JOINT_NAMES[leg][1], JOINT_NAMES[leg][2]
        };
        goal.trajectory.points.reserve(n + 1);

        for (int i = 0; i <= n; ++i) {
            double t_abs = std::min(static_cast<double>(i) * dt, step_period_);

            double stride_y, lift_frac;
            legPhaseAtTime(leg, t_abs, vx_scale, stride_y, lift_frac);

            double coxa, femur;
            directAngles(stride_y, lift_frac, coxa, femur);

            trajectory_msgs::msg::JointTrajectoryPoint pt;
            pt.positions  = { clamp(coxa), clamp(femur), TIBIA_FIXED };
            pt.velocities = { 0.0, 0.0, 0.0 };
            pt.time_from_start = rclcpp::Duration::from_seconds(t_abs);
            goal.trajectory.points.push_back(pt);
        }

        return goal;
    }

    // ── Send cycle goal — auto-loops seamlessly while WALKING ─────────────
    void sendCycleGoal(int leg, double vx)
    {
        auto goal = buildCycleGoal(leg, vx);
        auto opts = rclcpp_action::Client<FollowJT>::SendGoalOptions();

        opts.goal_response_callback =
            [this, leg](const GoalHandleFJT::SharedPtr& handle) {
                legs_cycling_[leg] = static_cast<bool>(handle);
                if (!handle)
                    RCLCPP_WARN(get_logger(), "Cycle goal REJECTED leg_%d", leg);
            };

        opts.result_callback =
            [this, leg](const GoalHandleFJT::WrappedResult& result) {
                legs_cycling_[leg] = false;
                if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                    if (state_ == State::WALKING) {
                        double vx;
                        { std::lock_guard<std::mutex> lk(vel_mutex_); vx = vel_x_; }
                        sendCycleGoal(leg, vx);  // seamless loop
                    }
                } else if (result.code != rclcpp_action::ResultCode::CANCELED) {
                    RCLCPP_WARN(get_logger(), "Leg %d cycle ended code=%d",
                                leg, static_cast<int>(result.code));
                }
            };

        action_clients_[leg]->async_send_goal(goal, opts);
    }

    void cancelAllCycleGoals()
    {
        for (int i = 0; i < NUM_LEGS; ++i) {
            if (legs_cycling_[i]) {
                action_clients_[i]->async_cancel_all_goals();
                legs_cycling_[i] = false;
            }
        }
    }

    // ── STAGE 1: STAND ────────────────────────────────────────────────────
    //
    // Smoothly moves ALL legs from sim start pose (all joints = 0.0) to
    // the standing pose in one clean trajectory.
    //
    // 2-point trajectory:
    //   t=0.0s : coxa=0, femur=FEMUR_ZERO=0.0,   tibia=-0.75  [sim start]
    //   t=3.0s : coxa=0, femur=FEMUR_STAND=+0.0873, tibia=-0.75  [standing]
    //
    // This is a gentle +5 deg femur move → body rises from z=-76mm to z=-63mm.
    // All 6 legs move simultaneously and identically.
    //
    void sendStandGoal()
    {
        RCLCPP_INFO(get_logger(),
            "STAGE 1 — STAND: femur  %.4f → %.4f rad  (%.1f → %.1f mm body height)  over %.1fs",
            FEMUR_ZERO, FEMUR_STAND, -76.5, -62.6, stand_duration_);

        for (int i = 0; i < NUM_LEGS; ++i) {
            FollowJT::Goal goal;
            goal.trajectory.joint_names = {
                JOINT_NAMES[i][0], JOINT_NAMES[i][1], JOINT_NAMES[i][2]
            };

            // Point 0 — sim initial position (all zeros)
            trajectory_msgs::msg::JointTrajectoryPoint pt0;
            pt0.positions  = { 0.0, FEMUR_ZERO, TIBIA_FIXED };
            pt0.velocities = { 0.0, 0.0, 0.0 };
            pt0.time_from_start = rclcpp::Duration::from_seconds(0.0);
            goal.trajectory.points.push_back(pt0);

            // Point 1 — standing pose
            trajectory_msgs::msg::JointTrajectoryPoint pt1;
            pt1.positions  = { 0.0, FEMUR_STAND, TIBIA_FIXED };
            pt1.velocities = { 0.0, 0.0, 0.0 };
            pt1.time_from_start = rclcpp::Duration::from_seconds(stand_duration_);
            goal.trajectory.points.push_back(pt1);

            auto opts = rclcpp_action::Client<FollowJT>::SendGoalOptions();

            // Monitor leg 5 only — it finishes last, then log ready
            if (i == NUM_LEGS - 1) {
                opts.result_callback =
                    [this](const GoalHandleFJT::WrappedResult& result) {
                        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                            RCLCPP_INFO(get_logger(),
                                "STAGE 1 complete — standing at foot_z=-62.6mm — waiting /cmd_vel");
                        } else {
                            RCLCPP_WARN(get_logger(),
                                "Stand goal ended code=%d", static_cast<int>(result.code));
                        }
                    };
            }

            action_clients_[i]->async_send_goal(goal, opts);
        }
    }

    // ── STAGE 2: WALK ─────────────────────────────────────────────────────
    void startWalking(double vx)
    {
        RCLCPP_INFO(get_logger(),
            "STAGE 2 — WALKING  vx=%.1f mm/s  coxa_max=±%.2f deg",
            vx,
            std::abs(std::atan2(stride_length_*0.5*std::min(1.5, std::abs(vx) /
                (stride_length_ / (step_period_*0.5))), TOTAL_REACH)) * 180.0 / M_PI);
        last_sent_vx_ = vx;
        state_ = State::WALKING;
        for (int i = 0; i < NUM_LEGS; ++i) {
            sendCycleGoal(i, vx);
        }
    }

    // ── /cmd_vel callback ─────────────────────────────────────────────────
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        const double new_vx    = msg->linear.x  * 1000.0;  // m/s → mm/s
        const double new_vy    = msg->linear.y  * 1000.0;
        const double new_omega = msg->angular.z;

        { std::lock_guard<std::mutex> lk(vel_mutex_);
          vel_x_ = new_vx; vel_y_ = new_vy; vel_omega_ = new_omega; }

        const bool nonzero = (std::abs(new_vx)    > 1.0  ||
                              std::abs(new_vy)    > 1.0  ||
                              std::abs(new_omega) > 0.01);

        if (state_ == State::STANDING && nonzero) {
            // Start walking
            startWalking(new_vx);

        } else if (state_ == State::WALKING && nonzero) {
            // Re-plan if velocity changed significantly
            const double delta = std::abs(new_vx - last_sent_vx_);
            if (delta > vel_debounce_) {
                RCLCPP_INFO(get_logger(),
                    "Velocity %.1f → %.1f mm/s — replanning gait",
                    last_sent_vx_, new_vx);
                cancelAllCycleGoals();
                last_sent_vx_ = new_vx;
                for (int i = 0; i < NUM_LEGS; ++i) sendCycleGoal(i, new_vx);
            }

        } else if (state_ == State::WALKING && !nonzero) {
            // Return to stand
            RCLCPP_INFO(get_logger(), "cmd_vel = 0 — returning to STAND");
            cancelAllCycleGoals();
            state_ = State::STANDING;
            last_sent_vx_ = 0.0;
            // Return: go back to stand pose smoothly
            sendReturnToStand();
        }
    }

    // ── Return to stand after walking ─────────────────────────────────────
    // Sends a single-point goal to FEMUR_STAND (coxa=0) so the robot
    // returns to a clean symmetric standing pose after walking stops.
    void sendReturnToStand()
    {
        RCLCPP_INFO(get_logger(), "Returning to stand pose...");
        for (int i = 0; i < NUM_LEGS; ++i) {
            FollowJT::Goal goal;
            goal.trajectory.joint_names = {
                JOINT_NAMES[i][0], JOINT_NAMES[i][1], JOINT_NAMES[i][2]
            };
            trajectory_msgs::msg::JointTrajectoryPoint pt;
            pt.positions  = { 0.0, FEMUR_STAND, TIBIA_FIXED };
            pt.velocities = { 0.0, 0.0, 0.0 };
            pt.time_from_start = rclcpp::Duration::from_seconds(1.5);
            goal.trajectory.points.push_back(pt);
            action_clients_[i]->async_send_goal(goal, {});
        }
    }

    // ── Timer: wait for all 6 action servers, then stand ──────────────────
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
                "All 6 servers ready — sending STAND goal");
            state_ = State::STANDING;
            timer_->cancel();
            sendStandGoal();
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HexapodWalkNode>());
    rclcpp::shutdown();
    return 0;
}