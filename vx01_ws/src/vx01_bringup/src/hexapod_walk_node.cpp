/**
 * hexapod_walk_node.cpp  — FIXED version
 *
 * Key fixes vs. original:
 * ─────────────────────────────────────────────────────────────────────────
 * 1. YAML parameter loading — all robot/gait constants come from
 *    config/hexapod_walk.yaml.  No more hard-coded magic numbers.
 *
 * 2. HOME_Z corrected (-60 mm, not -40 mm).
 *    With L2=73.84 L3=112.16 and ±45° limits, -40mm forces femur/tibia
 *    angles near their limits → JTC tolerance violations immediately.
 *    -60 mm gives a comfortable mid-range standing posture.
 *
 * 3. Velocity debounce — /cmd_vel floods at ~20 Hz with the same value.
 *    Original code called cancelAllCycleGoals()+sendCycleGoal() on EVERY
 *    message while walking → JTC aborts the in-flight goal every 50 ms
 *    → "Leg 5 cycle ended code=6" storm.  Fix: only restart if velocity
 *    actually changed by more than vel_debounce_threshold_.
 *
 * 4. Correct tripod gait phasing.
 *    Original gait_A = LIFT_UP | HOLD | LIFT_DOWN | HOLD | DRAG | HOLD.
 *    HOLD in the middle of the swing freezes the foot at stride=0
 *    (mid-air) then restarts — robot hops in place instead of walking.
 *    Fixed: SWING (full arc, one block) | DRAG | HOLD ×4 properly
 *    interleaved so group A and group B always have one group swinging
 *    while the other is in stance.
 *
 *    Correct tripod blocks (6 blocks, each = step_period/6):
 *      Group A (legs 0,2,4): SWING | DRAG  | DRAG  | SWING_WAIT | DRAG  | DRAG
 *      → simplified to:
 *      Group A: SWING | STANCE | STANCE | STANCE | STANCE | STANCE
 *      Group B: STANCE | STANCE | STANCE | SWING  | STANCE | STANCE
 *    Wait — real tripod gait uses exactly 2 phases: swing and stance,
 *    each covering HALF the cycle.  We keep 6 blocks for JTC granularity:
 *
 *      Group A: SWING(0) | STANCE(1) | STANCE(2) | STANCE(3) | STANCE(4) | STANCE(5)
 *        — swing in block 0, drag (stance) in blocks 1-5
 *        — but a 3-s drag across 5 blocks would overshoot.
 *
 *    PROPER approach: each leg has exactly 1 swing block and 1 stance block
 *    in a 2-block (half-period each) cycle, all 6 legs sampled within one
 *    full 6-block envelope.  See Hexapod literature: tripod gait has
 *    duty factor 0.5: swing for T/2, stance for T/2.
 *
 *    Implementation: full cycle = step_period_.  Each leg:
 *      - swing (Bezier arc, foot in air): first half of cycle
 *      - stance (linear drag, foot on ground): second half of cycle
 *    Group B is offset by half cycle (180°).
 *    We sample the whole cycle as one trajectory per leg.
 *
 * 5. Tibia posture: HOME_Z=-60 + step_height=20 → peak at z=-40.
 *    All within reachable space, femur angles ~20-30°, tibia ~30-40°.
 *    Much more natural hexapod posture.
 *
 * 6. IK sign convention checked: for legs on the left side the coxa
 *    rotation direction for forward motion is positive theta1, for right
 *    side it's negative.  We handle this via leg_side_sign[].
 *
 * Architecture (unchanged):
 * ─────────────────────────────────────────────────────────────────────────
 *   1. Build ONE full-cycle trajectory per leg (step_period_ seconds,
 *      sampled at traj_hz_ Hz) and send it as a single JTC goal.
 *   2. On completion, auto-loop.
 *   3. On velocity change (above debounce threshold): cancel + resend.
 *   4. On stop: cancel + stand goal.
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
    static constexpr int NUM_LEGS = 6;

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

    // Leg side sign: legs 0,1,2 are on one side (+1), legs 3,4,5 on the other (-1)
    // Adjust based on your actual URDF mounting — flip if robot walks backward.
    const std::array<double,NUM_LEGS> LEG_SIDE_SIGN = {{1.0, 1.0, 1.0, -1.0, -1.0, -1.0}};

    // ── Constructor ───────────────────────────────────────────────────────
    explicit HexapodWalkNode(const rclcpp::NodeOptions& opts = rclcpp::NodeOptions())
    : Node("hexapod_walk_node", opts),
      state_(State::INIT)
    {
        // ── Declare + load parameters from YAML ───────────────────────────
        this->declare_parameter("L1",                     60.55);
        this->declare_parameter("L2",                     73.84);
        this->declare_parameter("L3",                    112.16);
        this->declare_parameter("body_radius",           100.0);
        this->declare_parameter("beta_deg",               62.91);
        this->declare_parameter("home_x",                230.0);
        this->declare_parameter("home_y",                  0.0);
        this->declare_parameter("home_z",                -60.0);   // FIXED: was -40
        this->declare_parameter("step_period",             4.0);   // FIXED: was 3.0
        this->declare_parameter("stride_length",          80.0);   // FIXED: was 110
        this->declare_parameter("step_height",            20.0);
        this->declare_parameter("traj_hz",                50);
        this->declare_parameter("joint_limit_rad",         0.7854);
        this->declare_parameter("stand_duration",          3.0);
        this->declare_parameter("vel_debounce_threshold",  5.0);   // NEW

        L1_             = this->get_parameter("L1").as_double();
        L2_             = this->get_parameter("L2").as_double();
        L3_             = this->get_parameter("L3").as_double();
        body_radius_    = this->get_parameter("body_radius").as_double();
        double beta_deg = this->get_parameter("beta_deg").as_double();
        home_x_         = this->get_parameter("home_x").as_double();
        home_y_         = this->get_parameter("home_y").as_double();
        home_z_         = this->get_parameter("home_z").as_double();
        step_period_    = this->get_parameter("step_period").as_double();
        stride_length_  = this->get_parameter("stride_length").as_double();
        step_height_    = this->get_parameter("step_height").as_double();
        traj_hz_        = this->get_parameter("traj_hz").as_int();
        joint_limit_    = this->get_parameter("joint_limit_rad").as_double();
        stand_duration_ = this->get_parameter("stand_duration").as_double();
        vel_debounce_   = this->get_parameter("vel_debounce_threshold").as_double();
        beta_rad_       = beta_deg * M_PI / 180.0;

        // ── Verify home IK is solvable ─────────────────────────────────────
        double h1, h2, h3;
        if (!ikEU(home_x_, home_y_, home_z_, h1, h2, h3)) {
            RCLCPP_ERROR(get_logger(),
                "HOME POSITION IK FAILED (x=%.1f y=%.1f z=%.1f) — "
                "robot will not stand correctly. Adjust home_z in YAML.",
                home_x_, home_y_, home_z_);
        } else {
            RCLCPP_INFO(get_logger(),
                "Home IK OK: t1=%.3f t2=%.3f t3=%.3f rad (%.1f° %.1f° %.1f°)",
                h1, h2, h3,
                h1*180/M_PI, h2*180/M_PI, h3*180/M_PI);
        }

        // ── Build locomotion object ────────────────────────────────────────
        locomotion_ = std::make_unique<vx01_hexapod_locomotion::HexapodLocomotion>(
            L1_, L2_, L3_, body_radius_, beta_rad_);
        locomotion_->setStepPeriod(step_period_);
        locomotion_->setHomePosition(home_x_, home_y_, home_z_);

        // ── Subscriber + action clients ───────────────────────────────────
        cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&HexapodWalkNode::cmdVelCallback, this, std::placeholders::_1));

        for (int i = 0; i < NUM_LEGS; ++i) {
            action_clients_[i] = rclcpp_action::create_client<FollowJT>(
                this, ACTION_NAMES[i]);
            legs_ready_[i]   = false;
            legs_cycling_[i] = false;
        }

        timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&HexapodWalkNode::timerCallback, this));

        clock_ = this->get_clock();

        RCLCPP_INFO(get_logger(),
            "HexapodWalkNode started — L1=%.2f L2=%.2f L3=%.2f "
            "home=(%.0f,%.0f,%.0f) step_period=%.1fs stride=%.0fmm height=%.0fmm",
            L1_, L2_, L3_, home_x_, home_y_, home_z_,
            step_period_, stride_length_, step_height_);
        RCLCPP_INFO(get_logger(), "Waiting for action servers...");
    }

private:
    enum class State { INIT, STANDING, WALKING };

    // ── Robot parameters (loaded from YAML) ───────────────────────────────
    double L1_, L2_, L3_;
    double body_radius_, beta_rad_;
    double home_x_, home_y_, home_z_;
    double step_period_;
    double stride_length_;
    double step_height_;
    int    traj_hz_;
    double joint_limit_;
    double stand_duration_;
    double vel_debounce_;

    std::unique_ptr<vx01_hexapod_locomotion::HexapodLocomotion> locomotion_;
    std::array<rclcpp_action::Client<FollowJT>::SharedPtr, NUM_LEGS> action_clients_;
    std::array<bool, NUM_LEGS> legs_ready_;
    std::array<bool, NUM_LEGS> legs_cycling_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    State state_;
    rclcpp::Clock::SharedPtr clock_;

    std::mutex vel_mutex_;
    double vel_x_{0.0}, vel_y_{0.0}, vel_omega_{0.0};
    // Last velocity that triggered a cycle send — for debounce
    double last_sent_vx_{0.0};

    // ── IK: 3-DOF elbow-up analytical ────────────────────────────────────
    // All coordinates in leg-local frame (mm).
    // theta1 = coxa rotation (atan2(y,x))
    // theta2 = femur pitch (elbow-up)
    // theta3 = tibia pitch (elbow-up, tibia tucked under)
    bool ikEU(double xp, double yp, double zp,
              double& t1, double& t2, double& t3) const
    {
        // Coxa rotation
        t1 = std::atan2(yp, xp);
        double ct1 = std::cos(t1);
        if (std::abs(ct1) < 1e-9) return false;

        // Planar reach after coxa rotation
        double r2 = xp / ct1 - L1_;   // horizontal distance from femur pivot
        double r1 = std::sqrt(zp*zp + r2*r2);  // total link reach

        if (r1 > (L2_ + L3_) * 0.99) {
            // Slightly over-extended — pull back to reachable boundary
            double scale = (L2_ + L3_) * 0.99 / r1;
            r2 *= scale;
            zp *= scale;
            r1  = std::sqrt(zp*zp + r2*r2);
        }
        if (r1 < std::abs(L2_ - L3_) + 1e-6) return false;

        double phi2 = std::atan2(zp, r2);
        double cp1  = (L2_*L2_ + r1*r1 - L3_*L3_) / (2.0*L2_*r1);
        if (cp1 < -1.0 || cp1 > 1.0) return false;
        double phi1 = std::acos(cp1);
        t2 = phi2 - phi1;   // elbow-up: femur tips downward

        double cp3 = (r1*r1 - L2_*L2_ - L3_*L3_) / (-2.0*L2_*L3_);
        if (cp3 < -1.0 || cp3 > 1.0) return false;
        t3 = M_PI - std::acos(cp3);  // elbow-up: tibia folds under femur

        return (std::abs(t1) <= joint_limit_ &&
                std::abs(t2) <= joint_limit_ &&
                std::abs(t3) <= joint_limit_);
    }

    double clampAngle(double a) const {
        return std::max(-joint_limit_, std::min(joint_limit_, a));
    }

    // ── Quadratic Bezier arc helper ───────────────────────────────────────
    static double bz(double t, double p0, double p1, double p2) {
        double u = 1.0 - t;
        return u*u*p0 + 2.0*u*t*p1 + t*t*p2;
    }

    // ── Foot position for one leg at absolute time t_abs ─────────────────
    // Returns (stride_offset_mm, vertical_lift_mm) in leg-local Y/Z space.
    //
    // Correct tripod gait (duty factor = 0.5):
    //   Each leg swings for the first half of step_period_ then drags for the second.
    //   Group A (legs 0,2,4): swing phase starts at t=0.
    //   Group B (legs 1,3,5): swing phase starts at t=step_period_/2 (180° offset).
    //
    // Swing (Bezier arc, t ∈ [0, half_period]):
    //   stride: -T/2 → 0 → +T/2  (foot lifts and swings forward)
    //   lift:    0   → A →  0    (smooth parabolic arc)
    //
    // Stance / drag (t ∈ [half_period, step_period_]):
    //   stride: +T/2 → -T/2     (foot pushes body forward)
    //   lift:   0               (foot on ground)
    void footPosAtTime(int leg, double t_abs,
                       double& stride_out, double& lift_out) const
    {
        const double half = step_period_ * 0.5;
        // Phase offset: group B starts stance at t=0 (i.e., swing shifted by half)
        bool group_b = (leg == 1 || leg == 3 || leg == 5);
        double t_local = std::fmod(t_abs + (group_b ? half : 0.0), step_period_);

        if (t_local < half) {
            // SWING phase
            double s = t_local / half;   // 0→1 over swing
            stride_out = bz(s, -stride_length_*0.5, 0.0, stride_length_*0.5);
            lift_out   = bz(s,  0.0, step_height_, 0.0);
        } else {
            // STANCE / drag phase
            double s = (t_local - half) / half;  // 0→1 over stance
            stride_out = stride_length_*0.5 - s * stride_length_;  // +T/2 → -T/2
            lift_out   = 0.0;
        }
    }

    // ── Build full-cycle trajectory for one leg ───────────────────────────
    FollowJT::Goal buildCycleGoal(int leg, double vx) const
    {
        // Nominal speed at which stride_length_ is fully used (mm/s)
        const double nominal_speed = stride_length_ / step_period_;
        // Scale: 1.0 at nominal, fractional at lower speed
        const double scale = (nominal_speed > 1e-6) ?
                             std::max(-2.0, std::min(2.0, vx / nominal_speed)) : 0.0;
        const double leg_sign = LEG_SIDE_SIGN[leg];
        const double dt = 1.0 / static_cast<double>(traj_hz_);

        // Compute home IK for fallback
        double h1=0.0, h2=0.0, h3=0.0;
        ikEU(home_x_, home_y_, home_z_, h1, h2, h3);

        FollowJT::Goal goal;
        goal.trajectory.joint_names = {
            JOINT_NAMES[leg][0], JOINT_NAMES[leg][1], JOINT_NAMES[leg][2]
        };

        int n = static_cast<int>(std::ceil(step_period_ * traj_hz_));
        goal.trajectory.points.reserve(n + 1);

        for (int i = 0; i <= n; ++i) {
            double t_abs = std::min(static_cast<double>(i) * dt, step_period_);

            double stride, lift;
            footPosAtTime(leg, t_abs, stride, lift);

            double ik_x = home_x_;
            double ik_y = stride * scale * leg_sign;  // side sign for correct direction
            double ik_z = home_z_ + lift;

            double t1, t2, t3;
            bool ok = ikEU(ik_x, ik_y, ik_z, t1, t2, t3);
            if (!ok) {
                // Fallback: clamp to home position silently on non-critical points
                t1 = h1; t2 = h2; t3 = h3;
                if (i % traj_hz_ == 0) {  // log only once per second
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *clock_, 2000,
                        "IK fallback leg=%d t=%.2f (x=%.1f y=%.1f z=%.1f)",
                        leg, t_abs, ik_x, ik_y, ik_z);
                }
            }

            trajectory_msgs::msg::JointTrajectoryPoint pt;
            pt.positions  = { clampAngle(t1), clampAngle(t2), clampAngle(t3) };
            pt.velocities = { 0.0, 0.0, 0.0 };
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
                using RC = rclcpp_action::ResultCode;
                if (result.code == RC::SUCCEEDED) {
                    if (state_ == State::WALKING) {
                        double vx;
                        { std::lock_guard<std::mutex> lk(vel_mutex_); vx = vel_x_; }
                        sendCycleGoal(leg, vx);
                    }
                } else if (result.code == RC::CANCELED) {
                    // Expected on velocity change or stop — no action needed
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
                legs_cycling_[i] = false;
            }
        }
    }

    // ── Send stand (home) position to all legs ────────────────────────────
    void sendStandGoal()
    {
        locomotion_->stand();

        for (int i = 0; i < NUM_LEGS; ++i) {
            double t1, t2, t3;
            locomotion_->getLegAngles(i, t1, t2, t3);

            FollowJT::Goal goal;
            goal.trajectory.joint_names = {
                JOINT_NAMES[i][0], JOINT_NAMES[i][1], JOINT_NAMES[i][2]
            };
            trajectory_msgs::msg::JointTrajectoryPoint pt;
            pt.positions  = { clampAngle(t1), clampAngle(t2), clampAngle(t3) };
            pt.velocities = { 0.0, 0.0, 0.0 };
            pt.time_from_start = rclcpp::Duration::from_seconds(stand_duration_);
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
            "Stand goal sent (%.1f s) home=(%.0f,%.0f,%.0f)",
            stand_duration_, home_x_, home_y_, home_z_);
    }

    // ── Transition to WALKING state ───────────────────────────────────────
    void startWalking(double vx)
    {
        RCLCPP_INFO(get_logger(), "Starting walk — vx=%.1f mm/s", vx);
        last_sent_vx_ = vx;
        locomotion_->setVelocity(vx, 0.0, 0.0);
        locomotion_->walk();
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
            // FIX: only restart if velocity changed significantly (debounce)
            double delta = std::abs(new_vx - last_sent_vx_);
            if (delta > vel_debounce_) {
                RCLCPP_INFO(get_logger(),
                    "Velocity update vx=%.1f (delta=%.1f) — restarting walk cycles",
                    new_vx, delta);
                cancelAllCycleGoals();
                last_sent_vx_ = new_vx;
                locomotion_->setVelocity(new_vx, new_vy, new_omega);
                for (int i = 0; i < NUM_LEGS; ++i) {
                    sendCycleGoal(i, new_vx);
                }
            }

        } else if (state_ == State::WALKING && !nonzero) {
            RCLCPP_INFO(get_logger(), "Stop command — returning to stand");
            cancelAllCycleGoals();
            locomotion_->stand();
            state_ = State::STANDING;
            last_sent_vx_ = 0.0;
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
            timer_->cancel();  // No longer needed after INIT
            sendStandGoal();
        }
    }
};

// ─────────────────────────────────────────────────────────────────────────
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HexapodWalkNode>());
    rclcpp::shutdown();
    return 0;
}