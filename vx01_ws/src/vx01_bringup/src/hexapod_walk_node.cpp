/**
 * hexapod_walk_node.cpp  —  VX01 Hexapod  (ZERO IK / AUTO-WALK)
 *
 * ── Geometry (L1=60.55  L2=73.84  L3=112.16 mm) ──────────────────────────────
 *
 *   TIBIA_FIXED = -0.6981 rad (-40 deg)  → tibia tucked under femur, natural look
 *   FEMUR_STAND = +0.1222 rad (+7  deg)  → foot_z = -52 mm  (body raised)
 *   FEMUR_LIFT  = +0.2094 rad (+12 deg)  → foot_z = -37 mm  (14mm swing lift)
 *   TOTAL_REACH = 227.9 mm               (foot X at stand — used for coxa atan2)
 *
 *   All joints well within ±0.785398 rad (±45 deg) hardware limit.
 *
 * ── Direct joint angles — NO IK ──────────────────────────────────────────────
 *
 *   coxa  = atan2(stride_y, TOTAL_REACH)   max ±10 deg at stride=80mm ✓
 *   femur = blend(FEMUR_STAND → FEMUR_LIFT, lift_frac)
 *   tibia = TIBIA_FIXED always
 *
 * ── Startup — AUTO WALK, no cmd_vel needed ────────────────────────────────────
 *   1. Wait for all 6 JTC action servers
 *   2. STAND: smooth move to stand pose over 2s
 *   3. WALK:  auto-starts tripod gait immediately after stand
 *
 * ── Tripod gait ───────────────────────────────────────────────────────────────
 *   Group A (legs 0,2,4): SWING first half,  STANCE second half
 *   Group B (legs 1,3,5): STANCE first half, SWING  second half
 *   3 legs always grounded — stable tripod at every instant.
 *   Each leg sends its own JTC goal and auto-loops on completion → zero gap.
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

    // ── Geometry-derived constants (verified, within joint limits) ─────────
    //
    //  TIBIA_FIXED = -40 deg: tibia folds neatly under femur (natural tucked look)
    //  FEMUR_STAND = +7  deg: body ~52mm above ground, stable standing height
    //  FEMUR_LIFT  = +12 deg: ~14mm foot lift at swing apex — enough to clear ground
    //  TOTAL_REACH = 227.9mm: used in coxa = atan2(stride_y, TOTAL_REACH)
    //
    static constexpr double TIBIA_FIXED  = -0.6981;  // rad = -40 deg
    static constexpr double FEMUR_STAND  =  0.1222;  // rad = +7  deg → z=-52mm
    static constexpr double FEMUR_LIFT   =  0.2094;  // rad = +12 deg → z=-37mm
    static constexpr double TOTAL_REACH  =  227.9;   // mm
    static constexpr double JOINT_LIMIT  =  0.7854;  // rad = ±45 deg

    // ── Default walk speed ─────────────────────────────────────────────────
    static constexpr double DEFAULT_VX   = 50.0;     // mm/s forward walk speed

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
        declare_parameter("step_period",    2.0);   // gait cycle time (s) — shorter = faster
        declare_parameter("stride_length", 80.0);   // total stride Y (mm)
        declare_parameter("traj_hz",        50);    // waypoints per second
        declare_parameter("stand_duration",  2.0);  // stand-up time (s)
        declare_parameter("walk_speed",    DEFAULT_VX); // default fwd speed mm/s

        step_period_    = get_parameter("step_period").as_double();
        stride_length_  = get_parameter("stride_length").as_double();
        traj_hz_        = get_parameter("traj_hz").as_int();
        stand_duration_ = get_parameter("stand_duration").as_double();

        double walk_speed = get_parameter("walk_speed").as_double();

        { std::lock_guard<std::mutex> lk(vel_mutex_); vel_x_ = walk_speed; }

        clock_ = get_clock();

        // cmd_vel still accepted — overrides the auto-walk speed
        cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&HexapodWalkNode::cmdVelCallback, this, std::placeholders::_1));

        for (int i = 0; i < NUM_LEGS; ++i) {
            action_clients_[i] = rclcpp_action::create_client<FollowJT>(this, ACTION_NAMES[i]);
            legs_ready_[i]     = false;
            legs_cycling_[i]   = false;
        }

        // Poll for action server readiness at 10 Hz
        timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&HexapodWalkNode::timerCallback, this));

        RCLCPP_INFO(get_logger(), "================================================");
        RCLCPP_INFO(get_logger(), " HexapodWalkNode  —  AUTO-WALK / ZERO IK");
        RCLCPP_INFO(get_logger(), "  TIBIA_FIXED = %+.4f rad (%+.1f deg)  [tucked]",
                    TIBIA_FIXED, TIBIA_FIXED*180.0/M_PI);
        RCLCPP_INFO(get_logger(), "  FEMUR_STAND = %+.4f rad  foot_z = -52mm",  FEMUR_STAND);
        RCLCPP_INFO(get_logger(), "  FEMUR_LIFT  = %+.4f rad  foot_z = -37mm  (14mm lift)", FEMUR_LIFT);
        RCLCPP_INFO(get_logger(), "  TOTAL_REACH = %.1f mm", TOTAL_REACH);
        RCLCPP_INFO(get_logger(), "  period=%.1fs  stride=%.0fmm  hz=%d  auto_vx=%.1fmm/s",
                    step_period_, stride_length_, traj_hz_, walk_speed);
        RCLCPP_INFO(get_logger(), "================================================");
    }

private:
    enum class State { INIT, STANDING, WALKING };

    double step_period_;
    double stride_length_;
    int    traj_hz_;
    double stand_duration_;

    std::array<rclcpp_action::Client<FollowJT>::SharedPtr, NUM_LEGS> action_clients_;
    std::array<bool, NUM_LEGS> legs_ready_;
    std::array<bool, NUM_LEGS> legs_cycling_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Clock::SharedPtr clock_;

    State  state_;
    std::mutex vel_mutex_;
    double vel_x_{DEFAULT_VX};
    double last_sent_vx_{0.0};

    // ── Clamp to joint limit ───────────────────────────────────────────────
    static double clamp(double v) {
        return std::max(-JOINT_LIMIT, std::min(JOINT_LIMIT, v));
    }

    // ── Quadratic Bezier ──────────────────────────────────────────────────
    static double bz(double t, double p0, double p1, double p2) {
        double u = 1.0 - t;
        return u*u*p0 + 2.0*u*t*p1 + t*t*p2;
    }

    // ── Direct joint angle computation — no IK ────────────────────────────
    //
    //  stride_y  [mm]: foot position in stride direction
    //                  + = forward,  − = backward
    //  lift_frac [0..1]: 0 = foot on ground, 1 = full swing apex
    //
    //  coxa  = atan2(stride_y, TOTAL_REACH)
    //          → sweeps foot forward/backward, max ±10 deg at stride=80mm
    //  femur = FEMUR_STAND + lift_frac × (FEMUR_LIFT − FEMUR_STAND)
    //          → raises foot during swing
    //  tibia = TIBIA_FIXED (always, damaged servos)
    //
    static void directAngles(double stride_y, double lift_frac,
                              double& coxa_out, double& femur_out)
    {
        coxa_out  = std::atan2(stride_y, TOTAL_REACH);
        femur_out = FEMUR_STAND + lift_frac * (FEMUR_LIFT - FEMUR_STAND);
    }

    // ── Gait phase at absolute time ───────────────────────────────────────
    //
    // Tripod gait, duty = 0.5:
    //   Group A (0,2,4): SWING [0..half),  STANCE [half..T)
    //   Group B (1,3,5): offset by half → antiphase, always 3 legs on ground
    //
    // SWING  — Bezier arc (foot in air, moves forward):
    //   stride_y  = Bezier(−S, 0, +S)
    //   lift_frac = Bezier(0,  1,  0)
    //
    // STANCE — linear push (foot on ground, body moves forward):
    //   stride_y  = +S → −S  (linear)
    //   lift_frac = 0
    //
    void legPhaseAtTime(int leg, double t_abs, double vx_scale,
                        double& stride_y_out, double& lift_frac_out) const
    {
        const double half  = step_period_ * 0.5;
        const bool group_b = (leg == 1 || leg == 3 || leg == 5);
        double t_loc = std::fmod(t_abs + (group_b ? half : 0.0), step_period_);

        const double S = (stride_length_ * 0.5) * vx_scale;  // half-stride mm

        if (t_loc < half) {
            // SWING
            const double s = t_loc / half;
            stride_y_out  = bz(s, -S,  0.0, +S);
            lift_frac_out = bz(s, 0.0, 1.0, 0.0);
        } else {
            // STANCE — linear
            const double s = (t_loc - half) / half;
            stride_y_out  = +S - s * (2.0 * S);
            lift_frac_out = 0.0;
        }
    }

    // ── Build one full gait-cycle JTC goal (direct angles, zero IK) ───────
    FollowJT::Goal buildCycleGoal(int leg, double vx) const
    {
        const double nom_spd  = (step_period_ > 1e-9)
                                ? stride_length_ / (step_period_ * 0.5) : 1.0;
        const double vx_scale = (nom_spd > 1e-6)
                                ? std::max(-1.5, std::min(1.5, vx / nom_spd)) : 0.0;

        const double dt = 1.0 / static_cast<double>(traj_hz_);
        const int    n  = static_cast<int>(std::ceil(step_period_ * traj_hz_));

        FollowJT::Goal goal;
        goal.trajectory.joint_names = {
            JOINT_NAMES[leg][0], JOINT_NAMES[leg][1], JOINT_NAMES[leg][2]
        };
        goal.trajectory.points.reserve(n + 1);

        for (int i = 0; i <= n; ++i) {
            const double t_abs = std::min(static_cast<double>(i) * dt, step_period_);

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
                        sendCycleGoal(leg, vx);  // ← seamless zero-gap loop
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
    // 2-point trajectory: sim start (femur=0) → FEMUR_STAND over stand_duration_.
    // After completion → immediately auto-starts walking (no cmd_vel needed).
    //
    void sendStandGoal()
    {
        RCLCPP_INFO(get_logger(),
            "STAND: femur 0.0 → %.4f rad over %.1fs",
            FEMUR_STAND, stand_duration_);

        for (int i = 0; i < NUM_LEGS; ++i) {
            FollowJT::Goal goal;
            goal.trajectory.joint_names = {
                JOINT_NAMES[i][0], JOINT_NAMES[i][1], JOINT_NAMES[i][2]
            };

            // pt0 — sim start (all joints = initial_positions.yaml values)
            trajectory_msgs::msg::JointTrajectoryPoint pt0;
            pt0.positions  = { 0.0, 0.0, TIBIA_FIXED };
            pt0.velocities = { 0.0, 0.0, 0.0 };
            pt0.time_from_start = rclcpp::Duration::from_seconds(0.0);
            goal.trajectory.points.push_back(pt0);

            // pt1 — standing pose
            trajectory_msgs::msg::JointTrajectoryPoint pt1;
            pt1.positions  = { 0.0, FEMUR_STAND, TIBIA_FIXED };
            pt1.velocities = { 0.0, 0.0, 0.0 };
            pt1.time_from_start = rclcpp::Duration::from_seconds(stand_duration_);
            goal.trajectory.points.push_back(pt1);

            auto opts = rclcpp_action::Client<FollowJT>::SendGoalOptions();

            // Monitor leg 5 only — when it completes, all 6 are at stand pose
            if (i == NUM_LEGS - 1) {
                opts.result_callback =
                    [this](const GoalHandleFJT::WrappedResult& result) {
                        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                            RCLCPP_INFO(get_logger(),
                                "Standing complete — AUTO-STARTING tripod walk!");
                            double vx;
                            { std::lock_guard<std::mutex> lk(vel_mutex_); vx = vel_x_; }
                            startWalking(vx);
                        } else {
                            RCLCPP_WARN(get_logger(),
                                "Stand ended code=%d — retrying",
                                static_cast<int>(result.code));
                            sendStandGoal();
                        }
                    };
            }

            action_clients_[i]->async_send_goal(goal, opts);
        }
    }

    // ── STAGE 2: WALK (auto-started after stand) ──────────────────────────
    void startWalking(double vx)
    {
        RCLCPP_INFO(get_logger(),
            "WALKING — vx=%.1f mm/s  coxa_max=±%.1f deg  period=%.1fs",
            vx,
            std::abs(std::atan2(stride_length_ * 0.5, TOTAL_REACH)) * 180.0 / M_PI,
            step_period_);
        last_sent_vx_ = vx;
        state_ = State::WALKING;
        for (int i = 0; i < NUM_LEGS; ++i) {
            sendCycleGoal(i, vx);
        }
    }

    // ── Return to stand ───────────────────────────────────────────────────
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
            pt.time_from_start = rclcpp::Duration::from_seconds(1.0);
            goal.trajectory.points.push_back(pt);
            action_clients_[i]->async_send_goal(goal, {});
        }
    }

    // ── /cmd_vel — optional override of auto-walk speed ───────────────────
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        const double new_vx    = msg->linear.x * 1000.0;  // m/s → mm/s
        const double new_vy    = msg->linear.y * 1000.0;
        const double new_omega = msg->angular.z;

        { std::lock_guard<std::mutex> lk(vel_mutex_);
          vel_x_ = new_vx; }

        const bool nonzero = (std::abs(new_vx)    > 1.0  ||
                              std::abs(new_vy)    > 1.0  ||
                              std::abs(new_omega) > 0.01);

        if (state_ == State::WALKING) {
            if (nonzero) {
                const double delta = std::abs(new_vx - last_sent_vx_);
                if (delta > 5.0) {
                    RCLCPP_INFO(get_logger(),
                        "Speed %.1f → %.1f mm/s — replanning",
                        last_sent_vx_, new_vx);
                    cancelAllCycleGoals();
                    last_sent_vx_ = new_vx;
                    for (int i = 0; i < NUM_LEGS; ++i) sendCycleGoal(i, new_vx);
                }
            } else {
                // Zero cmd_vel → stop and stand
                RCLCPP_INFO(get_logger(), "cmd_vel=0 — stopping, returning to stand");
                cancelAllCycleGoals();
                state_ = State::STANDING;
                last_sent_vx_ = 0.0;
                sendReturnToStand();
            }
        } else if (state_ == State::STANDING && nonzero) {
            // Resume walking from stand
            startWalking(new_vx);
        }
    }

    // ── Timer: poll servers → stand → auto-walk ───────────────────────────
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
                "All 6 servers ready — standing up then auto-walking");
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