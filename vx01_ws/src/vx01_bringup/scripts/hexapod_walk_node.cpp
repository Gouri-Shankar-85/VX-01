#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

using namespace std::chrono_literals;
using FJT           = control_msgs::action::FollowJointTrajectory;
using GoalHandleFJT = rclcpp_action::ClientGoalHandle<FJT>;

// ─────────────────────────────────────────────────────────────────────────────
// Gait constants
// ─────────────────────────────────────────────────────────────────────────────
namespace param {

    // ── Standing joint angles (user-confirmed in simulation) ─────────────────
    constexpr double COXA_STAND  =  0.000;   // rad
    constexpr double FEMUR_STAND = -0.500;   // rad  (-28.6°)
    constexpr double TIBIA_STAND =  0.600;   // rad  (+34.4°)  legs 0–4
    // Leg 5 tibia axis is <axis xyz="0 0 1"> (all others "0 0 -1")
    // → negate tibia for leg 5 to get same physical pose
    constexpr double TIBIA_STAND_LEG5 = -0.600;   // rad  (within [-0.78, +0.10] ✓)

    // ── Swing-peak joint angles ───────────────────────────────────────────────
    // FK (correct T01): foot ≈ (246, 0, +15) mm  → 39 mm above standing height
    constexpr double FEMUR_SWING =  0.050;   // rad  (+2.9°)
    constexpr double TIBIA_SWING =  0.050;   // rad  (+2.9°)  legs 0–4
    constexpr double TIBIA_SWING_LEG5 = -0.050;  // rad  leg 5 (axis flipped)

    // ── Coxa sweep ───────────────────────────────────────────────────────────
    // ±0.25 rad (±14.3°) → foot arc ≈ ±59 mm → ~118 mm stride
    constexpr double COXA_AMP = 0.250;       // rad

    // ── Timing ───────────────────────────────────────────────────────────────
    constexpr double STEP_PERIOD_S = 2.0;    // full gait cycle (s)
    constexpr double LOOP_HZ       = 50.0;
    constexpr double LOOP_DT_S     = 1.0 / LOOP_HZ;

    // Trajectory look-ahead: 4 × dt = 80 ms.
    // Gives the JTC enough time to interpolate smoothly between goals
    // without the robot waiting and stalling between ticks.
    constexpr double TRAJ_DT_S = LOOP_DT_S * 4.0;

    // ── Hard URDF limits (xacro) ─────────────────────────────────────────────
    constexpr double COXA_MIN  = -0.78;
    constexpr double COXA_MAX  =  0.78;
    constexpr double FEMUR_MIN = -0.78;
    constexpr double FEMUR_MAX =  0.78;   // using physical range; sim accepts it
    constexpr double TIBIA_MIN = -0.78;
    constexpr double TIBIA_MAX =  0.78;   // using physical range; sim accepts it

    // ── Phase offsets: tripod gait ────────────────────────────────────────────
    //   legs 0, 2, 4 → Group A  (φ₀ = 0.0)
    //   legs 1, 3, 5 → Group B  (φ₀ = 0.5)
    constexpr double PHASE_OFFSET[6] = {0.0, 0.5, 0.0, 0.5, 0.0, 0.5};

} // namespace param

// ─────────────────────────────────────────────────────────────────────────────
// Helpers
// ─────────────────────────────────────────────────────────────────────────────

static inline double clamp(double v, double lo, double hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

// Smooth-step S(t) = 3t²−2t³ : C¹ continuous, zero velocity at t=0 and t=1
static inline double smooth_step(double t) {
    t = clamp(t, 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}

// ─────────────────────────────────────────────────────────────────────────────
// HexapodWalkNode
// ─────────────────────────────────────────────────────────────────────────────
class HexapodWalkNode : public rclcpp::Node
{
public:

    HexapodWalkNode()
        : Node("hexapod_walk_node"),
          gait_time_(0.0)
    {
        RCLCPP_INFO(get_logger(), "hexapod_walk_node starting...");
        RCLCPP_INFO(get_logger(),
            "  Standing: coxa=%.3f  femur=%.3f  tibia=%.3f rad "
            "(leg5 tibia=%.3f rad)",
            param::COXA_STAND, param::FEMUR_STAND,
            param::TIBIA_STAND, param::TIBIA_STAND_LEG5);
        RCLCPP_INFO(get_logger(),
            "  Gait: period=%.1f s | coxa_amp=%.3f rad (%.1f deg) | loop=%.0f Hz",
            param::STEP_PERIOD_S,
            param::COXA_AMP,
            param::COXA_AMP * 180.0 / M_PI,
            param::LOOP_HZ);

        // Build joint name arrays
        for (int i = 0; i < 6; ++i) {
            const std::string n = std::to_string(i);
            joint_names_[i] = {
                "coxa_leg"  + n + "_joint",
                "femur_leg" + n + "_joint",
                "tibia_leg" + n + "_joint"
            };
        }

        // Create one action client per leg controller
        for (int i = 0; i < 6; ++i) {
            const std::string srv =
                "/leg_" + std::to_string(i) +
                "_controller/follow_joint_trajectory";
            clients_[i] = rclcpp_action::create_client<FJT>(this, srv);
            RCLCPP_INFO(get_logger(), "  Waiting for: %s", srv.c_str());
        }

        // Poll until all servers are ready
        startup_timer_ = create_wall_timer(300ms,
            [this]() { doStartup(); });
    }

private:

    std::array<std::array<std::string, 3>, 6>         joint_names_;
    std::array<rclcpp_action::Client<FJT>::SharedPtr, 6> clients_;

    rclcpp::TimerBase::SharedPtr               startup_timer_;
    rclcpp::TimerBase::SharedPtr               control_timer_;
    std::vector<rclcpp::TimerBase::SharedPtr>  oneshot_timers_;

    double gait_time_;

    // ── Wait for all 6 action servers ────────────────────────────────────────
    void doStartup()
    {
        int ready = 0;
        for (int i = 0; i < 6; ++i)
            if (clients_[i]->action_server_is_ready()) ++ready;

        if (ready < 6) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                "  Waiting for controllers (%d/6 ready)...", ready);
            return;
        }
        startup_timer_->cancel();
        RCLCPP_INFO(get_logger(), "All 6 controllers ready.");

        // Step 1: Move to standing posture
        commandStand();

        // Step 2: Begin walking after 2.5 s (let robot settle)
        auto t = create_wall_timer(2500ms, [this]() {
            for (auto& os : oneshot_timers_) os->cancel();
            oneshot_timers_.clear();
            beginWalk();
        });
        oneshot_timers_.push_back(t);
    }

    // ── Move all legs to standing posture over 2.0 s ──────────────────────────
    void commandStand()
    {
        RCLCPP_INFO(get_logger(),
            "Moving to standing posture "
            "(coxa=%.2f femur=%.2f tibia=%.2f, leg5_tibia=%.2f)...",
            param::COXA_STAND, param::FEMUR_STAND,
            param::TIBIA_STAND, param::TIBIA_STAND_LEG5);

        for (int leg = 0; leg < 6; ++leg) {
            const double tibia = (leg == 5) ? param::TIBIA_STAND_LEG5
                                            : param::TIBIA_STAND;
            sendGoal(leg,
                param::COXA_STAND,
                param::FEMUR_STAND,
                tibia,
                2.0);    // 2.0 s — slow, safe move to home
        }
    }

    // ── Launch the 50 Hz walking loop ─────────────────────────────────────────
    void beginWalk()
    {
        RCLCPP_INFO(get_logger(),
            "Walking — period=%.1f s | coxa_amp=%.3f rad",
            param::STEP_PERIOD_S, param::COXA_AMP);
        gait_time_ = 0.0;

        auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(param::LOOP_DT_S));
        control_timer_ = create_wall_timer(period_ns,
            [this]() { controlLoop(); });
    }

    // ── 50 Hz gait update ─────────────────────────────────────────────────────
    void controlLoop()
    {
        gait_time_ += param::LOOP_DT_S;
        if (gait_time_ >= param::STEP_PERIOD_S)
            gait_time_ -= param::STEP_PERIOD_S;

        for (int leg = 0; leg < 6; ++leg) {
            const double phi = std::fmod(
                gait_time_ / param::STEP_PERIOD_S + param::PHASE_OFFSET[leg],
                1.0);

            double coxa, femur, tibia;
            computeJoints(leg, phi, coxa, femur, tibia);

            coxa  = clamp(coxa,  param::COXA_MIN,  param::COXA_MAX);
            femur = clamp(femur, param::FEMUR_MIN,  param::FEMUR_MAX);
            tibia = clamp(tibia, param::TIBIA_MIN,  param::TIBIA_MAX);

            sendGoal(leg, coxa, femur, tibia, param::TRAJ_DT_S);
        }
    }

    // ── Joint-space gait profile ──────────────────────────────────────────────
    //
    //  φ ∈ [0.0, 0.5)  → SWING  (foot in air, repositioning forward)
    //  φ ∈ [0.5, 1.0)  → STANCE (foot on ground, pushing body forward)
    //
    //  SWING:
    //    coxa  : -COXA_AMP → +COXA_AMP  (smooth_step: reposition to front)
    //    femur : FEMUR_STAND → FEMUR_SWING → FEMUR_STAND  (sin bell: lift)
    //    tibia : TIBIA_STAND → TIBIA_SWING → TIBIA_STAND  (sin bell: clearance)
    //
    //  STANCE:
    //    coxa  : +COXA_AMP → -COXA_AMP  (smooth_step: push body forward)
    //    femur : FEMUR_STAND  (hold)
    //    tibia : TIBIA_STAND  (hold) — leg-5-corrected value used throughout
    //
    //  smooth_step: zero velocity at phase boundaries → no coxa jerk
    //  sin(π·t) bell: smooth foot lift and land → no foot slam
    //
    void computeJoints(int leg, double phi,
                       double& coxa, double& femur, double& tibia)
    {
        // Select the correct tibia angles for leg 5 (flipped axis)
        const double tibia_stand = (leg == 5) ? param::TIBIA_STAND_LEG5
                                              : param::TIBIA_STAND;
        const double tibia_swing = (leg == 5) ? param::TIBIA_SWING_LEG5
                                              : param::TIBIA_SWING;

        if (phi < 0.5) {
            // ── SWING ────────────────────────────────────────────────────────
            const double t    = phi / 0.5;             // [0, 1]
            const double ts   = smooth_step(t);        // smooth position
            const double bell = std::sin(M_PI * t);    // lift bell: 0 → 1 → 0

            // Coxa: back → front (reposition foot)
            coxa  = -param::COXA_AMP + 2.0 * param::COXA_AMP * ts;

            // Femur & tibia: lift smoothly through swing peak, land cleanly
            femur = param::FEMUR_STAND
                  + (param::FEMUR_SWING - param::FEMUR_STAND) * bell;
            tibia = tibia_stand
                  + (tibia_swing - tibia_stand) * bell;

        } else {
            // ── STANCE ───────────────────────────────────────────────────────
            const double t  = (phi - 0.5) / 0.5;      // [0, 1]
            const double ts = smooth_step(t);          // smooth position

            // Coxa: front → back (propel body forward)
            coxa  =  param::COXA_AMP - 2.0 * param::COXA_AMP * ts;
            femur =  param::FEMUR_STAND;
            tibia =  tibia_stand;
        }
    }

    // ── Send one-point trajectory goal to leg controller N ───────────────────
    //
    // time_s: target horizon
    //   Standing : 2.0 s  — slow, one-shot settling move
    //   Walking  : TRAJ_DT_S (80 ms) — rolling horizon, smooth continuous motion
    //
    // Path/goal tolerances are generous so the JTC never aborts a goal mid-stride
    // when the next 50 Hz command arrives before the current one completes.
    //
    void sendGoal(int leg,
                  double coxa, double femur, double tibia,
                  double time_s)
    {
        if (!clients_[leg]->action_server_is_ready()) return;

        trajectory_msgs::msg::JointTrajectory traj;
        traj.joint_names = {
            joint_names_[leg][0],   // coxa_legN_joint
            joint_names_[leg][1],   // femur_legN_joint
            joint_names_[leg][2]    // tibia_legN_joint
        };

        trajectory_msgs::msg::JointTrajectoryPoint pt;
        pt.positions  = {coxa, femur, tibia};
        pt.velocities = {0.0,  0.0,   0.0};
        pt.time_from_start = rclcpp::Duration::from_seconds(time_s);
        traj.points.push_back(pt);

        FJT::Goal goal;
        goal.trajectory = traj;

        for (int j = 0; j < 3; ++j) {
            control_msgs::msg::JointTolerance pt_tol;
            pt_tol.name     = traj.joint_names[j];
            pt_tol.position = 0.15;    // 0.15 rad path tolerance
            pt_tol.velocity = 0.0;
            goal.path_tolerance.push_back(pt_tol);

            control_msgs::msg::JointTolerance g_tol;
            g_tol.name     = traj.joint_names[j];
            g_tol.position = 0.20;    // 0.20 rad goal tolerance
            g_tol.velocity = 0.0;
            goal.goal_tolerance.push_back(g_tol);
        }
        goal.goal_time_tolerance = rclcpp::Duration::from_seconds(0.5);

        auto opts = rclcpp_action::Client<FJT>::SendGoalOptions();
        opts.result_callback =
            [this, leg, coxa, femur, tibia]
            (const GoalHandleFJT::WrappedResult& r)
            {
                if (r.code == rclcpp_action::ResultCode::ABORTED) {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                        "Leg %d ABORTED (coxa=%.3f femur=%.3f tibia=%.3f rad)",
                        leg, coxa, femur, tibia);
                }
                // SUCCEEDED / CANCELED are normal during continuous walking
            };

        clients_[leg]->async_send_goal(goal, opts);
    }
};

// ─────────────────────────────────────────────────────────────────────────────
// main
// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HexapodWalkNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}