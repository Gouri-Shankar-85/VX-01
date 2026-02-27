/**
 * hexapod_walk_node.cpp
 *
 * Hexapod walk node with ALL TIBIA SERVOS FIXED at -0.75 rad.
 * Only coxa and femur joints are driven during walking.
 *
 * Startup sequence:
 *   1. Wait for all 6 leg action servers to become ready.
 *   2. INIT_POSTURE: Move every leg to (coxa=0, femur=-0.785, tibia=-0.75) over
 *      init_pose_duration seconds.  This is a safe "tucked" position.
 *   3. STANDING: Move every leg to the home IK position computed by 2-DOF IK
 *      (tibia stays at TIBIA_FIXED = -0.75).
 *   4. WALKING: On cmd_vel command, run tripod gait with tibia locked.
 *
 * 2-DOF IK:
 *   Because the tibia is fixed, femur+tibia form a virtual rigid link:
 *     Lv    = sqrt(L2^2 + L3^2 + 2*L2*L3*cos(TIBIA_FIXED))
 *     alpha = atan2(L3*sin(TIBIA_FIXED), L2 + L3*cos(TIBIA_FIXED))
 *   Then:  theta1 = atan2(yp, xp)
 *          r      = xp/cos(theta1) - L1
 *          theta2 = atan2(zp, r) - alpha
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

class HexapodWalkNode : public rclcpp::Node
{
public:
    static constexpr int NUM_LEGS = 6;

    // Tibia is FIXED at this angle for ALL operations (damaged servos)
    static constexpr double TIBIA_FIXED = -0.75;  // rad (~-43 deg)

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
        // ── Robot geometry parameters ─────────────────────────────────────
        declare_parameter("L1",            60.55);   // coxa length  (mm)
        declare_parameter("L2",            73.84);   // femur length (mm)
        declare_parameter("L3",           112.16);   // tibia length (mm)
        declare_parameter("body_radius",  100.0);    // coxa pivot from centre (mm)
        declare_parameter("beta_deg",      62.91);   // leg mounting angle (deg)

        // ── Home foot position in leg-local frame ─────────────────────────
        declare_parameter("home_x",        223.0);   // reach distance (mm) — matches stand femur=+0.10 rad
        declare_parameter("home_y",          0.0);   // lateral offset (mm)
        declare_parameter("home_z",        -60.0);   // standing height (mm, negative = down)

        // ── Gait parameters ───────────────────────────────────────────────
        declare_parameter("step_period",     4.0);   // full cycle time (s)
        declare_parameter("stride_length",  80.0);   // total stride length (mm)
        declare_parameter("step_height",    20.0);   // foot lift height (mm)
        declare_parameter("traj_hz",        50);     // trajectory sample rate (Hz)
        declare_parameter("joint_limit_rad", 0.7854);// ±45 deg per joint

        // ── Timing ────────────────────────────────────────────────────────
        declare_parameter("init_pose_duration", 2.0);  // time to reach init posture (s)
        declare_parameter("stand_duration",      4.0);  // time to reach stand pose (longer for tuck->stand) (s)
        declare_parameter("vel_debounce_threshold", 5.0); // mm/s change to trigger re-plan

        // ── Initial posture angles (safe tucked pose, sent first) ────────────
        // coxa=0, femur=-0.785 (~-45 deg), tibia=TIBIA_FIXED=-0.75
        declare_parameter("init_coxa_angle",   0.0);
        declare_parameter("init_femur_angle", -0.785);
        // NOTE: tibia is always forced to TIBIA_FIXED internally

        // ── Stand posture angles (direct joint angles, NO IK) ─────────────
        // stand_femur_angle=+0.10 rad (+5.7 deg): femur angled slightly up so body clears ground by 60mm
        // Formula: foot_z = L2*sin(femur) + L3*sin(femur+tibia) = -60mm at femur=+0.10, tibia=-0.75
        declare_parameter("stand_coxa_angle",  0.0);
        declare_parameter("stand_femur_angle", +0.10);   // +5.7 deg: body raised, foot at (223,0,-60)

        L1_             = get_parameter("L1").as_double();
        L2_             = get_parameter("L2").as_double();
        L3_             = get_parameter("L3").as_double();
        body_radius_    = get_parameter("body_radius").as_double();
        double beta_deg = get_parameter("beta_deg").as_double();
        home_x_         = get_parameter("home_x").as_double();
        home_y_         = get_parameter("home_y").as_double();
        home_z_         = get_parameter("home_z").as_double();
        step_period_    = get_parameter("step_period").as_double();
        stride_length_  = get_parameter("stride_length").as_double();
        step_height_    = get_parameter("step_height").as_double();
        traj_hz_        = get_parameter("traj_hz").as_int();
        joint_limit_    = get_parameter("joint_limit_rad").as_double();
        init_pose_dur_  = get_parameter("init_pose_duration").as_double();
        stand_duration_ = get_parameter("stand_duration").as_double();
        vel_debounce_   = get_parameter("vel_debounce_threshold").as_double();
        init_coxa_          = get_parameter("init_coxa_angle").as_double();
        init_femur_         = get_parameter("init_femur_angle").as_double();
        stand_coxa_angle_   = get_parameter("stand_coxa_angle").as_double();
        stand_femur_angle_  = get_parameter("stand_femur_angle").as_double();
        beta_rad_           = beta_deg * M_PI / 180.0;

        // Precompute virtual-link properties for fixed tibia
        Lv_    = std::sqrt(L2_*L2_ + L3_*L3_ + 2.0*L2_*L3_*std::cos(TIBIA_FIXED));
        alpha_ = std::atan2(L3_*std::sin(TIBIA_FIXED), L2_ + L3_*std::cos(TIBIA_FIXED));

        locomotion_ = std::make_unique<vx01_hexapod_locomotion::HexapodLocomotion>(
            L1_, L2_, L3_, body_radius_, beta_rad_);
        locomotion_->setStepPeriod(step_period_);
        locomotion_->setStepLength(stride_length_);
        locomotion_->setStepHeight(step_height_);
        locomotion_->setHomePosition(home_x_, home_y_, home_z_);

        clock_ = get_clock();

        cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&HexapodWalkNode::cmdVelCallback, this, std::placeholders::_1));

        for (int i = 0; i < NUM_LEGS; ++i) {
            action_clients_[i] = rclcpp_action::create_client<FollowJT>(this, ACTION_NAMES[i]);
            legs_ready_[i]     = false;
            legs_cycling_[i]   = false;
        }

        // Poll for action server readiness
        timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&HexapodWalkNode::timerCallback, this));

        RCLCPP_INFO(get_logger(),
            "HexapodWalkNode started — TIBIA FIXED at %.4f rad", TIBIA_FIXED);
        RCLCPP_INFO(get_logger(),
            "Robot: L1=%.2f L2=%.2f L3=%.2f  Lv=%.2f alpha=%.4f rad",
            L1_, L2_, L3_, Lv_, alpha_);
        RCLCPP_INFO(get_logger(),
            "Home: (%.0f, %.0f, %.0f) mm  period=%.1fs stride=%.0fmm height=%.0fmm",
            home_x_, home_y_, home_z_, step_period_, stride_length_, step_height_);
        RCLCPP_INFO(get_logger(),
            "Init posture : coxa=%.4f  femur=%.4f  tibia=%.4f rad",
            init_coxa_, init_femur_, TIBIA_FIXED);
        RCLCPP_INFO(get_logger(),
            "Stand posture: coxa=%.4f  femur=%.4f  tibia=%.4f rad  (direct angles, no IK)",
            stand_coxa_angle_, stand_femur_angle_, TIBIA_FIXED);
    }

private:
    enum class State { INIT, INIT_POSTURE, STANDING, WALKING };

    // ── Robot parameters ──────────────────────────────────────────────────
    double L1_, L2_, L3_;
    double body_radius_, beta_rad_;
    double home_x_, home_y_, home_z_;
    double step_period_, stride_length_, step_height_;
    int    traj_hz_;
    double joint_limit_;
    double init_pose_dur_;
    double stand_duration_;
    double vel_debounce_;
    double init_coxa_, init_femur_;
    double stand_coxa_angle_, stand_femur_angle_;

    // Virtual link for 2-DOF IK (fixed tibia)
    double Lv_;     // effective femur+tibia length (mm)
    double alpha_;  // effective angle offset (rad)

    // ── ROS objects ───────────────────────────────────────────────────────
    std::unique_ptr<vx01_hexapod_locomotion::HexapodLocomotion> locomotion_;
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

    // ── 2-DOF Inverse Kinematics (tibia fixed at TIBIA_FIXED) ─────────────
    //
    // Solves for theta1 (coxa yaw) and theta2 (femur pitch).
    // Tibia is always output as TIBIA_FIXED.
    //
    // Returns false if target is out of reach or violates joint limits.
    bool ikFixed(double xp, double yp, double zp,
                 double& t1, double& t2) const
    {
        // Coxa yaw
        t1 = std::atan2(yp, xp);
        double ct1 = std::cos(t1);
        if (std::abs(ct1) < 1e-9) return false;

        // Distance in the vertical leg plane, after subtracting coxa
        double r = xp / ct1 - L1_;
        double z = zp;

        // Clamp if slightly beyond virtual-link reach (numerical safety)
        double d = std::sqrt(r*r + z*z);
        if (d > Lv_ * 0.99) {
            double sc = (Lv_ * 0.99) / d;
            r *= sc;
            z *= sc;
        }
        if (d < 1e-6) return false;

        // Femur angle: aim virtual link at foot then subtract alpha offset
        t2 = std::atan2(z, r) - alpha_;

        return (std::abs(t1) <= joint_limit_ &&
                std::abs(t2) <= joint_limit_);
    }

    double clamp(double a) const {
        return std::max(-joint_limit_, std::min(joint_limit_, a));
    }

    // ── Quadratic Bezier scalar ───────────────────────────────────────────
    static double bz(double t, double p0, double p1, double p2) {
        double u = 1.0 - t;
        return u*u*p0 + 2.0*u*t*p1 + t*t*p2;
    }

    // ── Foot stride/lift position at absolute gait time ───────────────────
    //
    // Tripod gait, duty factor 0.5:
    //   Group A (legs 0, 2, 4): swing first half, drag second half
    //   Group B (legs 1, 3, 5): drag first half, swing second half
    //
    // Swing: Bezier arc, foot from rear (-T/2) over apex to front (+T/2)
    // Drag:  linear stance, foot slides from front (+T/2) to rear (-T/2)
    void footPosAtTime(int leg, double t_abs,
                       double& stride_out, double& lift_out) const
    {
        double half  = step_period_ * 0.5;
        // Group B legs start half a cycle offset so they are always in antiphase
        bool group_b = (leg == 1 || leg == 3 || leg == 5);
        double t_loc = std::fmod(t_abs + (group_b ? half : 0.0), step_period_);

        if (t_loc < half) {
            // SWING — smooth Bezier arc
            double s   = t_loc / half;
            stride_out = bz(s, -stride_length_ * 0.5, 0.0,  stride_length_ * 0.5);
            lift_out   = bz(s,  0.0,                  step_height_,  0.0);
        } else {
            // DRAG — linear stance
            double s   = (t_loc - half) / half;
            stride_out = stride_length_ * 0.5 - s * stride_length_;
            lift_out   = 0.0;
        }
    }

    // ── Build one full-cycle JTC goal for one leg ─────────────────────────
    FollowJT::Goal buildCycleGoal(int leg, double vx) const
    {
        // Scale stride by actual vs nominal velocity
        double nominal_speed = (step_period_ > 1e-6)
                               ? stride_length_ / (step_period_ * 0.5)
                               : 1.0;
        double scale = (nominal_speed > 1e-6)
                       ? std::max(-2.0, std::min(2.0, vx / nominal_speed))
                       : 0.0;

        double dt = 1.0 / static_cast<double>(traj_hz_);
        int    n  = static_cast<int>(std::ceil(step_period_ * traj_hz_));

        // Fallback angles — use direct stand angles (same as standing pose)
        double h1 = stand_coxa_angle_;
        double h2 = stand_femur_angle_;

        FollowJT::Goal goal;
        goal.trajectory.joint_names = {
            JOINT_NAMES[leg][0], JOINT_NAMES[leg][1], JOINT_NAMES[leg][2]
        };
        goal.trajectory.points.reserve(n + 1);

        for (int i = 0; i <= n; ++i) {
            double t_abs = std::min(static_cast<double>(i) * dt, step_period_);

            double stride, lift;
            footPosAtTime(leg, t_abs, stride, lift);

            // IK target in leg-local frame
            double ik_x = home_x_;
            double ik_y = stride * scale;
            double ik_z = home_z_ + lift;

            double t1, t2;
            if (!ikFixed(ik_x, ik_y, ik_z, t1, t2)) {
                // Use home angles on IK failure — never leave joint uncontrolled
                t1 = h1; t2 = h2;
                RCLCPP_WARN_THROTTLE(get_logger(), *clock_, 2000,
                    "IK fallback leg=%d t=%.2f (%.1f, %.1f, %.1f)",
                    leg, t_abs, ik_x, ik_y, ik_z);
            }

            trajectory_msgs::msg::JointTrajectoryPoint pt;
            pt.positions  = { clamp(t1), clamp(t2), TIBIA_FIXED };
            pt.velocities = { 0.0, 0.0, 0.0 };
            pt.time_from_start = rclcpp::Duration::from_seconds(t_abs);
            goal.trajectory.points.push_back(pt);
        }

        return goal;
    }

    // ── Send cycle goal and auto-loop when WALKING ────────────────────────
    void sendCycleGoal(int leg, double vx)
    {
        auto goal = buildCycleGoal(leg, vx);
        auto opts = rclcpp_action::Client<FollowJT>::SendGoalOptions();

        opts.goal_response_callback =
            [this, leg](const GoalHandleFJT::SharedPtr& handle) {
                legs_cycling_[leg] = (bool)handle;
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
                        sendCycleGoal(leg, vx);   // keep looping
                    }
                } else if (result.code != rclcpp_action::ResultCode::CANCELED) {
                    RCLCPP_WARN(get_logger(), "Leg %d cycle ended with code=%d",
                                leg, (int)result.code);
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

    // ── Step 1: Initial posture ────────────────────────────────────────────
    // Sends ALL legs to (coxa=0, femur=-0.785, tibia=-0.75) smoothly.
    // After the LAST leg confirms success, transitions to sendStandGoal().
    void sendInitPoseGoal()
    {
        RCLCPP_INFO(get_logger(),
            "INIT POSTURE → coxa=%.4f  femur=%.4f  tibia=%.4f  (over %.1f s)",
            init_coxa_, init_femur_, TIBIA_FIXED, init_pose_dur_);

        for (int i = 0; i < NUM_LEGS; ++i) {
            FollowJT::Goal goal;
            goal.trajectory.joint_names = {
                JOINT_NAMES[i][0], JOINT_NAMES[i][1], JOINT_NAMES[i][2]
            };

            trajectory_msgs::msg::JointTrajectoryPoint pt;
            pt.positions  = { clamp(init_coxa_), clamp(init_femur_), TIBIA_FIXED };
            pt.velocities = { 0.0, 0.0, 0.0 };
            pt.time_from_start = rclcpp::Duration::from_seconds(init_pose_dur_);
            goal.trajectory.points.push_back(pt);

            auto opts = rclcpp_action::Client<FollowJT>::SendGoalOptions();

            // Only attach result callback to leg 5 (last one).
            // When it finishes, all 6 legs should be in init posture.
            if (i == NUM_LEGS - 1) {
                opts.result_callback =
                    [this](const GoalHandleFJT::WrappedResult& result) {
                        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                            RCLCPP_INFO(get_logger(),
                                "Init posture reached — transitioning to STANDING");
                            state_ = State::STANDING;
                            sendStandGoal();
                        } else {
                            RCLCPP_ERROR(get_logger(),
                                "Init posture goal failed (code=%d) — retrying",
                                (int)result.code);
                            state_ = State::INIT_POSTURE;
                            sendInitPoseGoal();
                        }
                    };
            }

            action_clients_[i]->async_send_goal(goal, opts);
        }
    }

    // ── Step 2: Stand — direct joint angles (NO IK) ───────────────────────
    // We send known-good direct angles instead of computing IK from a Cartesian
    // home position. This is safer and avoids IK singularities or mismatches
    // caused by the fixed tibia constraint.
    //
    // Stand angles:
    //   coxa  = 0.0    (neutral yaw, leg points straight out)
    //   femur = stand_femur_angle_  (+0.10 rad = +5.7 deg: legs push body UP, foot at z=-60mm)
    //   tibia = TIBIA_FIXED = -0.75 rad  (fixed, never changes)
    void sendStandGoal()
    {
        RCLCPP_INFO(get_logger(),
            "STAND → coxa=%.4f  femur=%.4f  tibia=%.4f  (direct angles, over %.1f s)",
            stand_coxa_angle_, stand_femur_angle_, TIBIA_FIXED, stand_duration_);

        for (int i = 0; i < NUM_LEGS; ++i) {
            FollowJT::Goal goal;
            goal.trajectory.joint_names = {
                JOINT_NAMES[i][0], JOINT_NAMES[i][1], JOINT_NAMES[i][2]
            };

            trajectory_msgs::msg::JointTrajectoryPoint pt;
            pt.positions  = { clamp(stand_coxa_angle_), clamp(stand_femur_angle_), TIBIA_FIXED };
            pt.velocities = { 0.0, 0.0, 0.0 };
            pt.time_from_start = rclcpp::Duration::from_seconds(stand_duration_);
            goal.trajectory.points.push_back(pt);

            action_clients_[i]->async_send_goal(goal, {});
        }

        locomotion_->stand();

        RCLCPP_INFO(get_logger(), "Stand goal sent — waiting for /cmd_vel to walk");
    }

    // ── Step 3: Start walking ─────────────────────────────────────────────
    void startWalking(double vx)
    {
        RCLCPP_INFO(get_logger(), "WALKING vx=%.1f mm/s", vx);
        last_sent_vx_ = vx;
        locomotion_->setVelocity(vx, 0.0, 0.0);
        locomotion_->walk();
        state_ = State::WALKING;
        for (int i = 0; i < NUM_LEGS; ++i) {
            sendCycleGoal(i, vx);
        }
    }

    // ── /cmd_vel callback ─────────────────────────────────────────────────
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // Convert m/s → mm/s for internal units
        double new_vx    = msg->linear.x  * 1000.0;
        double new_vy    = msg->linear.y  * 1000.0;
        double new_omega = msg->angular.z;

        { std::lock_guard<std::mutex> lk(vel_mutex_);
          vel_x_ = new_vx; vel_y_ = new_vy; vel_omega_ = new_omega; }

        bool nonzero = (std::abs(new_vx)    > 1.0  ||
                        std::abs(new_vy)    > 1.0  ||
                        std::abs(new_omega) > 0.01);

        if (state_ == State::STANDING && nonzero) {
            // Start walking from standing
            startWalking(new_vx);

        } else if (state_ == State::WALKING && nonzero) {
            // Update velocity if it changed significantly
            double delta = std::abs(new_vx - last_sent_vx_);
            if (delta > vel_debounce_) {
                RCLCPP_INFO(get_logger(),
                    "Velocity update %.1f → %.1f mm/s", last_sent_vx_, new_vx);
                cancelAllCycleGoals();
                last_sent_vx_ = new_vx;
                locomotion_->setVelocity(new_vx, new_vy, new_omega);
                for (int i = 0; i < NUM_LEGS; ++i) sendCycleGoal(i, new_vx);
            }

        } else if (state_ == State::WALKING && !nonzero) {
            // Stop walking, return to stand
            RCLCPP_INFO(get_logger(), "cmd_vel zero — returning to STAND");
            cancelAllCycleGoals();
            state_ = State::STANDING;
            last_sent_vx_ = 0.0;
            sendStandGoal();
        }
    }

    // ── Timer: wait for all action servers ───────────────────────────────
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
                "All 6 action servers ready — sending initial posture");
            state_ = State::INIT_POSTURE;
            timer_->cancel();   // stop polling
            sendInitPoseGoal();
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