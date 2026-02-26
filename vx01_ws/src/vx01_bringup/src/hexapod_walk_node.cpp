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
        declare_parameter("L1",                    60.55);
        declare_parameter("L2",                    73.84);
        declare_parameter("L3",                   112.16);
        declare_parameter("body_radius",           100.0);
        declare_parameter("beta_deg",               62.91);
        declare_parameter("home_x",                230.0);
        declare_parameter("home_y",                  0.0);
        declare_parameter("home_z",                -60.0);
        declare_parameter("step_period",             4.0);
        declare_parameter("stride_length",          80.0);
        declare_parameter("step_height",            20.0);
        declare_parameter("traj_hz",                50);
        declare_parameter("joint_limit_rad",         0.7854);
        declare_parameter("stand_duration",          3.0);
        declare_parameter("vel_debounce_threshold",  5.0);

        // Initial posture angles applied before stand IK
        declare_parameter("init_coxa_angle",   0.0);
        declare_parameter("init_femur_angle", -0.785);
        declare_parameter("init_tibia_angle",  0.6);
        declare_parameter("init_pose_duration", 2.0);

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
        stand_duration_ = get_parameter("stand_duration").as_double();
        vel_debounce_   = get_parameter("vel_debounce_threshold").as_double();
        init_coxa_      = get_parameter("init_coxa_angle").as_double();
        init_femur_     = get_parameter("init_femur_angle").as_double();
        init_tibia_     = get_parameter("init_tibia_angle").as_double();
        init_pose_dur_  = get_parameter("init_pose_duration").as_double();
        beta_rad_       = beta_deg * M_PI / 180.0;

        locomotion_ = std::make_unique<vx01_hexapod_locomotion::HexapodLocomotion>(
            L1_, L2_, L3_, body_radius_, beta_rad_);
        locomotion_->setStepPeriod(step_period_);
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

        timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&HexapodWalkNode::timerCallback, this));

        RCLCPP_INFO(get_logger(),
            "HexapodWalkNode started — L1=%.2f L2=%.2f L3=%.2f "
            "home=(%.0f,%.0f,%.0f) period=%.1fs stride=%.0fmm height=%.0fmm",
            L1_, L2_, L3_, home_x_, home_y_, home_z_,
            step_period_, stride_length_, step_height_);

        RCLCPP_INFO(get_logger(),
            "Initial posture: coxa=%.3f femur=%.3f tibia=%.3f rad",
            init_coxa_, init_femur_, init_tibia_);
    }

private:
    enum class State { INIT, INIT_POSTURE, STANDING, WALKING };

    double L1_, L2_, L3_;
    double body_radius_, beta_rad_;
    double home_x_, home_y_, home_z_;
    double step_period_, stride_length_, step_height_;
    int    traj_hz_;
    double joint_limit_;
    double stand_duration_;
    double vel_debounce_;
    double init_coxa_, init_femur_, init_tibia_, init_pose_dur_;

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
    double last_sent_vx_{0.0};

    // ── Analytical IK (elbow-up) in leg-local frame ──────────────────────
    // Uses L2^2 in cosine rule for phi1 (not L3^2 as was wrong in original)
    bool ikEU(double xp, double yp, double zp,
              double& t1, double& t2, double& t3) const
    {
        t1 = std::atan2(yp, xp);
        double ct1 = std::cos(t1);
        if (std::abs(ct1) < 1e-9) return false;

        double r2 = xp / ct1 - L1_;
        double r1 = std::sqrt(zp*zp + r2*r2);

        // Pull back if slightly over-extended
        double max_reach = (L2_ + L3_) * 0.99;
        if (r1 > max_reach) {
            double sc = max_reach / r1;
            r2 *= sc;
            zp *= sc;
            r1  = max_reach;
        }
        if (r1 < std::abs(L2_ - L3_) + 1e-6) return false;

        double phi2 = std::atan2(zp, r2);

        // Cosine rule: L2 is the link being solved for, L3 is the opposite
        double cp1 = (L2_*L2_ + r1*r1 - L3_*L3_) / (2.0*L2_*r1);
        if (cp1 < -1.0 || cp1 > 1.0) return false;
        double phi1 = std::acos(cp1);
        t2 = phi2 - phi1;   // elbow-up: femur tips downward

        double cp3 = (r1*r1 - L2_*L2_ - L3_*L3_) / (-2.0*L2_*L3_);
        if (cp3 < -1.0 || cp3 > 1.0) return false;
        t3 = M_PI - std::acos(cp3);

        return (std::abs(t1) <= joint_limit_ &&
                std::abs(t2) <= joint_limit_ &&
                std::abs(t3) <= joint_limit_);
    }

    double clamp(double a) const {
        return std::max(-joint_limit_, std::min(joint_limit_, a));
    }

    // ── Quadratic Bezier scalar ───────────────────────────────────────────
    static double bz(double t, double p0, double p1, double p2) {
        double u = 1.0 - t;
        return u*u*p0 + 2.0*u*t*p1 + t*t*p2;
    }

    // ── Compute foot (y-stride, z-lift) at absolute time t within one cycle
    //
    // Tripod gait, duty factor 0.5:
    //   Group A (0,2,4): swing first half, drag second half
    //   Group B (1,3,5): drag first half, swing second half
    //
    // Swing (Bezier): foot goes from rear (-T/2) over arc to front (+T/2), lifts to A
    // Drag (stance):  foot slides from front (+T/2) back to rear (-T/2) on ground
    void footPosAtTime(int leg, double t_abs,
                       double& stride_out, double& lift_out) const
    {
        double half   = step_period_ * 0.5;
        bool group_b  = (leg == 1 || leg == 3 || leg == 5);
        double t_local = std::fmod(t_abs + (group_b ? half : 0.0), step_period_);

        if (t_local < half) {
            // SWING — Bezier arc
            double s   = t_local / half;
            stride_out = bz(s, -stride_length_*0.5, 0.0, stride_length_*0.5);
            lift_out   = bz(s,  0.0, step_height_, 0.0);
        } else {
            // DRAG — linear stance
            double s   = (t_local - half) / half;
            stride_out = stride_length_*0.5 - s * stride_length_;
            lift_out   = 0.0;
        }
    }

    // ── Build one full-cycle JTC goal for one leg ─────────────────────────
    FollowJT::Goal buildCycleGoal(int leg, double vx) const
    {
        double nominal_speed = stride_length_ / (step_period_ * 0.5);
        double scale = (nominal_speed > 1e-6)
                       ? std::max(-2.0, std::min(2.0, vx / nominal_speed))
                       : 0.0;

        double dt = 1.0 / static_cast<double>(traj_hz_);
        int    n  = static_cast<int>(std::ceil(step_period_ * traj_hz_));

        // Precompute home IK for fallback
        double h1=0.0, h2=0.0, h3=0.0;
        ikEU(home_x_, 0.0, home_z_, h1, h2, h3);

        FollowJT::Goal goal;
        goal.trajectory.joint_names = {
            JOINT_NAMES[leg][0], JOINT_NAMES[leg][1], JOINT_NAMES[leg][2]
        };
        goal.trajectory.points.reserve(n + 1);

        for (int i = 0; i <= n; ++i) {
            double t_abs = std::min(static_cast<double>(i) * dt, step_period_);

            double stride, lift;
            footPosAtTime(leg, t_abs, stride, lift);

            // In leg-local frame: x=reach, y=stride*scale, z=home_z+lift
            double ik_x = home_x_;
            double ik_y = stride * scale;
            double ik_z = home_z_ + lift;

            double t1, t2, t3;
            if (!ikEU(ik_x, ik_y, ik_z, t1, t2, t3)) {
                t1 = h1; t2 = h2; t3 = h3;
                RCLCPP_WARN_THROTTLE(get_logger(), *clock_, 2000,
                    "IK fallback leg=%d t=%.2f (%.1f,%.1f,%.1f)",
                    leg, t_abs, ik_x, ik_y, ik_z);
            }

            trajectory_msgs::msg::JointTrajectoryPoint pt;
            pt.positions  = { clamp(t1), clamp(t2), clamp(t3) };
            pt.velocities = { 0.0, 0.0, 0.0 };
            pt.time_from_start = rclcpp::Duration::from_seconds(t_abs);
            goal.trajectory.points.push_back(pt);
        }

        return goal;
    }

    // ── Send cycle goal with auto-loop on success ─────────────────────────
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
                        sendCycleGoal(leg, vx);
                    }
                } else if (result.code != rclcpp_action::ResultCode::CANCELED) {
                    RCLCPP_WARN(get_logger(), "Leg %d goal ended code=%d",
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

    // ── Send initial posture goal (before standing IK) ────────────────────
    // Moves all joints to coxa=init_coxa_, femur=init_femur_, tibia=init_tibia_
    // This tucks the robot into a safe pose before IK-computed standing begins.
    void sendInitPoseGoal()
    {
        RCLCPP_INFO(get_logger(),
            "Sending initial posture (coxa=%.3f femur=%.3f tibia=%.3f) over %.1fs",
            init_coxa_, init_femur_, init_tibia_, init_pose_dur_);

        for (int i = 0; i < NUM_LEGS; ++i) {
            FollowJT::Goal goal;
            goal.trajectory.joint_names = {
                JOINT_NAMES[i][0], JOINT_NAMES[i][1], JOINT_NAMES[i][2]
            };
            trajectory_msgs::msg::JointTrajectoryPoint pt;
            pt.positions  = { clamp(init_coxa_), clamp(init_femur_), clamp(init_tibia_) };
            pt.velocities = { 0.0, 0.0, 0.0 };
            pt.time_from_start = rclcpp::Duration::from_seconds(init_pose_dur_);
            goal.trajectory.points.push_back(pt);

            auto opts = rclcpp_action::Client<FollowJT>::SendGoalOptions();
            if (i == NUM_LEGS - 1) {
                // After last leg reaches initial pose, send stand goal
                opts.result_callback =
                    [this](const GoalHandleFJT::WrappedResult& result) {
                        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                            RCLCPP_INFO(get_logger(), "Initial posture reached — standing");
                            state_ = State::STANDING;
                            sendStandGoal();
                        }
                    };
            }
            action_clients_[i]->async_send_goal(goal, opts);
        }
    }

    // ── Send stand (IK home) position to all legs ─────────────────────────
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
            pt.positions  = { clamp(t1), clamp(t2), clamp(t3) };
            pt.velocities = { 0.0, 0.0, 0.0 };
            pt.time_from_start = rclcpp::Duration::from_seconds(stand_duration_);
            goal.trajectory.points.push_back(pt);

            action_clients_[i]->async_send_goal(goal, {});
        }

        RCLCPP_INFO(get_logger(),
            "Stand goal sent home=(%.0f,%.0f,%.0f)", home_x_, home_y_, home_z_);
    }

    void startWalking(double vx)
    {
        RCLCPP_INFO(get_logger(), "Walking vx=%.1f mm/s", vx);
        last_sent_vx_ = vx;
        locomotion_->setVelocity(vx, 0.0, 0.0);
        locomotion_->walk();
        state_ = State::WALKING;
        for (int i = 0; i < NUM_LEGS; ++i) {
            sendCycleGoal(i, vx);
        }
    }

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        double new_vx    = msg->linear.x  * 1000.0;
        double new_vy    = msg->linear.y  * 1000.0;
        double new_omega = msg->angular.z;

        { std::lock_guard<std::mutex> lk(vel_mutex_);
          vel_x_ = new_vx; vel_y_ = new_vy; vel_omega_ = new_omega; }

        bool nonzero = (std::abs(new_vx)    > 1.0 ||
                        std::abs(new_vy)    > 1.0 ||
                        std::abs(new_omega) > 0.01);

        if (state_ == State::STANDING && nonzero) {
            startWalking(new_vx);

        } else if (state_ == State::WALKING && nonzero) {
            double delta = std::abs(new_vx - last_sent_vx_);
            if (delta > vel_debounce_) {
                RCLCPP_INFO(get_logger(), "Velocity update %.1f → %.1f mm/s",
                            last_sent_vx_, new_vx);
                cancelAllCycleGoals();
                last_sent_vx_ = new_vx;
                locomotion_->setVelocity(new_vx, new_vy, new_omega);
                for (int i = 0; i < NUM_LEGS; ++i) sendCycleGoal(i, new_vx);
            }

        } else if (state_ == State::WALKING && !nonzero) {
            RCLCPP_INFO(get_logger(), "Stop — returning to stand");
            cancelAllCycleGoals();
            state_ = State::STANDING;
            last_sent_vx_ = 0.0;
            sendStandGoal();
        }
    }

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
            RCLCPP_INFO(get_logger(), "All servers ready — sending initial posture");
            state_ = State::INIT_POSTURE;
            timer_->cancel();
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