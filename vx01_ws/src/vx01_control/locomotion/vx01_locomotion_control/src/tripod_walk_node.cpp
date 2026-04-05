#include "vx01_locomotion_control/tripod_walk_node.hpp"

#include <sensor_msgs/msg/joint_state.hpp>
#include <builtin_interfaces/msg/duration.hpp>
#include <std_msgs/msg/empty.hpp>

#include <chrono>
#include <cmath>
#include <functional>

using namespace std::chrono_literals;
using namespace vx01_hexapod_locomotion;

namespace vx01_locomotion_control {

TripodWalkNode::TripodWalkNode(const rclcpp::NodeOptions & options)
: Node("tripod_walk_node", options)
{
    declareParameters();
    loadParameters();
    initLocomotion();
    initTF();
    initActionClients();
    initJointStateSubscriber();

    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10,
        std::bind(&TripodWalkNode::cmdVelCallback, this, std::placeholders::_1));

    // Go-home service: publish empty message to /hexapod/go_home
    go_home_sub_ = create_subscription<std_msgs::msg::Empty>(
        "/hexapod/go_home", 10,
        [this](const std_msgs::msg::Empty::SharedPtr) {
            RCLCPP_INFO(get_logger(), "Go-home command received.");
            walking_ = false;
            locomotion_->stand();
            sendStandUpTrajectory();
        });

    const double block_period = step_period_ / 6.0;
    gait_timer_ = rclcpp::create_timer(
        this, get_clock(),
        rclcpp::Duration::from_seconds(block_period),
        std::bind(&TripodWalkNode::gaitCycleTimer, this));
    gait_timer_->cancel();

    stand_up_timer_ = rclcpp::create_timer(
        this, get_clock(),
        rclcpp::Duration::from_seconds(2.0),
        [this]() { stand_up_timer_->cancel(); sendStandUpTrajectory(); });

    RCLCPP_INFO(get_logger(), "TripodWalkNode ready — moving to stand pose in 2s.");
}

void TripodWalkNode::declareParameters()
{
    declare_parameter("L1",                   60.55);
    declare_parameter("L2",                   73.84);
    declare_parameter("L3",                  112.16);
    declare_parameter("body_radius",          105.66);
    declare_parameter("beta_angle",           M_PI / 4.0);
    declare_parameter("home_x",              223.03);
    declare_parameter("home_y",                0.0);
    declare_parameter("home_z",              -72.82);
    declare_parameter("step_length",           60.0);
    declare_parameter("step_height",           30.0);
    declare_parameter("step_period",            4.2);
    declare_parameter("stand_duration",         3.0);
    declare_parameter("max_linear_vel",         0.15);
    declare_parameter("max_angular_vel",        0.5);
    declare_parameter("trajectory_waypoints",  12);
    declare_parameter("base_frame",            "base_link");

    declare_parameter("leg_controller_names", std::vector<std::string>{
        "leg_0_controller","leg_1_controller","leg_2_controller",
        "leg_3_controller","leg_4_controller","leg_5_controller"});

    declare_parameter("coxa_frames", std::vector<std::string>{
        "coxa_link_0","coxa_link_1","coxa_link_2",
        "coxa_link_3","coxa_link_4","coxa_link_5"});

    for (int i = 0; i < 6; ++i) {
        declare_parameter("leg_joint_names." + std::to_string(i),
            std::vector<std::string>{
                "coxa_leg"  + std::to_string(i) + "_joint",
                "femur_leg" + std::to_string(i) + "_joint",
                "tibia_leg" + std::to_string(i) + "_joint"});
    }
}

void TripodWalkNode::loadParameters()
{
    L1_              = get_parameter("L1").as_double();
    L2_              = get_parameter("L2").as_double();
    L3_              = get_parameter("L3").as_double();
    body_radius_     = get_parameter("body_radius").as_double();
    beta_angle_      = get_parameter("beta_angle").as_double();
    home_x_          = get_parameter("home_x").as_double();
    home_y_          = get_parameter("home_y").as_double();
    home_z_          = get_parameter("home_z").as_double();
    step_length_     = get_parameter("step_length").as_double();
    step_height_     = get_parameter("step_height").as_double();
    step_period_     = get_parameter("step_period").as_double();
    stand_duration_  = get_parameter("stand_duration").as_double();
    max_linear_vel_  = get_parameter("max_linear_vel").as_double();
    max_angular_vel_ = get_parameter("max_angular_vel").as_double();
    n_waypoints_     = get_parameter("trajectory_waypoints").as_int();
    base_frame_      = get_parameter("base_frame").as_string();

    auto ctrl_names = get_parameter("leg_controller_names").as_string_array();
    auto cxframes   = get_parameter("coxa_frames").as_string_array();

    for (int i = 0; i < 6; ++i) {
        controller_names_[i] = ctrl_names[i];
        coxa_frames_[i]      = cxframes[i];
        joint_names_[i]      = get_parameter(
            "leg_joint_names." + std::to_string(i)).as_string_array();
    }

    const double b = beta_angle_;
    leg_angles_ = {0.0, b, 2.0*b, M_PI, -2.0*b, -b};

    for (int i = 0; i < 18; ++i) current_joint_state_[i] = 0.0;
}

void TripodWalkNode::initLocomotion()
{
    locomotion_ = std::make_shared<HexapodLocomotion>(
        L1_, L2_, L3_,
        body_radius_, beta_angle_,
        home_x_, home_y_, home_z_,
        step_length_, step_height_, step_period_);
    locomotion_->stand();
}

void TripodWalkNode::initTF()
{
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void TripodWalkNode::initActionClients()
{
    for (int i = 0; i < 6; ++i) {
        const std::string action_name =
            "/" + controller_names_[i] + "/follow_joint_trajectory";
        action_clients_[i] =
            rclcpp_action::create_client<FollowJointTrajectory>(this, action_name);
        goal_active_[i] = false;
    }
    RCLCPP_INFO(get_logger(), "Waiting for leg action servers...");
    for (int i = 0; i < 6; ++i) {
        if (!action_clients_[i]->wait_for_action_server(10s)) {
            RCLCPP_WARN(get_logger(), "Action server for leg %d not available", i);
        }
    }
    RCLCPP_INFO(get_logger(), "All action servers connected.");
}

void TripodWalkNode::initJointStateSubscriber()
{
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        std::bind(&TripodWalkNode::jointStateCallback, this, std::placeholders::_1));
}

void TripodWalkNode::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    for (size_t k = 0; k < msg->name.size(); ++k) {
        for (int leg = 0; leg < 6; ++leg) {
            for (int j = 0; j < 3; ++j) {
                if (msg->name[k] == joint_names_[leg][j]) {
                    current_joint_state_[leg * 3 + j] = msg->position[k];
                }
            }
        }
    }
    joint_state_received_ = true;
}

// Compute per-leg stride amplitude in leg-local X and Y for given cmd_vel.
void TripodWalkNode::legStride(int leg_id, double& stride_x, double& stride_y) const
{
    const double a       = leg_angles_[leg_id];
    // Distance from robot center to foot = body_radius_ + home_x_
    const double R       = body_radius_ + home_x_;
    const double r_x     = std::cos(a) * R;
    const double r_y     = std::sin(a) * R;
    
    // Foot velocity in base frame (cmd_vel - omega x r)
    // Actually, when robot moves with cmd_vel, foot must move opposite during stance
    // So foot motion in base frame relative to ground is V_base + omega x r
    // We want the swing vector to be strictly this relative motion scaled by step_length/2.
    const double v_x     = cmd_vx_ - cmd_omega_ * r_y;
    const double v_y     = cmd_vy_ + cmd_omega_ * r_x;
    
    // Convert base velocity vector to leg-local frame
    const double loc_v_x = v_x * std::cos(a) + v_y * std::sin(a);
    const double loc_v_y = -v_x * std::sin(a) + v_y * std::cos(a);
    
    // Calculate scaling to not exceed physical step_length
    const double speed   = std::hypot(v_x, v_y);
    const double scale   = (max_linear_vel_ > 1e-9 && speed > 1e-9) 
                           ? std::min(1.0, speed / max_linear_vel_) 
                           : 0.0;
                           
    const double normalized_step = (speed > 1e-9) ? (step_length_ / 2.0) * scale / speed : 0.0;
    stride_x = loc_v_x * normalized_step;
    stride_y = loc_v_y * normalized_step;
}

// IK for a given (leg_x, leg_y, leg_z) in leg-local frame.
// Returns false if unreachable.
bool TripodWalkNode::computeIK(double lx, double ly, double lz,
                                double& t1, double& t2, double& t3) const
{
    kinematics::InverseKinematics ik(L1_, L2_, L3_);
    return ik.compute(lx, ly, lz, t1, t2, t3);
}

// Swing waypoint at normalised t in [0,1] over the full swing arc:
//   leg_x, leg_y: 3D quadratic Bezier blending stride_x, stride_y
//   leg_z: 3D quadratic Bezier reaching step_height at t=0.5
std::array<double, 3> TripodWalkNode::swingWaypoint(int leg_id, double t) const
{
    t = std::clamp(t, 0.0, 1.0);
    double stride_x = 0.0, stride_y = 0.0;
    legStride(leg_id, stride_x, stride_y);
    
    // 3D Bezier curve
    // P1: Start of swing (leg pushed back)
    const double P1x = home_x_ - stride_x;
    const double P1y = -stride_y;
    const double P1z = home_z_;
    
    // P2: Apex of swing
    const double P2x = home_x_;
    const double P2y = 0.0;
    const double P2z = home_z_ + 2.0 * step_height_;
    
    // P3: End of swing (leg pushed forward)
    const double P3x = home_x_ + stride_x;
    const double P3y = stride_y;
    const double P3z = home_z_;
    
    const double u  = 1.0 - t;
    const double a_b = u * u;
    const double b_b = 2.0 * u * t;
    const double c_b = t * t;
    
    const double leg_x = a_b * P1x + b_b * P2x + c_b * P3x;
    const double leg_y = a_b * P1y + b_b * P2y + c_b * P3y;
    const double leg_z = a_b * P1z + b_b * P2z + c_b * P3z;

    double t1 = 0.0, t2 = 0.0, t3 = 0.0;
    if (!computeIK(leg_x, leg_y, leg_z, t1, t2, t3)) {
        t1 = current_joint_state_[leg_id*3+0];
        t2 = current_joint_state_[leg_id*3+1];
        t3 = current_joint_state_[leg_id*3+2];
    }
    return {t1, t2, t3};
}

// Stance/drag waypoint at normalised t in [0,1]:
//   leg slides from (stride_x, stride_y) to (-stride_x, -stride_y)
//   leg_z = home_z constant
std::array<double, 3> TripodWalkNode::stanceWaypoint(int leg_id, double t) const
{
    t = std::clamp(t, 0.0, 1.0);
    double stride_x = 0.0, stride_y = 0.0;
    legStride(leg_id, stride_x, stride_y);
    
    // stance slides from forward to backward
    const double leg_x = home_x_ + stride_x * (1.0 - 2.0 * t);
    const double leg_y = stride_y * (1.0 - 2.0 * t);

    double t1 = 0.0, t2 = 0.0, t3 = 0.0;
    if (!computeIK(leg_x, leg_y, home_z_, t1, t2, t3)) {
        t1 = current_joint_state_[leg_id*3+0];
        t2 = current_joint_state_[leg_id*3+1];
        t3 = current_joint_state_[leg_id*3+2];
    }
    return {t1, t2, t3};
}

trajectory_msgs::msg::JointTrajectory
TripodWalkNode::buildSwingTrajectory(int leg_id, double block_duration)
{
    trajectory_msgs::msg::JointTrajectory traj;
    traj.joint_names = joint_names_[leg_id];

    const int block       = locomotion_->getGaitBlock();
    const int swing_start = (leg_id == 0 || leg_id == 2 || leg_id == 4) ? 0 : 3;
    const int swing_sub   = block - swing_start;

    for (int wp = 0; wp <= n_waypoints_; ++wp) {
        const double local_t  = static_cast<double>(wp) / n_waypoints_;
        const double global_t = (static_cast<double>(swing_sub) + local_t) / 3.0;

        auto angles = swingWaypoint(leg_id, global_t);

        trajectory_msgs::msg::JointTrajectoryPoint pt;
        pt.positions  = {angles[0], angles[1], angles[2]};
        pt.velocities = {0.0, 0.0, 0.0};
        pt.time_from_start = rclcpp::Duration::from_seconds(local_t * block_duration);
        traj.points.push_back(pt);
    }
    return traj;
}

trajectory_msgs::msg::JointTrajectory
TripodWalkNode::buildStanceTrajectory(int leg_id, double block_duration)
{
    trajectory_msgs::msg::JointTrajectory traj;
    traj.joint_names = joint_names_[leg_id];

    const int block      = locomotion_->getGaitBlock();
    const int drag_start = (leg_id == 0 || leg_id == 2 || leg_id == 4) ? 3 : 0;
    const int drag_sub   = block - drag_start;

    for (int wp = 0; wp <= n_waypoints_; ++wp) {
        const double local_t  = static_cast<double>(wp) / n_waypoints_;
        const double global_t = (static_cast<double>(drag_sub) + local_t) / 3.0;

        auto angles = stanceWaypoint(leg_id, global_t);

        trajectory_msgs::msg::JointTrajectoryPoint pt;
        pt.positions  = {angles[0], angles[1], angles[2]};
        pt.velocities = {0.0, 0.0, 0.0};
        pt.time_from_start = rclcpp::Duration::from_seconds(local_t * block_duration);
        traj.points.push_back(pt);
    }
    return traj;
}

void TripodWalkNode::sendLegTrajectory(int leg_id, bool is_swing, double block_duration)
{
    auto traj = is_swing
        ? buildSwingTrajectory(leg_id, block_duration)
        : buildStanceTrajectory(leg_id, block_duration);

    traj.header.stamp    = now();
    traj.header.frame_id = base_frame_;

    auto goal = FollowJointTrajectory::Goal();
    goal.trajectory          = traj;
    goal.goal_time_tolerance = rclcpp::Duration::from_seconds(2.0);

    goal_active_[leg_id] = true;

    auto send_opts = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();

    send_opts.result_callback =
        [this, leg_id](const GoalHandleFJT::WrappedResult & result) {
            goal_active_[leg_id] = false;
            if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_WARN(get_logger(), "Leg %d trajectory did not succeed (code=%d)",
                            leg_id, static_cast<int>(result.code));
            }
        };

    send_opts.feedback_callback =
        [](GoalHandleFJT::SharedPtr,
           const std::shared_ptr<const FollowJointTrajectory::Feedback>) {};

    action_clients_[leg_id]->async_send_goal(goal, send_opts);
}

void TripodWalkNode::sendStandUpTrajectory()
{
    if (!joint_state_received_) {
        RCLCPP_WARN(get_logger(), "No joint states yet — retrying stand-up in 1s.");
        stand_up_timer_ = rclcpp::create_timer(
            this, get_clock(), rclcpp::Duration::from_seconds(1.0),
            [this]() { stand_up_timer_->cancel(); sendStandUpTrajectory(); });
        return;
    }

    locomotion_->stand();
    const auto home_angles = locomotion_->getJointAngles();

    RCLCPP_INFO(get_logger(), "Sending stand-up trajectory (%.1fs).", stand_duration_);

    auto stand_done = std::make_shared<int>(0);

    for (int leg = 0; leg < 6; ++leg) {
        trajectory_msgs::msg::JointTrajectory traj;
        traj.header.stamp    = now();
        traj.header.frame_id = base_frame_;
        traj.joint_names     = joint_names_[leg];

        trajectory_msgs::msg::JointTrajectoryPoint p0, p1;

        p0.positions  = {current_joint_state_[leg*3+0],
                         current_joint_state_[leg*3+1],
                         current_joint_state_[leg*3+2]};
        p0.velocities = {0.0, 0.0, 0.0};
        p0.time_from_start = rclcpp::Duration::from_seconds(0.0);

        p1.positions  = {home_angles[leg*3+0],
                         home_angles[leg*3+1],
                         home_angles[leg*3+2]};
        p1.velocities = {0.0, 0.0, 0.0};
        p1.time_from_start = rclcpp::Duration::from_seconds(stand_duration_);

        traj.points = {p0, p1};

        auto goal = FollowJointTrajectory::Goal();
        goal.trajectory          = traj;
        goal.goal_time_tolerance = rclcpp::Duration::from_seconds(2.0);

        goal_active_[leg] = true;
        auto send_opts    = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();

        send_opts.result_callback =
            [this, leg, stand_done](const GoalHandleFJT::WrappedResult & result) {
                goal_active_[leg] = false;
                if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
                    RCLCPP_WARN(get_logger(), "Stand-up leg %d failed (code=%d)",
                                leg, static_cast<int>(result.code));
                }
                (*stand_done)++;
                if (*stand_done == 6) {
                    RCLCPP_INFO(get_logger(), "Stand pose reached. Ready to walk.");
                    gait_timer_->reset();
                }
            };

        action_clients_[leg]->async_send_goal(goal, send_opts);
    }
}

void TripodWalkNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    const double speed    = std::hypot(msg->linear.x, msg->linear.y);
    const bool any_motion = speed > 1e-3 || std::abs(msg->angular.z) > 1e-3;

    cmd_vx_    = std::clamp(msg->linear.x,  -max_linear_vel_,  max_linear_vel_);
    cmd_vy_    = std::clamp(msg->linear.y,  -max_linear_vel_,  max_linear_vel_);
    cmd_omega_ = std::clamp(msg->angular.z, -max_angular_vel_, max_angular_vel_);

    locomotion_->setVelocity(cmd_vx_, cmd_vy_, cmd_omega_);

    if (any_motion && !walking_) {
        walking_ = true;
        half_cycle_group_ = 0;  // Start with group A swinging
        legs_done_        = 0;
        locomotion_->walk();
        RCLCPP_INFO(get_logger(), "Walking: vx=%.3f vy=%.3f omega=%.3f",
                    cmd_vx_, cmd_vy_, cmd_omega_);
        // Kick off the first half-cycle immediately
        sendHalfCycle();
    } else if (!any_motion && walking_) {
        walking_ = false;
        locomotion_->stand();
        RCLCPP_INFO(get_logger(), "Stopping — returning to stand.");
    }
}

// Sends full swing + stance trajectories for all 6 legs for current half_cycle_group_.
// Groups: 0 => A swings (legs 0,2,4), 1 => B swings (legs 1,3,5)
void TripodWalkNode::sendHalfCycle()
{
    if (!walking_) return;

    // Bump the generation counter — stale callbacks from the previous cycle
    // will see their captured cycle_id != current cycle_id_ and bail out.
    const int this_cycle = ++cycle_id_;
    legs_done_.store(0);

    const double half_dur     = step_period_ / 2.0;
    const bool   group_a_swings = (half_cycle_group_ == 0);

    RCLCPP_INFO(get_logger(), "Half-cycle %d: %s swings",
                this_cycle, group_a_swings ? "A(0,2,4)" : "B(1,3,5)");

    for (int leg = 0; leg < 6; ++leg) {
        const bool in_group_a = (leg == 0 || leg == 2 || leg == 4);
        const bool is_swing   = group_a_swings ? in_group_a : !in_group_a;
        sendFullHalfCycleTraj(leg, is_swing, half_dur, this_cycle);
    }
}

void TripodWalkNode::sendFullHalfCycleTraj(
    int leg_id, bool is_swing, double duration, int this_cycle)
{
    trajectory_msgs::msg::JointTrajectory traj;
    traj.joint_names     = joint_names_[leg_id];
    traj.header.stamp    = now();
    traj.header.frame_id = base_frame_;

    const int n_wp = n_waypoints_ * 3;  // 3x waypoints for smooth full half-cycle

    for (int wp = 0; wp <= n_wp; ++wp) {
        const double t = static_cast<double>(wp) / static_cast<double>(n_wp);
        const auto angles = is_swing ? swingWaypoint(leg_id, t)
                                     : stanceWaypoint(leg_id, t);
        trajectory_msgs::msg::JointTrajectoryPoint pt;
        pt.positions       = {angles[0], angles[1], angles[2]};
        pt.velocities      = {0.0, 0.0, 0.0};
        pt.time_from_start = rclcpp::Duration::from_seconds(t * duration);
        traj.points.push_back(pt);
    }

    auto goal = FollowJointTrajectory::Goal();
    goal.trajectory          = traj;
    goal.goal_time_tolerance = rclcpp::Duration::from_seconds(1.5);

    goal_active_[leg_id] = true;

    auto send_opts = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();

    send_opts.result_callback =
        [this, leg_id, this_cycle](const GoalHandleFJT::WrappedResult & result) {
            goal_active_[leg_id] = false;

            // CRITICAL: ignore stale callbacks from a previous cycle
            if (this_cycle != cycle_id_.load()) return;

            if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_DEBUG(get_logger(),
                    "Leg %d cycle %d: code=%d (canceled/aborted — normal during stop)",
                    leg_id, this_cycle, static_cast<int>(result.code));
            }

            // Count toward this cycle's completion
            const int done = ++legs_done_;
            if (done == 6 && walking_) {
                // All 6 legs finished — advance to next half-cycle
                half_cycle_group_ = 1 - half_cycle_group_;
                sendHalfCycle();  // this bumps cycle_id_, invalidating this_cycle
            }
        };

    send_opts.feedback_callback =
        [](GoalHandleFJT::SharedPtr,
           const std::shared_ptr<const FollowJointTrajectory::Feedback>) {};

    action_clients_[leg_id]->async_send_goal(goal, send_opts);
}

// Legacy gaitCycleTimer and executeGaitBlock kept empty — driving is done via sendHalfCycle()
void TripodWalkNode::gaitCycleTimer() {}
void TripodWalkNode::executeGaitBlock() {}


}  // namespace vx01_locomotion_control

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<vx01_locomotion_control::TripodWalkNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}