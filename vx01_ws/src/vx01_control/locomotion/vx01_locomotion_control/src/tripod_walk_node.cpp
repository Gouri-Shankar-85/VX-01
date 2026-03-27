#include "vx01_locomotion_control/tripod_walk_node.hpp"

#include <sensor_msgs/msg/joint_state.hpp>
#include <builtin_interfaces/msg/duration.hpp>

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
    declare_parameter("step_length",           30.0);
    declare_parameter("step_height",           20.0);
    declare_parameter("step_period",            3.0);
    declare_parameter("stand_duration",         3.0);
    declare_parameter("max_linear_vel",         0.15);
    declare_parameter("max_angular_vel",        0.5);
    declare_parameter("trajectory_waypoints",   8);
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
        locomotion_->walk();
        RCLCPP_INFO(get_logger(), "Walking: vx=%.3f vy=%.3f omega=%.3f",
                    cmd_vx_, cmd_vy_, cmd_omega_);
    } else if (!any_motion && walking_) {
        walking_ = false;
        locomotion_->stand();
        RCLCPP_INFO(get_logger(), "Stopping — returning to stand.");
    }
}

void TripodWalkNode::gaitCycleTimer()
{
    if (!walking_) return;
    executeGaitBlock();
    locomotion_->update(step_period_ / 6.0);
}

void TripodWalkNode::executeGaitBlock()
{
    const double block_duration = step_period_ / 6.0;
    for (int leg = 0; leg < 6; ++leg) {
        if (goal_active_[leg]) continue;
        sendLegTrajectory(leg, locomotion_->isSwingPhase(leg), block_duration);
    }
}

std::array<double, 3> TripodWalkNode::computeSwingWaypoint(int leg_id, double global_t)
{
    double th1, th2, th3;
    locomotion_->sampleSwingAtGlobalT(leg_id, global_t, th1, th2, th3);
    return {th1, th2, th3};
}

std::array<double, 3> TripodWalkNode::computeStanceWaypoint(int leg_id, double global_t)
{
    double th1, th2, th3;
    locomotion_->sampleDragAtGlobalT(leg_id, global_t, th1, th2, th3);
    return {th1, th2, th3};
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

        auto angles = computeSwingWaypoint(leg_id, global_t);

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

        auto angles = computeStanceWaypoint(leg_id, global_t);

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
    goal.goal_time_tolerance = rclcpp::Duration::from_seconds(0.5);

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

}  // namespace vx01_locomotion_control

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<vx01_locomotion_control::TripodWalkNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}