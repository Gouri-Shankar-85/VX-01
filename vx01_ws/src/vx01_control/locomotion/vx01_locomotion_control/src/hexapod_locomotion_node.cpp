#include "vx01_locomotion_control/hexapod_locomotion_node.hpp"
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

namespace vx01_locomotion_control {

HexapodLocomotionNode::HexapodLocomotionNode(const rclcpp::NodeOptions& options)
: Node("hexapod_locomotion_node", options), standby_done_(false)
{
    declare_parameter("L1", 60.55);
    declare_parameter("L2", 73.84);
    declare_parameter("L3", 112.16);
    declare_parameter("body_radius", 100.0);
    declare_parameter("beta_angle", 1.0977);
    declare_parameter("home_x", 108.67);
    declare_parameter("home_y", 0.0);
    declare_parameter("home_z", -80.0);
    declare_parameter("step_length", 110.0);
    declare_parameter("step_height", 22.78);
    declare_parameter("step_period", 2.0);
    declare_parameter("standby_coxa", 0.0);
    declare_parameter("standby_femur", -0.785);
    declare_parameter("standby_tibia", 0.6);
    declare_parameter("standby_duration", 2.0);
    declare_parameter("update_rate", 50.0);
    declare_parameter("leg_controllers", std::vector<std::string>{
        "leg_0_controller","leg_1_controller","leg_2_controller",
        "leg_3_controller","leg_4_controller","leg_5_controller"});

    double L1           = get_parameter("L1").as_double();
    double L2           = get_parameter("L2").as_double();
    double L3           = get_parameter("L3").as_double();
    double body_radius  = get_parameter("body_radius").as_double();
    double beta_angle   = get_parameter("beta_angle").as_double();
    double home_x       = get_parameter("home_x").as_double();
    double home_y       = get_parameter("home_y").as_double();
    double home_z       = get_parameter("home_z").as_double();
    double step_length  = get_parameter("step_length").as_double();
    double step_height  = get_parameter("step_height").as_double();
    step_period_        = get_parameter("step_period").as_double();
    standby_coxa_       = get_parameter("standby_coxa").as_double();
    standby_femur_      = get_parameter("standby_femur").as_double();
    standby_tibia_      = get_parameter("standby_tibia").as_double();
    standby_duration_   = get_parameter("standby_duration").as_double();
    update_rate_        = get_parameter("update_rate").as_double();
    controller_names_   = get_parameter("leg_controllers").as_string_array();

    locomotion_ = std::make_shared<vx01_hexapod_locomotion::HexapodLocomotion>(
        L1, L2, L3, body_radius, beta_angle,
        home_x, home_y, home_z,
        step_length, step_height, step_period_);

    joint_names_ = {
        {"coxa_leg0_joint", "femur_leg0_joint", "tibia_leg0_joint"},
        {"coxa_leg1_joint", "femur_leg1_joint", "tibia_leg1_joint"},
        {"coxa_leg2_joint", "femur_leg2_joint", "tibia_leg2_joint"},
        {"coxa_leg3_joint", "femur_leg3_joint", "tibia_leg3_joint"},
        {"coxa_leg4_joint", "femur_leg4_joint", "tibia_leg4_joint"},
        {"coxa_leg5_joint", "femur_leg5_joint", "tibia_leg5_joint"},
    };

    for (int i = 0; i < 6; ++i) {
        auto client = rclcpp_action::create_client<FollowJointTrajectory>(
            this,
            "/" + controller_names_[i] + "/follow_joint_trajectory");
        action_clients_.push_back(client);
    }

    RCLCPP_INFO(get_logger(), "Waiting for controllers...");
    rclcpp::sleep_for(3s);

    sendStandbyPose();

    auto standby_wait = static_cast<int>((standby_duration_ + 0.5) * 1000);
    rclcpp::sleep_for(std::chrono::milliseconds(standby_wait));

    startWalking();

    double block_period_ms = (step_period_ / 6.0) * 1000.0;
    gait_timer_ = create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(block_period_ms)),
        std::bind(&HexapodLocomotionNode::gaitUpdate, this));
}

void HexapodLocomotionNode::sendStandbyPose()
{
    RCLCPP_INFO(get_logger(), "Moving to standby pose...");
    for (int i = 0; i < 6; ++i) {
        sendLegTrajectory(i, standby_coxa_, standby_femur_, standby_tibia_, standby_duration_);
    }
}

void HexapodLocomotionNode::startWalking()
{
    RCLCPP_INFO(get_logger(), "Starting tripod gait walk...");
    locomotion_->walk();
    standby_done_ = true;
}

void HexapodLocomotionNode::gaitUpdate()
{
    if (!standby_done_) return;

    const double dt = step_period_ / 6.0;
    locomotion_->update(dt);

    const double traj_duration = dt;
    
    for (int i = 0; i < 6; ++i) {
        double t1, t2, t3;
        locomotion_->getLegAngles(i, t1, t2, t3);

        RCLCPP_INFO(get_logger(), "Leg %d: t1=%.3f t2=%.3f t3=%.3f", i, t1, t2, t3);

        sendLegTrajectory(i, t1, t2, t3, traj_duration);
    }
}

void HexapodLocomotionNode::sendLegTrajectory(int leg_index,
                                               double theta1, double theta2, double theta3,
                                               double duration_sec)
{
    if (!action_clients_[leg_index]->action_server_is_ready()) return;

    auto goal = FollowJointTrajectory::Goal();
    goal.trajectory.joint_names = joint_names_[leg_index];

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {theta1, theta2, theta3};
    point.time_from_start = rclcpp::Duration::from_seconds(duration_sec);

    goal.trajectory.points.push_back(point);
    goal.trajectory.header.stamp = now();

    action_clients_[leg_index]->async_send_goal(goal);
}

bool HexapodLocomotionNode::allClientsReady()
{
    for (auto& client : action_clients_) {
        if (!client->action_server_is_ready()) return false;
    }
    return true;
}

}  // namespace vx01_locomotion_control

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<vx01_locomotion_control::HexapodLocomotionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
