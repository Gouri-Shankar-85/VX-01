#include "vx01_locomotion_control/hexapod_locomotion_node.hpp"
#include "vx01_hexapod_locomotion/kinematics/inverse_kinematics.hpp"
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

namespace vx01_locomotion_control {

HexapodLocomotionNode::HexapodLocomotionNode(const rclcpp::NodeOptions& options)
: Node("hexapod_locomotion_node", options), standby_done_(false),
  last_sent_block_(-1), block_period_(0.0),
  last_sent_angles_(18, 0.0)
{
    declare_parameter("L1", 60.55);
    declare_parameter("L2", 73.84);
    declare_parameter("L3", 112.16);
    declare_parameter("body_radius", 100.0);
    declare_parameter("beta_angle", 1.0977);
    declare_parameter("home_x", 227.689);
    declare_parameter("home_y", 0.0);
    declare_parameter("home_z", -61.379);  
    declare_parameter("step_length", 120.0);
    declare_parameter("step_height",  40.0); 
    declare_parameter("step_period",   4.0);
    declare_parameter("standby_coxa",  0.0);
    declare_parameter("standby_femur", -0.7156);
    declare_parameter("standby_tibia",  0.6000);
    declare_parameter("standby_duration", 5.0);
    declare_parameter("update_rate", 50.0);
    declare_parameter("leg_controllers", std::vector<std::string>{
        "leg_0_controller","leg_1_controller","leg_2_controller",
        "leg_3_controller","leg_4_controller","leg_5_controller"});

    double L1          = get_parameter("L1").as_double();
    double L2          = get_parameter("L2").as_double();
    double L3          = get_parameter("L3").as_double();
    double body_radius = get_parameter("body_radius").as_double();
    double beta_angle  = get_parameter("beta_angle").as_double();
    double home_x      = get_parameter("home_x").as_double();
    double home_y      = get_parameter("home_y").as_double();
    double home_z      = get_parameter("home_z").as_double();
    double step_length = get_parameter("step_length").as_double();
    double step_height = get_parameter("step_height").as_double();
    step_period_       = get_parameter("step_period").as_double();
    block_period_      = step_period_ / 6.0;
    standby_coxa_      = get_parameter("standby_coxa").as_double();
    standby_femur_     = get_parameter("standby_femur").as_double();
    standby_tibia_     = get_parameter("standby_tibia").as_double();
    standby_duration_  = get_parameter("standby_duration").as_double();
    update_rate_       = get_parameter("update_rate").as_double();
    controller_names_  = get_parameter("leg_controllers").as_string_array();

    // Safety check: local-frame swing apex must stay within reach.
    // sampleSwingAtGlobalT uses IK at (home_x, 0, home_z+step_height).
    // r2_local = home_x - L1 (constant), r1_apex = sqrt(r2_local^2 + (home_z+sh)^2)
    {
        const double r2_local  = home_x - L1;
        const double z_apex    = home_z + step_height;
        const double r1_apex   = std::sqrt(r2_local*r2_local + z_apex*z_apex);
        const double max_reach = L2 + L3;
        if (z_apex >= 0.0 || r1_apex > max_reach) {
            RCLCPP_WARN(get_logger(),
                "step_height=%.1f mm puts apex z=%.1f mm, r1=%.1f mm (max=%.1f mm). "
                "Clamping step_height.", step_height, z_apex, r1_apex, max_reach);
            // Clamp: apex at 90% max reach
            const double r1_safe = max_reach * 0.90;
            const double z_safe  = -std::sqrt(std::max(0.0, r1_safe*r1_safe - r2_local*r2_local));
            step_height = std::max(5.0, z_safe - home_z);
            RCLCPP_WARN(get_logger(), "step_height clamped to %.1f mm", step_height);
        }
    }

    locomotion_ = std::make_shared<vx01_hexapod_locomotion::HexapodLocomotion>(
        L1, L2, L3, body_radius, beta_angle,
        home_x, home_y, home_z,
        step_length, step_height, step_period_);

    // Derive stance femur/tibia from IK at home position (local frame, y=0).
    // This matches the local-frame IK used in sampleSwingAtGlobalT exactly.
    {
        vx01_hexapod_locomotion::kinematics::InverseKinematics ik_home(L1, L2, L3);
        double th1 = 0.0, th2 = 0.0, th3 = 0.0;
        if (ik_home.compute(home_x, 0.0, home_z, th1, th2, th3)) {
            locomotion_->setStancePose(th2, th3);
            standby_femur_ = th2;
            standby_tibia_ = th3;
            RCLCPP_INFO(get_logger(),
                "Stance IK at (%.3f, 0, %.3f): femur=%.4f rad (%.1f°)  tibia=%.4f rad (%.1f°)",
                home_x, home_z, th2, th2*180.0/M_PI, th3, th3*180.0/M_PI);
        } else {
            RCLCPP_WARN(get_logger(),
                "IK failed at home — using YAML standby angles.");
            locomotion_->setStancePose(standby_femur_, standby_tibia_);
        }
    }

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
            this, "/" + controller_names_[i] + "/follow_joint_trajectory");
        action_clients_.push_back(client);
    }

    RCLCPP_INFO(get_logger(), "Waiting for controllers...");
    rclcpp::sleep_for(3s);

    sendStandbyPose();

    auto standby_wait = static_cast<int>((standby_duration_ + 0.5) * 1000);
    rclcpp::sleep_for(std::chrono::milliseconds(standby_wait));

    startWalking();

    double update_period_ms = 1000.0 / update_rate_;
    gait_timer_ = create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(update_period_ms)),
        std::bind(&HexapodLocomotionNode::gaitUpdate, this));
}

void HexapodLocomotionNode::sendStandbyPose()
{
    RCLCPP_INFO(get_logger(), "Moving to standby pose...");
    for (int i = 0; i < 6; ++i)
        sendLegTrajectory(i, standby_coxa_, standby_femur_, standby_tibia_, standby_duration_);
}

void HexapodLocomotionNode::startWalking()
{
    RCLCPP_INFO(get_logger(), "Moving to gait start position...");

    locomotion_->walk();
    locomotion_->setVelocity(1.0, 0.0, 0.0);

    for (int i = 0; i < 6; ++i) {
        double th1, th2, th3;
        if (locomotion_->isSwingPhase(i)) {
            locomotion_->sampleSwingAtGlobalT(i, 0.0, th1, th2, th3);  // rear
        } else {
            locomotion_->sampleDragAtGlobalT(i, 0.0, th1, th2, th3);   // front (+T/2)
        }
        last_sent_angles_[i*3+0] = th1;
        last_sent_angles_[i*3+1] = th2;
        last_sent_angles_[i*3+2] = th3;
        sendLegTrajectory(i, th1, th2, th3, 2.0);
    }

    rclcpp::sleep_for(std::chrono::milliseconds(2200));
    standby_done_ = true;
    RCLCPP_INFO(get_logger(), "Starting tripod gait walk...");
}

void HexapodLocomotionNode::gaitUpdate()
{
    if (!standby_done_) return;

    const double dt = 1.0 / update_rate_;
    locomotion_->update(dt);

    int current_block = locomotion_->getGaitBlock();
    if (current_block == last_sent_block_) return;
    last_sent_block_ = current_block;

    if (current_block != 0 && current_block != 3) return;

    RCLCPP_INFO(get_logger(), "Block %d — swing: %s  stance: %s (holding still)",
        current_block,
        current_block == 0 ? "0,2,4" : "1,3,5",
        current_block == 0 ? "1,3,5" : "0,2,4");

    const double group_duration = block_period_ * 3.0 * 0.95;
    rclcpp::Time now = this->get_clock()->now();
    const int N = 12;

    for (int i = 0; i < 6; ++i) {
        if (!action_clients_[i]->action_server_is_ready()) continue;

        auto goal = FollowJointTrajectory::Goal();
        goal.trajectory.joint_names  = joint_names_[i];
        goal.trajectory.header.stamp = now;

        if (locomotion_->isSwingPhase(i)) {
            RCLCPP_INFO(get_logger(), "Leg %d SWING:", i);
            for (int k = 1; k <= N; ++k) {
                double global_t = static_cast<double>(k) / N;
                double th1, th2, th3;
                locomotion_->sampleSwingAtGlobalT(i, global_t, th1, th2, th3);
                RCLCPP_INFO(get_logger(),
                    "  t=%.2f  coxa=%.3f(%.1f°)  femur=%.3f(%.1f°)  tibia=%.3f(%.1f°)",
                    global_t, th1,th1*180/M_PI, th2,th2*180/M_PI, th3,th3*180/M_PI);
                // Track the final swing pose so stance hold is correct next cycle
                if (k == N) {
                    last_sent_angles_[i*3+0] = th1;
                    last_sent_angles_[i*3+1] = th2;
                    last_sent_angles_[i*3+2] = th3;
                }
                trajectory_msgs::msg::JointTrajectoryPoint pt;
                pt.positions       = {th1, th2, th3};
                pt.time_from_start = rclcpp::Duration::from_seconds(group_duration * global_t);
                goal.trajectory.points.push_back(pt);
            }
        } else {
            double th1 = last_sent_angles_[i*3+0];
            double th2 = last_sent_angles_[i*3+1];
            double th3 = last_sent_angles_[i*3+2];

            // Fallback on first cycle before any pose has been tracked
            if (std::abs(th1) < 1e-6 && std::abs(th2) < 1e-6 && std::abs(th3) < 1e-6) {
                locomotion_->sampleDragAtGlobalT(i, 0.0, th1, th2, th3);
                last_sent_angles_[i*3+0] = th1;
                last_sent_angles_[i*3+1] = th2;
                last_sent_angles_[i*3+2] = th3;
            }

            trajectory_msgs::msg::JointTrajectoryPoint pt;
            pt.positions       = {th1, th2, th3};
            pt.time_from_start = rclcpp::Duration::from_seconds(group_duration);
            goal.trajectory.points.push_back(pt);
        }

        action_clients_[i]->async_send_goal(goal);
    }
}

void HexapodLocomotionNode::sendLegTrajectory(int leg_index,
                                               double theta1, double theta2, double theta3,
                                               double duration_sec)
{
    if (!action_clients_[leg_index]->action_server_is_ready()) return;

    // Track what we last commanded to each leg for stance hold
    last_sent_angles_[leg_index*3+0] = theta1;
    last_sent_angles_[leg_index*3+1] = theta2;
    last_sent_angles_[leg_index*3+2] = theta3;

    auto goal = FollowJointTrajectory::Goal();
    goal.trajectory.joint_names  = joint_names_[leg_index];
    goal.trajectory.header.stamp = rclcpp::Time(0);

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions       = {theta1, theta2, theta3};
    point.time_from_start = rclcpp::Duration::from_seconds(duration_sec);
    goal.trajectory.points.push_back(point);

    action_clients_[leg_index]->async_send_goal(goal);
}

bool HexapodLocomotionNode::allClientsReady()
{
    for (auto& client : action_clients_)
        if (!client->action_server_is_ready()) return false;
    return true;
}

}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<vx01_locomotion_control::HexapodLocomotionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}