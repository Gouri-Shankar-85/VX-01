#include "vx01_locomotion_control/hexapod_forward_walk_node.hpp"
#include "vx01_hexapod_locomotion/kinematics/inverse_kinematics.hpp"
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

namespace vx01_locomotion_control {

HexapodForwardWalkNode::HexapodForwardWalkNode(const rclcpp::NodeOptions& options)
: rclcpp_lifecycle::LifecycleNode("hexapod_forward_walk_node", options),
  standby_done_(false), last_sent_block_(-1), block_period_(0.0),
  last_sent_angles_(18, 0.0)
{
    declareParameters();
}

void HexapodForwardWalkNode::declareParameters()
{
    declare_parameter("L1",               60.55);
    declare_parameter("L2",               73.84);
    declare_parameter("L3",              112.16);
    declare_parameter("body_radius",     100.0);
    declare_parameter("beta_angle",        1.0977);
    declare_parameter("home_x",          227.689);
    declare_parameter("home_y",            0.0);
    declare_parameter("home_z",          -61.379);
    declare_parameter("step_length",     120.0);
    declare_parameter("step_height",      40.0);
    declare_parameter("step_period",       4.0);
    declare_parameter("standby_coxa",      0.0);
    declare_parameter("standby_femur",    -0.7156);
    declare_parameter("standby_tibia",     0.6000);
    declare_parameter("standby_duration",  5.0);
    declare_parameter("update_rate",      50.0);
    declare_parameter("leg_controllers",
        std::vector<std::string>{
            "leg_0_controller","leg_1_controller","leg_2_controller",
            "leg_3_controller","leg_4_controller","leg_5_controller"});
}

void HexapodForwardWalkNode::loadParameters()
{
    L1_               = get_parameter("L1").as_double();
    L2_               = get_parameter("L2").as_double();
    L3_               = get_parameter("L3").as_double();
    body_radius_      = get_parameter("body_radius").as_double();
    beta_angle_       = get_parameter("beta_angle").as_double();
    home_x_           = get_parameter("home_x").as_double();
    home_y_           = get_parameter("home_y").as_double();
    home_z_           = get_parameter("home_z").as_double();
    step_length_      = get_parameter("step_length").as_double();
    step_height_      = get_parameter("step_height").as_double();
    step_period_      = get_parameter("step_period").as_double();
    standby_coxa_     = get_parameter("standby_coxa").as_double();
    standby_femur_    = get_parameter("standby_femur").as_double();
    standby_tibia_    = get_parameter("standby_tibia").as_double();
    standby_duration_ = get_parameter("standby_duration").as_double();
    update_rate_      = get_parameter("update_rate").as_double();
    controller_names_ = get_parameter("leg_controllers").as_string_array();
    block_period_     = step_period_ / 6.0;
}

CallbackReturn HexapodForwardWalkNode::on_configure(const rclcpp_lifecycle::State&)
{
    RCLCPP_INFO(get_logger(), "Configuring...");
    loadParameters();

    // Clamp step height if needed
    {
        const double r2       = home_x_ - L1_;
        const double z_apex   = home_z_ + step_height_;
        const double r1_apex  = std::sqrt(r2*r2 + z_apex*z_apex);
        const double max_reach= L2_ + L3_;
        if (z_apex >= 0.0 || r1_apex > max_reach) {
            const double r1_safe = max_reach * 0.90;
            const double z_safe  = -std::sqrt(std::max(0.0, r1_safe*r1_safe - r2*r2));
            step_height_ = std::max(5.0, z_safe - home_z_);
            RCLCPP_WARN(get_logger(), "step_height clamped to %.1f mm", step_height_);
        }
    }

    locomotion_ = std::make_shared<vx01_hexapod_locomotion::HexapodLocomotion>(
        L1_, L2_, L3_, body_radius_, beta_angle_,
        home_x_, home_y_, home_z_,
        step_length_, step_height_, step_period_);

    // Compute stance angles from IK at home position
    {
        vx01_hexapod_locomotion::kinematics::InverseKinematics ik(L1_, L2_, L3_);
        double th1=0.0, th2=0.0, th3=0.0;
        if (ik.compute(home_x_, 0.0, home_z_, th1, th2, th3)) {
            locomotion_->setStancePose(th2, th3);
            standby_femur_ = th2;
            standby_tibia_ = th3;
            RCLCPP_INFO(get_logger(),
                "Stance IK: femur=%.4f rad  tibia=%.4f rad", th2, th3);
        } else {
            RCLCPP_WARN(get_logger(), "IK failed at home — using YAML standby angles.");
            locomotion_->setStancePose(standby_femur_, standby_tibia_);
        }
    }

    joint_names_ = {
        {"coxa_leg0_joint","femur_leg0_joint","tibia_leg0_joint"},
        {"coxa_leg1_joint","femur_leg1_joint","tibia_leg1_joint"},
        {"coxa_leg2_joint","femur_leg2_joint","tibia_leg2_joint"},
        {"coxa_leg3_joint","femur_leg3_joint","tibia_leg3_joint"},
        {"coxa_leg4_joint","femur_leg4_joint","tibia_leg4_joint"},
        {"coxa_leg5_joint","femur_leg5_joint","tibia_leg5_joint"},
    };

    for (int i = 0; i < 6; ++i) {
        action_clients_.push_back(
            rclcpp_action::create_client<FollowJointTrajectory>(
                this, "/" + controller_names_[i] + "/follow_joint_trajectory"));
    }

    last_sent_angles_.assign(18, 0.0);
    RCLCPP_INFO(get_logger(), "Configured.");
    return CallbackReturn::SUCCESS;
}

CallbackReturn HexapodForwardWalkNode::on_activate(const rclcpp_lifecycle::State&)
{
    RCLCPP_INFO(get_logger(), "Activating — waiting for controllers...");
    rclcpp::sleep_for(2s);

    standby_done_    = false;
    last_sent_block_ = -1;
    last_sent_angles_.assign(18, 0.0);

    sendStandbyPose();
    auto standby_wait = static_cast<int>((standby_duration_ + 0.5) * 1000);
    rclcpp::sleep_for(std::chrono::milliseconds(standby_wait));
    startWalking();

    double period_ms = 1000.0 / update_rate_;
    gait_timer_ = create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(period_ms)),
        std::bind(&HexapodForwardWalkNode::gaitUpdate, this));

    RCLCPP_INFO(get_logger(), "Active — walking.");
    return CallbackReturn::SUCCESS;
}

CallbackReturn HexapodForwardWalkNode::on_deactivate(const rclcpp_lifecycle::State&)
{
    RCLCPP_INFO(get_logger(), "Deactivating — stopping gait.");
    if (gait_timer_) { gait_timer_->cancel(); gait_timer_.reset(); }

    standby_done_ = false;
    locomotion_->stop();
    sendStandbyPose();

    RCLCPP_INFO(get_logger(), "Deactivated.");
    return CallbackReturn::SUCCESS;
}

CallbackReturn HexapodForwardWalkNode::on_cleanup(const rclcpp_lifecycle::State&)
{
    locomotion_.reset();
    action_clients_.clear();
    joint_names_.clear();
    RCLCPP_INFO(get_logger(), "Cleaned up.");
    return CallbackReturn::SUCCESS;
}

CallbackReturn HexapodForwardWalkNode::on_shutdown(const rclcpp_lifecycle::State&)
{
    if (gait_timer_) { gait_timer_->cancel(); gait_timer_.reset(); }
    locomotion_.reset();
    RCLCPP_INFO(get_logger(), "Shutdown.");
    return CallbackReturn::SUCCESS;
}

void HexapodForwardWalkNode::sendStandbyPose()
{
    RCLCPP_INFO(get_logger(), "Moving to standby pose...");
    for (int i = 0; i < 6; ++i)
        sendLegTrajectory(i, standby_coxa_, standby_femur_, standby_tibia_, standby_duration_);
}

void HexapodForwardWalkNode::startWalking()
{
    locomotion_->walk();
    locomotion_->setVelocity(1.0, 0.0, 0.0);

    for (int i = 0; i < 6; ++i) {
        double th1, th2, th3;
        if (locomotion_->isSwingPhase(i))
            locomotion_->sampleSwingAtGlobalT(i, 0.0, th1, th2, th3);
        else
            locomotion_->sampleDragAtGlobalT(i, 0.0, th1, th2, th3);

        last_sent_angles_[i*3+0] = th1;
        last_sent_angles_[i*3+1] = th2;
        last_sent_angles_[i*3+2] = th3;
        sendLegTrajectory(i, th1, th2, th3, 2.0);
    }

    rclcpp::sleep_for(2200ms);
    standby_done_ = true;
    RCLCPP_INFO(get_logger(), "Tripod gait started.");
}

void HexapodForwardWalkNode::gaitUpdate()
{
    if (!standby_done_) return;

    locomotion_->update(1.0 / update_rate_);

    int block = locomotion_->getGaitBlock();
    if (block == last_sent_block_) return;
    last_sent_block_ = block;
    if (block != 0 && block != 3) return;

    const double group_duration = block_period_ * 3.0 * 0.95;
    rclcpp::Time now = get_clock()->now();
    const int N = 12;

    for (int i = 0; i < 6; ++i) {
        if (!action_clients_[i]->action_server_is_ready()) continue;

        auto goal = FollowJointTrajectory::Goal();
        goal.trajectory.joint_names  = joint_names_[i];
        goal.trajectory.header.stamp = now;

        if (locomotion_->isSwingPhase(i)) {
            {
                double th1, th2, th3;
                locomotion_->sampleSwingAtGlobalT(i, 0.0, th1, th2, th3);
                trajectory_msgs::msg::JointTrajectoryPoint pt;
                pt.positions       = {th1, th2, th3};
                pt.time_from_start = rclcpp::Duration::from_seconds(0.0);
                goal.trajectory.points.push_back(pt);
            }
            for (int k = 1; k <= N; ++k) {
                double gt = static_cast<double>(k) / N;
                double th1, th2, th3;
                locomotion_->sampleSwingAtGlobalT(i, gt, th1, th2, th3);
                if (k == N) {
                    last_sent_angles_[i*3+0] = th1;
                    last_sent_angles_[i*3+1] = th2;
                    last_sent_angles_[i*3+2] = th3;
                }
                trajectory_msgs::msg::JointTrajectoryPoint pt;
                pt.positions       = {th1, th2, th3};
                pt.time_from_start = rclcpp::Duration::from_seconds(group_duration * gt);
                goal.trajectory.points.push_back(pt);
            }
        } else {
            double th1 = last_sent_angles_[i*3+0];
            double th2 = last_sent_angles_[i*3+1];
            double th3 = last_sent_angles_[i*3+2];
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

void HexapodForwardWalkNode::sendLegTrajectory(int leg_index,
    double theta1, double theta2, double theta3, double duration_sec)
{
    if (!action_clients_[leg_index]->action_server_is_ready()) return;

    last_sent_angles_[leg_index*3+0] = theta1;
    last_sent_angles_[leg_index*3+1] = theta2;
    last_sent_angles_[leg_index*3+2] = theta3;

    auto goal = FollowJointTrajectory::Goal();
    goal.trajectory.joint_names  = joint_names_[leg_index];
    goal.trajectory.header.stamp = rclcpp::Time(0);

    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.positions       = {theta1, theta2, theta3};
    pt.time_from_start = rclcpp::Duration::from_seconds(duration_sec);
    goal.trajectory.points.push_back(pt);

    action_clients_[leg_index]->async_send_goal(goal);
}

}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<vx01_locomotion_control::HexapodForwardWalkNode>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}