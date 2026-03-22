#include "vx01_locomotion_control/hexapod_turn_node.hpp"
#include "vx01_hexapod_locomotion/kinematics/inverse_kinematics.hpp"
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

namespace vx01_locomotion_control {

HexapodTurnNode::HexapodTurnNode(const rclcpp::NodeOptions& options)
: Node("hexapod_turn_node", options),
  standby_done_(false), last_sent_block_(-1), block_period_(0.0),
  last_sent_angles_(18, 0.0)
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
    declare_parameter("turn_direction",    1);   // +1=CCW (left), -1=CW (right)
    declare_parameter("leg_controllers",
        std::vector<std::string>{
            "leg_0_controller","leg_1_controller","leg_2_controller",
            "leg_3_controller","leg_4_controller","leg_5_controller"});

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
    turn_direction_   = get_parameter("turn_direction").as_int();
    controller_names_ = get_parameter("leg_controllers").as_string_array();

    block_period_ = step_period_ / 6.0;

    // Clamp turn_direction to ±1
    turn_direction_ = (turn_direction_ >= 0) ? +1 : -1;

    const double b = beta_angle_;
    leg_angles_ = { 0.0, b, 2.0*b, M_PI, -2.0*b, -b };

    {
        const double r2_local  = home_x_ - L1_;
        const double z_apex    = home_z_ + step_height_;
        const double r1_apex   = std::sqrt(r2_local*r2_local + z_apex*z_apex);
        const double max_reach = L2_ + L3_;
        if (z_apex >= 0.0 || r1_apex > max_reach) {
            RCLCPP_WARN(get_logger(),
                "step_height=%.1f mm puts apex z=%.1f mm, r1=%.1f mm (max=%.1f mm). "
                "Clamping step_height.", step_height_, z_apex, r1_apex, max_reach);
            const double r1_safe = max_reach * 0.90;
            const double z_safe  = -std::sqrt(std::max(0.0, r1_safe*r1_safe - r2_local*r2_local));
            step_height_ = std::max(5.0, z_safe - home_z_);
            RCLCPP_WARN(get_logger(), "step_height clamped to %.1f mm", step_height_);
        }
    }

    locomotion_ = std::make_shared<vx01_hexapod_locomotion::HexapodLocomotion>(
        L1_, L2_, L3_, body_radius_, beta_angle_,
        home_x_, home_y_, home_z_,
        step_length_, step_height_, step_period_);

    {
        vx01_hexapod_locomotion::kinematics::InverseKinematics ik_home(L1_, L2_, L3_);
        double th1 = 0.0, th2 = 0.0, th3 = 0.0;
        if (ik_home.compute(home_x_, 0.0, home_z_, th1, th2, th3)) {
            locomotion_->setStancePose(th2, th3);
            standby_femur_ = th2;
            standby_tibia_ = th3;
            RCLCPP_INFO(get_logger(),
                "Stance IK at (%.3f, 0, %.3f): femur=%.4f rad (%.1f°)  tibia=%.4f rad (%.1f°)",
                home_x_, home_z_, th2, th2*180.0/M_PI, th3, th3*180.0/M_PI);
        } else {
            RCLCPP_WARN(get_logger(), "IK failed at home — using YAML standby angles.");
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
    startTurning();

    double update_period_ms = 1000.0 / update_rate_;
    gait_timer_ = create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(update_period_ms)),
        std::bind(&HexapodTurnNode::gaitUpdate, this));
}

void HexapodTurnNode::sendStandbyPose()
{
    RCLCPP_INFO(get_logger(), "Moving to standby pose...");
    for (int i = 0; i < 6; ++i)
        sendLegTrajectory(i, standby_coxa_, standby_femur_, standby_tibia_, standby_duration_);
}

void HexapodTurnNode::startTurning()
{
    RCLCPP_INFO(get_logger(), "Moving to turn start position (dir=%+d)...", turn_direction_);

    locomotion_->walk();   

    for (int i = 0; i < 6; ++i) {
        double th1, th2, th3;
        if (locomotion_->isSwingPhase(i)) {
            sampleTurnSwingAt(i, 0.0, th1, th2, th3);
        } else {
            sampleTurnDragAt(i, 0.0, th1, th2, th3);
        }
        last_sent_angles_[i*3+0] = th1;
        last_sent_angles_[i*3+1] = th2;
        last_sent_angles_[i*3+2] = th3;
        sendLegTrajectory(i, th1, th2, th3, 2.0);
    }

    rclcpp::sleep_for(std::chrono::milliseconds(2200));
    standby_done_ = true;
    RCLCPP_INFO(get_logger(), "Starting turn-in-place gait...");
}

void HexapodTurnNode::gaitUpdate()
{
    if (!standby_done_) return;

    const double dt = 1.0 / update_rate_;
    locomotion_->update(dt);

    int current_block = locomotion_->getGaitBlock();
    if (current_block == last_sent_block_) return;
    last_sent_block_ = current_block;

    if (current_block != 0 && current_block != 3) return;

    RCLCPP_INFO(get_logger(), "Block %d — swing: %s  stance: %s",
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
            RCLCPP_INFO(get_logger(), "Leg %d TURN-SWING:", i);

            // Anchor point at t=0 — ensures controller starts from current pose,
            // preventing the compressed-lift bug on legs 1,3,5.
            {
                double th1, th2, th3;
                sampleTurnSwingAt(i, 0.0, th1, th2, th3);
                trajectory_msgs::msg::JointTrajectoryPoint pt;
                pt.positions       = {th1, th2, th3};
                pt.time_from_start = rclcpp::Duration::from_seconds(0.0);
                goal.trajectory.points.push_back(pt);
            }

            for (int k = 1; k <= N; ++k) {
                double global_t = static_cast<double>(k) / N;
                double th1, th2, th3;
                sampleTurnSwingAt(i, global_t, th1, th2, th3);
                RCLCPP_INFO(get_logger(),
                    "  t=%.2f  coxa=%.3f(%.1f°)  femur=%.3f(%.1f°)  tibia=%.3f(%.1f°)",
                    global_t,
                    th1, th1*180.0/M_PI,
                    th2, th2*180.0/M_PI,
                    th3, th3*180.0/M_PI);

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

            if (std::abs(th1) < 1e-6 && std::abs(th2) < 1e-6 && std::abs(th3) < 1e-6) {
                sampleTurnDragAt(i, 0.0, th1, th2, th3);
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

void HexapodTurnNode::sampleTurnSwingAt(int leg_index, double global_t,
                                         double& theta1, double& theta2, double& theta3)
{
    global_t = std::max(0.0, std::min(1.0, global_t));

    const double half = step_length_ / 2.0;
    const double u    = 1.0 - global_t;

    const double y_fwd = u*u*(-half) + global_t*global_t*(half);
    const double foot_y = static_cast<double>(turn_direction_) * y_fwd;

    // Bezier height arc:  0 → step_height_ → 0
    const double foot_z = home_z_ + 2.0 * u * global_t * step_height_;

    vx01_hexapod_locomotion::kinematics::InverseKinematics ik(L1_, L2_, L3_);
    double th1 = 0.0, th2 = 0.0, th3 = 0.0;
    bool ok = ik.compute(home_x_, foot_y, foot_z, th1, th2, th3);
    if (!ok) {
        RCLCPP_WARN(get_logger(),
            "[sampleTurnSwingAt] IK failed leg=%d t=%.2f foot=(%.1f,%.1f,%.1f)",
            leg_index, global_t, home_x_, foot_y, foot_z);
        th1 = last_sent_angles_[leg_index*3+0];
        th2 = last_sent_angles_[leg_index*3+1];
        th3 = last_sent_angles_[leg_index*3+2];
    }
    theta1 = th1;
    theta2 = th2;
    theta3 = th3;
}

void HexapodTurnNode::sampleTurnDragAt(int leg_index, double global_t,
                                        double& theta1, double& theta2, double& theta3)
{
    global_t = std::max(0.0, std::min(1.0, global_t));

    const double half = step_length_ / 2.0;

    const double y_fwd = half - 2.0 * half * global_t;
    const double foot_y = static_cast<double>(turn_direction_) * y_fwd;

    vx01_hexapod_locomotion::kinematics::InverseKinematics ik(L1_, L2_, L3_);
    double th1 = 0.0, th2 = 0.0, th3 = 0.0;
    bool ok = ik.compute(home_x_, foot_y, home_z_, th1, th2, th3);
    if (!ok) {
        RCLCPP_WARN(get_logger(),
            "[sampleTurnDragAt] IK failed leg=%d t=%.2f y=%.1f",
            leg_index, global_t, foot_y);
        th1 = last_sent_angles_[leg_index*3+0];
        th2 = last_sent_angles_[leg_index*3+1];
        th3 = last_sent_angles_[leg_index*3+2];
    }
    theta1 = th1;
    theta2 = th2;
    theta3 = th3;
}

void HexapodTurnNode::sendLegTrajectory(int leg_index,
                                         double theta1, double theta2, double theta3,
                                         double duration_sec)
{
    if (!action_clients_[leg_index]->action_server_is_ready()) return;

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

bool HexapodTurnNode::allClientsReady()
{
    for (auto& client : action_clients_)
        if (!client->action_server_is_ready()) return false;
    return true;
}

}  

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<vx01_locomotion_control::HexapodTurnNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}