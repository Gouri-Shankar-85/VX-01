#include <rclcpp/rclcpp.hpp>
#include <vx01_msgs/msg/robot_mode.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

class ModeManagerNode : public rclcpp::Node {
public:
    ModeManagerNode() : Node("mode_manager_node"), current_mode_("") {
        sub_mode_ = this->create_subscription<vx01_msgs::msg::RobotMode>(
            "/robot_mode", 10,
            std::bind(&ModeManagerNode::modeCallback, this, std::placeholders::_1));

        pub_arms_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/drone_arm_controller/joint_trajectory", 10);
        
        for (int i = 0; i < 6; ++i) {
            pub_legs_[i] = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
                "/leg_" + std::to_string(i) + "_controller/joint_trajectory", 10);
        }

        RCLCPP_INFO(this->get_logger(), "VX-01 Mode Manager Initialized.");
    }

private:
    void modeCallback(const vx01_msgs::msg::RobotMode::SharedPtr msg) {
        if (msg->mode == current_mode_) return;
        current_mode_ = msg->mode;
        
        RCLCPP_INFO(this->get_logger(), "Transitioning to %s mode", current_mode_.c_str());

        if (current_mode_ == "HEXAPOD") {
            publishArmTrajectory(1.57);
            RCLCPP_INFO(this->get_logger(), "Drone arms folded.");
        } 
        else if (current_mode_ == "DRONE") {
            publishArmTrajectory(0.0);
            
            for (int i = 0; i < 6; ++i) {
                publishLegTrajectory(i, 0.0, 1.57, 0.0); 
            }
            RCLCPP_INFO(this->get_logger(), "Drone arms deployed, hexapod legs folded.");
        }
    }

    void publishArmTrajectory(double angle) {
        trajectory_msgs::msg::JointTrajectory msg;
        msg.joint_names = {"arm_joint0", "arm_joint1", "arm_joint2", "arm_joint3"};
        
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = {angle, angle, angle, angle};
        point.time_from_start = rclcpp::Duration::from_seconds(2.0);
        
        msg.points.push_back(point);
        pub_arms_->publish(msg);
    }

    void publishLegTrajectory(int leg_idx, double coxa, double femur, double tibia) {
        trajectory_msgs::msg::JointTrajectory msg;
        std::string s = std::to_string(leg_idx);
        msg.joint_names = {"coxa_leg" + s + "_joint", "femur_leg" + s + "_joint", "tibia_leg" + s + "_joint"};
        
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = {coxa, femur, tibia};
        point.time_from_start = rclcpp::Duration::from_seconds(2.0);
        
        msg.points.push_back(point);
        pub_legs_[leg_idx]->publish(msg);
    }

    rclcpp::Subscription<vx01_msgs::msg::RobotMode>::SharedPtr sub_mode_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_arms_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_legs_[6];
    std::string current_mode_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ModeManagerNode>());
    rclcpp::shutdown();
    return 0;
}
