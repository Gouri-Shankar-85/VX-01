#include "vx01_mavros_bridge/motor_commander.hpp"
#include <tf2/LinearMath/Quaternion.h>

namespace vx01_mavros_bridge {

    MotorCommander::MotorCommander(rclcpp::Node::SharedPtr node)
        : node_(node),
        roll_(0.0), pitch_(0.0),
        yaw_(0.0), thrust_(0.0),
        is_active_(false) {

        // Create attitude publisher
        attitude_pub_ = node_->create_publisher
            mavros_msgs::msg::AttitudeTarget>(
            "/vx01/mavros/setpoint_raw/attitude", 10);

        RCLCPP_INFO(node_->get_logger(),
            "MotorCommander initialized");
    }

    void MotorCommander::activate() {
        is_active_ = true;
        RCLCPP_INFO(node_->get_logger(),
            "MotorCommander activated");
    }

    void MotorCommander::deactivate() {
        // Safe stop before deactivate
        emergencyStop();
        is_active_ = false;
        RCLCPP_INFO(node_->get_logger(),
            "MotorCommander deactivated");
    }

    bool MotorCommander::isActive() const {
        return is_active_;
    }

    void MotorCommander::setRoll(double roll) {
        roll_ = roll;
    }

    void MotorCommander::setPitch(double pitch) {
        pitch_ = pitch;
    }

    void MotorCommander::setYaw(double yaw) {
        yaw_ = yaw;
    }

    void MotorCommander::setThrust(double thrust) {
        // Clamp thrust to 0.0 - 1.0
        thrust_ = std::max(0.0, std::min(1.0, thrust));
    }

    void MotorCommander::setAttitudeThrust(double roll, double pitch,
                                            double yaw, double thrust) {
        setRoll(roll);
        setPitch(pitch);
        setYaw(yaw);
        setThrust(thrust);
    }

    void MotorCommander::hover() {
        // Level attitude with 50% thrust
        setAttitudeThrust(0.0, 0.0, yaw_, 0.5);
        publishCommands();
        RCLCPP_INFO(node_->get_logger(), "Hovering");
    }

    void MotorCommander::land() {
        // Gradually reduce thrust
        setAttitudeThrust(0.0, 0.0, yaw_, 0.3);
        publishCommands();
        RCLCPP_INFO(node_->get_logger(), "Landing");
    }

    void MotorCommander::emergencyStop() {
        // Cut all thrust immediately
        setAttitudeThrust(0.0, 0.0, 0.0, 0.0);
        publishCommands();
        RCLCPP_WARN(node_->get_logger(), "EMERGENCY STOP");
    }

    void MotorCommander::publishCommands() {
        if (!is_active_) {
            return;
        }

        auto msg = mavros_msgs::msg::AttitudeTarget();

        // Set timestamp
        msg.header.stamp = node_->now();
        msg.header.frame_id = "base_link";

        // Ignore body rate fields
        // Use attitude + thrust only
        msg.type_mask =
            mavros_msgs::msg::AttitudeTarget::IGNORE_ROLL_RATE |
            mavros_msgs::msg::AttitudeTarget::IGNORE_PITCH_RATE |
            mavros_msgs::msg::AttitudeTarget::IGNORE_YAW_RATE;

        // Convert roll pitch yaw to quaternion
        tf2::Quaternion q;
        q.setRPY(roll_, pitch_, yaw_);

        msg.orientation.x = q.x();
        msg.orientation.y = q.y();
        msg.orientation.z = q.z();
        msg.orientation.w = q.w();

        // Set thrust (0.0 to 1.0)
        msg.thrust = static_cast<float>(thrust_);

        attitude_pub_->publish(msg);
    }

    double MotorCommander::getRoll() const {
        return roll_;
    }

    double MotorCommander::getPitch() const {
        return pitch_;
    }

    double MotorCommander::getYaw() const {
        return yaw_;
    }

    double MotorCommander::getThrust() const {
        return thrust_;
    }

} 