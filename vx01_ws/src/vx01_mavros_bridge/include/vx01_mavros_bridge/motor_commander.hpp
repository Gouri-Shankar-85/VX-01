#ifndef VX01_MAVROS_BRIDGE_MOTOR_COMMANDER_HPP
#define VX01_MAVROS_BRIDGE_MOTOR_COMMANDER_HPP

#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/attitude_target.hpp>
#include <mavros_msgs/msg/rc_in.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/float64.hpp>

namespace vx01_mavros_bridge {

    class MotorCommander {
        
        private:
            rclcpp::Node::SharedPtr node_;

            // Publishers
            rclcpp::Publisher<mavros_msgs::msg::AttitudeTarget>::SharedPtr
                attitude_pub_;

            // Current commands
            double roll_;
            double pitch_;
            double yaw_;
            double thrust_;

            // State
            bool is_active_;

        public:
            // Constructor
            MotorCommander(rclcpp::Node::SharedPtr node);

            // Activate/Deactivate
            void activate();
            void deactivate();
            bool isActive() const;

            // Set attitude commands
            void setRoll(double roll);
            void setPitch(double pitch);
            void setYaw(double yaw);
            void setThrust(double thrust);

            // Set all at once
            void setAttitudeThrust(double roll, double pitch,
                                double yaw, double thrust);

            // Send hover command (maintain position)
            void hover();

            // Send landing command (reduce thrust gradually)
            void land();

            // Emergency stop (cut motors)
            void emergencyStop();

            // Publish current commands
            void publishCommands();

            // Getters
            double getRoll() const;
            double getPitch() const;
            double getYaw() const;
            double getThrust() const;
    };

} 

#endif