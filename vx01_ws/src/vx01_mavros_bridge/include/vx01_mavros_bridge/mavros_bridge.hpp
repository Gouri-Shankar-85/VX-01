#ifndef VX01_MAVROS_BRIDGE_MAVROS_BRIDGE_HPP
#define VX01_MAVROS_BRIDGE_MAVROS_BRIDGE_HPP

#include <rclcpp/rclcpp.hpp>
#include "vx01_mavros_bridge/flight_mode_manager.hpp"
#include "vx01_mavros_bridge/motor_commander.hpp"
#include "vx01_mavros_bridge/telemetry_handler.hpp"
#include <vx01_msgs/msg/drone_state.hpp>
#include <vx01_msgs/msg/motor_thrust.hpp>

namespace vx01_mavros_bridge {

    class MavrosBridge : public rclcpp::Node {
    
        private:

            // Core components
            std::shared_ptr<FlightModeManager> flight_mode_manager_;
            std::shared_ptr<MotorCommander> motor_commander_;
            std::shared_ptr<TelemetryHandler> telemetry_handler_;

            // Subscribers (from ROS2 controllers)
            rclcpp::Subscription<vx01_msgs::msg::MotorThrust>::SharedPtr
                thrust_cmd_sub_;

            // Publishers (to ROS2 system)
            rclcpp::Publisher<vx01_msgs::msg::DroneState>::SharedPtr
                drone_state_pub_;

            // Timers
            rclcpp::TimerBase::SharedPtr telemetry_timer_;
            rclcpp::TimerBase::SharedPtr command_timer_;

            // State
            bool is_initialized_;

        public:
            // Constructor
            MavrosBridge();

            // Destructor
            ~MavrosBridge();

            // Initialize
            bool initialize();

            // Shutdown
            void shutdown();

        private:
            // Timer callbacks
            void telemetryTimerCallback();
            void commandTimerCallback();

            // Subscriber callbacks
            void thrustCommandCallback(
                const vx01_msgs::msg::MotorThrust::SharedPtr msg);

            // Publish drone state
            void publishDroneState();
    };

    } 

#endif