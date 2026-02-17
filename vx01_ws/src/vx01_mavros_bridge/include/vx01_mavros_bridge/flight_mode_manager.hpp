#ifndef VX01_MAVROS_BRIDGE_FLIGHT_MODE_MANAGER_HPP
#define VX01_MAVROS_BRIDGE_FLIGHT_MODE_MANAGER_HPP

#include <rclcpp/rclcpp.hpp>
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include <string>

namespace vx01_mavros_bridge {

    enum class FlightMode {
        STABILIZE,      // Manual stabilized
        ALT_HOLD,       // Altitude hold
        LOITER,         // GPS position hold
        GUIDED,         // Full autonomous
        LAND,           // Auto land
        RTL,            // Return to launch
        UNKNOWN
    };

    class FlightModeManager {
        
        private:
            rclcpp::Node::SharedPtr node_;

            // Service clients
            rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr
                arming_client_;
            rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr
                set_mode_client_;

            // Subscribers
            rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr
                state_sub_;

            // Current state
            mavros_msgs::msg::State current_state_;
            bool is_connected_;
            bool is_armed_;
            FlightMode current_mode_;

        public:
            // Constructor
            FlightModeManager(rclcpp::Node::SharedPtr node);

            // Arm/Disarm
            bool arm();
            bool disarm();

            // Set flight mode
            bool setMode(FlightMode mode);
            bool setModeByString(const std::string& mode_str);

            // Getters
            bool isConnected() const;
            bool isArmed() const;
            FlightMode getCurrentMode() const;
            std::string getCurrentModeString() const;
            mavros_msgs::msg::State getCurrentState() const;

            // Convert enum to string
            static std::string modeToString(FlightMode mode);

            // Convert string to enum
            static FlightMode stringToMode(const std::string& mode_str);

        private:
            // State callback
            void stateCallback(const mavros_msgs::msg::State::SharedPtr msg);
    };

}

#endif