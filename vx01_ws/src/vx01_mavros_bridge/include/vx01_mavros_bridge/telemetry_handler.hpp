#ifndef VX01_MAVROS_BRIDGE_TELEMETRY_HANDLER_HPP
#define VX01_MAVROS_BRIDGE_TELEMETRY_HANDLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64.hpp>

namespace vx01_mavros_bridge {

    class TelemetryHandler {
        private:
            rclcpp::Node::SharedPtr node_;

            // Subscribers
            rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr
                state_sub_;
            rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr
                battery_sub_;
            rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr
                gps_sub_;
            rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
                local_pos_sub_;
            rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr
                altitude_sub_;

            // Current telemetry data
            mavros_msgs::msg::State current_state_;
            sensor_msgs::msg::BatteryState current_battery_;
            sensor_msgs::msg::NavSatFix current_gps_;
            geometry_msgs::msg::PoseStamped current_local_pos_;
            double current_altitude_;

            // Connection flags
            bool state_received_;
            bool battery_received_;
            bool gps_received_;
            bool position_received_;

        public:
            // Constructor
            TelemetryHandler(rclcpp::Node::SharedPtr node);

            // Getters for state
            bool isConnected() const;
            bool isArmed() const;
            std::string getFlightMode() const;

            // Getters for battery
            double getBatteryVoltage() const;
            double getBatteryPercentage() const;
            bool isBatteryLow() const;

            // Getters for GPS
            double getLatitude() const;
            double getLongitude() const;
            double getGpsAltitude() const;
            bool isGpsFixed() const;

            // Getters for local position
            double getLocalX() const;
            double getLocalY() const;
            double getLocalZ() const;

            // Getters for altitude
            double getAltitude() const;

            // Check if data received
            bool isStateReceived() const;
            bool isBatteryReceived() const;
            bool isGpsReceived() const;
            bool isPositionReceived() const;

            // Print telemetry summary
            void printTelemetry() const;

        private:
            // Callbacks
            void stateCallback(
                const mavros_msgs::msg::State::SharedPtr msg);

            void batteryCallback(
                const sensor_msgs::msg::BatteryState::SharedPtr msg);

            void gpsCallback(
                const sensor_msgs::msg::NavSatFix::SharedPtr msg);

            void localPositionCallback(
                const geometry_msgs::msg::PoseStamped::SharedPtr msg);

            void altitudeCallback(
                const std_msgs::msg::Float64::SharedPtr msg);
    };

} 
#endif