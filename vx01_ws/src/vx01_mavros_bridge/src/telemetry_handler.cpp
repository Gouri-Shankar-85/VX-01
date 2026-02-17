#include "vx01_mavros_bridge/telemetry_handler.hpp"

namespace vx01_mavros_bridge {

TelemetryHandler::TelemetryHandler(rclcpp::Node::SharedPtr node)
    : node_(node),
      current_altitude_(0.0),
      state_received_(false),
      battery_received_(false),
      gps_received_(false),
      position_received_(false) {

    state_sub_ = node_->create_subscription<mavros_msgs::msg::State>(
        "/vx01/mavros/state", 10,
        std::bind(&TelemetryHandler::stateCallback,
                  this, std::placeholders::_1));

    battery_sub_ = node_->create_subscription<sensor_msgs::msg::BatteryState>(
        "/vx01/mavros/battery", 10,
        std::bind(&TelemetryHandler::batteryCallback,
                  this, std::placeholders::_1));

    gps_sub_ = node_->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/vx01/mavros/global_position/global", 10,
        std::bind(&TelemetryHandler::gpsCallback,
                  this, std::placeholders::_1));

    local_pos_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/vx01/mavros/local_position/pose", 10,
        std::bind(&TelemetryHandler::localPositionCallback,
                  this, std::placeholders::_1));

    altitude_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
        "/vx01/mavros/global_position/rel_alt", 10,
        std::bind(&TelemetryHandler::altitudeCallback,
                  this, std::placeholders::_1));

    RCLCPP_INFO(node_->get_logger(), "TelemetryHandler initialized");
}

bool TelemetryHandler::isConnected() const { return current_state_.connected; }
bool TelemetryHandler::isArmed() const { return current_state_.armed; }
std::string TelemetryHandler::getFlightMode() const { return current_state_.mode; }
double TelemetryHandler::getBatteryVoltage() const { return current_battery_.voltage; }
double TelemetryHandler::getBatteryPercentage() const { return current_battery_.percentage * 100.0; }
bool TelemetryHandler::isBatteryLow() const { return getBatteryPercentage() < 20.0; }
double TelemetryHandler::getLatitude() const { return current_gps_.latitude; }
double TelemetryHandler::getLongitude() const { return current_gps_.longitude; }
double TelemetryHandler::getGpsAltitude() const { return current_gps_.altitude; }
bool TelemetryHandler::isGpsFixed() const { return current_gps_.status.status >= 0; }
double TelemetryHandler::getLocalX() const { return current_local_pos_.pose.position.x; }
double TelemetryHandler::getLocalY() const { return current_local_pos_.pose.position.y; }
double TelemetryHandler::getLocalZ() const { return current_local_pos_.pose.position.z; }
double TelemetryHandler::getAltitude() const { return current_altitude_; }
bool TelemetryHandler::isStateReceived() const { return state_received_; }
bool TelemetryHandler::isBatteryReceived() const { return battery_received_; }
bool TelemetryHandler::isGpsReceived() const { return gps_received_; }
bool TelemetryHandler::isPositionReceived() const { return position_received_; }

void TelemetryHandler::printTelemetry() const {
    RCLCPP_INFO(node_->get_logger(), "========= Telemetry =========");
    RCLCPP_INFO(node_->get_logger(), "Connected : %s", isConnected() ? "YES" : "NO");
    RCLCPP_INFO(node_->get_logger(), "Armed     : %s", isArmed() ? "YES" : "NO");
    RCLCPP_INFO(node_->get_logger(), "Mode      : %s", getFlightMode().c_str());
    RCLCPP_INFO(node_->get_logger(), "Battery   : %.1f%% (%.2fV)",
        getBatteryPercentage(), getBatteryVoltage());
    RCLCPP_INFO(node_->get_logger(), "Altitude  : %.2f m", getAltitude());
    RCLCPP_INFO(node_->get_logger(), "Position  : X=%.2f Y=%.2f Z=%.2f",
        getLocalX(), getLocalY(), getLocalZ());
    RCLCPP_INFO(node_->get_logger(), "GPS       : Lat=%.6f Lon=%.6f",
        getLatitude(), getLongitude());
    RCLCPP_INFO(node_->get_logger(), "=============================");
}

void TelemetryHandler::stateCallback(
    const mavros_msgs::msg::State::SharedPtr msg) {
    current_state_ = *msg;
    state_received_ = true;
}

void TelemetryHandler::batteryCallback(
    const sensor_msgs::msg::BatteryState::SharedPtr msg) {
    current_battery_ = *msg;
    battery_received_ = true;
    if (isBatteryLow()) {
        RCLCPP_WARN(node_->get_logger(),
            "LOW BATTERY: %.1f%%", getBatteryPercentage());
    }
}

void TelemetryHandler::gpsCallback(
    const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    current_gps_ = *msg;
    gps_received_ = true;
}

void TelemetryHandler::localPositionCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    current_local_pos_ = *msg;
    position_received_ = true;
}

void TelemetryHandler::altitudeCallback(
    const std_msgs::msg::Float64::SharedPtr msg) {
    current_altitude_ = msg->data;
}

} // namespace vx01_mavros_bridge
