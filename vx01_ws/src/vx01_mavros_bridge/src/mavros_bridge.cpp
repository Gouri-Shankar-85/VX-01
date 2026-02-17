#include "vx01_mavros_bridge/mavros_bridge.hpp"
#include <thread>
#include <chrono>

namespace vx01_mavros_bridge {

MavrosBridge::MavrosBridge()
    : Node("mavros_bridge_node"),
      is_initialized_(false) {

    RCLCPP_INFO(get_logger(), "Creating MavrosBridge node...");

    flight_mode_manager_ = std::make_shared<FlightModeManager>(shared_from_this());
    motor_commander_ = std::make_shared<MotorCommander>(shared_from_this());
    telemetry_handler_ = std::make_shared<TelemetryHandler>(shared_from_this());

    thrust_cmd_sub_ = create_subscription<vx01_msgs::msg::MotorThrust>(
        "/vx01/drone/thrust_command", 10,
        std::bind(&MavrosBridge::thrustCommandCallback,
                  this, std::placeholders::_1));

    drone_state_pub_ = create_publisher<vx01_msgs::msg::DroneState>(
        "/vx01/drone/state", 10);

    telemetry_timer_ = create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&MavrosBridge::telemetryTimerCallback, this));

    command_timer_ = create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&MavrosBridge::commandTimerCallback, this));

    RCLCPP_INFO(get_logger(), "MavrosBridge node created");
}

MavrosBridge::~MavrosBridge() {
    shutdown();
}

bool MavrosBridge::initialize() {
    RCLCPP_INFO(get_logger(), "Initializing MavrosBridge...");
    RCLCPP_INFO(get_logger(), "Waiting for Pix6 connection...");

    int retry = 0;
    while (!telemetry_handler_->isConnected() && retry < 30) {
        rclcpp::spin_some(shared_from_this());
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        retry++;
        RCLCPP_INFO(get_logger(), "Waiting... (%d/30)", retry);
    }

    if (!telemetry_handler_->isConnected()) {
        RCLCPP_ERROR(get_logger(), "Failed to connect to Pix6");
        return false;
    }

    RCLCPP_INFO(get_logger(), "Connected to Pix6!");
    motor_commander_->activate();
    is_initialized_ = true;

    RCLCPP_INFO(get_logger(), "MavrosBridge initialized successfully");
    return true;
}

void MavrosBridge::shutdown() {
    RCLCPP_INFO(get_logger(), "Shutting down MavrosBridge...");

    if (motor_commander_) {
        motor_commander_->deactivate();
    }

    if (flight_mode_manager_ && flight_mode_manager_->isArmed()) {
        flight_mode_manager_->disarm();
    }

    is_initialized_ = false;
    RCLCPP_INFO(get_logger(), "MavrosBridge shutdown complete");
}

void MavrosBridge::telemetryTimerCallback() {
    if (!is_initialized_) { return; }

    publishDroneState();

    if (telemetry_handler_->isBatteryLow()) {
        RCLCPP_WARN(get_logger(), "LOW BATTERY: %.1f%%",
            telemetry_handler_->getBatteryPercentage());

        if (telemetry_handler_->getBatteryPercentage() < 10.0) {
            RCLCPP_ERROR(get_logger(), "CRITICAL BATTERY! Auto landing...");
            flight_mode_manager_->setMode(FlightMode::LAND);
        }
    }
}

void MavrosBridge::commandTimerCallback() {
    if (!is_initialized_) { return; }
    motor_commander_->publishCommands();
}

void MavrosBridge::thrustCommandCallback(
    const vx01_msgs::msg::MotorThrust::SharedPtr msg) {
    if (!is_initialized_) { return; }
    motor_commander_->setAttitudeThrust(
        msg->roll, msg->pitch, msg->yaw, msg->thrust);
}

void MavrosBridge::publishDroneState() {
    auto msg = vx01_msgs::msg::DroneState();
    msg.header.stamp = now();
    msg.header.frame_id = "base_link";
    msg.connected = telemetry_handler_->isConnected();
    msg.armed = telemetry_handler_->isArmed();
    msg.flight_mode = telemetry_handler_->getFlightMode();
    msg.battery_voltage = telemetry_handler_->getBatteryVoltage();
    msg.battery_percentage = telemetry_handler_->getBatteryPercentage();
    msg.local_x = telemetry_handler_->getLocalX();
    msg.local_y = telemetry_handler_->getLocalY();
    msg.local_z = telemetry_handler_->getLocalZ();
    msg.altitude = telemetry_handler_->getAltitude();
    msg.latitude = telemetry_handler_->getLatitude();
    msg.longitude = telemetry_handler_->getLongitude();
    msg.gps_fixed = telemetry_handler_->isGpsFixed();
    drone_state_pub_->publish(msg);
}

} // namespace vx01_mavros_bridge

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<vx01_mavros_bridge::MavrosBridge>();

    if (!node->initialize()) {
        RCLCPP_ERROR(rclcpp::get_logger("main"),
            "Failed to initialize MavrosBridge");
        return 1;
    }

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
