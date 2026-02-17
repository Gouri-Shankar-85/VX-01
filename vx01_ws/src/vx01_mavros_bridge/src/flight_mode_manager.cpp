#include "vx01_mavros_bridge/flight_mode_manager.hpp"

namespace vx01_mavros_bridge {

    FlightModeManager::FlightModeManager(rclcpp::Node::SharedPtr node)
        : node_(node), is_connected_(false),
        is_armed_(false), current_mode_(FlightMode::UNKNOWN) {

        // Create service clients
        arming_client_ = node_->create_client<mavros_msgs::srv::CommandBool>(
            "/vx01/mavros/cmd/arming");

        set_mode_client_ = node_->create_client<mavros_msgs::srv::SetMode>(
            "/vx01/mavros/set_mode");

        // Subscribe to state
        state_sub_ = node_->create_subscription<mavros_msgs::msg::State>(
            "/vx01/mavros/state", 10,
            std::bind(&FlightModeManager::stateCallback,
                    this, std::placeholders::_1));

        RCLCPP_INFO(node_->get_logger(),
            "FlightModeManager initialized");
    }

    bool FlightModeManager::arm() {
        if (!is_connected_) {
            RCLCPP_ERROR(node_->get_logger(),
                "Cannot arm: not connected to Pix6");
            return false;
        }

        auto request = std::make_shared
            mavros_msgs::srv::CommandBool::Request>();
        request->value = true;

        auto future = arming_client_->async_send_request(request);

        if (rclcpp::spin_until_future_complete(node_, future) ==
            rclcpp::FutureReturnCode::SUCCESS) {
            if (future.get()->success) {
                RCLCPP_INFO(node_->get_logger(), "Armed successfully");
                is_armed_ = true;
                return true;
            }
        }

        RCLCPP_ERROR(node_->get_logger(), "Failed to arm");
        return false;
    }

    bool FlightModeManager::disarm() {
        if (!is_connected_) {
            RCLCPP_ERROR(node_->get_logger(),
                "Cannot disarm: not connected to Pix6");
            return false;
        }

        auto request = std::make_shared
            mavros_msgs::srv::CommandBool::Request>();
        request->value = false;

        auto future = arming_client_->async_send_request(request);

        if (rclcpp::spin_until_future_complete(node_, future) ==
            rclcpp::FutureReturnCode::SUCCESS) {
            if (future.get()->success) {
                RCLCPP_INFO(node_->get_logger(), "Disarmed successfully");
                is_armed_ = false;
                return true;
            }
        }

        RCLCPP_ERROR(node_->get_logger(), "Failed to disarm");
        return false;
    }

    bool FlightModeManager::setMode(FlightMode mode) {
        return setModeByString(modeToString(mode));
    }

    bool FlightModeManager::setModeByString(const std::string& mode_str) {
        if (!is_connected_) {
            RCLCPP_ERROR(node_->get_logger(),
                "Cannot set mode: not connected to Pix6");
            return false;
        }

        auto request = std::make_shared
            mavros_msgs::srv::SetMode::Request>();
        request->custom_mode = mode_str;

        auto future = set_mode_client_->async_send_request(request);

        if (rclcpp::spin_until_future_complete(node_, future) ==
            rclcpp::FutureReturnCode::SUCCESS) {
            if (future.get()->mode_sent) {
                RCLCPP_INFO(node_->get_logger(),
                    "Mode set to: %s", mode_str.c_str());
                current_mode_ = stringToMode(mode_str);
                return true;
            }
        }

        RCLCPP_ERROR(node_->get_logger(),
            "Failed to set mode: %s", mode_str.c_str());
        return false;
    }

    bool FlightModeManager::isConnected() const {
        return is_connected_;
    }

    bool FlightModeManager::isArmed() const {
        return is_armed_;
    }

    FlightMode FlightModeManager::getCurrentMode() const {
        return current_mode_;
    }

    std::string FlightModeManager::getCurrentModeString() const {
        return modeToString(current_mode_);
    }

    mavros_msgs::msg::State FlightModeManager::getCurrentState() const {
        return current_state_;
    }

    std::string FlightModeManager::modeToString(FlightMode mode) {
        switch (mode) {
            case FlightMode::STABILIZE: return "STABILIZE";
            case FlightMode::ALT_HOLD:  return "ALT_HOLD";
            case FlightMode::LOITER:    return "LOITER";
            case FlightMode::GUIDED:    return "GUIDED";
            case FlightMode::LAND:      return "LAND";
            case FlightMode::RTL:       return "RTL";
            default:                    return "UNKNOWN";
        }
    }

    FlightMode FlightModeManager::stringToMode(const std::string& mode_str) {
        if (mode_str == "STABILIZE") return FlightMode::STABILIZE;
        if (mode_str == "ALT_HOLD")  return FlightMode::ALT_HOLD;
        if (mode_str == "LOITER")    return FlightMode::LOITER;
        if (mode_str == "GUIDED")    return FlightMode::GUIDED;
        if (mode_str == "LAND")      return FlightMode::LAND;
        if (mode_str == "RTL")       return FlightMode::RTL;
        return FlightMode::UNKNOWN;
    }

    void FlightModeManager::stateCallback(
        const mavros_msgs::msg::State::SharedPtr msg) {
        current_state_ = *msg;
        is_connected_ = msg->connected;
        is_armed_ = msg->armed;
        current_mode_ = stringToMode(msg->mode);
    }

} 