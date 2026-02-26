#include "vx01_hexapod_hardware/hexapod_hardware_interface.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>

namespace vx01_hexapod_hardware {

    HexapodHardwareInterface::HexapodHardwareInterface()
        : logger_(rclcpp::get_logger("HexapodHardwareInterface")) {}

    HexapodHardwareInterface::~HexapodHardwareInterface() {
        if (serial_ && serial_->isOpen()) {
            disconnectFromHardware();
        }
    }

    hardware_interface::CallbackReturn HexapodHardwareInterface::on_init(
        const hardware_interface::HardwareInfo& info)
    {
        if (hardware_interface::SystemInterface::on_init(info) !=
            hardware_interface::CallbackReturn::SUCCESS) {
            return hardware_interface::CallbackReturn::ERROR;
        }

        serial_port_ = info_.hardware_parameters.at("serial_port");
        baud_rate_   = std::stoi(info_.hardware_parameters.at("baud_rate"));

        RCLCPP_INFO(logger_, "Serial port: %s  baud: %d",
                    serial_port_.c_str(), baud_rate_);

        hw_positions_.resize(info_.joints.size(), 0.0);
        hw_velocities_.resize(info_.joints.size(), 0.0);
        hw_efforts_.resize(info_.joints.size(), 0.0);
        hw_commands_.resize(info_.joints.size(), 0.0);

        for (const auto& joint : info_.joints) {
            joint_names_.push_back(joint.name);
            RCLCPP_INFO(logger_, "Joint: %s", joint.name.c_str());
        }

        // Build serial + protocol + controller objects here (no connection yet)
        serial_   = std::make_shared<communication::SerialInterface>(serial_port_, baud_rate_);
        maestro_  = std::make_shared<communication::MaestroProtocol>(serial_);
        servo_controller_ = std::make_shared<servo::ServoController>(maestro_);

        if (!loadServoConfigurations()) {
            RCLCPP_ERROR(logger_, "Failed to load servo configurations");
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(logger_, "on_init OK — %zu joints", joint_names_.size());
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn HexapodHardwareInterface::on_configure(
        const rclcpp_lifecycle::State&)
    {
        RCLCPP_INFO(logger_, "on_configure");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn HexapodHardwareInterface::on_cleanup(
        const rclcpp_lifecycle::State&)
    {
        RCLCPP_INFO(logger_, "on_cleanup");
        disconnectFromHardware();
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn HexapodHardwareInterface::on_activate(
        const rclcpp_lifecycle::State&)
    {
        RCLCPP_INFO(logger_, "on_activate — connecting to Maestro");

        if (!connectToHardware()) {
            RCLCPP_ERROR(logger_, "Failed to connect to Maestro on %s", serial_port_.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (!servo_controller_->initialize()) {
            RCLCPP_ERROR(logger_, "Servo initialisation failed");
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Read current hardware positions and sync commands to them
        if (!servo_controller_->readPositions()) {
            RCLCPP_WARN(logger_, "Could not read initial positions — defaulting to 0");
        }

        for (size_t i = 0; i < joint_names_.size(); ++i) {
            hw_positions_[i] = servo_controller_->getJointAngleByIndex(i);
            hw_commands_[i]  = hw_positions_[i];
        }

        RCLCPP_INFO(logger_, "Hardware activated");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn HexapodHardwareInterface::on_deactivate(
        const rclcpp_lifecycle::State&)
    {
        RCLCPP_INFO(logger_, "on_deactivate");
        servo_controller_->goHome();
        disconnectFromHardware();
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface>
    HexapodHardwareInterface::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> si;
        for (size_t i = 0; i < joint_names_.size(); ++i) {
            si.emplace_back(joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
            si.emplace_back(joint_names_[i], hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
            si.emplace_back(joint_names_[i], hardware_interface::HW_IF_EFFORT,   &hw_efforts_[i]);
        }
        return si;
    }

    std::vector<hardware_interface::CommandInterface>
    HexapodHardwareInterface::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> ci;
        for (size_t i = 0; i < joint_names_.size(); ++i) {
            ci.emplace_back(joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_commands_[i]);
        }
        return ci;
    }

    hardware_interface::return_type HexapodHardwareInterface::read(
        const rclcpp::Time&, const rclcpp::Duration& period)
    {
        if (!servo_controller_->readPositions()) {
            RCLCPP_WARN_THROTTLE(logger_, *rclcpp::Clock::make_shared(), 2000,
                                 "Failed to read servo positions");
            return hardware_interface::return_type::OK;  // Don't error — keep controller alive
        }

        double dt = period.seconds();
        for (size_t i = 0; i < joint_names_.size(); ++i) {
            double new_pos = servo_controller_->getJointAngleByIndex(i);
            hw_velocities_[i] = (dt > 1e-9) ? (new_pos - hw_positions_[i]) / dt : 0.0;
            hw_positions_[i]  = new_pos;
            hw_efforts_[i]    = 0.0;
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type HexapodHardwareInterface::write(
        const rclcpp::Time&, const rclcpp::Duration&)
    {
        for (size_t i = 0; i < joint_names_.size(); ++i) {
            servo_controller_->setJointAngleByIndex(i, hw_commands_[i]);
        }

        if (!servo_controller_->writeCommands()) {
            RCLCPP_WARN_THROTTLE(logger_, *rclcpp::Clock::make_shared(), 2000,
                                 "Failed to write servo commands");
        }

        return hardware_interface::return_type::OK;
    }

    bool HexapodHardwareInterface::loadServoConfigurations()
    {
        // Load servo_mapping.yaml from the vx01_bringup package config folder
        std::string pkg_share;
        try {
            pkg_share = ament_index_cpp::get_package_share_directory("vx01_bringup");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(logger_, "Cannot find vx01_bringup package: %s", e.what());
            return false;
        }

        std::string yaml_path = pkg_share + "/config/hexapod/servo_mapping.yaml";
        if (!std::filesystem::exists(yaml_path)) {
            RCLCPP_ERROR(logger_, "servo_mapping.yaml not found at: %s", yaml_path.c_str());
            return false;
        }

        RCLCPP_INFO(logger_, "Loading servo map from: %s", yaml_path.c_str());

        YAML::Node root = YAML::LoadFile(yaml_path);
        YAML::Node mapping = root["servo_mapping"];

        if (!mapping) {
            RCLCPP_ERROR(logger_, "No 'servo_mapping' key in YAML");
            return false;
        }

        for (const auto& joint_name : joint_names_) {
            if (!mapping[joint_name]) {
                RCLCPP_ERROR(logger_, "No servo mapping for joint: %s", joint_name.c_str());
                return false;
            }

            YAML::Node cfg = mapping[joint_name];

            int    channel         = cfg["channel"].as<int>();
            double min_pulse       = cfg["min_pulse"].as<double>();
            double max_pulse       = cfg["max_pulse"].as<double>();
            double min_angle       = cfg["min_angle"].as<double>();
            double max_angle       = cfg["max_angle"].as<double>();
            double offset          = cfg["offset"].as<double>(0.0);
            bool   reversed        = cfg["reversed"].as<bool>(false);
            double max_speed       = cfg["max_speed"].as<double>(0.0);
            double max_accel       = cfg["max_acceleration"].as<double>(0.0);

            servo::ServoConfig config(
                channel, joint_name,
                min_pulse, max_pulse,
                min_angle, max_angle,
                offset, reversed,
                max_speed, max_accel
            );

            servo_controller_->addServo(config);

            RCLCPP_INFO(logger_, "  %-22s → channel %2d  reversed=%s  offset=%.3f",
                        joint_name.c_str(), channel,
                        reversed ? "true" : "false", offset);
        }

        return true;
    }

    bool HexapodHardwareInterface::connectToHardware()
    {
        if (!serial_->open()) {
            RCLCPP_ERROR(logger_, "Cannot open serial port: %s", serial_port_.c_str());
            return false;
        }

        uint16_t errors = 0;
        if (maestro_->getErrors(errors) && errors != 0) {
            RCLCPP_WARN(logger_, "Maestro startup errors: 0x%04X", errors);
        }

        RCLCPP_INFO(logger_, "Connected to Maestro on %s", serial_port_.c_str());
        return true;
    }

    void HexapodHardwareInterface::disconnectFromHardware()
    {
        if (serial_ && serial_->isOpen()) {
            serial_->close();
        }
    }

}

PLUGINLIB_EXPORT_CLASS(
    vx01_hexapod_hardware::HexapodHardwareInterface,
    hardware_interface::SystemInterface
)