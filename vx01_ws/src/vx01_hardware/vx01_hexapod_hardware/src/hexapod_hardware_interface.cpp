#include "vx01_hexapod_hardware/hexapod_hardware_interface.hpp"
#include <pluginlib/class_list_macros.hpp>

namespace vx01_hexapod_hardware {


    struct HWData {
        std::vector<double> positions_;
    };

    HexapodHardwareInterface::HexapodHardwareInterface()
        : logger_(rclcpp::get_logger("HexapodHardwareINterface")) {}

    HexapodHardwareInterface::~HexapodHardwareInterface() {
        if (serial_ && serial_->isOpen()) {
            disconnectFromHardware();
        }
    }

    hardware_interface::CallbackReturn HexapodHardwareInterface::on_init(
        const hardware_interface::HardwareInfo& info) {

        if (hardware_interface::SystemInterface::on_init(info) !=
            hardware_interface::CallbackReturn::SUCCESS) {

            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(logger_, "Initializing Hexapod Hardware Interface...");

        serial_port_ = info_.hardware_parameters["serial_port"];
        baud_rate_ = std::stoi(info_.hardware_parameters["baud_rate"]);

        RCLCPP_INFO(logger_, "Serial port: %s", serial_port_.c_str());
        RCLCPP_INFO(logger_, "Baud rate: %d", baud_rate_);

        hw_positions_.resize(info_.joints.size(), 0.0);
        hw_velocities_.resize(info_.joints.size(), 0.0);
        hw_efforts_.resize(info_.joints.size(), 0.0);
        hw_commands_.resize(info_.joints.size(), 0.0);

        for (const auto& joint : info_.joints) {
            joint_names_.push_back(joint.name);
            RCLCPP_INFO(logger_, "Found joint: %s", joint.name.c_str());
        }

        RCLCPP_INFO(logger_, "Total joints: %zu", joint_names_.size());

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn HexapodHardwareInterface::on_cleanup(
        const rclcpp_lifecycle::State&) {

        RCLCPP_INFO(logger_, "Cleaning up Hexapod Hardware Interface...");

        disconnectFromHardware();

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn HexapodHardwareInterface::on_activate(
        const rclcpp_lifecycle::State&) {

        RCLCPP_INFO(logger_, "Activating Hexapod Hardware Interface...");

        if (!connectToHardware()) {
            RCLCPP_ERROR(logger_, "Failed to connect to Hardware");
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (!servo_controller_->initialize()) {
            RCLCPP_ERROR(logger_, "Failed to initialize servos");
            return hardware_interface::CallbackReturn::ERROR;
        }
        
        if (!servo_controller_->goHome()) {
            RCLCPP_ERROR(logger_, "Failed to move to home position");
            return hardware_interface::CallbackReturn::ERROR;
        }
        
        if (!servo_controller_->readPositions()) {
            RCLCPP_ERROR(logger_, "Failed to read initial positions");
            return hardware_interface::CallbackReturn::ERROR;
        }
        
        for (size_t i = 0; i < joint_names_.size(); i++) {
            hw_positions_[i] = servo_controller_->getJointAngleByIndex(i);
            hw_commands_[i] = hw_positions_[i];
        }

        RCLCPP_INFO(logger_, "Hexapod Hardware Interface activated");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn HexapodHardwareInterface::on_deactivate(
        const rclcpp_lifecycle::State& ) {
        
        RCLCPP_INFO(logger_, "Deactivating Hexapod Hardware Interface...");
        
        servo_controller_->goHome();
        
        disconnectFromHardware();
        
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> 
    HexapodHardwareInterface::export_state_interfaces() {
        
        std::vector<hardware_interface::StateInterface> state_interfaces;
        
        for (size_t i = 0; i < joint_names_.size(); i++) {

            state_interfaces.emplace_back(
                hardware_interface::StateInterface(
                    joint_names_[i], 
                    hardware_interface::HW_IF_POSITION, 
                    &hw_positions_[i]));
            
            state_interfaces.emplace_back(
                hardware_interface::StateInterface(
                    joint_names_[i], 
                    hardware_interface::HW_IF_VELOCITY, 
                    &hw_velocities_[i]));
            
            state_interfaces.emplace_back(
                hardware_interface::StateInterface(
                    joint_names_[i], 
                    hardware_interface::HW_IF_EFFORT, 
                    &hw_efforts_[i]));
        }
        
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> 
    HexapodHardwareInterface::export_command_interfaces() {
        
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        
        for (size_t i = 0; i < joint_names_.size(); i++) {
            command_interfaces.emplace_back(
                hardware_interface::CommandInterface(
                    joint_names_[i], 
                    hardware_interface::HW_IF_POSITION, 
                    &hw_commands_[i]));
        }
        
        return command_interfaces;
    }

    hardware_interface::return_type HexapodHardwareInterface::read(
        const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
        
        if (!servo_controller_->readPositions()) {
            RCLCPP_WARN(logger_, "Failed to read servo positions");
            return hardware_interface::return_type::ERROR;
        }
        
        for (size_t i = 0; i < joint_names_.size(); i++) {
            double new_position = servo_controller_->getJointAngleByIndex(i);
            
            hw_velocities_[i] = new_position - hw_positions_[i];
            
            hw_positions_[i] = new_position;
            hw_efforts_[i] = 0.0;  
        }
        
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type HexapodHardwareInterface::write(
        const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
        
        for (size_t i = 0; i < joint_names_.size(); i++) {
            servo_controller_->setJointAngleByIndex(i, hw_commands_[i]);
        }
        
        if (!servo_controller_->writeCommands()) {
            RCLCPP_WARN(logger_, "Failed to write servo commands");
            return hardware_interface::return_type::ERROR;
        }
        
        return hardware_interface::return_type::OK;
    }

    bool HexapodHardwareInterface::loadServoConfigurations() {
        
        RCLCPP_INFO(logger_, "Loading servo configurations...");
        
        for (size_t i = 0; i < joint_names_.size(); i++) {
            const auto& joint = info_.joints[i];
            
            int servo_id = std::stoi(joint.parameters.at("servo_id"));
            double min_pulse = std::stod(joint.parameters.at("min_pulse"));
            double max_pulse = std::stod(joint.parameters.at("max_pulse"));
            double min_angle = std::stod(joint.parameters.at("min_angle"));
            double max_angle = std::stod(joint.parameters.at("max_angle"));
            double offset = std::stod(joint.parameters.at("offset"));
            bool reversed = (joint.parameters.at("reversed") == "true");
            double max_speed = std::stod(joint.parameters.at("max_speed"));
            double max_acceleration = std::stod(joint.parameters.at("max_acceleration"));
            
            servo::ServoConfig config(
                servo_id, joint.name,
                min_pulse, max_pulse,
                min_angle, max_angle,
                offset, reversed,
                max_speed, max_acceleration
            );
            
            servo_controller_->addServo(config);
            
            RCLCPP_INFO(logger_, "Loaded config for joint: %s (servo_id: %d)", 
                        joint.name.c_str(), servo_id);
        }
        
        return true;
    }

    bool HexapodHardwareInterface::connectToHardware() {
        RCLCPP_INFO(logger_, "Connecting to Pololu Maestro...");
        
        if (!serial_->open()) {
            RCLCPP_ERROR(logger_, "Failed to open serial port: %s", serial_port_.c_str());
            return false;
        }
        
        RCLCPP_INFO(logger_, "Successfully connected to Maestro");
        
        uint16_t errors;
        if (servo_controller_->getErrors(errors)) {
            if (errors != 0) {
                RCLCPP_WARN(logger_, "Maestro reported errors: 0x%04X", errors);
            }
        }
        
        return true;
    }

    void HexapodHardwareInterface::disconnectFromHardware() {
        RCLCPP_INFO(logger_, "Disconnecting from hardware...");
        
        if (serial_ && serial_->isOpen()) {
            serial_->close();
        }
    }

} 

// Export the hardware interface as a plugin
PLUGINLIB_EXPORT_CLASS(
    vx01_hexapod_hardware::HexapodHardwareInterface,
    hardware_interface::SystemInterface
)