#include "vx01_drone_hardware/drone_hardware_interface.hpp"
#include <pluginlib/class_list_macros.hpp>

namespace vx01_drone_hardware {

    DroneHardwareInterface::DroneHardwareInterface()
        : logger_(rclcpp::get_logger("DroneHardwareInterface")) {}

    DroneHardwareInterface::~DroneHardwareInterface() {
        if (servo_controller_) {
            servo_controller_->shutdown();
        }
    }

    hardware_interface::CallbackReturn DroneHardwareInterface::on_init(
        const hardware_interface::HardwareInfo& info) {

        if (hardware_interface::SystemInterface::on_init(info) !=
            hardware_interface::CallbackReturn::SUCCESS) {
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(logger_, "Initializing Drone Hardware Interface...");

        // Get hardware parameters
        gpio_chip_ = info_.hardware_parameters["gpio_chip"];
        pwm_frequency_ = std::stoi(
            info_.hardware_parameters["pwm_frequency"]);

        RCLCPP_INFO(logger_, "GPIO chip: %s", gpio_chip_.c_str());
        RCLCPP_INFO(logger_, "PWM frequency: %d Hz", pwm_frequency_);

        // Initialize joint vectors
        hw_positions_.resize(info_.joints.size(), 0.0);
        hw_velocities_.resize(info_.joints.size(), 0.0);
        hw_commands_.resize(info_.joints.size(), 0.0);

        // Store joint names
        for (const auto& joint : info_.joints) {
            joint_names_.push_back(joint.name);
            RCLCPP_INFO(logger_, "Found joint: %s", joint.name.c_str());
        }

        RCLCPP_INFO(logger_, "Total joints: %zu", joint_names_.size());

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn DroneHardwareInterface::on_configure(
        const rclcpp_lifecycle::State& /*previous_state*/) {

        RCLCPP_INFO(logger_, "Configuring Drone Hardware Interface...");

        // Create servo controller
        servo_controller_ = std::make_shared<servo::ArmServoController>(
            pwm_frequency_, gpio_chip_);

        // Load servo configurations
        if (!loadServoConfigurations()) {
            RCLCPP_ERROR(logger_, "Failed to load servo configurations");
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(logger_, "Configuration complete");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn DroneHardwareInterface::on_cleanup(
        const rclcpp_lifecycle::State& /*previous_state*/) {

        RCLCPP_INFO(logger_, "Cleaning up Drone Hardware Interface...");

        if (servo_controller_) {
            servo_controller_->shutdown();
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn DroneHardwareInterface::on_activate(
        const rclcpp_lifecycle::State& /*previous_state*/) {

        RCLCPP_INFO(logger_, "Activating Drone Hardware Interface...");

        // Initialize servos
        if (!servo_controller_->initialize()) {
            RCLCPP_ERROR(logger_, "Failed to initialize arm servos");
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Move to home position
        if (!servo_controller_->goHome()) {
            RCLCPP_ERROR(logger_, "Failed to move to home position");
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Set initial state
        for (size_t i = 0; i < joint_names_.size(); i++) {
            hw_positions_[i] = servo_controller_->getJointAngleByIndex(i);
            hw_commands_[i] = hw_positions_[i];
            hw_velocities_[i] = 0.0;
        }

        RCLCPP_INFO(logger_, "Drone Hardware Interface activated");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn DroneHardwareInterface::on_deactivate(
        const rclcpp_lifecycle::State& /*previous_state*/) {

        RCLCPP_INFO(logger_, "Deactivating Drone Hardware Interface...");

        // Safe shutdown
        servo_controller_->shutdown();

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface>
    DroneHardwareInterface::export_state_interfaces() {

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
        }

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface>
    DroneHardwareInterface::export_command_interfaces() {

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

    hardware_interface::return_type DroneHardwareInterface::read(
        const rclcpp::Time& /*time*/,
        const rclcpp::Duration& /*period*/) {

        for (size_t i = 0; i < joint_names_.size(); i++) {
            double new_position =
                servo_controller_->getJointAngleByIndex(i);

            // Calculate velocity
            hw_velocities_[i] = new_position - hw_positions_[i];
            hw_positions_[i] = new_position;
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type DroneHardwareInterface::write(
        const rclcpp::Time& /*time*/,
        const rclcpp::Duration& /*period*/) {

        // Set commanded angles
        for (size_t i = 0; i < joint_names_.size(); i++) {
            servo_controller_->setJointAngleByIndex(i, hw_commands_[i]);
        }

        // Write to servos
        if (!servo_controller_->writeCommands()) {
            RCLCPP_WARN(logger_, "Failed to write servo commands");
            return hardware_interface::return_type::ERROR;
        }

        return hardware_interface::return_type::OK;
    }

    bool DroneHardwareInterface::loadServoConfigurations() {
        RCLCPP_INFO(logger_, "Loading arm servo configurations...");

        for (size_t i = 0; i < info_.joints.size(); i++) {
            const auto& joint = info_.joints[i];

            // Get parameters from URDF
            int gpio_pin = std::stoi(
                joint.parameters.at("gpio_pin"));
            double min_pulse = std::stod(
                joint.parameters.at("min_pulse_us"));
            double max_pulse = std::stod(
                joint.parameters.at("max_pulse_us"));
            double min_angle = std::stod(
                joint.parameters.at("min_angle"));
            double max_angle = std::stod(
                joint.parameters.at("max_angle"));
            double offset = std::stod(
                joint.parameters.at("offset"));
            bool reversed = (joint.parameters.at("reversed") == "true");

            // Create servo config
            servo::ArmServoConfig config(
                i, joint.name,
                gpio_pin,
                min_pulse, max_pulse,
                min_angle, max_angle,
                offset, reversed
            );

            servo_controller_->addServo(config);

            RCLCPP_INFO(logger_,
                "Loaded config for joint: %s (GPIO pin: %d)",
                joint.name.c_str(), gpio_pin);
        }

        return true;
    }

} 

// Export plugin
PLUGINLIB_EXPORT_CLASS(
    vx01_drone_hardware::DroneHardwareInterface,
    hardware_interface::SystemInterface
)