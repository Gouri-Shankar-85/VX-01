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

    // ------------------------------------------------------------------ //
    //  on_init
    // ------------------------------------------------------------------ //
    hardware_interface::CallbackReturn HexapodHardwareInterface::on_init(
        const hardware_interface::HardwareInfo& info)
    {
        if (hardware_interface::SystemInterface::on_init(info) !=
            hardware_interface::CallbackReturn::SUCCESS) {
            return hardware_interface::CallbackReturn::ERROR;
        }

        serial_port_ = info_.hardware_parameters.at("serial_port");
        baud_rate_   = std::stoi(info_.hardware_parameters.at("baud_rate"));

        double max_interp_step = 0.02;  // default
        if (info_.hardware_parameters.count("max_interpolation_step")) {
            max_interp_step = std::stod(info_.hardware_parameters.at("max_interpolation_step"));
        }

        RCLCPP_INFO(logger_, "Serial port: %s  baud: %d  max_interpolation_step: %.4f rad/cycle",
                    serial_port_.c_str(), baud_rate_, max_interp_step);

        hw_positions_.resize(info_.joints.size(), 0.0);
        hw_velocities_.resize(info_.joints.size(), 0.0);
        hw_efforts_.resize(info_.joints.size(), 0.0);
        hw_commands_.resize(info_.joints.size(), 0.0);

        for (const auto& joint : info_.joints) {
            joint_names_.push_back(joint.name);
            RCLCPP_INFO(logger_, "Joint: %s", joint.name.c_str());
        }

        serial_          = std::make_shared<communication::SerialInterface>(serial_port_, baud_rate_);
        maestro_         = std::make_shared<communication::MaestroProtocol>(serial_);
        servo_controller_= std::make_shared<servo::ServoController>(maestro_);

        // Apply interpolation speed before any servo is activated
        servo_controller_->setMaxStepPerCycle(max_interp_step);

        if (!loadServoConfigurations()) {
            RCLCPP_ERROR(logger_, "Failed to load servo configurations");
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(logger_, "on_init OK — %zu joints", joint_names_.size());
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    // ------------------------------------------------------------------ //
    //  on_configure / on_cleanup
    // ------------------------------------------------------------------ //
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

    // ------------------------------------------------------------------ //
    //  on_activate
    // ------------------------------------------------------------------ //
    hardware_interface::CallbackReturn HexapodHardwareInterface::on_activate(
        const rclcpp_lifecycle::State&)
    {
        RCLCPP_INFO(logger_, "on_activate — connecting to Maestro on %s", serial_port_.c_str());

        if (!connectToHardware()) {
            RCLCPP_ERROR(logger_, "Failed to connect to Maestro on %s", serial_port_.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (!servo_controller_->initialize()) {
            RCLCPP_WARN(logger_, "Servo initialisation had warnings — continuing anyway");
        }

        for (size_t i = 0; i < joint_names_.size(); ++i) {
            hw_positions_[i] = 0.0;
            hw_commands_[i]  = 0.0;
        }

        consecutive_write_failures_ = 0;
        is_active_ = true;
        RCLCPP_INFO(logger_, "Hardware activated — %zu joints ready  (interpolation step: %.4f rad/cycle)",
                    joint_names_.size(), servo_controller_->getMaxStepPerCycle());
        return hardware_interface::CallbackReturn::SUCCESS;
    }

        hardware_interface::CallbackReturn HexapodHardwareInterface::on_deactivate(
        const rclcpp_lifecycle::State&)
    {
        RCLCPP_INFO(logger_, "on_deactivate");
        is_active_ = false;

        // Only try to send servos home if the port is still alive
        if (serial_ && serial_->isOpen()) {
            servo_controller_->goHome();
        } else {
            RCLCPP_WARN(logger_, "Serial port not open during deactivate — skipping goHome");
        }

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
        double dt = period.seconds();
        for (size_t i = 0; i < joint_names_.size(); ++i) {
            double new_pos        = hw_commands_[i];
            hw_velocities_[i]     = (dt > 1e-9) ? (new_pos - hw_positions_[i]) / dt : 0.0;
            hw_positions_[i]      = new_pos;
            hw_efforts_[i]        = 0.0;
        }
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type HexapodHardwareInterface::write(
        const rclcpp::Time&, const rclcpp::Duration&)
    {
        if (!is_active_) {
            return hardware_interface::return_type::OK;
        }

        // ---- Auto-reconnect if the serial port was lost ---- //
        if (!serial_->isOpen()) {
            consecutive_write_failures_++;

            // Throttle reconnect attempts — try every ~2 s at 50 Hz = 100 cycles
            if (consecutive_write_failures_ % RECONNECT_RETRY_INTERVAL == 1) {
                RCLCPP_WARN(logger_,
                    "Serial port lost (failure #%d) — attempting reconnect on %s ...",
                    consecutive_write_failures_, serial_port_.c_str());

                if (serial_->reopen()) {
                    RCLCPP_INFO(logger_, "Reconnected to Maestro on %s", serial_port_.c_str());
                    consecutive_write_failures_ = 0;
                    // Re-apply speed/accel limits after reconnect
                    servo_controller_->initialize();
                } else {
                    RCLCPP_ERROR_THROTTLE(logger_, *rclcpp::Clock::make_shared(), 5000,
                        "Reconnect failed — will retry. Check USB cable and Maestro power.");
                    return hardware_interface::return_type::OK;
                }
            } else {
                // Not time to retry yet — return silently
                return hardware_interface::return_type::OK;
            }
        }

        for (size_t i = 0; i < joint_names_.size(); ++i) {
            servo_controller_->setJointAngleByIndex(i, hw_commands_[i]);
        }

        // ---- Interpolate + send to Maestro ---- //
        if (!servo_controller_->writeCommands()) {
            // writeCommands() failing once is not fatal — the serial layer
            // already printed the error and marked is_open_=false if it was EIO.
            RCLCPP_WARN_THROTTLE(logger_, *rclcpp::Clock::make_shared(), 2000,
                                 "Failed to write servo commands");
        } else {
            // Reset failure counter on a clean write
            consecutive_write_failures_ = 0;
        }

        return hardware_interface::return_type::OK;
    }

    // ------------------------------------------------------------------ //
    //  loadServoConfigurations()
    // ------------------------------------------------------------------ //
    bool HexapodHardwareInterface::loadServoConfigurations()
    {
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

        YAML::Node root    = YAML::LoadFile(yaml_path);
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

            int    channel   = cfg["channel"].as<int>();
            double min_pulse = cfg["min_pulse"].as<double>();
            double max_pulse = cfg["max_pulse"].as<double>();
            double min_angle = cfg["min_angle"].as<double>();
            double max_angle = cfg["max_angle"].as<double>();
            double offset    = cfg["offset"].as<double>(0.0);
            bool   reversed  = cfg["reversed"].as<bool>(false);
            double max_speed = cfg["max_speed"].as<double>(0.0);
            double max_accel = cfg["max_acceleration"].as<double>(0.0);

            servo::ServoConfig config(
                channel, joint_name,
                min_pulse, max_pulse,
                min_angle, max_angle,
                offset, reversed,
                max_speed, max_accel
            );

            servo_controller_->addServo(config);

            RCLCPP_INFO(logger_, "  %-22s → ch %2d  reversed=%-5s  offset=%.3f",
                        joint_name.c_str(), channel,
                        reversed ? "true" : "false", offset);
        }

        return true;
    }

    // ------------------------------------------------------------------ //
    //  connectToHardware() / disconnectFromHardware()
    // ------------------------------------------------------------------ //
    bool HexapodHardwareInterface::connectToHardware()
    {
        if (!serial_->open()) {
            RCLCPP_ERROR(logger_, "Cannot open serial port: %s", serial_port_.c_str());
            return false;
        }

        // Read and log any pre-existing Maestro errors
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

}  // namespace vx01_hexapod_hardware

PLUGINLIB_EXPORT_CLASS(
    vx01_hexapod_hardware::HexapodHardwareInterface,
    hardware_interface::SystemInterface
)