#ifndef VX01_DRONE_HARDWARE_DRONE_HARDWARE_INTERFACE_HPP
#define VX01_DRONE_HARDWARE_DRONE_HARDWARE_INTERFACE_HPP

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "vx01_drone_hardware/servo/arm_servo_controller.hpp"
#include "vx01_drone_hardware/servo/arm_servo_config.hpp"

#include <vector>
#include <string>
#include <memory>

namespace vx01_drone_hardware {

    class DroneHardwareInterface : public hardware_interface::SystemInterface {

        private:
            // Servo controller
            std::shared_ptr<servo::ArmServoController> servo_controller_;

            // Hardware parameters
            std::string gpio_chip_;
            int pwm_frequency_;

            // Joint state interfaces
            std::vector<double> hw_positions_;
            std::vector<double> hw_velocities_;

            // Joint command interfaces
            std::vector<double> hw_commands_;

            // Joint names
            std::vector<std::string> joint_names_;

            // Logger
            rclcpp::Logger logger_;

        public:
            DroneHardwareInterface();

            ~DroneHardwareInterface();

            hardware_interface::CallbackReturn on_init(
                const hardware_interface::HardwareInfo& info) override;

            hardware_interface::CallbackReturn on_configure(
                const rclcpp_lifecycle::State& previous_state) override;

            hardware_interface::CallbackReturn on_cleanup(
                const rclcpp_lifecycle::State& previous_state) override;

            hardware_interface::CallbackReturn on_activate(
                const rclcpp_lifecycle::State& previous_state) override;

            hardware_interface::CallbackReturn on_deactivate(
                const rclcpp_lifecycle::State& previous_state) override;

            // Interfaces
            std::vector<hardware_interface::StateInterface>
                export_state_interfaces() override;

            std::vector<hardware_interface::CommandInterface>
                export_command_interfaces() override;

            // Read/Write
            hardware_interface::return_type read(
                const rclcpp::Time& time,
                const rclcpp::Duration& period) override;

            hardware_interface::return_type write(
                const rclcpp::Time& time,
                const rclcpp::Duration& period) override;

        private:
            // Helper
            bool loadServoConfigurations();
    };

} 

#endif