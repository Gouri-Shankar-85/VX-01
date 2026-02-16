#ifndef VX01_HEXAPOD_HARDWARE_HEXAPOD_HARDWARE_INTERFACE_HPP
#define VX01_HEXAPOD_HARDWARE_HEXAPOD_HARDWARE_INTERFACE_HPP

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "vx01_hexapod_hardware/servo/servo_controller.hpp"
#include "vx01_hexapod_hardware/communication/serial_interface.hpp"
#include "vx01_hexapod_hardware/communication/maestro_protocol.hpp"

#include <vector>
#include <string>
#include <memory>

namespace vx01_hexapod_hardware {

    class HexapodHardwareInterface : public hardware_interface::SystemInterface {

        private: 

            std::shared_ptr<communication::SerialInterface> serial_;
            std::shared_ptr<communication::MaestroProtocol> maestro_;
            std::shared_ptr<servo::ServoController> servo_controller_;

            std::string serial_port_;
            int baud_rate_;

            std::vector<double> hw_positions_;            
            std::vector<double> hw_velocities_;
            std::vector<double> hw_efforts_;

            std::vector<double> hw_commands_;

            std::vector<std::string> joint_names_;

            rclcpp::Logger logger_;


        public:

            HexapodHardwareInterface();

            ~HexapodHardwareInterface();

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

            std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

            std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

            hardware_interface::return_type read(
                const rclcpp::Time& time, const rclcpp::Duration& period) override;

            hardware_interface::return_type write(
                const rclcpp::Time& time, const rclcpp::Duration& period) override;

        private:

            bool loadServoConfigurations();
            bool connectToHardware();
            void disconnectFromHardware();
    };
}

#endif