#ifndef VX01_DRONE_HARDWARE_ARM_SERVO_CONTROLLER_HPP
#define VX01_DRONE_HARDWARE_ARM_SERVO_CONTROLLER_HPP

#include "vx01_drone_hardware/servo/arm_servo_config.hpp"
#include "vx01_drone_hardware/gpio/pwm_controller.hpp"
#include "vx01_drone_hardware/gpio/gpio_interface.hpp"
#include <vector>
#include <memory>
#include <map>
#include <string>

namespace vx01_drone_hardware {

    namespace servo {

        class ArmServoController {
            
            private:
                std::vector<ArmServoConfig> servo_configs_;
                std::vector<std::shared_ptr<gpio::PwmController>> pwm_controllers_;
                std::map<std::string, int> joint_name_to_index_;

                // Current state
                std::vector<double> current_angles_;
                std::vector<double> command_angles_;

                // PWM settings
                int pwm_frequency_;
                std::string gpio_chip_;

            public:
                // Constructor
                ArmServoController(int pwm_frequency, const std::string& gpio_chip);

                // Add servo
                void addServo(const ArmServoConfig& config);

                // Get servo count
                int getServoCount() const;

                // Initialize all servos
                bool initialize();

                // Move all servos to home (0 degrees)
                bool goHome();

                // Set commanded angle by joint name
                bool setJointAngle(const std::string& joint_name, double angle);

                // Set commanded angle by index
                bool setJointAngleByIndex(int index, double angle);

                // Get current angle by joint name
                double getJointAngle(const std::string& joint_name) const;

                // Get current angle by index
                double getJointAngleByIndex(int index) const;

                // Write commands to all servos
                bool writeCommands();

                // Shutdown all servos safely
                void shutdown();
        };

    } 
} 

#endif