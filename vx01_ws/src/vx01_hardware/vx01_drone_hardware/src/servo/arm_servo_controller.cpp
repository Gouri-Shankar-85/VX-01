#include "vx01_drone_hardware/servo/arm_servo_controller.hpp"
#include <iostream>

namespace vx01_drone_hardware {

    namespace servo {

        ArmServoController::ArmServoController(int pwm_frequency,
                                            const std::string& gpio_chip)
            : pwm_frequency_(pwm_frequency), gpio_chip_(gpio_chip) {}

        void ArmServoController::addServo(const ArmServoConfig& config) {

            servo_configs_.push_back(config);

            // Map joint name to index
            int index = servo_configs_.size() - 1;
            joint_name_to_index_[config.getJointName()] = index;

            // Create GPIO interface for this servo
            std::shared_ptr<gpio::GpioInterface> gpio =
                std::make_shared<gpio::GpioInterface>(
                    config.getGpioPin(), gpio_chip_);

            // Create PWM controller for this servo
            std::shared_ptr<gpio::PwmController> pwm =
                std::make_shared<gpio::PwmController>(gpio, pwm_frequency_);

            pwm_controllers_.push_back(pwm);

            // Initialize state
            current_angles_.push_back(0.0);
            command_angles_.push_back(0.0);

            std::cout << "Added servo: " << config.getJointName()
                    << " on GPIO pin: " << config.getGpioPin() << std::endl;
        }

        int ArmServoController::getServoCount() const {
            return servo_configs_.size();
        }

        bool ArmServoController::initialize() {
            if (servo_configs_.empty()) {
                std::cerr << "No servos configured" << std::endl;
                return false;
            }

            std::cout << "Initializing "
                    << servo_configs_.size()
                    << " arm servos..." << std::endl;

            for (size_t i = 0; i < pwm_controllers_.size(); i++) {
                // Open GPIO
                if (!pwm_controllers_[i]->isRunning()) {
                    // Start PWM
                    if (!pwm_controllers_[i]->start()) {
                        std::cerr << "Failed to start PWM for servo: "
                                << servo_configs_[i].getJointName()
                                << std::endl;
                        return false;
                    }
                }

                // Set initial neutral pulse
                double neutral_pulse = servo_configs_[i].angleToPulse(0.0);
                if (!pwm_controllers_[i]->setPulseWidth(neutral_pulse)) {
                    std::cerr << "Failed to set initial pulse for servo: "
                            << servo_configs_[i].getJointName()
                            << std::endl;
                    return false;
                }

                std::cout << "Initialized servo: "
                        << servo_configs_[i].getJointName()
                        << std::endl;
            }

            std::cout << "All arm servos initialized" << std::endl;
            return true;
        }

        bool ArmServoController::goHome() {
            std::cout << "Moving all arm servos to home (0 degrees)..."
                    << std::endl;

            for (size_t i = 0; i < servo_configs_.size(); i++) {
                command_angles_[i] = 0.0;
            }

            return writeCommands();
        }

        bool ArmServoController::setJointAngle(const std::string& joint_name,
                                                double angle) {
            auto it = joint_name_to_index_.find(joint_name);
            if (it == joint_name_to_index_.end()) {
                std::cerr << "Joint not found: " << joint_name << std::endl;
                return false;
            }

            return setJointAngleByIndex(it->second, angle);
        }

        bool ArmServoController::setJointAngleByIndex(int index, double angle) {
            if (index < 0 ||
                index >= static_cast<int>(servo_configs_.size())) {
                std::cerr << "Invalid servo index: " << index << std::endl;
                return false;
            }

            command_angles_[index] = angle;
            return true;
        }

        double ArmServoController::getJointAngle(
            const std::string& joint_name) const {

            auto it = joint_name_to_index_.find(joint_name);
            if (it == joint_name_to_index_.end()) {
                return 0.0;
            }

            return getJointAngleByIndex(it->second);
        }

        double ArmServoController::getJointAngleByIndex(int index) const {
            if (index < 0 ||
                index >= static_cast<int>(current_angles_.size())) {
                return 0.0;
            }

            return current_angles_[index];
        }

        bool ArmServoController::writeCommands() {
            bool success = true;

            for (size_t i = 0; i < servo_configs_.size(); i++) {
                // Convert angle to pulse width
                double pulse = servo_configs_[i].angleToPulse(command_angles_[i]);

                // Send to PWM controller
                if (!pwm_controllers_[i]->setPulseWidth(pulse)) {
                    std::cerr << "Failed to write command for servo: "
                            << servo_configs_[i].getJointName()
                            << std::endl;
                    success = false;
                }

                // Update current angle
                current_angles_[i] = command_angles_[i];
            }

            return success;
        }

        void ArmServoController::shutdown() {
            std::cout << "Shutting down arm servos..." << std::endl;

            // Move to home before shutdown
            goHome();

            // Stop all PWM controllers
            for (size_t i = 0; i < pwm_controllers_.size(); i++) {
                if (pwm_controllers_[i]->isRunning()) {
                    pwm_controllers_[i]->stop();
                }
            }

            std::cout << "All arm servos shutdown" << std::endl;
        }

    } 
} 