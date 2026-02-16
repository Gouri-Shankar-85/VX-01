#include "vx01_hexapod_hardware/servo/servo_controller.hpp"
#include <iostream>
#include <cmath>

namespace vx01_hexapod_hardware {

    namespace servo {

        ServoController::ServoController(std::shared_ptr<communication::MaestroProtocol> maestro)
            : maestro_(maestro) {}

        void ServoController::addServo(const ServoConfig& config) {

            servo_configs_.push_back(config);
            
            // Map joint name to index
            int index = servo_configs_.size() - 1;
            joint_name_to_index_[config.getJointName()] = index;
            
            // Initialize state vectors
            current_angles_.push_back(0.0);
            command_angles_.push_back(0.0);
            current_pulses_.push_back(1500.0);  // Neutral position
        }

        int ServoController::getServoCount() const {
            return servo_configs_.size();
        }

        bool ServoController::initialize() {
            
            if (servo_configs_.empty()) {
                std::cerr << "No servos configured" << std::endl;
                return false;
            }
            
            std::cout << "Initializing " << servo_configs_.size() << " servos..." << std::endl;
            
            // Set speed and acceleration limits for each servo
            for (size_t i = 0; i < servo_configs_.size(); i++) {
                const ServoConfig& config = servo_configs_[i];
                int servo_id = config.getServoId();
                
                // Set speed limit
                uint16_t speed = static_cast<uint16_t>(config.getMaxSpeed());
                if (!maestro_->setSpeed(servo_id, speed)) {
                    std::cerr << "Failed to set speed for servo " << servo_id << std::endl;
                    return false;
                }
                
                // Set acceleration limit
                uint16_t accel = static_cast<uint16_t>(config.getMaxAcceleration());
                if (!maestro_->setAcceleration(servo_id, accel)) {
                    std::cerr << "Failed to set acceleration for servo " << servo_id << std::endl;
                    return false;
                }
                
                std::cout << "Configured servo " << servo_id 
                        << " (" << config.getJointName() << ")" << std::endl;
            }
            
            std::cout << "Servo initialization complete" << std::endl;
            return true;
        }

        bool ServoController::goHome() {
            std::cout << "Moving all servos to home position..." << std::endl;
            
            // Set all servos to neutral position (0 radians)
            for (size_t i = 0; i < servo_configs_.size(); i++) {
                command_angles_[i] = 0.0;
            }
            
            return writeCommands();
        }

        bool ServoController::setJointAngle(const std::string& joint_name, double angle) {

            auto it = joint_name_to_index_.find(joint_name);
            if (it == joint_name_to_index_.end()) {
                std::cerr << "Joint not found: " << joint_name << std::endl;
                return false;
            }
            
            int index = it->second;
            return setJointAngleByIndex(index, angle);
        }

        bool ServoController::setJointAngleByIndex(int index, double angle) {

            if (index < 0 || index >= static_cast<int>(servo_configs_.size())) {
                std::cerr << "Invalid servo index: " << index << std::endl;
                return false;
            }
            
            command_angles_[index] = angle;
            return true;
        }

        double ServoController::getJointAngle(const std::string& joint_name) const {

            auto it = joint_name_to_index_.find(joint_name);
            if (it == joint_name_to_index_.end()) {
                std::cerr << "Joint not found: " << joint_name << std::endl;
                return 0.0;
            }
            
            int index = it->second;
            return getJointAngleByIndex(index);
        }

        double ServoController::getJointAngleByIndex(int index) const {

            if (index < 0 || index >= static_cast<int>(current_angles_.size())) {
                return 0.0;
            }
            
            return current_angles_[index];
        }

        bool ServoController::writeCommands() {

            bool success = true;
            
            for (size_t i = 0; i < servo_configs_.size(); i++) {
                const ServoConfig& config = servo_configs_[i];
                int servo_id = config.getServoId();
                double angle = command_angles_[i];
                
                // Convert angle to pulse width
                double pulse_us = config.angleToPulse(angle);
                
                // Convert to target value (quarter-microseconds)
                uint16_t target = communication::MaestroProtocol::microsecondsToTarget(pulse_us);
                
                // Send command to Maestro
                if (!maestro_->setTarget(servo_id, target)) {
                    std::cerr << "Failed to set target for servo " << servo_id << std::endl;
                    success = false;
                }
            }
            
            return success;
        }

        bool ServoController::readPositions() {

            bool success = true;
            
            for (size_t i = 0; i < servo_configs_.size(); i++) {
                const ServoConfig& config = servo_configs_[i];
                int servo_id = config.getServoId();
                
                uint16_t position;
                if (maestro_->getPosition(servo_id, position)) {
                    // Convert from quarter-microseconds to microseconds
                    double pulse_us = communication::MaestroProtocol::targetToMicroseconds(position);
                    current_pulses_[i] = pulse_us;
                    
                    // Convert to angle
                    current_angles_[i] = config.pulseToAngle(pulse_us);
                } else {
                    std::cerr << "Failed to read position for servo " << servo_id << std::endl;
                    success = false;
                }
            }
            
            return success;
        }

        bool ServoController::isMoving() {

            bool moving = false;
            if (!maestro_->getMovingState(moving)) {
                std::cerr << "Failed to get moving state" << std::endl;
                return false;
            }
            return moving;
        }

        bool ServoController::getErrors(uint16_t& errors) {
            return maestro_->getErrors(errors);
        }

        ServoConfig ServoController::getServoConfig(int index) const {

            if (index < 0 || index >= static_cast<int>(servo_configs_.size())) {
                return ServoConfig();
            }
            return servo_configs_[index];
        }

        ServoConfig ServoController::getServoConfigByName(const std::string& joint_name) const {

            auto it = joint_name_to_index_.find(joint_name);
            if (it == joint_name_to_index_.end()) {
                return ServoConfig();
            }
            
            int index = it->second;
            return getServoConfig(index);
        }

    } 
} 