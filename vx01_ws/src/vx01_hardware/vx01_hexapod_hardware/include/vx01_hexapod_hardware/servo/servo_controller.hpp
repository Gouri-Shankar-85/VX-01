#ifndef VX01_HEXAPOD_HARDWARE_SERVO_CONTROLLER_HPP
#define VX01_HEXAPOD_HARDWARE_SERVO_CONTROLLER_HPP

#include "vx01_hexapod_hardware/servo/servo_config.hpp"
#include "vx01_hexapod_hardware/communication/maestro_protocol.hpp"
#include <vector>
#include <memory>
#include <map>

namespace vx01_hexapod_hardware {

    namespace servo {

        class ServoController {

            private:

                std::shared_ptr<communication::MaestroProtocol> maestro_;
                std::vector<ServoConfig> servo_configs_;
                std::map<std::string, int> joint_name_to_index_;
                
                // Current state
                std::vector<double> current_angles_;     
                std::vector<double> command_angles_;     
                std::vector<double> current_pulses_;     

            public:

                ServoController(std::shared_ptr<communication::MaestroProtocol> maestro);
                
                // Add servo configuration
                void addServo(const ServoConfig& config);
                
                // Get number of servos
                int getServoCount() const;
                
                // Initialize all servos (set speed and acceleration limits)
                bool initialize();
                
                // Move all servos to home position
                bool goHome();
                
                // Set commanded angle for a joint by name
                bool setJointAngle(const std::string& joint_name, double angle);
                
                // Set commanded angle for a joint by index
                bool setJointAngleByIndex(int index, double angle);
                
                // Get current angle for a joint by name
                double getJointAngle(const std::string& joint_name) const;
                
                // Get current angle for a joint by index
                double getJointAngleByIndex(int index) const;
                
                // Write commands to all servos
                bool writeCommands();
                
                // Read current positions from all servos
                bool readPositions();
                
                // Check if any servo is moving
                bool isMoving();
                
                // Get errors from Maestro
                bool getErrors(uint16_t& errors);
                
                // Get servo config by index
                ServoConfig getServoConfig(int index) const;
                
                // Get servo config by joint name
                ServoConfig getServoConfigByName(const std::string& joint_name) const;
        };
    }
}

#endif