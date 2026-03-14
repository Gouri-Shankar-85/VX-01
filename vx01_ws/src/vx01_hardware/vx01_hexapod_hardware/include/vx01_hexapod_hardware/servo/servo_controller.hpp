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
                std::vector<double> current_angles_;     // read back from hardware
                std::vector<double> command_angles_;     // interpolated position sent to Maestro
                std::vector<double> target_angles_;      // goal position set by JTC
                std::vector<double> current_pulses_;     // raw pulse widths (µs)

                // Maximum angular change per write() cycle (radians).
                // At 50 Hz, 0.02 rad/cycle ≈ 57 °/s — a smooth walking speed.
                // Set to 0.0 to disable interpolation (instant, original behaviour).
                double max_step_per_cycle_ = 0.02;

            public:

                ServoController(std::shared_ptr<communication::MaestroProtocol> maestro);

                // Add servo configuration
                void addServo(const ServoConfig& config);

                // Get number of servos
                int getServoCount() const;

                // Set the maximum angular step per write() cycle (radians).
                // Call this after construction, before activate.
                // 0.0 = unlimited (reverts to original instant behaviour).
                void setMaxStepPerCycle(double step_rad);

                // Get current max step setting
                double getMaxStepPerCycle() const;

                // Initialize all servos (set speed and acceleration limits)
                bool initialize();

                // Move all servos to home position (instant snap, ignores interpolation)
                bool goHome();

                // Set commanded angle for a joint by name
                // (stores in target_angles_; actual movement is interpolated in writeCommands)
                bool setJointAngle(const std::string& joint_name, double angle);

                // Set commanded angle for a joint by index
                // (stores in target_angles_; actual movement is interpolated in writeCommands)
                bool setJointAngleByIndex(int index, double angle);

                // Get current interpolated angle for a joint by name
                double getJointAngle(const std::string& joint_name) const;

                // Get current interpolated angle for a joint by index
                double getJointAngleByIndex(int index) const;

                // Write commands to all servos.
                // Advances command_angles_ one max_step_per_cycle_ toward target_angles_
                // before sending to the Maestro.
                bool writeCommands();

                // Read current positions from all servos
                bool readPositions();

                // Check if any servo is moving
                bool isMoving();

                // Returns true if every joint has reached its target
                // (all |target - command| < tolerance_rad)
                bool hasReachedTarget(double tolerance_rad = 1e-4) const;

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