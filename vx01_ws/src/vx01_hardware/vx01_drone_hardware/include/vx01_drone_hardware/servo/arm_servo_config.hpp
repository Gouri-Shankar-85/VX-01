#ifndef VX01_DRONE_HARDWARE_ARM_SERVO_CONFIG_HPP
#define VX01_DRONE_HARDWARE_ARM_SERVO_CONFIG_HPP

#include <string>

namespace vx01_drone_hardware {

    namespace servo {

        class ArmServoConfig {
            
            private:
                int servo_id_;            // Servo ID (0-3)
                std::string joint_name_;  // Joint name (e.g. "drone_arm0_joint")
                int gpio_pin_;            // GPIO pin on RDK X5

                // Pulse width limits (microseconds)
                double min_pulse_;        // Minimum pulse width (500us)
                double max_pulse_;        // Maximum pulse width (2500us)

                // Angle limits (radians)
                double min_angle_;        // Minimum angle (-90 degrees)
                double max_angle_;        // Maximum angle (+90 degrees)

                // Calibration
                double offset_;           // Angle offset (radians)
                bool reversed_;           // Reverse direction

            public:
                // Constructor
                ArmServoConfig(int servo_id, const std::string& joint_name,
                            int gpio_pin,
                            double min_pulse, double max_pulse,
                            double min_angle, double max_angle,
                            double offset, bool reversed);

                // Default constructor
                ArmServoConfig();

                // Getters
                int getServoId() const;
                std::string getJointName() const;
                int getGpioPin() const;
                double getMinPulse() const;
                double getMaxPulse() const;
                double getMinAngle() const;
                double getMaxAngle() const;
                double getOffset() const;
                bool isReversed() const;

                // Setters
                void setServoId(int servo_id);
                void setJointName(const std::string& joint_name);
                void setGpioPin(int gpio_pin);
                void setMinPulse(double min_pulse);
                void setMaxPulse(double max_pulse);
                void setMinAngle(double min_angle);
                void setMaxAngle(double max_angle);
                void setOffset(double offset);
                void setReversed(bool reversed);

                // Convert angle to pulse width
                double angleToPulse(double angle) const;

                // Convert pulse width to angle
                double pulseToAngle(double pulse) const;
        };

    } 
} 

#endif