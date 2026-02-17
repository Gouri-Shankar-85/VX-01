#include "vx01_drone_hardware/servo/arm_servo_config.hpp"
#include <cmath>
#include <algorithm>

namespace vx01_drone_hardware {

    namespace servo {

        ArmServoConfig::ArmServoConfig(int servo_id, const std::string& joint_name,
                                    int gpio_pin,
                                    double min_pulse, double max_pulse,
                                    double min_angle, double max_angle,
                                    double offset, bool reversed)
            : servo_id_(servo_id), joint_name_(joint_name),
            gpio_pin_(gpio_pin),
            min_pulse_(min_pulse), max_pulse_(max_pulse),
            min_angle_(min_angle), max_angle_(max_angle),
            offset_(offset), reversed_(reversed) {}

        ArmServoConfig::ArmServoConfig()
            : servo_id_(0), joint_name_(""),
            gpio_pin_(0),
            min_pulse_(500.0), max_pulse_(2500.0),
            min_angle_(-M_PI_2), max_angle_(M_PI_2),
            offset_(0.0), reversed_(false) {}

        int ArmServoConfig::getServoId() const {
            return servo_id_;
        }

        std::string ArmServoConfig::getJointName() const {
            return joint_name_;
        }

        int ArmServoConfig::getGpioPin() const {
            return gpio_pin_;
        }

        double ArmServoConfig::getMinPulse() const {
            return min_pulse_;
        }

        double ArmServoConfig::getMaxPulse() const {
            return max_pulse_;
        }

        double ArmServoConfig::getMinAngle() const {
            return min_angle_;
        }

        double ArmServoConfig::getMaxAngle() const {
            return max_angle_;
        }

        double ArmServoConfig::getOffset() const {
            return offset_;
        }

        bool ArmServoConfig::isReversed() const {
            return reversed_;
        }

        void ArmServoConfig::setServoId(int servo_id) {
            servo_id_ = servo_id;
        }

        void ArmServoConfig::setJointName(const std::string& joint_name) {
            joint_name_ = joint_name;
        }

        void ArmServoConfig::setGpioPin(int gpio_pin) {
            gpio_pin_ = gpio_pin;
        }

        void ArmServoConfig::setMinPulse(double min_pulse) {
            min_pulse_ = min_pulse;
        }

        void ArmServoConfig::setMaxPulse(double max_pulse) {
            max_pulse_ = max_pulse;
        }

        void ArmServoConfig::setMinAngle(double min_angle) {
            min_angle_ = min_angle;
        }

        void ArmServoConfig::setMaxAngle(double max_angle) {
            max_angle_ = max_angle;
        }

        void ArmServoConfig::setOffset(double offset) {
            offset_ = offset;
        }

        void ArmServoConfig::setReversed(bool reversed) {
            reversed_ = reversed;
        }

        double ArmServoConfig::angleToPulse(double angle) const {
            // Apply offset
            double calibrated = angle + offset_;

            // Apply reversal
            if (reversed_) {
                calibrated = -calibrated;
            }

            // Clamp to valid range
            calibrated = std::max(min_angle_, std::min(max_angle_, calibrated));

            // Linear interpolation from angle to pulse
            double normalized = (calibrated - min_angle_) / (max_angle_ - min_angle_);
            double pulse = min_pulse_ + normalized * (max_pulse_ - min_pulse_);

            return pulse;
        }

        double ArmServoConfig::pulseToAngle(double pulse) const {
            // Clamp pulse to valid range
            pulse = std::max(min_pulse_, std::min(max_pulse_, pulse));

            // Linear interpolation from pulse to angle
            double normalized = (pulse - min_pulse_) / (max_pulse_ - min_pulse_);
            double angle = min_angle_ + normalized * (max_angle_ - min_angle_);

            // Remove reversal
            if (reversed_) {
                angle = -angle;
            }

            // Remove offset
            angle -= offset_;

            return angle;
        }

    } 
} 