#include "vx01_hexapod_hardware/servo/servo_config.hpp"
#include <cmath>
#include <algorithm>

namespace vx01_hexapod_hardware {

    namespace servo {

        ServoConfig::ServoConfig(int servo_id, const std::string& joint_name,
                                double min_pulse, double max_pulse,
                                double min_angle, double max_angle,
                                double offset, bool reversed,
                                double max_speed, double max_acceleration)
            : servo_id_(servo_id), joint_name_(joint_name),
            min_pulse_(min_pulse), max_pulse_(max_pulse),
            min_angle_(min_angle), max_angle_(max_angle),
            offset_(offset), reversed_(reversed),
            max_speed_(max_speed), max_acceleration_(max_acceleration) {}

        ServoConfig::ServoConfig()
            : servo_id_(0), joint_name_(""),
            min_pulse_(500.0), max_pulse_(2500.0),
            min_angle_(-M_PI_2), max_angle_(M_PI_2),
            offset_(0.0), reversed_(false),
            max_speed_(0.0), max_acceleration_(0.0) {}

        int ServoConfig::getServoId() const {
            return servo_id_;
        }

        std::string ServoConfig::getJointName() const {
            return joint_name_;
        }

        double ServoConfig::getMinPulse() const {
            return min_pulse_;
        }

        double ServoConfig::getMaxPulse() const {
            return max_pulse_;
        }

        double ServoConfig::getMinAngle() const {
            return min_angle_;
        }

        double ServoConfig::getMaxAngle() const {
            return max_angle_;
        }

        double ServoConfig::getOffset() const {
            return offset_;
        }

        bool ServoConfig::isReversed() const {
            return reversed_;
        }

        double ServoConfig::getMaxSpeed() const {
            return max_speed_;
        }

        double ServoConfig::getMaxAcceleration() const {
            return max_acceleration_;
        }

        void ServoConfig::setServoId(int servo_id) {
            servo_id_ = servo_id;
        }

        void ServoConfig::setJointName(const std::string& joint_name) {
            joint_name_ = joint_name;
        }

        void ServoConfig::setMinPulse(double min_pulse) {
            min_pulse_ = min_pulse;
        }

        void ServoConfig::setMaxPulse(double max_pulse) {
            max_pulse_ = max_pulse;
        }

        void ServoConfig::setMinAngle(double min_angle) {
            min_angle_ = min_angle;
        }

        void ServoConfig::setMaxAngle(double max_angle) {
            max_angle_ = max_angle;
        }

        void ServoConfig::setOffset(double offset) {
            offset_ = offset;
        }

        void ServoConfig::setReversed(bool reversed) {
            reversed_ = reversed;
        }

        void ServoConfig::setMaxSpeed(double max_speed) {
            max_speed_ = max_speed;
        }

        void ServoConfig::setMaxAcceleration(double max_acceleration) {
            max_acceleration_ = max_acceleration;
        }

        double ServoConfig::angleToPulse(double angle) const {
            // Apply calibration first
            double calibrated_angle = applyCalibration(angle);
            
            // Clamp angle to valid range
            calibrated_angle = std::max(min_angle_, std::min(max_angle_, calibrated_angle));
            
            // Linear interpolation from angle range to pulse range
            double normalized = (calibrated_angle - min_angle_) / (max_angle_ - min_angle_);
            double pulse = min_pulse_ + normalized * (max_pulse_ - min_pulse_);
            
            return pulse;
        }

        double ServoConfig::pulseToAngle(double pulse) const {
            // Clamp pulse to valid range
            pulse = std::max(min_pulse_, std::min(max_pulse_, pulse));
            
            // Linear interpolation from pulse range to angle range
            double normalized = (pulse - min_pulse_) / (max_pulse_ - min_pulse_);
            double angle = min_angle_ + normalized * (max_angle_ - min_angle_);
            
            // Remove calibration
            return removeCalibration(angle);
        }

        double ServoConfig::applyCalibration(double raw_angle) const {
            double calibrated = raw_angle;
            
            // Apply offset
            calibrated += offset_;
            
            // Apply reversal
            if (reversed_) {
                calibrated = -calibrated;
            }
            
            return calibrated;
        }

        double ServoConfig::removeCalibration(double calibrated_angle) const {
            double raw = calibrated_angle;
            
            // Remove reversal
            if (reversed_) {
                raw = -raw;
            }
            
            // Remove offset
            raw -= offset_;
            
            return raw;
        }

    } 
} 