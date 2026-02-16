#ifndef VX01_HEXAPOD_HARDWARE_SERVO_CONFIG_HPP
#define VX01_HEXAPOD_HARDWARE_SERVO_CONFIG_HPP

#include <string>

namespace vx01_hexapod_hardware {

    namespace servo {

        class ServoConfig {

            private:
                int servo_id_;
                std::string joint_name_;

                double min_pulse_;
                double max_pulse_;

                double min_angle_;
                double max_angle_;

                double offset_;
                bool reversed_;

                double max_speed_;
                double max_acceleration_;

            public:

                ServoConfig(int servo_id, const std::string& joint_name,
                            double min_pulse, double max_pulse,
                            double min_angle, double max_angle,
                            double offset, bool reversed, 
                            double max_speed, double max_acceleration);

                ServoConfig();

                int getServoId() const;
                std::string getJointName() const;
                double getMinPulse() const;
                double getMaxPulse() const;
                double getMinAngle() const;
                double getMaxAngle() const;
                double getOffset() const;
                bool isReversed() const;
                double getMaxSpeed() const;
                double getMaxAcceleration() const;
                
                void setServoId(int servo_id);
                void setJointName(const std::string& joint_name);
                void setMinPulse(double min_pulse);
                void setMaxPulse(double max_pulse);
                void setMinAngle(double min_angle);
                void setMaxAngle(double max_angle);
                void setOffset(double offset);
                void setReversed(bool reversed);
                void setMaxSpeed(double max_speed);
                void setMaxAcceleration(double max_acceleration);
                
                double angleToPulse(double angle) const;
                
                double pulseToAngle(double pulse) const;
                
                double applyCalibration(double raw_angle) const;
                
                double removeCalibration(double calibrated_angle) const;
        };
    }
}

#endif