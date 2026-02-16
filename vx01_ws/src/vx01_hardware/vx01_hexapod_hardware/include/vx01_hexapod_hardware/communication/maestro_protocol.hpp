#ifndef VX01_HEXAPOD_HARDWARE_MAESTRO_PROTOCOL_HPP
#define VX01_HEXAPOD_HARDWARE_MAESTRO_PROTOCOL_HPP

#include "vx01_hexapod_hardware/communication/serial_interface.hpp"
#include <memory>

namespace vx01_hexapod_hardware {

    namespace communication {

        class MaestroProtocol {
            
            private:
                std::shared_ptr<SerialInterface> serial_;
                uint8_t device_number_; //Maestro device number (default : 12)

            public:

                MaestroProtocol(std::shared_ptr<SerialInterface> serial, uint8_t device_number = 12);

                bool setTarget(uint8_t channel, uint16_t target);

                bool setSpeed(uint8_t channel, uint16_t speed);

                bool setAcceleration(uint8_t channel, uint16_t acceleration);

                bool getPosition(uint8_t channel, u_int16_t& position);

                bool getMovingState(bool& is_moving);

                bool getErrors(uint16_t& errors);

                bool goHome();

                static uint16_t microsecondsToTarget(double microseconds);

                static double targetToMicroseconds(uint16_t target);

                static uint16_t angleToTarget(double angle, double min_angle, double max_angle);

                static double targetToAngle(uint16_t target, double min_angle, double max_angle);
        };
    }
}

#endif