#ifndef VX01_DRONE_HARDWARE_PWM_CONTROLLER_HPP
#define VX01_DRONE_HARDWARE_PWM_CONTROLLER_HPP

#include "vx01_drone_hardware/gpio/gpio_interface.hpp"
#include <memory>

namespace vx01_drone_hardware {

    namespace gpio {

        class PwmController {

            private:

                std::shared_ptr<GpioInterface> gpio_;
                int pwm_frequency_;       // Hz (50Hz for servos)
                double pulse_width_us_;   // Current pulse width in microseconds
                bool is_running_;

            public:

                PwmController(std::shared_ptr<GpioInterface> gpio, int pwm_frequency);

                ~PwmController();

                bool start();

                void stop();

                bool isRunning() const;

                bool setPulseWidth(double pulse_width_us);

                double getPulseWidth() const;

                int getPwmFrequency() const;
        };

    } 
} 

#endif