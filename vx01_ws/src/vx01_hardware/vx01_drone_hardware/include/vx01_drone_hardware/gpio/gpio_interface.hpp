#ifndef VX01_DRONE_HARDWARE_GPIO_INTERFACE_HPP
#define VX01_DRONE_HARDWARE_GPIO_INTERFACE_HPP

#include <string>

namespace vx01_drone_hardware {

    namespace gpio {

        class GpioInterface {

            private:
                int pin_number_;
                std::string gpio_chip_;
                int fd_;
                bool is_open_;

            public:
                // Constructor
                GpioInterface(int pin_number, const std::string& gpio_chip);

                // Destructor
                ~GpioInterface();

                // Open GPIO pin
                bool open();

                // Close GPIO pin
                void close();

                // Check if open
                bool isOpen() const;

                // Get pin number
                int getPinNumber() const;
        };
    } 
} 

#endif