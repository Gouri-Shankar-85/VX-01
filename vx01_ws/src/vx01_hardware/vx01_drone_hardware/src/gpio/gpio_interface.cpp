#include "vx01_drone_hardware/gpio/gpio_interface.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include <string>

namespace vx01_drone_hardware {

    namespace gpio {

        GpioInterface::GpioInterface(int pin_number, const std::string& gpio_chip)
            : pin_number_(pin_number), gpio_chip_(gpio_chip),
            fd_(-1), is_open_(false) {}

        GpioInterface::~GpioInterface() {
            if (is_open_) {
                close();
            }
        }

        bool GpioInterface::open() {
            
            if (is_open_) {
                return true;
            }

            // Export GPIO pin
            std::string export_path = "/sys/class/gpio/export";
            int export_fd = ::open(export_path.c_str(), O_WRONLY);

            if (export_fd < 0) {
                std::cerr << "Failed to open GPIO export" << std::endl;
                return false;
            }

            std::string pin_str = std::to_string(pin_number_);
            write(export_fd, pin_str.c_str(), pin_str.size());
            ::close(export_fd);

            // Set direction to output
            std::string direction_path = "/sys/class/gpio/gpio" +
                                        std::to_string(pin_number_) +
                                        "/direction";

            int dir_fd = ::open(direction_path.c_str(), O_WRONLY);
            if (dir_fd < 0) {
                std::cerr << "Failed to set GPIO direction" << std::endl;
                return false;
            }

            write(dir_fd, "out", 3);
            ::close(dir_fd);

            // Open value file
            std::string value_path = "/sys/class/gpio/gpio" +
                                    std::to_string(pin_number_) +
                                    "/value";

            fd_ = ::open(value_path.c_str(), O_WRONLY);
            if (fd_ < 0) {
                std::cerr << "Failed to open GPIO value for pin: "
                        << pin_number_ << std::endl;
                return false;
            }

            is_open_ = true;
            std::cout << "GPIO pin " << pin_number_ << " opened" << std::endl;
            return true;
        }

        void GpioInterface::close() {

            if (is_open_ && fd_ >= 0) {
                ::close(fd_);
                fd_ = -1;

                // Unexport GPIO pin
                std::string unexport_path = "/sys/class/gpio/unexport";
                int unexport_fd = ::open(unexport_path.c_str(), O_WRONLY);
                if (unexport_fd >= 0) {
                    std::string pin_str = std::to_string(pin_number_);
                    write(unexport_fd, pin_str.c_str(), pin_str.size());
                    ::close(unexport_fd);
                }

                is_open_ = false;
                std::cout << "GPIO pin " << pin_number_ << " closed" << std::endl;
            }
        }

        bool GpioInterface::isOpen() const {
            return is_open_;
        }

        int GpioInterface::getPinNumber() const {
            return pin_number_;
        }

    }
} 