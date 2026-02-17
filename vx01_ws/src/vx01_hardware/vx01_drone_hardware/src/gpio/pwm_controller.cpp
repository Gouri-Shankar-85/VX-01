#include "vx01_drone_hardware/gpio/pwm_controller.hpp"
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <string>

namespace vx01_drone_hardware {

    namespace gpio {

        PwmController::PwmController(std::shared_ptr<GpioInterface> gpio, int pwm_frequency)
            : gpio_(gpio), pwm_frequency_(pwm_frequency),
            pulse_width_us_(1500.0), is_running_(false) {}

        PwmController::~PwmController() {
            if (is_running_) {
                stop();
            }
        }

        bool PwmController::start() {
            if (!gpio_->isOpen()) {
                std::cerr << "GPIO not open for pin: "
                        << gpio_->getPinNumber() << std::endl;
                return false;
            }

            // Use Linux PWM subsystem via sysfs
            // Path: /sys/class/pwm/pwmchipX/pwmX/
            std::string pwm_chip_path = "/sys/class/pwm/pwmchip0";

            // Export PWM channel
            std::string export_path = pwm_chip_path + "/export";
            int export_fd = ::open(export_path.c_str(), O_WRONLY);

            if (export_fd < 0) {
                std::cerr << "Failed to export PWM channel" << std::endl;
                return false;
            }

            std::string channel = std::to_string(gpio_->getPinNumber());
            write(export_fd, channel.c_str(), channel.size());
            ::close(export_fd);

            // Set PWM period (in nanoseconds)
            // Period = 1/frequency * 1e9
            // For 50Hz: 20000000 ns = 20ms
            std::string pwm_path = pwm_chip_path + "/pwm" +
                                    std::to_string(gpio_->getPinNumber());

            std::string period_path = pwm_path + "/period";
            int period_fd = ::open(period_path.c_str(), O_WRONLY);

            if (period_fd < 0) {
                std::cerr << "Failed to set PWM period" << std::endl;
                return false;
            }

            long period_ns = 1000000000L / pwm_frequency_;
            std::string period_str = std::to_string(period_ns);
            write(period_fd, period_str.c_str(), period_str.size());
            ::close(period_fd);

            // Set initial duty cycle (pulse width)
            setPulseWidth(pulse_width_us_);

            // Enable PWM
            std::string enable_path = pwm_path + "/enable";
            int enable_fd = ::open(enable_path.c_str(), O_WRONLY);

            if (enable_fd < 0) {
                std::cerr << "Failed to enable PWM" << std::endl;
                return false;
            }

            write(enable_fd, "1", 1);
            ::close(enable_fd);

            is_running_ = true;
            std::cout << "PWM started on pin: " << gpio_->getPinNumber()
                    << " at " << pwm_frequency_ << "Hz" << std::endl;
            return true;
        }

        void PwmController::stop() {
            if (!is_running_) {
                return;
            }

            std::string pwm_path = "/sys/class/pwm/pwmchip0/pwm" +
                                    std::to_string(gpio_->getPinNumber());

            // Disable PWM
            std::string enable_path = pwm_path + "/enable";
            int enable_fd = ::open(enable_path.c_str(), O_WRONLY);

            if (enable_fd >= 0) {
                write(enable_fd, "0", 1);
                ::close(enable_fd);
            }

            // Unexport PWM channel
            std::string unexport_path = "/sys/class/pwm/pwmchip0/unexport";
            int unexport_fd = ::open(unexport_path.c_str(), O_WRONLY);

            if (unexport_fd >= 0) {
                std::string channel = std::to_string(gpio_->getPinNumber());
                write(unexport_fd, channel.c_str(), channel.size());
                ::close(unexport_fd);
            }

            is_running_ = false;
            std::cout << "PWM stopped on pin: "
                    << gpio_->getPinNumber() << std::endl;
        }

        bool PwmController::setPulseWidth(double pulse_width_us) {
            if (!is_running_ && !gpio_->isOpen()) {
                return false;
            }

            // Clamp pulse width to valid range
            if (pulse_width_us < 500.0)  pulse_width_us = 500.0;
            if (pulse_width_us > 2500.0) pulse_width_us = 2500.0;

            pulse_width_us_ = pulse_width_us;

            // Convert microseconds to nanoseconds
            long duty_ns = static_cast<long>(pulse_width_us * 1000.0);

            std::string pwm_path = "/sys/class/pwm/pwmchip0/pwm" +
                                    std::to_string(gpio_->getPinNumber());

            std::string duty_path = pwm_path + "/duty_cycle";
            int duty_fd = ::open(duty_path.c_str(), O_WRONLY);

            if (duty_fd < 0) {
                std::cerr << "Failed to set duty cycle for pin: "
                        << gpio_->getPinNumber() << std::endl;
                return false;
            }

            std::string duty_str = std::to_string(duty_ns);
            write(duty_fd, duty_str.c_str(), duty_str.size());
            ::close(duty_fd);

            return true;
        }

        bool PwmController::isRunning() const {
            return is_running_;
        }

        double PwmController::getPulseWidth() const {
            return pulse_width_us_;
        }

        int PwmController::getPwmFrequency() const {
            return pwm_frequency_;
        }

    } 
} 