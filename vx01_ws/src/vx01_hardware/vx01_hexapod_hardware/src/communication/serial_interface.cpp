#include "vx01_hexapod_hardware/communication/serial_interface.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <cstring>
#include <iostream>

namespace vx01_hexapod_hardware {

    namespace communication {

        SerialInterface::SerialInterface(const std::string& port_name, int baud_rate)
            : port_name_(port_name), baud_rate_(baud_rate), serial_fd_(-1), is_open_(false) {
        }

        SerialInterface::~SerialInterface() {
            if(is_open_) {
                close();
            }
        }

        bool SerialInterface::open() {

            if (is_open_) {
                return true;
            }

            serial_fd_ = ::open(port_name_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);

            if (serial_fd_ < 0) {
                std::cerr << "Failed to open USB serial port: " << port_name_ << std::endl;
                std::cerr << "Make sure Pololu Maestro is connected via USB" << std::endl;
                return false;
            }

            struct termios options;
            tcgetattr(serial_fd_, &options);

            // set baud rate

            speed_t baud;
            switch (baud_rate_) {
                case 9600:   baud = B9600;   break;
                case 19200:  baud = B19200;  break;
                case 38400:  baud = B38400;  break;
                case 57600:  baud = B57600;  break;
                case 115200: baud = B115200; break;
                default:     baud = B9600;   break;
            }

            cfsetispeed(&options, baud);
            cfsetospeed(&options, baud);

            // 8N1 mode (8 data bits, no parity, 1 stop bit)
            options.c_cflag &= ~PARENB;  // No parity
            options.c_cflag &= ~CSTOPB;  // 1 stop bit
            options.c_cflag &= ~CSIZE;
            options.c_cflag |= CS8;      // 8 data bits
            
            // Disable hardware flow control
            options.c_cflag &= ~CRTSCTS;
            
            // Enable receiver
            options.c_cflag |= CREAD | CLOCAL;
            
            // Raw input mode
            options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
            
            // Raw output mode
            options.c_oflag &= ~OPOST;
            
            // Disable software flow control
            options.c_iflag &= ~(IXON | IXOFF | IXANY);
            
            // Timeout settings (0.1 second timeout)
            options.c_cc[VMIN] = 0;
            options.c_cc[VTIME] = 1;
            
            // Apply settings
            tcsetattr(serial_fd_, TCSANOW, &options);
            
            // Flush buffers
            flush();
            
            is_open_ = true;
            std::cout << "Successfully opened USB connection to Maestro: " << port_name_ << std::endl;
            return true;
        }

        void SerialInterface::close() {
            if (is_open_ && serial_fd_ >= 0) {
                ::close(serial_fd_);
                serial_fd_ = -1;
                is_open_ = false;
                std::cout << "Closed USB connection to Maestro" << std::endl;
            }
        }

        bool SerialInterface::isOpen() const {
            return is_open_;
        }

        bool SerialInterface::writeByte(uint8_t byte) {
            if (!is_open_) {
                return false;
            }

            ssize_t result = write(serial_fd_, &byte, 1);
            return (result == 1);
        }

        bool SerialInterface::writeBytes(const std::vector<uint8_t>& bytes) {
            if (!is_open_ || bytes.empty()) {
                return false;
            }

            ssize_t result = write(serial_fd_, bytes.data(), bytes.size());
            return (result == static_cast<ssize_t>(bytes.size()));
        }

        bool SerialInterface::readByte(uint8_t& byte, int timeout_ms) {
            if (!is_open_) {
                return false;
            }
            
            struct termios options;
            tcgetattr(serial_fd_, &options);
            options.c_cc[VTIME] = timeout_ms / 100;  // Timeout in deciseconds
            tcsetattr(serial_fd_, TCSANOW, &options);
            
            ssize_t result = read(serial_fd_, &byte, 1);
            return (result == 1);
        }

        bool SerialInterface::readBytes(std::vector<uint8_t>& bytes, size_t count, int timeout_ms) {
            if (!is_open_ || count == 0) {
                return false;
            }
            
            bytes.resize(count);
            
            // Set timeout
            struct termios options;
            tcgetattr(serial_fd_, &options);
            options.c_cc[VTIME] = timeout_ms / 100;  // Timeout in deciseconds
            tcsetattr(serial_fd_, TCSANOW, &options);
            
            size_t total_read = 0;
            while (total_read < count) {
                ssize_t result = read(serial_fd_, bytes.data() + total_read, count - total_read);
                if (result <= 0) {
                    return false;
                }
                total_read += result;
            }
            
            return true;
        }

        void SerialInterface::flush() {
            if (is_open_) {
                tcflush(serial_fd_, TCIOFLUSH);
            }
        }

        std::string SerialInterface::getPortName() const {
            return port_name_;
        }

        int SerialInterface::getBaudRate() const {
            return baud_rate_;
        }
    }
}