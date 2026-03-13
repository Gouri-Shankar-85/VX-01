#include "vx01_hexapod_hardware/communication/serial_interface.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <cstring>
#include <iostream>
#include <cerrno>

namespace vx01_hexapod_hardware {

    namespace communication {

        SerialInterface::SerialInterface(const std::string& port_name, int baud_rate)
            : port_name_(port_name), baud_rate_(baud_rate), serial_fd_(-1), is_open_(false) {}

        SerialInterface::~SerialInterface() {
            if (is_open_) {
                close();
            }
        }

        // ------------------------------------------------------------------ //
        //  Private helper — configure termios on the already-open fd
        // ------------------------------------------------------------------ //
        bool SerialInterface::applyPortSettings() {
            struct termios options;
            if (tcgetattr(serial_fd_, &options) != 0) {
                std::cerr << "tcgetattr failed: " << strerror(errno) << std::endl;
                return false;
            }

            speed_t baud;
            switch (baud_rate_) {
                case 9600:   baud = B9600;   break;
                case 19200:  baud = B19200;  break;
                case 38400:  baud = B38400;  break;
                case 57600:  baud = B57600;  break;
                case 115200: baud = B115200; break;
                default:
                    std::cerr << "Unsupported baud rate " << baud_rate_
                              << " — falling back to 115200" << std::endl;
                    baud = B115200;
                    break;
            }

            cfsetispeed(&options, baud);
            cfsetospeed(&options, baud);

            // 8N1
            options.c_cflag &= ~PARENB;   // No parity
            options.c_cflag &= ~CSTOPB;   // 1 stop bit
            options.c_cflag &= ~CSIZE;
            options.c_cflag |=  CS8;      // 8 data bits
            options.c_cflag &= ~CRTSCTS;  // No hardware flow control
            options.c_cflag |=  CREAD | CLOCAL;

            // Raw mode
            options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
            options.c_oflag &= ~OPOST;
            options.c_iflag &= ~(IXON | IXOFF | IXANY);
            options.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

            // Non-blocking read with 0.1 s timeout
            options.c_cc[VMIN]  = 0;
            options.c_cc[VTIME] = 1;

            if (tcsetattr(serial_fd_, TCSANOW, &options) != 0) {
                std::cerr << "tcsetattr failed: " << strerror(errno) << std::endl;
                return false;
            }
            return true;
        }

        // ------------------------------------------------------------------ //
        //  open()
        // ------------------------------------------------------------------ //
        bool SerialInterface::open() {
            if (is_open_) {
                return true;
            }

            // O_NDELAY prevents the open() call from blocking waiting for DCD/DSR.
            // We clear it afterwards so reads/writes behave normally.
            serial_fd_ = ::open(port_name_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
            if (serial_fd_ < 0) {
                std::cerr << "Failed to open serial port: " << port_name_
                          << "  errno=" << errno << " (" << strerror(errno) << ")" << std::endl;
                std::cerr << "Make sure the Pololu Maestro is connected via USB" << std::endl;
                return false;
            }

            // Restore blocking I/O
            if (fcntl(serial_fd_, F_SETFL, 0) < 0) {
                std::cerr << "fcntl F_SETFL failed: " << strerror(errno) << std::endl;
                ::close(serial_fd_);
                serial_fd_ = -1;
                return false;
            }

            if (!applyPortSettings()) {
                ::close(serial_fd_);
                serial_fd_ = -1;
                return false;
            }

            flush();

            is_open_ = true;
            std::cout << "Opened serial port " << port_name_
                      << " at " << baud_rate_ << " baud" << std::endl;
            return true;
        }

        // ------------------------------------------------------------------ //
        //  close()
        // ------------------------------------------------------------------ //
        void SerialInterface::close() {
            if (is_open_ && serial_fd_ >= 0) {
                ::close(serial_fd_);
                serial_fd_ = -1;
                is_open_   = false;
                std::cout << "Closed serial port " << port_name_ << std::endl;
            }
        }

        bool SerialInterface::isOpen() const {
            return is_open_;
        }

        // ------------------------------------------------------------------ //
        //  reopen() — call this after an EIO to try to recover
        // ------------------------------------------------------------------ //
        bool SerialInterface::reopen() {
            std::cerr << "Attempting to reopen " << port_name_ << " ..." << std::endl;
            close();
            // Small delay to let the OS settle after a USB glitch
            usleep(200'000);  // 200 ms
            return open();
        }

        // ------------------------------------------------------------------ //
        //  writeByte()
        // ------------------------------------------------------------------ //
        bool SerialInterface::writeByte(uint8_t byte) {
            if (!is_open_) {
                return false;
            }

            ssize_t result = ::write(serial_fd_, &byte, 1);
            if (result != 1) {
                std::cerr << "writeByte failed: errno=" << errno
                          << " (" << strerror(errno) << ")" << std::endl;
                // EIO means the USB device disconnected — mark port as closed
                if (errno == EIO) {
                    is_open_   = false;
                    ::close(serial_fd_);
                    serial_fd_ = -1;
                }
                return false;
            }
            return true;
        }

        // ------------------------------------------------------------------ //
        //  writeBytes()
        // ------------------------------------------------------------------ //
        bool SerialInterface::writeBytes(const std::vector<uint8_t>& bytes) {
            if (!is_open_ || bytes.empty()) {
                return false;
            }

            ssize_t result = ::write(serial_fd_, bytes.data(), bytes.size());
            if (result != static_cast<ssize_t>(bytes.size())) {
                std::cerr << "writeBytes failed: wrote " << result
                          << "/" << bytes.size()
                          << " errno=" << errno
                          << " (" << strerror(errno) << ")" << std::endl;
                // EIO means the USB device disconnected — mark port as closed
                // so the hardware interface can attempt a reconnect
                if (errno == EIO) {
                    is_open_   = false;
                    ::close(serial_fd_);
                    serial_fd_ = -1;
                }
                return false;
            }
            return true;
        }

        // ------------------------------------------------------------------ //
        //  readByte()
        // ------------------------------------------------------------------ //
        bool SerialInterface::readByte(uint8_t& byte, int timeout_ms) {
            if (!is_open_) {
                return false;
            }

            struct termios options;
            tcgetattr(serial_fd_, &options);
            options.c_cc[VTIME] = static_cast<cc_t>(timeout_ms / 100);
            tcsetattr(serial_fd_, TCSANOW, &options);

            ssize_t result = ::read(serial_fd_, &byte, 1);
            if (result != 1) {
                if (errno == EIO) {
                    is_open_   = false;
                    ::close(serial_fd_);
                    serial_fd_ = -1;
                }
                return false;
            }
            return true;
        }

        // ------------------------------------------------------------------ //
        //  readBytes()
        // ------------------------------------------------------------------ //
        bool SerialInterface::readBytes(std::vector<uint8_t>& bytes, size_t count, int timeout_ms) {
            if (!is_open_ || count == 0) {
                return false;
            }

            bytes.resize(count);

            struct termios options;
            tcgetattr(serial_fd_, &options);
            options.c_cc[VTIME] = static_cast<cc_t>(timeout_ms / 100);
            tcsetattr(serial_fd_, TCSANOW, &options);

            size_t total_read = 0;
            while (total_read < count) {
                ssize_t result = ::read(serial_fd_, bytes.data() + total_read, count - total_read);
                if (result <= 0) {
                    if (errno == EIO) {
                        is_open_   = false;
                        ::close(serial_fd_);
                        serial_fd_ = -1;
                    }
                    return false;
                }
                total_read += result;
            }
            return true;
        }

        // ------------------------------------------------------------------ //
        //  flush()
        // ------------------------------------------------------------------ //
        void SerialInterface::flush() {
            if (is_open_ && serial_fd_ >= 0) {
                tcflush(serial_fd_, TCIOFLUSH);
            }
        }

        std::string SerialInterface::getPortName() const { return port_name_; }
        int         SerialInterface::getBaudRate()  const { return baud_rate_; }

    }  // namespace communication
}  // namespace vx01_hexapod_hardware