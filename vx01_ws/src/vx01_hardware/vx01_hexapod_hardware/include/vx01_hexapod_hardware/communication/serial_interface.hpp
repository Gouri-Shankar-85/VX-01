#ifndef VX01_HEXAPOD_HARDWARE_SERIAL_INTERFACE_HPP
#define VX01_HEXAPOD_HARDWARE_SERIAL_INTERFACE_HPP

#include <string>
#include <vector>
#include <atomic>

namespace vx01_hexapod_hardware {

    namespace communication {

        class SerialInterface {

            private:
                std::string port_name_;
                int baud_rate_;
                int serial_fd_;
                bool is_open_;

                // Apply termios settings to the open fd
                bool applyPortSettings();

            public:
                SerialInterface(const std::string& port_name, int baud_rate);

                ~SerialInterface();

                bool open();

                void close();

                bool isOpen() const;

                // Attempt to close and reopen the port (call after EIO)
                bool reopen();

                bool writeByte(uint8_t byte);

                bool writeBytes(const std::vector<uint8_t>& bytes);

                bool readByte(uint8_t& byte, int timeout_ms = 100);

                bool readBytes(std::vector<uint8_t>& bytes, size_t count, int timeout_ms = 100);

                void flush();

                std::string getPortName() const;

                int getBaudRate() const;
        };
    }
}

#endif