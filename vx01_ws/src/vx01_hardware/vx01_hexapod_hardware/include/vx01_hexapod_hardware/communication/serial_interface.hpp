#ifndef VX01_HEXAPOD_HARDWARE_SERIAL_INTERFACE_HPP
#define VX01_HEXAPOD_HARDWARE_SERIAL_INTERFACE_HPP

#include <string>
#include <vector>

namespace vx01_hexapod_hardware {

    namespace communication {

        class SerialInterface {

            private:
                std::string port_name_;
                int baud_rate_;
                int serial_fd_;
                bool is_open_;

            public: 
                SerialInterface(const std::string& port_name, int baud_rate);

                ~SerialInterface();

                bool open(); // open serial port

                void close(); //close serial port

                bool isOpen() const; //check port is open

                bool writeByte(uint8_t byte); //write single byte

                bool writeBytes(const std::vector<uint8_t>& bytes); // Write multiple bytes

                bool readByte(uint8_t& byte, int timeout_ms = 100); // Read single byte (blocking with timeout)

                bool readBytes(std::vector<uint8_t>& bytes, size_t count, int timeout_ms = 100); // Read multiple bytes (blocking with timeout)

                void flush(); // Flush input/output buffers

                std::string getPortName() const; // Get port name

                int getBaudRate() const; //Get baud rate
        };
    }
}

#endif