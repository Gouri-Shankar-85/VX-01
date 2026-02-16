#include "vx01_hexapod_hardware/communication/maestro_protocol.hpp"
#include <iostream>
#include <cmath>

namespace vx01_hexapod_hardware {

    namespace communication {

        MaestroProtocol::MaestroProtocol(std::shared_ptr<SerialInterface> serial, uint8_t device_number)
            :serial_(serial), device_number_(device_number) {}

        bool MaestroProtocol::setTarget(uint8_t channel, uint16_t target) {
            
            if (!serial_->isOpen()) {
                std::cerr << "Serialport not opened" << std::endl;
                return false;
            }

            if (channel > 17) {
                std::cerr << "Invalid Channel: " << static_cast<int>(channel) <<std::endl;
                return false;
            }

            // Pololu Protocol: Set Target
            // Command: 0x84, channel, target_low, target_high
            std::vector<uint8_t> command(4);
            command[0] = 0x84;
            command[1] = channel;
            command[2] = target & 0x7F;
            command[3] = (target >> 7) & 0x7F;

            return serial_->writeBytes(command);
        }

        bool MaestroProtocol::setSpeed(uint8_t channel, uint16_t speed) {

            if (!serial_->isOpen()) {
                return false;
            }
            
            if (channel > 17) {
                return false;
            }
            
            // Pololu Protocol: Set Speed
            // Command: 0x87, channel, speed_low, speed_high
            std::vector<uint8_t> command(4);
            command[0] = 0x87;  // Set Speed command
            command[1] = channel;
            command[2] = speed & 0x7F;
            command[3] = (speed >> 7) & 0x7F;
            
            return serial_->writeBytes(command);
        }

        bool MaestroProtocol::setAcceleration(uint8_t channel, uint16_t acceleration) {

            if (!serial_->isOpen()) {
                return false;
            }
            
            if (channel > 17) {
                return false;
            }
            
            // Pololu Protocol: Set Acceleration
            // Command: 0x89, channel, accel_low, accel_high
            std::vector<uint8_t> command(4);
            command[0] = 0x89;  // Set Acceleration command
            command[1] = channel;
            command[2] = acceleration & 0x7F;
            command[3] = (acceleration >> 7) & 0x7F;
            
            return serial_->writeBytes(command);
        }

        bool MaestroProtocol::getPosition(uint8_t channel, uint16_t& position) {

            if (!serial_->isOpen()) {
                return false;
            }
            
            if (channel > 17) {
                return false;
            }
            
            // Pololu Protocol: Get Position
            // Command: 0x90, channel
            // Response: position_low, position_high
            std::vector<uint8_t> command(2);
            command[0] = 0x90;  // Get Position command
            command[1] = channel;
            
            if (!serial_->writeBytes(command)) {
                return false;
            }
            
            // Read response (2 bytes)
            std::vector<uint8_t> response;
            if (!serial_->readBytes(response, 2, 200)) {
                return false;
            }
            
            position = response[0] + (response[1] << 8);
            return true;
        }

        bool MaestroProtocol::getMovingState(bool& is_moving) {

            if (!serial_->isOpen()) {
                return false;
            }
            
            // Pololu Protocol: Get Moving State
            // Command: 0x93
            // Response: 0x00 if not moving, 0x01 if moving
            uint8_t command = 0x93;
            
            if (!serial_->writeByte(command)) {
                return false;
            }
            
            uint8_t response;
            if (!serial_->readByte(response, 200)) {
                return false;
            }
            
            is_moving = (response != 0x00);
            return true;
        }

        bool MaestroProtocol::getErrors(uint16_t& errors) {

            if (!serial_->isOpen()) {
                return false;
            }
            
            // Pololu Protocol: Get Errors
            // Command: 0xA1
            // Response: error_low, error_high
            uint8_t command = 0xA1;
            
            if (!serial_->writeByte(command)) {
                return false;
            }
            
            std::vector<uint8_t> response;
            if (!serial_->readBytes(response, 2, 200)) {
                return false;
            }
            
            errors = response[0] + (response[1] << 8);
            return true;
        }

        bool MaestroProtocol::goHome() {

            if (!serial_->isOpen()) {
                return false;
            }
            
            // Pololu Protocol: Go Home
            // Command: 0xA2
            uint8_t command = 0xA2;
            
            return serial_->writeByte(command);
        }

        uint16_t MaestroProtocol::microsecondsToTarget(double microseconds) {

            return static_cast<uint16_t>(microseconds * 4.0);
        }

        double MaestroProtocol::targetToMicroseconds(uint16_t target) {
            return static_cast<double>(target) / 4.0;
        }

        uint16_t MaestroProtocol::angleToTarget(double angle, double min_angle, double max_angle) {

            double min_us = 500.0;
            double max_us = 2500.0;
            
            double normalized = (angle - min_angle) / (max_angle - min_angle);
            double microseconds = min_us + normalized * (max_us - min_us);
            
            if (microseconds < min_us) microseconds = min_us;
            if (microseconds > max_us) microseconds = max_us;
            
            return microsecondsToTarget(microseconds);
        }

        double MaestroProtocol::targetToAngle(uint16_t target, double min_angle, double max_angle) {
            double microseconds = targetToMicroseconds(target);
            
            double min_us = 500.0;
            double max_us = 2500.0;
            
            double normalized = (microseconds - min_us) / (max_us - min_us);
            return min_angle + normalized * (max_angle - min_angle);
        }
    }
}