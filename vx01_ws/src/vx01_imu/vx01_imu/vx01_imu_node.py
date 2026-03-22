#!/usr/bin/env python3

import math
import struct
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
import serial


# ---------------------------------------------------------------------------
# VX-01 IMU driver — HiWonder 10-axis IMU (Witmotion binary protocol)
# Packet format: 0x55 <type> <8 bytes> <checksum>
# Types: 0x51 acceleration | 0x52 angular velocity | 0x53 angle | 0x54 mag
# ---------------------------------------------------------------------------

class VX01ImuNode(Node):

    def __init__(self):
        super().__init__('vx01_imu_node')

        self.declare_parameter('port',      '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 9600)
        self.declare_parameter('frame_id',  'imu_link')

        self.port     = self.get_parameter('port').value
        self.baud     = self.get_parameter('baud_rate').value
        self.frame_id = self.get_parameter('frame_id').value

        self.pub_imu = self.create_publisher(Imu,           '/imu',     10)
        self.pub_mag = self.create_publisher(MagneticField, '/imu/mag', 10)

        # Latest parsed values
        self.acceleration    = [0.0, 0.0, 0.0]
        self.angular_velocity = [0.0, 0.0, 0.0]
        self.angle_degree    = [0.0, 0.0, 0.0]
        self.magnetometer    = [0.0, 0.0, 0.0]

        # Serial read buffer
        self.key  = 0
        self.buff = {}

        self._open_serial()

        self.driver_thread = threading.Thread(target=self._read_loop, daemon=True)
        self.driver_thread.start()

    def _open_serial(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.5)
            self.get_logger().info(f'IMU opened on {self.port} at {self.baud} baud')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open {self.port}: {e}')
            raise

    def _read_loop(self):
        while rclpy.ok():
            try:
                waiting = self.ser.inWaiting()
            except Exception as e:
                self.get_logger().error(f'IMU serial error: {e}')
                return

            if waiting > 0:
                data = self.ser.read(waiting)
                for byte in data:
                    if self._handle_byte(byte):
                        self._publish_imu()

    def _handle_byte(self, raw_byte: int) -> bool:
        """Feed one byte into the packet parser. Returns True when angle packet complete."""
        self.buff[self.key] = raw_byte
        self.key += 1

        if self.buff[0] != 0x55:
            self.key = 0
            self.buff = {}
            return False

        if self.key < 11:
            return False

        data = list(self.buff.values())
        angle_ready = False

        if not (sum(data[0:10]) & 0xFF == data[10]):
            self.key = 0
            self.buff = {}
            return False

        ptype = self.buff[1]

        if ptype == 0x51:
            vals = self._unpack(data[2:10])
            self.acceleration = [vals[i] / 32768.0 * 16 * 9.8 for i in range(3)]

        elif ptype == 0x52:
            vals = self._unpack(data[2:10])
            self.angular_velocity = [vals[i] / 32768.0 * 2000 * math.pi / 180 for i in range(3)]

        elif ptype == 0x53:
            vals = self._unpack(data[2:10])
            self.angle_degree = [vals[i] / 32768.0 * 180 for i in range(3)]
            angle_ready = True

        elif ptype == 0x54:
            vals = self._unpack(data[2:10])
            self.magnetometer = [float(vals[i]) for i in range(3)]
            self._publish_mag()

        self.key = 0
        self.buff = {}
        return angle_ready

    def _unpack(self, raw: list) -> list:
        return list(struct.unpack('hhhh', bytearray(raw)))

    def _publish_imu(self):
        msg                 = Imu()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        # Convert Euler angles (degrees) to radians then to quaternion
        roll  = self.angle_degree[0] * math.pi / 180
        pitch = self.angle_degree[1] * math.pi / 180
        yaw   = self.angle_degree[2] * math.pi / 180

        qx, qy, qz, qw = self._euler_to_quaternion(roll, pitch, yaw)
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw

        msg.angular_velocity.x = self.angular_velocity[0]
        msg.angular_velocity.y = self.angular_velocity[1]
        msg.angular_velocity.z = self.angular_velocity[2]

        msg.linear_acceleration.x = self.acceleration[0]
        msg.linear_acceleration.y = self.acceleration[1]
        msg.linear_acceleration.z = self.acceleration[2]

        # Diagonal covariance — conservative estimates
        msg.orientation_covariance[0]         = 0.01
        msg.orientation_covariance[4]         = 0.01
        msg.orientation_covariance[8]         = 0.01
        msg.angular_velocity_covariance[0]    = 0.01
        msg.angular_velocity_covariance[4]    = 0.01
        msg.angular_velocity_covariance[8]    = 0.01
        msg.linear_acceleration_covariance[0] = 0.1
        msg.linear_acceleration_covariance[4] = 0.1
        msg.linear_acceleration_covariance[8] = 0.1

        self.pub_imu.publish(msg)

    def _publish_mag(self):
        msg                  = MagneticField()
        msg.header.stamp     = self.get_clock().now().to_msg()
        msg.header.frame_id  = self.frame_id
        msg.magnetic_field.x = self.magnetometer[0]
        msg.magnetic_field.y = self.magnetometer[1]
        msg.magnetic_field.z = self.magnetometer[2]
        self.pub_mag.publish(msg)

    @staticmethod
    def _euler_to_quaternion(roll: float, pitch: float, yaw: float):
        qx = (np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2)
              - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2))
        qy = (np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
              + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2))
        qz = (np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
              - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2))
        qw = (np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2)
              + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2))
        return qx, qy, qz, qw

    def destroy_node(self):
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VX01ImuNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
