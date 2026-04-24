"""
aerial_controller_node.py — Velocity passthrough & arm/disarm for VX-01 drone.

Subscribes:
    /drone/cmd_vel   (geometry_msgs/TwistStamped)  — velocity commands
    /drone/arm       (std_msgs/Bool)               — arm (True) / disarm (False)
    /mavros/state    (mavros_msgs/State)            — FCU state

Publishes:
    /mavros/setpoint_velocity/cmd_vel (geometry_msgs/TwistStamped)

Services called:
    /mavros/cmd/arming   (mavros_msgs/CommandBool)
    /mavros/set_mode     (mavros_msgs/SetMode)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Bool
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode


class AerialControllerNode(Node):

    def __init__(self):
        super().__init__('aerial_controller_node')

        # ── State 
        self._connected = False
        self._armed = False
        self._mode = 'STABILIZE'
        self._latest_cmd = TwistStamped()
        self._last_cmd_time = self.get_clock().now()
        self._has_cmd = False

        # ── Subscribers 
        self.create_subscription(
            TwistStamped, '/drone/cmd_vel', self._cmd_vel_cb, 10)
        self.create_subscription(
            State, '/mavros/state', self._state_cb, 10)
        self.create_subscription(
            Bool, '/drone/arm', self._arm_cb, 10)

        # ── Publisher 
        self._vel_pub = self.create_publisher(
            TwistStamped, '/mavros/setpoint_velocity/cmd_vel', 10)

        # ── Service clients 
        self._arm_client = self.create_client(
            CommandBool, '/mavros/cmd/arming')
        self._mode_client = self.create_client(
            SetMode, '/mavros/set_mode')

        # ── Heartbeat timer (10 Hz) 
        self.create_timer(0.1, self._heartbeat)

        self.get_logger().info(
            'AerialController ready. '
            'Publish TwistStamped → /drone/cmd_vel | '
            'Bool → /drone/arm')

    # ── Callbacks 

    def _cmd_vel_cb(self, msg: TwistStamped):
        self._latest_cmd = msg
        self._last_cmd_time = self.get_clock().now()
        self._has_cmd = True

    def _state_cb(self, msg: State):
        self._connected = msg.connected
        self._armed = msg.armed
        self._mode = msg.mode

    def _arm_cb(self, msg: Bool):
        self._send_arm(msg.data)

    # ── Heartbeat — streams setpoints at 10 Hz 

    def _heartbeat(self):
        if not self._connected:
            return

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'

        if self._has_cmd:
            age = (self.get_clock().now() - self._last_cmd_time).nanoseconds / 1e9
            if age < 0.5:
                cmd.twist = self._latest_cmd.twist

        self._vel_pub.publish(cmd)

    # ── Service calls 

    def _send_arm(self, arm: bool):
        if not self._arm_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('Arming service not available')
            return
        req = CommandBool.Request()
        req.value = arm
        future = self._arm_client.call_async(req)
        future.add_done_callback(
            lambda f: self._arm_response(f, arm))

    def _arm_response(self, future, arm: bool):
        try:
            resp = future.result()
            tag = 'ARM' if arm else 'DISARM'
            if resp.success:
                self.get_logger().info(f'{tag} successful')
            else:
                self.get_logger().warn(f'{tag} FAILED (result={resp.result})')
        except Exception as e:
            self.get_logger().error(f'Arm service call failed: {e}')

    def set_mode(self, mode: str):
        """Public helper — used by drone_flight_manager."""
        if not self._mode_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('SetMode service not available')
            return
        req = SetMode.Request()
        req.custom_mode = mode
        future = self._mode_client.call_async(req)
        future.add_done_callback(
            lambda f: self.get_logger().info(
                f'SetMode → {mode}: {"OK" if f.result().mode_sent else "FAIL"}'))


def main(args=None):
    rclpy.init(args=args)
    node = AerialControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
