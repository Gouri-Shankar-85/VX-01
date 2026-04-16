import sys
import tty
import termios

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class TeleopNode(Node):

    LINEAR_STEP = 0.05
    ANGULAR_STEP = 0.3
    MAX_LINEAR = 0.20
    MAX_ANGULAR = 0.60

    def __init__(self):
        super().__init__('teleop_node')
        self._pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._vx = 0.0
        self._vy = 0.0
        self._omega = 0.0

    def _get_key(self) -> str:
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
        return key

    def _clamp(self, val: float, limit: float) -> float:
        return max(-limit, min(limit, val))

    def _publish(self):
        msg = Twist()
        msg.linear.x = self._vx
        msg.linear.y = self._vy
        msg.angular.z = self._omega
        self._pub.publish(msg)
        print(
            f'\r  vx={self._vx:+.2f}  vy={self._vy:+.2f}  omega={self._omega:+.2f}  ',
            end='',
            flush=True,
        )

    def _stop(self):
        self._vx = 0.0
        self._vy = 0.0
        self._omega = 0.0

    def run(self):
        print('VX-01 Hexapod Teleop')
        print('─' * 38)
        print('  W / S   →  forward / backward')
        print('  A / D   →  turn left / turn right')
        print('  Q / E   →  strafe left / strafe right')
        print('  SPACE   →  stop')
        print('  Ctrl+C  →  quit')
        print('─' * 38)
        self._publish()

        while True:
            key = self._get_key()

            if key == 'w':
                self._vx += self.LINEAR_STEP
            elif key == 's':
                self._vx -= self.LINEAR_STEP
            elif key == 'a':
                self._omega += self.ANGULAR_STEP
            elif key == 'd':
                self._omega -= self.ANGULAR_STEP
            elif key == 'q':
                self._vy += self.LINEAR_STEP
            elif key == 'e':
                self._vy -= self.LINEAR_STEP
            elif key == ' ':
                self._stop()
            elif key == '\x03':
                break

            self._vx = self._clamp(self._vx, self.MAX_LINEAR)
            self._vy = self._clamp(self._vy, self.MAX_LINEAR)
            self._omega = self._clamp(self._omega, self.MAX_ANGULAR)

            self._publish()


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    node._stop()
    node._publish()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
