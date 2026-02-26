#!/usr/bin/env python3
"""
hexapod_teleop_key.py
─────────────────────
Keyboard teleoperation for the VX-01 hexapod.
Publishes geometry_msgs/Twist on /cmd_vel.

Key bindings
────────────
  W / S   : forward / backward
  A / D   : strafe left / strafe right
  Q / E   : rotate left (CCW) / rotate right (CW)
  SPACE   : stop immediately
  +  / -  : increase / decrease linear speed
  [ / ]   : increase / decrease angular speed
  CTRL+C  : quit

The hexapod_walk_node subscribes to /cmd_vel and converts:
  linear.x  (m/s) → mm/s  forward velocity
  linear.y  (m/s) → mm/s  lateral velocity
  angular.z (rad/s) → yaw rate

Run with:
  ros2 run vx01_bringup hexapod_teleop_key
"""

import sys
import tty
import termios
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# ── Key mappings ──────────────────────────────────────────────────────────
KEY_BINDINGS = {
    'w': ( 1,  0,  0),   # forward
    's': (-1,  0,  0),   # backward
    'a': ( 0,  1,  0),   # strafe left
    'd': ( 0, -1,  0),   # strafe right
    'q': ( 0,  0,  1),   # rotate CCW
    'e': ( 0,  0, -1),   # rotate CW
    'W': ( 1,  0,  0),
    'S': (-1,  0,  0),
    'A': ( 0,  1,  0),
    'D': ( 0, -1,  0),
    'Q': ( 0,  0,  1),
    'E': ( 0,  0, -1),
}

SPEED_KEYS  = {'+': 1.1, '=': 1.1, '-': 0.9, '_': 0.9}
ANGULAR_KEYS = {']': 1.1, '[': 0.9}

STOP_KEYS   = {' ', '\x03'}   # Space or Ctrl+C

BANNER = """
╔══════════════════════════════════════════╗
║      VX-01 Hexapod Keyboard Teleop       ║
╠══════════════════════════════════════════╣
║  W/S  : Forward / Backward               ║
║  A/D  : Strafe Left / Right              ║
║  Q/E  : Rotate CCW / CW                  ║
║  SPACE: Stop                             ║
║  +/-  : Speed up / down (linear)         ║
║  [/]  : Speed up / down (angular)        ║
║  Ctrl+C: Quit                            ║
╚══════════════════════════════════════════╝
Linear speed: {lin:.3f} m/s   Angular speed: {ang:.3f} rad/s
"""

# ═════════════════════════════════════════════════════════════════════════
class HexapodTeleopNode(Node):

    def __init__(self):
        super().__init__('hexapod_teleop_key')

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Default speeds
        self.linear_speed  = 0.05   # m/s  → 50 mm/s in the locomotion library
        self.angular_speed = 0.3    # rad/s

        # Current velocity state
        self.vx    = 0.0
        self.vy    = 0.0
        self.omega = 0.0

        # Publish at 10 Hz so the walk node keeps getting updates
        self.timer_ = self.create_timer(0.1, self.publish_cmd)

        self.get_logger().info("Teleop node started")

    def publish_cmd(self):
        msg = Twist()
        msg.linear.x  = self.vx
        msg.linear.y  = self.vy
        msg.angular.z = self.omega
        self.publisher_.publish(msg)

    def set_velocity(self, vx, vy, omega):
        self.vx    = vx
        self.vy    = vy
        self.omega = omega

    def stop(self):
        self.vx = self.vy = self.omega = 0.0
        # Publish stop immediately
        self.publish_cmd()


# ── Terminal helpers ──────────────────────────────────────────────────────
def get_key(settings):
    """Read one keypress from stdin (raw mode)."""
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def print_status(lin, ang):
    sys.stdout.write('\r' + ' ' * 60)
    sys.stdout.write(
        f'\r  Linear: {lin:.3f} m/s   Angular: {ang:.3f} rad/s   '
        f'(+/- to change speed)')
    sys.stdout.flush()


# ── Main ─────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = HexapodTeleopNode()

    # Spin in a background thread so we can block on keyboard input here
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    settings = termios.tcgetattr(sys.stdin)

    print(BANNER.format(lin=node.linear_speed, ang=node.angular_speed))

    try:
        while rclpy.ok():
            key = get_key(settings)

            if key in STOP_KEYS:
                node.stop()
                if key == '\x03':   # Ctrl+C
                    print("\nStopping and quitting...")
                    break
                else:
                    print_status(node.linear_speed, node.angular_speed)
                continue

            if key in KEY_BINDINGS:
                dx, dy, dw = KEY_BINDINGS[key]
                node.set_velocity(
                    dx * node.linear_speed,
                    dy * node.linear_speed,
                    dw * node.angular_speed
                )
                print_status(node.linear_speed, node.angular_speed)

            elif key in SPEED_KEYS:
                node.linear_speed *= SPEED_KEYS[key]
                node.linear_speed  = max(0.005, min(0.5, node.linear_speed))
                print_status(node.linear_speed, node.angular_speed)

            elif key in ANGULAR_KEYS:
                node.angular_speed *= ANGULAR_KEYS[key]
                node.angular_speed  = max(0.05, min(2.0, node.angular_speed))
                print_status(node.linear_speed, node.angular_speed)

    except Exception as e:
        print(f"\nException: {e}")
    finally:
        # Restore terminal settings and send stop
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.stop()
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)


if __name__ == '__main__':
    main()