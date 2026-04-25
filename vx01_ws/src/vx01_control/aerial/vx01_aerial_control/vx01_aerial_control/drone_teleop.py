#!/usr/bin/env python3

import sys
import tty
import termios
import threading
import rclpy
from rclpy.node import Node
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.srv import CommandBool, SetMode

msg = """
========================================
VX-01 Drone Manual Teleop Active
========================================

Flight Controls:
[ W/S ] : Pitch (Forward / Backward)
[ A/D ] : Roll (Left / Right)
[ Q/E ] : Yaw (Rotate Left / Right)
[ UP/DOWN Arrows ] : Throttle (Altitude +/-)
[ X ]   : Center all joysticks/sticks immediately

System Commands:
[ Enter ] : ARM drone
[ Space ] : DISARM drone
[ M ]     : Switch to STABILIZE mode
[ N ]     : Switch to ALT_HOLD mode

CTRL-C to quit
"""

class DroneTeleop(Node):
    def __init__(self):
        super().__init__('drone_teleop')
        self._rc_pub = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)
        self._arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self._mode_client = self.create_client(SetMode, '/mavros/set_mode')

        self._channels = [0] * 18
        # Mode 2 mapping
        self._channels[0] = 1500 # Roll
        self._channels[1] = 1500 # Pitch
        self._channels[2] = 1000 # Throttle default
        self._channels[3] = 1500 # Yaw
        
        self._step = 25 # Increment step
        
        self.create_timer(0.05, self._publish_rc)
        
    def _publish_rc(self):
        rc_msg = OverrideRCIn()
        rc_msg.channels = self._channels
        self._rc_pub.publish(rc_msg)

    def set_arm(self, arm_state):
        if not self._arm_client.wait_for_service(timeout_sec=1.0):
            return
        req = CommandBool.Request()
        req.value = arm_state
        self._arm_client.call_async(req)

    def set_mode(self, mode):
        if not self._mode_client.wait_for_service(timeout_sec=1.0):
            return
        req = SetMode.Request()
        req.custom_mode = mode
        self._mode_client.call_async(req)

    def handle_key(self, key):
        # Roll (Channel 0)
        if key == 'a':
            self._channels[0] = max(1000, self._channels[0] - self._step)
        elif key == 'd':
            self._channels[0] = min(2000, self._channels[0] + self._step)
        
        # Pitch (Channel 1) (Forward is 1000, Back is 2000)
        elif key == 'w':
            self._channels[1] = max(1000, self._channels[1] - self._step)
        elif key == 's':
            self._channels[1] = min(2000, self._channels[1] + self._step)

        # Yaw (Channel 3)
        elif key == 'q':
            self._channels[3] = max(1000, self._channels[3] - self._step)
        elif key == 'e':
            self._channels[3] = min(2000, self._channels[3] + self._step)

        # Throttle (Channel 2)
        elif key == '\x1b[A': # Up Arrow
            self._channels[2] = min(2000, self._channels[2] + self._step)
        elif key == '\x1b[B': # Down Arrow
            self._channels[2] = max(1000, self._channels[2] - self._step)
            
        # Commands
        elif key == '\r' or key == '\n':
            self.set_mode('STABILIZE')
            self.set_arm(True)
        elif key == ' ':
            self.set_arm(False)
            self._channels[2] = 1000 # Drop throttle instantly on disarm!
        elif key == 'm':
            self.set_mode('STABILIZE')
        elif key == 'n':
            self.set_mode('ALT_HOLD')
            
        elif key == 'x': # Reset to neutral
            self._channels[0] = 1500
            self._channels[1] = 1500
            self._channels[3] = 1500
            print("\nSticks centered!")


def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    if key == '\x1b':
        key += sys.stdin.read(2)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main():
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = DroneTeleop()
    
    print(msg)
    
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    
    try:
        while True:
            key = get_key(settings)
            if key == '\x03': # CTRL-C
                break
            
            node.handle_key(key)
            
            # Print current state
            r = int((node._channels[0] - 1500) / 5.0)
            p = int((node._channels[1] - 1500) / 5.0)
            t = int((node._channels[2] - 1000) / 10.0)
            y = int((node._channels[3] - 1500) / 5.0)
            sys.stdout.write(f"\rRoll: {r:>4}% | Pitch: {p:>4}% | Thr: {t:>3}% | Yaw: {y:>4}%   ")
            sys.stdout.flush()
            
    except Exception as e:
        print(e)
    finally:
        node.set_arm(False)
        rc_msg = OverrideRCIn()
        rc_msg.channels = [0] * 18 # Unset overrides
        node._rc_pub.publish(rc_msg)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        rclpy.shutdown()


if __name__ == '__main__':
    main()
