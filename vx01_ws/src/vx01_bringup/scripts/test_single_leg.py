#!/usr/bin/env python3
"""
Single Leg Test Script for VX-01 Hexapod
Tests each leg individually by moving joints
through their range of motion

Usage:
  python3 test_single_leg.py --leg 0        # Test leg 0
  python3 test_single_leg.py --leg 0 --joint coxa   # Test only coxa
  python3 test_single_leg.py --all          # Test all legs one by one
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import argparse
import time
import sys


class SingleLegTester(Node):

    def __init__(self, leg_number: int, joint_name: str):
        super().__init__('single_leg_tester')

        self.leg_number = leg_number
        self.joint_name = joint_name

        # All 18 joint names
        self.all_joints = [
            'coxa_leg0_joint',  'femur_leg0_joint',  'tibia_leg0_joint',
            'coxa_leg1_joint',  'femur_leg1_joint',  'tibia_leg1_joint',
            'coxa_leg2_joint',  'femur_leg2_joint',  'tibia_leg2_joint',
            'coxa_leg3_joint',  'femur_leg3_joint',  'tibia_leg3_joint',
            'coxa_leg4_joint',  'femur_leg4_joint',  'tibia_leg4_joint',
            'coxa_leg5_joint',  'femur_leg5_joint',  'tibia_leg5_joint',
        ]

        # Home position for all joints
        # theta1=0.0, theta2=0.6911, theta3=-2.0647
        self.home_positions = [
            0.0,  0.6911, -2.0647,   # Leg 0
            0.0,  0.6911, -2.0647,   # Leg 1
            0.0,  0.6911, -2.0647,   # Leg 2
            0.0,  0.6911, -2.0647,   # Leg 3
            0.0,  0.6911, -2.0647,   # Leg 4
            0.0,  0.6911, -2.0647,   # Leg 5
        ]

        # Current commanded positions
        self.current_positions = list(self.home_positions)

        # Joint limits
        self.joint_limits = {
            'coxa':  {'min': -1.5707, 'max': 1.5707},
            'femur': {'min': -1.5707, 'max': 1.5707},
            'tibia': {'min': -2.3561, 'max': 0.0},
        }

        # Publisher to position controller
        self.cmd_pub = self.create_publisher(
            Float64MultiArray,
            '/hexapod_position_controller/commands',
            10
        )

        # Subscriber to joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Current actual positions
        self.actual_positions = {}

        self.get_logger().info(
            f'SingleLegTester initialized for Leg {leg_number}')

    def joint_state_callback(self, msg: JointState):
        """Store actual joint positions"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.actual_positions[name] = msg.position[i]

    def get_joint_index(self, leg: int, joint: str) -> int:
        """Get index in position array for a specific joint"""
        joint_map = {
            'coxa':  leg * 3 + 0,
            'femur': leg * 3 + 1,
            'tibia': leg * 3 + 2,
        }
        return joint_map.get(joint, -1)

    def publish_positions(self):
        """Publish current commanded positions"""
        msg = Float64MultiArray()
        msg.data = self.current_positions
        self.cmd_pub.publish(msg)

    def move_to_home(self):
        """Move all joints to home position"""
        self.get_logger().info('Moving to HOME position...')
        self.current_positions = list(self.home_positions)
        self.publish_positions()
        time.sleep(2.0)
        self.get_logger().info('HOME position reached')

    def move_joint(self, leg: int, joint: str,
                   angle: float, wait: float = 1.5):
        """Move a single joint to a specific angle"""
        index = self.get_joint_index(leg, joint)
        if index < 0:
            self.get_logger().error(f'Invalid joint: {joint}')
            return

        # Clamp to limits
        limits = self.joint_limits[joint]
        angle = max(limits['min'], min(limits['max'], angle))

        self.current_positions[index] = angle
        self.publish_positions()

        self.get_logger().info(
            f'  Leg {leg} {joint}: {angle:.4f} rad '
            f'({(angle * 57.2958):.1f} deg)')

        time.sleep(wait)

    def test_coxa(self, leg: int):
        """Test coxa joint (horizontal rotation)"""
        self.get_logger().info(
            f'--- Testing LEG {leg} COXA (horizontal) ---')

        # Center
        self.get_logger().info('  Coxa: CENTER (0 deg)')
        self.move_joint(leg, 'coxa', 0.0)

        # Right (+45 deg)
        self.get_logger().info('  Coxa: RIGHT (+45 deg)')
        self.move_joint(leg, 'coxa', 0.7853)

        # Center
        self.get_logger().info('  Coxa: CENTER')
        self.move_joint(leg, 'coxa', 0.0)

        # Left (-45 deg)
        self.get_logger().info('  Coxa: LEFT (-45 deg)')
        self.move_joint(leg, 'coxa', -0.7853)

        # Back to center
        self.get_logger().info('  Coxa: CENTER')
        self.move_joint(leg, 'coxa', 0.0)

    def test_femur(self, leg: int):
        """Test femur joint (up/down)"""
        self.get_logger().info(
            f'--- Testing LEG {leg} FEMUR (up/down) ---')

        # Home position
        self.get_logger().info('  Femur: HOME (39.6 deg)')
        self.move_joint(leg, 'femur', 0.6911)

        # Up (-45 deg)
        self.get_logger().info('  Femur: UP (-45 deg)')
        self.move_joint(leg, 'femur', -0.7853)

        # Home
        self.get_logger().info('  Femur: HOME')
        self.move_joint(leg, 'femur', 0.6911)

        # Down (+45 deg)
        self.get_logger().info('  Femur: DOWN (+45 deg)')
        self.move_joint(leg, 'femur', 0.7853)

        # Back to home
        self.get_logger().info('  Femur: HOME')
        self.move_joint(leg, 'femur', 0.6911)

    def test_tibia(self, leg: int):
        """Test tibia joint (knee bend)"""
        self.get_logger().info(
            f'--- Testing LEG {leg} TIBIA (knee) ---')

        # Home position
        self.get_logger().info('  Tibia: HOME (-118.3 deg)')
        self.move_joint(leg, 'tibia', -2.0647)

        # Straight (-45 deg)
        self.get_logger().info('  Tibia: STRAIGHT (-45 deg)')
        self.move_joint(leg, 'tibia', -0.7853)

        # Home
        self.get_logger().info('  Tibia: HOME')
        self.move_joint(leg, 'tibia', -2.0647)

        # Bent more (-135 deg)
        self.get_logger().info('  Tibia: BENT (-135 deg)')
        self.move_joint(leg, 'tibia', -2.3561)

        # Back to home
        self.get_logger().info('  Tibia: HOME')
        self.move_joint(leg, 'tibia', -2.0647)

    def test_full_leg(self, leg: int):
        """Test complete leg - all 3 joints"""
        self.get_logger().info(
            f'===== TESTING FULL LEG {leg} =====')

        # Move to home first
        self.move_to_home()

        # Test each joint
        self.test_coxa(leg)
        time.sleep(0.5)

        self.test_femur(leg)
        time.sleep(0.5)

        self.test_tibia(leg)
        time.sleep(0.5)

        # End at home
        self.move_to_home()

        self.get_logger().info(
            f'===== LEG {leg} TEST COMPLETE =====')

    def test_lift_leg(self, leg: int):
        """Test lifting leg - realistic motion"""
        self.get_logger().info(
            f'--- Testing LEG {leg} LIFT motion ---')

        # Home
        self.move_joint(leg, 'coxa',  0.0)
        self.move_joint(leg, 'femur', 0.6911)
        self.move_joint(leg, 'tibia', -2.0647)
        time.sleep(1.0)

        # Lift up
        self.get_logger().info('  LIFTING leg...')
        self.move_joint(leg, 'femur', 0.3)
        self.move_joint(leg, 'tibia', -1.5)
        time.sleep(1.0)

        # Step forward
        self.get_logger().info('  STEPPING forward...')
        self.move_joint(leg, 'coxa', 0.3)
        time.sleep(1.0)

        # Put down
        self.get_logger().info('  PUTTING DOWN leg...')
        self.move_joint(leg, 'femur', 0.6911)
        self.move_joint(leg, 'tibia', -2.0647)
        time.sleep(1.0)

        # Back to home
        self.move_joint(leg, 'coxa', 0.0)
        time.sleep(1.0)

        self.get_logger().info('  LIFT test complete')

    def run_test(self):
        """Run the configured test"""
        # Wait for subscribers
        self.get_logger().info('Waiting for controller...')
        time.sleep(2.0)

        # Move to home first
        self.move_to_home()

        if self.joint_name == 'all':
            # Test all joints of the leg
            self.test_full_leg(self.leg_number)
        elif self.joint_name == 'coxa':
            self.test_coxa(self.leg_number)
            self.move_to_home()
        elif self.joint_name == 'femur':
            self.test_femur(self.leg_number)
            self.move_to_home()
        elif self.joint_name == 'tibia':
            self.test_tibia(self.leg_number)
            self.move_to_home()
        elif self.joint_name == 'lift':
            self.test_lift_leg(self.leg_number)
            self.move_to_home()
        else:
            self.get_logger().error(f'Unknown joint: {self.joint_name}')


def test_all_legs(node_class):
    """Test all 6 legs one by one"""
    for leg in range(6):
        print(f'\n=============================')
        print(f'  STARTING TEST: LEG {leg}')
        print(f'=============================\n')

        rclpy.init()
        tester = node_class(leg, 'all')

        try:
            tester.run_test()
            rclpy.spin_once(tester, timeout_sec=1.0)
        except KeyboardInterrupt:
            print('\nTest interrupted!')
        finally:
            tester.destroy_node()
            rclpy.shutdown()

        print(f'\nLeg {leg} done! Press Enter for next leg...')
        input()


def main():
    parser = argparse.ArgumentParser(
        description='VX-01 Single Leg Test Script')

    parser.add_argument(
        '--leg', type=int, default=0,
        choices=[0, 1, 2, 3, 4, 5],
        help='Leg number to test (0-5)')

    parser.add_argument(
        '--joint', type=str, default='all',
        choices=['all', 'coxa', 'femur', 'tibia', 'lift'],
        help='Joint to test (default: all)')

    parser.add_argument(
        '--all-legs', action='store_true',
        help='Test all 6 legs one by one')

    args = parser.parse_args()

    if args.all_legs:
        test_all_legs(SingleLegTester)
        return

    # Test single leg
    rclpy.init()
    tester = SingleLegTester(args.leg, args.joint)

    print(f'\n=============================')
    print(f'  TESTING LEG {args.leg}')
    print(f'  JOINT: {args.joint}')
    print(f'=============================\n')

    try:
        tester.run_test()
        rclpy.spin_once(tester, timeout_sec=1.0)

    except KeyboardInterrupt:
        print('\nTest interrupted by user!')
        tester.move_to_home()

    finally:
        tester.destroy_node()
        rclpy.shutdown()

    print('\nTest complete!')


if __name__ == '__main__':
    main()