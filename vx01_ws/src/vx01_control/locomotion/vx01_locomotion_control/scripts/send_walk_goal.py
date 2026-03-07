#!/usr/bin/env python3
"""
send_walk_goal.py – CLI test client for the vx01/walk action server.

Usage examples:
    # Walk forward at 5 cm/s for 10 seconds
    python3 send_walk_goal.py --vx 0.05 --duration 10

    # Turn in place at 0.3 rad/s for 5 seconds
    python3 send_walk_goal.py --omega 0.3 --duration 5

    # Walk forward + strafe left indefinitely (cancel with Ctrl-C)
    python3 send_walk_goal.py --vx 0.05 --vy 0.02 --duration 0

    # Override gait params
    python3 send_walk_goal.py --vx 0.03 --duration 8 \\
        --step-length 90 --step-height 25 --step-period 1.8
"""

import argparse
import sys
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from vx01_locomotion_control.action import Walk


class WalkClient(Node):

    def __init__(self, args: argparse.Namespace):
        super().__init__('walk_test_client')
        self._args   = args
        self._client = ActionClient(self, Walk, 'vx01/walk')

    def send(self):
        self.get_logger().info('Waiting for vx01/walk action server…')
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action server not available!')
            return

        goal = Walk.Goal()
        goal.velocity_x     = self._args.vx
        goal.velocity_y     = self._args.vy
        goal.velocity_omega = self._args.omega
        goal.duration       = self._args.duration
        goal.step_length    = self._args.step_length
        goal.step_height    = self._args.step_height
        goal.step_period    = self._args.step_period

        self.get_logger().info(
            f'Goal → vx={goal.velocity_x:.3f} m/s  '
            f'vy={goal.velocity_y:.3f} m/s  '
            f'omega={goal.velocity_omega:.3f} rad/s  '
            f'duration={goal.duration:.1f} s'
        )

        future = self._client.send_goal_async(
            goal, feedback_callback=self._on_feedback)
        future.add_done_callback(self._on_goal_response)

    # ── Callbacks ─────────────────────────────────────────────────────────────
    def _on_goal_response(self, future):
        gh = future.result()
        if not gh.accepted:
            self.get_logger().error('Goal REJECTED')
            rclpy.shutdown()
            return
        self.get_logger().info('Goal ACCEPTED')
        gh.get_result_async().add_done_callback(self._on_result)

    def _on_feedback(self, msg):
        fb = msg.feedback
        j  = fb.joint_angles
        self.get_logger().info(
            f'  t={fb.elapsed_time:6.2f}s  '
            f'block={fb.gait_block}  '
            f'leg0=[{j[0]:.3f},{j[1]:.3f},{j[2]:.3f}]',
            throttle_duration_sec=0.5)

    def _on_result(self, future):
        res = future.result().result
        tag = 'SUCCESS' if res.success else 'FAILED'
        self.get_logger().info(
            f'Result: {tag}  elapsed={res.elapsed_time:.2f}s  '
            f'"{res.message}"')
        rclpy.shutdown()


def main():
    ap = argparse.ArgumentParser(description='Send Walk goal to VX-01')
    ap.add_argument('--vx',          type=float, default=0.05,
                    help='Forward velocity  (m/s, default 0.05)')
    ap.add_argument('--vy',          type=float, default=0.0,
                    help='Lateral velocity  (m/s, default 0.0)')
    ap.add_argument('--omega',       type=float, default=0.0,
                    help='Yaw rate         (rad/s, default 0.0)')
    ap.add_argument('--duration',    type=float, default=10.0,
                    help='Walk duration    (s, 0=indefinite, default 10)')
    ap.add_argument('--step-length', type=float, default=0.0,
                    dest='step_length',
                    help='Stride length    (mm, 0=use server default)')
    ap.add_argument('--step-height', type=float, default=0.0,
                    dest='step_height',
                    help='Foot lift height (mm, 0=use server default)')
    ap.add_argument('--step-period', type=float, default=0.0,
                    dest='step_period',
                    help='Gait cycle period(s,  0=use server default)')
    args = ap.parse_args()

    rclpy.init()
    node = WalkClient(args)
    node.send()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nInterrupted')
    sys.exit(0)


if __name__ == '__main__':
    main()
