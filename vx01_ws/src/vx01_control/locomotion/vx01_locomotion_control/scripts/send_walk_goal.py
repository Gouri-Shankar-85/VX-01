#!/usr/bin/env python3
"""
send_walk_goal.py – CLI test client for the vx01/walk action server.

The robot walks until you kill the NODE (Ctrl-C the launch file),
OR until the --duration timer expires, OR until you Ctrl-C this script.

Usage examples:
    # Walk forward indefinitely (until you kill the node)
    python3 send_walk_goal.py --vx 0.05

    # Walk forward for 10 seconds then stop
    python3 send_walk_goal.py --vx 0.05 --duration 10

    # Turn in place at 0.3 rad/s indefinitely
    python3 send_walk_goal.py --omega 0.3

    # Walk forward + strafe left, override gait params
    python3 send_walk_goal.py --vx 0.05 --vy 0.02 \\
        --step-length 90 --step-height 25 --step-period 1.8
"""

import argparse
import sys
import signal
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from vx01_locomotion_control.action import Walk


class WalkClient(Node):

    def __init__(self, args: argparse.Namespace):
        super().__init__('walk_test_client')
        self._args        = args
        self._client      = ActionClient(self, Walk, 'vx01/walk')
        self._goal_handle = None

    def send(self):
        self.get_logger().info('Waiting for vx01/walk action server…')
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action server not available!')
            rclpy.shutdown()
            return

        goal = Walk.Goal()
        goal.velocity_x     = self._args.vx
        goal.velocity_y     = self._args.vy
        goal.velocity_omega = self._args.omega
        goal.duration       = self._args.duration   # 0 = indefinite
        goal.step_length    = self._args.step_length
        goal.step_height    = self._args.step_height
        goal.step_period    = self._args.step_period

        dur_str = f'{goal.duration:.1f} s' if goal.duration > 0 else 'indefinite'
        self.get_logger().info(
            f'Sending goal → '
            f'vx={goal.velocity_x:.3f} m/s  '
            f'vy={goal.velocity_y:.3f} m/s  '
            f'omega={goal.velocity_omega:.3f} rad/s  '
            f'duration={dur_str}'
        )

        future = self._client.send_goal_async(
            goal, feedback_callback=self._on_feedback)
        future.add_done_callback(self._on_goal_response)

    def cancel(self):
        """Called on Ctrl-C to cancel the active goal cleanly."""
        if self._goal_handle is not None:
            self.get_logger().info('Cancelling walk goal…')
            self._goal_handle.cancel_goal_async()

    # ── Callbacks ──────────────────────────────────────────────────────────
    def _on_goal_response(self, future):
        gh = future.result()
        if not gh.accepted:
            self.get_logger().error('Goal REJECTED by server')
            rclpy.shutdown()
            return
        self._goal_handle = gh
        self.get_logger().info('Goal ACCEPTED – robot is walking')
        gh.get_result_async().add_done_callback(self._on_result)

    def _on_feedback(self, msg):
        fb = msg.feedback
        j  = fb.joint_angles
        # Only log coxa/femur/tibia for leg 0 (first 3 values)
        # These are the RAW DH angles from the library.
        # After publish_leg_trajectory conversion:
        #   URDF coxa  = -j[0]
        #   URDF femur =  j[1] + femur_urdf_offset
        #   URDF tibia = -j[2]
        self.get_logger().info(
            f'  t={fb.elapsed_time:6.2f}s  block={fb.gait_block}  '
            f'leg0_DH=[{j[0]:.3f},{j[1]:.3f},{j[2]:.3f}]  '
            f'leg0_URDF=[{-j[0]:.3f},{j[1]-0.7273:.3f},{-j[2]:.3f}]',
            throttle_duration_sec=0.5)

    def _on_result(self, future):
        res = future.result().result
        tag = 'SUCCESS' if res.success else 'CANCELLED'
        self.get_logger().info(
            f'Result: {tag}  elapsed={res.elapsed_time:.2f}s  "{res.message}"')
        rclpy.shutdown()


def main():
    ap = argparse.ArgumentParser(description='Send Walk goal to VX-01')
    ap.add_argument('--vx',          type=float, default=0.05,
                    help='Forward velocity  (m/s, default 0.05)')
    ap.add_argument('--vy',          type=float, default=0.0,
                    help='Lateral velocity  (m/s, default 0.0)')
    ap.add_argument('--omega',       type=float, default=0.0,
                    help='Yaw rate         (rad/s, default 0.0)')
    ap.add_argument('--duration',    type=float, default=0.0,
                    help='Walk duration (s). 0=indefinite until node killed. default=0')
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

    # Install Ctrl-C handler to cancel the goal gracefully
    def _sigint_handler(sig, frame):
        print('\nCtrl-C caught – cancelling goal…')
        node.cancel()

    signal.signal(signal.SIGINT, _sigint_handler)

    try:
        rclpy.spin(node)
    except Exception:
        pass

    sys.exit(0)


if __name__ == '__main__':
    main()