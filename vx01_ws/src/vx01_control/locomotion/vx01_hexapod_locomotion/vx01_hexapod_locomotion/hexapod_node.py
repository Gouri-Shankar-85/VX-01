import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration as RosDuration

from .gait import Leg, TripodGait


def _ros_duration(seconds: float) -> RosDuration:
    sec = int(seconds)
    ns = int((seconds - sec) * 1_000_000_000)
    return RosDuration(sec=sec, nanosec=ns)


class HexapodNode(Node):

    GROUP_A = TripodGait.GROUP_A
    GROUP_B = TripodGait.GROUP_B

    def __init__(self):
        super().__init__('hexapod_node')
        self._declare_params()
        self._load_params()
        self._init_legs()
        self._init_publishers()

        self._walking = False
        self._phase = 0
        self._vx = 0.0
        self._vy = 0.0
        self._omega = 0.0
        
        self._foot_offsets = [(0.0, 0.0) for _ in range(6)]

        self._cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel_cb, 10)

        half = self._step_period / 2.0
        self._gait_timer = self.create_timer(half, self._gait_tick)
        self._gait_timer.cancel()

        self._stand_timer = self.create_timer(2.0, self._initial_stand)

    def _declare_params(self):
        self.declare_parameter('L1', 60.55)
        self.declare_parameter('L2', 73.84)
        self.declare_parameter('L3', 112.16)
        self.declare_parameter('body_radius', 105.66)
        self.declare_parameter('home_reach', 227.689)
        self.declare_parameter('home_z', -61.379)
        self.declare_parameter('step_length', 60.0)
        self.declare_parameter('step_height', 30.0)
        self.declare_parameter('step_period', 2.0)
        self.declare_parameter('stand_duration', 3.0)
        self.declare_parameter('waypoints', 10)
        self.declare_parameter('coxa_angles',
            [-1.5708, -0.7854, 0.7854, 1.5708, 2.3920, -2.3562])
        self.declare_parameter('controller_names', [
            'leg_0_controller', 'leg_1_controller', 'leg_2_controller',
            'leg_3_controller', 'leg_4_controller', 'leg_5_controller'])

    def _load_params(self):
        self._L1 = self.get_parameter('L1').value
        self._L2 = self.get_parameter('L2').value
        self._L3 = self.get_parameter('L3').value
        self._body_radius = self.get_parameter('body_radius').value
        self._home_reach = self.get_parameter('home_reach').value
        self._home_z = self.get_parameter('home_z').value
        self._step_length = self.get_parameter('step_length').value
        self._step_height = self.get_parameter('step_height').value
        self._step_period = self.get_parameter('step_period').value
        self._stand_duration = self.get_parameter('stand_duration').value
        self._waypoints = self.get_parameter('waypoints').value
        self._coxa_angles = self.get_parameter('coxa_angles').value
        self._controller_names = self.get_parameter('controller_names').value

        self._joint_names = [
            [f'coxa_leg{i}_joint', f'femur_leg{i}_joint', f'tibia_leg{i}_joint']
            for i in range(6)
        ]

    def _init_legs(self):
        self._legs = [
            Leg(i, self._coxa_angles[i], self._body_radius,
                self._home_reach, self._home_z,
                self._L1, self._L2, self._L3)
            for i in range(6)
        ]

    def _init_publishers(self):
        self._pubs = [
            self.create_publisher(
                JointTrajectory,
                f'/{self._controller_names[i]}/joint_trajectory',
                10)
            for i in range(6)
        ]

    def _initial_stand(self):
        ready = all(pub.get_subscription_count() > 0 for pub in self._pubs)
        if ready:
            self.get_logger().info('Controllers active. Sending stand pose...')
            self._stand_timer.cancel()
            self._foot_offsets = [(0.0, 0.0) for _ in range(6)]
            self._send_stand()

    def _send_stand(self):

        for i in range(6):
            hx, hy, hz = self._legs[i].home
            angles = self._legs[i].solve_ik(hx, hy, hz)
            
            if angles is None:
                continue
            
            cmd1 = 0.0 - angles[0]
            cmd2 = 0.1274 - angles[1]
            cmd3 = -0.6580 - angles[2]

            traj = JointTrajectory()
            traj.joint_names = self._joint_names[i]
            pt = JointTrajectoryPoint()
            pt.positions = [cmd1, cmd2, cmd3]

            pt.time_from_start = _ros_duration(self._stand_duration)
            traj.points = [pt]
            self._publish(i, traj)

    def _cmd_vel_cb(self, msg: Twist):
        self._vx = msg.linear.x
        self._vy = msg.linear.y
        self._omega = msg.angular.z

        moving = (
            abs(self._vx) > 0.001 or
            abs(self._vy) > 0.001 or
            abs(self._omega) > 0.001
        )

        if moving and not self._walking:
            self._walking = True
            self._phase = 0
            self._execute_phase()
            self._gait_timer.reset()

        elif not moving and self._walking:
            self._walking = False
            self._gait_timer.cancel()
            self._foot_offsets = [(0.0, 0.0) for _ in range(6)]
            self._send_stand()

    def _gait_tick(self):
        if not self._walking:
            return
        self._phase = 1 - self._phase
        self._execute_phase()

    def _execute_phase(self):

        swing = self.GROUP_A if self._phase == 0 else self.GROUP_B
        stance = self.GROUP_B if self._phase == 0 else self.GROUP_A
        half_dur = self._step_period / 2.0

        for leg_id in swing:
            dx, dy = self._foot_stride(leg_id)
            self._publish(leg_id, self._build_swing(leg_id, dx, dy, half_dur))

        for leg_id in stance:
            dx, dy = self._foot_stride(leg_id)
            self._publish(leg_id, self._build_stance(leg_id, dx, dy, half_dur))

    def _foot_stride(self, leg_id: int):
    
        half_period = self._step_period / 2.0
        hx, hy, _ = self._legs[leg_id].home

        lx = self._vx * half_period * 500.0
        ly = self._vy * half_period * 500.0

        rx = -hy * self._omega * half_period * 0.5
        ry = hx * self._omega * half_period * 0.5

        dx = lx + rx
        dy = ly + ry

        limit = self._step_length / 2.0
        mag = math.hypot(dx, dy)
        if mag > limit:
            dx = dx * limit / mag
            dy = dy * limit / mag

        return dx, dy

    def _build_swing(self, leg_id: int, dx: float, dy: float, duration: float):
        leg = self._legs[leg_id]
        hx, hy, hz = leg.home
        N = self._waypoints

        traj = JointTrajectory()
        traj.joint_names = self._joint_names[leg_id]
        
        start_dx, start_dy = self._foot_offsets[leg_id]
        end_dx, end_dy = dx, dy

        for i in range(1, N + 1):
            t = i / N
            # Interpolate from current offset to target offset
            bx = hx + start_dx + (end_dx - start_dx) * t
            by = hy + start_dy + (end_dy - start_dy) * t
            bz = hz + self._step_height * math.sin(math.pi * t)

            angles = leg.solve_ik(bx, by, bz)
            if angles is None:
                angles = leg.solve_ik(hx, hy, hz)
            if angles is None:
                continue

            cmd1 = 0.0 - angles[0]
            cmd2 = 0.1274 - angles[1]
            cmd3 = -0.6580 - angles[2]

            pt = JointTrajectoryPoint()
            pt.positions = [cmd1, cmd2, cmd3]
            pt.time_from_start = _ros_duration(t * duration)
            traj.points.append(pt)

        self._foot_offsets[leg_id] = (end_dx, end_dy)
        return traj

    def _build_stance(self, leg_id: int, dx: float, dy: float, duration: float):
        leg = self._legs[leg_id]
        hx, hy, hz = leg.home
        N = self._waypoints

        traj = JointTrajectory()
        traj.joint_names = self._joint_names[leg_id]

        start_dx, start_dy = self._foot_offsets[leg_id]
        end_dx, end_dy = -dx, -dy

        for i in range(1, N + 1):
            t = i / N
            bx = hx + start_dx + (end_dx - start_dx) * t
            by = hy + start_dy + (end_dy - start_dy) * t

            angles = leg.solve_ik(bx, by, hz)
            if angles is None:
                angles = leg.solve_ik(hx, hy, hz)
            if angles is None:
                continue

            cmd1 = 0.0 - angles[0]
            cmd2 = 0.1274 - angles[1]
            cmd3 = -0.6580 - angles[2]

            pt = JointTrajectoryPoint()
            pt.positions = [cmd1, cmd2, cmd3]
            pt.time_from_start = _ros_duration(t * duration)
            traj.points.append(pt)

        self._foot_offsets[leg_id] = (end_dx, end_dy)
        return traj

    def _publish(self, leg_id: int, traj: JointTrajectory):
        if not traj.points:
            return
        traj.header.stamp = self.get_clock().now().to_msg()
        self._pubs[leg_id].publish(traj)


def main(args=None):
    rclpy.init(args=args)
    node = HexapodNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
