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


def _bezier3(P0, P1, P2, P3, t):
    """Cubic Bézier interpolation between four 3-D control points."""
    mt = 1.0 - t
    return (
        mt**3 * P0[0] + 3*mt**2*t * P1[0] + 3*mt*t**2 * P2[0] + t**3 * P3[0],
        mt**3 * P0[1] + 3*mt**2*t * P1[1] + 3*mt*t**2 * P2[1] + t**3 * P3[1],
        mt**3 * P0[2] + 3*mt**2*t * P1[2] + 3*mt*t**2 * P2[2] + t**3 * P3[2],
    )


class HexapodNode(Node):

    GROUP_A = TripodGait.GROUP_A  # (0, 2, 4)
    GROUP_B = TripodGait.GROUP_B  # (1, 3, 5)

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

        # Foot offset from home in world XY (mm), updated each half-cycle
        self._foot_offsets = [(0.0, 0.0) for _ in range(6)]

        self._cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel_cb, 10)

        half = self._step_period / 2.0
        self._gait_timer = self.create_timer(half, self._gait_tick)
        self._gait_timer.cancel()

        self.get_logger().info('Hexapod node initialized. Waiting for manual commands on /cmd_vel')

    #  params

    def _declare_params(self):
        self.declare_parameter('L1', 60.55)
        self.declare_parameter('L2', 73.84)
        self.declare_parameter('L3', 112.16)
        self.declare_parameter('body_radius', 105.66)
        self.declare_parameter('home_reach', 120.0)
        self.declare_parameter('home_z', -150.0)
        self.declare_parameter('step_length', 80.0)
        self.declare_parameter('step_height', 60.0)
        self.declare_parameter('step_period', 1.2)
        self.declare_parameter('stand_duration', 2.0)
        self.declare_parameter('waypoints', 15)
        self.declare_parameter('coxa_angles',
            [-1.5708, -0.7854, 0.7854, 1.5708, 2.3920, -2.3562])
        self.declare_parameter('stand_femur_cmd', -0.9)
        self.declare_parameter('stand_tibia_cmd', 0.7)
        self.declare_parameter('controller_names', [
            'leg_0_controller', 'leg_1_controller', 'leg_2_controller',
            'leg_3_controller', 'leg_4_controller', 'leg_5_controller'])

    def _load_params(self):
        self._L1             = self.get_parameter('L1').value
        self._L2             = self.get_parameter('L2').value
        self._L3             = self.get_parameter('L3').value
        self._body_radius    = self.get_parameter('body_radius').value
        self._home_reach     = self.get_parameter('home_reach').value
        self._home_z         = self.get_parameter('home_z').value
        self._step_length    = self.get_parameter('step_length').value
        self._step_height    = self.get_parameter('step_height').value
        self._step_period    = self.get_parameter('step_period').value
        self._stand_duration = self.get_parameter('stand_duration').value
        self._waypoints      = self.get_parameter('waypoints').value
        self._coxa_angles    = self.get_parameter('coxa_angles').value
        self._controller_names = self.get_parameter('controller_names').value

        self._stand_femur_cmd = self.get_parameter('stand_femur_cmd').value
        self._stand_tibia_cmd = self.get_parameter('stand_tibia_cmd').value

        self._joint_names = [
            [f'coxa_leg{i}_joint', f'femur_leg{i}_joint', f'tibia_leg{i}_joint']
            for i in range(6)
        ]

        # Calculate home_reach and home_z dynamically from the desired stand pose commands
        angles = self._cmd_to_angles([0.0, self._stand_femur_cmd, self._stand_tibia_cmd])
        hx, hy, hz = self._fk(angles)
        self._home_reach = math.hypot(hx, hy)
        self._home_z = hz
        self.get_logger().info(f'FK from custom stand pose -> reach: {self._home_reach:.1f}, z: {hz:.1f}')

    #  init

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

    #  stand

    def _initial_stand(self):
        ready = all(pub.get_subscription_count() > 0 for pub in self._pubs)
        if ready:
            self.get_logger().info('All controllers active – sending stand pose.')
            self._stand_timer.cancel()
            self._foot_offsets = [(0.0, 0.0) for _ in range(6)]
            self._send_stand()

    def _send_stand(self):
        cmd1 = 0.0
        cmd2 = self._stand_femur_cmd
        cmd3 = self._stand_tibia_cmd

        for i in range(6):
            pt = JointTrajectoryPoint()
            pt.positions = [cmd1, cmd2, cmd3]
            pt.time_from_start = _ros_duration(self._stand_duration)

            traj = JointTrajectory()
            traj.joint_names = self._joint_names[i]
            traj.points = [pt]
            self._publish(i, traj)

    #  cmd_vel

    def _cmd_vel_cb(self, msg: Twist):
        self._vx    = msg.linear.x
        self._vy    = msg.linear.y
        self._omega = msg.angular.z

        moving = (
            abs(self._vx)    > 0.001 or
            abs(self._vy)    > 0.001 or
            abs(self._omega) > 0.001
        )

        if moving and not self._walking:
            self._walking = True
            self._phase   = 0
            self._foot_offsets = [(0.0, 0.0) for _ in range(6)]
            self._execute_phase()
            self._gait_timer.reset()

        elif not moving and self._walking:
            self._walking = False
            self._gait_timer.cancel()
            # Do NOT call _send_stand() - just stop walking and hold current pose

    #  gait

    def _gait_tick(self):
        if not self._walking:
            return
        self._phase = 1 - self._phase
        self._execute_phase()

    def _execute_phase(self):
        swing   = self.GROUP_A if self._phase == 0 else self.GROUP_B
        stance  = self.GROUP_B if self._phase == 0 else self.GROUP_A
        half_dur = self._step_period / 2.0

        for leg_id in swing:
            dx, dy = self._foot_stride(leg_id)
            self._publish(leg_id, self._build_swing(leg_id, dx, dy, half_dur))

        for leg_id in stance:
            dx, dy = self._foot_stride(leg_id)
            self._publish(leg_id, self._build_stance(leg_id, dx, dy, half_dur))

    #  stride

    def _foot_stride(self, leg_id: int):
        """
        Compute the half-stride displacement (dx, dy) in world XY for one leg.

        Convention:
          • In stance the foot sweeps from +dx to −dx  (total 2·dx in half_period)
          • In swing  the foot sweeps from −dx to +dx  (arc through the air)
          • Body translates at vx = (2·dx) / step_period  (consistent with ROS Twist m/s)
        """
        half_period = self._step_period / 2.0
        hx, hy, _   = self._legs[leg_id].home

        # Linear contribution  (vx/vy in m/s → mm displacement)
        lx = self._vx * half_period * 1000.0 * 0.5   # = vx[mm/s] * half_period / 2
        ly = self._vy * half_period * 1000.0 * 0.5

        # Rotational contribution (omega in rad/s, foot at (hx, hy) mm)
        # arc ≈ radius × angle, tangential direction = (-hy, hx) / |r|
        angle = self._omega * half_period * 0.5       # swept angle for half-stride
        rx    = -hy * angle
        ry    =  hx * angle

        dx = lx + rx
        dy = ly + ry

        # Clamp to half of step_length so feet stay reachable
        limit = self._step_length / 2.0
        mag   = math.hypot(dx, dy)
        if mag > limit:
            scale = limit / mag
            dx *= scale
            dy *= scale

        return dx, dy

    #  swing (Bézier)

    def _build_swing(self, leg_id: int, dx: float, dy: float, duration: float):
        """
        Cubic Bézier swing trajectory:
          P0 → start foot on ground
          P1 → P0 lifted straight up by step_height  (fast liftoff)
          P2 → P3 lifted straight up by step_height  (controlled landing)
          P3 → end foot on ground
        """
        leg = self._legs[leg_id]
        hx, hy, hz = leg.home
        N  = self._waypoints

        start_dx, start_dy = self._foot_offsets[leg_id]
        end_dx,   end_dy   = dx, dy

        sx, sy = hx + start_dx, hy + start_dy
        ex, ey = hx + end_dx,   hy + end_dy
        h      = self._step_height

        # Cubic Bézier control points
        P0 = (sx, sy, hz)
        P1 = (sx, sy, hz + h)           # lift straight from start
        P2 = (ex, ey, hz + h)           # hover over end
        P3 = (ex, ey, hz)               # touch down at end

        traj = JointTrajectory()
        traj.joint_names = self._joint_names[leg_id]

        for i in range(1, N + 1):
            t  = i / N
            bx, by, bz = _bezier3(P0, P1, P2, P3, t)

            angles = leg.solve_ik(bx, by, bz)
            if angles is None:
                angles = leg.solve_ik(hx, hy, hz)   # fall back to home
            if angles is None:
                continue

            pt = JointTrajectoryPoint()
            pt.positions      = self._angles_to_cmd(angles)
            pt.time_from_start = _ros_duration(t * duration)
            traj.points.append(pt)

        self._foot_offsets[leg_id] = (end_dx, end_dy)
        return traj

    #  stance (linear)

    def _build_stance(self, leg_id: int, dx: float, dy: float, duration: float):
        """
        Linear stance sweep: foot moves from current offset to −(dx, dy).
        The ground reaction propels the body forward.
        """
        leg = self._legs[leg_id]
        hx, hy, hz = leg.home
        N  = self._waypoints

        start_dx, start_dy = self._foot_offsets[leg_id]
        end_dx,   end_dy   = -dx, -dy

        traj = JointTrajectory()
        traj.joint_names = self._joint_names[leg_id]

        for i in range(1, N + 1):
            t  = i / N
            bx = hx + start_dx + (end_dx - start_dx) * t
            by = hy + start_dy + (end_dy - start_dy) * t

            angles = leg.solve_ik(bx, by, hz)
            if angles is None:
                angles = leg.solve_ik(hx, hy, hz)
            if angles is None:
                continue

            pt = JointTrajectoryPoint()
            pt.positions      = self._angles_to_cmd(angles)
            pt.time_from_start = _ros_duration(t * duration)
            traj.points.append(pt)

        self._foot_offsets[leg_id] = (end_dx, end_dy)
        return traj

    #  helpers

    def _cmd_to_angles(self, cmds):
        theta1 = 0.0 - cmds[0]
        theta2 = 0.1274 - cmds[1]
        theta3 = -0.658 - cmds[2]
        return [theta1, theta2, theta3]

    def _fk(self, angles):
        theta1, theta2, theta3 = angles
        r_plane = self._L1 + self._L2 * math.cos(theta2) + self._L3 * math.cos(theta2 + theta3)
        z = self._L2 * math.sin(theta2) + self._L3 * math.sin(theta2 + theta3)
        x = r_plane * math.cos(theta1)
        y = r_plane * math.sin(theta1)
        return x, y, z

    def _angles_to_cmd(self, angles):
        """
        Convert geometric IK angles (θ1, θ2, θ3) to URDF joint commands.

        URDF convention for all joints:   axis xyz="0 0 -1"
          physical_angle = urdf_offset  +  (-1) × cmd
          =>  cmd = urdf_offset  −  physical_angle

        Offsets extracted from joint origin rpy:
          coxa  (rpy "… …  0.0   ") → z-offset = 0.0
          femur (rpy "… 0.12741 0") → z-offset = 0.12741  (pitch in rolled frame)
          tibia (rpy "0 0 -0.65799") → z-offset = −0.65799
        """
        cmd1 = 0.0    - angles[0]   # coxa
        cmd2 = 0.1274 - angles[1]   # femur
        cmd3 = -0.658 - angles[2]   # tibia
        return [cmd1, cmd2, cmd3]

    def _single_point_traj(self, leg_id: int, angles, duration: float):
        traj = JointTrajectory()
        traj.joint_names = self._joint_names[leg_id]
        pt  = JointTrajectoryPoint()
        pt.positions      = self._angles_to_cmd(angles)
        pt.time_from_start = _ros_duration(duration)
        traj.points = [pt]
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