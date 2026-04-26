import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import TwistStamped, PoseStamped
from std_msgs.msg import Bool
from mavros_msgs.msg import State, EstimatorStatus
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL

from enum import Enum, auto


class FlightPhase(Enum):
    IDLE                  = auto()
    WAITING_FOR_CONNECTION = auto()
    STREAMING_SETPOINTS   = auto()
    SETTING_GUIDED        = auto()
    ARMING                = auto()
    TAKING_OFF            = auto()
    HOVERING              = auto()
    LANDING               = auto()


class DroneFlightManager(Node):

    def __init__(self):
        super().__init__('drone_flight_manager')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('takeoff_altitude',      2.0)
        self.declare_parameter('auto_takeoff',          True)
        self.declare_parameter('hover_duration',        15.0)   
        self.declare_parameter('setpoint_stream_count', 200)    

        self._takeoff_alt    = self.get_parameter('takeoff_altitude').value
        self._auto_takeoff   = self.get_parameter('auto_takeoff').value
        self._hover_duration = self.get_parameter('hover_duration').value
        self._stream_target  = self.get_parameter('setpoint_stream_count').value

        # ── State ────────────────────────────────────────────────────────────
        self._phase        = FlightPhase.WAITING_FOR_CONNECTION
        self._connected    = False
        self._armed        = False
        self._mode         = ''
        self._altitude     = 0.0
        self._stream_count = 0

        self._latest_cmd       = TwistStamped()
        self._last_cmd_time    = self.get_clock().now()
        self._hover_start_time = None
        self._has_external_cmd = False

        # Retry timestamps
        self._last_mode_req_time = None
        self._last_arm_req_time  = None

        # ── Subscribers ──────────────────────────────────────────────────────
        self.create_subscription(
            State, '/mavros/state', self._state_cb, 10)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10)

        self.create_subscription(
            PoseStamped, '/mavros/local_position/pose',
            self._pose_cb, sensor_qos)

        self.create_subscription(
            TwistStamped, '/drone/cmd_vel', self._cmd_vel_cb, 10)
        self.create_subscription(
            Bool, '/drone/land', self._land_cb, 10)

        # ── Publisher ────────────────────────────────────────────────────────
        self._vel_pub = self.create_publisher(
            TwistStamped, '/mavros/setpoint_velocity/cmd_vel', 10)

        # ── Service clients ──────────────────────────────────────────────────
        self._arm_client     = self.create_client(CommandBool, '/mavros/cmd/arming')
        self._mode_client    = self.create_client(SetMode,     '/mavros/set_mode')
        self._takeoff_client = self.create_client(CommandTOL,  '/mavros/cmd/takeoff')

        # ── Main loop timer (20 Hz) ──────────────────────────────────────────
        self.create_timer(0.05, self._tick)

        self.get_logger().info(
            f'DroneFlightManager initialized. '
            f'auto_takeoff={self._auto_takeoff}, '
            f'altitude={self._takeoff_alt}m, '
            f'stream_count={self._stream_target}')

    # ── Subscriber callbacks ─────────────────────────────────────────────────

    def _state_cb(self, msg: State):
        self._connected = msg.connected
        self._armed     = msg.armed
        self._mode      = msg.mode

    def _pose_cb(self, msg: PoseStamped):
        self._altitude = msg.pose.position.z

    def _cmd_vel_cb(self, msg: TwistStamped):
        self._latest_cmd    = msg
        self._last_cmd_time = self.get_clock().now()
        self._has_external_cmd = True

    def _land_cb(self, msg: Bool):
        if msg.data and self._phase == FlightPhase.HOVERING:
            self.get_logger().info('Land command received — switching to LAND mode')
            self._phase = FlightPhase.LANDING
            self._call_set_mode('LAND')

    # ── Main state machine ───────────────────────────────────────────────────

    def _tick(self):

        self._publish_setpoint()

        if self._phase == FlightPhase.WAITING_FOR_CONNECTION:
            if self._connected:
                self.get_logger().info('MAVROS connected!')
                if self._auto_takeoff:
                    self._phase        = FlightPhase.STREAMING_SETPOINTS
                    self._stream_count = 0
                    self.get_logger().info(
                        f'Streaming {self._stream_target} setpoints '
                        f'({self._stream_target / 20.0:.0f}s) before EKF check...')
                else:
                    self._phase = FlightPhase.IDLE
                    self.get_logger().info(
                        'auto_takeoff=False — waiting for manual commands')

        elif self._phase == FlightPhase.STREAMING_SETPOINTS:
            self._stream_count += 1
            if self._stream_count >= self._stream_target:
                self.get_logger().info('Setpoint stream complete — setting GUIDED mode')
                self._phase = FlightPhase.SETTING_GUIDED
                self._call_set_mode('GUIDED')

        elif self._phase == FlightPhase.SETTING_GUIDED:
            if self._mode == 'GUIDED':
                self.get_logger().info('GUIDED mode confirmed — arming')
                self._phase = FlightPhase.ARMING
                self._call_arm(True)
            else:
                now = self.get_clock().now()
                if (self._last_mode_req_time is None or
                        (now - self._last_mode_req_time).nanoseconds / 1e9 > 2.0):
                    self._call_set_mode('GUIDED')
                    self._last_mode_req_time = now

        elif self._phase == FlightPhase.ARMING:
            if self._armed:
                self.get_logger().info(
                    f'Armed! Commanding takeoff to {self._takeoff_alt}m')
                self._phase = FlightPhase.TAKING_OFF
                self._call_takeoff(self._takeoff_alt)
            else:
                now = self.get_clock().now()
                if (self._last_arm_req_time is None or
                        (now - self._last_arm_req_time).nanoseconds / 1e9 > 2.0):
                    self._call_arm(True)
                    self._last_arm_req_time = now

        elif self._phase == FlightPhase.TAKING_OFF:
            self.get_logger().info(
                f'Climbing... altitude={self._altitude:.2f}m / {self._takeoff_alt}m',
                throttle_duration_sec=2.0)
            if self._altitude >= self._takeoff_alt * 0.90:
                self.get_logger().info(
                    f'Reached {self._altitude:.1f}m — hovering for {self._hover_duration}s.')
                self._phase            = FlightPhase.HOVERING
                self._hover_start_time = self.get_clock().now()

        elif self._phase == FlightPhase.HOVERING:
            if self._hover_start_time is not None:
                elapsed = (self.get_clock().now() - self._hover_start_time).nanoseconds / 1e9
                self.get_logger().info(
                    f'Hovering... {elapsed:.0f}/{self._hover_duration:.0f}s '
                    f'alt={self._altitude:.2f}m',
                    throttle_duration_sec=3.0)
                if elapsed >= self._hover_duration:
                    self.get_logger().info(
                        f'Hover duration ({self._hover_duration}s) expired — landing.')
                    self._phase = FlightPhase.LANDING
                    self._call_set_mode('LAND')

        elif self._phase == FlightPhase.LANDING:
            if not self._armed:
                self.get_logger().info('Landed and disarmed.')
                self._phase = FlightPhase.IDLE

    # ── Setpoint publisher ───────────────────────────────────────────────────

    def _publish_setpoint(self):
        cmd = TwistStamped()
        cmd.header.stamp    = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'

        if self._phase == FlightPhase.HOVERING and self._has_external_cmd:
            age = (self.get_clock().now() - self._last_cmd_time).nanoseconds / 1e9
            if age < 0.5:
                cmd.twist = self._latest_cmd.twist

        self._vel_pub.publish(cmd)

    # ── Service helpers ──────────────────────────────────────────────────────

    def _call_set_mode(self, mode: str):
        if not self._mode_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('SetMode service unavailable')
            return
        req = SetMode.Request()
        req.custom_mode = mode
        future = self._mode_client.call_async(req)
        future.add_done_callback(
            lambda f: self.get_logger().info(
                f'SetMode → {mode}: '
                f'{"OK" if f.result().mode_sent else "FAIL"}'))

    def _call_arm(self, arm: bool):
        if not self._arm_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Arming service unavailable')
            return
        req = CommandBool.Request()
        req.value = arm
        future = self._arm_client.call_async(req)
        future.add_done_callback(
            lambda f: self.get_logger().info(
                f'{"ARM" if arm else "DISARM"}: '
                f'{"OK" if f.result().success else "FAIL"}'))

    def _call_takeoff(self, altitude: float):
        if not self._takeoff_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Takeoff service unavailable')
            return
        req = CommandTOL.Request()
        req.altitude  = altitude
        req.latitude  = 0.0
        req.longitude = 0.0
        req.min_pitch = 0.0
        req.yaw       = 0.0
        future = self._takeoff_client.call_async(req)
        future.add_done_callback(
            lambda f: self.get_logger().info(
                f'Takeoff to {altitude}m: '
                f'{"OK" if f.result().success else "FAIL"}'))


def main(args=None):
    rclpy.init(args=args)
    node = DroneFlightManager()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()