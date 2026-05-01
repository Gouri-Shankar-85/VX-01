#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from sensor_msgs.msg import BatteryState
import yaml
import os

from vx01_msgs.msg import VictimArray, MissionState
from ament_index_python.packages import get_package_share_directory


class MissionManagerNode(Node):

    LOW_BATTERY_THRESHOLD = 20.0  # percent

    def __init__(self, config_path: str):
        super().__init__('mission_manager_node')

        topics = self._load_topics(config_path)

        self.state              = MissionState()
        self.state.mission_phase = 'IDLE'
        self.state.victim_found  = False
        self.state.victims_detected = 0

        self.battery_ok = True

        self.create_subscription(Bool,        topics['system_ready'],      self._cb_ready,    10)
        self.create_subscription(String,      topics['operator_command'],  self._cb_command,  10)
        self.create_subscription(VictimArray, topics['victim_detections'], self._cb_victims,  10)
        self.create_subscription(BatteryState,topics['battery_state'],     self._cb_battery,  10)
        self.create_subscription(String,      topics['navigation_status'], self._cb_nav,      10)

        self.pub = self.create_publisher(MissionState, topics['mission_mode'], 10)
        self.create_timer(1.0, self._publish)

    def _load_topics(self, path: str) -> dict:
        with open(path) as f:
            return yaml.safe_load(f)['topics']

    def _publish(self):
        self.pub.publish(self.state)

    def _cb_ready(self, msg: Bool):
        if msg.data and self.state.mission_phase == 'IDLE':
            self.state.mission_phase = 'SEARCHING'

    def _cb_command(self, msg: String):
        cmd = msg.data.upper()
        if cmd == 'START':
            self.state.mission_phase = 'SEARCHING'
        elif cmd == 'ABORT':
            self.state.mission_phase = 'RETURNING'
        elif cmd == 'RESET':
            self.state.mission_phase     = 'IDLE'
            self.state.victim_found      = False
            self.state.victims_detected  = 0

    def _cb_victims(self, msg: VictimArray):
        if not msg.victims:
            return
        count = len(msg.victims)
        self.state.victims_detected += count
        self.state.victim_found      = True
        if self.state.mission_phase == 'SEARCHING':
            self.state.mission_phase = 'VICTIM_FOUND'

    def _cb_battery(self, msg: BatteryState):
        pct = msg.percentage * 100.0
        if pct < self.LOW_BATTERY_THRESHOLD and self.state.mission_phase not in ('RETURNING', 'COMPLETE'):
            self.get_logger().warn(f'Low battery {pct:.1f}% — returning to base.')
            self.state.mission_phase = 'RETURNING'

    def _cb_nav(self, msg: String):
        if msg.data == 'REACHED_BASE' and self.state.mission_phase == 'RETURNING':
            self.state.mission_phase = 'COMPLETE'


def main(args=None):
    rclpy.init(args=args)
    share_dir = get_package_share_directory('vx01_perception')
    config = os.path.join(share_dir, 'config', 'topics.yaml')
    node   = MissionManagerNode(config)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
