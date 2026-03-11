#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import yaml
import os

from vx01_msgs.msg import Terrain, Walkability, MissionState, RobotMode


class ModeManagerNode(Node):

    WALKABILITY_THRESHOLD = 0.5

    def __init__(self, config_path: str):
        super().__init__('mode_manager_node')

        topics = self._load_topics(config_path)

        self.terrain      = None
        self.walkability  = None
        self.mission      = None

        self.create_subscription(Terrain,     topics['terrain_type'],      self._cb_terrain,     10)
        self.create_subscription(Walkability, topics['walkability_score'], self._cb_walkability, 10)
        self.create_subscription(MissionState,topics['mission_mode'],      self._cb_mission,     10)

        self.pub = self.create_publisher(RobotMode, topics['robot_mode'], 10)
        self.create_timer(0.5, self._decide)

    def _load_topics(self, path: str) -> dict:
        with open(path) as f:
            return yaml.safe_load(f)['topics']

    def _cb_terrain(self, msg: Terrain):
        self.terrain = msg

    def _cb_walkability(self, msg: Walkability):
        self.walkability = msg

    def _cb_mission(self, msg: MissionState):
        self.mission = msg

    def _decide(self):
        if self.walkability is None or self.mission is None:
            return

        if self.mission.mission_phase == 'IDLE':
            mode = 'HEXAPOD'
        elif not self.walkability.walkable:
            mode = 'DRONE'
        else:
            mode = 'HEXAPOD'

        msg      = RobotMode()
        msg.mode = mode
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    config = os.path.join(os.path.dirname(__file__), '..', '..', 'config', 'topics.yaml')
    node   = ModeManagerNode(config)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
