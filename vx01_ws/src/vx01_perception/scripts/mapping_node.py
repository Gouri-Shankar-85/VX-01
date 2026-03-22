#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu, PointCloud2
from nav_msgs.msg import OccupancyGrid, Odometry
import yaml
import os


class MappingNode(Node):

    def __init__(self, config_path: str):
        super().__init__('mapping_node')

        topics = self._load_topics(config_path)

        self.create_subscription(Image,       topics['camera_color'],  self._cb_color, 10)
        self.create_subscription(Image,       topics['camera_depth'],  self._cb_depth, 10)
        self.create_subscription(PointCloud2, topics['camera_points'], self._cb_cloud, 10)
        self.create_subscription(Imu,         topics['imu'],           self._cb_imu,   10)

        self.pub_map   = self.create_publisher(OccupancyGrid, topics['map'],             10)
        self.pub_odom  = self.create_publisher(Odometry,      topics['odom'],            10)
        self.pub_cloud = self.create_publisher(PointCloud2,   topics['point_cloud_map'], 10)

    def _load_topics(self, path: str) -> dict:
        with open(path) as f:
            return yaml.safe_load(f)['topics']

    def _cb_color(self, msg: Image):
        pass

    def _cb_depth(self, msg: Image):
        pass

    def _cb_cloud(self, msg: PointCloud2):
        self.pub_cloud.publish(msg)

    def _cb_imu(self, msg: Imu):
        pass


def main(args=None):
    rclpy.init(args=args)
    config = os.path.expanduser('~/vx-01/vx01_ws/src/vx01_perception/config/topics.yaml')
    node   = MappingNode(config)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()