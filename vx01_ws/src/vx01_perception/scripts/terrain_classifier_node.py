#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
from cv_bridge import CvBridge
import yaml
import os

from vx01_msgs.msg import Terrain, Walkability, MissionState


class TerrainClassifierNode(Node):

    SLOPE_THRESH_STEEP = 30.0
    GAP_DEPTH_DIFF     = 0.5   # metres
    GAP_AREA_RATIO     = 0.05

    def __init__(self, config_path: str):
        super().__init__('terrain_classifier_node')

        topics = self._load_topics(config_path)
        self.bridge = CvBridge()
        self.active = True

        self.create_subscription(Image,        topics['camera_depth'], self._cb_depth,   10)
        self.create_subscription(MissionState, topics['mission_mode'], self._cb_mission, 10)

        self.pub_terrain = self.create_publisher(Terrain,     topics['terrain_type'],      10)
        self.pub_walk    = self.create_publisher(Walkability, topics['walkability_score'],  10)

    def _load_topics(self, path: str) -> dict:
        with open(path) as f:
            return yaml.safe_load(f)['topics']

    def _cb_mission(self, msg: MissionState):
        self.active = msg.mission_phase != 'IDLE'

    def _cb_depth(self, msg: Image):
        if not self.active:
            return

        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough').astype(np.float32)
        depth = depth / 1000.0  # mm to metres
        depth[depth == 0] = np.nan

        terrain = self._classify(depth)
        walk    = self._walkability(terrain, depth)

        self.pub_terrain.publish(terrain)
        self.pub_walk.publish(walk)

    def _classify(self, depth: np.ndarray) -> Terrain:
        slope    = self._estimate_slope(depth)
        gap_flag = self._detect_gap(depth)

        t              = Terrain()
        t.slope        = float(slope)
        t.gap_detected = bool(gap_flag)   # explicit cast — numpy.bool_ is not accepted

        if gap_flag:
            t.terrain_type = 'GAP'
        elif slope > self.SLOPE_THRESH_STEEP:
            t.terrain_type = 'STAIRS'
        elif slope > 15.0:
            t.terrain_type = 'DEBRIS'
        elif slope > 5.0:
            t.terrain_type = 'OBSTACLE'
        else:
            t.terrain_type = 'WALKABLE'

        return t

    def _estimate_slope(self, depth: np.ndarray) -> float:
        valid = depth[~np.isnan(depth)]
        if valid.size < 100:
            return 0.0
        h, w   = depth.shape
        mid_y  = h // 2
        strip  = depth[mid_y - 10: mid_y + 10, :]
        row    = np.nanmean(strip, axis=0)
        row    = row[~np.isnan(row)]
        if row.size < 2:
            return 0.0
        grad      = np.abs(np.gradient(row))
        valid_grad = grad[~np.isnan(grad)]
        if valid_grad.size == 0:
            return 0.0
        slope_rad = np.arctan(np.mean(valid_grad))
        return float(np.degrees(slope_rad))

    def _detect_gap(self, depth: np.ndarray) -> bool:
        h, w = depth.shape
        roi  = depth[h // 2:, :]
        if np.all(np.isnan(roi)):
            return False
        mean_depth = np.nanmean(roi)
        far_pixels = np.sum(roi > mean_depth + self.GAP_DEPTH_DIFF)
        ratio      = far_pixels / roi.size
        return bool(ratio > self.GAP_AREA_RATIO)   # explicit cast

    def _walkability(self, terrain: Terrain, depth: np.ndarray) -> Walkability:
        scores = {
            'WALKABLE': 1.0,
            'OBSTACLE': 0.4,
            'DEBRIS':   0.3,
            'STAIRS':   0.2,
            'GAP':      0.0,
        }
        score         = scores.get(terrain.terrain_type, 0.0)
        slope_penalty = min(terrain.slope / 90.0, 1.0) * 0.3
        score         = max(0.0, score - slope_penalty)

        w          = Walkability()
        w.score    = float(score)
        w.walkable = bool(score >= 0.5)   # explicit cast
        return w


def main(args=None):
    rclpy.init(args=args)
    config = os.path.expanduser('~/vx-01/vx01_ws/src/vx01_perception/config/topics.yaml')
    node   = TerrainClassifierNode(config)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()