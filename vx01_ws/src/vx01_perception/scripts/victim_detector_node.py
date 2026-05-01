#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import yaml
import os
import base64

from vx01_msgs.msg import Victim, VictimArray, MissionState
from ament_index_python.packages import get_package_share_directory


class VictimDetectorNode(Node):

    CONF_THRESHOLD = 0.5
    NMS_THRESHOLD  = 0.4
    INPUT_SIZE     = (416, 416)

    def __init__(self, config_path: str):
        super().__init__('victim_detector_node')

        topics = self._load_topics(config_path)
        self.bridge  = CvBridge()
        self.depth   = None
        self.active  = True
        self.victim_id_counter = 0

        # Camera intrinsics — taken from camera_info
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        self.net, self.output_layers, self.classes = self._load_yolo()

        self.create_subscription(Image,        topics['camera_color'],      self._cb_color,       10)
        self.create_subscription(Image,        topics['camera_depth'],      self._cb_depth,       10)
        self.create_subscription(CameraInfo,   topics['camera_depth_info'], self._cb_camera_info, 10)
        self.create_subscription(MissionState, topics['mission_mode'],      self._cb_mission,     10)

        self.pub = self.create_publisher(VictimArray, topics['victim_detections'], 10)

    def _load_topics(self, path: str) -> dict:
        with open(path) as f:
            return yaml.safe_load(f)['topics']

    def _load_yolo(self):
        pkg_dir  = os.path.dirname(os.path.abspath(__file__))
        weights  = os.path.join(pkg_dir, 'models', 'yolov4-tiny.weights')
        cfg      = os.path.join(pkg_dir, 'models', 'yolov4-tiny.cfg')
        names    = os.path.join(pkg_dir, 'models', 'coco.names')

        net = cv2.dnn.readNet(weights, cfg)
        net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

        layer_names   = net.getLayerNames()
        output_layers = [layer_names[i - 1] for i in net.getUnconnectedOutLayers()]

        with open(names) as f:
            classes = [line.strip() for line in f.readlines()]

        return net, output_layers, classes

    def _cb_camera_info(self, msg: CameraInfo):
        if self.fx is None:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.get_logger().info(
                f'Camera intrinsics received — fx:{self.fx:.1f} fy:{self.fy:.1f} '
                f'cx:{self.cx:.1f} cy:{self.cy:.1f}'
            )

    def _cb_mission(self, msg: MissionState):
        self.active = msg.mission_phase in ('SEARCHING', 'VICTIM_FOUND')

    def _cb_depth(self, msg: Image):
        self.depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def _cb_color(self, msg: Image):
        if not self.active:
            return
        if self.depth is None:
            self.get_logger().warn('No depth frame yet — skipping.', throttle_duration_sec=5.0)
            return
        if self.fx is None:
            self.get_logger().warn('No camera_info yet — skipping.', throttle_duration_sec=5.0)
            return

        frame   = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        detects = self._detect(frame)
        victims = [self._build_victim(d, frame) for d in detects]

        out         = VictimArray()
        out.header  = msg.header
        out.victims = victims
        self.pub.publish(out)

    def _detect(self, frame: np.ndarray) -> list:
        h, w = frame.shape[:2]
        blob = cv2.dnn.blobFromImage(frame, 1 / 255.0, self.INPUT_SIZE, swapRB=True, crop=False)
        self.net.setInput(blob)
        outs = self.net.forward(self.output_layers)

        boxes, confidences, class_ids = [], [], []
        for out in outs:
            for det in out:
                scores   = det[5:]
                class_id = int(np.argmax(scores))
                conf     = float(scores[class_id])
                if conf < self.CONF_THRESHOLD or self.classes[class_id] != 'person':
                    continue
                cx, cy, bw, bh = det[0] * w, det[1] * h, det[2] * w, det[3] * h
                x = int(cx - bw / 2)
                y = int(cy - bh / 2)
                boxes.append([x, y, int(bw), int(bh)])
                confidences.append(conf)
                class_ids.append(class_id)

        indices = cv2.dnn.NMSBoxes(boxes, confidences, self.CONF_THRESHOLD, self.NMS_THRESHOLD)
        if len(indices) == 0:
            return []
        return [(boxes[i], confidences[i], class_ids[i]) for i in indices.flatten()]

    def _build_victim(self, detection: tuple, frame: np.ndarray) -> Victim:
        (x, y, w, h), conf, class_id = detection

        u = x + w // 2
        v = y + h // 2

        v_c = int(min(max(v, 0), self.depth.shape[0] - 1))
        u_c = int(min(max(u, 0), self.depth.shape[1] - 1))

        raw     = float(self.depth[v_c, u_c])
        depth_m = raw / 1000.0  # mm → metres

        X = (u - self.cx) * depth_m / self.fx
        Y = (v - self.cy) * depth_m / self.fy
        Z = depth_m

        y1 = max(0, y)
        x1 = max(0, x)
        y2 = min(frame.shape[0], y + h)
        x2 = min(frame.shape[1], x + w)
        crop = frame[y1:y2, x1:x2]
        
        b64_str = ""
        if crop.size > 0:
            _, buffer = cv2.imencode('.jpg', crop)
            b64_str = base64.b64encode(buffer).decode('utf-8')

        self.victim_id_counter += 1

        victim                          = Victim()
        victim.id                       = self.victim_id_counter
        victim.detected                 = True
        victim.label                    = self.classes[class_id]
        victim.confidence               = conf
        victim.position.header.frame_id = 'camera_depth_optical_frame'
        victim.position.point.x         = X
        victim.position.point.y         = Y
        victim.position.point.z         = Z
        victim.image_base64             = b64_str
        return victim


def main(args=None):
    rclpy.init(args=args)
    share_dir = get_package_share_directory('vx01_perception')
    config = os.path.join(share_dir, 'config', 'topics.yaml')
    node   = VictimDetectorNode(config)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()