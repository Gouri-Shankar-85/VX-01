#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import BatteryState, NavSatFix, Imu
from std_msgs.msg import Float64, UInt32
from geometry_msgs.msg import TwistStamped

class QosRelay(Node):
    def __init__(self):
        super().__init__('qos_relay')
        
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Define topics to relay (type, internal_topic, public_topic)
        self.topics = [
            (BatteryState, '/mavros_internal/battery', '/mavros/battery'),
            (NavSatFix, '/mavros_internal/global_position/global', '/mavros/global_position/global'),
            (Float64, '/mavros_internal/global_position/rel_alt', '/mavros/global_position/rel_alt'),
            (Float64, '/mavros_internal/global_position/compass_hdg', '/mavros/global_position/compass_hdg'),
            (UInt32, '/mavros_internal/global_position/raw/satellites', '/mavros/global_position/raw/satellites'),
            (Imu, '/mavros_internal/imu/data', '/mavros/imu/data'),
            (TwistStamped, '/mavros_internal/local_position/velocity_local', '/mavros/local_position/velocity_local')
        ]
        
        self.subs = []
        self.pubs = []
        
        for msg_type, sub_topic, pub_topic in self.topics:
            pub = self.create_publisher(msg_type, pub_topic, reliable_qos)
            self.pubs.append(pub)
            
            # Use default argument capture for the callback
            sub = self.create_subscription(
                msg_type,
                sub_topic,
                lambda msg, p=pub: p.publish(msg),
                best_effort_qos
            )
            self.subs.append(sub)
            
        self.get_logger().info('QoS Relay initialized: Best Effort -> Reliable')

def main(args=None):
    rclpy.init(args=args)
    node = QosRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
