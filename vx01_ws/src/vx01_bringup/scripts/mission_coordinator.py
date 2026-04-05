#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist
from mavros_msgs.srv import SetMode
from std_msgs.msg import String
from vx01_msgs.msg import VictimArray, Terrain, RobotMode, MissionState
import time

class MissionCoordinator(Node):
    def __init__(self):
        super().__init__('mission_coordinator')
        self.get_logger().info("VX-01 Mission Coordinator Booting...")
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.mission_state_pub = self.create_publisher(MissionState, '/mission_state', 10)
        self.robot_mode_pub = self.create_publisher(RobotMode, '/robot_mode', 10)
        
        # Subscribers
        self.create_subscription(VictimArray, '/victim_detections', self.victim_cb, 10)
        self.create_subscription(Terrain, '/terrain_type', self.terrain_cb, 10)
        
        # Service Clients
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        
        # State Variables
        self.state = "INIT"
        self.terrain = "NORMAL"
        self.victims = []
        self.mission_msg = MissionState()
        
        self.timer = self.create_timer(1.0, self.mission_loop)

    def victim_cb(self, msg):
        self.victims = msg.victims

    def terrain_cb(self, msg):
        self.terrain = msg.terrain_type

    def update_phase(self, phase):
        self.state = phase
        self.mission_msg.mission_phase = phase
        self.mission_state_pub.publish(self.mission_msg)
        self.get_logger().info(f"MISSION PHASE: {phase}")

    def mission_loop(self):
        if self.state == "INIT":
            self.update_phase("HEXAPOD_STAND")
            # Trigger leg stand logic indirectly
            time.sleep(2.0)
            self.update_phase("PATROL_SEARCH")

        elif self.state == "PATROL_SEARCH":
            # Command Hexapod to walk forward slowly
            twist = Twist()
            twist.linear.x = 0.2
            if self.terrain == "ROUGH":
                self.get_logger().warn("Rough terrain detected! Slowing down.")
                twist.linear.x = 0.1
            
            self.cmd_vel_pub.publish(twist)
            
            # Victim logic
            if len(self.victims) > 0:
                self.update_phase("VICTIM_DETECTED")
                # Stop walking
                self.cmd_vel_pub.publish(Twist())

        elif self.state == "VICTIM_DETECTED":
            victim = self.victims[0]
            self.get_logger().info(f"Identified Victim ID: {victim.id} at ({victim.location.x}, {victim.location.y})")
            self.get_logger().info("Terrain complex. Engaging DRONE FLIGHT MODE.")
            
            # Switch to drone mode
            mode_msg = RobotMode()
            mode_msg.mode = "DRONE"
            self.robot_mode_pub.publish(mode_msg)
            
            time.sleep(2.0)
            self.update_phase("FLY_TO_VICTIM")

        elif self.state == "FLY_TO_VICTIM":
            self.get_logger().info("Taking off...")
            # Call MAVROS SetMode "GUIDED" then Takeoff logic usually handled here
            # Simulating flight time
            time.sleep(5.0)
            self.update_phase("AIDING_VICTIM")

        elif self.state == "AIDING_VICTIM":
            self.get_logger().info("Delivering medical payload...")
            time.sleep(3.0)
            self.update_phase("RETURN_TO_HOME")

        elif self.state == "RETURN_TO_HOME":
            self.get_logger().info("Mission Accomplished. Switching to RTL.")
            req = SetMode.Request()
            req.custom_mode = "RTL"
            self.set_mode_client.call_async(req)
            self.update_phase("COMPLETED")

def main(args=None):
    rclpy.init(args=args)
    node = MissionCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Mission aborted manually.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
