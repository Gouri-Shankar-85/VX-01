import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener
from math import sqrt
import time

class WalkTest(Node):
    def __init__(self):
        super().__init__('walk_test')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.start_pose = None
        self.timer = self.create_timer(0.1, self.tick)
        self.ticks = 0

    def tick(self):
        msg = Twist()
        msg.linear.x = 0.15
        self.pub.publish(msg)
        self.ticks += 1
        
        try:
            trans = self.tf_buffer.lookup_transform('world', 'base_link', rclpy.time.Time())
            x, y, z = trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z
            if self.start_pose is None:
                self.start_pose = (x, y, z)
            else:
                dist = sqrt((x - self.start_pose[0])**2 + (y - self.start_pose[1])**2)
                if self.ticks % 10 == 0:
                    print(f"Time: {self.ticks/10.0}s | Moved: {dist:.3f}m | Z/Height: {z:.3f}m | X: {x:.3f}, Y: {y:.3f}", flush=True)
        except Exception as e:
            if self.ticks % 10 == 0:
                print(f"TF Error: {e}", flush=True)

        if self.ticks > 150:
            print("Test complete.", flush=True)
            self.destroy_node()
            rclpy.shutdown()

rclpy.init()
n = WalkTest()
rclpy.spin(n)
