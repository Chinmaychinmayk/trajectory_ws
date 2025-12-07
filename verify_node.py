import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import time

class VerificationNode(Node):
    def __init__(self):
        super().__init__('verification_node')
        self.pub = self.create_publisher(Odometry, '/odom', 10)
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.cmd_received = False
        self.timer = self.create_timer(0.1, self.publish_odom)
        self.start_time = time.time()

    def publish_odom(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.orientation.w = 1.0
        self.pub.publish(msg)

    def cmd_callback(self, msg):
        if not self.cmd_received:
            self.get_logger().info(f'Received cmd_vel: linear={msg.linear.x}, angular={msg.angular.z}')
            self.cmd_received = True

def main(args=None):
    rclpy.init(args=args)
    node = VerificationNode()
    
    try:
        start = time.time()
        while rclpy.ok() and (time.time() - start < 10.0):
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.cmd_received:
                print("VERIFICATION SUCCESS: /cmd_vel received.")
                break
    except Exception as e:
        print(f"Error: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
