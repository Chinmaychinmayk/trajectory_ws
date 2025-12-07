import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
import time

from rclpy.qos import QoSProfile, DurabilityPolicy

class PathListener(Node):
    def __init__(self):
        super().__init__('path_listener')
        qos_profile = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.subscription = self.create_subscription(
            Path,
            '/smooth_path',
            self.listener_callback,
            qos_profile)
        self.received = False

    def listener_callback(self, msg):
        self.get_logger().info(f'Received path with {len(msg.poses)} points')
        if msg.poses:
            first = msg.poses[0].pose.position
            last = msg.poses[-1].pose.position
            mid = msg.poses[len(msg.poses)//2].pose.position
            self.get_logger().info(f'Start: ({first.x}, {first.y})')
            self.get_logger().info(f'Mid:   ({mid.x}, {mid.y})')
            self.get_logger().info(f'End:   ({last.x}, {last.y})')
            
            # Print last 5 points to see if it drops to zero
            for i in range(1, 6):
                p = msg.poses[-i].pose.position
                self.get_logger().info(f'End-{i-1}: ({p.x}, {p.y})')
                
        self.received = True

def main(args=None):
    rclpy.init(args=args)
    path_listener = PathListener()
    
    start = time.time()
    while not path_listener.received and time.time() - start < 10.0:
        rclpy.spin_once(path_listener, timeout_sec=1.0)
        
    path_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
