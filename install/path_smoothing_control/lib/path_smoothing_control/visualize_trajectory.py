#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import threading

class TrajectoryVisualizer(Node):
    def __init__(self):
        super().__init__('trajectory_visualizer')

        self.waypoint_path = []
        self.smooth_path = []
        self.robot_positions = []

        # Subscribers
        self.create_subscription(Path, '/planned_path', self.waypoint_callback, 10)
        self.create_subscription(Path, '/smooth_path', self.smooth_path_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Setup plot
        self.fig, self.ax = plt.subplots(figsize=(10, 8))
        self.setup_plot()

        # Animation
        self.ani = FuncAnimation(self.fig, self.update_plot, interval=100, blit=False)

    def setup_plot(self):
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_title('Robot Trajectory Tracking')
        self.ax.grid(True, alpha=0.3)
        self.ax.set_aspect('equal')

        # Initialize plot elements
        self.waypoint_line, = self.ax.plot([], [], 'ro', label='Waypoints', markersize=8)
        self.smooth_line, = self.ax.plot([], [], 'g-', label='Smooth Path', linewidth=2, alpha=0.7)
        self.robot_line, = self.ax.plot([], [], 'b-', label='Robot Path', linewidth=1.5, alpha=0.8)
        self.robot_marker, = self.ax.plot([], [], 'bs', label='Robot', markersize=10)

        self.ax.legend(loc='upper right')

    def waypoint_callback(self, msg):
        self.waypoint_path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def smooth_path_callback(self, msg):
        self.smooth_path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.robot_positions.append((x, y))

        # Keep last 1000 positions
        if len(self.robot_positions) > 1000:
            self.robot_positions.pop(0)

    def update_plot(self, frame):
        # Update waypoints
        if self.waypoint_path:
            wx, wy = zip(*self.waypoint_path)
            self.waypoint_line.set_data(wx, wy)

        # Update smooth path
        if self.smooth_path:
            sx, sy = zip(*self.smooth_path)
            self.smooth_line.set_data(sx, sy)

        # Update robot trajectory
        if self.robot_positions:
            rx, ry = zip(*self.robot_positions)
            self.robot_line.set_data(rx, ry)

            # Current robot position
            self.robot_marker.set_data([rx[-1]], [ry[-1]])

            # Auto-scale axes
            all_points = self.smooth_path + self.robot_positions
            if all_points:
                all_x, all_y = zip(*all_points)
                if all_x and all_y:
                    margin = 0.5
                    self.ax.set_xlim(min(all_x) - margin, max(all_x) + margin)
                    self.ax.set_ylim(min(all_y) - margin, max(all_y) + margin)

        return self.waypoint_line, self.smooth_line, self.robot_line, self.robot_marker

def main(args=None):
    rclpy.init(args=args)
    visualizer = TrajectoryVisualizer()

    # Spin in background
    spin_thread = threading.Thread(target=rclpy.spin, args=(visualizer,), daemon=True)
    spin_thread.start()

    # Show plot
    plt.show()

    visualizer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
