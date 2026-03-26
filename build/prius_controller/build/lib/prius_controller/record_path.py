import csv
import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class PathRecorder(Node):
    def __init__(self):
        super().__init__('path_recorder')

        self.odom_sub = self.create_subscription(
            Odometry,
            '/model/prius_hybrid/odometry',
            self.odom_callback,
            10
        )

        self.output_path = Path.home() / 'ros2_ws' / 'recorded_path.csv'
        self.min_spacing = 1.0  # meters between saved points
        self.last_x = None
        self.last_y = None
        self.points_saved = 0

        with open(self.output_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['x', 'y'])

        self.get_logger().info(f'Recording path to {self.output_path}')

    def odom_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if self.last_x is None or self.last_y is None:
            self.save_point(x, y)
            return

        dist = math.hypot(x - self.last_x, y - self.last_y)
        if dist >= self.min_spacing:
            self.save_point(x, y)

    def save_point(self, x: float, y: float):
        with open(self.output_path, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([x, y])

        self.last_x = x
        self.last_y = y
        self.points_saved += 1
        self.get_logger().info(
            f'Saved point #{self.points_saved}: ({x:.2f}, {y:.2f})'
        )


def main(args=None):
    rclpy.init(args=args)
    node = PathRecorder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
