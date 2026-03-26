import csv
import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/model/prius_hybrid/odometry',
            self.odom_callback,
            10
        )

        self.timer = self.create_timer(0.1, self.control_loop)

        self.x = None
        self.y = None
        self.yaw = None

        self.path_file = Path.home() / 'ros2_ws' / 'recorded_path.csv'
        self.path_points = self.load_path(self.path_file)
        self.current_index = 0

        self.goal_tolerance = 2.0
        self.max_speed = 3.0
        self.k_steer = 2.5
        self.lookahead_points = 3

        self.get_logger().info(f'Loaded {len(self.path_points)} path points')

    def load_path(self, path_file: Path):
        points = []
        if not path_file.exists():
            raise FileNotFoundError(f'Path file not found: {path_file}')

        with open(path_file, 'r', newline='') as f:
            reader = csv.DictReader(f)
            for row in reader:
                points.append((float(row['x']), float(row['y'])))

        if not points:
            raise RuntimeError('No path points found in CSV')

        return points

    def odom_callback(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        self.yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)

    def quaternion_to_yaw(self, x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    def control_loop(self):
        if self.x is None or self.yaw is None:
            return

        if self.current_index >= len(self.path_points):
            msg = Twist()
            self.cmd_pub.publish(msg)
            self.get_logger().info('Finished recorded path')
            return

        # Skip points that are already reached
        while self.current_index < len(self.path_points) - 1:
            px, py = self.path_points[self.current_index]
            if math.hypot(px - self.x, py - self.y) < self.goal_tolerance:
                self.current_index += 1
            else:
                break

        if self.current_index >= len(self.path_points):
            msg = Twist()
            self.cmd_pub.publish(msg)
            self.get_logger().info('Finished recorded path')
            return

        # Aim ahead for smoother tracking
        look_idx = min(self.current_index + self.lookahead_points, len(self.path_points) - 1)
        goal_x, goal_y = self.path_points[look_idx]

        dx = goal_x - self.x
        dy = goal_y - self.y
        distance = math.hypot(dx, dy)

        target_yaw = math.atan2(dy, dx)
        heading_error = self.normalize_angle(target_yaw - self.yaw)

        msg = Twist()
        steering = max(min(self.k_steer * heading_error, 2.0), -2.0)
        msg.angular.z = steering

        if abs(heading_error) > 1.0:
            msg.linear.x = 0.2
        elif abs(heading_error) > 0.5:
            msg.linear.x = 0.5
        else:
            msg.linear.x = self.max_speed

        self.cmd_pub.publish(msg)

        self.get_logger().info(
            f'Idx={self.current_index} LookIdx={look_idx} '
            f'Pose=({self.x:.2f}, {self.y:.2f}) '
            f'Goal=({goal_x:.2f}, {goal_y:.2f}) '
            f'Dist={distance:.2f} '
            f'Yaw={self.yaw:.2f} TargetYaw={target_yaw:.2f} '
            f'HeadingErr={heading_error:.2f} '
            f'Cmd=({msg.linear.x:.2f}, {msg.angular.z:.2f})'
        )
def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
