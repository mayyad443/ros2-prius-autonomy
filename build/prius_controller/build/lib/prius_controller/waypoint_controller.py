import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class PriusWaypointController(Node):
    def __init__(self):
        super().__init__('prius_waypoint_controller')

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

        self.waypoints = [
            (-300.0, 100.0),
            (225.0, -40.0),
            (100.0, -40.0),
            (100.0, -10.0),
            (307.0, 1.0),
        ]
        self.current_waypoint_index = 0

        self.goal_tolerance = 6.0
        self.max_speed = 4.0
        self.k_steer = 6.0

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

        if self.current_waypoint_index >= len(self.waypoints):
            msg = Twist()
            self.cmd_pub.publish(msg)
            self.get_logger().info('Finished all waypoints')
            return

        goal_x, goal_y = self.waypoints[self.current_waypoint_index]

        dx = goal_x - self.x
        dy = goal_y - self.y
        distance = math.sqrt(dx * dx + dy * dy)

        if distance < self.goal_tolerance:
            self.get_logger().info(
                f'Reached waypoint {self.current_waypoint_index}: ({goal_x}, {goal_y})'
            )
            self.current_waypoint_index += 1
            return

        target_yaw = math.atan2(dy, dx)
        heading_error = self.normalize_angle(target_yaw - self.yaw)

        msg = Twist()
        steering = max(min(self.k_steer * heading_error, 5.0), -5.0)
        msg.angular.z = steering
        if abs(heading_error) > 0.8:
    	     msg.linear.x = 0.2
        elif abs(heading_error) > 0.4:
             msg.linear.x = 0.5
        else:
    	     msg.linear.x = self.max_speed
        self.cmd_pub.publish(msg)
        self.get_logger().info(
            f'Pose=({self.x:.2f}, {self.y:.2f}) '
            f'Goal=({goal_x:.2f}, {goal_y:.2f}) '
            f'Dist={distance:.2f} '
            f'Yaw={self.yaw:.2f} '
            f'TargetYaw={target_yaw:.2f} '
            f'HeadingErr={heading_error:.2f}'
	    f'Cmd=({msg.linear.x:.2f}, {msg.angular.z:.2f})'
        )


def main(args=None):
    rclpy.init(args=args)
    node = PriusWaypointController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
