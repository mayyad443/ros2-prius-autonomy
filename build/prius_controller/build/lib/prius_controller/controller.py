import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class PriusController(Node):
    def __init__(self):
        super().__init__('prius_controller')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.move_robot)

    def move_robot(self):
        msg = Twist()
        msg.linear.x = 2.0   # forward speed
        msg.angular.z = 0.0  # turning
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PriusController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
