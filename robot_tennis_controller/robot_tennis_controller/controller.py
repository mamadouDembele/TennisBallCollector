import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, Twist

import numpy as np

class Controller(Node):

    def __init__(self):
        super().__init__('controller')

        timer_period = 0.05
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Publisher
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 25)

        # Subscriber
        self.robot_subscription = self.create_subscription(Pose, 'pose_rob', self.robot_position_callback, 25)
        self.ball_subscription = self.create_subscription(Pose, 'pose_ball', self.ball_position_callback, 25)

        # Ball and robot position
        self.X = np.zeros((3, 1))
        self.B = np.zeros((3, 1))

    def robot_position_callback(self, data):
        self.X = np.array([[data.position.x], [data.position.y], [data.position.z]])
    
    def ball_position_callback(self, data):
        self.B = np.array([[data.position.x], [data.position.y], [data.position.z]])

    def timer_callback(self):
        d = (self.B - self.X)
        if not np.allclose(d, np.zeros((3, 1))):
            d /= np.linalg.norm(self.B - self.X)

        # Building message
        msg = Twist()
        msg.linear.x = d[0, 0]
        msg.linear.y = d[1, 0]

        # Publishing
        self.publisher.publish(msg)
        self.get_logger().info("Publishing: {}, {}, {}".format(msg.linear.x, msg.linear.y, msg.angular.z))


def main(args=None):
    rclpy.init(args=args)
    controller = Controller()
    rclpy.spin(controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
