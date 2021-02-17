import rclpy
from rclpy.node import Node

from std_msgs import FLoat32
from geometry_msgs.msg import Pose, Twist
from sensor_msgs import Imu
from tf.transformations import euler_from_quaternion

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
        self.robot_angle_subscription = self.create_subscription(Imu, 'imu_plugin/out', self.robot_angle_callback, 25)
        self.ball_subscription = self.create_subscription(Pose, 'pose_ball', self.ball_position_callback, 25)

        # Ball and robot position
        self.X = np.zeros((3, 1))
        self.B = np.zeros((3, 1))

        # Changing control law
        isNear = False
        distance_near = 3
        distance_threshold = 0.5

    def robot_position_callback(self, data):
        self.X[0, 0] = data.position.x
        self.X[1, 0] = data.position.y

    def robot_angle_callback(self, data):
        (roll, pitch, yaw) = euler_from_quaternion ([data.orientation.x, data.orientation.y, data.orientation.z, orientation_data.orientation.w])
        self.X[2, 0] = yaw
    
    def ball_position_callback(self, data):
        self.B = np.array([[data.position.x], [data.position.y], [data.angle.z]])

    def timer_callback(self):
        # Position error processing
        e = (self.B - self.X)

        # Switching control law
        if e < self.distance_near - distance_threshold:
            self.isNear = True
        if e > self.distance_near + distance_threshold:
            self.isNear = False

        # Computing angle control
        if self.isNear:
            theta = self.B[2, 0] - self.X[2, 0]
        else:
            theta = np.arctan2(e[1, 0], e[0, 0]) - self.X[2, 0]
        
        # Computing position control
        if not np.allclose(e, np.zeros((3, 1))):
            e /= np.linalg.norm(e)
        else:
            e = np.zeros((3, 1))

        # Building message
        msg = Twist()
        msg.linear.x = e[0, 0] * np.cos(theta)
        msg.angular.z = theta

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
