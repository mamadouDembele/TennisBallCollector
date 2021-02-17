import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, Twist
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

import numpy as np

import math
 
def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

    

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

        self.robot_odometry_subscription = self.create_subscription(Odometry, 'odom', self.odometry_callback, 25)

        # Ball and robot position
        self.X = np.zeros((3, 1))
        self.B = np.zeros((3, 1))

        # Changing control law
        self.isNear = False
        self.distance_near = 3
        self.distance_threshold = 0.5

    def robot_position_callback(self, data):
        self.X[0, 0] = data.position.x
        self.X[1, 0] = data.position.y

    def odometry_callback(self, data):
        self.X[0, 0] = data.pose.pose.position.x
        self.X[1, 0] = data.pose.pose.position.y
        (roll, pitch, yaw) = euler_from_quaternion (data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
        self.X[2, 0] = yaw

    def robot_angle_callback(self, data):
        (roll, pitch, yaw) = euler_from_quaternion (data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w)
        self.X[2, 0] = yaw
    
    def ball_position_callback(self, data):
        (roll, pitch, yaw) = euler_from_quaternion (data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w)
        self.B = np.array([[data.position.x], [data.position.y], [yaw]])

    def timer_callback(self):
        # Position error processing
        e = (self.B - self.X)

        # Switching control law
        if np.linalg.norm(e) < self.distance_near - self.distance_threshold:
            self.isNear = True
        if np.linalg.norm(e) > self.distance_near + self.distance_threshold:
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

        kp = 3

        # Building message
        msg = Twist()
        msg.linear.x = kp * e[0, 0] * np.cos(theta)
        msg.angular.z = theta

        # Publishing
        self.publisher.publish(msg)
        self.get_logger().info("Robot: {}, {}, {}".format(self.X[0, 0], self.X[1, 0], self.X[2, 0]))
        self.get_logger().info("Ball: {}, {}, {}".format(self.B[0, 0], self.B[1, 0], self.B[2, 0]))
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
