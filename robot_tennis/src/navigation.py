import math
import numpy
import sys

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Bool
from rclpy.node import Node
from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry


class TennisCollectorNavigation(Node):

    def __init__(self):
        
        super().__init__('navigation')
        
        self.is_ball_collected = False
        self.is_ball_on_court = False
        self.pose_ball_x = 0.0
        self.pose_ball_y = 0.0
        self.pose_rob_x = 0.0
        self.pose_rob_y = 0.0

        self.state = 0
        self.x_wp = 0.0
        self.y_wp = 0.0
        self.move = False

        self.waypoint_pub = self.create_publisher(, 'waypoint', qos)

        self.ball_sub = self.create_subscription(
            PoseStamped,
            'ball',
            self.ball_callback,
            qos)
        
        self.rob_sub = self.create_subscription(
            PoseStamped,
            'robot',
            self.rob_callback,
            qos)

        self.is_ball_collected_sub = self.create_subscription(
            Bool,
            'ibc',
            self.rob_callback,
            qos)

        self.is_ball_on_court_sub = self.create_subscription(
            Bool,
            'iboc',
            self.rob_callback,
            qos)

    def ball_callback(self, msg):

        self.is_ball_collected = msg.pose.pose.position.x
        self.is_ball_on_court = msg.pose.pose.position.x
        self.pose_ball_x = msg.pose.pose.position.x
        self.pose_ball_y = msg.pose.pose.position.y

    def rob_callback(self, msg):

        self.pose_rob_x = msg.pose.pose.position.x
        self.pose_rob_y = msg.pose.pose.position.y

    def waypoint(self):

        # STATE 1: GO TO THE OTHER SIDE OF THE COURT BYPASSING THE NET
        #-------------------------------------------------------------------
        if self.state == 1:

            if self.is_ball_on_court == True:
                if sign(self.pose_rob.y) != sign(self.pose_ball_y):
                    self.x_wp = 6.5*sign(self.poserob.x)
                    self.y_wp = 0.0
                else:
                    self.state = 2
            else :
                self.state = 0
        #-------------------------------------------------------------------


        # STATE 2: GO THE NEXT BALL TO COLLECT IT
        #-------------------------------------------------------------------
        if self.state == 2:

            if self.is_ball_on_court == True:
                if sign(self.pose_rob.y) != sign(self.pose_ball_y):
                    state = 2
                else:
                    self.x_wp = self.pose_ball_x
                    self.y_wp = self.pose_ball_y
            else:
                self.state = 0
        #-------------------------------------------------------------------


        # STATE 3: BRING THE BALL INTO THE COLLECTING AREAS
        #-------------------------------------------------------------------
        if self.state == 3:

            if self.is_ball_collected == True:
                self.x_wp = 7.0*sign(self.pose_rob.y)
                self.y_wp = 14.0*sign(self.pose_rob.y)
            else:
                self.state = 2
        #-------------------------------------------------------------------


        # STATE 0: IDLE
        #-------------------------------------------------------------------
        else: # state == 0

            if self.is_ball_on_court == True
                self.state = 2
                self.move == True
            else:
                if 6.0 < abs(self.pose_rob.x) < 7.0 and abs(self.pose_rob.y) < .5:
                    # stop robot
                    self.move = False
                else:
                    self.state = 1
                    self.move = True
        #-------------------------------------------------------------------

        self.wp_pub.publish( "publish self.wp" )
