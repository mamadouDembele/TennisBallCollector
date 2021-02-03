#!/usr/bin/env python3

"""
This program is demonstration for face and object detection using haar-like features.
The program finds faces in a camera image or video stream and displays a red box around them.
Original C implementation by:  ?
Python implementation by: Roman Stanchak, James Bowman
Updated: Copyright (c) 2016, Tal Regev.
"""

import sys
import os
from optparse import OptionParser
from rclpy.qos import DurabilityPolicy, QoSProfile, HistoryPolicy, ReliabilityPolicy

import rclpy
import sensor_msgs.msg
from cv_bridge import CvBridge
import cv2
import numpy as np

class Tracker:
    """
    A basic color tracker, it will look for colors in a range and
    create an x and y offset valuefrom the midpoint
    """

    def __init__(self, height, width, color_lower, color_upper):
        self.color_lower = color_lower
        self.color_upper = color_upper
        self.midx = int(width / 2)
        self.midy = int(height / 2)
        self.xoffset = 0
        self.yoffset = 0

    def draw_arrows(self, frame):
        """Show the direction vector output in the cv2 window"""
        #cv2.putText(frame,"Color:", (0, 35), cv2.FONT_HERSHEY_SIMPLEX, 1, 255, thickness=2)
        cv2.arrowedLine(frame, (self.midx, self.midy),
                        (self.midx + self.xoffset, self.midy - self.yoffset),
                        (0, 0, 255), 5)
        return frame

    def track(self, frame):
        """Simple HSV color space tracking"""
        # resize the frame, blur it, and convert it to the HSV
        # color space
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # construct a mask for the color then perform
        # a series of dilations and erosions to remove any small
        # blobs left in the mask
        mask = cv2.inRange(hsv, self.color_lower, self.color_upper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # find contours in the mask and initialize the current
        # (x, y) center of the ball
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0]
        center = None

        count = 0
        centers = []
        for c in cnts:
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            
            
            # only proceed if the radius meets a minimum size
            if radius < 50:
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                cv2.circle(frame, (int(x), int(y)), int(radius),
                           (0, 255, 255), 2)
                cv2.circle(frame, center, 1, (0, 0, 255), -1)
                centers.append(list(center))
                self.xoffset = int(center[0] - self.midx)
                self.yoffset = int(self.midy - center[1])
                count += 1 
            
            else:
                self.xoffset = 0
                self.yoffset = 0
                
        else:
            self.xoffset = 0
            self.yoffset = 0
        print("nb detected balls : ", count)
        return centers, frame

def getBallPosition(centers):
    courtWidthPix = 1035
    courtWidthM = 23.77 
    unit = courtWidthM / courtWidthPix
    for center in centers:
        center[0] = center[0] * unit
        center[1] = center[1] * unit
    return centers

def getProjectedFrame(frame):

    w, h = 1376, 800
    # y vers le bas & x vers la droite
    bg = [32,682]
    hg = [32, 37]
    hd = [1247,37]
    bd = [1247, 682]

    pts1 = np.float32([hg, hd, bg, bd])
    pts2 = np.float32([[0,0],[w,0],[0,h],[w, h]])

    M = cv2.getPerspectiveTransform(pts1,pts2)

    dst = cv2.warpPerspective(frame,M,(w,h))
    return dst

def computerVision(frame):
    frame = getProjectedFrame(frame)

    green_lower = (30, 50, 50)
    green_upper = (70, 255, 255)
    red_lower = (160, 50, 50)
    red_upper = (180, 255, 255)
    
    height, width = frame.shape[0], frame.shape[1]

    greentracker = Tracker(height, width, green_lower, green_upper)
    redtracker = Tracker(height, width, red_lower, red_upper)
    
    ball_centers, frame = greentracker.track(frame)
    robot_centers, frame = redtracker.track(frame)
    
    ball_centers = getBallPosition(ball_centers)
    robot_centers = getBallPosition(robot_centers)

    return ball_centers, frame 





def detect_and_draw(imgmsg):
    br = CvBridge()
    img = br.imgmsg_to_cv2(imgmsg, "bgr8")
    ball_centers, frame = computerVision(img)
    cv2.imshow("Frame", frame)
    cv2.waitKey(6)
    
    # cv2.imwrite('/tmp/post_im.png', img)



def main(args=None):
    if args is None:
        args = sys.argv
    rclpy.init(args=args)

    qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_ALL, depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
    br = CvBridge()

    node = rclpy.create_node('rosfacedetect')
    node_logger = node.get_logger()
    sub_img = node.create_subscription(sensor_msgs.msg.Image, '/zenith_camera/image_raw', detect_and_draw, qos_profile)

    while rclpy.ok():
      try:
        rclpy.spin_once(node)
      except KeyboardInterrupt:
        node_logger.info("shutting down: keyboard interrupt")
        break

    node_logger.info("test_completed")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()