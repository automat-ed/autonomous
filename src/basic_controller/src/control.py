#!/usr/bin/env python

import sys
import os
import rospy
from rospy.numpy_msg import numpy_msg
from custom_msgs.msg import ControlStamped
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from std_msgs.msg import String


file_dir = os.path.dirname(__file__)
sys.path.append(file_dir)

import path_detection

class Control:
    def __init__(self):
        rospy.init_node('basic_control')

        self.bridge = CvBridge()

        # Subscribers
        self.camera_sub = rospy.Subscriber('/camera', Image, self.detect_path, queue_size=10)

        # Publishers
        self.control_pub = rospy.Publisher('/cmd', ControlStamped, queue_size=10)


        # The controller node now sends data indefinetely to the topic
        # Once input data from other sensor nodes (camera, lidar, etc ...) will start to arrive
        # the send_command() function will be positioned in appropriate callback functions
        # in response to the sensor input
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.send_command()
            rate.sleep()

    def send_command(self):
        msg = ControlStamped()
        msg.control.speed = 5
        msg.control.acceleration = 0
        msg.control.steering_angle = 0
        self.control_pub.publish(msg)


    def detect_path(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data)

        cv2.imshow('window', cv_image)
        cv2.waitKey(3)

def main(args):
    _ = Control()


if __name__ == '__main__':
    main(sys.argv)
