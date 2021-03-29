#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from std_msgs.msg import Bool, Float64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from darknet_ros_msgs.msg import BoundingBoxes


class DepthEstimation:
    def __init__(self):
        rospy.init_node('depth_estimation')
        rospy.loginfo("Starting depth_estimation")

        # State
        self.range = None
        self.bounding_boxes = []

        # ROS parameters
        self.frequency = rospy.get_param("frequency", default=10)
        self.max_dist = rospy.get_param("max_dist", default=20)
        self.threshold_dist = rospy.get_param("threshold_dist", default=5)

        # ROS Publisher
        self.stop_pub = rospy.Publisher("/state_machine/stop", Bool, queue_size=1)
        self.dist_pub = rospy.Publisher("/human/dist", Float64, queue_size=1)

        # ROS Subscribers
        rospy.Subscriber("/multi_sense/range/data", Image, self.range_callback)
        rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.darknet_ros_callback)

        # Initialize cv_bridge
        self.bridge = CvBridge()

        # Set loop timer
        rospy.Timer(rospy.Duration(1 / self.frequency), self.loop)

    def range_callback(self, data):
        try:
            self.range = self.bridge.imgmsg_to_cv2(data)
        except CvBridgeError as e:
            print(e)

    def darknet_ros_callback(self, data):
        # Reset bounding box list
        self.bounding_boxes = []

        # Filter for people
        for box in data.bounding_boxes:
            if box.Class == "person":
                self.bounding_boxes.append(box)

    def loop(self, event):
        if len(self.bounding_boxes) == 0:
            rospy.logwarn("No humans!")
            return

        min_dist = self.max_dist
        for box in self.bounding_boxes:
            rospy.loginfo(f"x_min: {box.xmin}, x_max: {box.xmax}, y_min: {box.ymin}, y_max: {box.ymax}")

            # Ensure same size
            sub_image = self.range[box.ymin: box.ymax, box.xmin: box.xmax]

            # cv2.imshow("Sub image", sub_image.reshape(box.ymax - box.ymin, box.xmax - box.xmin))
            # cv2.waitKey(1)

            min_dist = min(min_dist, self.calc_dist(sub_image))

        if min_dist < self.threshold_dist:
            self.dist_pub.publish(min_dist)

    def calc_dist(self, img):
        # Flatten array
        flat_img = img.reshape(-1,)

        # Filter out np.inf values
        filtered_img = np.array([pixel for pixel in flat_img if pixel != np.inf])

        if np.min(filtered_img) == self.max_dist:
            return self.max_dist

        # Filter out max distance
        filtered_img = np.array([pixel for pixel in flat_img if pixel != self.max_dist])
        return np.mean(filtered_img)


if __name__ == "__main__":
    try:
        _ = DepthEstimation()
        rospy.spin()
    except KeyboardInterrupt:
        pass
