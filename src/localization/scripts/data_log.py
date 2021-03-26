#!/usr/bin/env python

import os
import re
import rospy
import pandas as pd
from pathlib import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


class DataLogger():

    def __init__(self):
        # Initialize ROS node
        rospy.init_node('data_collector')

        # ROS Parameters
        self.frequency = rospy.get_param('~frequency', 1)
        self.gps_gt_path = rospy.get_param('~gps_gt', 'gps_gt.csv')
        self.gps_global_path = rospy.get_param('~gps_global', 'gps_local.csv')
        self.gps_local_path = rospy.get_param('~gps_local', 'gps_global.csv')

        # Wait for move_base goal to be set
        rospy.wait_for_message("/move_base_simple/goal", PoseStamped)

        # ROS Subscribers
        rospy.Subscriber('/ground_truth/pose', Odometry, self.gps_gt_callback)
        rospy.Subscriber('/odometry/filtered/global', Odometry, self.gps_global_callback)
        rospy.Subscriber('/odometry/filtered/local', Odometry, self.gps_local_callback)

        # Used to store the latest messages from each topic
        self.cur_gps_gt = {"x": 0, "y": 0}
        self.cur_gps_global = {"x": 0, "y": 0}
        self.cur_gps_local = {"x": 0, "y": 0}

        # Used to store collected data to be written to file
        self.gps_gt = {"x": [], "y": [], "time": []}
        self.gps_global = {"x": [], "y": [], "time": []}
        self.gps_local = {"x": [], "y": [], "time": []}

        rospy.loginfo("Recording data...")

        # Timer to save data periodically (based on parameter)
        rospy.Timer(rospy.Duration.from_sec(1 / self.frequency), self.push_state)

    def gps_gt_callback(self, msg):
        self.cur_gps_gt["x"] = msg.pose.pose.position.x
        self.cur_gps_gt["y"] = msg.pose.pose.position.y

    def gps_global_callback(self, msg):
        self.cur_gps_global["x"] = msg.pose.pose.position.x
        self.cur_gps_global["y"] = msg.pose.pose.position.y

    def gps_local_callback(self, msg):
        self.cur_gps_local["x"] = msg.pose.pose.position.x
        self.cur_gps_local["y"] = msg.pose.pose.position.y

    def push_state(self, event):
        # Get current time to time stamp each position
        cur_time = rospy.Time.now()

        # Push current state to lists
        self.gps_gt["x"].append(self.cur_gps_gt["x"])
        self.gps_gt["y"].append(self.cur_gps_gt["y"])
        self.gps_gt["time"].append(cur_time.to_sec())

        self.gps_global["x"].append(self.cur_gps_global["x"])
        self.gps_global["y"].append(self.cur_gps_global["y"])
        self.gps_global["time"].append(cur_time.to_sec())

        self.gps_local["x"].append(self.cur_gps_local["x"])
        self.gps_local["y"].append(self.cur_gps_local["y"])
        self.gps_local["time"].append(cur_time.to_sec())

    def run(self):
        rospy.spin()
        self.save()

    def save(self):
        # Save data sets to their configured files
        rospy.loginfo("Saving gps ground truth...")
        gt_path = self.generate_file_path(self.gps_gt_path)
        with open(gt_path, 'w') as f:
            # Convert to Pandas DataFrame
            pd_gt = pd.DataFrame(self.gps_gt)
            pd_gt_csv = pd_gt.to_csv()
            f.write(pd_gt_csv)

        rospy.loginfo("Saving gps global...")
        global_path = self.generate_file_path(self.gps_global_path)
        with open(global_path, 'w') as f:
            # Convert to Pandas DataFrame
            pd_global = pd.DataFrame(self.gps_global)
            pd_global_csv = pd_global.to_csv()
            f.write(pd_global_csv)

        rospy.loginfo("Saving gps local...")
        local_path = self.generate_file_path(self.gps_local_path)
        with open(local_path, 'w') as f:
            # Convert to Pandas DataFrame
            pd_local = pd.DataFrame(self.gps_local)
            pd_local_csv = pd_local.to_csv()
            f.write(pd_local_csv)

        rospy.loginfo("Successfully saved all data.")

    def generate_file_path(self, path):
        file_path = Path(os.path.expanduser(path))
        while file_path.is_file():
            # Find number
            p = re.compile(r'(?<=\()\d+(?=\).)')
            reg_search = re.search(p, file_path.name)

            if reg_search is None:
                new_name = file_path.stem + "-(1)" + file_path.suffix
            else:
                new_name = re.sub(p, str(int(reg_search.group()) + 1), file_path.name)

            file_path = Path(file_path.parent, new_name)

        return str(file_path)


if __name__ == "__main__":
    DataLogger().run()
