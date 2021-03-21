import rospy
import pandas as pd
from nav_msgs.msg import Odometry


class DataCollector():

    def __init__(self):
        # Initialize ROS node
        rospy.init_node('data_collector')

        # ROS Parameters
        self.frequency = rospy.get_param('~frequency', 1)
        self.gps_gt_path = rospy.get_param('~gps_gt', 'gps_gt.csv')
        self.gps_global_path = rospy.get_param('~gps_global', 'gps_local.csv')
        self.gps_local_path = rospy.get_param('~gps_local', 'gps_global.csv')

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
        #rospy.loginfo("GPS ground truth: ({}, {})".format(self.cur_gps_gt["x"], self.cur_gps_gt["y"]))
        self.gps_gt["x"].append(self.cur_gps_gt["x"])
        self.gps_gt["y"].append(self.cur_gps_gt["y"])
        self.gps_gt["time"].append(cur_time.to_sec())

        #rospy.loginfo("GPS global: ({}, {})".format(self.cur_gps_gt["x"], self.cur_gps_gt["y"]))
        self.gps_global["x"].append(self.cur_gps_global["x"])
        self.gps_global["y"].append(self.cur_gps_global["y"])
        self.gps_global["time"].append(cur_time.to_sec())

        #rospy.loginfo("GPS local: ({}, {})".format(self.cur_gps_gt["x"], self.cur_gps_gt["y"]))
        self.gps_local["x"].append(self.cur_gps_local["x"])
        self.gps_local["y"].append(self.cur_gps_local["y"])
        self.gps_local["time"].append(cur_time.to_sec())

    def run(self):
        rospy.spin()

    def __del__(self):
        # Save data sets to their configured files
        rospy.loginfo("Saving gps ground truth...")
        with open(self.gps_gt_path, 'w') as f:
            # Convert to Pandas DataFrame
            pd_gt = pd.DataFrame(self.gps_gt)
            pd_gt_csv = pd_gt.to_csv()
            f.write(pd_gt_csv)

        rospy.loginfo("Saving gps global...")
        with open(self.gps_global_path, 'w') as f:
            # Convert to Pandas DataFrame
            pd_global = pd.DataFrame(self.gps_global)
            pd_global_csv = pd_global.to_csv()
            f.write(pd_global_csv)

        rospy.loginfo("Saving gps local...")
        with open(self.gps_local_path, 'w') as f:
            # Convert to Pandas DataFrame
            pd_local = pd.DataFrame(self.gps_local)
            pd_local_csv = pd_local.to_csv()
            f.write(pd_local_csv)

        rospy.loginfo("Successfully saved all data.")


if __name__ == "__main__":
    DataCollector().run()
