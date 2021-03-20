import rospy
import pandas as pd
from nav_msgs.msg import Odometry


class Collector():

    def __init__(self):
        # Initialize ROS node
        rospy.init_node('data_collector')

        # ROS Parameters
        self.frequency = rospy.get_param('~frequency', 10)
        self.gps_gt_file_path = rospy.get_param('~gps_gt', 'gps_gt.txt')
        self.gps_global_file_path = rospy.get_param('~gps_global', 'gps_local.txt')
        self.gps_local_file_path = rospy.get_param('~gps_local', 'gps_global.txt')

        # ROS Subscribers
        #rospy.Subscriber('/ground_truth/pose', Odometry, self.gps_gt_callback)
        rospy.Subscriber('odometry/filtered/local', Odometry, self.gps_local_callback)
        rospy.Subscriber('odometry/filtered/global', Odometry, self.gps_global_callback)

        # Used to store the latest messages from each topic
        self.cur_gps_gt = {"x": 0, "y": 0}
        self.cur_gps_local = {"x": 0, "y": 0}
        self.cur_gps_global = {"x": 0, "y": 0}

        # Used to store collected data to be written to file
        self.gps_gt = {"x": [], "y": [], "time": []}
        self.gps_local = {"x": [], "y": [], "time": []}
        self.gps_global = {"x": [], "y": [], "time": []}

        # Timer to save data periodically (based on parameter)
        rospy.Timer(rospy.Duration.from_sec(1 / self.frequency), self.push_state)

    def gps_gt_callback(self, msg):
        self.cur_gps_gt["x"] = msg.pose.pose.position.x
        self.cur_gps_gt["y"] = msg.pose.pose.position.y

    def gps_local_callback(self, msg):
        self.cur_gps_local["x"] = msg.pose.pose.position.x
        self.cur_gps_local["y"] = msg.pose.pose.position.y

    def gps_global_callback(self, msg):
        self.cur_gps_global["x"] = msg.pose.pose.position.x
        self.cur_gps_global["y"] = msg.pose.pose.position.y

    def push_state(self):
        # Get current time to time stamp each position
        cur_time = rospy.Time.now()

        # Push current state to lists
        rospy.loginfo("GPS ground truth: ({}, {}) }",
                      self.cur_gps_gt["x"],
                      self.cur_gps_gt["y"])
        self.gps_gt["x"] = self.cur_gps_gt["x"]
        self.gps_gt["y"] = self.cur_gps_gt["y"]
        self.gps_gt["time"] = cur_time.to_sec()

        rospy.loginfo("GPS global: ({}, {}) }",
                      self.cur_gps_gt["x"],
                      self.cur_gps_gt["y"])
        self.gps_global["x"] = self.cur_gps_global["x"]
        self.gps_global["y"] = self.cur_gps_global["y"]
        self.gps_global["time"] = cur_time.to_sec()

        rospy.loginfo("GPS local: ({}, {}) }",
                      self.cur_gps_gt["x"],
                      self.cur_gps_gt["y"])
        self.gps_local["x"] = self.gps_local["x"]
        self.gps_local["y"] = self.gps_local["y"]
        self.gps_local["time"] = cur_time.to_sec()

    def __del__(self):
        # Save data sets to their configured files
        with open(self.gps_gt_file_path, 'w') as f:
            # Convert to Pandas DataFrame
            pd_gt = pd.DataFrame(self.gps_gt)
            f.write(pd_gt.to_csv())

        with open(self.gps_global_file_path, 'w') as f:
            # Convert to Pandas DataFrame
            pd_global = pd.DataFrame(self.gps_global)
            f.write(pd_global.to_csv())

        with open(self.gps_local_file_path, 'w') as f:
            # Convert to Pandas DataFrame
            pd_local = pd.DataFrame(self.gps_local)
            f.write(pd_local.to_csv())


def main():
    collector = Collector()

    rospy.sleep(150)
    print("Data has been collected")
    collector.write_to_file()


if __name__ == "__main__":
    main()
