#!/usr/bin/env python

import subprocess
import rospy
from automated_msgs.msg import NetworkStatus, NetworkStatusStamped, RobotState, RobotStateStamped
from std_msgs.msg import Bool


class StateMachine:
    def __init__(self):
        rospy.init_node("state_machine")

        # Initialize state
        self.robot_state = RobotState.OFF
        self.network_state = NetworkStatus.DISCONNECTED
        self.start = False
        self.emergency = False
        self.reset = False
        self.prev_state = None

        # ROS Parameters
        self.frequency = rospy.get_param("~frequency", 50)
        self.launch_file = rospy.get_param("~launch_file")

        rospy.loginfo(self.launch_file)

        # Define subscribers
        rospy.Subscriber("/state_machine/connection", NetworkStatusStamped, self.network_callback)
        rospy.Subscriber("/state_machine/start", Bool, self.start_callback)
        rospy.Subscriber("/state_machine/emergency", Bool, self.emergency_callback)
        rospy.Subscriber("/state_machine/reset", Bool, self.reset_callback)

        # Define publishers
        self.state_pub = rospy.Publisher("/state_machine/state", RobotStateStamped, queue_size=1)

        # Use timer to run main loop of state machine
        rospy.Timer(rospy.Duration(1 / self.frequency), self.loop)

        rospy.loginfo("state_machine :: Initialized state machine!")

    def off_state(self):
        # Reset must be false in OFF state
        self.reset = False

        # Transition if CONNECTED to Portal
        if self.network_state == NetworkStatus.CONNECTED:
            rospy.loginfo("state_machine :: Connected to Portal!")
            rospy.loginfo("state_machine :: Robot state change: {} --> {}"
                          .format(self.robot_state, RobotState.READY))
            self.launch_nodes()
            self.robot_state = RobotState.READY
        else:
            # Limit log message so it doesn't log every time
            rospy.loginfo_once("state_machine :: Waiting for Portal connection...")

    def ready_state(self):
        # If an emergency stop is requested, move to EMERGENCY state
        if self.emergency:
            rospy.logwarn("state_machine :: Processing emergency stop signal...")
            rospy.loginfo("state_machine :: Robot state change: {} --> {}"
                          .format(self.robot_state, RobotState.EMERGENCY))
            self.robot_state = RobotState.EMERGENCY

        # If we disconnect, move to WAITING state
        elif self.network_state == NetworkStatus.DISCONNECTED:
            rospy.logwarn("state_machine :: Portal disconnected. Waiting for reconnect...")
            rospy.loginfo("state_machine :: Robot state change: {} --> {}"
                          .format(self.robot_state, RobotState.WAITING))
            self.robot_state = RobotState.WAITING

        # If there's a Portal connection error, move to EMERGENCY state
        elif self.network_state == NetworkStatus.ERROR:
            rospy.logerr("state_machine :: Portal conenction error!")
            rospy.loginfo("state_machine :: Robot state change: {} --> {}"
                          .format(self.robot_state, RobotState.EMERGENCY))
            self.robot_state = RobotState.EMERGENCY

        # If we receive start signal, move to DRIVING state
        elif self.start:
            rospy.loginfo("state_machine :: Start signal is true!")
            rospy.loginfo("state_machine :: Robot state change: {} --> {}"
                          .format(self.robot_state, RobotState.DRIVING))
            self.robot_state = RobotState.DRIVING

        else:
            # Limit log message so it doesn't log every time
            rospy.loginfo_once("state_machine :: Waiting for start signal...")

    def driving_state(self):
        # If an emergency stop is requested, move to EMERGENCY state
        if self.emergency:
            rospy.logwarn("state_machine :: Processing emergency stop signal...")
            rospy.loginfo("state_machine :: Robot state change: {} --> {}"
                          .format(self.robot_state, RobotState.EMERGENCY))
            self.robot_state = RobotState.EMERGENCY

        # If we disconnect, move to WAITING state
        elif self.network_state == NetworkStatus.DISCONNECTED:
            rospy.logwarn("state_machine :: Portal disconnected. Waiting for reconnect...")
            rospy.loginfo("state_machine :: Robot state change: {} --> {}"
                          .format(self.robot_state, RobotState.WAITING))
            self.robot_state = RobotState.WAITING

        # If there's a Portal connection error, move to EMERGENCY state
        elif self.network_state == NetworkStatus.ERROR:
            rospy.logerr("state_machine :: Portal conenction error!")
            rospy.loginfo("state_machine :: Robot state change: {} --> {}"
                          .format(self.robot_state, RobotState.EMERGENCY))
            self.robot_state = RobotState.EMERGENCY

        # Move to READY state if start flag is false
        elif not self.start:
            rospy.loginfo("state_machine :: Processing stop signal...")
            rospy.loginfo("state_machine :: Robot state change: {} --> {}"
                          .format(self.robot_state, RobotState.READY))
            self.robot_state = RobotState.READY

        if self.robot_state != RobotState.DRIVING:
            # TODO(angus): stop the robot
            pass

    def waiting_state(self):
        # Transition if CONNECTED to Portal
        if self.network_state == NetworkStatus.CONNECTED:
            rospy.loginfo("state_machine :: Connected to Portal!")
            rospy.loginfo("state_machine :: Robot state change: {} --> {}"
                          .format(self.robot_state, RobotState.READY))
            # TODO(angus): launch ROS nodes
            self.robot_state = RobotState.READY

        # If there's a Portal connection error, move to EMERGENCY state
        elif self.network_state == NetworkStatus.ERROR:
            rospy.logerr("state_machine :: Portal conenction error!")
            rospy.loginfo("state_machine :: Robot state change: {} --> {}"
                          .format(self.robot_state, RobotState.EMERGENCY))
            self.robot_state = RobotState.EMERGENCY

    def emergency_state(self):
        rospy.logerr_once("state_machine :: Entered into emergency state!")
        self.kill_nodes()

        if self.reset:
            rospy.loginfo("state_machine :: Robot state change: {} --> {}"
                          .format(self.robot_state, RobotState.OFF))
            self.robot_state = RobotState.OFF

    def loop(self, event):
        if self.robot_state == RobotState.OFF:
            self.off_state()

        elif self.robot_state == RobotState.READY:
            self.ready_state()

        elif self.robot_state == RobotState.DRIVING:
            self.driving_state()

        elif self.robot_state == RobotState.WAITING:
            self.waiting_state()

        elif self.robot_state == RobotState.EMERGENCY:
            self.emergency_state()

        else:
            rospy.logfatal("state_machine :: State machine has entered invalid state: {}".format(self.robot_state))
            rospy.loginfo("state_machine :: Robot state change: {} --> {}"
                          .format(self.robot_state, RobotState.EMERGENCY))
            self.robot_state = RobotState.EMERGENCY

        # Publish state
        self.publish_state()

    def network_callback(self, msg):
        if msg.status.status != self.network_state:
            rospy.loginfo("state_machine :: Network state change: {} --> {}"
                          .format(self.network_state, msg.status.status))
            self.network_state = msg.status.status

    def start_callback(self, msg):
        if msg.data != self.start:
            if msg.data:
                rospy.loginfo("state_machine :: Starting robot...")
                self.start = True
            else:
                rospy.loginfo("state_machine :: Stopping robot...")
                self.start = False

    def emergency_callback(self, msg):
        if msg.data and not self.emergency:
            rospy.logwarn("state_machine :: Emergency stop requested!")
            self.emergency = True

    def reset_callback(self, msg):
        if msg.data and not self.reset:
            rospy.loginfo("state_machine :: Resetting robot...")
            self.reset = True

    def launch_nodes(self):
        self.launch = subprocess.Popen(["roslaunch", self.launch_file, "--no-summary"])

    def kill_nodes(self):
        if self.launch is not None:
            try:
                self.launch.terminate()
                self.launch.wait(timeout=1)
            except subprocess.TimeoutExpired:
                self.launch.kill()
                self.launch.wait(timeout=1)

    def publish_state(self):
        msg = RobotStateStamped()
        msg.header.stamp = rospy.Time.now()
        msg.state.robot_state = self.robot_state
        msg.state.network_status.status = self.network_state
        self.state_pub.publish(msg)

        # Update prev state
        self.prev_state = msg


if __name__ == "__main__":
    _ = StateMachine()
    rospy.spin()
