#!/usr/bin/env python

import rospy
import socketio
from sensor_msgs.msg import NavSatFix
from automated_msgs.msg import NetworkStatus, NetworkStatusStamped, RobotStateStamped


class PortalConnection():
    def __init__(self):
        rospy.init_node("portal_connection")

        # Initialize state dictionary to hold all information for the Portal
        self.state = {"battery": 90}

        # Initialize attributes
        self.sio = socketio.Client()
        self.connected = False

        # Define ROS parameters
        self.portal_uri = rospy.get_param("~uri", "http://localhost:2000")
        self.frequency = rospy.get_param("~frequency", 10)
        self.secret = rospy.get_param("~secret")

        # Define subscribers
        rospy.Subscriber("/gps/filtered", NavSatFix, self.gps_callback)
        rospy.Subscriber("/state_machine/state", RobotStateStamped, self.robot_state_callback)

        # Define publishers
        self.network_state_pub = rospy.Publisher("/state_machine/connection", NetworkStatusStamped, queue_size=1)

        # Use timer to periodically publish state of this class
        rospy.Timer(rospy.Duration(1 / self.frequency), self.emit_state)

        # Set reconnect rate
        self.reconnect_rate = rospy.Rate(0.1)

        # Handle socketio events
        @self.sio.event
        def connect():
            rospy.loginfo("portal_connection :: Connected to Portal!")
            self.publish_network_status(NetworkStatus.CONNECTED)
            self.connected = True

        @self.sio.event
        def connect_error(e):
            self.publish_network_status(NetworkStatus.ERROR)
            rospy.logerr("portal_connection :: " + e)

        @self.sio.event
        def disconnect():
            rospy.logwarn("portal_connection :: Connection to Portal broken.")
            self.publish_network_status(NetworkStatus.DISCONNECTED)
            self.connected = False

        # Connect to Portal
        while not self.connected and not rospy.is_shutdown():
            try:
                self.sio.connect(self.portal_uri, headers={"secret": self.secret})
            except socketio.exceptions.ConnectionError:
                self.reconnect_rate.sleep()

        # Keep running until ROS is shut down
        rospy.spin()

    def emit_state(self, _):
        if (self.connected):
            try:
                self.sio.emit("robot_detail", self.state)
            except socketio.exceptions.BadNamespaceError:
                rospy.logwarn("portal_connection :: Failed to send data to Portal...will retry in a bit")

    def gps_callback(self, msg):
        self.state["gps"] = {"lat": msg.latitude, "lng": msg.longitude}

    def robot_state_callback(self, msg):
        self.state["state"] = {"state": str(msg.state.robot_state)}

    def publish_network_status(self, status):
        msg = NetworkStatusStamped()
        msg.header.stamp = rospy.Time.now()
        msg.status.status = status
        self.network_state_pub.publish(msg)


if __name__ == '__main__':
    try:
        _ = PortalConnection()
    except KeyboardInterrupt:
        pass
