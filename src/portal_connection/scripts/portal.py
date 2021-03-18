#!/usr/bin/env python

import rospy
import socketio
from sensor_msgs.msg import NavSatFix


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

        # Use timer to periodically publish state of this class
        rospy.Timer(rospy.Duration(1 / self.frequency), self.emit_state)

        # Handle socketio events
        @self.sio.event
        def connect():
            rospy.loginfo("Connected to Portal!")
            self.connected = True

        @self.sio.event
        def connect_error(e):
            rospy.logerr(e['message'])

        @self.sio.event
        def disconnect():
            rospy.logwarn("Connection to Portal broken.")
            self.connected = False

        # Connect to Portal
        try:
            self.sio.connect(self.portal_uri, headers={"secret": self.secret})
        except socketio.exceptions.ConnectionError:
            return

        # Keep running until ROS is shut down
        while not rospy.is_shutdown():
            rospy.spin()

    def emit_state(self, _):
        if (self.connected):
            try:
                self.sio.emit("robot_detail", self.state)
            except socketio.exceptions.BadNamespaceError:
                rospy.logwarn("Failed to send data to Portal...will retry in a bit")

    def gps_callback(self, msg):
        self.state["gps"] = {"lat": msg.latitude, "lng": msg.longitude}

if __name__ == '__main__':
    try:
        _ = PortalConnection()
    except KeyboardInterrupt:
        pass
