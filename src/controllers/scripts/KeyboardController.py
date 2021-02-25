#!/usr/bin/env python

import rospy
from pynput.keyboard import Key, Listener, KeyCode
from geometry_msgs.msg import Twist


class KeyboardController:
    def __init__(self):
        rospy.loginfo("Starting Python Keyboard Controller...")
        
        rospy.init_node('keyboard_controller')

        # ROS Publisher
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # ROS Params
        self.a_scale = rospy.get_param('~avel_scale', 0.3)
        self.l_scale = rospy.get_param('~lvel_scale', 0.3)

        # Set rate to 50Hz
        self.rate = rospy.Rate(50)

        # Initialize keyboard listener
        self.keyboard_listener = Listener(on_press=self.key_press, on_release=self.on_release)

    def run(self):
        self.keyboard_listener.start()

        # Prevent script from terminating
        while not rospy.is_shutdown():
            self.rate.sleep()

    def stop(self):
        self.keyboard_listener.stop()

    def key_press(self, key):
        if (key == Key.up):
            msg = Twist()
            msg.linear.x = 1.0 * self.l_scale
            self.cmd_pub.publish(msg)
        elif (key == Key.down):
            msg = Twist()
            msg.linear.x = -1.0 * self.l_scale
            self.cmd_pub.publish(msg)
        elif (key == Key.left):
            msg = Twist()
            msg.angular.z = 1.0 * self.a_scale
            self.cmd_pub.publish(msg)
        elif (key == Key.right):
            msg = Twist()
            msg.angular.z = -1.0 * self.a_scale
            self.cmd_pub.publish(msg)
        else:
            msg = Twist()
            self.cmd_pub.publish(msg)

    def on_release(self, key):
        if (key == KeyCode.from_char('q')):
            rospy.loginfo("Quit")
            return False
        else:
            msg = Twist()
            self.cmd_pub.publish(msg)


if __name__ == '__main__':
    try:
        controller = KeyboardController()
        controller.run()
    except rospy.ROSInterruptException:
        controller.stop()
        pass
