#!/usr/bin/env python3

import sys
import rospy
from custom_msgs.msg import ControlStamped



class Control:
	def __init__(self):
		rospy.init_node('control', anonymous=True)

		# Subscribers
		

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
		msg.control.speed = 0
		msg.control.acceleration = 0
		msg.control.steering_angle = 0
		self.control_pub.publish(msg)





def main(args):
	controller = Control()

if __name__ == '__main__':
	main(sys.argv)
