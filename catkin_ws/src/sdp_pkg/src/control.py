#!/usr/bin/env python3

import sys
import rospy
from sdp_pkg.msg import Control_msg



class Control:
	def __init__(self):
		rospy.init_node('control', anonymous=True)

		# Subscribers
		

		# Publishers
		self.control_pub = rospy.Publisher('/cmd', Control_msg, queue_size=10)



	def send_command(self):	
		msg = Control_msg()
		msg.speed = 0
		msg.acceleration = 0
		msg.angle = 0
		self.control_pub.publish(msg)



def main(args):
	controller = Control()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print('Shutting down')


if __name__ == 'main':
	main(sys.argv)