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
		
		
		while not rospy.is_shutdown():
		    self.send_command()
		
		

	def send_command(self):	
		msg = ControlStamped()
		msg.speed = 0
		msg.acceleration = 0
		msg.angle = 0
		self.control_pub.publish(msg)




def main(args):
	controller = Control()


if __name__ == 'main':
	main(sys.argv)
