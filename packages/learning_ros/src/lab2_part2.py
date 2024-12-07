#!/usr/bin/env python3

import rospy

# turtlesim (package) - basic simulator for turtle that move in 2D environment
# msg (module inside turtlesim package) - contains message definitions for communication
# Pose (specific message type) - position and orientation of turtle in simulator. Includes fields like x, y, theta, linear vel, angular vel
from turtlesim.msg import Pose

# turtlesim_helper (package) - provides extra features for working with turtlesim such as helper nodes, custom messages, utilities
# UnitsLabelled (custom message type) - represent a labled unit of some mesaurement
from turtlesim_helper.msg import UnitsLabelled

import math

class CalculateDistance:
	def __init__(self):
		# initialize node named distance_calculator
		# ROS node - single process that can communicate with other nodes via topics, services, actions
		rospy.init_node('distance_calculator')
		
		# store prev position of turtle, updated as turtle moves
		# these are needed to calculate distance
		self.prev_x = None
		self.prev_y = None
		
		# ROS publisher - publish messages of type UnitsLabelled to topic distance_traveled
		# no. messages queued for delivery is 10 to manage memory
		self.pub = rospy.Publisher('distance_traveled', UnitsLabelled, queue_size=10)
		
		# subscribe to topic turtle1/pose providing position and orientation
		# self.pose_callback called each time new message is received on topic
		rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
		
	def pose_callback(self, msg):
		if self.prev_x is not None and self.prev_y is not None:
			# calculate distance using pythagoreon theorem
			distance = math.sqrt((msg.x - self.prev_x)**2 + (msg.y - self.prev_y)**2)
			
			# create and publish message
			output_msg = UnitsLabelled()
			output_msg.value = distance
			output_msg.units = "meters"
			self.pub.publish(output_msg)
		
		# update prev positions
		self.prev_x = msg.x
		self.prev_y = msg.y

if __name__ == '__main__':
    distance_calculator = CalculateDistance()
    rospy.spin()
