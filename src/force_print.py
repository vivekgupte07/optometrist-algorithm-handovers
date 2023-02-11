#! /usr/bin/env python

import rospy
import math
from intera_core_msgs.msg import EndpointState

counter = 0


def force_callback(msg):
	global counter
	force_x = msg.wrench.force.x
	force_y = msg.wrench.force.y
	force_z = msg.wrench.force.z
	eq_force = math.sqrt(force_x**2+force_y**2+force_z**2)
	counter+=1
	print(counter, eq_force)
rospy.init_node('force_sub')
sub=rospy.Subscriber('robot/limb/right/endpoint_state', EndpointState, force_callback)
rospy.spin()