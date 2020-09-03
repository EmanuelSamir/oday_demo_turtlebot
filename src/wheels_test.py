#!/usr/bin/env python

import sys
import copy
import rospy
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
import time


def move_linear(robot_base, set_time, fwd = True):
	start = time.time()
	vel = geometry_msgs.msg.Twist()
	if fwd:
		vel.linear.x = 0.1
	elif not fwd:
		vel.linear.x = -0.1
	while time.time() - start < set_time:
		robot_base.publish(vel)
	vel.linear.x = 0
	robot_base.publish(vel)

def move_angular(robot_base, set_time, ckw = True):
	start = time.time()
	vel = geometry_msgs.msg.Twist()
	if ckw:
		vel.angular.z = -0.5
	elif ckw:
		vel.angular.z = 0.5
	while time.time() - start < set_time:
		robot_base.publish(vel)
	vel.angular.z = 0
	robot_base.publish(vel)


if __name__ == '__main__':
	rospy.init_node('vel_test',
                anonymous=True)

	robot_base = rospy.Publisher('/cmd_vel', geometry_msgs.msg.Twist, queue_size = 10)

	move_angular(robot_base, 0.2)
	move_linear(robot_base, 0.1)
	rospy.spin()