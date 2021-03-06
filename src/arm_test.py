#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


def set_arm_pose(arm, x, y, z, w = 1, xo = 0, yo = 0, zo = 0, time = 3):
	pose_goal = geometry_msgs.msg.Pose()
	pose_goal.orientation.x = xo #1.0
	pose_goal.orientation.y = yo #1.0
	pose_goal.orientation.z = zo #1.0

	pose_goal.orientation.w = w #1.0
	pose_goal.position.x = x #0.26
	pose_goal.position.y = y #0
	pose_goal.position.z = z #0.09

	arm.set_pose_target(pose_goal)
	arm.go(wait=True)

	rospy.sleep(2)

	arm.stop()


def set_gripper(gripper, action):
	OPEN = 0.019
	CLOSE = 0.014
	gripper_value = gripper.get_current_joint_values()
	if action:
		gripper_value[0] = OPEN
		gripper_value[1] = OPEN

	elif not action:
		gripper_value[0] = CLOSE
		gripper_value[1] = CLOSE
	gripper.set_joint_value_target(gripper_value)
	gripper.go(wait=True)
	rospy.sleep(1)
	gripper.stop()



if __name__ == '__main__':
	moveit_commander.roscpp_initialize(sys.argv)
	rospy.init_node('move_group_python_interface',
                anonymous=True)

	robot = moveit_commander.RobotCommander()
	scene = moveit_commander.PlanningSceneInterface()

	group_names = robot.get_group_names()
	print "============ Robot Groups:", robot.get_group_names()

	group_name = "arm"
	arm = moveit_commander.MoveGroupCommander(group_name)
	
	group_name = "gripper"
	gripper = moveit_commander.MoveGroupCommander(group_name)
	
	display_trajectory_publisher=rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory, queue_size=1)
	

	while not rospy.is_shutdown():
		set_gripper(gripper, True)
		set_arm_pose(arm, 0.27, 0, 0.055, yo = 0.1986693, w = 0.9800666)
		rospy.sleep(1)
		set_arm_pose(arm, 0.27, 0, 0.085)
		set_gripper(gripper, False)	
		set_arm_pose(arm, 0.13, 0, 0.26)
		rospy.sleep(2)
	moveit_commander.roscpp_shutdown()
	