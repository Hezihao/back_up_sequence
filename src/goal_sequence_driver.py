#! /usr/bin/env python

import rospy
import actionlib
import time
import numpy as np

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler


def create_quaternion(goal_num):

	# read position & orientation(euler) from param and transfer them
	x = rospy.get_param("/neo_goal_sequence_driver/goals/goal_"+str(goal_num+1)+"/X")
	y = rospy.get_param("/neo_goal_sequence_driver/goals/goal_"+str(goal_num+1)+"/Y")
	theta = np.radians(rospy.get_param("/neo_goal_sequence_driver/goals/goal_"+str(goal_num+1)+"/theta"))
	quaternion = quaternion_from_euler(0, 0, theta)
	return [x, y, quaternion]

def create_goal(pose):

	# create goal from position & orientation(quaternion)
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = "map"
	goal.target_pose.header.stamp = rospy.Time.now()
	goal.target_pose.pose.position.x = pose[0]
	goal.target_pose.pose.position.y = pose[1]
	goal.target_pose.pose.orientation.z = pose[2][2]
	goal.target_pose.pose.orientation.w = pose[2][3]
	return goal


if __name__ == '__main__':

	rospy.init_node('goal_sequence_driver')
	client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
	if(client.wait_for_server()):

		rospy.loginfo("Client server up.")
	i = 0
	while(True):

		try:
	
			pose = create_quaternion(i)
		except:

			rospy.loginfo("All goals reached, sequence drive task completed.")
			break
		goal = create_goal(pose)
		client.send_goal(goal)
		while(True):
			
			if(client.wait_for_result(rospy.Duration.from_sec(5.0)) == True):

				rospy.loginfo("Goal reached.")
				# just for test
				time.sleep(2)
				break
		i += 1