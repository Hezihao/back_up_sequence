#! /usr/bin/env python

import rospy
import actionlib
import time
import numpy as np

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler


def create_quaternion(goal_num):

	# read euler from param and transfer it
	x = rospy.get_param("/move_base/goals/goal_"+str(goal_num+1)+"/X")
	y = rospy.get_param("/move_base/goals/goal_"+str(goal_num+1)+"/Y")
	theta = np.radians(rospy.get_param("/move_base/goals/goal_"+str(goal_num+1)+"/theta"))
	print(rospy.get_param("/move_base/goals/goal_"+str(goal_num+1)+"/theta"))
	quaternion = quaternion_from_euler(x, y, theta)
	return quaternion

def create_goal(q):

	# create goal from quaternion data
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = "map"
	goal.target_pose.header.stamp = rospy.Time.now()
	goal.target_pose.pose.position.x = q[0]
	goal.target_pose.pose.position.y = q[1]
	goal.target_pose.pose.orientation.z = q[2]
	goal.target_pose.pose.orientation.w = q[3]
	return goal


if __name__ == '__main__':

	rospy.init_node('goal_sequence_driver')
	client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
	if(client.wait_for_server()):

		print("Client server up.")

	for i in range(3):
	
		quaternion = create_quaternion(i)
		goal = create_goal(quaternion)
		client.send_goal(goal)
		while(True):
			
			if(client.wait_for_result(rospy.Duration.from_sec(5.0)) == True):

				print("Goal reached.")
				# just for test
				time.sleep(2)
				break
		
		if(i == 2):
			
			print("All goals reached, sequence drive task completed.")