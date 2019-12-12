#! /usr/bin/env python

import rospy
import actionlib
import time
import numpy as np

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from neo_goal_sequence_driver.srv import command, commandResponse

# set manually
service_called = True

# transformation of data: euler -> quaternion
def create_quaternion(goal_num):

	# read position & orientation(euler) from param and transfer them
	x = rospy.get_param("/neo_goal_sequence_driver/goals/goal_"+str(goal_num+1)+"/X")
	y = rospy.get_param("/neo_goal_sequence_driver/goals/goal_"+str(goal_num+1)+"/Y")
	theta = np.radians(rospy.get_param("/neo_goal_sequence_driver/goals/goal_"+str(goal_num+1)+"/theta"))
	quaternion = quaternion_from_euler(0, 0, theta)
	return [x, y, quaternion]

# creat goal structure for move_base node
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

# the call back function of service
def goal_sequence_driver(cmd):

	client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
	if(client.wait_for_server()):

		rospy.loginfo("Client server up.")
	# need to check if the service is asked to be shut down
	if(cmd.switch == "down"):
		
		rospy.loginfo("goal_sequence_driver down.")
		rospy.set_param('/neo_goal_sequence_driver/switch', False)

		return "Shut down."
		
	elif(cmd.switch == "up"):

		rospy.loginfo("goal_sequence_driver up.")
		rospy.set_param('/neo_goal_sequence_driver/switch', True)
	rospy.set_param('/neo_goal_sequence_driver/patience', cmd.patience)
	if(cmd.infi_param == "True"):

		rospy.set_param('/neo_goal_sequence_driver/infi_param', True)
	elif(cmd.infi_param == "False"):

		rospy.set_param('/neo_goal_sequence_driver/infi_param', False)
	try:
	
		rospy.get_param('/neo_goal_sequence_driver/occupied')
	except:

		rospy.set_param('/neo_goal_sequence_driver/occupied', False)
	i = 0
	if(not rospy.get_param('/neo_goal_sequence_driver/occupied')):
		rospy.set_param('/neo_goal_sequence_driver/occupied', True)
		while(not rospy.is_shutdown() and rospy.get_param('/neo_goal_sequence_driver/switch')):

			try:
		
				pose = create_quaternion(i)
			except:

				rospy.loginfo("All goals reached, sequence drive task completed.")
				i = 0
				if(not rospy.get_param('/neo_goal_sequence_driver/infi_param')):
					
					break
			goal = create_goal(pose)
			client.send_goal(goal)
			while(not rospy.is_shutdown()):
				
				if(client.wait_for_result(rospy.Duration.from_sec(rospy.get_param('/neo_goal_sequence_driver/patience'))) == True):

					rospy.loginfo("Goal reached.")
					# just for test
					time.sleep(1)
					break
			i += 1
	rospy.set_param('/neo_goal_sequence_driver/occupied', False)
	return "Service done."

# server of this service
def goal_sequence_driver_server():

	rospy.init_node('goal_sequence_driver_server')
	rospy.loginfo("goal_sequence_driver_server is up.")
	server = rospy.Service('goal_sequence_driver', command, goal_sequence_driver)
	rospy.spin()


if __name__ == '__main__':

	if(service_called):

		goal_sequence_driver_server()
	else:

		rospy.init_node('goal_sequence_driver_server')
		rospy.loginfo("goal_sequence_driver is not responding to any request.")